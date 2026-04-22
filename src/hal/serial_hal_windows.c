#include "hal/serial_hal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef _WIN32

int serial_hal_windows_init(struct serial_hal_device *dev) {
  (void)dev;
  return -ENOTSUP;
}

void serial_hal_windows_deinit(struct serial_hal_device *dev) { (void)dev; }

#else

#include <windows.h>

struct serial_win_ctx {
  HANDLE hComm;
  enum serial_hal_flow_control flow;
};

static int serial_win_validate_config(const struct serial_hal_config *cfg) {
  if (!cfg || !cfg->device[0])
    return -EINVAL;
  if (cfg->baud_rate < 150 || cfg->baud_rate > 115200)
    return -EINVAL;
  if (cfg->data_bits != 7 && cfg->data_bits != 8)
    return -EINVAL;
  if (cfg->stop_bits != 1 && cfg->stop_bits != 2)
    return -EINVAL;
  if (cfg->parity != SERIAL_HAL_PARITY_NONE &&
      cfg->parity != SERIAL_HAL_PARITY_EVEN &&
      cfg->parity != SERIAL_HAL_PARITY_ODD)
    return -EINVAL;
  return 0;
}

static int serial_win_apply_config_fd(HANDLE hComm,
                                      const struct serial_hal_config *cfg) {
  DCB dcb = {0};

  dcb.DCBlength = sizeof(DCB);
  if (!GetCommState(hComm, &dcb))
    return -EIO;

  dcb.BaudRate = cfg->baud_rate;
  dcb.ByteSize = cfg->data_bits;
  dcb.StopBits = (cfg->stop_bits == 2) ? TWOSTOPBITS : ONESTOPBIT;

  if (cfg->parity == SERIAL_HAL_PARITY_NONE) {
    dcb.Parity = NOPARITY;
    dcb.fParity = FALSE;
  } else if (cfg->parity == SERIAL_HAL_PARITY_EVEN) {
    dcb.Parity = EVENPARITY;
    dcb.fParity = TRUE;
  } else {
    dcb.Parity = ODDPARITY;
    dcb.fParity = TRUE;
  }

  if (!SetCommState(hComm, &dcb))
    return -EIO;

  return 0;
}

static int serial_win_apply_flow_control_fd(HANDLE hComm,
                                            enum serial_hal_flow_control flow) {
  DCB dcb = {0};

  dcb.DCBlength = sizeof(DCB);
  if (!GetCommState(hComm, &dcb))
    return -EIO;

  /* reset flow control flags */
  dcb.fOutxCtsFlow = FALSE;
  dcb.fOutxDsrFlow = FALSE;
  dcb.fDtrControl = DTR_CONTROL_DISABLE;
  dcb.fRtsControl = RTS_CONTROL_DISABLE;
  dcb.fInX = FALSE;
  dcb.fOutX = FALSE;

  switch (flow) {
  case SERIAL_HAL_FLOW_NONE:
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    break;
  case SERIAL_HAL_FLOW_XON_XOFF:
    dcb.fInX = TRUE;
    dcb.fOutX = TRUE;
    break;
  case SERIAL_HAL_FLOW_RTS_CTS:
    dcb.fOutxCtsFlow = TRUE;
    dcb.fRtsControl = RTS_CONTROL_HANDSHAKE;
    break;
  case SERIAL_HAL_FLOW_DTR_DSR:
    dcb.fOutxDsrFlow = TRUE;
    dcb.fDtrControl = DTR_CONTROL_HANDSHAKE;
    break;
  default:
    return -EINVAL;
  }

  if (!SetCommState(hComm, &dcb))
    return -EIO;

  return 0;
}

static int serial_win_open(struct serial_hal_device *dev,
                           const struct serial_hal_config *cfg) {
  struct serial_win_ctx *ctx;
  char port_path[SERIAL_HAL_DEVICE_NAME_MAX + 10];
  HANDLE hComm;
  int ret;

  if (!dev || !cfg)
    return -EINVAL;

  ctx = dev->priv;
  if (!ctx || dev->is_open)
    return -EINVAL;

  ret = serial_win_validate_config(cfg);
  if (ret)
    return ret;

  /* Note for line 132: WinAPI requires \\.\ prefix for COM ports above COM9 */
  snprintf(port_path, sizeof(port_path), "\\\\.\\%s", cfg->device);

  hComm = CreateFileA(port_path, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                      OPEN_EXISTING, 0, NULL);

  if (hComm == INVALID_HANDLE_VALUE)
    return -ENODEV;

  ret = serial_win_apply_config_fd(hComm, cfg);
  if (ret)
    goto err_close;

  ret = serial_win_apply_flow_control_fd(hComm, SERIAL_HAL_FLOW_NONE);
  if (ret)
    goto err_close;

  ctx->hComm = hComm;
  ctx->flow = SERIAL_HAL_FLOW_NONE;
  dev->active_cfg = *cfg;
  dev->is_open = true;

  return 0;

err_close:
  CloseHandle(hComm);
  return ret;
}

static int serial_win_close(struct serial_hal_device *dev) {
  struct serial_win_ctx *ctx;

  if (!dev)
    return -EINVAL;

  ctx = dev->priv;
  if (!ctx || !dev->is_open)
    return 0;

  CloseHandle(ctx->hComm);
  ctx->hComm = INVALID_HANDLE_VALUE;
  dev->is_open = false;

  return 0;
}

static ssize_t serial_win_read(struct serial_hal_device *dev, void *buf,
                               size_t len, uint32_t timeout_ms) {
  struct serial_win_ctx *ctx;
  COMMTIMEOUTS timeouts = {0};
  DWORD bytes_read;

  if (!dev || !buf || !len)
    return -EINVAL;

  ctx = dev->priv;
  if (!ctx || !dev->is_open || ctx->hComm == INVALID_HANDLE_VALUE)
    return -ENODEV;

  /* configure blocking semantics with specified timeout */
  timeouts.ReadIntervalTimeout = MAXDWORD;
  timeouts.ReadTotalTimeoutMultiplier = MAXDWORD;
  timeouts.ReadTotalTimeoutConstant = timeout_ms;
  SetCommTimeouts(ctx->hComm, &timeouts);

  if (!ReadFile(ctx->hComm, buf, (DWORD)len, &bytes_read, NULL))
    return -EIO;

  if (bytes_read == 0)
    return -ETIMEDOUT;

  return (ssize_t)bytes_read;
}

static ssize_t serial_win_write(struct serial_hal_device *dev, const void *buf,
                                size_t len, uint32_t timeout_ms) {
  struct serial_win_ctx *ctx;
  COMMTIMEOUTS timeouts = {0};
  DWORD bytes_written;

  if (!dev || !buf)
    return -EINVAL;

  if (!len)
    return 0;

  ctx = dev->priv;
  if (!ctx || !dev->is_open || ctx->hComm == INVALID_HANDLE_VALUE)
    return -ENODEV;

  timeouts.WriteTotalTimeoutConstant = timeout_ms;
  timeouts.WriteTotalTimeoutMultiplier = 0;
  SetCommTimeouts(ctx->hComm, &timeouts);

  if (!WriteFile(ctx->hComm, buf, (DWORD)len, &bytes_written, NULL))
    return -EIO;

  return (ssize_t)bytes_written;
}

static int serial_win_set_config(struct serial_hal_device *dev,
                                 const struct serial_hal_config *cfg) {
  struct serial_win_ctx *ctx;
  int ret;

  if (!dev || !cfg)
    return -EINVAL;

  ret = serial_win_validate_config(cfg);
  if (ret)
    return ret;

  ctx = dev->priv;
  if (!ctx)
    return -EINVAL;

  if (dev->is_open) {
    ret = serial_win_apply_config_fd(ctx->hComm, cfg);
    if (ret)
      return ret;
  }

  dev->active_cfg = *cfg;
  return 0;
}

static int serial_win_set_flow_control(struct serial_hal_device *dev,
                                       enum serial_hal_flow_control flow) {
  struct serial_win_ctx *ctx;
  int ret;

  if (!dev)
    return -EINVAL;

  ctx = dev->priv;
  if (!ctx || !dev->is_open || ctx->hComm == INVALID_HANDLE_VALUE)
    return -ENODEV;

  ret = serial_win_apply_flow_control_fd(ctx->hComm, flow);
  if (ret)
    return ret;

  ctx->flow = flow;
  return 0;
}

static int
serial_win_set_modem_lines(struct serial_hal_device *dev,
                           const struct serial_hal_modem_lines *lines) {
  struct serial_win_ctx *ctx;

  if (!dev || !lines)
    return -EINVAL;

  ctx = dev->priv;
  if (!ctx || !dev->is_open || ctx->hComm == INVALID_HANDLE_VALUE)
    return -ENODEV;

  /* DTR Control */
  if (!EscapeCommFunction(ctx->hComm, lines->dtr ? SETDTR : CLRDTR))
    return -EIO;

  /* RTS Control */
  if (!EscapeCommFunction(ctx->hComm, lines->rts ? SETRTS : CLRRTS))
    return -EIO;

  return 0;
}

static int serial_win_get_modem_lines(struct serial_hal_device *dev,
                                      struct serial_hal_modem_lines *lines) {
  struct serial_win_ctx *ctx;
  DWORD status;

  if (!dev || !lines)
    return -EINVAL;

  ctx = dev->priv;
  if (!ctx || !dev->is_open || ctx->hComm == INVALID_HANDLE_VALUE)
    return -ENODEV;

  if (!GetCommModemStatus(ctx->hComm, &status))
    return -EIO;

  memset(lines, 0, sizeof(*lines));
  lines->cts = !!(status & MS_CTS_ON);
  lines->dsr = !!(status & MS_DSR_ON);
  lines->ri = !!(status & MS_RING_ON);
  lines->dcd = !!(status & MS_RLSD_ON);

  return 0;
}

static int serial_win_flush(struct serial_hal_device *dev) {
  struct serial_win_ctx *ctx;

  if (!dev)
    return -EINVAL;

  ctx = dev->priv;
  if (!ctx || !dev->is_open || ctx->hComm == INVALID_HANDLE_VALUE)
    return -ENODEV;

  if (!PurgeComm(ctx->hComm, PURGE_RXCLEAR | PURGE_TXCLEAR))
    return -EIO;

  return 0;
}

static void serial_win_destroy(struct serial_hal_device *dev) {
  struct serial_win_ctx *ctx;

  if (!dev)
    return;

  ctx = dev->priv;
  if (!ctx)
    return;

  if (ctx->hComm != INVALID_HANDLE_VALUE)
    CloseHandle(ctx->hComm);

  free(ctx);
  memset(&dev->active_cfg, 0, sizeof(dev->active_cfg));
  dev->priv = NULL;
  dev->ops = NULL;
  dev->is_open = false;
}

static const struct serial_hal_ops serial_win_ops = {
    .open = serial_win_open,
    .close = serial_win_close,
    .read = serial_win_read,
    .write = serial_win_write,
    .set_config = serial_win_set_config,
    .set_flow_control = serial_win_set_flow_control,
    .set_modem_lines = serial_win_set_modem_lines,
    .get_modem_lines = serial_win_get_modem_lines,
    .flush = serial_win_flush,
    .destroy = serial_win_destroy,
};

int serial_hal_windows_init(struct serial_hal_device *dev) {
  struct serial_win_ctx *ctx;

  if (!dev)
    return -EINVAL;

  memset(dev, 0, sizeof(*dev));

  ctx = calloc(1, sizeof(*ctx));
  if (!ctx)
    return -ENOMEM;

  ctx->hComm = INVALID_HANDLE_VALUE;
  ctx->flow = SERIAL_HAL_FLOW_NONE;

  dev->ops = &serial_win_ops;
  dev->priv = ctx;

  return 0;
}

void serial_hal_windows_deinit(struct serial_hal_device *dev) {
  if (!dev || !dev->ops || !dev->ops->destroy)
    return;

  dev->ops->destroy(dev);
}

#endif
