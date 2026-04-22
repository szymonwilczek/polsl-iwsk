#include "hal/serial_hal.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32

int serial_hal_linux_init(struct serial_hal_device *dev) {
  (void)dev;
  return -ENOTSUP;
}

void serial_hal_linux_deinit(struct serial_hal_device *dev) { (void)dev; }

#else

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

struct serial_linux_ctx {
  int fd;
  enum serial_hal_flow_control flow;
};

static int serial_linux_validate_config(const struct serial_hal_config *cfg) {
  if (!cfg)
    return -EINVAL;

  if (!cfg->device[0])
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

static int serial_linux_map_baud(uint32_t baud_rate, speed_t *speed) {
  if (!speed)
    return -EINVAL;

  switch (baud_rate) {
  case 150:
    *speed = B150;
    return 0;
  case 300:
    *speed = B300;
    return 0;
  case 600:
    *speed = B600;
    return 0;
  case 1200:
    *speed = B1200;
    return 0;
  case 2400:
    *speed = B2400;
    return 0;
  case 4800:
    *speed = B4800;
    return 0;
  case 9600:
    *speed = B9600;
    return 0;
  case 19200:
    *speed = B19200;
    return 0;
  case 38400:
    *speed = B38400;
    return 0;
  case 57600:
    *speed = B57600;
    return 0;
  case 115200:
    *speed = B115200;
    return 0;
  default:
    return -EINVAL;
  }
}

static void serial_linux_set_raw_mode(struct termios *tio) {
  tio->c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL |
                    IXON | IXOFF | IXANY);
  tio->c_oflag &= ~OPOST;
  tio->c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
}

static int serial_linux_apply_config_fd(int fd,
                                        const struct serial_hal_config *cfg) {
  struct termios tio;
  speed_t speed;
  int ret;

  if (fd < 0 || !cfg)
    return -EINVAL;

  if (tcgetattr(fd, &tio) < 0)
    return -errno;

  serial_linux_set_raw_mode(&tio);

  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~CSIZE;

  if (cfg->data_bits == 7)
    tio.c_cflag |= CS7;
  else
    tio.c_cflag |= CS8;

  if (cfg->parity == SERIAL_HAL_PARITY_NONE) {
    tio.c_cflag &= ~PARENB;
  } else {
    tio.c_cflag |= PARENB;
    if (cfg->parity == SERIAL_HAL_PARITY_ODD)
      tio.c_cflag |= PARODD;
    else
      tio.c_cflag &= ~PARODD;
  }

  if (cfg->stop_bits == 2)
    tio.c_cflag |= CSTOPB;
  else
    tio.c_cflag &= ~CSTOPB;

  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;

  ret = serial_linux_map_baud(cfg->baud_rate, &speed);
  if (ret)
    return ret;

  if (cfsetispeed(&tio, speed) < 0)
    return -errno;

  if (cfsetospeed(&tio, speed) < 0)
    return -errno;

  if (tcsetattr(fd, TCSANOW, &tio) < 0)
    return -errno;

  return 0;
}

static int
serial_linux_apply_flow_control_fd(int fd, enum serial_hal_flow_control flow) {
  struct termios tio;

  if (fd < 0)
    return -EINVAL;

  if (tcgetattr(fd, &tio) < 0)
    return -errno;

  tio.c_iflag &= ~(IXON | IXOFF | IXANY);
#ifdef CRTSCTS
  tio.c_cflag &= ~CRTSCTS;
#endif

  switch (flow) {
  case SERIAL_HAL_FLOW_NONE:
    break;
  case SERIAL_HAL_FLOW_XON_XOFF:
    tio.c_iflag |= IXON | IXOFF;
    break;
  case SERIAL_HAL_FLOW_RTS_CTS:
#ifdef CRTSCTS
    tio.c_cflag |= CRTSCTS;
#else
    return -ENOTSUP;
#endif
    break;
  case SERIAL_HAL_FLOW_DTR_DSR:
    return -ENOTSUP;
  default:
    return -EINVAL;
  }

  if (tcsetattr(fd, TCSANOW, &tio) < 0)
    return -errno;

  return 0;
}

static int serial_linux_wait_fd(int fd, short events, uint32_t timeout_ms) {
  struct pollfd pfd;
  int ret;

  pfd.fd = fd;
  pfd.events = events;
  pfd.revents = 0;

  ret = poll(&pfd, 1, (int)timeout_ms);
  if (ret < 0)
    return -errno;

  if (!ret)
    return -ETIMEDOUT;

  if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
    return -EIO;

  if (!(pfd.revents & events))
    return -EAGAIN;

  return 0;
}

static int serial_linux_open(struct serial_hal_device *dev,
                             const struct serial_hal_config *cfg) {
  struct serial_linux_ctx *ctx;
  int fd;
  int ret;

  if (!dev || !cfg)
    return -EINVAL;

  ctx = dev->priv;
  if (!ctx)
    return -EINVAL;

  if (dev->is_open)
    return -EALREADY;

  ret = serial_linux_validate_config(cfg);
  if (ret)
    return ret;

  fd = open(cfg->device, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0)
    return -errno;

  ret = serial_linux_apply_config_fd(fd, cfg);
  if (ret)
    goto err_close_fd;

  ret = serial_linux_apply_flow_control_fd(fd, SERIAL_HAL_FLOW_NONE);
  if (ret)
    goto err_close_fd;

  ctx->fd = fd;
  ctx->flow = SERIAL_HAL_FLOW_NONE;
  dev->active_cfg = *cfg;
  dev->is_open = true;

  return 0;

err_close_fd:
  close(fd);
  return ret;
}

static int serial_linux_close(struct serial_hal_device *dev) {
  struct serial_linux_ctx *ctx;

  if (!dev)
    return -EINVAL;

  ctx = dev->priv;
  if (!ctx)
    return -EINVAL;

  if (!dev->is_open)
    return 0;

  if (close(ctx->fd) < 0)
    return -errno;

  ctx->fd = -1;
  dev->is_open = false;

  return 0;
}

static ssize_t serial_linux_read(struct serial_hal_device *dev, void *buf,
                                 size_t len, uint32_t timeout_ms) {
  struct serial_linux_ctx *ctx;
  int ret;
  ssize_t bytes;

  if (!dev || !buf || !len)
    return -EINVAL;

  ctx = dev->priv;
  if (!ctx || !dev->is_open || ctx->fd < 0)
    return -ENODEV;

  ret = serial_linux_wait_fd(ctx->fd, POLLIN, timeout_ms);
  if (ret)
    return ret;

  bytes = read(ctx->fd, buf, len);
  if (bytes < 0)
    return -errno;

  return bytes;
}

static ssize_t serial_linux_write(struct serial_hal_device *dev,
                                  const void *buf, size_t len,
                                  uint32_t timeout_ms) {
  struct serial_linux_ctx *ctx;
  const unsigned char *ptr;
  size_t total;

  if (!dev || !buf)
    return -EINVAL;

  if (!len)
    return 0;

  ctx = dev->priv;
  if (!ctx || !dev->is_open || ctx->fd < 0)
    return -ENODEV;

  ptr = buf;
  total = 0;

  while (total < len) {
    ssize_t written;
    int ret;

    ret = serial_linux_wait_fd(ctx->fd, POLLOUT, timeout_ms);
    if (ret) {
      if (total)
        return (ssize_t)total;
      return ret;
    }

    written = write(ctx->fd, ptr + total, len - total);
    if (written < 0) {
      if (errno == EINTR)
        continue;

      if (total)
        return (ssize_t)total;

      return -errno;
    }

    if (!written)
      break;

    total += (size_t)written;
  }

  return (ssize_t)total;
}

static int serial_linux_set_config(struct serial_hal_device *dev,
                                   const struct serial_hal_config *cfg) {
  struct serial_linux_ctx *ctx;
  int ret;

  if (!dev || !cfg)
    return -EINVAL;

  ret = serial_linux_validate_config(cfg);
  if (ret)
    return ret;

  ctx = dev->priv;
  if (!ctx)
    return -EINVAL;

  if (dev->is_open) {
    ret = serial_linux_apply_config_fd(ctx->fd, cfg);
    if (ret)
      return ret;
  }

  dev->active_cfg = *cfg;

  return 0;
}

static int serial_linux_set_flow_control(struct serial_hal_device *dev,
                                         enum serial_hal_flow_control flow) {
  struct serial_linux_ctx *ctx;
  int ret;

  if (!dev)
    return -EINVAL;

  ctx = dev->priv;
  if (!ctx || !dev->is_open || ctx->fd < 0)
    return -ENODEV;

  ret = serial_linux_apply_flow_control_fd(ctx->fd, flow);
  if (ret)
    return ret;

  ctx->flow = flow;

  return 0;
}

static int
serial_linux_set_modem_lines(struct serial_hal_device *dev,
                             const struct serial_hal_modem_lines *lines) {
  struct serial_linux_ctx *ctx;
  int status;

  if (!dev || !lines)
    return -EINVAL;

  ctx = dev->priv;
  if (!ctx || !dev->is_open || ctx->fd < 0)
    return -ENODEV;

  if (ioctl(ctx->fd, TIOCMGET, &status) < 0)
    return -errno;

  if (lines->dtr)
    status |= TIOCM_DTR;
  else
    status &= ~TIOCM_DTR;

  if (lines->rts)
    status |= TIOCM_RTS;
  else
    status &= ~TIOCM_RTS;

  if (ioctl(ctx->fd, TIOCMSET, &status) < 0)
    return -errno;

  return 0;
}

static int serial_linux_get_modem_lines(struct serial_hal_device *dev,
                                        struct serial_hal_modem_lines *lines) {
  struct serial_linux_ctx *ctx;
  int status;

  if (!dev || !lines)
    return -EINVAL;

  ctx = dev->priv;
  if (!ctx || !dev->is_open || ctx->fd < 0)
    return -ENODEV;

  if (ioctl(ctx->fd, TIOCMGET, &status) < 0)
    return -errno;

  memset(lines, 0, sizeof(*lines));
  lines->dtr = !!(status & TIOCM_DTR);
  lines->rts = !!(status & TIOCM_RTS);
  lines->cts = !!(status & TIOCM_CTS);
  lines->dsr = !!(status & TIOCM_DSR);
  lines->dcd = !!(status & TIOCM_CAR);
  lines->ri = !!(status & TIOCM_RNG);

  return 0;
}

static int serial_linux_flush(struct serial_hal_device *dev) {
  struct serial_linux_ctx *ctx;

  if (!dev)
    return -EINVAL;

  ctx = dev->priv;
  if (!ctx || !dev->is_open || ctx->fd < 0)
    return -ENODEV;

  if (tcflush(ctx->fd, TCIOFLUSH) < 0)
    return -errno;

  return 0;
}

static void serial_linux_destroy(struct serial_hal_device *dev) {
  struct serial_linux_ctx *ctx;

  if (!dev)
    return;

  ctx = dev->priv;
  if (!ctx)
    return;

  if (ctx->fd >= 0)
    close(ctx->fd);

  free(ctx);
  memset(&dev->active_cfg, 0, sizeof(dev->active_cfg));
  dev->priv = NULL;
  dev->ops = NULL;
  dev->is_open = false;
}

static const struct serial_hal_ops serial_linux_ops = {
    .open = serial_linux_open,
    .close = serial_linux_close,
    .read = serial_linux_read,
    .write = serial_linux_write,
    .set_config = serial_linux_set_config,
    .set_flow_control = serial_linux_set_flow_control,
    .set_modem_lines = serial_linux_set_modem_lines,
    .get_modem_lines = serial_linux_get_modem_lines,
    .flush = serial_linux_flush,
    .destroy = serial_linux_destroy,
};

int serial_hal_linux_init(struct serial_hal_device *dev) {
  struct serial_linux_ctx *ctx;

  if (!dev)
    return -EINVAL;

  memset(dev, 0, sizeof(*dev));

  ctx = calloc(1, sizeof(*ctx));
  if (!ctx)
    return -ENOMEM;

  ctx->fd = -1;
  ctx->flow = SERIAL_HAL_FLOW_NONE;

  dev->ops = &serial_linux_ops;
  dev->priv = ctx;

  return 0;
}

void serial_hal_linux_deinit(struct serial_hal_device *dev) {
  if (!dev || !dev->ops || !dev->ops->destroy)
    return;

  dev->ops->destroy(dev);
}

#endif
