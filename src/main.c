#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE 1

#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "hal/serial_hal.h"
#include "protocol/modbus_core.h"
#include "rs232/rs232_logic.h"

#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_CYAN "\x1b[36m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_RESET "\x1b[0m"

#ifdef _WIN32
#include <windows.h>
#ifndef ENABLE_VIRTUAL_TERMINAL_PROCESSING
#define ENABLE_VIRTUAL_TERMINAL_PROCESSING 0x0004
#endif

/**
 * @brief Enables ANSI color codes in Windows console.
 */
static void enable_windows_ansi(void) {
  HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
  if (hOut != INVALID_HANDLE_VALUE) {
    DWORD dwMode = 0;
    if (GetConsoleMode(hOut, &dwMode)) {
      dwMode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;
      SetConsoleMode(hOut, dwMode);
    }
  }
}
#else
#include <dirent.h>
#include <time.h>
#endif

#define RX_BUFFER_SIZE 1024
#define PORT_NAME_MAX 32
#define PORT_LIST_MAX 32
#define SLAVE_TEXT_MAX 240

struct app_context {
  struct serial_hal_device dev;
  struct serial_hal_config cfg;
  struct rs232_terminator_config term;
  enum serial_hal_flow_control flow;

  atomic_bool is_running; /* whole app alive */
  atomic_bool rx_listen;  /* passive RS-232 display thread active */
  atomic_bool slave_mode; /* MODBUS slave listen loop active */
  pthread_t rx_thread;
  pthread_mutex_t dev_lock; /* serialize device access for transactions */

  struct serial_hal_modem_lines current_lines;

  /* reassembly buffer for the passive RS-232 listener (USB UART fragments) */
  uint8_t rx_acc[RX_BUFFER_SIZE];
  size_t rx_acc_len;
  uint64_t rx_acc_last_ms; /* last time a byte landed in rx_acc */

  /* MODBUS station state */
  enum modbus_mode mb_mode; /* ASCII or RTU */
  enum modbus_role role;
  uint8_t slave_address;               /* 1..247 */
  uint32_t mb_timeout_ms;              /* 0..10000, transaction timeout */
  uint8_t mb_retries;                  /* 0..5 */
  uint32_t mb_gap_ms;                  /* 0..1000, inter-character gap */
  char slave_text[SLAVE_TEXT_MAX + 1]; /* text held by slave (func 2 read) */
};

static uint64_t now_ms(void) {
#ifdef _WIN32
  return (uint64_t)GetTickCount64();
#else
  struct timespec ts;
  if (clock_gettime(CLOCK_MONOTONIC, &ts) < 0)
    return 0;
  return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
#endif
}

/*
 * Read a whole line from stdin, strip trailing CR/LF.
 * Returns false on EOF.
 */
static bool read_line(char *buf, size_t size) {
  if (!fgets(buf, (int)size, stdin))
    return false;
  size_t n = strlen(buf);
  while (n && (buf[n - 1] == '\n' || buf[n - 1] == '\r'))
    buf[--n] = '\0';
  return true;
}

/* Print a byte buffer as a HEX dump */
static void print_hex(const char *label, const uint8_t *data, size_t len) {
  char hex[RX_BUFFER_SIZE * 3];
  size_t hex_len = 0;
  if (rs232_bytes_to_hex(data, len, true, true, hex, sizeof(hex), &hex_len) ==
      0) {
    printf(ANSI_COLOR_MAGENTA "%s" ANSI_COLOR_RESET " [%zu B]: %s\n", label,
           len, hex);
  } else {
    printf(ANSI_COLOR_MAGENTA "%s" ANSI_COLOR_RESET " [%zu B]: <too long>\n",
           label, len);
  }
}

static const char *parity_name(enum serial_hal_parity p) {
  switch (p) {
  case SERIAL_HAL_PARITY_EVEN:
    return "E";
  case SERIAL_HAL_PARITY_ODD:
    return "O";
  default:
    return "N";
  }
}

static const char *flow_name(enum serial_hal_flow_control f) {
  switch (f) {
  case SERIAL_HAL_FLOW_XON_XOFF:
    return "XON/XOFF (software)";
  case SERIAL_HAL_FLOW_RTS_CTS:
    return "RTS/CTS (hardware)";
  case SERIAL_HAL_FLOW_DTR_DSR:
    return "DTR/DSR (hardware)";
  default:
    return "None";
  }
}

static const char *term_name(enum rs232_terminator_mode m) {
  switch (m) {
  case RS232_TERMINATOR_CR:
    return "CR";
  case RS232_TERMINATOR_LF:
    return "LF";
  case RS232_TERMINATOR_CRLF:
    return "CR-LF";
  case RS232_TERMINATOR_CUSTOM:
    return "custom";
  default:
    return "none";
  }
}

/**
 * @brief Read a MODBUS ASCII frame (until LF) honoring continuity rules.
 *
 * @param overall_ms maximum wait for the first byte.
 * @param gap_ms     maximum wait between consecutive bytes (0 = use overall).
 * @return number of bytes on success, negative errno on timeout/error.
 */
static ssize_t read_frame_lf(struct app_context *ctx, uint8_t *buf, size_t size,
                             uint32_t overall_ms, uint32_t gap_ms) {
  size_t len = 0;
  bool started = false;
  uint64_t deadline = now_ms() + overall_ms;

  while (len < size) {
    uint32_t wait;
    if (!started) {
      uint64_t t = now_ms();
      if (t >= deadline)
        return -ETIMEDOUT;
      wait = (uint32_t)(deadline - t);
    } else {
      wait = gap_ms ? gap_ms : 200;
    }

    ssize_t n = serial_hal_read(&ctx->dev, buf + len, 1, wait);
    if (n == -ETIMEDOUT || n == -EAGAIN) {
      if (started)
        return -ETIMEDOUT; /* inter-character gap exceeded -> broken frame */
      return -ETIMEDOUT;
    }
    if (n < 0)
      return n;
    if (n == 0)
      continue;

    started = true;
    len += (size_t)n;
    if (buf[len - 1] == MODBUS_ASCII_LF)
      return (ssize_t)len;
  }
  return -ENOSPC;
}

/* Dispatch MODBUS encode/decode/receive on the selected mode (ASCII or RTU) */

static int mb_encode(struct app_context *ctx, const struct modbus_frame *frame,
                     uint8_t *wire, size_t size, size_t *len) {
  if (ctx->mb_mode == MODBUS_MODE_RTU)
    return modbus_rtu_encode(frame, wire, size, len);
  return modbus_ascii_encode(frame, (char *)wire, size, len);
}

static int mb_decode(struct app_context *ctx, const uint8_t *buf, size_t len,
                     struct modbus_frame *frame) {
  if (ctx->mb_mode == MODBUS_MODE_RTU)
    return modbus_rtu_decode(buf, len, frame);
  return modbus_ascii_decode((const char *)buf, len, frame);
}

/**
 * @brief MODBUS frame reader:
 *        wait for the first byte, settle so the whole burst lands in
 *        the kernel buffer, then drain it.
 *
 * Reading byte-by-byte concurrently with arrival makes the PL2303 driver drop
 * bytes intermittently.
 * Waiting for the burst to buffer and then draining it in large reads avoids
 * that race.
 * ASCII frames end at LF; RTU frames end at the inter-character gap.
 */
static ssize_t mb_read_frame(struct app_context *ctx, uint8_t *buf, size_t size,
                             uint32_t overall_ms, uint32_t gap_ms) {
  bool want_lf = (ctx->mb_mode != MODBUS_MODE_RTU);
  uint32_t gap = gap_ms ? gap_ms : 30;
  if (gap < 20)
    gap = 20;
  size_t len = 0;
  uint64_t deadline = now_ms() + overall_ms;

  /* wait for first byte of the reply */
  for (;;) {
    uint64_t t = now_ms();
    if (t >= deadline)
      return -ETIMEDOUT;
    ssize_t n = serial_hal_read(&ctx->dev, buf, 1, (uint32_t)(deadline - t));
    if (n == -ETIMEDOUT || n == -EAGAIN)
      return -ETIMEDOUT;
    if (n < 0)
      return n;
    if (n > 0) {
      len = 1;
      break;
    }
  }

  /* let remainder of the frame arrive into the OS buffer */
  usleep(60000);

  /* drain buffered burst in large reads */
  while (len < size) {
    if (want_lf && memchr(buf, MODBUS_ASCII_LF, len))
      break;
    ssize_t n = serial_hal_read(&ctx->dev, buf + len, size - len, gap);
    if (n == -ETIMEDOUT || n == -EAGAIN)
      break; /* gap elapsed -> end of frame */
    if (n < 0)
      return n;
    len += (size_t)n;
  }
  return (ssize_t)len;
}

static const char *mb_mode_name(enum modbus_mode m) {
  return m == MODBUS_MODE_RTU ? "RTU" : "ASCII";
}

static void slave_send_exception(struct app_context *ctx, uint8_t addr,
                                 uint8_t func, uint8_t exc_code) {
  struct modbus_frame f;
  uint8_t wire[MODBUS_MAX_ADU_SIZE * 2];
  size_t wire_len = 0;

  memset(&f, 0, sizeof(f));
  f.server_address = addr;
  f.pdu.function_code = (uint8_t)(func | 0x80);
  f.pdu.data[0] = exc_code;
  f.pdu.data_len = 1;

  if (mb_encode(ctx, &f, wire, sizeof(wire), &wire_len) == 0) {
    print_hex("[SLAVE TX]", wire, wire_len);
    serial_hal_write(&ctx->dev, wire, wire_len, 500);
  }
}

static void slave_send_normal(struct app_context *ctx,
                              const struct modbus_frame *resp) {
  uint8_t wire[MODBUS_MAX_ADU_SIZE * 2];
  size_t wire_len = 0;
  if (mb_encode(ctx, resp, wire, sizeof(wire), &wire_len) == 0) {
    print_hex("[SLAVE TX]", wire, wire_len);
    serial_hal_write(&ctx->dev, wire, wire_len, 500);
  }
}

static void slave_handle_frame(struct app_context *ctx, const uint8_t *raw,
                               size_t raw_len) {
  struct modbus_frame req;
  int ret;

  print_hex("\n[SLAVE RX]", raw, raw_len);

  ret = mb_decode(ctx, raw, raw_len, &req);
  if (ret == -EPROTO || ret == -EMSGSIZE || ret == -EILSEQ) {
    printf(ANSI_COLOR_YELLOW "[SLAVE] Not a MODBUS frame, ignored.\n"
                             "> " ANSI_COLOR_RESET);
    fflush(stdout);
    return;
  }
  if (ret == -EBADMSG || !req.is_valid) {
    printf(ANSI_COLOR_RED "[SLAVE] Bad LRC (got %02X) -> frame rejected.\n"
                          "> " ANSI_COLOR_RESET,
           req.checksum);
    fflush(stdout);
    return; /* per spec: silently drop frames with checksum error */
  }

  bool broadcast = (req.server_address == 0);
  if (!broadcast && req.server_address != ctx->slave_address) {
    printf(ANSI_COLOR_YELLOW
           "[SLAVE] Frame addressed to %02X, not us (%02X). Ignored.\n"
           "> " ANSI_COLOR_RESET,
           req.server_address, ctx->slave_address);
    fflush(stdout);
    return;
  }

  switch (req.pdu.function_code) {
  case 0x01: { /* write text master -> slave */
    size_t n = req.pdu.data_len;
    if (n > SLAVE_TEXT_MAX)
      n = SLAVE_TEXT_MAX;
    memcpy(ctx->slave_text, req.pdu.data, n);
    ctx->slave_text[n] = '\0';
    printf(ANSI_COLOR_GREEN "[SLAVE] Tekst odebrany: \"%s\"\n" ANSI_COLOR_RESET,
           ctx->slave_text);
    if (!broadcast) {
      struct modbus_frame resp;
      memset(&resp, 0, sizeof(resp));
      resp.server_address = ctx->slave_address;
      resp.pdu.function_code = 0x01; /* normal ack, echo length */
      resp.pdu.data[0] = (uint8_t)n;
      resp.pdu.data_len = 1;
      slave_send_normal(ctx, &resp);
    } else {
      printf(ANSI_COLOR_CYAN
             "[SLAVE] Broadcast -> no response sent.\n" ANSI_COLOR_RESET);
    }
    break;
  }
  case 0x02: { /* read text slave -> master */
    if (broadcast) {
      printf(ANSI_COLOR_CYAN "[SLAVE] Read on broadcast ignored (no "
                             "response).\n" ANSI_COLOR_RESET);
      break;
    }
    struct modbus_frame resp;
    memset(&resp, 0, sizeof(resp));
    resp.server_address = ctx->slave_address;
    resp.pdu.function_code = 0x02;
    size_t n = strlen(ctx->slave_text);
    if (n > MODBUS_MAX_ADU_SIZE)
      n = MODBUS_MAX_ADU_SIZE;
    memcpy(resp.pdu.data, ctx->slave_text, n);
    resp.pdu.data_len = n;
    printf(
        ANSI_COLOR_GREEN
        "[SLAVE] Read request -> sending stored text \"%s\"\n" ANSI_COLOR_RESET,
        ctx->slave_text);
    slave_send_normal(ctx, &resp);
    break;
  }
  default:
    printf(ANSI_COLOR_RED
           "[SLAVE] Unsupported function %02X.\n" ANSI_COLOR_RESET,
           req.pdu.function_code);
    if (!broadcast)
      slave_send_exception(ctx, ctx->slave_address, req.pdu.function_code,
                           0x01 /* ILLEGAL FUNCTION */);
    break;
  }
  printf("> ");
  fflush(stdout);
}

/* sentinel byte that marks the end of passive message, per terminator */
static bool passive_sentinel(const struct app_context *ctx, uint8_t *out) {
  switch (ctx->term.mode) {
  case RS232_TERMINATOR_CR:
    *out = '\r';
    return true;
  case RS232_TERMINATOR_LF:
  case RS232_TERMINATOR_CRLF:
    *out = '\n';
    return true;
  case RS232_TERMINATOR_CUSTOM:
    if (ctx->term.custom_len) {
      *out = ctx->term.custom[ctx->term.custom_len - 1];
      return true;
    }
    return false;
  default:
    return false; /* no terminator: cannot delimit, flush each chunk */
  }
}

/* classify and act on one complete reassembled message */
static void passive_process(struct app_context *ctx, uint8_t *msg, size_t len) {
  if (!len)
    return;
  print_hex("\n[RX]", msg, len);

  struct modbus_frame frame;
  if (len >= 9 && msg[0] == MODBUS_ASCII_START_CHAR &&
      modbus_ascii_decode((const char *)msg, len, &frame) == 0 &&
      frame.is_valid) {
    printf(ANSI_COLOR_GREEN "[MODBUS VALID]" ANSI_COLOR_RESET
                            " Node: %02X | Func: %02X | Data Len: %zu\n> ",
           frame.server_address, frame.pdu.function_code, frame.pdu.data_len);
    fflush(stdout);
    return;
  }

  /* PING responder:
   * Reply PONG so the peer can measure round-trip delay.
   * Search for "PING" anywhere in the message so a probe is still recognised
   * even when preceded by garbage
   */
  bool is_ping = false;
  for (size_t i = 0; len >= 4 && i + 4 <= len; ++i) {
    if (memcmp(msg + i, "PING", 4) == 0) {
      is_ping = true;
      break;
    }
  }
  if (is_ping) {
    uint8_t out[16];
    size_t out_len = 0;
    if (rs232_apply_terminator((const uint8_t *)"PONG", 4, &ctx->term, out,
                               sizeof(out), &out_len) == 0) {
      serial_hal_write(&ctx->dev, out, out_len, 500);
      printf(ANSI_COLOR_CYAN
             "[PING] Responded with PONG.\n> " ANSI_COLOR_RESET);
      fflush(stdout);
      return;
    }
  }

  msg[len] = '\0';
  printf(ANSI_COLOR_YELLOW "[RAW TEXT]" ANSI_COLOR_RESET " %s\n> ", msg);
  fflush(stdout);
}

static void passive_step(struct app_context *ctx) {
  ssize_t n = serial_hal_read(&ctx->dev, ctx->rx_acc + ctx->rx_acc_len,
                              sizeof(ctx->rx_acc) - 1 - ctx->rx_acc_len, 50);
  if (n <= 0) {
    /* drop stale, never-terminated partial so the next message starts from
     * clean buffer */
    if (ctx->rx_acc_len && now_ms() - ctx->rx_acc_last_ms > 150)
      ctx->rx_acc_len = 0;
    return;
  }
  ctx->rx_acc_len += (size_t)n;
  ctx->rx_acc_last_ms = now_ms();

  uint8_t sentinel;
  if (!passive_sentinel(ctx, &sentinel)) {
    /* no terminator configured: emit what we have immediately */
    passive_process(ctx, ctx->rx_acc, ctx->rx_acc_len);
    ctx->rx_acc_len = 0;
    return;
  }

  /* drain every complete message (sentinel-terminated) from the buffer */
  for (;;) {
    uint8_t *nl = memchr(ctx->rx_acc, sentinel, ctx->rx_acc_len);
    if (!nl)
      break;
    size_t msg_len = (size_t)(nl - ctx->rx_acc) + 1;
    passive_process(ctx, ctx->rx_acc, msg_len);
    memmove(ctx->rx_acc, ctx->rx_acc + msg_len, ctx->rx_acc_len - msg_len);
    ctx->rx_acc_len -= msg_len;
  }

  /* guard against runaway peer that never sends the terminator */
  if (ctx->rx_acc_len >= sizeof(ctx->rx_acc) - 1) {
    passive_process(ctx, ctx->rx_acc, ctx->rx_acc_len);
    ctx->rx_acc_len = 0;
  }
}

static void *rx_worker_thread(void *arg) {
  struct app_context *ctx = (struct app_context *)arg;

  while (atomic_load(&ctx->is_running)) {
    if (atomic_load(&ctx->slave_mode)) {
      uint8_t frame[RX_BUFFER_SIZE];
      pthread_mutex_lock(&ctx->dev_lock);
      ssize_t n = mb_read_frame(ctx, frame, sizeof(frame), 300, ctx->mb_gap_ms);
      if (n > 0)
        slave_handle_frame(ctx, frame, (size_t)n);
      pthread_mutex_unlock(&ctx->dev_lock);
      continue;
    }

    if (atomic_load(&ctx->rx_listen)) {
      pthread_mutex_lock(&ctx->dev_lock);
      passive_step(ctx);
      pthread_mutex_unlock(&ctx->dev_lock);
      /* yield so manual send on the main thread is never starved when poll
       * returns immediately */
      usleep(5000);
      continue;
    }

    usleep(20000);
  }
  return NULL;
}

static void show_config(const struct app_context *ctx) {
  printf("\n" ANSI_COLOR_CYAN "--- Current configuration ---" ANSI_COLOR_RESET
         "\n");
  printf("Port      : %s\n", ctx->cfg.device);
  printf("Format    : %u baud, %u%s%u\n", ctx->cfg.baud_rate,
         ctx->cfg.data_bits, parity_name(ctx->cfg.parity), ctx->cfg.stop_bits);
  printf("Flow ctrl : %s\n", flow_name(ctx->flow));
  printf("Terminator: %s\n", term_name(ctx->term.mode));
  printf("MODBUS    : role=%s, mode=%s, slave_addr=%u, timeout=%ums, "
         "retries=%u, gap=%ums\n",
         ctx->role == MODBUS_ROLE_MASTER ? "MASTER" : "SLAVE",
         mb_mode_name(ctx->mb_mode), ctx->slave_address, ctx->mb_timeout_ms,
         ctx->mb_retries, ctx->mb_gap_ms);
}

static void configure_serial(struct app_context *ctx) {
  char in[64];
  struct serial_hal_config newcfg = ctx->cfg;

  printf("Baud rate (150..115200) [%u]: ", ctx->cfg.baud_rate);
  fflush(stdout);
  if (read_line(in, sizeof(in)) && in[0])
    newcfg.baud_rate = (uint32_t)strtoul(in, NULL, 10);

  printf("Data bits (7/8) [%u]: ", ctx->cfg.data_bits);
  fflush(stdout);
  if (read_line(in, sizeof(in)) && in[0])
    newcfg.data_bits = (uint8_t)atoi(in);

  printf("Parity (n/e/o) [%s]: ", parity_name(ctx->cfg.parity));
  fflush(stdout);
  if (read_line(in, sizeof(in)) && in[0]) {
    if (in[0] == 'e' || in[0] == 'E')
      newcfg.parity = SERIAL_HAL_PARITY_EVEN;
    else if (in[0] == 'o' || in[0] == 'O')
      newcfg.parity = SERIAL_HAL_PARITY_ODD;
    else
      newcfg.parity = SERIAL_HAL_PARITY_NONE;
  }

  printf("Stop bits (1/2) [%u]: ", ctx->cfg.stop_bits);
  fflush(stdout);
  if (read_line(in, sizeof(in)) && in[0])
    newcfg.stop_bits = (uint8_t)atoi(in);

  pthread_mutex_lock(&ctx->dev_lock);
  int ret = serial_hal_set_config(&ctx->dev, &newcfg);
  pthread_mutex_unlock(&ctx->dev_lock);

  if (ret == 0) {
    ctx->cfg = newcfg;
    printf(ANSI_COLOR_GREEN "Applied: %u %u%s%u\n" ANSI_COLOR_RESET,
           newcfg.baud_rate, newcfg.data_bits, parity_name(newcfg.parity),
           newcfg.stop_bits);
  } else {
    printf(ANSI_COLOR_RED
           "Invalid parameters (err %d). Unchanged.\n" ANSI_COLOR_RESET,
           ret);
  }
}

static void configure_flow(struct app_context *ctx) {
  char in[16];
  printf("Flow control: 0=None 1=XON/XOFF 2=RTS/CTS 3=DTR/DSR [%d]: ",
         (int)ctx->flow);
  fflush(stdout);
  if (!read_line(in, sizeof(in)) || !in[0])
    return;

  enum serial_hal_flow_control f = ctx->flow;
  switch (in[0]) {
  case '0':
    f = SERIAL_HAL_FLOW_NONE;
    break;
  case '1':
    f = SERIAL_HAL_FLOW_XON_XOFF;
    break;
  case '2':
    f = SERIAL_HAL_FLOW_RTS_CTS;
    break;
  case '3':
    f = SERIAL_HAL_FLOW_DTR_DSR;
    break;
  default:
    return;
  }

  pthread_mutex_lock(&ctx->dev_lock);
  int ret = serial_hal_set_flow_control(&ctx->dev, f);
  pthread_mutex_unlock(&ctx->dev_lock);

  if (ret == 0) {
    ctx->flow = f;
    printf(ANSI_COLOR_GREEN "Flow control: %s\n" ANSI_COLOR_RESET,
           flow_name(f));
  } else {
    printf(ANSI_COLOR_RED
           "Backend rejected flow mode (err %d).\n" ANSI_COLOR_RESET,
           ret);
  }
}

static void configure_terminator(struct app_context *ctx) {
  char in[64];
  printf("Terminator: 0=None 1=CR 2=LF 3=CR-LF 4=Custom [%s]: ",
         term_name(ctx->term.mode));
  fflush(stdout);
  if (!read_line(in, sizeof(in)) || !in[0])
    return;

  struct rs232_terminator_config t;
  memset(&t, 0, sizeof(t));
  switch (in[0]) {
  case '0':
    t.mode = RS232_TERMINATOR_NONE;
    break;
  case '1':
    t.mode = RS232_TERMINATOR_CR;
    break;
  case '2':
    t.mode = RS232_TERMINATOR_LF;
    break;
  case '3':
    t.mode = RS232_TERMINATOR_CRLF;
    break;
  case '4': {
    t.mode = RS232_TERMINATOR_CUSTOM;
    printf("Custom terminator as HEX (1-2 bytes, e.g. '0D 0A'): ");
    fflush(stdout);
    if (read_line(in, sizeof(in)) && in[0]) {
      size_t out_len = 0;
      if (rs232_hex_to_bytes(in, strlen(in), t.custom, sizeof(t.custom),
                             &out_len) == 0 &&
          out_len >= 1 && out_len <= 2) {
        t.custom_len = out_len;
      } else {
        printf(ANSI_COLOR_RED "Invalid custom terminator.\n" ANSI_COLOR_RESET);
        return;
      }
    } else {
      return;
    }
    break;
  }
  default:
    return;
  }

  if (rs232_validate_terminator(&t) == 0) {
    ctx->term = t;
    printf(ANSI_COLOR_GREEN "Terminator set: %s\n" ANSI_COLOR_RESET,
           term_name(t.mode));
  } else {
    printf(ANSI_COLOR_RED "Invalid terminator config.\n" ANSI_COLOR_RESET);
  }
}

static void send_payload(struct app_context *ctx, const uint8_t *payload,
                         size_t len) {
  uint8_t out[RX_BUFFER_SIZE + RS232_TERMINATOR_CUSTOM_MAX];
  size_t out_len = 0;
  if (rs232_apply_terminator(payload, len, &ctx->term, out, sizeof(out),
                             &out_len) != 0) {
    printf(ANSI_COLOR_RED "Payload too large.\n" ANSI_COLOR_RESET);
    return;
  }
  bool was_listening = atomic_load(&ctx->rx_listen);
  atomic_store(&ctx->rx_listen, false);
  pthread_mutex_lock(&ctx->dev_lock);
  ssize_t w = serial_hal_write(&ctx->dev, out, out_len, 1000);
  pthread_mutex_unlock(&ctx->dev_lock);
  atomic_store(&ctx->rx_listen, was_listening);
  if (w < 0) {
    printf(ANSI_COLOR_RED "Write failed (err %zd).\n" ANSI_COLOR_RESET, w);
    return;
  }
  print_hex("[TX]", out, out_len);
}

static void send_text(struct app_context *ctx) {
  char in[RX_BUFFER_SIZE];
  printf("Text to send (window 'Nadawanie'): ");
  fflush(stdout);
  if (read_line(in, sizeof(in)))
    send_payload(ctx, (const uint8_t *)in, strlen(in));
}

static void send_binary_hex(struct app_context *ctx) {
  char in[RX_BUFFER_SIZE];
  uint8_t bytes[RX_BUFFER_SIZE];
  size_t out_len = 0;
  printf("Bytes as HEX (e.g. 'DE AD BE EF'): ");
  fflush(stdout);
  if (!read_line(in, sizeof(in)) || !in[0])
    return;
  if (rs232_hex_to_bytes(in, strlen(in), bytes, sizeof(bytes), &out_len) != 0) {
    printf(ANSI_COLOR_RED "Invalid HEX input.\n" ANSI_COLOR_RESET);
    return;
  }
  send_payload(ctx, bytes, out_len);
}

static void send_binary_file(struct app_context *ctx) {
  char path[512];
  uint8_t buf[RX_BUFFER_SIZE];
  printf("Path to binary file: ");
  fflush(stdout);
  if (!read_line(path, sizeof(path)) || !path[0])
    return;
  FILE *f = fopen(path, "rb");
  if (!f) {
    printf(ANSI_COLOR_RED "Cannot open file.\n" ANSI_COLOR_RESET);
    return;
  }
  size_t total = 0;
  size_t r;
  bool was_listening = atomic_load(&ctx->rx_listen);
  atomic_store(&ctx->rx_listen, false);
  pthread_mutex_lock(&ctx->dev_lock);
  while ((r = fread(buf, 1, sizeof(buf), f)) > 0) {
    if (serial_hal_write(&ctx->dev, buf, r, 2000) < 0)
      break;
    total += r;
  }
  pthread_mutex_unlock(&ctx->dev_lock);
  atomic_store(&ctx->rx_listen, was_listening);
  fclose(f);
  printf(ANSI_COLOR_GREEN "Sent %zu bytes from file.\n" ANSI_COLOR_RESET,
         total);
}

/** send + wait for reply within a timeout */
static void rs232_transaction(struct app_context *ctx) {
  char in[RX_BUFFER_SIZE];
  uint8_t out[RX_BUFFER_SIZE + RS232_TERMINATOR_CUSTOM_MAX];
  uint8_t rx[RX_BUFFER_SIZE];
  size_t out_len = 0;
  uint32_t timeout_ms;

  printf("Query text: ");
  fflush(stdout);
  if (!read_line(in, sizeof(in)))
    return;
  printf("Response timeout [ms]: ");
  fflush(stdout);
  char tin[32];
  if (!read_line(tin, sizeof(tin)) || !tin[0])
    timeout_ms = 1000;
  else
    timeout_ms = (uint32_t)strtoul(tin, NULL, 10);

  if (rs232_apply_terminator((const uint8_t *)in, strlen(in), &ctx->term, out,
                             sizeof(out), &out_len) != 0) {
    printf(ANSI_COLOR_RED "Query too large.\n" ANSI_COLOR_RESET);
    return;
  }

  /* take port exclusively so the passive listener wont steal the reply */
  atomic_store(&ctx->rx_listen, false);
  usleep(80000);
  pthread_mutex_lock(&ctx->dev_lock);

  serial_hal_flush(&ctx->dev);
  serial_hal_write(&ctx->dev, out, out_len, 1000);
  print_hex("[TX]", out, out_len);

  /* reassemble reply until the terminator sentinel (or timeout) so fragmented
   * USB-UART response is collected into one message */
  uint8_t sentinel;
  bool has_sentinel = passive_sentinel(ctx, &sentinel);
  ssize_t n;
  if (has_sentinel) {
    size_t got = 0;
    uint64_t deadline = now_ms() + timeout_ms;
    n = -ETIMEDOUT;
    while (got < sizeof(rx) - 1) {
      uint64_t t = now_ms();
      if (t >= deadline)
        break;
      ssize_t r =
          serial_hal_read(&ctx->dev, rx + got, 1, (uint32_t)(deadline - t));
      if (r == -ETIMEDOUT || r == -EAGAIN)
        break;
      if (r < 0) {
        n = r;
        break;
      }
      if (r == 0)
        continue;
      got += (size_t)r;
      if (rx[got - 1] == sentinel) {
        n = (ssize_t)got;
        break;
      }
    }
    if (n == -ETIMEDOUT && got > 0)
      n = (ssize_t)got; /* partial but something arrived */
  } else {
    n = serial_hal_read(&ctx->dev, rx, sizeof(rx) - 1, timeout_ms);
  }

  pthread_mutex_unlock(&ctx->dev_lock);
  atomic_store(&ctx->rx_listen, true);

  if (n == -ETIMEDOUT) {
    printf(ANSI_COLOR_RED
           "[TRANSACTION] Timeout after %ums, no reply.\n" ANSI_COLOR_RESET,
           timeout_ms);
  } else if (n < 0) {
    printf(ANSI_COLOR_RED "[TRANSACTION] Read error %zd.\n" ANSI_COLOR_RESET,
           n);
  } else {
    rx[n] = '\0';
    print_hex("[RX]", rx, (size_t)n);
    printf(ANSI_COLOR_GREEN "[TRANSACTION] Reply: %s\n" ANSI_COLOR_RESET, rx);
  }
}

/*
 * PING with round-trip delay measurement.
 * Sends "PING"+terminator and waits for "PONG" reply, reassembling the answer
 * until the line terminator so fragmented USB-UART reads are handled
 */
static void do_ping(struct app_context *ctx) {
  uint8_t probe[16];
  uint8_t rx[64];
  size_t probe_len = 0;

  if (rs232_apply_terminator((const uint8_t *)"PING", 4, &ctx->term, probe,
                             sizeof(probe), &probe_len) != 0)
    return;

  atomic_store(&ctx->rx_listen, false);
  usleep(80000);
  pthread_mutex_lock(&ctx->dev_lock);

  serial_hal_flush(&ctx->dev);
  uint64_t start = now_ms();
  serial_hal_write(&ctx->dev, probe, probe_len, 1000);
  ssize_t n = read_frame_lf(ctx, rx, sizeof(rx), 2000, 100);
  uint64_t elapsed = now_ms() - start;

  pthread_mutex_unlock(&ctx->dev_lock);
  atomic_store(&ctx->rx_listen, true);

  if (n >= 4 && memcmp(rx, "PONG", 4) == 0) {
    printf(
        ANSI_COLOR_GREEN
        "[PING] OK. Round-trip delay: %llu ms, %zd bytes.\n" ANSI_COLOR_RESET,
        (unsigned long long)elapsed, n);
  } else if (n == -ETIMEDOUT) {
    printf(ANSI_COLOR_RED
           "[PING] Timeout - no PONG. Is the peer running?\n" ANSI_COLOR_RESET);
  } else if (n > 0) {
    print_hex("[PING] Unexpected reply", rx, (size_t)n);
  } else {
    printf(ANSI_COLOR_RED "[PING] Failed (err %zd).\n" ANSI_COLOR_RESET, n);
  }
}

static void master_transaction(struct app_context *ctx) {
  char in[RX_BUFFER_SIZE];
  struct modbus_frame req;
  uint8_t wire[MODBUS_MAX_ADU_SIZE * 2];
  uint8_t rx[RX_BUFFER_SIZE];
  size_t wire_len = 0;
  unsigned addr;
  unsigned func;

  printf("Slave address (0 = broadcast, 1..247): ");
  fflush(stdout);
  if (!read_line(in, sizeof(in)))
    return;
  addr = (unsigned)strtoul(in, NULL, 10);
  if (addr > 247) {
    printf(ANSI_COLOR_RED "Address out of range.\n" ANSI_COLOR_RESET);
    return;
  }

  printf("Function (1 = write text, 2 = read text): ");
  fflush(stdout);
  if (!read_line(in, sizeof(in)))
    return;
  func = (unsigned)strtoul(in, NULL, 10);

  memset(&req, 0, sizeof(req));
  req.server_address = (uint8_t)addr;
  req.pdu.function_code = (uint8_t)func;

  if (func == 1) {
    printf("Text to write into slave: ");
    fflush(stdout);
    if (!read_line(in, sizeof(in)))
      return;
    size_t tlen = strlen(in);
    if (tlen > MODBUS_MAX_ADU_SIZE)
      tlen = MODBUS_MAX_ADU_SIZE;
    memcpy(req.pdu.data, in, tlen);
    req.pdu.data_len = tlen;
  } else if (func == 2) {
    req.pdu.data_len = 0;
    if (addr == 0) {
      printf(ANSI_COLOR_RED
             "Read (func 2) cannot be broadcast.\n" ANSI_COLOR_RESET);
      return;
    }
  } else {
    printf("Arguments as HEX (optional): ");
    fflush(stdout);
    if (read_line(in, sizeof(in)) && in[0]) {
      size_t dl = 0;
      if (rs232_hex_to_bytes(in, strlen(in), req.pdu.data, MODBUS_MAX_ADU_SIZE,
                             &dl) == 0)
        req.pdu.data_len = dl;
    }
  }

  if (mb_encode(ctx, &req, wire, sizeof(wire), &wire_len) != 0) {
    printf(ANSI_COLOR_RED "Failed to encode frame.\n" ANSI_COLOR_RESET);
    return;
  }

  bool broadcast = (addr == 0);
  uint32_t timeout = ctx->mb_timeout_ms ? ctx->mb_timeout_ms : 1000;

  atomic_store(&ctx->rx_listen, false);
  atomic_store(&ctx->slave_mode, false);
  usleep(80000);

  for (int attempt = 0; attempt <= ctx->mb_retries; ++attempt) {
    pthread_mutex_lock(&ctx->dev_lock);
    serial_hal_flush(&ctx->dev);
    serial_hal_write(&ctx->dev, wire, wire_len, 1000);
    print_hex("[MASTER TX]", wire, wire_len);

    if (broadcast) {
      pthread_mutex_unlock(&ctx->dev_lock);
      printf(ANSI_COLOR_CYAN "[MASTER] Broadcast sent (no reply "
                             "expected).\n" ANSI_COLOR_RESET);
      break;
    }

    /* wait for our TX to physically drain before reading
     * USB-UART can clip the first reply byte if it arrives mid-turnaround,
     * so delay is scaled to transmitted frame length (~10 bits per byte) */
    uint32_t drain_us =
        (uint32_t)((wire_len * 10ULL * 1000000ULL) / ctx->cfg.baud_rate) + 3000;
    usleep(drain_us);
    ssize_t n = mb_read_frame(ctx, rx, sizeof(rx), timeout, ctx->mb_gap_ms);
    pthread_mutex_unlock(&ctx->dev_lock);

    if (n <= 0) {
      printf(
          ANSI_COLOR_YELLOW
          "[MASTER] No reply (attempt %d/%d, timeout %ums).\n" ANSI_COLOR_RESET,
          attempt + 1, ctx->mb_retries + 1, timeout);
      continue;
    }

    print_hex("[MASTER RX]", rx, (size_t)n);
    struct modbus_frame resp;
    int ret = mb_decode(ctx, rx, (size_t)n, &resp);
    if (ret != 0 || !resp.is_valid) {
      printf(
          ANSI_COLOR_RED
          "[MASTER] Invalid reply (LRC/format). Retrying.\n" ANSI_COLOR_RESET);
      continue;
    }

    if (resp.pdu.function_code & 0x80) {
      printf(ANSI_COLOR_RED "[MASTER] Exception response: func %02X, code "
                            "%02X.\n" ANSI_COLOR_RESET,
             resp.pdu.function_code, resp.pdu.data_len ? resp.pdu.data[0] : 0);
    } else if (func == 2) {
      char txt[MODBUS_MAX_ADU_SIZE + 1];
      size_t tl = resp.pdu.data_len;
      if (tl > MODBUS_MAX_ADU_SIZE)
        tl = MODBUS_MAX_ADU_SIZE;
      memcpy(txt, resp.pdu.data, tl);
      txt[tl] = '\0';
      printf(ANSI_COLOR_GREEN
             "[MASTER] Tekst odebrany ze slave: \"%s\"\n" ANSI_COLOR_RESET,
             txt);
    } else {
      printf(ANSI_COLOR_GREEN
             "[MASTER] Normal response, func %02X.\n" ANSI_COLOR_RESET,
             resp.pdu.function_code);
    }
    break; /* success */
  }

  atomic_store(&ctx->rx_listen, true);
}

static void configure_master_params(struct app_context *ctx) {
  char in[32];
  printf("Transaction timeout [ms] (0..10000) [%u]: ", ctx->mb_timeout_ms);
  fflush(stdout);
  if (read_line(in, sizeof(in)) && in[0]) {
    uint32_t v = (uint32_t)strtoul(in, NULL, 10);
    if (v <= 10000)
      ctx->mb_timeout_ms = v;
  }
  printf("Retransmissions (0..5) [%u]: ", ctx->mb_retries);
  fflush(stdout);
  if (read_line(in, sizeof(in)) && in[0]) {
    int v = atoi(in);
    if (v >= 0 && v <= 5)
      ctx->mb_retries = (uint8_t)v;
  }
  printf("Inter-character gap [ms] (0..1000) [%u]: ", ctx->mb_gap_ms);
  fflush(stdout);
  if (read_line(in, sizeof(in)) && in[0]) {
    uint32_t v = (uint32_t)strtoul(in, NULL, 10);
    if (v <= 1000)
      ctx->mb_gap_ms = v;
  }
  printf(ANSI_COLOR_GREEN
         "Master params: timeout=%ums, retries=%u, gap=%ums\n" ANSI_COLOR_RESET,
         ctx->mb_timeout_ms, ctx->mb_retries, ctx->mb_gap_ms);
}

static void set_slave_address(struct app_context *ctx) {
  char in[32];
  printf("Slave address (1..247) [%u]: ", ctx->slave_address);
  fflush(stdout);
  if (read_line(in, sizeof(in)) && in[0]) {
    int v = atoi(in);
    if (v >= 1 && v <= 247)
      ctx->slave_address = (uint8_t)v;
    else
      printf(ANSI_COLOR_RED "Out of range 1..247.\n" ANSI_COLOR_RESET);
  }
  printf("Inter-character gap [ms] (0..1000) [%u]: ", ctx->mb_gap_ms);
  fflush(stdout);
  if (read_line(in, sizeof(in)) && in[0]) {
    uint32_t v = (uint32_t)strtoul(in, NULL, 10);
    if (v <= 1000)
      ctx->mb_gap_ms = v;
  }
  printf(ANSI_COLOR_GREEN "Slave addr=%u, gap=%ums\n" ANSI_COLOR_RESET,
         ctx->slave_address, ctx->mb_gap_ms);
}

static void run_slave_mode(struct app_context *ctx) {
  char in[16];
  printf(ANSI_COLOR_CYAN "\n[SLAVE] Listening as address %u (gap=%ums). "
                         "Press ENTER to stop.\n" ANSI_COLOR_RESET,
         ctx->slave_address, ctx->mb_gap_ms);
  atomic_store(&ctx->rx_listen, false);
  atomic_store(&ctx->slave_mode, true);
  read_line(in, sizeof(in)); /* blocks until user stops slave mode */
  atomic_store(&ctx->slave_mode, false);
  printf(ANSI_COLOR_CYAN "[SLAVE] Stopped.\n" ANSI_COLOR_RESET);
}

static void modbus_menu(struct app_context *ctx) {
  char in[32];
  for (;;) {
    printf("\n" ANSI_COLOR_CYAN
           "=== MODBUS (role: %s, mode: %s) ===" ANSI_COLOR_RESET "\n",
           ctx->role == MODBUS_ROLE_MASTER ? "MASTER" : "SLAVE",
           mb_mode_name(ctx->mb_mode));
    printf("1. Set role (Master/Slave)\n");
    printf("2. Master: run transaction (func 1 write / 2 read)\n");
    printf("3. Master: set params (timeout/retries/gap)\n");
    printf("4. Slave: set address & gap\n");
    printf("5. Slave: enter listen mode\n");
    printf("6. Toggle mode (ASCII/RTU)\n");
    printf("b. Back\n> ");
    fflush(stdout);
    if (!read_line(in, sizeof(in)))
      return;

    if (in[0] == 'b' || in[0] == 'B')
      return;
    switch (in[0]) {
    case '1':
      printf("Role: 0=Master 1=Slave: ");
      fflush(stdout);
      if (read_line(in, sizeof(in)) && in[0]) {
        ctx->role = (in[0] == '1') ? MODBUS_ROLE_SLAVE : MODBUS_ROLE_MASTER;
        printf(ANSI_COLOR_GREEN "Role = %s\n" ANSI_COLOR_RESET,
               ctx->role == MODBUS_ROLE_MASTER ? "MASTER" : "SLAVE");
      }
      break;
    case '2':
      if (ctx->role != MODBUS_ROLE_MASTER)
        printf(ANSI_COLOR_YELLOW
               "Station is in SLAVE role.\n" ANSI_COLOR_RESET);
      master_transaction(ctx);
      break;
    case '3':
      configure_master_params(ctx);
      break;
    case '4':
      set_slave_address(ctx);
      break;
    case '5':
      if (ctx->role != MODBUS_ROLE_MASTER)
        run_slave_mode(ctx);
      else
        printf(ANSI_COLOR_YELLOW
               "Set role to Slave first (option 1).\n" ANSI_COLOR_RESET);
      break;
    case '6':
      ctx->mb_mode = (ctx->mb_mode == MODBUS_MODE_ASCII) ? MODBUS_MODE_RTU
                                                         : MODBUS_MODE_ASCII;
      printf(ANSI_COLOR_GREEN "MODBUS mode = %s\n" ANSI_COLOR_RESET,
             mb_mode_name(ctx->mb_mode));
      break;
    default:
      break;
    }
  }
}

/** enumerate serial ports physically present on the system */
static int list_serial_ports(char ports[][PORT_NAME_MAX], int max) {
  int count = 0;
#ifdef _WIN32
  for (int i = 1; i <= 64 && count < max; ++i) {
    char path[24];
    snprintf(path, sizeof(path), "\\\\.\\COM%d", i);
    HANDLE h = CreateFileA(path, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                           OPEN_EXISTING, 0, NULL);
    if (h != INVALID_HANDLE_VALUE) {
      CloseHandle(h);
      snprintf(ports[count++], PORT_NAME_MAX, "COM%d", i);
    }
  }
#else
  static const char *prefix[] = {"ttyUSB", "ttyACM", "ttyS"};
  DIR *d = opendir("/dev");
  if (!d)
    return 0;
  struct dirent *e;
  while ((e = readdir(d)) != NULL && count < max) {
    for (size_t k = 0; k < sizeof(prefix) / sizeof(prefix[0]); ++k) {
      if (strncmp(e->d_name, prefix[k], strlen(prefix[k])) == 0) {
        snprintf(ports[count++], PORT_NAME_MAX, "/dev/%.24s", e->d_name);
        break;
      }
    }
  }
  closedir(d);
#endif
  return count;
}

/** list present ports, let the user pick one, and reopen device on it */
static void select_port(struct app_context *ctx) {
  char ports[PORT_LIST_MAX][PORT_NAME_MAX];
  char in[64];
  char old[SERIAL_HAL_DEVICE_NAME_MAX];
  int n = list_serial_ports(ports, PORT_LIST_MAX);

  if (n == 0) {
    printf(ANSI_COLOR_RED "No serial ports detected.\n" ANSI_COLOR_RESET);
    return;
  }
  printf("Detected serial ports (presence-checked):\n");
  for (int i = 0; i < n; ++i)
    printf("  %d. %s\n", i + 1, ports[i]);
  printf("Select [1..%d] (current %s): ", n, ctx->cfg.device);
  fflush(stdout);
  if (!read_line(in, sizeof(in)) || !in[0])
    return;
  int sel = atoi(in);
  if (sel < 1 || sel > n) {
    printf(ANSI_COLOR_RED "Invalid selection.\n" ANSI_COLOR_RESET);
    return;
  }

  strncpy(old, ctx->cfg.device, sizeof(old) - 1);
  old[sizeof(old) - 1] = '\0';

  bool was_listening = atomic_load(&ctx->rx_listen);
  atomic_store(&ctx->rx_listen, false);
  atomic_store(&ctx->slave_mode, false);
  usleep(80000);
  pthread_mutex_lock(&ctx->dev_lock);

  serial_hal_close(&ctx->dev);
  strncpy(ctx->cfg.device, ports[sel - 1], sizeof(ctx->cfg.device) - 1);
  ctx->cfg.device[sizeof(ctx->cfg.device) - 1] = '\0';

  int r = serial_hal_open(&ctx->dev, &ctx->cfg);
  if (r < 0) {
    strncpy(ctx->cfg.device, old, sizeof(ctx->cfg.device) - 1);
    serial_hal_open(&ctx->dev, &ctx->cfg);
    printf(ANSI_COLOR_RED
           "Failed to open %s (err %d). Reverted to %s.\n" ANSI_COLOR_RESET,
           ports[sel - 1], r, old);
  } else {
    serial_hal_set_flow_control(&ctx->dev, ctx->flow);
    ctx->current_lines.dtr = true;
    ctx->current_lines.rts = true;
    serial_hal_set_modem_lines(&ctx->dev, &ctx->current_lines);
    printf(ANSI_COLOR_GREEN "Port switched to %s.\n" ANSI_COLOR_RESET,
           ctx->cfg.device);
  }

  pthread_mutex_unlock(&ctx->dev_lock);
  atomic_store(&ctx->rx_listen, was_listening);
}

/** autobauding: probe baud rates until the peer answers, adopt the match */
static void autobaud(struct app_context *ctx) {
  static const uint32_t candidates[] = {1200,  2400,  4800,  9600,
                                        19200, 38400, 57600, 115200};
  uint8_t probe[16];
  uint8_t rx[64];
  size_t probe_len = 0;
  uint32_t original = ctx->cfg.baud_rate;
  bool found = false;

  if (rs232_apply_terminator((const uint8_t *)"PING", 4, &ctx->term, probe,
                             sizeof(probe), &probe_len) != 0)
    return;

  printf(ANSI_COLOR_CYAN
         "[AUTOBAUD] Probing baud rates (peer must be in passive"
         " listen)...\n" ANSI_COLOR_RESET);

  atomic_store(&ctx->rx_listen, false);
  atomic_store(&ctx->slave_mode, false);
  usleep(80000);
  pthread_mutex_lock(&ctx->dev_lock);

  for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); ++i) {
    struct serial_hal_config c = ctx->cfg;
    c.baud_rate = candidates[i];
    if (serial_hal_set_config(&ctx->dev, &c) != 0)
      continue;
    usleep(50000); /* let adapter apply the new baud */
    serial_hal_flush(&ctx->dev);
    /* stay silent so the peer's listener drops any garbage it received
     * from previous (wrong-baud) probe and is ready for clean PING */
    usleep(350000);
    serial_hal_flush(&ctx->dev);
    serial_hal_write(&ctx->dev, probe, probe_len, 500);
    ssize_t n = read_frame_lf(ctx, rx, sizeof(rx), 1000, 100);
    bool ok = false;
    for (ssize_t j = 0; n >= 4 && j + 4 <= n; ++j) {
      if (memcmp(rx + j, "PONG", 4) == 0) {
        ok = true;
        break;
      }
    }
    printf("  %6u baud -> %s\n", candidates[i],
           ok ? "PONG (match)" : "no reply");
    if (ok) {
      ctx->cfg.baud_rate = candidates[i];
      found = true;
      break;
    }
  }

  if (!found) {
    struct serial_hal_config c = ctx->cfg;
    c.baud_rate = original;
    serial_hal_set_config(&ctx->dev, &c);
    ctx->cfg.baud_rate = original;
  }

  pthread_mutex_unlock(&ctx->dev_lock);
  atomic_store(&ctx->rx_listen, true);

  if (found)
    printf(ANSI_COLOR_GREEN
           "[AUTOBAUD] Detected peer baud rate: %u\n" ANSI_COLOR_RESET,
           ctx->cfg.baud_rate);
  else
    printf(ANSI_COLOR_RED
           "[AUTOBAUD] No peer responded; baud left at %u.\n" ANSI_COLOR_RESET,
           original);
}

static void print_menu(void) {
  printf("\n" ANSI_COLOR_CYAN "=== IWSK: RS-232 & MODBUS ===" ANSI_COLOR_RESET
         "\n");
  printf(
      " Config : p=select port  c=serial params  f=flow ctrl  t=terminator\n");
  printf("          i=info  a=autobaud\n");
  printf(" RS-232 : 1=send text  2=send HEX  3=send file\n");
  printf("          4=transaction(timeout)  5=PING(round-trip)\n");
  printf(" Modem  : d=toggle DTR  r=toggle RTS  l=line status\n");
  printf(" MODBUS : m=MODBUS station menu\n");
  printf(" q=quit\n> ");
  fflush(stdout);
}

int main(int argc, char **argv) {
  struct app_context ctx;
  char input[256];
  int ret;
  int exit_code = EXIT_SUCCESS;
  bool rx_thread_started = false;

  if (argc < 2) {
    fprintf(stderr,
            "Usage: %s <serial_port> (e.g. /tmp/ttyV0 or /dev/ttyUSB0)\n",
            argv[0]);
    return EXIT_FAILURE;
  }

  memset(&ctx, 0, sizeof(ctx));
  atomic_init(&ctx.is_running, true);
  atomic_init(&ctx.rx_listen, true);
  atomic_init(&ctx.slave_mode, false);
  pthread_mutex_init(&ctx.dev_lock, NULL);

  /* MODBUS defaults */
  ctx.mb_mode = MODBUS_MODE_ASCII;
  ctx.role = MODBUS_ROLE_MASTER;
  ctx.slave_address = 1;
  ctx.mb_timeout_ms = 1000;
  ctx.mb_retries = 5;
  ctx.mb_gap_ms = 100;
  strcpy(ctx.slave_text, "Hello from slave");

  /* default terminator CR-LF */
  ctx.term.mode = RS232_TERMINATOR_CRLF;
  ctx.flow = SERIAL_HAL_FLOW_NONE;

#ifdef _WIN32
  enable_windows_ansi();
  if (serial_hal_windows_init(&ctx.dev) < 0) {
    fprintf(stderr, "Failed to init Windows HAL.\n");
    return EXIT_FAILURE;
  }
#else
  if (serial_hal_linux_init(&ctx.dev) < 0) {
    fprintf(stderr, "Failed to init Linux HAL.\n");
    return EXIT_FAILURE;
  }
#endif

  memset(&ctx.cfg, 0, sizeof(ctx.cfg));
  strncpy(ctx.cfg.device, argv[1], sizeof(ctx.cfg.device) - 1);
  ctx.cfg.baud_rate = 9600;
  ctx.cfg.data_bits = 8;
  ctx.cfg.stop_bits = 1;
  ctx.cfg.parity = SERIAL_HAL_PARITY_NONE;

  if ((ret = serial_hal_open(&ctx.dev, &ctx.cfg)) < 0) {
    fprintf(stderr,
            "Failed to open %s (Error: %d). Check permissions or socat.\n",
            argv[1], ret);
    exit_code = EXIT_FAILURE;
    goto cleanup;
  }

  ctx.current_lines.dtr = true;
  ctx.current_lines.rts = true;
  serial_hal_set_modem_lines(&ctx.dev, &ctx.current_lines);

  if (pthread_create(&ctx.rx_thread, NULL, rx_worker_thread, &ctx) != 0) {
    fprintf(stderr, "Failed to spawn RX thread.\n");
    exit_code = EXIT_FAILURE;
    goto cleanup;
  }
  rx_thread_started = true;

  while (atomic_load(&ctx.is_running)) {
    print_menu();
    if (!read_line(input, sizeof(input)))
      break;

    switch (input[0]) {
    case 'q':
    case 'Q':
      atomic_store(&ctx.is_running, false);
      break;
    case 'c':
      configure_serial(&ctx);
      break;
    case 'f':
      configure_flow(&ctx);
      break;
    case 't':
      configure_terminator(&ctx);
      break;
    case 'i':
      show_config(&ctx);
      break;
    case 'p':
      select_port(&ctx);
      break;
    case 'a':
      autobaud(&ctx);
      break;
    case '1':
      send_text(&ctx);
      break;
    case '2':
      send_binary_hex(&ctx);
      break;
    case '3':
      send_binary_file(&ctx);
      break;
    case '4':
      rs232_transaction(&ctx);
      break;
    case '5':
      do_ping(&ctx);
      break;
    case 'd':
      ctx.current_lines.dtr = !ctx.current_lines.dtr;
      serial_hal_set_modem_lines(&ctx.dev, &ctx.current_lines);
      printf(ANSI_COLOR_YELLOW "[MODEM]" ANSI_COLOR_RESET " DTR -> %d\n",
             ctx.current_lines.dtr);
      break;
    case 'r':
      ctx.current_lines.rts = !ctx.current_lines.rts;
      serial_hal_set_modem_lines(&ctx.dev, &ctx.current_lines);
      printf(ANSI_COLOR_YELLOW "[MODEM]" ANSI_COLOR_RESET " RTS -> %d\n",
             ctx.current_lines.rts);
      break;
    case 'l': {
      struct serial_hal_modem_lines st;
      if (serial_hal_get_modem_lines(&ctx.dev, &st) == 0)
        printf(ANSI_COLOR_YELLOW
               "[STATUS]" ANSI_COLOR_RESET
               " DTR:%d RTS:%d | DSR:%d CTS:%d DCD:%d RI:%d\n",
               st.dtr, st.rts, st.dsr, st.cts, st.dcd, st.ri);
      else
        printf(ANSI_COLOR_RED "Failed to read modem lines.\n" ANSI_COLOR_RESET);
      break;
    }
    case 'm':
      modbus_menu(&ctx);
      break;
    default:
      break;
    }
  }

  printf("Shutting down gracefully...\n");

cleanup:
  if (rx_thread_started) {
    atomic_store(&ctx.is_running, false);
    pthread_join(ctx.rx_thread, NULL);
  }
  if (ctx.dev.is_open)
    serial_hal_close(&ctx.dev);

  pthread_mutex_destroy(&ctx.dev_lock);

#ifdef _WIN32
  serial_hal_windows_deinit(&ctx.dev);
#else
  serial_hal_linux_deinit(&ctx.dev);
#endif

  return exit_code;
}
