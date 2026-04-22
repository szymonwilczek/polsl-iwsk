#include "rs232/rs232_logic.h"

#include <errno.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

static int rs232_get_terminator(const struct rs232_terminator_config *cfg,
                                const uint8_t **term, size_t *term_len) {
  static const uint8_t cr = '\r';
  static const uint8_t lf = '\n';
  static const uint8_t crlf[2] = {'\r', '\n'};

  if (!cfg || !term || !term_len)
    return -EINVAL;

  switch (cfg->mode) {
  case RS232_TERMINATOR_NONE:
    *term = NULL;
    *term_len = 0;
    return 0;
  case RS232_TERMINATOR_CR:
    *term = &cr;
    *term_len = 1;
    return 0;
  case RS232_TERMINATOR_LF:
    *term = &lf;
    *term_len = 1;
    return 0;
  case RS232_TERMINATOR_CRLF:
    *term = crlf;
    *term_len = sizeof(crlf);
    return 0;
  case RS232_TERMINATOR_CUSTOM:
    if (!cfg->custom_len || cfg->custom_len > RS232_TERMINATOR_CUSTOM_MAX)
      return -EINVAL;
    *term = cfg->custom;
    *term_len = cfg->custom_len;
    return 0;
  default:
    return -EINVAL;
  }
}

int rs232_validate_terminator(const struct rs232_terminator_config *cfg) {
  const uint8_t *term;
  size_t term_len;

  return rs232_get_terminator(cfg, &term, &term_len);
}

int rs232_apply_terminator(const uint8_t *payload, size_t payload_len,
                           const struct rs232_terminator_config *cfg,
                           uint8_t *out, size_t out_size, size_t *out_len) {
  const uint8_t *term;
  size_t term_len;
  size_t total_len;
  int ret;

  if ((!payload && payload_len) || !cfg || !out || !out_len)
    return -EINVAL;

  ret = rs232_get_terminator(cfg, &term, &term_len);
  if (ret)
    return ret;

  total_len = payload_len + term_len;
  if (out_size < total_len)
    return -ENOSPC;

  if (payload_len)
    memcpy(out, payload, payload_len);

  if (term_len)
    memcpy(out + payload_len, term, term_len);

  *out_len = total_len;

  return 0;
}

int rs232_strip_terminator(uint8_t *data, size_t *data_len,
                           const struct rs232_terminator_config *cfg) {
  const uint8_t *term;
  size_t term_len;
  int ret;

  if (!data_len || !cfg)
    return -EINVAL;

  ret = rs232_get_terminator(cfg, &term, &term_len);
  if (ret)
    return ret;

  if (!term_len)
    return 0;

  if (!data || *data_len < term_len)
    return -EBADMSG;

  if (memcmp(data + (*data_len - term_len), term, term_len))
    return -EBADMSG;

  *data_len -= term_len;

  return 0;
}

static bool rs232_is_hex_separator(char c) {
  if (c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '-' || c == ':')
    return true;

  return false;
}

static int rs232_hex_nibble(char c, uint8_t *value) {
  if (!value)
    return -EINVAL;

  if (c >= '0' && c <= '9') {
    *value = (uint8_t)(c - '0');
    return 0;
  }

  if (c >= 'a' && c <= 'f') {
    *value = (uint8_t)(c - 'a' + 10);
    return 0;
  }

  if (c >= 'A' && c <= 'F') {
    *value = (uint8_t)(c - 'A' + 10);
    return 0;
  }

  return -EINVAL;
}

static char rs232_hex_char(uint8_t nibble, bool upper_case) {
  if (nibble < 10)
    return (char)('0' + nibble);

  if (upper_case)
    return (char)('A' + (nibble - 10));

  return (char)('a' + (nibble - 10));
}

int rs232_bytes_to_hex(const uint8_t *input, size_t input_len, bool upper_case,
                       bool spaced, char *output, size_t output_size,
                       size_t *output_len) {
  size_t out_pos;
  size_t i;
  size_t needed;

  if ((!input && input_len) || !output || !output_len)
    return -EINVAL;

  needed = input_len * 2;
  if (spaced && input_len > 1)
    needed += input_len - 1;

  if (output_size <= needed)
    return -ENOSPC;

  out_pos = 0;
  for (i = 0; i < input_len; ++i) {
    output[out_pos++] = rs232_hex_char((uint8_t)(input[i] >> 4), upper_case);
    output[out_pos++] = rs232_hex_char((uint8_t)(input[i] & 0x0F), upper_case);

    if (spaced && i + 1 < input_len)
      output[out_pos++] = ' ';
  }

  output[out_pos] = '\0';
  *output_len = out_pos;

  return 0;
}

int rs232_hex_to_bytes(const char *hex_text, size_t hex_len, uint8_t *output,
                       size_t output_size, size_t *output_len) {
  int high_nibble;
  size_t out_pos;
  size_t i;

  if ((!hex_text && hex_len) || !output || !output_len)
    return -EINVAL;

  high_nibble = -1;
  out_pos = 0;

  for (i = 0; i < hex_len; ++i) {
    uint8_t nibble;
    int ret;

    if (rs232_is_hex_separator(hex_text[i]))
      continue;

    ret = rs232_hex_nibble(hex_text[i], &nibble);
    if (ret)
      return ret;

    if (high_nibble < 0) {
      high_nibble = (int)nibble;
      continue;
    }

    if (out_pos >= output_size)
      return -ENOSPC;

    output[out_pos++] = (uint8_t)((high_nibble << 4) | (int)nibble);
    high_nibble = -1;
  }

  if (high_nibble >= 0)
    return -EINVAL;

  *output_len = out_pos;

  return 0;
}

static uint64_t rs232_now_us(void) {
#ifdef _WIN32
  LARGE_INTEGER freq;
  LARGE_INTEGER counter;

  if (!QueryPerformanceFrequency(&freq) || !freq.QuadPart)
    return 0;

  if (!QueryPerformanceCounter(&counter))
    return 0;

  return (uint64_t)((counter.QuadPart * 1000000ULL) / freq.QuadPart);
#else
  struct timeval tv;

  if (gettimeofday(&tv, NULL) < 0)
    return 0;

  return ((uint64_t)tv.tv_sec * 1000000ULL) + (uint64_t)tv.tv_usec;
#endif
}

int rs232_ping(struct serial_hal_device *dev,
               const struct rs232_ping_request *req, uint8_t *rx_buf,
               size_t rx_buf_size, struct rs232_ping_result *result) {
  uint64_t start_us;
  uint64_t end_us;
  ssize_t bytes_written;
  ssize_t bytes_read;

  if (!dev || !req || !rx_buf || !rx_buf_size || !result)
    return -EINVAL;

  if (!req->tx_data || !req->tx_len)
    return -EINVAL;

  if (req->rx_prefix_len && !req->rx_prefix)
    return -EINVAL;

  memset(result, 0, sizeof(*result));

  start_us = rs232_now_us();

  bytes_written =
      serial_hal_write(dev, req->tx_data, req->tx_len, req->tx_timeout_ms);
  if (bytes_written < 0)
    return (int)bytes_written;

  if ((size_t)bytes_written != req->tx_len)
    return -EIO;

  bytes_read = serial_hal_read(dev, rx_buf, rx_buf_size, req->rx_timeout_ms);
  if (bytes_read < 0)
    return (int)bytes_read;

  end_us = rs232_now_us();
  if (end_us >= start_us)
    result->round_trip_us = end_us - start_us;

  result->rx_len = bytes_read;

  if (req->rx_prefix_len) {
    if ((size_t)bytes_read < req->rx_prefix_len)
      return -EBADMSG;

    if (memcmp(rx_buf, req->rx_prefix, req->rx_prefix_len))
      return -EBADMSG;
  }

  return 0;
}
