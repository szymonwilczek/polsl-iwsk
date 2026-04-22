#include "protocol/modbus_core.h"

#include <errno.h>
#include <string.h>

/* Fast HEX encoding lookup table */
static const char hex_lut[] = "0123456789ABCDEF";

/**
 * @brief Helper: Convert a single byte to two HEX characters.
 */
static inline void modbus_byte_to_hex(uint8_t byte, char *out) {
  out[0] = hex_lut[byte >> 4];
  out[1] = hex_lut[byte & 0x0F];
}

/**
 * @brief Helper: Convert a single HEX character to a nibble value.
 */
static inline int modbus_hex_to_nibble(char c, uint8_t *nibble) {
  if (c >= '0' && c <= '9') {
    *nibble = c - '0';
    return 0;
  }
  if (c >= 'A' && c <= 'F') {
    *nibble = c - 'A' + 10;
    return 0;
  }
  if (c >= 'a' && c <= 'f') {
    *nibble = c - 'a' + 10;
    return 0;
  }
  return -EINVAL;
}

/**
 * @brief Helper: Convert two HEX characters to a single byte.
 */
static inline int modbus_hex_to_byte(const char *in, uint8_t *out) {
  uint8_t high, low;

  if (modbus_hex_to_nibble(in[0], &high) < 0)
    return -EINVAL;

  if (modbus_hex_to_nibble(in[1], &low) < 0)
    return -EINVAL;

  *out = (high << 4) | low;
  return 0;
}

uint8_t modbus_calc_lrc(const uint8_t *data, size_t len) {
  uint8_t lrc = 0;
  size_t i;

  if (!data || !len)
    return 0;

  /*
   * LRC is the two's complement of the sum of bytes (modulo 256).
   */
  for (i = 0; i < len; ++i) {
    lrc += data[i];
  }

  return (uint8_t)(-((int8_t)lrc));
}

int modbus_ascii_encode(const struct modbus_frame *frame, char *out_buf,
                        size_t out_size, size_t *out_len) {
  size_t required_size;
  size_t offset = 0;
  size_t i;
  uint8_t lrc_data[MODBUS_MAX_ADU_SIZE + 2]; /* Address (1) + Func (1) + Data */
  size_t lrc_len = 0;

  if (!frame || !out_buf || !out_len)
    return -EINVAL;

  if (frame->pdu.data_len > MODBUS_MAX_ADU_SIZE)
    return -EMSGSIZE;

  /* * Required memory:
   * 1 (':') + 2 (Addr) + 2 (Func) + (2 * data_len) + 2 (LRC) + 2 (CR LF)
   * = 9 + (2 * data_len)
   */
  required_size = 9 + (frame->pdu.data_len * 2);
  if (out_size < required_size)
    return -ENOBUFS;

  /* Prepare buffer for LRC calculation */
  lrc_data[lrc_len++] = frame->server_address;
  lrc_data[lrc_len++] = frame->pdu.function_code;
  for (i = 0; i < frame->pdu.data_len; ++i) {
    lrc_data[lrc_len++] = frame->pdu.data[i];
  }

  /* Start character */
  out_buf[offset++] = MODBUS_ASCII_START_CHAR;

  /* Server address */
  modbus_byte_to_hex(frame->server_address, &out_buf[offset]);
  offset += 2;

  /* Function code */
  modbus_byte_to_hex(frame->pdu.function_code, &out_buf[offset]);
  offset += 2;

  /* Payload data */
  for (i = 0; i < frame->pdu.data_len; ++i) {
    modbus_byte_to_hex(frame->pdu.data[i], &out_buf[offset]);
    offset += 2;
  }

  /* Calculate and append LRC */
  modbus_byte_to_hex(modbus_calc_lrc(lrc_data, lrc_len), &out_buf[offset]);
  offset += 2;

  /* Frame terminator (CR LF) */
  out_buf[offset++] = MODBUS_ASCII_CR;
  out_buf[offset++] = MODBUS_ASCII_LF;

  *out_len = offset;
  return 0;
}

int modbus_ascii_decode(const char *in_buf, size_t in_len,
                        struct modbus_frame *frame) {
  size_t offset = 1; /* skip the start character ':' */
  size_t pdu_hex_len;
  size_t pdu_bin_len;
  size_t i;
  uint8_t calculated_lrc;
  uint8_t received_lrc;
  uint8_t lrc_data[MODBUS_MAX_ADU_SIZE + 2];
  size_t lrc_len = 0;

  if (!in_buf || !frame)
    return -EINVAL;

  memset(frame, 0, sizeof(*frame));

  /* Minimum length: ':', Addr(2), Func(2), LRC(2), CR, LF -> 9 characters */
  if (in_len < 9)
    return -EMSGSIZE;

  if (in_buf[0] != MODBUS_ASCII_START_CHAR)
    return -EPROTO;

  if (in_buf[in_len - 2] != MODBUS_ASCII_CR ||
      in_buf[in_len - 1] != MODBUS_ASCII_LF)
    return -EPROTO;

  /* Length of the HEX data payload (excluding ':', LRC, and CR LF) */
  pdu_hex_len = in_len - 1 - 2 - 2;

  if (pdu_hex_len % 2 != 0)
    return -EPROTO; /* invalid HEX character parity */

  pdu_bin_len = pdu_hex_len / 2;
  if (pdu_bin_len < 2) /* must contain at least address and function code */
    return -EPROTO;

  /* Server address */
  if (modbus_hex_to_byte(&in_buf[offset], &frame->server_address) < 0)
    return -EILSEQ;
  lrc_data[lrc_len++] = frame->server_address;
  offset += 2;

  /* Function code */
  if (modbus_hex_to_byte(&in_buf[offset], &frame->pdu.function_code) < 0)
    return -EILSEQ;
  lrc_data[lrc_len++] = frame->pdu.function_code;
  offset += 2;

  /* PDU payload */
  frame->pdu.data_len = pdu_bin_len - 2; /* Exclude address and function */
  if (frame->pdu.data_len > MODBUS_MAX_ADU_SIZE)
    return -EMSGSIZE;

  for (i = 0; i < frame->pdu.data_len; ++i) {
    if (modbus_hex_to_byte(&in_buf[offset], &frame->pdu.data[i]) < 0)
      return -EILSEQ;
    lrc_data[lrc_len++] = frame->pdu.data[i];
    offset += 2;
  }

  /* LRC */
  if (modbus_hex_to_byte(&in_buf[offset], &received_lrc) < 0)
    return -EILSEQ;

  /* validate LRC */
  calculated_lrc = modbus_calc_lrc(lrc_data, lrc_len);
  frame->checksum = received_lrc;

  if (calculated_lrc != received_lrc) {
    frame->is_valid = false;
    return -EBADMSG; /* checksum mismatch */
  }

  frame->is_valid = true;
  return 0;
}
