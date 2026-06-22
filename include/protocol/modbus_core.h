#ifndef MODBUS_CORE_H
#define MODBUS_CORE_H

/**
 * @file modbus_core.h
 * @brief Hardware-agnostic MODBUS protocol engine (ASCII/RTU).
 *
 * @details
 * This module isolates the Modbus frame construction, parsing, and validation
 * from the physical transport layer. It operates purely on memory buffers.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MODBUS_MAX_ADU_SIZE 256U
#define MODBUS_ASCII_START_CHAR ':'
#define MODBUS_ASCII_CR '\r'
#define MODBUS_ASCII_LF '\n'

/**
 * @brief Supported MODBUS operational modes.
 */
enum modbus_mode {
  MODBUS_MODE_ASCII = 0,
  MODBUS_MODE_RTU, /* Optional functionality */
};

/**
 * @brief Station role in the network.
 */
enum modbus_role {
  MODBUS_ROLE_MASTER = 0,
  MODBUS_ROLE_SLAVE,
};

/**
 * @brief Standardized Modbus Protocol Data Unit (PDU).
 */
struct modbus_pdu {
  uint8_t function_code;
  uint8_t data[MODBUS_MAX_ADU_SIZE];
  size_t data_len;
};

/**
 * @brief Standardized Modbus Application Data Unit (ADU).
 */
struct modbus_frame {
  uint8_t server_address;
  struct modbus_pdu pdu;
  uint8_t checksum; /* LRC for ASCII, CRC for RTU */
  bool is_valid;
};

/**
 * @brief Calculate Longitudinal Redundancy Check (LRC) for ASCII mode.
 *
 * @param data Buffer containing raw binary bytes (before hex encoding).
 * @param len Number of bytes to process.
 * @return Computed 8-bit LRC value.
 */
uint8_t modbus_calc_lrc(const uint8_t *data, size_t len);

/**
 * @brief Calculate the MODBUS RTU CRC-16 (polynomial 0xA001).
 *
 * @param data Buffer containing raw binary bytes (address + PDU).
 * @param len Number of bytes to process.
 * @return Computed 16-bit CRC value (to be transmitted low byte first).
 */
uint16_t modbus_calc_crc(const uint8_t *data, size_t len);

/**
 * @brief Encode a Modbus frame into an ASCII wire-format buffer.
 *
 * @details
 * Handles the start char (':'), hex-encoding of address/function/data,
 * LRC calculation and appending, and the CR/LF terminator.
 *
 * @param frame Populated frame to encode.
 * @param out_buf Buffer for the resulting ASCII string.
 * @param out_size Size of the output buffer.
 * @param out_len Pointer to store the final encoded length.
 * @return 0 on success, negative errno on failure.
 */
int modbus_ascii_encode(const struct modbus_frame *frame, char *out_buf,
                        size_t out_size, size_t *out_len);

/**
 * @brief Decode an ASCII wire-format buffer into a Modbus frame.
 *
 * @details
 * Validates start char, strips CR/LF, decodes hex to binary,
 * and validates the embedded LRC against the calculated LRC.
 *
 * @param in_buf Raw ASCII string received from transport.
 * @param in_len Length of the input string.
 * @param frame Pointer to store the decoded frame.
 * @return 0 on success (is_valid = true), negative errno on parsing error.
 */
int modbus_ascii_decode(const char *in_buf, size_t in_len,
                        struct modbus_frame *frame);

/**
 * @brief Encode a Modbus frame into an RTU wire-format buffer.
 *
 * @details
 * Layout: address, function, data, CRC-16 (low byte first). No framing chars;
 * RTU frames are delimited on the wire by a silent interval (>= 3.5 chars).
 *
 * @param frame Populated frame to encode.
 * @param out_buf Buffer for the resulting binary bytes.
 * @param out_size Size of the output buffer.
 * @param out_len Pointer to store the final encoded length.
 * @return 0 on success, negative errno on failure.
 */
int modbus_rtu_encode(const struct modbus_frame *frame, uint8_t *out_buf,
                      size_t out_size, size_t *out_len);

/**
 * @brief Decode an RTU wire-format buffer into a Modbus frame.
 *
 * @param in_buf Raw binary bytes received from transport.
 * @param in_len Length of the input buffer.
 * @param frame Pointer to store the decoded frame (is_valid set on good CRC).
 * @return 0 on success, negative errno on parsing/CRC error.
 */
int modbus_rtu_decode(const uint8_t *in_buf, size_t in_len,
                      struct modbus_frame *frame);

#ifdef __cplusplus
}
#endif

#endif
