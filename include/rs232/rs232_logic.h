#ifndef RS232_LOGIC_H
#define RS232_LOGIC_H

/**
 * @file rs232_logic.h
 * @brief RS-232 transport utilities independent from hardware backend.
 *
 * @details
 * This module keeps data framing and diagnostics away from HAL internals.
 * It can be reused for terminal mode, GUI/TUI integrations, and test
 * tooling without exposing termios/WinAPI details to upper layers.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hal/serial_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RS232_TERMINATOR_CUSTOM_MAX 16U

/**
 * @brief Supported message terminator modes.
 */
enum rs232_terminator_mode {
  RS232_TERMINATOR_NONE = 0,
  RS232_TERMINATOR_CR,
  RS232_TERMINATOR_LF,
  RS232_TERMINATOR_CRLF,
  RS232_TERMINATOR_CUSTOM,
};

/**
 * @brief Message terminator configuration.
 */
struct rs232_terminator_config {
  enum rs232_terminator_mode mode;
  uint8_t custom[RS232_TERMINATOR_CUSTOM_MAX];
  size_t custom_len;
};

/**
 * @brief Input parameters for ping transaction.
 */
struct rs232_ping_request {
  const uint8_t *tx_data;
  size_t tx_len;
  const uint8_t *rx_prefix;
  size_t rx_prefix_len;
  uint32_t tx_timeout_ms;
  uint32_t rx_timeout_ms;
};

/**
 * @brief Ping transaction result.
 */
struct rs232_ping_result {
  uint64_t round_trip_us;
  ssize_t rx_len;
};

/**
 * @brief Validate terminator configuration.
 *
 * @param cfg Terminator configuration.
 * @return 0 on success, negative errno on invalid setup.
 */
int rs232_validate_terminator(const struct rs232_terminator_config *cfg);

/**
 * @brief Append configured terminator to payload.
 *
 * @param payload Input payload (may be NULL when payload_len is 0).
 * @param payload_len Input payload length in bytes.
 * @param cfg Terminator configuration.
 * @param out Output buffer receiving payload + terminator.
 * @param out_size Output buffer size in bytes.
 * @param out_len Output resulting frame length in bytes.
 * @return 0 on success, negative errno on failure.
 */
int rs232_apply_terminator(const uint8_t *payload, size_t payload_len,
                           const struct rs232_terminator_config *cfg,
                           uint8_t *out, size_t out_size, size_t *out_len);

/**
 * @brief Remove configured terminator from message tail.
 *
 * @param data Input/output data buffer.
 * @param data_len Input/output message length in bytes.
 * @param cfg Terminator configuration.
 * @return 0 on success, negative errno on failure.
 */
int rs232_strip_terminator(uint8_t *data, size_t *data_len,
                           const struct rs232_terminator_config *cfg);

/**
 * @brief Convert raw bytes to ASCII hex string.
 *
 * @param input Input data (may be NULL when input_len is 0).
 * @param input_len Input length in bytes.
 * @param upper_case Use uppercase hex digits when true.
 * @param spaced Insert spaces between bytes when true.
 * @param output Output buffer for NUL-terminated text.
 * @param output_size Output buffer size in bytes.
 * @param output_len Output text length without trailing NUL.
 * @return 0 on success, negative errno on failure.
 */
int rs232_bytes_to_hex(const uint8_t *input, size_t input_len, bool upper_case,
                       bool spaced, char *output, size_t output_size,
                       size_t *output_len);

/**
 * @brief Convert ASCII hex text to raw bytes.
 *
 * @details
 * Spaces, tabs, CR/LF, '-' and ':' separators are ignored.
 *
 * @param hex_text Input text (may be NULL when hex_len is 0).
 * @param hex_len Input text length in bytes.
 * @param output Output buffer for decoded bytes.
 * @param output_size Output buffer size in bytes.
 * @param output_len Decoded byte length.
 * @return 0 on success, negative errno on failure.
 */
int rs232_hex_to_bytes(const char *hex_text, size_t hex_len, uint8_t *output,
                       size_t output_size, size_t *output_len);

/**
 * @brief Execute link ping and measure round-trip delay.
 *
 * @param dev Opened HAL device.
 * @param req Ping request parameters.
 * @param rx_buf Buffer for response bytes.
 * @param rx_buf_size Response buffer size.
 * @param result Ping result with timing and response length.
 * @return 0 on success, negative errno on failure.
 */
int rs232_ping(struct serial_hal_device *dev,
               const struct rs232_ping_request *req, uint8_t *rx_buf,
               size_t rx_buf_size, struct rs232_ping_result *result);

#ifdef __cplusplus
}
#endif

#endif
