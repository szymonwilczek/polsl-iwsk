#ifndef SERIAL_HAL_H
#define SERIAL_HAL_H

/**
 * @file serial_hal.h
 * @brief Public HAL contract for RS-232 / RS-485 serial transports.
 *
 * @details
 * This interface isolates hardware details from business logic.
 *
 * Upper layers (RS-232 message handling, MODBUS state machines, and future
 * presentation backends - ncurses and GUI) depend on this header only.
 * Backend-specific details (termios, WinAPI, emulator quirks) stay hidden
 * behind the operation table.
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef _WIN32
#include <basetsd.h>
typedef SSIZE_T ssize_t;
#else
#include <sys/types.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define SERIAL_HAL_DEVICE_NAME_MAX 256U

/**
 * @brief Parity mode for serial line.
 */
enum serial_hal_parity {
  SERIAL_HAL_PARITY_NONE = 0,
  SERIAL_HAL_PARITY_EVEN,
  SERIAL_HAL_PARITY_ODD,
};

/**
 * @brief Flow control mode.
 *
 * @details
 * DTR/DSR is exposed in API because the assignment allows automatic flow
 * control and also manual forcing of DTR/RTS states if needed.
 */
enum serial_hal_flow_control {
  SERIAL_HAL_FLOW_NONE = 0,
  SERIAL_HAL_FLOW_XON_XOFF,
  SERIAL_HAL_FLOW_DTR_DSR,
  SERIAL_HAL_FLOW_RTS_CTS,
};

/**
 * @brief Serial port runtime configuration.
 */
struct serial_hal_config {
  char device[SERIAL_HAL_DEVICE_NAME_MAX];
  uint32_t baud_rate;
  uint8_t data_bits;
  uint8_t stop_bits;
  enum serial_hal_parity parity;
  uint32_t read_timeout_ms;
  uint32_t write_timeout_ms;
};

/**
 * @brief Modem lines state.
 */
struct serial_hal_modem_lines {
  bool dtr;
  bool rts;
  bool cts;
  bool dsr;
  bool dcd;
  bool ri;
};

struct serial_hal_device;

/**
 * @brief Backend operation table.
 *
 * @details
 * Every platform backend exports the same callable set, so runtime policy
 * changes (automatic handshake vs manual DTR/RTS control) are handled by
 * backend implementation.
 */
struct serial_hal_ops {
  int (*open)(struct serial_hal_device *dev,
              const struct serial_hal_config *cfg);
  int (*close)(struct serial_hal_device *dev);
  ssize_t (*read)(struct serial_hal_device *dev, void *buf, size_t len,
                  uint32_t timeout_ms);
  ssize_t (*write)(struct serial_hal_device *dev, const void *buf, size_t len,
                   uint32_t timeout_ms);
  int (*set_config)(struct serial_hal_device *dev,
                    const struct serial_hal_config *cfg);
  int (*set_flow_control)(struct serial_hal_device *dev,
                          enum serial_hal_flow_control flow);
  int (*set_modem_lines)(struct serial_hal_device *dev,
                         const struct serial_hal_modem_lines *lines);
  int (*get_modem_lines)(struct serial_hal_device *dev,
                         struct serial_hal_modem_lines *lines);
  int (*flush)(struct serial_hal_device *dev);
  void (*destroy)(struct serial_hal_device *dev);
};

/**
 * @brief HAL object used by higher layers.
 */
struct serial_hal_device {
  const struct serial_hal_ops *ops;
  void *priv;
  struct serial_hal_config active_cfg;
  bool is_open;
};

/**
 * @brief Initialize Linux backend object.
 *
 * @param dev HAL object to initialize.
 * @return 0 on success, negative errno otherwise.
 */
int serial_hal_linux_init(struct serial_hal_device *dev);

/**
 * @brief Deinitialize Linux backend and release resources.
 *
 * @param dev HAL object previously initialized with serial_hal_linux_init().
 */
void serial_hal_linux_deinit(struct serial_hal_device *dev);

/**
 * @brief Initialize Windows backend object.
 *
 * @param dev HAL object to initialize.
 * @return 0 on success, negative errno otherwise.
 */
int serial_hal_windows_init(struct serial_hal_device *dev);

/**
 * @brief Deinitialize Windows backend and release resources.
 *
 * @param dev HAL object previously initialized with serial_hal_windows_init().
 */
void serial_hal_windows_deinit(struct serial_hal_device *dev);


static inline int serial_hal_open(struct serial_hal_device *dev,
                                  const struct serial_hal_config *cfg) {
  if (!dev || !dev->ops || !dev->ops->open)
    return -ENOSYS;

  return dev->ops->open(dev, cfg);
}

static inline int serial_hal_close(struct serial_hal_device *dev) {
  if (!dev || !dev->ops || !dev->ops->close)
    return -ENOSYS;

  return dev->ops->close(dev);
}

static inline ssize_t serial_hal_read(struct serial_hal_device *dev, void *buf,
                                      size_t len, uint32_t timeout_ms) {
  if (!dev || !dev->ops || !dev->ops->read)
    return -ENOSYS;

  return dev->ops->read(dev, buf, len, timeout_ms);
}

static inline ssize_t serial_hal_write(struct serial_hal_device *dev,
                                       const void *buf, size_t len,
                                       uint32_t timeout_ms) {
  if (!dev || !dev->ops || !dev->ops->write)
    return -ENOSYS;

  return dev->ops->write(dev, buf, len, timeout_ms);
}

static inline int serial_hal_set_config(struct serial_hal_device *dev,
                                        const struct serial_hal_config *cfg) {
  if (!dev || !dev->ops || !dev->ops->set_config)
    return -ENOSYS;

  return dev->ops->set_config(dev, cfg);
}

static inline int
serial_hal_set_flow_control(struct serial_hal_device *dev,
                            enum serial_hal_flow_control flow) {
  if (!dev || !dev->ops || !dev->ops->set_flow_control)
    return -ENOSYS;

  return dev->ops->set_flow_control(dev, flow);
}

static inline int
serial_hal_set_modem_lines(struct serial_hal_device *dev,
                           const struct serial_hal_modem_lines *lines) {
  if (!dev || !dev->ops || !dev->ops->set_modem_lines)
    return -ENOSYS;

  return dev->ops->set_modem_lines(dev, lines);
}

static inline int
serial_hal_get_modem_lines(struct serial_hal_device *dev,
                           struct serial_hal_modem_lines *lines) {
  if (!dev || !dev->ops || !dev->ops->get_modem_lines)
    return -ENOSYS;

  return dev->ops->get_modem_lines(dev, lines);
}

static inline int serial_hal_flush(struct serial_hal_device *dev) {
  if (!dev || !dev->ops || !dev->ops->flush)
    return -ENOSYS;

  return dev->ops->flush(dev);
}

static inline void serial_hal_destroy(struct serial_hal_device *dev) {
  if (!dev || !dev->ops || !dev->ops->destroy)
    return;

  dev->ops->destroy(dev);
}

#ifdef __cplusplus
}
#endif

#endif
