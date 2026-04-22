#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "hal/serial_hal.h"
#include "protocol/modbus_core.h"

#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_CYAN "\x1b[36m"
#define ANSI_COLOR_RESET "\x1b[0m"

#define RX_BUFFER_SIZE 512

struct app_context {
  struct serial_hal_device dev;
  atomic_bool is_running;
  pthread_t rx_thread;
};

/**
 * @brief Thread routine for continuously reading the serial port.
 */
static void *rx_worker_thread(void *arg) {
  struct app_context *ctx = (struct app_context *)arg;
  uint8_t rx_buf[RX_BUFFER_SIZE];
  char ascii_buf[RX_BUFFER_SIZE];
  struct modbus_frame frame;
  ssize_t bytes_read;
  int ret;

  while (atomic_load(&ctx->is_running)) {
    bytes_read = serial_hal_read(&ctx->dev, rx_buf, sizeof(rx_buf) - 1, 100);

    if (bytes_read > 0) {
      rx_buf[bytes_read] = '\0'; /* null-terminate for safe string ops */

      /* attempt to decode as MODBUS ASCII */
      ret =
          modbus_ascii_decode((const char *)rx_buf, (size_t)bytes_read, &frame);

      printf("\n" ANSI_COLOR_CYAN "[RX EVENT]" ANSI_COLOR_RESET
             " Received %zd bytes.\n",
             bytes_read);

      if (ret == 0 && frame.is_valid) {
        printf(ANSI_COLOR_GREEN "[MODBUS VALID]" ANSI_COLOR_RESET
                                " Node: %02X | Func: %02X | Data Len: %zu\n",
               frame.server_address, frame.pdu.function_code,
               frame.pdu.data_len);
      } else {
        /* fallback to raw text printing */
        printf(ANSI_COLOR_YELLOW "[RAW TEXT]" ANSI_COLOR_RESET " %s\n", rx_buf);
      }

      /* reprint prompt */
      printf("> ");
      fflush(stdout);
    }
  }

  return NULL;
}

/**
 * @brief Send a sample MODBUS ASCII frame (Function 0x01).
 */
static void send_modbus_test_frame(struct app_context *ctx) {
  struct modbus_frame tx_frame;
  char wire_buf[MODBUS_MAX_ADU_SIZE * 2];
  size_t wire_len = 0;

  memset(&tx_frame, 0, sizeof(tx_frame));
  tx_frame.server_address = 0x01;
  tx_frame.pdu.function_code = 0x01;
  tx_frame.pdu.data[0] = 'H';
  tx_frame.pdu.data[1] = 'I';
  tx_frame.pdu.data_len = 2;

  if (modbus_ascii_encode(&tx_frame, wire_buf, sizeof(wire_buf), &wire_len) ==
      0) {
    serial_hal_write(&ctx->dev, wire_buf, wire_len, 500);
    printf(ANSI_COLOR_GREEN "[TX]" ANSI_COLOR_RESET " Modbus frame sent: %.*s",
           (int)wire_len, wire_buf); /* CRLF is already in wire_buf */
  } else {
    printf(ANSI_COLOR_RED "Failed to encode MODBUS frame.\n" ANSI_COLOR_RESET);
  }
}

/**
 * @brief Display the main TUI menu.
 */
static void print_menu(void) {
  printf("\n" ANSI_COLOR_CYAN "=== IWSK: RS-232 PROJECT ===" ANSI_COLOR_RESET
         "\n");
  printf("1. Send raw text message\n");
  printf("2. Send test MODBUS ASCII frame (Node 1, Func 1)\n");
  printf("q. Quit application\n");
  printf("> ");
  fflush(stdout);
}

int main(int argc, char **argv) {
  struct app_context ctx;
  struct serial_hal_config cfg;
  char input[256];
  int ret;

  if (argc < 2) {
    fprintf(stderr,
            "Usage: %s <serial_port> (e.g. /tmp/ttyV0 or /dev/ttyUSB0)\n",
            argv[0]);
    return EXIT_FAILURE;
  }

  memset(&ctx, 0, sizeof(ctx));
  atomic_init(&ctx.is_running, true);

  /* HAL for Linux */
  if (serial_hal_linux_init(&ctx.dev) < 0) {
    fprintf(stderr, "Failed to init Linux HAL.\n");
    return EXIT_FAILURE;
  }

  /* configure port parameters (9600 8N1) */
  memset(&cfg, 0, sizeof(cfg));
  strncpy(cfg.device, argv[1], sizeof(cfg.device) - 1);
  cfg.baud_rate = 9600;
  cfg.data_bits = 8;
  cfg.stop_bits = 1;
  cfg.parity = SERIAL_HAL_PARITY_NONE;

  if ((ret = serial_hal_open(&ctx.dev, &cfg)) < 0) {
    fprintf(stderr,
            "Failed to open %s (Error: %d). Check permissions or socat.\n",
            argv[1], ret);
    serial_hal_linux_deinit(&ctx.dev);
    return EXIT_FAILURE;
  }

  /* background RX thread */
  if (pthread_create(&ctx.rx_thread, NULL, rx_worker_thread, &ctx) != 0) {
    fprintf(stderr, "Failed to spawn RX thread.\n");
    serial_hal_close(&ctx.dev);
    serial_hal_linux_deinit(&ctx.dev);
    return EXIT_FAILURE;
  }

  /* main event loop */
  while (atomic_load(&ctx.is_running)) {
    print_menu();

    if (!fgets(input, sizeof(input), stdin))
      break;

    if (input[0] == 'q' || input[0] == 'Q') {
      atomic_store(&ctx.is_running, false);
      break;
    } else if (input[0] == '1') {
      printf("Enter text to send: ");
      fflush(stdout);
      if (fgets(input, sizeof(input), stdin)) {
        serial_hal_write(&ctx.dev, input, strlen(input), 500);
      }
    } else if (input[0] == '2') {
      send_modbus_test_frame(&ctx);
    }
  }

  printf("Shutting down gracefully...\n");
  pthread_join(ctx.rx_thread, NULL);
  serial_hal_close(&ctx.dev);
  serial_hal_linux_deinit(&ctx.dev);

  return EXIT_SUCCESS;
}
