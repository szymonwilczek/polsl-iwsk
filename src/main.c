#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
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
#endif

#define RX_BUFFER_SIZE 512

struct app_context {
  struct serial_hal_device dev;
  atomic_bool is_running;
  pthread_t rx_thread;
  struct serial_hal_modem_lines current_lines;
};

/**
 * @brief Thread routine for continuously reading the serial port.
 */
static void *rx_worker_thread(void *arg) {
  struct app_context *ctx = (struct app_context *)arg;
  uint8_t rx_buf[RX_BUFFER_SIZE];
  struct modbus_frame frame;
  ssize_t bytes_read;
  int ret;

  while (atomic_load(&ctx->is_running)) {
    /* non-blocking wait using HAL timeout */
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
  printf("\n" ANSI_COLOR_CYAN "=== IWSK: RS-232 & MODBUS ===" ANSI_COLOR_RESET
         "\n");
  printf("1. Send raw text message\n");
  printf("2. Send test MODBUS ASCII frame (Node 1, Func 1)\n");
  printf("3. Toggle DTR (Data Terminal Ready) pin\n");
  printf("4. Toggle RTS (Request To Send) pin\n");
  printf("5. Read Modem Lines Status (DSR, CTS, etc.)\n");
  printf("q. Quit application\n");
  printf("> ");
  fflush(stdout);
}

int main(int argc, char **argv) {
  struct app_context ctx;
  struct serial_hal_config cfg;
  char input[256];
  int ret;
  int exit_code = EXIT_SUCCESS;
  bool rx_thread_started = false;

  if (argc < 2) {
    fprintf(stderr,
            "Usage: %s <serial_port> (e.g. /tmp/ttyV0 or /dev/ttyUSB0)\n",
            argv[0]);
    return EXIT_FAILURE; /* its fine to return, nothing allocated yet */
  }

  memset(&ctx, 0, sizeof(ctx));
  atomic_init(&ctx.is_running, true);

#ifdef _WIN32
  enable_windows_ansi();
#endif

  /* initialize HAL depending on OS */
#ifdef _WIN32
  if (serial_hal_windows_init(&ctx.dev) < 0) {
    fprintf(stderr, "Failed to init Windows HAL.\n");
    return EXIT_FAILURE; /* also fine, HAL init failed, nothing to clean up */
  }
#else
  if (serial_hal_linux_init(&ctx.dev) < 0) {
    fprintf(stderr, "Failed to init Linux HAL.\n");
    return EXIT_FAILURE;
  }
#endif

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
    exit_code = EXIT_FAILURE;
    goto cleanup;
  }

  /* initial state for DTR/RTS */
  ctx.current_lines.dtr = true;
  ctx.current_lines.rts = true;
  serial_hal_set_modem_lines(&ctx.dev, &ctx.current_lines);

  /* background RX thread */
  if (pthread_create(&ctx.rx_thread, NULL, rx_worker_thread, &ctx) != 0) {
    fprintf(stderr, "Failed to spawn RX thread.\n");
    exit_code = EXIT_FAILURE;
    goto cleanup;
  }
  rx_thread_started = true;

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
    } else if (input[0] == '3') {
      ctx.current_lines.dtr = !ctx.current_lines.dtr;
      serial_hal_set_modem_lines(&ctx.dev, &ctx.current_lines);
      printf(ANSI_COLOR_YELLOW "[MODEM]" ANSI_COLOR_RESET
                               " DTR toggled to %d\n",
             ctx.current_lines.dtr);
    } else if (input[0] == '4') {
      ctx.current_lines.rts = !ctx.current_lines.rts;
      serial_hal_set_modem_lines(&ctx.dev, &ctx.current_lines);
      printf(ANSI_COLOR_YELLOW "[MODEM]" ANSI_COLOR_RESET
                               " RTS toggled to %d\n",
             ctx.current_lines.rts);
    } else if (input[0] == '5') {
      struct serial_hal_modem_lines status;
      if (serial_hal_get_modem_lines(&ctx.dev, &status) == 0) {
        printf(ANSI_COLOR_YELLOW
               "[STATUS]" ANSI_COLOR_RESET
               " DTR:%d RTS:%d | DSR:%d CTS:%d DCD:%d RI:%d\n",
               status.dtr, status.rts, status.dsr, status.cts, status.dcd,
               status.ri);
      } else {
        printf(ANSI_COLOR_RED "Failed to read modem lines. Emulator might not "
                              "support it.\n" ANSI_COLOR_RESET);
      }
    }
  }

  printf("Shutting down gracefully...\n");

cleanup:
  /* stop and join the thread ONLY if it was successfully started */
  if (rx_thread_started) {
    atomic_store(&ctx.is_running, false);
    pthread_join(ctx.rx_thread, NULL);
  }

  /* close the port ONLY if its currently open */
  if (ctx.dev.is_open) {
    serial_hal_close(&ctx.dev);
  }

  /* deinitialize the correct HAL */
#ifdef _WIN32
  serial_hal_windows_deinit(&ctx.dev);
#else
  serial_hal_linux_deinit(&ctx.dev);
#endif

  return exit_code;
}
