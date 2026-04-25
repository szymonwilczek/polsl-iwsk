# RS-232 and MODBUS ASCII Communication Project

Szymon Wilczek

## 1. Project Overview
This project implements a serial communication system between two DTE (Data Terminal Equipment) stations. It focuses on the **RS-232** standard and the **MODBUS ASCII** protocol, providing a Hardware Abstraction Layer (HAL) for Linux, a custom protocol engine, and a multi-threaded Text User Interface (TUI).

The project was developed as part of the "Interfejsy w systemach komputerowych" (IWSK) course at the Silesian University of Technology (PolSL).

## 2. Architecture
The software is divided into three distinct layers to ensure modularity and ease of hardware migration:

* **HAL (Hardware Abstraction Layer):** Isolates OS-specific serial port handling (`termios` for Linux, `WinAPI` for Windows). It provides a unified interface for I/O operations and modem line control.
* **Protocol Engine (MODBUS/RS-232):** A hardware-agnostic layer that handles data framing, checksum (LRC) calculation, and ASCII encoding/decoding.
* **Presentation Layer (TUI):** A multi-threaded CLI application. The background thread handles asynchronous RX polling, while the main thread manages user input and UI rendering via ANSI escape codes.

## 3. Hardware Setup & Parameters
The system operates at an industry-standard **9600 bps (8N1)**. This baud rate was chosen as it provides excellent noise immunity and is the default for legacy industrial Modbus equipment. The frame terminator strictly follows the Modbus ASCII specification: **CR LF** (`\r\n`).

### Physical Connection
The system was successfully tested using a cross-platform hardware setup:
* 2x **UGREEN CR104 (PL2303)** USB-to-RS232 (DB9) adapters.
* 1x **Assmann Null Modem Cable** (Female-to-Female) with full crossover for hardware flow control.

## 4. Verification & Test Scenarios (Physical Hardware Logs)

The following tests were performed using a physical connection between a Linux workstation (Station A) and a Windows 11 laptop (Station B).
Also tested as Linux (Station A) and another Linux (Station B). Never tested Windows - Windows, since I don't have that OS on my main machine.

### Scenario 1: Loopback & Protocol Integrity (Raw Text & MODBUS)
**Goal:** Verify that the asynchronous buffer correctly concatenates physical UART byte streams and the Modbus engine calculates the LRC checksum correctly.

**Test Log (Linux to Windows):**

```text
> 2
[TX] Modbus frame sent: :010148496D

[RX EVENT] Received 13 bytes.
[MODBUS VALID] Node: 01 | Func: 01 | Data Len: 2
```

*Note:* The RX buffer uses `strchr()` to find the `\n` terminator, solving hardware fragmentation where bytes arrive sequentially (1 ms per byte at 9600 bps) rather than instantaneously as in a software emulator.

### Scenario 2: Error Handling (Corrupted Frames)

**Goal:** Ensure the system rejects invalid protocol data based on LRC checksum mismatch.
**Test Execution:** A raw text string mimicking a Modbus frame with an intentional LRC error (`:01014849FF\r\n`) was sent.
**Result:** The receiver correctly labeled it as `[RAW TEXT]` instead of `[MODBUS VALID]`, proving the `-EBADMSG` protocol engine rejection works properly.

### Scenario 3: Hardware Flow Control (Handshake) & OS Quirks

**Goal:** Verify physical control of modem lines using HAL (`TIOCMSET` on Linux / `EscapeCommFunction` on WinAPI).

#### Hardware Test 1: RTS -> CTS Flow

**Action:** Toggled RTS on Linux, read state on Windows.

```text
Linux:
> 4
[MODEM] RTS toggled to 0

Windows:
> 5
[STATUS] DTR:0 RTS:0 | DSR:0 CTS:0 DCD:0 RI:0
```

**Result:** Setting RTS to 0 on Linux successfully pulled the physical CTS line down to 0 on Windows, proving true hardware flow control.

#### Hardware Test 2: DTR -> DCD Cross-Wiring (Cable Specifics)

**Action:** Toggled DTR on Linux, read state on Windows.

```text
Linux:
> 3
[MODEM] DTR toggled to 0

Windows:
> 5
[STATUS] DTR:0 RTS:0 | DSR:0 CTS:1 DCD:0 RI:0
```

*Notes (OS & Hardware Quirks):*

1. **Cable Crossover:** Changing DTR on the sender changed DCD (Data Carrier Detect) on the receiver, not DSR. This is a common feature in some Null-Modem cables, simulating carrier detection.
2. **WinAPI vs termios:** In the Windows log, DTR and RTS read as `0` locally. This is because Windows' `GetCommModemStatus` only reports *input pins* from the device, whereas Linux's `ioctl(TIOCMGET)` returns the state of **both** input and output registers.

## 5. Build Instructions

The project uses CMake for cross-platform builds.

```bash
mkdir build && cd build
cmake ..
make
./polsl_iwsk_app_linux /dev/ttyUSB0
```

*Note:* The CMake configuration includes a custom target to automatically cross-compile the Windows `.exe` binary using MinGW.

## References
- Mielczarek W.: *Szeregowe interfejsy cyfrowe, Helion 1993*
- Mielczarek W.: *Interfejsy w systemach komputerowych cz.1 - Komunikacja przez port znakowy*
