# ltm2985_uart

LTM2985 UART driver — primary **control channel** for core PI and program steps in **`default`** experiment mode.

**Full documentation:** [docs/en/hardware.md](../../docs/en/hardware.md) · [docs/uk/hardware.md](../../docs/uk/hardware.md)

Service: `delatometry-ltm2985.service` · Simulator: `hardware/ltm_nodemcu_simulator/README.md`

If FTDI `urb stopped: -32` leaves the USB-serial port open with no RX, the node force-reopens after `stale_timeout_sec` (default 15s). After `max_stale_recoveries` (default 3) failed recoveries it exits so systemd (`Restart=always`) restarts the process.

**Документація українською:** [docs/uk/hardware.md](../../docs/uk/hardware.md)
