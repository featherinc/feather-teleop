# Teensy Ethernet → CAN Bridge

Firmware for a **Teensy 4.1** board that bridges the Jetson and robot CAN buses.

The Jetson talks to the Teensy over Ethernet (TCP port `6613`). The Teensy always takes a DHCP address and
advertises `feather-teensy.local`. It waits for a lease indefinitely and has no static address of its own.
`poll_ethernet()` keeps the DHCP client running and returns immediately until a lease arrives, so the CAN loop
keeps running during the wait. On a direct cable the Jetson is the DHCP server. On a router, both ends take a
lease from that router. The Jetson finds the board as `feather-teensy.local`. The Teensy translates those commands
into CAN frames for motors/devices, and sends telemetry and responses back over Ethernet.

Why this instead of USB CAN adapters:
- USB CAN has been unreliable for us
- The Teensy can do low-level message handling and buffering off the Jetson

## Layout

| Path | Role |
|------|------|
| `src/` | Firmware (C). Main loop in `teensy.h`; Arduino entry is `teensy.ino` |
| `src/comms/` | Ethernet server + CAN I/O (`QNEthernet`, `FlexCAN_T4`) |
| `src/commands/` | Opcode handlers: utilities, Robstride, EZMotion, SOC |
| `src/actuator.*` | Actuator state and telemetry ring buffers |
| `qdefs/` | Generated protocol headers (from Quill defs via `make qdefs`) |
| `emulator/` | Host-side Ethernet (TCP) + SocketCAN backends |
| `tests/` | Host unit tests |
| `teensy_flash.py` | Ethernet OTA flash |
| `recompilation.py` | Rebuild when sources change (hash-tracked) |

## Runtime

1. Init CAN, then Ethernet
2. Loop: `poll_ethernet()` then `poll_can()`
3. Ethernet frames carry opcode + payload; handlers in `all_commands.h` dispatch to motor/device drivers or board 
   utilities (echo, init, flash, reboot, telemetry, etc.)

Same sources build for hardware (`COMPILE_FOR_TEENSY`) or the host emulator (TCP + `vcan*`).

## Build & flash

One-time: `make install-cli`. Ethernet is `feathercli setup-ethernet` (also part of `feathercli install --all`):
a DHCP server already on the port leaves the Jetson as a client, and no answer makes the Jetson the DHCP server.
That choice is stored in NetworkManager and applied again after reboot. `libnss-mdns` lets the Jetson resolve
`feather-teensy.local`. Short the board gap for the
terminating resistor.

| Target | What it does |
|--------|----------------|
| `make compile` | Quill qdefs → stage sketch → build `.hex` into `bin/` |
| `make flash` | OTA flash over Ethernet |
| `make flash-usb` | USB flash via `teensy_loader_cli` |
| `make build` | compile + OTA flash, regenerating quill qdefs |
| `make build-noquill` | compile + OTA flash using the qdefs already in `qdefs/`. This is what the SDK runs when it auto-rebuilds |
| `make build-usb` | compile + USB flash |
| `make emulated` | Host binary (`bin/emulated_teensy`) |
| `make test` | Build and run unit tests |
| `make monitor` | Serial monitor (picocom) |

## Useful Tips

- on the teensy, the can bus's are index 0 closest to ethernet port, 1 in the middle, and 2 furthest away from 
  ethernet port
- to make that c99 warning go away, go to your vscode settings, search for "clangd fallback flags", and add the argument
  `-Wno-c99-designator` to it (no ticks, just the plain arg)
