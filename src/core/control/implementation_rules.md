# Implementation rules

This folder should follow the same coding style used in the STM32 motion MCU firmware.

Main points for this skeleton:

- Keep board-specific code in `src/board`.
- Keep Arduino and HardwareSerial use inside `src/platform` or board files.
- Keep control logic in `src/core/control` and feature modules.
- Use explicit control flow.
- Use fixed-width integer types.
- Use Allman braces.
