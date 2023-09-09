MTB-USB v4 Firmware
===================

This repository contains firmware for main MCU STM32F103 of MTB-USB v4 module.
MTB-USB module is a master module of MTBbus.

## Module functionality

 * Retransmit data between MTBbus and USB (PC).
 * Do MTBbus (RS485) time-critical operations.
 * Periodically poll MTBbus modules to let them report events.
 * Maintain list of active & inactive modules. Poll active modules frequently,
   poll rest of the modules less frequently to allow to find new modules.
 * Check if module responds to inquiry. When no response is get multiple times,
   report module as failed
 * Behaves as CDC in PC.

## LEDs behavior

 * Red led: bad checksum of mtbbus received.
 * Yellow led: USB send traffic.
   - Permanently on: computer does not read from USB CDC.
   - Permanently off: computer reads CDC, no data are sent.
   - Flashing: computer reads CDC, data are sent to computer.
 * Green led: MTBbus activity: polling MTBbus modules.
 * Blue led left: no meaning yet.
 * Blue led right: MTBbus powered.

## Build & requirements

 * Development:
   - `arm-none-eabi-gcc`
   - [bootloader](https://github.com/kmzbrnoI/mtb-usb-4-bootloader):
     build it and symlink `bootloader.bin` to built binary.
   - `make`
     ```bash
     $ make
     ```

 * Debugging:
   - `openocd`
   - `arm-none-eabi-gdb`
     ```bash
     $ openocd
     $ arm-none-eabi-gdb build/mtb-usb-4.elf
     (gdb) target extended-remote :3333
     (gdb) b main
     ```

 * Flashing:
   There are two independent options for firmware flashing to MTB-USB.

   1. `st-flash` (via STlink)
      MTB-USB module contains programming connector. Use STlink to program the MCU.
      ```bash
      $ make flash_with_bootloader
      ```
   2. Via DFU bootloader in MTB-USB (if bootloader already present in MTB-USB).
      ```bash
      $ ./reboot_to_dfu.sh
      $ make flash_dfu
      ```

## References

 * [MTB-USB v4 PCB](https://github.com/kmzbrnoI/mtb-usb-4-pcb)
 * [MTBbus protocol](https://github.com/kmzbrnoI/mtbbus-protocol)
 * [PC – MTB-USB module protocol](https://github.com/kmzbrnoI/mtbbus-protocol/tree/master/pc)
 * [MTB project (in czech only)](https://mtb.kmz-brno.cz/)

## License

This application is released under the [Apache License v2.0
](https://www.apache.org/licenses/LICENSE-2.0).
