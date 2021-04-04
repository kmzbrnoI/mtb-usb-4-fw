# MTB-USB v4 Firmware

## Toolchain

Flash:

```bash
$ # LD_LIBRARY_PATH=/lib/x86_64-linux-gnu/
$ st-flash --reset write build/mtb-usb-4.bin 0x08000000
```

Debug:

```bash
$ openocd
$ arm-none-eabi-gdb build/mtb-usb-4.elf
(gdb) target extended-remote :3333
(gdb) b main
```

Read serial in linux:

```bash
$ stty -F /dev/ttyACM0 raw
$ xxd /dev/ttyACM0
```