# Use GNU Debugger on STM32 CHIP

## Prerequisite

- st-link is installed on the host
- Official `arm-none-eabi-gdb` from ARM is accessible on the host (in this example, it is located in `$UTCOUPE_WORKSPACE/libs/gcc-arm-none-eabi-6-2017-q2-update/bin/`). _Your host might have it in its packages, but it is likely buggy._
- STM32 chip with embedded ST-Link V2 connected with USB cable
- Binary file to upload on STM32 chip in `elf` format.

## Steps

1) Disconnect all STM32 chips except the one to debug on the host.
2) Launch st-link with the command `st-utils`. It should print logs as follow:
    ```
    $ st-util
    st-util 1.5.1
    2020-03-01T12:27:56 INFO common.c: Loading device parameters....
    2020-03-01T12:27:56 INFO common.c: Device connected is: F334 device, id 0x10016438
    2020-03-01T12:27:56 INFO common.c: SRAM size: 0x3000 bytes (12 KiB), Flash: 0x10000 bytes (64 KiB) in pages of 2048 bytes
    2020-03-01T12:27:56 INFO gdb-server.c: Chip ID is 00000438, Core ID is  2ba01477.
    2020-03-01T12:27:56 INFO gdb-server.c: Listening at *:4242...
    ```
    A GDB server is openned on the localhost port `4242`.
3) In another terminal, open `arm-none-eabi-gdb` with your binary:
    ```
    $ $UTCOUPE_WORKSPACE/libs/gcc-arm-none-eabi-6-2017-q2-update/bin/arm-none-eabi-gdb my_binary.elf
    GNU gdb (GNU Tools for ARM Embedded Processors 6-2017-q2-update) 7.12.1.20170417-git
    [...]
    Reading symbols from my_binary.elf...done.
    ```
4) Type `target extended-remote :4242` to connect to the gdb server :
    ```
    (gdb) tar ext :4242
    Remote debugging using :4242
    0x08001320 in __gnu_unwind_pr_common ()
    ```
    In the `st-util` logs, you should see :
    ```
    2020-03-01T12:40:07 INFO gdb-server.c: Found 6 hw breakpoint registers
    2020-03-01T12:40:07 INFO gdb-server.c: GDB connected
    ```
5) Type `load` to upload the binary to the chip. It will use implicitely `st-flash` with correct parameters.
    ```
    (gdb) load
    Loading section .isr_vector, size 0x188 lma 0x8000000
    Loading section .text, size 0x8488 lma 0x8000190
    Loading section .rodata, size 0x744 lma 0x8008618
    Loading section .ARM.extab, size 0x74 lma 0x8008d5c
    Loading section .ARM, size 0x130 lma 0x8008dd0
    Loading section .init_array, size 0xc lma 0x8008f00
    Loading section .fini_array, size 0x4 lma 0x8008f0c
    Loading section .data, size 0x1e8 lma 0x8008f10
    Start address 0x8004bac, load size 37104
    Transfer rate: 14 KB/sec, 3710 bytes/write.
    ```
    and in `st-util` logs :
    ```
    2020-03-01T12:42:11 INFO common.c: Attempting to write 2048 (0x800) bytes to stm32 address: 134217728 (0x8000000)
    Flash page at addr: 0x08000000 erased
    2020-03-01T12:42:11 INFO common.c: Finished erasing 1 pages of 2048 (0x800) bytes
    2020-03-01T12:42:11 INFO common.c: Starting Flash write for VL/F0/F3/F1_XL core id
    2020-03-01T12:42:11 INFO flash_loader.c: Successfully loaded flash loader in sram

    2020-03-01T12:42:12 INFO common.c: Starting verification of write complete
    2020-03-01T12:42:12 INFO common.c: Flash written and verified! jolly good!
    [...]
    2020-03-01T12:42:14 INFO common.c: Flash written and verified! jolly good!
    ```
6) You can now use gdb as usual, but with a maximum number of breakpoints indicated in step 4).