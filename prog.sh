#! /bin/sh
DFU_UTIL=../chibios-stm/dfu-util/src/dfu-util
$DFU_UTIL -d 0483:df11 -a 0 -s 0x08000000:leave -D build/ch.bin
