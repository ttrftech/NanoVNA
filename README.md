NanoVNA - Low budget tiny handheld Vector Network Analyzer
==========================================================

# About

NanoVNA is very tiny handheld Vector Network Analyzer, works as
standalone, portable with battery, own lcd display. This project aim
to provide useful measuring tool for RF enthusiast.

This repository contains source of NanoVNA firmware.


## Prepare ARM Cross Tools

Install cross tools and dfu-util.

    $ brew tap PX4/px4
    $ brew install gcc-arm-none-eabi
    $ brew instal dfu-util

Otherwise, using toolchains inside LPCxpresso. Like this.

    $ PATH=$PATH:/Applications/lpcxpresso_7.8.0_426/lpcxpresso/tools/bin

## Building firmware

Fetch ChibiOS submodule into tree.

    $ cd nanovna
    $ git submodule update --init --recursive

Enter firmware directory and make it.

    $ make

## Burn it

Boot MCU in DFU mode. To do this, jumper BOOT0 pin at powering on.
And then, burn firmware using dfu-util via USB.

    $ dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D build/ch.bin


# Control NanoVNA from Host PC

NanoVNA is able to work standalone, but also be controlled from host
PC. Directory python contains sample script to control NanoVNA.


[EOF]
