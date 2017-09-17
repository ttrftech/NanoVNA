NanoVNA - Very tiny handheld Vector Network Analyzer
==========================================================

<div align="center">
<img src="/doc/nanovna.jpg" width="480px">
</div>

# About

NanoVNA is very tiny handheld Vector Network Analyzer (VNA). It is
standalone with lcd display, portable device with battery. This
project aim to provide an RF gadget but useful instrument for
enthusiast.

This repository contains source of NanoVNA firmware.


## Prepare ARM Cross Tools

Install cross tools and firmware updating tool.

    $ brew tap PX4/px4
    $ brew install gcc-arm-none-eabi
    $ brew instal dfu-util

Otherwise, use toolchains included inside LPCxpresso. Like this.

    $ PATH=$PATH:/Applications/lpcxpresso_7.8.0_426/lpcxpresso/tools/bin

## Build firmware

Fetch ChibiOS submodule into tree.

    $ cd nanovna
    $ git submodule update --init --recursive

Just make in the top directory.

    $ make

## Flash firmware

Boot MCU in DFU mode. To do this, jumper BOOT0 pin at powering device.
Then, burn firmware using dfu-util via USB.

    $ dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D build/ch.bin


# Control from PC

NanoVNA is able to work standalone, but also be controlled via USB serial interface from PC. There are sample scripts in python directory.

Preparation.

    $ pip install numpy
    $ pip install scikit-rf   

Plot reflection LOGMAG.

    $ cd python
    $ ./nanovna.py -p

Plot transmission LOGMAG.

    $ ./nanovna.py -p -P 1

Plot smithchart.

    $ ./nanovna.py -s

Show usage.

    $ ./nanovna.py -h

To use NanoVNA from Jupyter notebook, see [this page](/python/NanoVNA-example.ipynb).

## Reference

* [Schematics](/doc/nanovna-sch.pdf)
* [PCB Photo](/doc/nanovna-pcb-photo.jpg)
* [Block Diagram](/doc/nanovna-blockdiagram.png)
* Kit available from http://ttrftech.tumblr.com/kit/nanovna
* Credit: @edy555

[EOF]
