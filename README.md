NanoVNA - Very tiny handheld Vector Network Analyzer
==========================================================

[![GitHub release](http://img.shields.io/github/release/ttrftech/NanoVNA.svg?style=flat)][release]
[![CircleCI](https://circleci.com/gh/ttrftech/NanoVNA.svg?style=shield)](https://circleci.com/gh/ttrftech/NanoVNA)

[release]: https://github.com/ttrftech/NanoVNA/releases

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

**UPDATE**: Recent gcc version works to build NanoVNA, no need old version.

### MacOSX

Install cross tools and firmware updating tool.

    $ brew tap px4/px4
    $ brew install gcc-arm-none-eabi-80
    $ brew install dfu-util

### Linux (ubuntu)

Download arm cross tools from [here](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads).

    $ wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/8-2018q4/gcc-arm-none-eabi-8-2018-q4-major-linux.tar.bz2
    $ sudo tar xfj -C /usr/local gcc-arm-none-eabi-8-2018-q4-major-linux.tar.bz2
    $ PATH=/usr/local/gcc-arm-none-eabi-8-2018-q4-major/bin:$PATH
    $ sudo apt install -y dfu-util

## Fetch source code

Fetch source and submodule.

    $ git clone https://github.com/ttrftech/NanoVNA.git
    $ cd NanoVNA
    $ git submodule update --init --recursive

## Build

Just make in the directory.

    $ make

### Build firmware using docker

Using [this docker image](https://cloud.docker.com/u/edy555/repository/docker/edy555/arm-embedded) without installing arm toolchain.

    $ cd NanoVNA
    $ docker run -it --rm -v $(PWD):/work edy555/arm-embedded:8.2 make

## Flash firmware

First, make device enter DFU mode by one of following methods.

* Jumper BOOT0 pin at powering device
* Select menu Config->DFU (needs recent firmware)

Then, flash firmware using dfu-util via USB.

    $ dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D build/ch.bin

Or simply use make.

    $ make flash

## Control from PC

See [python directory](/python/README.md).


## Note

Hardware design material is disclosed to prevent bad quality clone. Please let me know if you would have your own unit.


## Reference

* [Schematics](/doc/nanovna-sch.pdf)
* [PCB Photo](/doc/nanovna-pcb-photo.jpg)
* [Block Diagram](/doc/nanovna-blockdiagram.png)
* Kit available from https://ttrf.tk/kit/nanovna

## Credit

* [@edy555](https://github.com/edy555)

### Contributors

* [@hugen79](https://github.com/hugen79)
* [@cho45](https://github.com/cho45)
