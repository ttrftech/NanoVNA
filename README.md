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

Requires gcc-4.9 to build firmware from source code. (Not work gcc-5.4 or lator, because of SRAM shortage that those runtime use more SRAM)

### MacOSX

Install cross tools and firmware updating tool.

    $ brew tap px4/px4
    $ brew install gcc-arm-none-eabi-49
    $ brew install dfu-util

Otherwise, use toolchains included inside LPCxpresso. Like this.

    $ PATH=$PATH:/Applications/lpcxpresso_7.8.0_426/lpcxpresso/tools/bin

### Linux (ubuntu)

Download arm cross tools from [here](https://launchpad.net/gcc-arm-embedded/4.9/4.9-2015-q3-update).
This version is 32-bit binary, so additional lib32z1 and lib32ncurses5 package required.

    $ wget https://launchpad.net/gcc-arm-embedded/4.9/4.9-2015-q3-update/+download/gcc-arm-none-eabi-4_9-2015q3-20150921-linux.tar.bz2
    $ sudo tar xfj -C /usr/local gcc-arm-none-eabi-4_9-2015q3-20150921-linux.tar.bz2
    $ PATH=/usr/local/gcc-arm-none-eabi-4_9-2015q3/bin:$PATH
    $ sudo apt install -y lib32z1 lib32ncurses5
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

If you can use docker, you can build firmware without installing arm toolchain.

    $ cd NanoVNA
    $ docker run -it --rm -v $(PWD):/work edy555/arm-embedded make


## Flash firmware

Boot MCU in DFU mode. To do this, jumper BOOT0 pin at powering device.
Then, burn firmware using dfu-util via USB.

    $ dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D build/ch.bin


## Control from PC

See [python directory](/python/README.md).


## Note

Hardware design material is disclosed to prevent bad quality clone. Please let me know if you would have your own unit.


## Reference

* [Schematics](/doc/nanovna-sch.pdf)
* [PCB Photo](/doc/nanovna-pcb-photo.jpg)
* [Block Diagram](/doc/nanovna-blockdiagram.png)
* Kit available from https://ttrf.tk/kit/nanovna
* Credit: @edy555

[EOF]
