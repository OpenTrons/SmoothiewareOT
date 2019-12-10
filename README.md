## Overview

*This is an Opentrons fork of the open-source [Smoothieware](https://github.com/smoothieware/smoothieware)*

A recent stable build can usually be found in the releases.

Smoothie is a free, opensource, high performance G-code interpreter and CNC controller written in Object-Oriented C++ for the LPC17xx micro-controller ( ARM Cortex M3 architecture ). It will run on a mBed, a LPCXpresso, a SmoothieBoard, R2C2 or any other LPC17xx-based board. The motion control part is a port of the awesome grbl.

For more information on smoothie as a whole, please check out their documentation [here](http://smoothieware.org/)

### Materials
Opentrons utilizes the `LPC1768` chip

### Quick Start
These are the quick steps to get OT Smoothie dependencies installed on your computer:
* Pull down a clone of the Opentrons Smoothie github fork project to your local machine.
* Install [GNU-ARM compiler](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads) for your Operating System.
* Ensure that your path is correctly set by searching for the location of: `arm-none-eabi-gcc/../../bin`
* You can check that your path is correct by typing `arm-none-eabi-gcc --help` and see that the function menu pops up, and also that it is a _gcc_ compiler *not* a _clang_ compiler

## Building and Compiling OT Smoothie
From a shell, switch into the root Smoothie project directory and run:
```
make clean build
```

This will generate a hex file which can be added onto an SD card to be placed in a smoothie board that will run our software. If you are receiving an error that the compiler is not
being recognized then try the following on a Unix/Linux distro: `PATH=$PATH:/path/to/bin/file/gcc-arm-none-eabi-8-2018-q4-major/bin make clean build`

If you are connected to a smoothie board, you can use the command
```
make opentrons
```
To automatically set up the board for you.

## Filing issues
Please follow the guide [here](https://github.com/Opentrons/opentrons/blob/edge/CONTRIBUTING.md#opening-issues)

## Contributing

Please take a look [here](https://github.com/Opentrons/opentrons/blob/edge/CONTRIBUTING.md#opening-pull-requests)

Contributions are very welcome !

## Releasing (for Opentrons devs)

When you want to release a version of the smoothie software for subsequent Robot OS updates, just create a github release in this repository. Travis will automatically build the tag and post it to the release. 

Note: You must create the github release first - don't create and push a tag from your remote. The release needs to be created for Travis to properly deploy the result.

Once you've made the release, go update the [package in buildroot](https://github.com/Opentrons/buildroot/blob/opentrons-develop/package/opentrons-smoothie-firmware/opentrons-smoothie-firmware.mk).

## Donate
We based our firmware off of the smoothie firmware which is developed by volunteers. If you find this software useful, want to say thanks and encourage development, please consider a
[donation!](https://paypal.me/smoothieware)

## License

Smoothieware is released under the [GNU GPL v3](http://www.gnu.org/licenses/gpl-3.0.en.html)
