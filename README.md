# Thingsee OS SDK

Thingsee OS is the real-time operating system powering [Thingsee One](http://thingsee.com), a smart, durable and connected sensor device. Thingsee OS is based on [NuttX](http://nuttx.org/).

## Installation

1) Pre-requisites:

GCC ARM Embedded toolchain is available at https://launchpad.net/gcc-arm-embedded
and maintained by ARM employee​s. Packages for Windows, Mac and Linux are
available for download. For Ubuntu, a PPA is provided for easy installation.
Toolchain provides GCC, G++, GDB for embedded ARM targets. Newlib is used to
provide C standard libraries. To install in Ubuntu 14.04, see following steps:

Add GCC ARM Embedded PPA as software source with command:

```
sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded
```

2) Install the GCC ARM Embedded toolchain:

```
sudo apt-get update
sudo apt-get install gcc-arm-none-eabi
```

How to Build:

```
cd nuttx/configs
../tools/configure.sh haltian_tsone/retail
make
```

## Updating firmware

Via the SD card

 1. Rename nuttx.oci to update.oci.
 2. Copy update.oci to Thingsee's mass memory (SD card).
 3. Shutdown and restart the device.
 4. Thingsee will flash the device automatically, when update.oci file is found on SD card at bootup.

Alternatively you can flash the firmware dfu file with dfu-util (Please use >= dfu-util 0.8-1).

The easiest way to activate DFU mode in the Thingsee device is to:

 1. Open the case to access the bottom side of the board.
 2. Locate “FLASH” and “RESET” buttons.
 3. Hold “FLASH” button and then press and release “RESET” button.
 4. Your device should now be in DFU mode and ready for flashing.

Plug the USB cable to the micro-USB connector of your device. Use following command:

```
 dfu-util -d 0483:df11 -a0 -D nuttx.dfu -s :leave
```

## Copyright & License

Copyright (c) 2015 Thingsee - Released (mostly) under the BSD license.
