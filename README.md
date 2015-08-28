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

## Copyright & License

Copyright (c) 2015 Thingsee - Released (mostly) under the BSD license.
