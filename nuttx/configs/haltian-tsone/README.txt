README
======

This NuttX configuration is for the Haltian Thingsee One board.

This configuration has been tested with "GNU Tools for ARM Embedded Processors"
toolchain: https://launchpad.net/gcc-arm-embedded

Thingsee One is generic platform for HW and SW development in IOT. It
includes several different connectivity modules, sensors, mass memories, power
management and interfaces for test and debugging purposes. It can be easily
expanded with other modules as there are test pads for access I2C, SPI and UART
as well as for supporting GPIOs. There are power supply and GND pads for powering
the external device.  Here is a list of the key components and features:

 - MCU: STM32L162VEY, Ultra-low-power ARM Cortex-M3 MCU with 512kB Flash,
   80kB RAM, 32MHz CPU, USB, AES

 - Modem: u-blox Sara-G350, Quad-band GPRS module with A-GPS support

 - GPS: u-blox Max-7Q, GPS/GLONASS receiver

 - WLAN: Murata LBWA1ZZVK7, IEEE 802.11 b/g compliant (TI CC3000 based)

 - Bluetooth low energy: TI CC2640, BT BLE 4.1 compliant 

 - Display: Sharp LS027B7DH01, 2,7” WQVGA monochrome HR-TFT with internal
   memory

 - SD Card: Micro SD card reader on SPI

 - Acceleration sensor: ST LIS2DH

 - Pressure sensor: ST LPS25H

 - Temperature and Humidity sensor: Sensirion SHT25

 - 9-axis sensor combo with Gyroscope, Magnetometer and Accelerometer: ST LSM9DS1

 - Capsense controller: Cypress CY8CMBR3108, Capacitive buttons, sliders and
   proximity

 - Ambient light sensor: MAX44009

 - USB: USB2.0 full-speed device mode and USB Charging compliant with BC1.2
   compatible charger detection

 - Regulators: ST STLQ015XG30, TI TPS62233 and TI TPS61093

 - Charging: TI BQ24251, 2A switch-mode Li-Ion battery charger with power-path

 - Battery: BAK rechargeable Li-ion 1900mAh

