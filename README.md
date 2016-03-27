# I2CBridge
LPC810: Serial to I2C bridge

## What's this?
Serial to I2C brige firmware for LPC810. This firmware accept commands from serial port, then execute I2C operations to return response through the serial port.

## Schematic
[![Schematic](https://raw.githubusercontent.com/toyoshim/I2CBridge/master/schem.png "Schematic")](https://upverter.com/toyoshim/564092a49959599c/I2CBridge/)

## How to build
This repository includes prebiult binaries under Release/ directory, but also you can build it by yourself with the LPCXpresso project files included in this repository too.
You need to import CMSIS_CORE_LPX8xx and lpc800_driver_lib from <lpcxpresso>/Examples/Legacy/NXP/LPC800/LPC8xx_Libraries.zip to use these libraries.

## How to use
Connect with I2C slave devices through I2C bus, and connect with your PC through USART.
You can read from or write to I2C devices through commands requested from USART.

### command byte format
~~~~
|7|6|5|4|3|2|1|0|
|e| cmd |  data |
~~~~

* e:
 * If 1, executes I2C access with parameters set. If cmd is for 2nd or 3rd data, execute 2 or 3 bytes read or write. Otherwise do for 1 byte.
* cmd:
 * 0 ... Set high 4-bits of I2C target address
 * 1 ... Set low 4-bits of I2C target address (low 3-bites and r/w bit)
 * 2 ... Set high 4-bits of I2C 1st data
 * 3 ... Set low 4-bits of I2C 1st data
 * 4 ... Set high 4-bits of I2C 2nd data
 * 5 ... Set low 4-bits of I2C 2nd data
 * 6 ... Set high 4-bits of I2C 3rd data
 * 7 ... Set low 4-bits of I2C 3rd data
* response:
 * 0x00 ... success, read data will follow if cmd is for read
 * 0xff ... error

### example
 * 0x0a, 0x10, 0x20, 0x31, 0x42, 0xd3 => 0x00
  * write, length = 2, address = 0x50, data = 0x01, 0x23, success
 * 0x0a, 0x11, 0x20, 0xe0 => 0x00, 0x01, 0x23, 0x45
  * read, length = 3, address = 0x50, success, data = 0x01, 0x23, 0x45
 * 0x0a, 0x13, 0x20, 0xa0 => 0xff
  * read, length = 1, address = 0x51, failure
