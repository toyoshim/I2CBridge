// Copyright 2016, Takashi Toyoshima <toyoshim@gmail.com>
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//    * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
//    * Neither the name of the authors nor the names of its contributors
// may be used to endorse or promote products derived from this software
// without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
#include <stdbool.h>

// from CMSIS_CORE_LPC8xx
#include "LPC8xx.h"

// from lpc800_driver_lib just for using register definitions.
#include "lpc8xx_i2c.h"

// Constant variables for readability.
enum {
  CLK_I2C = (1 << 5),
  CLK_UART0 = (1 << 14),
  CLK_IOCON = (1 << 18),

  PIO_OPENDRAIN = (1 << 10),

  UARTCLKDIV_ENABLED = 1,

  RESET_UART0_N = (1 << 3),
  RESET_I2C_N = (1 << 6),

  PINENABLE_DISABLE_SWCLK = (1 << 2),
  PINENABLE_DISABLE_SWDIO = (1 << 3),
  PINENABLE_DISABLE_RESET = (1 << 6),

  I2C_CFG_MSTEN = (1 << 0),
  I2C_CFG_TIMEOUTEN = (1 << 3),

  I2C_MSTTIME_LOW_2T = (2 << 0),
  I2C_MSTTIME_HIGH_2T = (2 << 4),

  USART_STAT_RXRDY = (1 << 0),
  USART_STAT_TXRDY = (1 << 2),

  USART_CFG_DISABLED = 0,
  USART_CFG_ENABLED = 1,
  USART_CFG_DATALEN_8BIT = (1 << 2),
  USART_CFG_PARITY_NO = 0,
  USART_CFG_STOPLEN_1BIT = 0,

  // Configurable parameters.
  CPU_CLOCK = 12000000,
  USART_CLOCK = 230400,
  I2C_RETRY = 1
};

#define USE_SWD
#  ifdef USE_SWD
// PIO0_1, PIO0_5: I2C SDA, SCL
// PIO0_2, PIO0_3: SWD SWDIO, SWCLK
// PIO0_0, PIO0_4: USART RX, TX
#  define DISABLED_PINS PINENABLE_DISABLE_RESET
#  define SCL_PIO PIO0_5
#  define SDA_PIO PIO0_1
#  define SCL_PIN 5
#  define SDA_PIN 1
#else
// PIO0_1, PIO0_5: LED, RST_N
// PIO0_2, PIO0_3: I2C SCL, SDA
// PIO0_0, PIO0_4: USART RX, TX
#  define DISABLED_PINS = (PINENABLE_DISABLE_SWCLK | PINENABLE_DISABLE_SWDIO)
#  define SCL_PIO PIO0_2
#  define SDA_PIO PIO0_3
#  define SCL_PIN 2
#  define SDA_PIN 3
#endif

// Functions for I2C.
void i2c_init() {
  // Pin configurations.
  LPC_SWM->PINENABLE0 |= DISABLED_PINS;

  LPC_IOCON->SCL_PIO = PIO_OPENDRAIN;
  LPC_IOCON->SDA_PIO = PIO_OPENDRAIN;

  LPC_SWM->PINASSIGN7 = (LPC_SWM->PINASSIGN7 & 0x00ffffffUL) | (SDA_PIN << 24);
  LPC_SWM->PINASSIGN8 = (LPC_SWM->PINASSIGN8 & 0xffffff00UL) | (SCL_PIN << 0);

  LPC_SYSCON->PRESETCTRL &= ~RESET_I2C_N;
  LPC_SYSCON->PRESETCTRL |= RESET_I2C_N;

  // Setup as I2C master running at 100KHz.
  LPC_I2C->DIV = CPU_CLOCK / 100000 / 4 - 1;
  LPC_I2C->MSTTIME = I2C_MSTTIME_LOW_2T | I2C_MSTTIME_HIGH_2T;
  LPC_I2C->CFG |= I2C_CFG_MSTEN;
}

bool i2c_check(uint32_t state) {
  while (!(LPC_I2C->STAT & STAT_MSTPEND));
  return (LPC_I2C->STAT & MASTER_STATE_MASK) == state;
}

bool i2c_master(uint8_t addr, uint8_t* data, uint8_t size) {
  bool read = addr & 1;

  uint8_t try;
  for (try = 0; try < I2C_RETRY; ++try) {
    LPC_I2C->MSTDAT = addr;
    LPC_I2C->MSTCTL = CTL_MSTSTART;
    if (i2c_check(read ? STAT_MSTRX : STAT_MSTTX))
      break;
  }
  // When NACK is detected, STOP is sent automatically.
  if (try == I2C_RETRY)
      return false;

  for (uint8_t i = 0; i < size; i++) {
    if (read)
      data[i] = LPC_I2C->MSTDAT;
    else
      LPC_I2C->MSTDAT = data[i];
    LPC_I2C->MSTCTL = CTL_MSTCONTINUE;
    if (!i2c_check(read ? STAT_MSTRX : STAT_MSTTX))
      return false;
  }
  LPC_I2C->MSTCTL = CTL_MSTSTOP;
  return i2c_check(STAT_MSTIDLE);
}

bool i2c_write(uint8_t addr, uint8_t* data, uint8_t size) {
  if (addr & 1)
    return false;
  return i2c_master(addr, data, size);
}

bool i2c_read(uint8_t addr, uint8_t* data, uint8_t size) {
  if (!(addr & 1))
    return false;
  return i2c_master(addr, data, size);
}

// Functions for USART
void usart_init() {
  // Assign USART RX and TX to PIO0_0 and PIO0_4.
  LPC_SWM->PINASSIGN0 = (LPC_SWM->PINASSIGN0 & 0xffff0000UL) | 0x00000004UL;

  LPC_SYSCON->UARTCLKDIV = UARTCLKDIV_ENABLED;
  LPC_SYSCON->PRESETCTRL &= ~RESET_UART0_N;
  LPC_SYSCON->PRESETCTRL |= RESET_UART0_N;
  LPC_USART0->CFG = USART_CFG_DATALEN_8BIT | USART_CFG_PARITY_NO
      | USART_CFG_STOPLEN_1BIT | USART_CFG_DISABLED;

  // Baudrate.
  LPC_USART0->BRG = CPU_CLOCK / 16 / USART_CLOCK - 1;
  LPC_SYSCON->UARTFRGDIV = 0xFF;
  LPC_SYSCON->UARTFRGMULT = (((CPU_CLOCK / 16) * (LPC_SYSCON->UARTFRGDIV + 1))
      / (USART_CLOCK * (LPC_USART0->BRG + 1))) - (LPC_SYSCON->UARTFRGDIV + 1);

  LPC_USART0->CFG |= USART_CFG_ENABLED;
}

void usart_putc(uint8_t c) {
  while (!(LPC_USART0->STAT & USART_STAT_TXRDY));
  LPC_USART0->TXDATA = c;
}

void usart_puts(const char* s) {
  while (*s)
    usart_putc(*s++);
}

uint8_t usart_getc() {
  while (!(LPC_USART0->STAT & USART_STAT_RXRDY));
  return LPC_USART0->RXDATA;
}

// I2CBridge main loop to handle bridge commands from USART to I2C.
// command byte format:
//  |7|6|5|4|3|2|1|0|
//  |e| cmd |  data |
// e: If e is 1 and cmd is not 0, executes a read or write I2C operation.
//    I2C data size is calculated from the cmd value by the following equation.
//      data size = cmd >> 1
//    If e is 1 and cmd is 0, executes a bridge mode configuration.
//      data:
//        0 ... run I2C at 100kHz
//        1 ... run I2C at 400kHz
//        2 ... run I2C at 1MHz
//        3 ... run as fast as possible
// cmd:
//   0 ... Set high 4-bits of I2C target address
//   1 ... Set low 4-bits of I2C target address (low 3-bites and r/w bit)
//   2 ... Set high 4-bits of I2C 1st data
//   3 ... Set low 4-bits of I2C 1st data
//   4 ... Set high 4-bits of I2C 2nd data
//   5 ... Set low 4-bits of I2C 2nd data
//   6 ... Set high 4-bits of I2C 3rd data
//   7 ... Set low 4-bits of I2C 3rd data
// response:
//   0x00 ... success, read data will follow if cmd is for read
//   0xff ... error
//
// example:
//   0x0a, 0x10, 0x20, 0x31, 0x42, 0xd3 => 0x00
//     write, length = 2, address = 0x50, data = 0x01, 0x23, success
//   0x0a, 0x11, 0x20, 0xe0 => 0x00, 0x01, 0x23, 0x45
//     read, length = 3, address = 0x50, success, data = 0x01, 0x23, 0x45
//   0x0a, 0x13, 0x20, 0xa0 => 0xff
//     read, length = 1, address = 0x51, failure
int main() {
  LPC_SYSCON->SYSAHBCLKCTRL |= CLK_IOCON | CLK_UART0 | CLK_I2C;

  i2c_init();
  usart_init();

  uint8_t address = 0;
  uint8_t data[3] = { 0, 0, 0 };
  for (;;) {
    uint8_t rxdata = usart_getc();
    uint8_t size = 1;
    if (0x80 == (rxdata & 0xf0)) {
      switch (rxdata) {
      case 0x80:
        LPC_I2C->DIV = CPU_CLOCK / 100000 / 4 - 1;
        break;
      case 0x81:
        LPC_I2C->DIV = CPU_CLOCK / 400000 / 4 - 1;
        break;
      case 0x82:
        LPC_I2C->DIV = CPU_CLOCK / 1000000 / 4 - 1;
        break;
      case 0x83:
        LPC_I2C->DIV = 0;
        break;
      }
      continue;
    } else switch (rxdata & 0x70) {
    case 0x00:  // Set Address High
      address = (address & 0x0f) | ((rxdata << 4) & 0xf0);
      break;
    case 0x10:  // Set Address Low
      address = (address & 0xf0) | (rxdata & 0x0f);
      break;
    case 0x20:  // Set Data1 High
      data[0] = (data[0] & 0x0f) | ((rxdata << 4) & 0xf0);
      break;
    case 0x30:  // Set Data1 Low
      data[0] = (data[0] & 0xf0) | (rxdata & 0x0f);
      break;
    case 0x40:  // Set Data2 High
      data[1] = (data[1] & 0x0f) | ((rxdata << 4) & 0xf0);
      size = 2;
      break;
    case 0x50:  // Set Data2 Low
      data[1] = (data[1] & 0xf0) | (rxdata & 0x0f);
      size = 2;
      break;
    case 0x60:  // Set Data3 High
      data[2] = (data[2] & 0x0f) | ((rxdata << 4) & 0xf0);
      size = 3;
      break;
    case 0x70:  // Set Data3 Low
      data[2] = (data[2] & 0xf0) | (rxdata & 0x0f);
      size = 3;
      break;
    }
    if (rxdata & 0x80) {
      bool read = address & 1;
      bool result =
          read ? i2c_read(address, data, size) : i2c_write(address, data, size);
      usart_putc(result ? 0 : 255);
      if (result && read) {
        for (uint8_t i = 0; i < size; ++i)
          usart_putc(data[i]);
      }
    }
  }
  return 0;
}
