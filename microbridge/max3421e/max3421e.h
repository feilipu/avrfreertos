/*
Copyright 2011 Niels Brouwers

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.#include <string.h>
*/

/**
 *
 * Library for the max3421e USB host controller shield produced by circuitsathome and Sparkfun.
 * This is a low-level interface that provides access to the internal registers and polls the
 * controller for state changes.
 *
 * This library is based on work done by Oleg Masurov, but has been ported to C and heavily
 * restructured. Control over the GPIO pins has been stripped.
 *
 * Note that the current incarnation of this library only supports the Arduino Mega with a
 * hardware mod to rewire the MISO, MOSI, and CLK SPI pins.
 *
 * http://www.circuitsathome.com/
 */
#ifndef _MAX3421E_H_
#define _MAX3421E_H_

#ifdef __cplusplus
extern "C" {
#endif


/**
 * Max3421e registers in host mode.
 */
enum
{
	MAX_REG_RCVFIFO = 0x08,
	MAX_REG_SNDFIFO = 0x10,
	MAX_REG_SUDFIFO = 0x20,
	MAX_REG_RCVBC = 0x30,
	MAX_REG_SNDBC = 0x38,
	MAX_REG_USBIRQ = 0x68,
	MAX_REG_USBIEN = 0x70,
	MAX_REG_CPUCTL = 0x80,
	MAX_REG_USBCTL = 0x78,
	MAX_REG_PINCTL = 0x88,
	MAX_REG_REVISION = 0x90,
	MAX_REG_FNADDR = 0x98,
	MAX_REG_GPINIRQ = 0xb0,
	MAX_REG_HIRQ = 0xc8,
	MAX_REG_HIEN = 0xd0,
	MAX_REG_MODE = 0xd8,
	MAX_REG_PERADDR = 0xe0,
	MAX_REG_HCTL = 0xe8,
	MAX_REG_HXFR = 0xf0,
	MAX_REG_HRSL = 0xf8
} max_registers;


#define SPI_CLOCK_DIV4 0x00
#define SPI_CLOCK_DIV16 0x01
#define SPI_CLOCK_DIV64 0x02
#define SPI_CLOCK_DIV128 0x03
#define SPI_CLOCK_DIV2 0x04
#define SPI_CLOCK_DIV8 0x05
#define SPI_CLOCK_DIV32 0x06
//#define SPI_CLOCK_DIV64 0x07 // Doubled define with 0x02

#define SPI_MODE0 0x00
#define SPI_MODE1 0x04
#define SPI_MODE2 0x08
#define SPI_MODE3 0x0C

#define SPI_MODE_MASK 0x0C  // CPOL = bit 3, CPHA = bit 2 on SPCR
#define SPI_CLOCK_MASK 0x03  // SPR1 = bit 1, SPR0 = bit 0 on SPCR
#define SPI_2XCLOCK_MASK 0x01  // SPI2X = bit 0 on SPSR

#define SPI_LSBFIRST 0
#define SPI_MSBFIRST 1

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#define SPI_PORT PORTB
#define SPI_PORT_DIR DDRB
#define SPI_BIT_MISO 0x8
#define SPI_BIT_MOSI 0x4
#define SPI_BIT_SCK 0x2
#define SPI_BIT_SS 0x1

#elif  defined(__AVR_ATmega1284P__)

#define SPI_PORT PORTB
#define SPI_PORT_DIR DDRB
#define SPI_BIT_SCK 0x80
#define SPI_BIT_MISO 0x40
#define SPI_BIT_MOSI 0x20
#define SPI_BIT_SS 0x10

#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)

#define SPI_PORT PORTB
#define SPI_PORT_DIR DDRB
#define SPI_BIT_SCK 0x20
#define SPI_BIT_MISO 0x10
#define SPI_BIT_MOSI 0x8
#define SPI_BIT_SS 0x4

#endif

#define SPI_MISO ((SPI_PORT & SPI_BIT_MISO) >> 3)
#define SPI_MOSI(x) { if (x) SPI_PORT |= SPI_BIT_MOSI; else SPI_PORT &= ~SPI_BIT_MOSI; }
#define SPI_SCK(x) { if (x) SPI_PORT |= SPI_BIT_SCK; else SPI_PORT &= ~SPI_BIT_SCK; }
#define SPI_SS(x) { if (x) SPI_PORT |= SPI_BIT_SS; else SPI_PORT &= ~SPI_BIT_SS; }



#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#define MAX_SS(x) { if (x) PORTB |= 0x10; else PORTB &= ~0x10; }
#define MAX_INT() ((PORTH & 0x40) >> 6)
#define MAX_GPX() ((PORTH & 0x20) >> 5)
#define MAX_RESET(x) { if (x) PORTH |= 0x10; else PORTH &= ~0x10; }

#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)

#define MAX_SS(x) SPI_SS(x)
#define MAX_INT() ((PORTB & 2) >> 1)
#define MAX_GPX() (PORTB & 1) // don't use the GPX pin with the Danger Shield - Conflict on pin 8 - lead cut.
#define MAX_RESET(x) { if (x) PORTD |= 0x80; else PORTD &= ~0x80; }

#endif

// Sparkfun botched their first attempt at cloning Oleg's max3421e shield and reversed the GPX and RESET pins.
// This hack is in place to make MicroBridge work with those shields. (see http://www.sparkfun.com/products/9628)
// note: I used #undef here to avoid a bunch of ugly nested #ifdefs above
#ifdef SFHACK

#undef MAX_GPX
#undef MAX_RESET

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

// Untested!
#define MAX_GPX() ((PORTH & 0x10) >> 4)
#define MAX_RESET(x) { if (x) PORTH |= 0x20; else PORTH &= ~0x20; }

#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)

// Untested!
#define MAX_GPX() ((PORTD & 0x80) >> 7)
#define MAX_RESET(x) { if (x) PORTB |= 1; else PORTB &= ~1; }

#endif

#endif

void spiBegin();
void spiSetClockDivider(uint8_t rate);

void max3421e_init();
void max3421e_write(uint8_t reg, uint8_t val);
uint8_t * max3421e_writeMultiple(uint8_t reg, uint8_t count, uint8_t * values);
void max3421e_gpioWr(uint8_t val);
uint8_t max3421e_read(uint8_t reg);
uint8_t * max3421e_readMultiple(uint8_t reg, uint8_t count, uint8_t * values);
uint8_t max3421e_gpioRd(void);
boolean max3421e_reset();
boolean max3421e_vbusPwr(boolean action);
void max3421e_busprobe(void);
void max3421e_powerOn();
uint8_t max3421e_getVbusState();

uint8_t max3421e_poll(void);

uint8_t max3421e_interruptHandler(void);
// uint8_t max3421e_gpxInterruptHandler(void); XXX Removed because of GPX conflict with Danger Shield.


#ifdef __cplusplus
}
#endif

#endif //_MAX3421E_H_
