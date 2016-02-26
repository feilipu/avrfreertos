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
 * Note that the current incarnation of this library only supports the Seeed MEGA ADK with
 * hardware connected to the MISO, MOSI, SCK, and SS SPI pins.
 *
 * INT - PE6
 * GPX - PJ3
 * HOST RESET - PJ2
 *
 */
#ifndef _MAX3421E_H_
#define _MAX3421E_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// AVR include files.
#include <avr/io.h>

#include "FreeRTOS.h"
#include "spi.h"


/**
 * Max3421e registers in host mode.
 */

typedef enum
{
	MAX_REG_RCVFIFO =	0x08,
	MAX_REG_SNDFIFO =	0x10,
	MAX_REG_SUDFIFO =	0x20,
	MAX_REG_RCVBC =		0x30,
	MAX_REG_SNDBC =		0x38,
	MAX_REG_USBIRQ =	0x68,
	MAX_REG_USBIEN =	0x70,
	MAX_REG_CPUCTL =	0x80,
	MAX_REG_USBCTL =	0x78,
	MAX_REG_PINCTL =	0x88,
	MAX_REG_REVISION =	0x90,
	MAX_REG_FNADDR =	0x98,
	MAX_REG_GPINIRQ =	0xb0,
	MAX_REG_HIRQ =		0xc8,
	MAX_REG_HIEN =		0xd0,
	MAX_REG_MODE =		0xd8,
	MAX_REG_PERADDR =	0xe0,
	MAX_REG_HCTL =		0xe8,
	MAX_REG_HXFR =		0xf0,
	MAX_REG_HRSL =		0xf8
} max_registers_t;


#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
/*
 * INT - PE6
 * GPX - PJ3
 * HOST RESET - PJ2
 */
#define MAX_SS(x) { if (x) PORTB |= _BV(PB0); else PORTB &= ~_BV(PB0); }
#define MAX_INT() ((PORTE & _BV(PE6)) >> 6)
#define MAX_GPX() ((PORTJ & _BV(PJ3)) >> 3)
#define MAX_RESET(x) { if (x) PORTJ |= _BV(PJ2); else PORTJ &= ~_BV(PJ2); }

#else
#warning no INT, GPX, RESET pins defined.

#endif


void      max3421e_init(void);
void      max3421e_write(max_registers_t reg, uint8_t val) __attribute__ ((flatten));
uint8_t * max3421e_writeMultiple(max_registers_t reg, uint8_t count, uint8_t * values);
uint8_t   max3421e_read(max_registers_t reg) __attribute__ ((flatten));
uint8_t * max3421e_readMultiple(max_registers_t reg, uint8_t count, uint8_t * values);
uint8_t   max3421e_reset(void);
uint8_t   max3421e_vbusPwr(uint8_t action);
void      max3421e_busprobe(void);
void      max3421e_powerOn(void);
uint8_t   max3421e_getVbusState() __attribute__ ((flatten));

uint8_t   max3421e_poll(void);

uint8_t   max3421e_interruptHandler(void) __attribute__ ((flatten));
uint8_t   max3421e_gpxInterruptHandler(void) __attribute__ ((flatten));


#ifdef __cplusplus
}
#endif

#endif //_MAX3421E_H_
