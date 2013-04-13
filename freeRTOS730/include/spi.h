/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

// AVR include files.
#include <avr/io.h>

#define SPI_TIMEOUT 1000		// Timeout to get access to SPI bus in mS

#define SPI_CLOCK_DIV4   0x00
#define SPI_CLOCK_DIV16  0x01
#define SPI_CLOCK_DIV64  0x02
#define SPI_CLOCK_DIV128 0x03
#define SPI_CLOCK_DIV2   0x04
#define SPI_CLOCK_DIV8   0x05
#define SPI_CLOCK_DIV32  0x06

#define SPI_MODE0 0x00
#define SPI_MODE1 0x04
#define SPI_MODE2 0x08
#define SPI_MODE3 0x0C

#define SPI_MODE_MASK    0x0C  // CPOL = bit 3, CPHA = bit 2 on SPCR
#define SPI_CLOCK_MASK   0x03  // SPR1 = bit 1, SPR0 = bit 0 on SPCR
#define SPI_2XCLOCK_MASK 0x01  // SPI2X = bit 0 on SPSR

#define SPI_LSBFIRST 0
#define SPI_MSBFIRST 1


#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)

#define SPI_PORT	 PORTB
#define SPI_PORT_DIR DDRB
#define SPI_PORT_PIN PINB
#define SPI_BIT_MISO 0x08
#define SPI_BIT_MOSI 0x04
#define SPI_BIT_SCK  0x02
#define SPI_BIT_SS   0x01			// Standard SS line Pin 53

#define SPSR_BIT_SPIF 0x80

#define SPI_BIT_SS_MEGA_WIZNET 0x10 //  these lines added for the EtherMega Wiznet support with SS on PB4
#define SPI_SS_MEGA_WIZNET(x) { if (x) SPI_PORT |= SPI_BIT_SS_MEGA_WIZNET; else SPI_PORT &= ~SPI_BIT_SS_MEGA_WIZNET; }

#if defined(portSD_CARD)			//  these lines added for the EtherMega SD Card support with SS on PG5

#define SPI_PORT_SS_MEGA_SD		PORTG
#define SPI_PORT_DIR_SS_MEGA_SD	DDRG
#define SPI_PORT_PIN_SS_MEGA_SD	PING
#define SPI_BIT_SS_MEGA_SD		0x20
#define SPI_SS_MEGA_SD(x) { if (x) SPI_PORT_SS_MEGA_SD |= SPI_BIT_SS_MEGA_SD; else SPI_PORT_SS_MEGA_SD &= ~SPI_BIT_SS_MEGA_SD; }
#endif

#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)

#define SPI_PORT	 PORTB
#define SPI_PORT_DIR DDRB
#define SPI_PORT_PIN PINB
#define SPI_BIT_SCK  0x20
#define SPI_BIT_MISO 0x10
#define SPI_BIT_MOSI 0x08
#define SPI_BIT_SS   0x04

#define SPSR_BIT_SPIF 0x80

#elif defined(__AVR_ATmega324P__)  || defined(__AVR_ATmega644P__)|| defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__)

#define SPI_PORT	 PORTB
#define SPI_PORT_DIR DDRB
#define SPI_PORT_PIN PINB
#define SPI_BIT_SCK  0x80
#define SPI_BIT_MISO 0x40
#define SPI_BIT_MOSI 0x20
#define SPI_BIT_SS   0x10

#define SPSR_BIT_SPIF0 0x80

#else
#error "SPI Interface not defined: Unknown MCU"
#endif

#define SPI_MISO ((SPI_PORT & SPI_BIT_MISO) >> 3)
#define SPI_MOSI(x) { if (x) SPI_PORT |= SPI_BIT_MOSI; else SPI_PORT &= ~SPI_BIT_MOSI; }
#define SPI_SCK(x)  { if (x) SPI_PORT |= SPI_BIT_SCK;  else SPI_PORT &= ~SPI_BIT_SCK; }
#define SPI_SS(x)   { if (x) SPI_PORT |= SPI_BIT_SS;   else SPI_PORT &= ~SPI_BIT_SS; }		// default Slave Select.

/* Options for Slave Select lines */
typedef enum {
	SS_PB2,			/* 0: Default Arduino */
	SS_PG5, 		/* 1: EtherMega SD Card */
	SS_PB4,			/* 2: EtherMega Wiznet Ethernet */
	SS_PB0			/* 3: Default EtherMega (Arduino Mega) Pin 53 Slave Select */
					/* 4: Add additional SS lines as necessary, and to spi.c */
} SPI_SLAVE_SELECT;

void spiSetClockDivider(uint8_t rate);
void spiSetBitOrder(uint8_t bitOrder);
void spiSetDataMode(uint8_t mode);

void spiAttachInterrupt();
void spiDetachInterrupt();

uint8_t spiSelect (SPI_SLAVE_SELECT SS_pin);
void spiDeselect (SPI_SLAVE_SELECT SS_pin);

void spiBegin(SPI_SLAVE_SELECT SS_pin);

void spiEnd();

/*
 * In testing with a Freetronics EtherMega driving an SD card
 * the system achieved the following results.
 *
 * Single byte transfer MOSI 3.750uS MISO 3.6250us
 * Multi- byte transfer MOSI 1.333uS MISO 1.3750uS
 *
 * Performance increase MOSI 2.8x    MISO 2.64x
 *
 * Worth doing if you can!
 *
 */

uint8_t spiTransfer(uint8_t data);

uint8_t spiMultiByteTx(const uint8_t *data, const uint16_t length);
uint8_t spiMultiByteRx(uint8_t *data, const uint16_t length);

uint8_t spiMultiByteTransfer(uint8_t *data, const uint16_t length);

#ifdef __cplusplus
}
#endif

#endif
