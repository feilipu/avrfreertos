/*
 * Copyright (c) 2010-2012 Digi International Inc.,
 * All rights not expressly granted are reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Digi International Inc. 11001 Bren Road East, Minnetonka, MN 55343
 * =======================================================================
 */

/**
	@addtogroup hal_hcs08
	@{
	@file xbee/platform_avr.h
	Header for AVR combined with freeRTOS

	This file is automatically included by xbee/platform.h.
*/

#ifndef __XBEE_PLATFORM_AVR
#define __XBEE_PLATFORM_AVR

	#include "FreeRTOS.h"
	#include "task.h"

	// GCC doesn't have an endian.h, so define its macros here.
	#define LITTLE_ENDIAN	1234
	#define BIG_ENDIAN		4321
	#define PDP_ENDIAN		3412

	#define BYTE_ORDER		LITTLE_ENDIAN

	#define FAR			// the "FAR" modifier is not used
	#define PACKED_STRUCT	struct __attribute__((packed))

	#define XBEE_NATIVE_64BIT // avr-libc has 64 bit handling

	/// todo These two are supposed to be in wpan_types.h, but for some reason Eclipse can't find them there.

	/// Typedef used to hold a 64-bit IEEE address, represented as 8 bytes,
	/// 4 16-bit values or 2 32-bit values.
	/// Note that (for now) all addr64_t elements are stored MSB-first (the order
	/// used in XBee frames).
	/// @todo update all addr64_t variables and structure elements to end in _be
	/// (big-endian) or _le (little-endian) where appropriate.  Add functions
	/// to convert 64-bit values between host byte order and big/little endian.
	typedef union {
		uint8_t			b[8];
		uint16_t		u[4];
		uint32_t		l[2];
		uint64_t		ll;		// added for completeness
	} addr64_t;

	/// Single structure to hold an 802.15.4 device's 64-bit IEEE/MAC address
	/// and 16-bit network address.
	typedef PACKED_STRUCT wpan_address_t{
		addr64_t	ieee;
		uint16_t	network;
	} wpan_address_t;

	#define INTERRUPT_ENABLE	portENTER_CRITICAL()
	#define INTERRUPT_DISABLE	portEXIT_CRITICAL()

	#ifdef __BIG_ENDIAN__
		#error "This platform is supposed to be LITTLE ENDIAN"
	#endif

	// This type isn't in stdint.h
	typedef enum
	{
		FALSE = 0,
		TRUE = 1
	} bool_t;


	#include "lib_util.h" // for the CRC16 and swap functions

	#define _f_memcpy		memcpy
	#define _f_memset		memset

	#define HAVE_SWAP_FUNCS		1

	// Include my AVRfreeRTOS standard serial port calls.
	#include "serial.h"

	// redefine the xbee serial handle to match my AVRfreeRTOS serial handle.
	#define xbee_serial_t xComPortHandle

	#include "xbee_platform.h"
	#include "xbee_device.h"
	#include "xbee_serial.h"

	/* Create reference to the handle for the serial port, USART0. */
	/* This variable is special, as it is used in the interrupt */
	extern xComPortHandle xSerialPort;

#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
	/* Create reference to the handle for the other serial port, USART1. */
	/* This variable is special, as it is used in the interrupt */
	extern xComPortHandle xSerial1Port;
#endif

	#define xbee_seconds_timer()			(xTaskGetTickCount() * portTICK_PERIOD_MS / 1000)
	#define xbee_millisecond_timer()		(xTaskGetTickCount() * portTICK_PERIOD_MS)

	// our millisecond timer has portTICK_PERIOD_MS resolution
	#define XBEE_MS_TIMER_RESOLUTION		portTICK_PERIOD_MS

	// In this configuration, we can't reset the XBee or see if it's awake.
	void xbee_reset_radio( xbee_dev_t *xbee, bool_t asserted);
	void xbee_reset_pin( xbee_dev_t *xbee, bool_t enable);
	int xbee_awake_pin( xbee_dev_t *xbee);

#endif		// __XBEE_PLATFORM_AVR
