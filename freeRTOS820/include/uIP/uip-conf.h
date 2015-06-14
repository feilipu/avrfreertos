
/******************************************************************************

  Filename:		uip-conf.h
  Description:	uIP configuration file

 ******************************************************************************

  This program is free software; you can redistribute it and/or modify it
  under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  This file is part of the uIP TCP/IP stack.

 *****************************************************************************/

#ifndef __UIP_CONF_H__
#define __UIP_CONF_H__

#include <inttypes.h>
#include <stdio.h>
#include <avr/io.h>

#include "uIP/psock.h"

/**
 * Statistics data type
 *
 * This typedef defines the data type used for keeping statistics in
 * uIP.
 *
 * \hideinitializer
 */
typedef uint16_t uip_stats_t;


// Commonly accessed uIP stack settings
//
#define MAX_TCP_CONNS				10 // Max TCP connections desired
#define MAX_TCP_LISTENPORTS			10 // Max TCP listening ports
#define MAX_UDP_CONNS				20 // Max UDP connections desired

#define UIP_UDP_ENABLED				1  // Enable UDP


/** Do we use IPv6 or not (default: no) */
// #define UIP_CONF_IPV6			1

/**
 * uIP buffer size.
 *
 * \hideinitializer
 */
// #define UIP_CONF_BUFFER_SIZE		800
// otherwise UIP_BUFSIZE is (UIP_LINK_MTU + UIP_LLH_LEN)

/**
 * UIP_CONF_EXTERNAL_BUFFER is defined, because we're using pvPortMalloc to put it on the heap.
 * This is part of freeRTOS heap management, and is used where the heap might optionally be in external memory.
 *
 * \hideinitializer
 */
#define UIP_CONF_EXTERNAL_BUFFER

/**
 * Maximum number of TCP connections.
 *
 * \hideinitializer
 */
#ifdef MAX_TCP_CONNS
#define UIP_CONF_MAX_CONNECTIONS	(MAX_TCP_CONNS)
#endif

/**
 * Maximum number of listening TCP ports.
 *
 * \hideinitializer
 */
#ifdef MAX_TCP_LISTENPORTS
#define UIP_CONF_MAX_LISTENPORTS	(MAX_TCP_LISTENPORTS)
#endif

/**
 * UIP_REASSEMBLY should be defined as 1 (ON) or 0 (OFF) in your project area
 *
 *  Turn on IP packet re-assembly.
 *  This will re-assemble ip packets that become fragmented
 *  which when assembled will fit in your UIP BUFFER.
 *
 *  Note: This will double the amount of RAM used by the UIP BUFFER.
 *
 * \hideinitializer
 */
#define UIP_REASSEMBLY 				0

/**
 * CPU byte order.
 *
 * \hideinitializer
 */
#define UIP_CONF_BYTE_ORDER      	LITTLE_ENDIAN

/**
 * Logging on or off
 *
 * \hideinitializer
 */
#define UIP_CONF_LOGGING         	0

/**
 * UDP support on or off
 *
 * \hideinitializer
 */
#define UIP_CONF_UDP             	UIP_UDP_ENABLED

/**
 * UDP checksums on or off
 *
 * \hideinitializer
 */
#define UIP_CONF_UDP_CHECKSUMS   	UIP_UDP_ENABLED

/**
 * uIP statistics on or off
 *
 * \hideinitializer
 */
#define UIP_CONF_STATISTICS      	0

/**
 * Broadcast support.
 *
 * \hideinitializer
 */
#define UIP_CONF_BROADCAST		 	UIP_UDP_ENABLED

/**
 * The maximum amount of concurrent UDP connections.
 *
 * \hideinitializer
 */

#ifdef MAX_UDP_CONNS
#define UIP_CONF_UDP_CONNS		 	(MAX_UDP_CONNS)
#endif

#endif /* __UIP_CONF_H__ */

/** @} */
/** @} */
