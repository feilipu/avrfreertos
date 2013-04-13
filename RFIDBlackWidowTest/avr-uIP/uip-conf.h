
/******************************************************************************

  Filename:		uip-conf.h
  Description:	uIP configuration file

 ******************************************************************************

  TCP/IP stack and driver for the WiShield 1.0 wireless devices

  Copyright(c) 2009 Async Labs Inc. All rights reserved.

  This program is free software; you can redistribute it and/or modify it
  under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  this program; if not, write to the Free Software Foundation, Inc., 59
  Temple Place - Suite 330, Boston, MA  02111-1307, USA.

  Contact Information:
  <asynclabs@asynclabs.com>

   Author               Date        Comment
  ---------------------------------------------------------------
   AsyncLabs			05/29/2009	Initial port

 *****************************************************************************/

#ifndef __UIP_CONF_H__
#define __UIP_CONF_H__

#include <inttypes.h>
#include <stdio.h>
#include <avr/io.h>

//Include app configuration
#include "apps-conf.h"

/**
 * 8 bit data type
 *
 * This typedef defines the 8-bit type used throughout uIP.
 *
 * \hideinitializer
 */
typedef uint8_t u8_t;

/**
 * 16 bit data type
 *
 * This typedef defines the 16-bit type used throughout uIP.
 *
 * \hideinitializer
 */
typedef uint16_t u16_t;

/**
 * Statistics data type
 *
 * This typedef defines the data type used for keeping statistics in
 * uIP.
 *
 * \hideinitializer
 */
typedef unsigned short uip_stats_t;

/**
 * Maximum number of TCP connections.
 *
 * \hideinitializer
 */
#define UIP_CONF_MAX_CONNECTIONS MAX_TCP_CONNS

/**
 * Maximum number of listening TCP ports.
 *
 * \hideinitializer
 */
#define UIP_CONF_MAX_LISTENPORTS MAX_TCP_LISTENPORTS

/**
 * uIP buffer size.
 *
 * \hideinitializer
 */

//#define UIP_SCAN // scan for APs using ZG2100 user contributed code.

#ifdef UIP_SCAN
#define UIP_CONF_BUFFER_SIZE				800
#else
#define UIP_CONF_BUFFER_SIZE				200 // normally 450
#endif // UIP_SCAN

/**
 * UIP_CONF_EXTERNAL_BUFFER is defined, because we're using pvPortMalloc to put it on the heap.
 * This is part of freeRTOS heap management
 *
 * \hideinitializer
 */
//#define UIP_CONF_EXTERNAL_BUFFER


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
#define UIP_REASSEMBLY 			0


/**
 * CPU byte order.
 *
 * \hideinitializer
 */
#define UIP_CONF_BYTE_ORDER      LITTLE_ENDIAN

/**
 * Logging on or off
 *
 * \hideinitializer
 */
#define UIP_CONF_LOGGING         0

/**
 * UDP support on or off
 *
 * \hideinitializer
 */
#define UIP_CONF_UDP             UIP_UDP_ENABLED

/**
 * UDP checksums on or off
 *
 * \hideinitializer
 */
#define UIP_CONF_UDP_CHECKSUMS   UIP_UDP_ENABLED

/**
 * uIP statistics on or off
 *
 * \hideinitializer
 */
#define UIP_CONF_STATISTICS      0

/**
 * Broadcast support.
 *
 * \hideinitializer
 */
#define UIP_CONF_BROADCAST		 UIP_UDP_ENABLED

/**
 * The maximum amount of concurrent UDP connections.
 *
 * \hideinitializer
 */
#define UIP_CONF_UDP_CONNS		 MAX_UDP_CONNS

#endif /* __UIP_CONF_H__ */

/** @} */
/** @} */
