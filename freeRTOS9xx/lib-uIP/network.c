
/******************************************************************************

  Filename:		network.c
  Description:	Network interface for the IINCHIP W5x00
 ******************************************************************************

  TCP/IP stack and driver for the Wiznet IINCHIP W5x00 devices

  This program is free software; you can redistribute it and/or modify it
  under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

 *****************************************************************************/

#include <string.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

/* Scheduler include files. */

#include "FreeRTOS.h"
#include "FreeRTOSBoardDefs.h"

#include "socket.h"

#ifdef __DEF_WIZCHIP_DBG__
/* serial interface include file. */
#include "serial.h"
#endif

#include "uIP/uip-global-conf.h"
#include "uIP/uip-arp.h"
#include "uIP/network.h"

#if defined(_WIZCHIP_)		// Definition in freeRTOSBoardDefs.h

void network_init(void)
{
	/*-----------------------------------------------------------*/
	/* Network related stuff */

#if (_WIZCHIP_ == 5100)
	WIZCHIP_init(); 										// reset W5200 - First call to make
	WIZCHIP_sysinit(0x55, 0x55);
#else
	WIZCHIP_init(); 										// reset W5200/W5500 - First call to make
	// TX MEM SIZE: Permissible values: 1:(1024) 2:(2048) 4:(4096) 8:(8192) 16:(16384)
	// RX MEM SIZE: Total for 8 ports shall equal not more than 16
	// Initialised in WIZCHIP_init() in wizchip_conf.c
	extern uint8_t txsize[_WIZCHIP_MAX_SOC_NUM_];
	extern uint8_t rxsize[_WIZCHIP_MAX_SOC_NUM_];

    WIZCHIP_sysinit ((uint8_t *)txsize, (uint8_t *)rxsize);	// set the Tx and Rx buffers
#endif

#if (_WIZCHIP_ != 5500) // xxx the Wiz550io has its MAC address loaded by the onboard MCU.
	extern uip_eth_addr  my_eth_addr; // the Ethernet address assigned in the application
	setSHAR(my_eth_addr.addr);
#endif

	if (!socket( MACRAW_SOCKET, Sn_MR_MACRAW, 0x00, 0x00) ) // Opens the MAC raw mode socket.
	{
#ifdef __DEF_WIZCHIP_DBG__
		xSerialPrint_P(PSTR("uIP MACRAW socket: 0, initialise fail..!\r\n"));
#endif
		return;

	}

}

uint16_t network_read(void)
{
	uint16_t len;
	if( (len = getSn_RX_RSR(MACRAW_SOCKET)) > 0 )		// Has MACRAW_SOCKET socket 0 received a packet?
		return macraw_recv( (uint8_t*)uip_buf, len);
	else
		return 0;
}

void network_send(void)
{
	if (uip_len > 0)
	{
		if( !macraw_send( (uint8_t*)uip_buf, uip_len) )
		{
#ifdef __DEF_WIZCHIP_DBG__
			xSerialPrint_P(PSTR("\r\nuIP MACRAW: Fatal Error(0)."));
#endif
		}
	}
}

void network_get_MAC(uint8_t* macaddr)
{
	getSHAR( macaddr); // get the local MAC address
}

void network_set_MAC(uint8_t* macaddr)
{
	setSHAR( macaddr ); // set local MAC address
}

#endif // #if defined(_WIZCHIP_)		// Definition in freeRTOSBoardDefs.h
