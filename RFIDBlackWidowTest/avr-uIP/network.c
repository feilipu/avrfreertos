
/******************************************************************************

  Filename:		network.c
  Description:	Network interface for the WiShield 1.0

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
   AsyncLabs			05/29/2009	Initial version

 *****************************************************************************/

#include <string.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#include "uip.h"
#include "uip_arp.h"
#include "config.h"
#include "g2100.h"

void network_init(void)
{

	zg_init();

	// set digital pin 2 (INT0) as ZG interrupt pin
	// OR
	// set digital pin 8 as ZG interrupt pin
	ZG2100_ISR_ENABLE();

	uip_init();
	uip_arp_init();    // must be done or sometimes arp doesn't work


#ifdef UIP_DHCP
	uip_dhcp_init(zg_get_mac(), 6);
#endif

#ifdef UIP_DNS
	uip_dns_init();
#endif

}

unsigned int network_read(void)
{
	return zg_get_rx_status();
}

void network_send(void)
{
	if (uip_len > 0) {
		if(uip_len <= UIP_LLH_LEN + 40){
			zg_set_buf(uip_buf, uip_len);
		}
		else{
			memcpy((uint8_t*)&uip_buf[54], (uint8_t*)uip_appdata, (uip_len-54));
			zg_set_buf(uip_buf, uip_len);
		}
		zg_set_tx_status(1);
	}
}

void network_get_MAC(uint8_t* macaddr)
{
	macaddr = zg_get_mac();
}

void network_set_MAC(uint8_t* macaddr)
{
}


#if defined USE_DIG2_INTR
// INT0 interrupt vector
ISR(INT0_vect)
#elif defined USE_DIG8_INTR
// PCINT0 interrupt vector
ISR(PCINT0_vect)
#endif
{

	zg_isr();

}


