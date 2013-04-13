
/******************************************************************************

  Filename:		network.h
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

#ifndef __NETWORK_H__
#define __NETWORK_H__

/*Initialize the network*/
void network_init(void);

/*Read from the network, returns number of read bytes*/
unsigned int network_read(void);

/*Send using the network*/
void network_send(void);

/*Sets the MAC address of the device*/
void network_set_MAC(uint8_t* mac);

/*Gets the MAC address of the device*/
void network_get_MAC(uint8_t* mac);

#endif /* __NETWORK_H__ */
