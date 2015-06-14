
/******************************************************************************

  Filename:		network.h
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

#ifndef __NETWORK_H__
#define __NETWORK_H__

/* Initialise the network */
void network_init(void);

/* Read from the network, returns number of read bytes */
uint16_t network_read(void);

/* Send using the network */
void network_send(void);

/* Sets the MAC address of the device */
void network_set_MAC(uint8_t* mac);

/* Gets the MAC address of the device */
void network_get_MAC(uint8_t* mac);

#endif /* __NETWORK_H__ */
