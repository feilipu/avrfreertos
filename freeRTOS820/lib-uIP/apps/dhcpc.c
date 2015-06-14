/*
 * Copyright (c) 2005, Swedish Institute of Computer Science
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <avr/io.h>
#include <avr/pgmspace.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "time.h"
#include "inet.h"
#include "uIP/uip.h"
#include "uIP/psock.h"
#include "uIP/sys/stimer.h"


#include "uIP/apps/udp-apps.h"
#include "uIP/apps/dhcpc.h"


#define DEBUG 1				// for DEBUG purposes, pretty obviously

#if DEBUG
#include <stdio.h>
#include "serial.h"		/* serial interface include file. */
#define PRINTF(...) xSerialxPrintf(&xSerialPort, __VA_ARGS__) // the PRINTF() statements, are realised through xSerialPrintf()
#else
#define PRINTF(...)
#endif

struct dhcp_msg {
  uint8_t op, htype, hlen, hops;
  uint8_t xid[4];
  uint16_t secs, flags;
  uint8_t ciaddr[4];
  uint8_t yiaddr[4];
  uint8_t siaddr[4];
  uint8_t giaddr[4];
  uint8_t chaddr[16];
#ifndef UIP_CONF_DHCP_LIGHT
  uint8_t sname[64];
  uint8_t file[128];
#endif
  uint8_t options[312];
};

#define DHCP_MSG_LEN      236

#define DHCP_OPTION_SUBNET_MASK   1
#define DHCP_OPTION_ROUTER        3
#define DHCP_OPTION_DNS_SERVER    6
#define DHCP_OPTION_REQ_IPADDR   50
#define DHCP_OPTION_LEASE_TIME   51
#define DHCP_OPTION_MSG_TYPE     53
#define DHCP_OPTION_SERVER_ID    54
#define DHCP_OPTION_REQ_LIST     55
#define DHCP_OPTION_END         255

/*---------------------------------------------------------------------------*/

static uint32_t xid;
static const uint8_t magic_cookie[4] = {99, 130, 83, 99};

struct dhcpc_state dhcpc_state; // this is the DHCP state variable.

/*---------------------------------------------------------------------------*/

static uint8_t *
add_msg_type(uint8_t *optptr, uint8_t type)
{
  *optptr++ = DHCP_OPTION_MSG_TYPE;
  *optptr++ = 1;
  *optptr++ = type;
  return optptr;
}
/*---------------------------------------------------------------------------*/

static uint8_t *
add_server_id(uint8_t *optptr)
{
  *optptr++ = DHCP_OPTION_SERVER_ID;
  *optptr++ = 4;
  memcpy(optptr, dhcpc_state.serverid, 4);
  return optptr + 4;
}
/*---------------------------------------------------------------------------*/

static uint8_t *
add_req_ipaddr(uint8_t *optptr)
{
  *optptr++ = DHCP_OPTION_REQ_IPADDR;
  *optptr++ = 4;
  memcpy(optptr, (uint8_t *)(dhcpc_state.ipaddr.u16), 4);
  return optptr + 4;
}
/*---------------------------------------------------------------------------*/

static uint8_t *
add_req_options(uint8_t *optptr)
{
  *optptr++ = DHCP_OPTION_REQ_LIST;
  *optptr++ = 3;
  *optptr++ = DHCP_OPTION_SUBNET_MASK;
  *optptr++ = DHCP_OPTION_ROUTER;
  *optptr++ = DHCP_OPTION_DNS_SERVER;
  return optptr;
}
/*---------------------------------------------------------------------------*/

static uint8_t *
add_end(uint8_t *optptr)
{
  *optptr++ = DHCP_OPTION_END;
  return optptr;
}
/*---------------------------------------------------------------------------*/

static void
create_msg(struct dhcp_msg *m)
{
  m->op = DHCP_BOOTREQUEST;
  m->htype = DHCP_HTYPE_10MB;
  m->hlen = dhcpc_state.mac_len;
  m->hops = 0;
  memcpy(m->xid, &xid, sizeof(m->xid));
  m->secs = 0;
  m->flags = UIP_HTONS(BOOTP_BROADCAST); /*  Broadcast bit. */
  /*  uip_ipaddr_copy(m->ciaddr, uip_hostaddr);*/
  memcpy(m->ciaddr, uip_hostaddr.u16, sizeof(m->ciaddr));
  memset(m->yiaddr, 0, sizeof(m->yiaddr));
  memset(m->siaddr, 0, sizeof(m->siaddr));
  memset(m->giaddr, 0, sizeof(m->giaddr));
  memcpy(m->chaddr, dhcpc_state.mac_addr, dhcpc_state.mac_len);
  memset(&m->chaddr[dhcpc_state.mac_len], 0, sizeof(m->chaddr) - dhcpc_state.mac_len);
#ifndef UIP_CONF_DHCP_LIGHT
  memset(m->sname, 0, sizeof(m->sname));
  memset(m->file, 0, sizeof(m->file));
#endif
  memcpy(m->options, magic_cookie, sizeof(magic_cookie));
}
/*---------------------------------------------------------------------------*/

static void
send_discover(void)
{
  uint8_t *end;
  struct dhcp_msg *m = (struct dhcp_msg *)uip_appdata;

  create_msg(m);

  end = add_msg_type(&m->options[4], DHCP_DISCOVER);
  end = add_req_options(end);
  end = add_end(end);

  uip_send(uip_appdata, (int)(end - (uint8_t *)uip_appdata));
}
/*---------------------------------------------------------------------------*/

static void
send_request(void)
{
  uint8_t *end;
  struct dhcp_msg *m = (struct dhcp_msg *)uip_appdata;

  create_msg(m);

  end = add_msg_type(&m->options[4], DHCP_REQUEST);
  end = add_server_id(end);
  end = add_req_ipaddr(end);
  end = add_end(end);

  uip_send(uip_appdata, (int)(end - (uint8_t *)uip_appdata));
}
/*---------------------------------------------------------------------------*/

static uint8_t
parse_options(uint8_t *optptr, int len)
{
  uint8_t *end = optptr + len;
  uint8_t type = 0;

  while(optptr < end) {
    switch(*optptr) {
    case DHCP_OPTION_SUBNET_MASK:
      memcpy(dhcpc_state.netmask.u16, optptr + 2, 4);
      break;
    case DHCP_OPTION_ROUTER:
      memcpy(dhcpc_state.default_router.u16, optptr + 2, 4);
      break;
    case DHCP_OPTION_DNS_SERVER:
      memcpy(dhcpc_state.dnsaddr.u16, optptr + 2, 4);
      break;
    case DHCP_OPTION_MSG_TYPE:
      type = *(optptr + 2);
      break;
    case DHCP_OPTION_SERVER_ID:
      memcpy(dhcpc_state.serverid, optptr + 2, 4);
      break;
    case DHCP_OPTION_LEASE_TIME:
      memcpy(dhcpc_state.lease_time, optptr + 2, 4);
      break;
    case DHCP_OPTION_END:
      return type;
    }

    optptr += optptr[1] + 2;
  }
  return type;
}
/*---------------------------------------------------------------------------*/

static uint8_t
parse_msg(void)
{
  struct dhcp_msg *m = (struct dhcp_msg *)uip_appdata;

  if(m->op == DHCP_BOOTREPLY &&
     memcmp(m->xid, &xid, sizeof(xid)) == 0 &&
     memcmp(m->chaddr, dhcpc_state.mac_addr, dhcpc_state.mac_len) == 0) {
    memcpy(dhcpc_state.ipaddr.u16, m->yiaddr, 4);
    return parse_options(&m->options[4], uip_datalen());
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
/*
 * Is this a "fresh" reply for me? If it is, return the type.
 */

static int8_t
msg_for_me(void)
{
  struct dhcp_msg *m = (struct dhcp_msg *)uip_appdata;
  uint8_t *optptr = &m->options[4];
  uint8_t *end = (uint8_t*)uip_appdata + uip_datalen();

  if(m->op == DHCP_BOOTREPLY &&
     memcmp(m->xid, &xid, sizeof(xid)) == 0 &&
     memcmp(m->chaddr, dhcpc_state.mac_addr, dhcpc_state.mac_len) == 0) {
    while(optptr < end) {
      if(*optptr == DHCP_OPTION_MSG_TYPE) {
	return *(optptr + 2);
      } else if (*optptr == DHCP_OPTION_END) {
	return -1;
      }
      optptr += optptr[1] + 2;
    }
  }
  return -1;
}
/*---------------------------------------------------------------------------*/
/*
#define DHCP_STATE_INITIAL      0
#define	DHCP_STATE_DISCOVER		1
#define	DHCP_STATE_REQUEST		2
#define	DHCP_STATE_LEASED		3
#define	DHCP_STATE_REREQUEST	4
#define	DHCP_STATE_RELEASE		5
*/

static void
handle_dhcp(process_event_t message)
{
	time_t seconds;

	switch( dhcpc_state.state )
	{
	case DHCP_STATE_INITIAL:

		dhcpc_state.state = DHCP_STATE_DISCOVER;
		xid++;

		send_discover();

		stimer_set(&dhcpc_state.stimer, (time_t)10 );	// normally set the timer for 60 seconds,
														// but because the ARP table is empty give DHCP server only 10 seconds.
		break;

	case DHCP_STATE_DISCOVER:

		if( !stimer_expired(&dhcpc_state.stimer) && message == DHCP_OFFER )
		{
			parse_msg();

			dhcpc_state.state = DHCP_STATE_REQUEST;
			xid++;

			send_request();

			stimer_set(&dhcpc_state.stimer, (time_t)10 ); // set the timer for 10 seconds

		} else {
			dhcpc_state.state = DHCP_STATE_INITIAL;
		}
		break;

	case DHCP_STATE_REQUEST:

		if( !stimer_expired(&dhcpc_state.stimer) && message == DHCP_ACK )
		{
			parse_msg();
			dhcpc_state.state = DHCP_STATE_LEASED;

			PRINTF("Got IP address %d.%d.%d.%d\n", uip_ipaddr_to_quad(&dhcpc_state.ipaddr));
			PRINTF("Got netmask %d.%d.%d.%d\n", uip_ipaddr_to_quad(&dhcpc_state.netmask));
			PRINTF("Got DNS server %d.%d.%d.%d\n", uip_ipaddr_to_quad(&dhcpc_state.dnsaddr));
			PRINTF("Got default router %d.%d.%d.%d\n", uip_ipaddr_to_quad(&dhcpc_state.default_router));
			PRINTF("Lease expires in %ld seconds\n", uip_ntohs(dhcpc_state.lease_time[0])*65536ul + uip_ntohs(dhcpc_state.lease_time[1]));

			dhcpc_configured(&dhcpc_state);

			#define MAX_TICKS (~((time_t)0) / 2)
			#define MAX_TICKS32 (~((time_t)0))

			if((uip_ntohs(dhcpc_state.lease_time[0])*65536ul + uip_ntohs(dhcpc_state.lease_time[1]))/2 <= MAX_TICKS32)
			{
				seconds = ( uip_ntohs(dhcpc_state.lease_time[0])*65536ul + uip_ntohs(dhcpc_state.lease_time[1]) )/2;
			} else {
				seconds = MAX_TICKS32;
			}

			stimer_set(&dhcpc_state.stimer, seconds); // set the timer for half of lease_time seconds

		} else {
			dhcpc_state.state = DHCP_STATE_INITIAL;
		}
		break;

	case DHCP_STATE_LEASED:
		if( stimer_expired(&dhcpc_state.stimer) )
		{
			dhcpc_state.state = DHCP_STATE_INITIAL;
		}
		break;

	case DHCP_STATE_REREQUEST:
	case DHCP_STATE_RELEASE:
	default:
		dhcpc_state.state = DHCP_STATE_INITIAL;
		break;
	}
}

/*---------------------------------------------------------------------------*/

void
dhcpc_init(const void *mac_addr, uint8_t mac_len)
{
  uip_ipaddr_t addr;

  dhcpc_state.mac_addr = mac_addr;
  dhcpc_state.mac_len  = mac_len;

  dhcpc_state.state = DHCP_STATE_INITIAL;
  uip_ipaddr(&addr, 255,255,255,255);
  dhcpc_state.conn = uip_udp_new(&addr, UIP_HTONS(IP_PORT_DHCP_SERVER));
  if(dhcpc_state.conn != NULL)
  {
    uip_udp_bind(dhcpc_state.conn, UIP_HTONS(IP_PORT_DHCP_CLIENT));
  }

}

/*---------------------------------------------------------------------------*/

void
dhcpc_request(void)
{
  uip_ipaddr_t ipaddr;

  if(dhcpc_state.state == DHCP_STATE_INITIAL) {
    uip_ipaddr(&ipaddr, 0,0,0,0);
    uip_sethostaddr(&ipaddr);
    handle_dhcp(0);
  }
}

/*---------------------------------------------------------------------------*/

/*
#define	DHCP_DISCOVER			1
#define DHCP_OFFER				2
#define	DHCP_REQUEST			3
#define	DHCP_DECLINE			4
#define	DHCP_ACK				5
#define DHCP_NAK				6
#define	DHCP_RELEASE			7
#define DHCP_INFORM				8
*/
void
dhcpc_appcall(void)
{
	int8_t message;

//	PRINTF("dhcpc_appcall() %d\r\n", dhcpc_state.state); // just for debugging

	if ( uip_newdata() && (message = msg_for_me()) > 0 )
	{
		handle_dhcp( (uint8_t) message );
	}

	if(dhcpc_state.state == DHCP_STATE_INITIAL || dhcpc_state.state == DHCP_STATE_LEASED)
	{
		handle_dhcp(0); // no message if DHCP_STATE_INITIAL or DHCP_STATE_LEASED, but we need to handle timer and initialisation.
	}
}
/*---------------------------------------------------------------------------*/
