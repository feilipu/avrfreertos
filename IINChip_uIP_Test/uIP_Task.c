
/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "time.h"
#include "inet.h"

/* serial interface include file. */
#include "serial.h"

/* uIP includes. */
#include "uIP/uip-global-conf.h"
#include "uIP/uipopt.h"
#include "uIP/uip.h"
#include "uIP/apps/tcp-apps.h"
#include "uIP/apps/udp-apps.h"

#include "uIP/network.h"

#if UIP_CONF_IPV6
#include "../lib-uIP/uip-nd6.h"
#endif

const uip_eth_addr  my_eth_addr = { .addr = {UIP_ETHADDR0, UIP_ETHADDR1, UIP_ETHADDR2, UIP_ETHADDR3, UIP_ETHADDR4, UIP_ETHADDR5}};

#ifdef UIP_CONF_EXTERNAL_BUFFER
/* A pointer for the packet buffer that contains incoming packets */
uint8_t * uip_buf; // pvPortMalloc the space on the heap later on.
#else
/* The packet buffer that contains incoming packets. */
uip_buf_t uip_aligned_buf; // for the new union definition.
#endif /* UIP_CONF_EXTERNAL_BUFFER */

/* The start of the uIP buffer, which will contain the frame headers. */
#define pucUIP_Buffer ( ( uip_eth_hdr * ) &uip_buf[ 0 ] )

/* uIP update frequencies. */
#define uipARP_FREQUENCY		( 10 ) // Call arp_timer every 10 seconds
#define uipSELF_ARP_FREQUENCY	( 10 ) // Send a gratuitous arp every 60 seconds

#if UIP_TCP
extern struct uip_tcp_conn *uip_conn;		/* uip_conn always points to the current connection. */
extern struct uip_tcp_conn uip_conns[UIP_TCP_CONNS];	/* The uip_conns array holds all TCP connections. */
#endif /* UIP_TCP */

#if UIP_UDP
extern struct uip_udp_conn *uip_udp_conn; /* uip_udp_conn always points to the current connection. */
extern struct uip_udp_conn uip_udp_conns[UIP_UDP_CONNS];	/* The uip_udp_conns array holds all UDP connections. */
#endif /* UIP_UDP */

/*-----------------------------------------------------------*/

void vuIP_Task( void *pvParameters )
{
#if ! defined(UIP_DHCP)
#if ! UIP_CONF_IPV6
	uip_ipaddr_t ipaddr;
#endif // UIP_CONF_IPV6
#endif // UIP_DHCP

	time_t xStartTime;
	time_t xCurrentTime;

#if ! UIP_CONF_IPV6
	uint8_t xARPTimer;		// maximum timer count is 255 seconds
	uint8_t xSelfARPTimer;	// maximum timer count is 255 seconds
#endif

#ifdef UIP_CONF_EXTERNAL_BUFFER
	// create the working buffer on the heap (so it can be moved later, for example if using Arduino Mega extRAM).
	if(uip_buf == NULL){// if there is no uip_buf buffer allocated (pointer is NULL), then allocate buffer.
		if( ( uip_buf = (uint8_t *) pvPortMalloc( sizeof(uip_buf_t) ) ) == NULL )
			vTaskEndScheduler();
	}
#endif /* UIP_CONF_EXTERNAL_BUFFER */

	/* Initialise the uIP Ethernet MAC address */
	uip_setethaddr(my_eth_addr);

#if ! defined(UIP_DHCP)
#if ! UIP_CONF_IPV6
	uip_ipaddr(&ipaddr, UIP_IPADDR0, UIP_IPADDR1, UIP_IPADDR2, UIP_IPADDR3);
	uip_sethostaddr(&ipaddr);
	uip_ipaddr(&ipaddr, UIP_DRIPADDR0, UIP_DRIPADDR1, UIP_DRIPADDR2, UIP_DRIPADDR3);
	uip_setdraddr(&ipaddr);
	uip_ipaddr(&ipaddr, UIP_NETMASK0, UIP_NETMASK1, UIP_NETMASK2, UIP_NETMASK3);
	uip_setnetmask(&ipaddr);
#endif // UIP_CONF_IPV6
#endif // UIP_DHCP

	/* Initialise the Physical layer, using the uip_eth_addr assigned to the const variable */
	network_init();

	/* Initialise the uIP TCP/IP stack. */
	uip_init();

#if ! UIP_CONF_IPV6
	uip_arp_init();    // must be done or sometimes arp doesn't work
#endif

#if UIP_DNS
	uip_dns_init();
#endif

	/* Initialise the TCP and UDP apps or any other application layer. */
	udp_apps_init();
	tcp_apps_init();

	xSerialxPrint_P(&xSerialPort, PSTR("Initialised vuIP_Task\r\n"));

	/* Initialise the local timers. */
	xStartTime = time(NULL);

#if ! UIP_CONF_IPV6
	xARPTimer = 0;
	xSelfARPTimer = 0;
#endif

	while(1)
	{
		/* Let the network device driver read an entire IP packet
		into the uip_buf. If it returns > 0, there is a packet in the
		uip_buf buffer. */

		uip_len = network_read();

		/* Was a packet placed in the uIP buffer? */
		if( uip_len > 0 )
		{
//			xSerialPrintf_P(PSTR("Frame received %d.\r\n"), uip_len);

#if !UIP_CONF_IPV6
			/* A packet is present in the uIP buffer. We call the
			appropriate ARP functions depending on what kind of packet we
			have received. If the packet is an IP packet, we should call
			uip_input() as well. */
			if( pucUIP_Buffer->type == uip_htons( UIP_ETHTYPE_IP ) )
			{
//				xSerialPrintf_P(PSTR("IPv4 packet received. uip_len %d\r\n"), uip_len);
				uip_arp_ipin();
				uip_input();

				/* If the above function invocation resulted in data that
				should be sent out on the network, the global variable
				uip_len is set to a value > 0. */
				if( uip_len > 0 )
				{
//					xSerialPrintf_P(PSTR("IPv4 packet transmitted. uip_len %d\r\n"), uip_len);
					uip_arp_out();
					network_send();
				}
			}
			else if( pucUIP_Buffer->type == uip_htons( UIP_ETHTYPE_ARP ) )
			{
//				xSerialPrint_P(PSTR("ARP packet received.\r\n"));
				uip_arp_arpin();

				/* If the above function invocation resulted in data that
				should be sent out on the network, the global variable
				uip_len is set to a value > 0. */
				network_send();

			}
#else
			/* A packet is present in the uIP buffer. We call the
			appropriate IPv6 functions depending on what kind of packet we
			have received. */
			if( pucUIP_Buffer->type == uip_htons( UIP_ETHTYPE_IPV6 ) )
			{
//				xSerialPrint_P(PSTR("IPv6 packet received.\r\n"));
				uip_input();

				/* If the above function invocation resulted in data that
				should be sent out on the network, the global variable
				uip_len is set to a value > 0. */
				network_send();

			}
#endif

		}
		else
		{
			/* The poll function returned 0, so no packet was
			received. Instead we check if it is time that we do the
			periodic processing, once per second. */
			xCurrentTime = time(NULL); // time increments once per second, so should always be 0.

			if( ( xCurrentTime - xStartTime ) > 0 )
			{
				/* Reset the timer. */
				xStartTime = xCurrentTime;

//				xSerialPrint((uint8_t *)"IP periodic processing\r\n");

				/* Periodic check of all connections. */
				for(uint8_t i = 0; i < UIP_TCP_CONNS; ++i )
				{
					uip_periodic( i );

					/* If the above function invocation resulted in data that
					should be sent out on the network, the global variable
					uip_len is set to a value > 0. */
#if !UIP_CONF_IPV6
					if( uip_len > 0 )
						uip_arp_out();
#endif
					network_send();

				}

#if UIP_UDP
				for(uint8_t i = 0; i < UIP_UDP_CONNS; ++i )
				{
					uip_udp_periodic( i );

					/* If the above function invocation resulted in data that
					should be sent out on the network, the global variable
					uip_len is set to a value > 0. */
#if !UIP_CONF_IPV6
					if( uip_len > 0 )
						uip_arp_out();
#endif
					network_send();
				}
#endif /* UIP_UDP */

#if !UIP_CONF_IPV6
				/* Periodically call the ARP timer function. */
				if( ++xARPTimer == uipARP_FREQUENCY )
				{
					xARPTimer = 0;
					uip_arp_timer();
				}

				// if nothing to TX and the self ARP timer expired
				// TX a broadcast ARP reply. This was implemented to
				// cause periodic TX to prevent a WiFi AP from disconnecting
				// us from the network.
				if (++xSelfARPTimer == uipSELF_ARP_FREQUENCY)
				{
					xSelfARPTimer = 0;
					uip_self_arp_out();
					network_send();
				}
#endif
			}
		}

//		xSerialPrintf_P(PSTR("uIP HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
//		vTaskDelay( ( 5 / portTICK_RATE_MS ) );
//		xSerialPrintf_P(PSTR("uIP Free Heap Size %u\r\n\n"),xPortGetFreeHeapSize() );
//		vTaskDelay( ( 5 / portTICK_RATE_MS ) );
	}
}

/*---------------------------------------------------------------------------*/

#if defined(UIP_DHCP)

void dhcpc_configured(const struct dhcpc_state *s)
{
    uip_ipaddr_t addr;

    uint8_t _ip_addr[4],_net_mask[4],_gateway[4];

    // byte swap the network info
    _ip_addr[0] = (s->ipaddr.u16[0]);
    _ip_addr[1] = (s->ipaddr.u16[0]) >> 8;
    _ip_addr[2] = (s->ipaddr.u16[1]);
    _ip_addr[3] = (s->ipaddr.u16[1]) >> 8;

    _net_mask[0] = (s->netmask.u16[0]);
    _net_mask[1] = (s->netmask.u16[0]) >> 8;
    _net_mask[2] = (s->netmask.u16[1]);
    _net_mask[3] = (s->netmask.u16[1]) >> 8;

    _gateway[0] = (s->default_router.u16[0]);
    _gateway[1] = (s->default_router.u16[0]) >> 8;
    _gateway[2] = (s->default_router.u16[1]);
    _gateway[3] = (s->default_router.u16[1]) >> 8;

    // re-init just in case
	uip_setethaddr(my_eth_addr);

    // set ip
    uip_ipaddr(&addr, _ip_addr[0], _ip_addr[1], _ip_addr[2], _ip_addr[3]);
    uip_sethostaddr(&addr);

    // set netmask
    uip_ipaddr(&addr,_net_mask[0], _net_mask[1], _net_mask[2], _net_mask[3]);
    uip_setnetmask(&addr);

    // set gateway
    uip_ipaddr(&addr,_gateway[0], _gateway[1], _gateway[2], _gateway[3]);
    uip_setdraddr(&addr);

}

void dhcpc_unconfigured(const struct dhcpc_state *s)
{
    uip_ipaddr_t addr;

    // re-init just in case
	uip_setethaddr(my_eth_addr);

    // set ip
    uip_ipaddr(&addr, 0, 0, 0, 0);
    uip_sethostaddr(&addr);

    // set netmask
    uip_ipaddr(&addr, 0, 0, 0, 0);
    uip_setnetmask(&addr);

    // set gateway
    uip_ipaddr(&addr, 0, 0, 0, 0);
    uip_setdraddr(&addr);
}


#endif // UIP_DHCP


/*-----------------------------------------------------------------------------------*/


#if defined(THIS_IS_FROM_UIPV6)

#include <string.h>

#include <uIP/uip-global-conf.h>
#include <uIP/sys/ttimer.h>

process_event_t tcpip_event;
#if UIP_CONF_ICMP6
process_event_t tcpip_icmp6_event;
#endif /* UIP_CONF_ICMP6 */

/* Periodic check of active connections. */
extern struct ttimer periodic;

#if UIP_CONF_IPV6 && UIP_CONF_IPV6_REASSEMBLY
/* Timer for reassembly. */
extern struct ttimer uip_reass_timer;
#endif

/**
 * \internal Structure for holding a TCP port and a process ID.
 */
struct listenport {
  uint16_t port;
  struct process *p;
};

static struct internal_state {
  struct listenport listenports[UIP_TCP_LISTENPORTS];
  struct process *p;
} s;

enum {
	PROCESS_EVENT_EXITED,
	PROCESS_EVENT_TIMER,
	TCP_POLL,
	UDP_POLL,
	PACKET_INPUT
};

/*---------------------------------------------------------------------------*/
static void
eventhandler(process_event_t ev, process_data_t data)
{
#if UIP_TCP
  static uint8_t i;
  struct listenport *l;
#endif /*UIP_TCP*/
  struct process *p;

  switch(ev) {
    case PROCESS_EVENT_EXITED:
      /* This is the event we get if a process has exited. We go through
         the TCP/IP tables to see if this process had any open
         connections or listening TCP ports. If so, we'll close those
         connections. */

      p = (struct process *)data;
#if UIP_TCP
      l = s.listenports;
      for(i = 0; i < UIP_TCP_LISTENPORTS; ++i) {
        if(l->p == p) {
          uip_unlisten(l->port);
          l->port = 0;
          l->p = PROCESS_NONE;
        }
        ++l;
      }

      {
        struct uip_tcp_conn *cptr;

        for(cptr = &uip_conns[0]; cptr < &uip_conns[UIP_TCP_CONNS]; ++cptr) {
          if(cptr->appstate.p == p) {
            cptr->appstate.p = PROCESS_NONE;
            cptr->tcpstateflags = UIP_CLOSED;
          }
        }
      }
#endif /* UIP_TCP */

#if UIP_UDP
      {
        struct uip_udp_conn *cptr;

        for(cptr = &uip_udp_conns[0];
            cptr < &uip_udp_conns[UIP_UDP_CONNS]; ++cptr) {
          if(cptr->appstate.p == p) {
            cptr->lport = 0;
          }
        }
      }
#endif /* UIP_UDP */
      break;

    case PROCESS_EVENT_TIMER:
      /* We get this event if one of our timers have expired. */
      {
        /* Check the clock so see if we should call the periodic uIP
           processing. */
        if(data == &periodic &&
           timer_expired(&periodic)) {
#if UIP_TCP
          for(i = 0; i < UIP_TCP_CONNS; ++i) {
            if(uip_conn_active(i)) {
              /* Only restart the timer if there are active
                 connections. */
              timer_restart(&periodic);
              uip_periodic(i);
#if UIP_CONF_IPV6
              tcpip_ipv6_output();
#else
              if(uip_len > 0) {
            	xSerialPrintf_P(PSTR("tcpip_output from periodic len %d\n"), uip_len);
                tcpip_output();
                xSerialPrintf_P(PSTR("tcpip_output after periodic len %d\n"), uip_len);
              }
#endif /* UIP_CONF_IPV6 */
            }
          }
#endif /* UIP_TCP */
#if UIP_CONF_IP_FORWARD
          uip_fw_periodic();
#endif /* UIP_CONF_IP_FORWARD */
        }

#if UIP_CONF_IPV6
#if UIP_CONF_IPV6_REASSEMBLY
        /*
         * check the timer for reassembly
         */
        if(data == &uip_reass_timer &&
           etimer_expired(&uip_reass_timer)) {
          uip_reass_over();
          tcpip_ipv6_output();
        }
#endif /* UIP_CONF_IPV6_REASSEMBLY */
        /*
         * check the different timers for neighbour discovery and
         * stateless autoconfiguration
         */
        if(data == &uip_ds6_timer_periodic &&
           timer_expired(&uip_ds6_timer_periodic)) {
          uip_ds6_periodic();
          tcpip_ipv6_output();
        }
#if !UIP_CONF_ROUTER
        if(data == &uip_ds6_timer_rs &&
           timer_expired(&uip_ds6_timer_rs)) {
          uip_ds6_send_rs();
          tcpip_ipv6_output();
        }
#endif /* !UIP_CONF_ROUTER */
        if(data == &uip_ds6_timer_periodic &&
           timer_expired(&uip_ds6_timer_periodic)) {
          uip_ds6_periodic();
          tcpip_ipv6_output();
        }
#endif /* UIP_CONF_IPV6 */
      }
      break;


#if UIP_TCP
    case TCP_POLL:
      if(data != NULL) {
        uip_poll_conn(data);
#if UIP_CONF_IPV6
        tcpip_ipv6_output();
#else /* UIP_CONF_IPV6 */
        if(uip_len > 0)
        {
          xSerialPrintf_P(PSTR("tcpip_output from tcp poll len %d\n"), uip_len);
          tcpip_output();
        }
#endif /* UIP_CONF_IPV6 */
        /* Start the periodic polling, if it isn't already active. */
        start_periodic_tcp_timer();
      }
      break;
#endif /* UIP_TCP */


#if UIP_UDP
    case UDP_POLL:
      if(data != NULL) {
        uip_udp_periodic_conn(data);
#if UIP_CONF_IPV6
        tcpip_ipv6_output();
#else
        if(uip_len > 0)
        {
          xSerialPrintf_P(PSTR("tcpip_output from udp poll len %d\n"), uip_len);
          tcpip_output();
        }
#endif /* UIP_UDP */
      }
      break;
#endif /* UIP_UDP */

    case PACKET_INPUT:
      packet_input();
      break;
  };
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(tcpip_process, ev, data)
{
  PROCESS_BEGIN();

#if UIP_TCP
 {
   for(uint8_t i = 0; i < UIP_TCP_LISTENPORTS; i++)
   {
     s.listenports[i].port = 0;
   }
   s.p = PROCESS_CURRENT();
 }
#endif

  tcpip_event = process_alloc_event();
#if UIP_CONF_ICMP6
  tcpip_icmp6_event = process_alloc_event();
#endif /* UIP_CONF_ICMP6 */
  timer_set(&periodic, CLOCK_SECOND / 2);

  uip_init();
#ifdef UIP_FALLBACK_INTERFACE
  UIP_FALLBACK_INTERFACE.init();
#endif
/* initialize RPL if configured for using RPL */
#if UIP_CONF_IPV6 && UIP_CONF_IPV6_RPL
  rpl_init();
#endif /* UIP_CONF_IPV6_RPL */

  while(1) {
    PROCESS_YIELD();
    eventhandler(ev, data);
  }

  PROCESS_END();
}
#endif

/*---------------------------------------------------------------------------*/


