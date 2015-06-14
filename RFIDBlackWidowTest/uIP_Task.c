
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


/* serial interface include file. */
//#include "lib_serial.h"

/* uIP includes. */
#undef UIP_HTONS

#include "avr-uIP/global-conf.h"
#include "avr-uIP/uip_arp.h"
#include "avr-uIP/network.h"
#include "avr-uIP/g2100.h"

#include "avr-uIP/apps-conf.h"

// TCP/IP parameters in data memory
uint8_t _enable_dhcp;

// setup the wireless mode; infrastructure - connect to AP; adhoc - connect to another WiFi device
// WIRELESS_MODE_INFRA or WIRELESS_MODE_ADHOC
uint8_t wireless_mode = WIRELESS_MODE_ADHOC;

// Establish SSID,
uint8_t ssid[] = {"pwnU"}; //  max 32 characters

uint8_t security_type = 0;               // 0 - open; 1 - WEP; 2 - WPA; 3 - WPA2

// WEP 128-bit keys
const uint8_t PROGMEM wep_keys[] = {
	0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d,	// Key 0
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	// Key 1
	// (could define up to 4 keys, but only use 2)
};

// WPA/WPA2 passphrase
const uint8_t PROGMEM security_passphrase[] = {"pwnU2"};	// max 64 characters

//EEPROM parameters (TCP/IP parameters)
uint8_t EEMEM ee_enable_dhcp=USE_DHCP;

uint8_t ssid_len;
uint8_t security_passphrase_len;

// signal the vuIP_TASK to resume execution.
SemaphoreHandle_t xZGIntrSemaphore;

static struct uip_eth_addr  my_eth_addr = { .addr = {UIP_ETHADDR0,UIP_ETHADDR1,UIP_ETHADDR2,UIP_ETHADDR3,UIP_ETHADDR4,UIP_ETHADDR5}};

#ifdef UIP_CONF_EXTERNAL_BUFFER
uint8_t *uip_buf;
#else
uint8_t uip_buf[UIP_BUFSIZE+2];
#endif /* UIP_CONF_EXTERNAL_BUFFER */

/* The start of the uIP buffer, which will contain the frame headers. */
#define pucUIP_Buffer ( ( struct uip_eth_hdr * ) &uip_buf[ 0 ] )

/* uIP update frequencies. */
#define RT_CLOCK_SECOND			( configTICK_RATE_HZ  )
#define uipARP_FREQUENCY		( 10 ) // Call arp_timer every 10 seconds
#define uipSELF_ARP_FREQUENCY	( 30 ) // Send a gratuitous arp every 30 seconds
#define uipMAX_BLOCK_TIME		( RT_CLOCK_SECOND / 4 )

/*-----------------------------------------------------------*/

void vuIP_Task( void *pvParameters )
{

	static volatile TickType_t xStartTime, xCurrentTime;
	portBASE_TYPE xARPTimer, xSelfARPTimer;

	uip_ipaddr_t ipaddr;

#ifdef UIP_CONF_EXTERNAL_BUFFER
	// create the working buffer on the heap (so it can be moved later, if using Arduino Mega extRAM).
	if(uip_buf == NULL){// if there is no uip_buf buffer allocated (pointer is NULL), then allocate buffer.
		if( (uip_buf = pvPortMalloc( sizeof(uint8_t) * (UIP_BUFSIZE + 2) )) == NULL )
			{vTaskEndScheduler();}
	}
#endif /* UIP_CONF_EXTERNAL_BUFFER */

	_enable_dhcp = eeprom_read_byte(&ee_enable_dhcp);
	if ((_enable_dhcp != 1) && (_enable_dhcp != 0))
	{   // if the setting is invalid, disable by default
		_enable_dhcp = 0;
		eeprom_write_byte(&ee_enable_dhcp,_enable_dhcp);
	}

	uip_setethaddr(my_eth_addr);

//_enable_dhcp = 1;
	if (_enable_dhcp)
	{
#ifdef __DHCPC_H__
		// setup the dhcp renew timer the make the first request
		dhcpc_init(&my_eth_addr, 6);
		dhcpc_request();
#endif
	}
	else
	{
		uip_ipaddr(ipaddr, UIP_IPADDR0, UIP_IPADDR1, UIP_IPADDR2, UIP_IPADDR3);
		uip_sethostaddr(ipaddr);
		uip_ipaddr(ipaddr, UIP_DRIPADDR0, UIP_DRIPADDR1, UIP_DRIPADDR2, UIP_DRIPADDR3);
		uip_setdraddr(ipaddr);
		uip_ipaddr(ipaddr, UIP_NETMASK0, UIP_NETMASK1, UIP_NETMASK2, UIP_NETMASK3);
		uip_setnetmask(ipaddr);
	}

	/* Initialize the uIP TCP/IP stack. */
	network_init();

	/* Initialize the HTTP server or any other application layer. */
	//httpd_init();
	simple_httpd_init();

	/* Initialise the local timers. */
	xStartTime = xTaskGetTickCount();
	xARPTimer = 0;
	xSelfARPTimer = 0;


	while(1)
	{
		/* Let the network device driver read an entire IP packet
		into the uip_buf. If it returns > 0, there is a packet in the
		uip_buf buffer. */

		uip_len = network_read();

		/* Was a packet placed in the uIP buffer? */
		if( uip_len > 0 )
		{
			/* A packet is present in the uIP buffer. We call the
			appropriate ARP functions depending on what kind of packet we
			have received. If the packet is an IP packet, we should call
			uip_input() as well. */
			if( pucUIP_Buffer->type == uip_htons( UIP_ETHTYPE_IP ) )
			{
				uip_arp_ipin();
				uip_input();

				/* If the above function invocation resulted in data that
				should be sent out on the network, the global variable
				uip_len is set to a value > 0. */
				if( uip_len > 0 )
				{
					uip_arp_out();
					network_send();
				}
			}
			else if( pucUIP_Buffer->type == uip_htons( UIP_ETHTYPE_ARP ) )
			{
				uip_arp_arpin();

				/* If the above function invocation resulted in data that
				should be sent out on the network, the global variable
				uip_len is set to a value > 0. */
				if( uip_len > 0 )
				{
					network_send();
				}
			}
		}
		else
		{
			/* The poll function returned 0, so no packet was
			received. Instead we check if it is time that we do the
			periodic processing. */
			xCurrentTime = xTaskGetTickCount();

			if( ( xCurrentTime - xStartTime ) >= RT_CLOCK_SECOND )
			{
				uint8_t i;

				/* Reset the timer. */
				xStartTime = xCurrentTime;

				/* Periodic check of all connections. */
				for( i = 0; i < UIP_CONNS; i++ )
				{
					uip_periodic( i );

					/* If the above function invocation resulted in data that
					should be sent out on the network, the global variable
					uip_len is set to a value > 0. */
					if( uip_len > 0 )
					{
						uip_arp_out();
						network_send();
					}
				}

				#if UIP_UDP
					for( i = 0; i < UIP_UDP_CONNS; i++ )
					{
						uip_udp_periodic( i );

						/* If the above function invocation resulted in data that
						should be sent out on the network, the global variable
						uip_len is set to a value > 0. */
						if( uip_len > 0 )
						{
							uip_arp_out();
							network_send();
						}
					}
				#endif /* UIP_UDP */

				/* Periodically call the ARP timer function. */
				if( ++xARPTimer == uipARP_FREQUENCY )
				{
					xARPTimer = 0;
					uip_arp_timer();
				}

				// if nothing to TX and the self ARP timer expired
				// TX a broadcast ARP reply. This was implemented to
				// cause periodic TX to prevent the AP from disconnecting
				// us from the network
				if (++xSelfARPTimer == uipSELF_ARP_FREQUENCY)
				{
					xSelfARPTimer = 0;
					uip_self_arp_out();
					network_send();
				}
			}
			else
			{
				/* We did not receive a packet, and there was no periodic
				processing to perform.  Block for a fixed period.  If a packet
				is received during this period we will be woken by the ISR
				giving us the Semaphore. */
				xSemaphoreTake( xZGIntrSemaphore, uipMAX_BLOCK_TIME );
			}
		}

		zg_drv_process();

//		xSerialPrintf_P(PSTR("uIP HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
//		vTaskDelay( ( 40 / portTICK_RATE_MS ) );
//		xSerialPrintf_P(PSTR("uIP Free Heap Size %u\r\n\n"),xPortGetFreeHeapSize() );
//		vTaskDelay( ( 40 / portTICK_RATE_MS ) );
	}
}
/*-----------------------------------------------------------------------------------*/


#ifdef __DHCPC_H__
void dhcpc_configured(const struct dhcpc_state *s)
{
    uip_ipaddr_t addr;

    // byte swap the network info
    _ip_addr[0] = (s->ipaddr[0]);
    _ip_addr[1] = (s->ipaddr[0]) >> 8;
    _ip_addr[2] = (s->ipaddr[1]);
    _ip_addr[3] = (s->ipaddr[1]) >> 8;

    _net_mask[0] = (s->netmask[0]);
    _net_mask[1] = (s->netmask[0]) >> 8;
    _net_mask[2] = (s->netmask[1]);
    _net_mask[3] = (s->netmask[1]) >> 8;

    _gateway[0] = (s->default_router[0]);
    _gateway[1] = (s->default_router[0]) >> 8;
    _gateway[2] = (s->default_router[1]);
    _gateway[3] = (s->default_router[1]) >> 8;

#ifdef DHCP_DEBUG
    eeprom_write_block (_ip_addr, &ee_ip_addr, 4);
    eeprom_write_block (_net_mask,&ee_net_mask,4);
    eeprom_write_block (_gateway, &ee_gateway, 4);
#endif
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

//  code to use dhcp server lease time removed due to uint16_t overflow
//  issues with calculating the time.  Just use 5 minutes instead.
    timer_set(&dhcp_timer, 5 * 60 * CLOCK_SECOND);

#ifdef DHCP_DEBUG
    // for now turn on the led when we get an ip
    led_high();
#endif
}
#endif

/*---------------------------------------------------------------------------*/


