/**
 * @file	ntp.c
 * @brief 	functions relevant to ntp
 */

#include <string.h>
#include <util/delay.h>

#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "serial.h"

#include "time.h"
#include "rtc.h"

#include "inet.h"

#if defined(_WIZCHIP_)		// Definition in freeRTOSBoardDefs.h

//----------------------------------------------------------------------------


static SOCKET NTPC_SOCK;					// Socket for the NTP client
static NTP_MSG *pNTPMSG;					// Pointer for the NTP message
static time_t reference_timestamp_sec;		// Time stamp of last NTP update in Y2K epoch

//----------------------------------------------------------------------------

static void send_NTP_request (SOCKET s);				// Request the network time from a NTP server
static uint8_t process_NTP(SOCKET s, uint16_t length);	// Process a NTP response

//----------------------------------------------------------------------------
/**
 *	Initialise the NTP daemon.
 */
uint8_t init_NTP (SOCKET s)
{

	if( getSn_SR(s) != SOCK_CLOSED )	// Check the preferred socket is available,
	{
		s = getSocket(SOCK_CLOSED, 0);	// otherwise find free socket,
		if(s == _WIZCHIP_MAX_SOC_NUM_ )        	// If there is no free socket?
			return 0;
	}

	xSerialPrintf_P(PSTR("\r\nNTP socket: %d,"),s);
	if(!socket(s, Sn_MR_UDP, IP_PORT_NTP, 0x00)) // initialise the socket for NTP service
	{
		xSerialPrint_P(PSTR(" fail..!\r\n"));
		return 0;
	}
	else
	{
		xSerialPrint_P(PSTR(" ok..!\r\n"));
	}

	if(pNTPMSG == NULL) // if there is no buffer allocated (pointer is NULL), then allocate buffer for all NTP functions.
	{
		if( !(pNTPMSG = (NTP_MSG *) pvPortMalloc( sizeof(NTP_MSG) ) ) )
			return 0;
	}

	NTPC_SOCK = s;

	return 1;
}

//----------------------------------------------------------------------------
/**
 *	Request the socket assigned to the NTP client
 */

SOCKET	get_NTP_socket(void)				// Get the socket assigned for NTP
{
	return NTPC_SOCK;
}

//----------------------------------------------------------------------------
/**
 *	Request the reference time, the last time that the NTP server responded
 */

uint32_t get_NTP_reference_time(void)			// Get the last reference time in Y2K epoch
{
	return reference_timestamp_sec;
}

//----------------------------------------------------------------------------
/**
 *	Check the current system_time from a NTP Server
 */

void	check_NTP (SOCKET s)				// Check system_time against NTP time
{
	uint16_t length;
	TickType_t xInitialTick;

	xInitialTick = xTaskGetTickCount();

	send_NTP_request(s);

	while( xTaskGetTickCount() < (xInitialTick + ( NTP_TIMEOUT / portTICK_PERIOD_MS )) )	// as long as time is remaining
	{

		if((length = getSn_RX_RSR(s)) > 0)		// Has NTP socket received a packet?
		{
			process_NTP (s, length);
			return;
		}
	}

}

//----------------------------------------------------------------------------
/**
 *	Request the current time from a NTP Server
 */
static void send_NTP_request (SOCKET s)
{
	un_l2cval ip;

	memset((void*)pNTPMSG,0,sizeof(NTP_MSG));

	pNTPMSG->li_vn_mode = 0b11100011;	// Leap Indicator 0b11; Protocol Version Number 0b100; Mode 0b011
	pNTPMSG->stratum = 15;				// Stratum 16 is the worst.
	pNTPMSG->poll = 10;
	pNTPMSG->precision = 0;
	// 8 bytes of zero for Root Delay & Root Dispersion
	pNTPMSG->reference_timestamp_sec = htonl(reference_timestamp_sec);	// time stamp of last system_time update
	pNTPMSG->origin_timestamp_sec = htonl(time(NULL) + NTP_OFFSET);		// system_time including the offset to NTP epoch.

	ip.lVal = htonl( inet_addr( (uint8_t*) NTP_SERVER_4));				// the NTP server we're using.

	if(0 == sendto(s, (uint8_t*)pNTPMSG, sizeof(NTP_MSG), ip.cVal, IP_PORT_NTP))
	{
		xSerialPrint_P(PSTR("\r\nNTP: Fatal Error."));
		return;
	}
	xSerialPrint_P(PSTR("\r\nsent NTP_REQUEST"));
}

//----------------------------------------------------------------------------
/**
 * Process the received NTP packet, and update the system timer.
 */
static uint8_t process_NTP (
	SOCKET s, 	/**< socket number */
	uint16_t length	/**< a size data to receive. */
	)
{

	un_l2cval svr_addr;
	uint16_t  svr_port;



	if ( recvfrom(s, (uint8_t *)pNTPMSG, length, svr_addr.cVal, &svr_port) )
	{
		if(pNTPMSG->transmit_timestamp_sec != 0)
		{
			set_system_time( reference_timestamp_sec = (ntohl(pNTPMSG->transmit_timestamp_sec) - NTP_OFFSET) );

#ifdef portRTC_DEFINED
			if (setDateTimeDS1307( localtime( (const time_t *)&reference_timestamp_sec) ) == pdTRUE)
				xSerialPrint_P( PSTR("\r\nRTC Setting successful.") );
#endif
			xSerialPrintf_P(PSTR("\r\nNTP message received... 0x%.8x"), reference_timestamp_sec );
			return 1;
		}
		else
			return 0;
	}
	else
		return 0;
}

#endif // #if   defined(_WIZCHIP_)		// Definition in freeRTOSBoardDefs.h
