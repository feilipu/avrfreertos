/*
*
@file		sockutil.c
@brief	Implementation of useful functions for W5100 & W5200
*
*/



#include <stdio.h>
#include <string.h>

#include <avr/io.h>
#include <util/delay.h> // for wait function

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "lib_util.h"  // swapl swaps

#include "serial.h"

#include "socket.h"

#if defined(_WIZCHIP_)		// Definition in freeRTOSBoardDefs.h


#ifndef NO_USE_SOCKUTIL_FUNC
/** This function converts HEX(0-F) to a character */
uint8_t D2C(
	uint8_t c	/**< is a Hex(0x00~0x0F) to convert to a character */
	)
{
	uint16_t t = (uint16_t) c;
	if (t >= 0 && t <= 9)
		return '0' + c;
	if (t >= 10 && t <= 15)
		return 'A' + c - 10;

	return c;
}

/** This function converts a character to HEX(0-F) */
uint8_t C2D(
	uint8_t c	/**< is a character('0'-'F') to convert to HEX */
	)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return 10 + c -'a';
	if (c >= 'A' && c <= 'F')
		return 10 + c -'A';

	return (uint8_t)c;
}


/** CONVERT STRING INTO INTEGER return a integer number */
uint16_t ATOI(
	uint8_t* str,	/**< is a pointer to convert */
	uint16_t base	/**< is a base value (must be in the range 2 - 16) */
	)
{
        uint16_t num = 0;
        while (*str !=0)
                num = num * base + C2D(*str++);
	return num;
}


/**CONVERT STRING INTO HEX OR DECIMAL return	success - 1, fail - 0 */
uint16_t ValidATOI(
	uint8_t* str, 	/**< is a pointer to string to be converted */
	uint16_t base, 	/**< is a base value (must be in the range 2 - 16) */
	uint16_t* ret	/**< is a integer pointer to return */
	)
{
	int c;
	uint8_t* tstr = str;
	if(str == 0 || *str == '\0') return 0;
	while(*tstr != '\0')
	{
		c = C2D(*tstr);
		if( c >= 0 && c < base) tstr++;
		else    return 0;
	}

	*ret = ATOI(str,base);
	return 1;
}



/** replace the specified character in a string with new character */
void replacetochar(
	uint8_t * str, 		/**< pointer to be replaced */
	uint8_t oldchar, 	/**< old character */
	uint8_t newchar	/**< new character */
	)
{
	for (uint16_t x = 0; str[x]; ++x)
		if (str[x] == oldchar) str[x] = newchar;
}
#endif

/////////////////////////////////////////////////////////////////////

/**
@brief	Convert 32bit Address(Host Ordering) into Dotted Decimal Format
@return 	a uint8_t pointer to a static buffer containing the text address in standard ".'' notation. Otherwise, it returns NULL.
*/
uint8_t * inet_ntoa(
	uint32_t addr	/**< Pointer variable to store converted value(INPUT) */
	)
{
	static uint8_t addr_str[16];
	memset(addr_str,0,16);
	sprintf_P((char *)addr_str,PSTR("%d.%d.%d.%d"),(int)(addr>>24 & 0xFF),(int)(addr>>16 & 0xFF),(int)(addr>>8 & 0xFF),(int)(addr & 0xFF));
	return (uint8_t *)addr_str;
}

#ifndef NO_USE_SOCKUTIL_FUNC
uint8_t* inet_ntoa_pad(uint32_t addr)
{
	static uint8_t addr_str[16];
	memset(addr_str,0,16);
	sprintf_P((char *)addr_str,PSTR("%03d.%03d.%03d.%03d"),(int)(addr>>24 & 0xFF),(int)(addr>>16 & 0xFF),(int)(addr>>8 & 0xFF),(int)(addr & 0xFF));
	return addr_str;
}

/**
@brief	Converts a string containing an (Ipv4) Internet Protocol decimal dotted address into a 32bit address
@return 	32bit address (Host Ordering)
*/
uint32_t inet_addr(
	uint8_t* addr	/**< dotted notation address string.  */
	)
{
	uint32_t inetaddr = 0;
	uint8_t taddr[30];
	uint8_t *nexttok;
	int num;
	strlcpy((char *)taddr,(char *)addr, 30);

	nexttok = taddr;
	for(uint8_t i = 0; i < 4 ; ++i)
	{
		nexttok = (uint8_t *)strtok((char *)nexttok,(const char *)".");
		if(nexttok[0] == '0' && nexttok[1] == 'x') num = ATOI(nexttok+2,0x10);
		else num = ATOI(nexttok,10);
		inetaddr = inetaddr << 8;
		inetaddr |= (num & 0xFF);
		nexttok = NULL;
	}
	return inetaddr;
}


/**
@brief	Verify dotted notation IP address string
@return 	success - 1, fail - 0
*/
uint8_t VerifyIPAddress(
	uint8_t* src	/**< pointer to IP address string */
	)
{
	uint16_t tnum;
	uint8_t tsrc[50];
	uint8_t* tok = tsrc;

	strlcpy((char *)tsrc,(char *)src, 50);

	for(uint8_t i = 0; i < 4; ++i)
	{
		tok = (uint8_t *)strtok((char *)tok,(const char *)".");
		if ( !tok ) return 0;
		if(tok[0] == '0' && tok[1] == 'x')
		{
			if(!ValidATOI(tok+2,0x10,&tnum)) return 0;
		}
		else if(!ValidATOI(tok,10,&tnum)) return 0;

		if(tnum < 0 || tnum > 255) return 0;
		tok = NULL;
	}
	return 1;
}


/**
@brief	Output destination IP address of appropriate channel
@return 	32bit destination address (Host Ordering)
*/
uint32_t GetDestAddr(
	SOCKET s	/**< Channel number which try to get destination IP Address */
	)
{
	uint32_t addr;

	addr =               WIZCHIP_read(Sn_DIPR0(s));
	addr = (addr << 8) + WIZCHIP_read(Sn_DIPR1(s));
	addr = (addr << 8) + WIZCHIP_read(Sn_DIPR2(s));
	return (addr << 8) + WIZCHIP_read(Sn_DIPR3(s));
}

/**
@brief	Output destination port number of appropriate channel
@return 	16bit destination port number
*/
uint16_t GetDestPort(
	SOCKET s	/**< Channel number which try to get destination port */
	)
{
	uint16_t port;

	port = WIZCHIP_read( Sn_DPORT0(s) );
	return (port << 8) + WIZCHIP_read( Sn_DPORT1(s));
}
#endif



/**
@brief	htons function converts a uint16_t from host to TCP/IP network byte order (which is big-endian).
@return 	the value in TCP/IP network byte order
*/
uint16_t htons(
	uint16_t hostshort	/**< hostshort  - A 16-bit number in host byte order.  */
	)
{
#if ( SYSTEM_ENDIAN == _ENDIAN_LITTLE_ )
	return swaps(hostshort);
#else
	return hostshort;
#endif
}

/**
@brief	htonl function converts a uint32_t from host to TCP/IP network byte order (which is big-endian).
@return 	the value in TCP/IP network byte order
*/
uint32_t htonl(
	uint32_t hostlong		/**< hostlong  - A 32-bit number in host byte order.  */
	)
{
#if ( SYSTEM_ENDIAN == _ENDIAN_LITTLE_ )
	return swapl(hostlong);
#else
	return hostlong;
#endif
}

/**
@brief	ntohs function converts a uint16_t from TCP/IP network byte order to host byte order (which is little-endian on Intel processors).
@return 	a 16-bit number in host byte order
*/
uint32_t ntohs(
	uint16_t netshort	/**< netshort - network ordering 16bit value */
	)
{
#if ( SYSTEM_ENDIAN == _ENDIAN_LITTLE_ )
	return htons(netshort);
#else
	return netshort;
#endif
}

/**
@brief	converts a uint32_t from TCP/IP network byte order to host byte order (which is little-endian on Intel processors).
@return 	a uint32_t number in host byte order
*/
uint32_t ntohl(uint32_t netlong)
{
#if ( SYSTEM_ENDIAN == _ENDIAN_LITTLE_ )
	return htonl(netlong);
#else
	return netlong;
#endif
}


#ifndef NO_USE_SOCKUTIL_FUNC
// destip : BigEndian
uint8_t CheckDestInLocal(uint32_t destip)
{

	uint8_t * pdestip = (uint8_t*)&destip;

		if((pdestip[0] & WIZCHIP_read(SUBR0)) != (WIZCHIP_read(SIPR0) & WIZCHIP_read(SUBR0)))
			return 1;	// Remote
		if((pdestip[1] & WIZCHIP_read(SUBR1)) != (WIZCHIP_read(SIPR1) & WIZCHIP_read(SUBR1)))
			return 1;	// Remote
		if((pdestip[2] & WIZCHIP_read(SUBR2)) != (WIZCHIP_read(SIPR2) & WIZCHIP_read(SUBR2)))
			return 1;	// Remote
		if((pdestip[3] & WIZCHIP_read(SUBR3)) != (WIZCHIP_read(SIPR3) & WIZCHIP_read(SUBR3)))
			return 1;	// Remote

	return 0;
}


/**
@brief	Get handle of socket which status is same to 'status'
@return 	socket number
*/
SOCKET getSocket(
	uint8_t status, 	/**< socket's status to be found */
	SOCKET start		/**< base of socket to be found */
	)
{
	if(start >= _WIZCHIP_MAX_SOC_NUM_) start = 0;

	for(SOCKET i = start; i < _WIZCHIP_MAX_SOC_NUM_ ; ++i) if( getSn_SR(i) == status ) return i;
	return _WIZCHIP_MAX_SOC_NUM_;
}


/**
@brief	Calculate checksum of a stream
@return 	checksum
*/
uint16_t checksum(
	uint8_t * src, 	/**< pointer to stream  */
	uint16_t len		/**< size of stream */
	)
{
	uint16_t  tsum;
	uint16_t sum, i, j;
	uint32_t lsum;

	j = len >> 1;

	lsum = 0;

	for(i = 0; i < j; ++i)
	{
		tsum = src[i * 2];
		tsum = tsum << 8;
		tsum += src[i * 2 + 1];
		lsum += tsum;
	}

	if (len % 2)
	{
		tsum = src[i * 2];
		lsum += (tsum << 8);
	}


	sum = lsum;
	sum = ~(sum + (lsum >> 16));
	return (uint16_t) sum;
}


/**
@brief	Get Source IP Address of W5100/W5200.
@return 	Source IP Address(32bit Address-Host Ordering)
*/
uint32_t GetIPAddress(void)
{
	uint32_t ip;

	ip =               WIZCHIP_read(SIPR0);
	ip =   (ip << 8) + WIZCHIP_read(SIPR1);
	ip =   (ip << 8) + WIZCHIP_read(SIPR2);
	return (ip << 8) + WIZCHIP_read(SIPR3);

}


/**
@brief	Get Gateway IP Address of W5100/W5200.
@return 	Gateway IP Address(32bit Address-Host Ordering)
*/
uint32_t GetGWAddress(void)
{
	uint32_t ip;

	ip =               WIZCHIP_read(GAR0);
	ip =   (ip << 8) + WIZCHIP_read(GAR1);
	ip =   (ip << 8) + WIZCHIP_read(GAR2);
	return (ip << 8) + WIZCHIP_read(GAR3);
}


/**
@brief	Get Subnet mask of W5100/W5200.
@return 	Subnet Mask(32bit Address-Host Ordering)
*/
uint32_t GetSubMask(void)
{
	uint32_t ip=0;

#if   (_WIZCHIP_ <= 5200)
	setSUBR();			// apply subnet mask address - solve Errata 2 & 3 v1.6
#endif

	ip =             WIZCHIP_read(SUBR0);
	ip = (ip << 8) + WIZCHIP_read(SUBR1);
	ip = (ip << 8) + WIZCHIP_read(SUBR2);
	ip = (ip << 8) + WIZCHIP_read(SUBR3);

#if   (_WIZCHIP_ <= 5200)
	clearSUBR();			// clear subnet mask address - solve Errata 2 & 3 v1.6
#endif

	return ip;
}


/**
@brief	Get Mac Address of W5100/W5200.
@return 	Subnet Mask(32bit Address-Host Ordering)
*/
void GetMacAddress(
	uint8_t* mac	/**< Pointer to store Mac Address (48bit Address)(INPUT, OUTPUT) */
	)
{
	*mac++ = WIZCHIP_read(SHAR0);
	*mac++ = WIZCHIP_read(SHAR1);
	*mac++ = WIZCHIP_read(SHAR2);
	*mac++ = WIZCHIP_read(SHAR3);
	*mac++ = WIZCHIP_read(SHAR4);
	*mac   = WIZCHIP_read(SHAR5);
}

void GetDestMacAddr(SOCKET s, uint8_t* mac)
{
	*mac++ = WIZCHIP_read(Sn_DHAR0(s));
	*mac++ = WIZCHIP_read(Sn_DHAR1(s));
	*mac++ = WIZCHIP_read(Sn_DHAR2(s));
	*mac++ = WIZCHIP_read(Sn_DHAR3(s));
	*mac++ = WIZCHIP_read(Sn_DHAR4(s));
	*mac   = WIZCHIP_read(Sn_DHAR5(s));
}


/**
@brief	Read established network information(G/W, IP, S/N, Mac) of W5100 and Output that through Serial.
		Mac Address is output into format of Dotted HexaDecimal. Others are output into format of Dotted Decimal Format.
*/
void GetNetConfig(void)
{
	uint8_t addr[6];
	uint32_t iaddr;
	xSerialPrint_P(PSTR("\r\n================================================\r\n"));
	xSerialPrint_P(PSTR("Network Configuration Information\r\n"));
	xSerialPrint_P(PSTR("================================================\r\n"));
	_delay_ms(8);
	GetMacAddress(addr);
	xSerialPrintf_P(PSTR("MAC ADDRESS      : 0x%02X.0x%02X.0x%02X.0x%02X.0x%02X.0x%02X\r\n"),addr[0],addr[1],addr[2],addr[3],addr[4],addr[5]);
	iaddr = GetSubMask();
	xSerialPrintf_P(PSTR("SUBNET MASK      : %s\r\n"),inet_ntoa(iaddr));
	iaddr = GetGWAddress();
	xSerialPrintf_P(PSTR("G/W IP ADDRESS   : %s\r\n"),inet_ntoa(iaddr));
	iaddr = GetIPAddress();
	xSerialPrintf_P(PSTR("LOCAL IP ADDRESS : %s\r\n"),inet_ntoa(iaddr));
	xSerialPrint_P(PSTR("================================================\r\n\n"));
}


#endif

#endif // #if   defined(_WIZCHIP_)		// Definition in freeRTOSBoardDefs.h
