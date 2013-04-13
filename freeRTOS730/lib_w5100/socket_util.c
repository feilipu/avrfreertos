/*
*
@file		sockutil.c
@brief	Implementation of useful function of W5100
*
*/



#include <stdio.h>
#include <string.h>

#include <avr/io.h>

/* Scheduler include files. */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include <lib_serial.h>
#include <w5100.h>
#include <socket.h>

/////////////////////////////////////////////////////////////////////////

uint16_t swaps(uint16_t i);
uint32_t swapl(uint32_t l);

/////////////////////////////////////////////////////////////////////////

uint16_t swaps(uint16_t i)
{
	uint16_t ret=0;
	ret = (i & 0xFF) << 8;
	ret |= ((i >> 8)& 0xFF);
	return ret;
}

uint32_t swapl(uint32_t l)
{
	uint32_t ret=0;
	ret = (l & 0xFF) << 24;
	ret |= ((l >> 8) & 0xFF) << 16;
	ret |= ((l >> 16) & 0xFF) << 8;
	ret |= ((l >> 24) & 0xFF);
	return ret;
}


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

/** This function converts HEX(0-F) to a character */
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
	uint16_t* ret	/**<  is a integer pointer to return */
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
	strcpy((char *)taddr,(char *)addr);

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

	strcpy((char *)tsrc,(char *)src);

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
	uint32_t addr=0;

	for(uint8_t i=0; i < 4; ++i)
	{
		addr <<=8;
		addr += W5100_READ(Sn_DIPR0(s)+i);
	}
	return addr;
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

	port = W5100_READ( Sn_DPORT0(s) );
	port = ((port & 0x00ff) << 8) + W5100_READ( Sn_DPORT1(s) );
	return port;
}
#endif



/**
@brief	htons function converts a uint16_t from host to TCP/IP network byte order (which is big-endian).
@return 	the value in TCP/IP network byte order
*/
uint16_t htons(
	uint16_t hostshort	/**< A 16-bit number in host byte order.  */
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
	uint32_t hostlong		/**< hostshort  - A 32-bit number in host byte order.  */
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
@return 	a 16-bit number in host byte order
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
	for(uint8_t i =0; i < 4; ++i)
	{
		if((pdestip[i] & W5100_READ(SUBR0+i)) != (W5100_READ(SIPR0+i) & W5100_READ(SUBR0+i)))
			return 1;	// Remote
	}
	return 0;
}


/**
@brief	Get handle of socket which status is same to 'status'
@return 	socket number
*/
SOCKET getSocket(
	uint8_t status, 	/**< socket's status to be found */
	SOCKET start			/**< base of socket to be found */
	)
{
	if(start > 3) start = 0;

	for(SOCKET i = start; i < MAX_SOCK_NUM ; ++i) if( getSn_SR(i)==status ) return i;
	return MAX_SOCK_NUM;
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
	register uint16_t  tsum;
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
@brief	Get Source IP Address of W5100.
@return 	Source IP Address(32bit Address-Host Ordering)
*/
uint32_t GetIPAddress(void)
{
	uint32_t ip=0;

	for(uint8_t i=0; i < 4; ++i)
	{
		ip <<= 8;
		ip += (uint8_t)W5100_READ(SIPR0+i);
	}
	return ip;
}


/**
@brief	Get Gateway IP Address of W5100.
@return 	Gateway IP Address(32bit Address-Host Ordering)
*/
uint32_t GetGWAddress(void)
{
	uint32_t ip=0;

	for(uint8_t i=0; i < 4; ++i)
	{
		ip <<= 8;
		ip += (uint8_t)W5100_READ(GAR0+i);
	}
	return ip;
}


/**
@brief	Get Subnet mask of W5100.
@return 	Subnet Mask(32bit Address-Host Ordering)
*/
uint32_t GetSubMask(void)
{
	uint32_t ip=0;

	applySUBR();			// apply subnet mask address - solve Errata 2 & 3 v1.6

	for(uint8_t i=0; i < 4; ++i)
	{
		ip <<= 8;
		ip += (uint8_t)W5100_READ(SUBR0+i);
	}

	clearSUBR();			// clear subnet mask address - solve Errata 2 & 3 v1.6

	return ip;
}


/**
@brief	Get Mac Address of W5100.
@return 	Subnet Mask(32bit Address-Host Ordering)
*/
void GetMacAddress(
	uint8_t* mac	/**< Pointer to store Mac Address (48bit Address)(INPUT, OUTPUT) */
	)
{
	for(uint8_t i=0; i<6; ++i) *mac++ = W5100_READ(SHAR0+i);
}

void GetDestMacAddr(SOCKET s, uint8_t* mac)
{
	for(uint8_t i=0; i<6; ++i) *mac++ = W5100_READ(Sn_DHAR0(s)+i);
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
