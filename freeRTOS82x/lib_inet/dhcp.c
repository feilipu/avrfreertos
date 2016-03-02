/**
 * @file	dhcp.c
 * @brief 	functions relevant to dhcp
 */

#include <string.h>
#include <util/delay.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "serial.h"

#include "inet.h"

#if defined(_WIZCHIP_)		// Definition in freeRTOSBoardDefs.h

un_l2cval get_sn_mask;						/**< Subnet mask received from the DHCP server */
uint8_t get_gw_ip[4];						/**< Gateway ip address received from the DHCP server */
uint8_t get_dns_ip[4] = "\x00\x00\x00\x00";	/**< DNS server ip address received from the DHCP server */
uint8_t get_source_ip[4] = {0,};			/**< source ip address received from the DHCP server */

static uint8_t dhcp_sip[4] = {0,};			/**< DHCP server ip address is discovered */
static uint8_t dhcp_real_sip[4] = {0,};		/**< For extract my DHCP server in a few DHCP servers */
static uint8_t old_sip[4];					/**< Previous local ip address received from DHCP server */

static uint8_t dhcp_state;					/**< DHCP client status */
static uint8_t retry_count;					/**< retry count */

static uint8_t DHCP_timeout;				/**< DHCP Timeout flag */
static un_l2cval lease_time;				/**< Leased time */
static TickType_t start_dhcp_time, next_dhcp_time;
											/**< DHCP Timer tick count */

static uint32_t DHCP_XID;
static SOCKET DHCPC_SOCK;					/**< Socket for the DHCP client */
static RIP_MSG *pRIPMSG;					/**< Pointer for the DHCP message */

#if (_WIZCHIP_ >= 5200)
// TX MEM SIZE- SOCKET 0:8KB, SOCKET 1:2KB, SOCKET2-7:1KB
// RX MEM SIZE- SOCKET 0:8KB, SOCKET 1:2KB, SOCKET2-7:1KB
extern uint8_t txsize[_WIZCHIP_MAX_SOC_NUM_];
extern uint8_t rxsize[_WIZCHIP_MAX_SOC_NUM_];
#endif


void (*dhcp_ip_update)(void) = 0;			/**< handler to be called when the IP address from DHCP server is updated */
void (*dhcp_ip_conflict)(void) = 0;			/**< handler to be called when the IP address from DHCP server is conflict */

uint8_t	init_dhcpc_ch(SOCKET s);			// Initialise the socket for DHCP client

static void send_DHCP_DISCOVER(SOCKET s);	/* Send the discovery message to the DHCP server */
static void send_DHCP_REQUEST(SOCKET s);	/* Send the request message to the DHCP server */
static void send_DHCP_RELEASE_DECLINE(SOCKET s,char msgtype);
											/**< send the release message to the DHCP server */
static uint8_t parseDHCPMSG(SOCKET s, uint16_t length);
											/* Receive the message from DHCP server and parse it. */
static void reset_DHCP_time(void);			/* Initialise DHCP Timer */
static uint8_t check_leasedIP(void);		/* Check the leased IP address	*/
static void check_DHCP_Timeout(void);		/* Check DHCP Timeout  */
static void set_DHCP_network(void);			/* Apply the leased IP address to EtherMega */
static void proc_ip_conflict(void);			/* called when the leased IP address is conflict */


SOCKET	get_DHCP_socket(void)				// Get the socket assigned for DHCP
{
	return DHCPC_SOCK;
}

/**
 * @brief		reset timeout value and retry count
 */
static void reset_DHCP_time(void)
{
	start_dhcp_time  = xTaskGetTickCount();
	next_dhcp_time = start_dhcp_time + DHCP_WAIT_TIME * 1000 / portTICK_PERIOD_MS;

	retry_count = 0;
}


/**
 * @brief		This function sends DHCP DISCOVER message to DHCP server.
 */
static void send_DHCP_DISCOVER(
	SOCKET s	/**< a socket number. */
	)
{
	uint8_t ip[4];
	uint8_t i;

	i = 0;

	*((uint32_t*)dhcp_sip) = 0;
	*((uint32_t*)dhcp_real_sip) = 0;

	memset((void*)pRIPMSG,0,sizeof(RIP_MSG));

	pRIPMSG->op = DHCP_BOOTREQUEST;
	pRIPMSG->htype = DHCP_HTYPE_10MB;
	pRIPMSG->hlen = DHCP_HLEN_ETHERNET;
	pRIPMSG->hops = DHCP_HOPS;
	pRIPMSG->xid = htonl(DHCP_XID);
	pRIPMSG->secs = htons(DHCP_SECS);
	pRIPMSG->flags = htons(DHCP_FLAGSBROADCAST);
	pRIPMSG->chaddr[0] = src_mac_addr[0];
	pRIPMSG->chaddr[1] = src_mac_addr[1];
	pRIPMSG->chaddr[2] = src_mac_addr[2];
	pRIPMSG->chaddr[3] = src_mac_addr[3];
	pRIPMSG->chaddr[4] = src_mac_addr[4];
	pRIPMSG->chaddr[5] = src_mac_addr[5];

	/* MAGIC_COOKIE */
	pRIPMSG->OPT[i++] = (uint8_t)((MAGIC_COOKIE >> 24)& 0xFF);
	pRIPMSG->OPT[i++] = (uint8_t)((MAGIC_COOKIE >> 16)& 0xFF);
	pRIPMSG->OPT[i++] = (uint8_t)((MAGIC_COOKIE >> 8)& 0xFF);
	pRIPMSG->OPT[i++] = (uint8_t)(MAGIC_COOKIE& 0xFF);

	/* Option Request Param. */
	pRIPMSG->OPT[i++] = dhcpMessageType;
	pRIPMSG->OPT[i++] = 0x01;
	pRIPMSG->OPT[i++] = DHCP_DISCOVER;

	// Client identifier
	pRIPMSG->OPT[i++] = dhcpClientIdentifier;
	pRIPMSG->OPT[i++] = 0x07;
	pRIPMSG->OPT[i++] = 0x01;
	pRIPMSG->OPT[i++] = src_mac_addr[0];
	pRIPMSG->OPT[i++] = src_mac_addr[1];
	pRIPMSG->OPT[i++] = src_mac_addr[2];
	pRIPMSG->OPT[i++] = src_mac_addr[3];
	pRIPMSG->OPT[i++] = src_mac_addr[4];
	pRIPMSG->OPT[i++] = src_mac_addr[5];

	// host name
	pRIPMSG->OPT[i++] = hostName;
	pRIPMSG->OPT[i++] = strlen(HOST_NAME)+3; // length of hostname + 3, to add 3 MAC bytes
	strlcpy((char*)&(pRIPMSG->OPT[i]),HOST_NAME, strlen(HOST_NAME));

	i+=strlen(HOST_NAME);

	pRIPMSG->OPT[i++] = src_mac_addr[3];
	pRIPMSG->OPT[i++] = src_mac_addr[4];
	pRIPMSG->OPT[i++] = src_mac_addr[5];


	pRIPMSG->OPT[i++] = dhcpParamRequest;
	pRIPMSG->OPT[i++] = 0x06;
	pRIPMSG->OPT[i++] = subnetMask;
	pRIPMSG->OPT[i++] = routersOnSubnet;
	pRIPMSG->OPT[i++] = dns;
	pRIPMSG->OPT[i++] = domainName;
	pRIPMSG->OPT[i++] = dhcpT1value;
	pRIPMSG->OPT[i++] = dhcpT2value;
	pRIPMSG->OPT[i++] = endOption;

	/* send broadcast packet */
	ip[0] = 255;
	ip[1] = 255;
	ip[2] = 255;
	ip[3] = 255;

	if(0 == sendto(s, (uint8_t *)pRIPMSG, sizeof(RIP_MSG), ip, IP_PORT_DHCP_SERVER))
	{
		xSerialPrint_P(PSTR("\r\nDHCP: Fatal Error(0)."));
		if ( dhcp_ip_conflict != 0 )
			(*dhcp_ip_conflict)();
	}

	xSerialPrint_P(PSTR("\r\nsent DHCP_DISCOVER\r\n"));
}


/**
 * @brief		This function sends DHCP REQUEST message to DHCP server.
 */
static void send_DHCP_REQUEST(
	SOCKET s	/**<  socket number */
	)
{
	un_l2cval ip;
	uint16_t i;

	i = 0;

	memset((void*)pRIPMSG,0,sizeof(RIP_MSG));


	pRIPMSG->op = DHCP_BOOTREQUEST;
	pRIPMSG->htype = DHCP_HTYPE_10MB;
	pRIPMSG->hlen = DHCP_HLEN_ETHERNET;
	pRIPMSG->hops = DHCP_HOPS;
	pRIPMSG->xid = htonl(DHCP_XID);
	pRIPMSG->secs = htons(DHCP_SECS);

	if(dhcp_state < DHCP_STATE_LEASED)
		pRIPMSG->flags = htons(DHCP_FLAGSBROADCAST);
	else
	{
		pRIPMSG->flags = 0;		// For Unicast
		pRIPMSG->ciaddr[0] = get_source_ip[0];
		pRIPMSG->ciaddr[1] = get_source_ip[1];
		pRIPMSG->ciaddr[2] = get_source_ip[2];
		pRIPMSG->ciaddr[3] = get_source_ip[3];
	}

	pRIPMSG->chaddr[0] = src_mac_addr[0];
	pRIPMSG->chaddr[1] = src_mac_addr[1];
	pRIPMSG->chaddr[2] = src_mac_addr[2];
	pRIPMSG->chaddr[3] = src_mac_addr[3];
	pRIPMSG->chaddr[4] = src_mac_addr[4];
	pRIPMSG->chaddr[5] = src_mac_addr[5];

	/* MAGIC_COOKIE */
	pRIPMSG->OPT[i++] = (uint8_t)((MAGIC_COOKIE >> 24) & 0xFF);
	pRIPMSG->OPT[i++] = (uint8_t)((MAGIC_COOKIE >> 16) & 0xFF);
	pRIPMSG->OPT[i++] = (uint8_t)((MAGIC_COOKIE >> 8) & 0xFF);
	pRIPMSG->OPT[i++] = (uint8_t)(MAGIC_COOKIE & 0xFF);

	/* Option Request Param. */
	pRIPMSG->OPT[i++] = dhcpMessageType;
	pRIPMSG->OPT[i++] = 0x01;
	pRIPMSG->OPT[i++] = DHCP_REQUEST;

	pRIPMSG->OPT[i++] = dhcpClientIdentifier;
	pRIPMSG->OPT[i++] = 0x07;
	pRIPMSG->OPT[i++] = 0x01;
	pRIPMSG->OPT[i++] = src_mac_addr[0];
	pRIPMSG->OPT[i++] = src_mac_addr[1];
	pRIPMSG->OPT[i++] = src_mac_addr[2];
	pRIPMSG->OPT[i++] = src_mac_addr[3];
	pRIPMSG->OPT[i++] = src_mac_addr[4];
	pRIPMSG->OPT[i++] = src_mac_addr[5];

	if(dhcp_state < DHCP_STATE_LEASED)
	{
		pRIPMSG->OPT[i++] = dhcpRequestedIPaddr;
		pRIPMSG->OPT[i++] = 0x04;
		pRIPMSG->OPT[i++] = get_source_ip[0];
		pRIPMSG->OPT[i++] = get_source_ip[1];
		pRIPMSG->OPT[i++] = get_source_ip[2];
		pRIPMSG->OPT[i++] = get_source_ip[3];

		pRIPMSG->OPT[i++] = dhcpServerIdentifier;
		pRIPMSG->OPT[i++] = 0x04;
		pRIPMSG->OPT[i++] = dhcp_sip[0];
		pRIPMSG->OPT[i++] = dhcp_sip[1];
		pRIPMSG->OPT[i++] = dhcp_sip[2];
		pRIPMSG->OPT[i++] = dhcp_sip[3];
	}

	// host name
	pRIPMSG->OPT[i++] = hostName;
	pRIPMSG->OPT[i++] = strlen(HOST_NAME)+3; // length of hostname + 3
	strlcpy((char*)&(pRIPMSG->OPT[i]),HOST_NAME, strlen(HOST_NAME));

	i+=strlen(HOST_NAME);

	pRIPMSG->OPT[i++] = src_mac_addr[3];
	pRIPMSG->OPT[i++] = src_mac_addr[4];
	pRIPMSG->OPT[i++] = src_mac_addr[5];

	pRIPMSG->OPT[i++] = dhcpParamRequest;
	pRIPMSG->OPT[i++] = 0x08;
	pRIPMSG->OPT[i++] = subnetMask;
	pRIPMSG->OPT[i++] = routersOnSubnet;
	pRIPMSG->OPT[i++] = dns;
	pRIPMSG->OPT[i++] = domainName;
	pRIPMSG->OPT[i++] = dhcpT1value;
	pRIPMSG->OPT[i++] = dhcpT2value;
	pRIPMSG->OPT[i++] = performRouterDiscovery;
	pRIPMSG->OPT[i++] = staticRoute;
	pRIPMSG->OPT[i++] = endOption;

	/* send broadcast packet */
	if(dhcp_state < DHCP_STATE_LEASED)
	{
		ip.lVal = 0xffffffff;
	}
	else
	{
		ip.cVal[0] = dhcp_sip[0];
		ip.cVal[1] = dhcp_sip[1];
		ip.cVal[2] = dhcp_sip[2];
		ip.cVal[3] = dhcp_sip[3];
	}

	if(0 == sendto(s, (uint8_t*)pRIPMSG, sizeof(RIP_MSG), ip.cVal, IP_PORT_DHCP_SERVER))
	{
		xSerialPrint_P(PSTR("\r\nDHCP: Fatal Error(1)."));
		if ( dhcp_ip_conflict != 0 )
			(*dhcp_ip_conflict)();
	}

	xSerialPrint_P(PSTR("\r\nsent DHCP_REQUEST\r\n"));

}


/**
 * @brief		This function sends DHCP RELEASE message to DHCP server.
 */
static void send_DHCP_RELEASE_DECLINE(
	SOCKET s,		/**< socket number */
	char msgtype	/**< 0 : RELEASE, Not Zero : DECLINE */
	)
{
	un_l2cval ip;
	uint16_t i;

	i = 0;

	memset((void*)pRIPMSG,0,sizeof(RIP_MSG));

	pRIPMSG->op = DHCP_BOOTREQUEST;
	pRIPMSG->htype = DHCP_HTYPE_10MB;
	pRIPMSG->hlen = DHCP_HLEN_ETHERNET;
	pRIPMSG->hops = DHCP_HOPS;
	pRIPMSG->xid = htonl(DHCP_XID);
	pRIPMSG->secs = htons(DHCP_SECS);
	pRIPMSG->flags = 0;	//DHCP_FLAGSBROADCAST;

	pRIPMSG->chaddr[0] = src_mac_addr[0];
	pRIPMSG->chaddr[1] = src_mac_addr[1];
	pRIPMSG->chaddr[2] = src_mac_addr[2];
	pRIPMSG->chaddr[3] = src_mac_addr[3];
	pRIPMSG->chaddr[4] = src_mac_addr[4];
	pRIPMSG->chaddr[5] = src_mac_addr[5];


	/* MAGIC_COOKIE */
	pRIPMSG->OPT[i++] = (uint8_t)((MAGIC_COOKIE >> 24) & 0xFF);
	pRIPMSG->OPT[i++] = (uint8_t)((MAGIC_COOKIE >> 16) & 0xFF);
	pRIPMSG->OPT[i++] = (uint8_t)((MAGIC_COOKIE >> 8) & 0xFF);
	pRIPMSG->OPT[i++] = (uint8_t)(MAGIC_COOKIE & 0xFF);

	/* Option Request Param. */
	pRIPMSG->OPT[i++] = dhcpMessageType;
	pRIPMSG->OPT[i++] = 0x01;
	pRIPMSG->OPT[i++] = ((!msgtype) ? DHCP_RELEASE : DHCP_DECLINE);

	pRIPMSG->OPT[i++] = dhcpClientIdentifier;
	pRIPMSG->OPT[i++] = 0x07;
	pRIPMSG->OPT[i++] = 0x01;
	pRIPMSG->OPT[i++] = src_mac_addr[0];
	pRIPMSG->OPT[i++] = src_mac_addr[1];
	pRIPMSG->OPT[i++] = src_mac_addr[2];
	pRIPMSG->OPT[i++] = src_mac_addr[3];
	pRIPMSG->OPT[i++] = src_mac_addr[4];
	pRIPMSG->OPT[i++] = src_mac_addr[5];

	pRIPMSG->OPT[i++] = dhcpServerIdentifier;
	pRIPMSG->OPT[i++] = 0x04;
	pRIPMSG->OPT[i++] = dhcp_sip[0];
	pRIPMSG->OPT[i++] = dhcp_sip[1];
	pRIPMSG->OPT[i++] = dhcp_sip[2];
	pRIPMSG->OPT[i++] = dhcp_sip[3];

	if(msgtype)
	{
		pRIPMSG->OPT[i++] = dhcpRequestedIPaddr;
		pRIPMSG->OPT[i++] = 0x04;
		pRIPMSG->OPT[i++] = get_source_ip[0];
		pRIPMSG->OPT[i++] = get_source_ip[1];
		pRIPMSG->OPT[i++] = get_source_ip[2];
		pRIPMSG->OPT[i++] = get_source_ip[3];
		pRIPMSG->OPT[i++] = endOption;
		xSerialPrint_P(PSTR("\r\nsent DHCP_DECLINE\r\n"));
	}
	else
	{
		pRIPMSG->OPT[i++] = endOption;
		xSerialPrint_P(PSTR("\r\nsent DHCP_RELEASE\r\n"));
	}

	if(!msgtype)
	{
		ip.cVal[0] = dhcp_sip[0];
		ip.cVal[1] = dhcp_sip[1];
		ip.cVal[2] = dhcp_sip[2];
		ip.cVal[3] = dhcp_sip[3];
	}
	else
	{
		ip.lVal = 0xffffffff;
	}

	if(0 == sendto(s, (uint8_t *)pRIPMSG, sizeof(RIP_MSG), ip.cVal, IP_PORT_DHCP_SERVER))
	{
		xSerialPrint_P(PSTR("\r\nDHCP: Fatal Error(2)."));
		if ( dhcp_ip_conflict != 0 )
			(*dhcp_ip_conflict)();
	}

}


/**
 * @brief		This function parses the reply message from DHCP server.
 * @return	success - return type, fail - 0
 */
static uint8_t parseDHCPMSG(
	SOCKET s, 	/**< socket number */
	uint16_t length	/**< a size data to receive. */
	)
{
	uint8_t svr_addr[6];
	uint16_t  svr_port;

	uint16_t len;
	uint8_t * p;
	uint8_t * e;
	uint8_t type, opt_len=0;

	len = recvfrom(s, (uint8_t *)pRIPMSG, length, svr_addr, &svr_port);

#ifdef DHCP_DEBUG
	xSerialPrintf_P(PSTR("\r\ndhcp_sip: %d.%d.%d.%d"),dhcp_sip[0],dhcp_sip[1],dhcp_sip[2],dhcp_sip[3]);
	xSerialPrintf_P(PSTR("\r\nDHCP_RIP: %d.%d.%d.%d"),dhcp_real_sip[0],dhcp_real_sip[1],dhcp_real_sip[2],dhcp_real_sip[3]);
	xSerialPrintf_P(PSTR("\r\nsvr_addr: %d.%d.%d.%d"),svr_addr[0],svr_addr[1],svr_addr[2],svr_addr[3]);
#endif

	if(pRIPMSG->op != DHCP_BOOTREPLY)
	{
		xSerialPrint_P(PSTR("\r\nDHCP : NO DHCP MSG"));
	}
	else
	{
		if (svr_port == IP_PORT_DHCP_SERVER)
		{
			if(memcmp(pRIPMSG->chaddr,src_mac_addr,6) != 0 || pRIPMSG->xid != htonl(DHCP_XID))
			{
#ifdef DHCP_DEBUG
				xSerialPrint_P(PSTR("\r\nNot my DHCP Message. This message is ignored..."));

				xSerialPrintf_P(PSTR("\tsrc_mac_addr(%02X.%02X.%02X."),src_mac_addr[0],src_mac_addr[1],src_mac_addr[2]);
				xSerialPrintf_P(PSTR("%02X.%02X.%02X)"),src_mac_addr[3],src_mac_addr[4],src_mac_addr[5]);
				xSerialPrintf_P(PSTR(", pRIPMSG->chaddr(%02X.%02X.%02X."),pRIPMSG->chaddr[0],pRIPMSG->chaddr[1],pRIPMSG->chaddr[2]);
				xSerialPrintf_P(PSTR("\r\n%02X.%02X.%02X)"),pRIPMSG->chaddr[3],pRIPMSG->chaddr[4],pRIPMSG->chaddr[5]);
				xSerialPrintf_P(PSTR("\r\n\tpRIPMSG->xid(%08lX), DHCP_XID(%08lX)"),pRIPMSG->xid,htonl(DHCP_XID));
				xSerialPrintf_P(PSTR("\r\n\tpRIMPMSG->yiaddr:%d.%d.%d.%d"),pRIPMSG->yiaddr[0],pRIPMSG->yiaddr[1],pRIPMSG->yiaddr[2],pRIPMSG->yiaddr[3]);
#endif
				return 0;
			}

			if( *((uint32_t*)dhcp_sip) != 0x00000000 )
			{
				if( *((uint32_t*)dhcp_real_sip) != *((uint32_t*)svr_addr) &&
					*((uint32_t*)dhcp_sip) != *((uint32_t*)svr_addr) )
				{
#ifdef DHCP_DEBUG
					xSerialPrint_P(PSTR("\r\nAnother DHCP sever send a response message. This is ignored."));
					xSerialPrintf_P(PSTR("\r\n\tIP:%d.%d.%d.%d"),svr_addr[0],svr_addr[1],svr_addr[2],svr_addr[3]);
#endif
					return 0;
				}
			}

			memcpy(get_source_ip,pRIPMSG->yiaddr,4);

			xSerialPrint_P(PSTR("\r\nDHCP MSG received..."));

			type = 0;
			p = (uint8_t *)(&pRIPMSG->op);
			p = p + 240;
			e = p + (len - 240);

#ifdef DHCP_DEBUG
			xSerialPrintf_P(PSTR("\r\nyiaddr : %d.%d.%d.%d"),get_source_ip[0],get_source_ip[1],get_source_ip[2],get_source_ip[3]);
			xSerialPrintf_P(PSTR("\r\np: 0x%08X  e: 0x%08X  len: %d\r\n"), (uint16_t)p, (uint16_t)e, len);
#endif
			while ( p < e )
			{
				switch ( *p++ )
				{
				case endOption :
				 	return type;
					break;
	       			case padOption :
					break;
				case dhcpMessageType :
					opt_len = *p++;
					type = *p;
#ifdef DHCP_DEBUG
					xSerialPrintf_P(PSTR("\r\ndhcpMessageType: %x"), type);
#endif
					break;
				case subnetMask :
					opt_len =* p++;
					memcpy(&get_sn_mask,p,4);
#ifdef DHCP_DEBUG
					xSerialPrint_P(PSTR("\r\nsubnetMask: "));
					xSerialPrintf_P(PSTR("\r\n%d.%d.%d.%d"),get_sn_mask.cVal[0],get_sn_mask.cVal[1],get_sn_mask.cVal[2],get_sn_mask.cVal[3]);
#endif
					break;
				case routersOnSubnet :
					opt_len = *p++;
					memcpy(get_gw_ip,p,4);
#ifdef DHCP_DEBUG
					xSerialPrint_P(PSTR("\r\nroutersOnSubnet: "));
					xSerialPrintf_P(PSTR("\r\n%d.%d.%d.%d"),get_gw_ip[0],get_gw_ip[1],get_gw_ip[2],get_gw_ip[3]);
#endif
					break;
				case dns :
					opt_len = *p++;
					memcpy(get_dns_ip,p,4);
					break;
				case dhcpIPaddrLeaseTime :
					opt_len = *p++;
					lease_time.lVal = ntohl(*((uint32_t*)p));
#ifdef DHCP_DEBUG
					xSerialPrintf_P(PSTR("\r\ndhcpIPaddrLeaseTime: %08lX"), lease_time.lVal);
#endif
					break;

				case dhcpServerIdentifier :
					opt_len = *p++;
#ifdef DHCP_DEBUG
					xSerialPrintf_P(PSTR("\r\ndhcp_sip: %d.%d.%d.%d"), dhcp_sip[0], dhcp_sip[1], dhcp_sip[2], dhcp_sip[3]);
#endif
					if( *((uint32_t*)dhcp_sip) == 0 ||
					    *((uint32_t*)dhcp_real_sip) == *((uint32_t*)svr_addr) ||
					    *((uint32_t*)dhcp_sip) == *((uint32_t*)svr_addr) )
					{
						memcpy(dhcp_sip,p,4);
						memcpy(dhcp_real_sip,svr_addr,4);	// Copy the real ip address of my DHCP server
#ifdef DHCP_DEBUG
						xSerialPrint_P(PSTR("\r\nMy dhcpServerIdentifier: "));
						xSerialPrintf_P(PSTR("\r\n%d.%d.%d.%d"), dhcp_sip[0], dhcp_sip[1], dhcp_sip[2], dhcp_sip[3]);
						xSerialPrint_P(PSTR("\r\nMy DHCP server real IP address: "));
						xSerialPrintf_P(PSTR("\r\n%d.%d.%d.%d"), dhcp_real_sip[0], dhcp_real_sip[1], dhcp_real_sip[2], dhcp_real_sip[3]);
#endif
					}
					else
					{
#ifdef DHCP_DEBUG
						xSerialPrint_P(PSTR("\r\nAnother dhcpServerIdentifier: "));
						xSerialPrintf_P(PSTR("\r\n\tMY(%d.%d.%d.%d) "), dhcp_sip[0], dhcp_sip[1], dhcp_sip[2], dhcp_sip[3]);
						xSerialPrintf_P(PSTR("\r\nAnother(%d.%d.%d.%d): "), svr_addr[0], svr_addr[1], svr_addr[2], svr_addr[3]);
#endif
					}

					break;
				default :
					opt_len = *p++;
#ifdef DHCP_DEBUG
					xSerialPrintf_P(PSTR("\r\nopt_len: %d"), opt_len);
#endif
					break;
				} // switch
				p+=opt_len;
			} // while
		} // if
	}

	return 0;
}


/**
 * @brief		This function checks the state of DHCP.
 */
void check_DHCP_state(
	SOCKET s	/**< socket number */
	)
{
	uint16_t len;
	uint8_t type;

	type = 0;

	if( s < _WIZCHIP_MAX_SOC_NUM_ && getSn_SR(s)!=SOCK_CLOSED)
	{
		if ((len = getSn_RX_RSR(s)) > 0)
		{
			 type = parseDHCPMSG(s, len);
		}
	}
	else if(!socket(s, Sn_MR_UDP, IP_PORT_DHCP_CLIENT, 0x00))
	{
		xSerialPrint_P(PSTR("\r\nFail to create the DHCPC_SOCK(%d)"));
	}


	switch ( dhcp_state )
	{
	case DHCP_STATE_DISCOVER :
		if (type == DHCP_OFFER)
		{
			xSerialPrint_P(PSTR("\r\nstate: DHCP_STATE_REQUEST\r\n"));
			dhcp_state = DHCP_STATE_REQUEST;
			send_DHCP_REQUEST(s);
			reset_DHCP_time();
		}
		else check_DHCP_Timeout();
		break;

	case DHCP_STATE_REQUEST :
		if (type == DHCP_ACK)
		{
			reset_DHCP_time();
			if (check_leasedIP())
			{
				xSerialPrint_P(PSTR("\r\nstate: DHCP_STATE_LEASED\r\n"));
				dhcp_state = DHCP_STATE_LEASED;
				set_DHCP_network();
			}
			else
			{
				xSerialPrint_P(PSTR("\r\nstate: DHCP_STATE_DISCOVER\r\n"));
				dhcp_state = DHCP_STATE_DISCOVER;
			}
		}
		else if (type == DHCP_NAK)
		{
			xSerialPrint_P(PSTR("\r\nstate: DHCP_STATE_DISCOVER\r\n"));
			dhcp_state = DHCP_STATE_DISCOVER;
			reset_DHCP_time();
		}
		else check_DHCP_Timeout();
		break;

	case DHCP_STATE_LEASED :
		if ((lease_time.lVal != 0xffffffff) && (((lease_time.lVal/2)* 1000 / portTICK_PERIOD_MS) < (xTaskGetTickCount() - start_dhcp_time)))
		{
			xSerialPrint_P(PSTR("\r\nstate: DHCP_STATE_REREQUEST\r\n"));
			type = 0;
			memcpy(old_sip,get_source_ip,4);
			DHCP_XID++;
			dhcp_state = DHCP_STATE_REREQUEST;
			send_DHCP_REQUEST(s);
			reset_DHCP_time();
		}
		break;

	case DHCP_STATE_REREQUEST :
		if (type == DHCP_ACK)
		{
			if(memcmp(old_sip,get_source_ip,4)!=0)
			{
				xSerialPrintf_P(PSTR("\r\nold_sip=%s,get_source_ip=%s"),inet_ntoa(ntohl(*((uint32_t*)old_sip))), inet_ntoa(ntohl(*((uint32_t*)get_source_ip))));
				if ( dhcp_ip_update != 0 )
					(*dhcp_ip_update)();
				xSerialPrint_P(PSTR("\r\nThe IP address from the DHCP server is updated."));
			}
			else
			{
				xSerialPrint_P(PSTR("\r\nstate: DHCP_STATE_LEASED same IP\r\n"));
			}
			dhcp_state = DHCP_STATE_LEASED;
			reset_DHCP_time();
		}
		else if (type == DHCP_NAK)
		{
			xSerialPrint_P(PSTR("\r\nstate: DHCP_STATE_DISCOVER\r\n"));
			dhcp_state = DHCP_STATE_DISCOVER;
			reset_DHCP_time();
		}
		else check_DHCP_Timeout();
		break;

	case DHCP_STATE_RELEASE :
		break;
	default :
		break;
	}
}


/**
 * @brief		This function checks the timeout of DHCP in each state.
 */
static void check_DHCP_Timeout(void)
{
	if (retry_count < DHCP_MAX_RETRY)
	{

		if ( next_dhcp_time < xTaskGetTickCount() )
		{ // XXX THINK about this some more, TickCount roll over is the potential issue.
			start_dhcp_time  = xTaskGetTickCount() ;
			next_dhcp_time = start_dhcp_time + DHCP_WAIT_TIME * 1000 / portTICK_PERIOD_MS;
			retry_count++;
			switch ( dhcp_state )
			{
			case DHCP_STATE_DISCOVER :
				xSerialPrint_P(PSTR("\r\n<timeout> state: DHCP_STATE_DISCOVER"));
				send_DHCP_DISCOVER(DHCPC_SOCK);
				break;

			case DHCP_STATE_REQUEST :
				xSerialPrint_P(PSTR("\r\n<timeout> state: DHCP_STATE_REQUEST"));
				send_DHCP_REQUEST(DHCPC_SOCK);
				break;

			case DHCP_STATE_REREQUEST :
				xSerialPrint_P(PSTR("\r\n<timeout> state: DHCP_STATE_REREQUEST"));
				send_DHCP_REQUEST(DHCPC_SOCK);
				break;

			default :
				break;
			}
		}
	}
	else
	{
		reset_DHCP_time();
		DHCP_timeout = 1;

		xSerialPrint_P(PSTR("\r\n<<timeout>> state: DHCP_STATE_DISCOVER"));
		dhcp_state = DHCP_STATE_DISCOVER;
		send_DHCP_DISCOVER(DHCPC_SOCK);
	}
}


/**
 * @brief		This function loads network info to W5100
 */
static void set_DHCP_network(void)
{

#if   (_WIZCHIP_ == 5100)

	WIZCHIP_sysinit(0x55, 0x55);
#else
	WIZCHIP_init();
	WIZCHIP_sysinit( (uint8_t *)txsize, (uint8_t *)rxsize );	// set the Tx and Rx buffers
#endif

#if   (_WIZCHIP_ != 5500) // xxx the Wiz550io has its MAC address loaded by the onboard MCU.
	setSHAR(src_mac_addr);
#endif

	setGAR(get_gw_ip);

#if   (_WIZCHIP_ <= 5200)
	saveSUBR((un_l2cval *)&get_sn_mask);
#else
	setSUBR((uint8_t *)&get_sn_mask);
#endif

	setSIPR(get_source_ip);

#ifdef __DEF_WIZCHIP_INT__
        setIMR(0xEF);
#endif


	xSerialPrintf_P(PSTR("\r\nDHCP set IP: %d.%d.%d.%d\r\n"), get_source_ip[0], get_source_ip[1], get_source_ip[2], get_source_ip[3]);
}


/**
 * @brief		check if a leased IP is valid
 * @return	0 : conflict, 1 : no conflict
 */
static uint8_t check_leasedIP(void)
{

//	uint16_t a;
#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrint_P(PSTR("\r\n<Check the IP Conflict : <skipped>"));
#endif
	// sendto is complete. that means there is a node which has a same IP.

	/* +200801 (hwkim) */
	//a = sendto(DHCPC_SOCK, (const uint8_t*)"CHECK_IP_CONFLICT", 17, get_source_ip, 5000);

/*	a=0; // Skip checking IP Conflict ; W5100 reply itself to ARP request with self-IP in non-switching network environment.
	if ( a> 0)
	{
		xSerialPrint_P(PSTR(" Conflict>\r\n"));
		send_DHCP_RELEASE_DECLINE(DHCPC_SOCK,1);
		if ( dhcp_ip_conflict != 0 )
			(*dhcp_ip_conflict)();
		return 0;
	} // */
#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrint_P(PSTR(" No Conflict>\r\n"));
#endif
	return 1;

}


/**
 * @brief		Get an IP from the DHCP server.
 * @return	0 : timeout, 1 : got dhcp ip
 */
uint8_t getIP_DHCPS()
{
	send_DHCP_DISCOVER(DHCPC_SOCK);
	dhcp_state = DHCP_STATE_DISCOVER;

	reset_DHCP_time();
	DHCP_timeout = 0;

	while (dhcp_state != DHCP_STATE_LEASED)
	{
		if (DHCP_timeout == 1)
		{
			return 0;
		}
		check_DHCP_state(DHCPC_SOCK);
		vTaskDelay(  128 / portTICK_PERIOD_MS  );
	}
	return 1;
}



/**
 * @brief		Get an IP from the DHCP server.
 */
void init_DHCP_client(
	SOCKET s,				/**< Preferred socket number for the DHCP client */
	void(*ip_update)(void),	/**< handler called when the leased IP address is updated */
	void(*ip_conflict)(void)	/**< handler called when the leased IP address is conflict */
	)
{

	if(!ip_update)	dhcp_ip_update = set_DHCP_network;
	else		dhcp_ip_update = ip_update;

	if(!ip_conflict) dhcp_ip_conflict = proc_ip_conflict;
	else		 dhcp_ip_conflict = ip_conflict;

	init_dhcpc_ch(s);

}


/**
 * @brief		Get an IP from the DHCP server.
 */
static void proc_ip_conflict(void)
{
	xSerialPrint_P(PSTR("\r\nThe IP Address from DHCP server is CONFLICT!!!\r\n"
			"Retry to get a IP address from DHCP server\r\n"));
}


/**
 * @brief		Initialise the socket for DHCP client
 */
uint8_t init_dhcpc_ch(SOCKET s)
{
	uint8_t ret;

	DHCP_XID = DHCP_INITIAL_XID;
	memset(get_source_ip,0,4);
	memset(get_gw_ip,0,4);

	get_sn_mask.lVal = 0;

#if   (_WIZCHIP_ == 5100)
    WIZCHIP_init();
	WIZCHIP_sysinit(0x55, 0x55);
#else
    WIZCHIP_init();
	WIZCHIP_sysinit( (uint8_t *)txsize, (uint8_t *)rxsize );// set the Tx and Rx buffers
#endif

#if   (_WIZCHIP_ != 5500) // xxx the Wiz550io has its MAC address loaded by the on-board MCU.
	setSHAR(src_mac_addr);
#endif

	setSIPR(get_source_ip);

#ifdef __DEF_WIZCHIP_INT__
       setIMR(0xEF);
#endif

#ifdef DHCP_DEBUG
{
	uint16_t i;
	xSerialPrint_P(PSTR("\r\nMAC Addr: "));
	for (i = 0; i < 5; i++) xSerialPrintf_P(PSTR("%02x."), src_mac_addr[i]);
	xSerialPrintf_P(PSTR("%02x"),src_mac_addr[5]);
}
#endif

	if( getSn_SR(s) != SOCK_CLOSED )	// Check the preferred socket is available,
	{
		s = getSocket(SOCK_CLOSED, 0);	// otherwise find free socket,
		if(s == _WIZCHIP_MAX_SOC_NUM_ )        	// If there is no free socket?
			ret = 0;
	} else {
		ret = 1;
	}

	xSerialPrintf_P(PSTR("\r\nDHCP socket: %d,"),s);
	if(!socket(s, Sn_MR_UDP, IP_PORT_DHCP_CLIENT, 0x00)) // initialise the socket for DHCP service
	{
		xSerialPrint_P(PSTR(" fail..!\r\n"));
		ret = 0;
	}
	else
	{
		xSerialPrint_P(PSTR(" ok..!\r\n"));
		ret = 1;
	}

	if(pRIPMSG == NULL) // if there is no buffer allocated (pointer is NULL), then allocate buffer for all DHCP functions.
	{
		if( !(pRIPMSG = (RIP_MSG *) pvPortMalloc( sizeof(RIP_MSG) ) ) )
			ret = 0;
		else
			ret = 1;
	}


	DHCPC_SOCK = s;

	return ret;
}

#endif // #if   defined(_WIZCHIP_)		// Definition in freeRTOSBoardDefs.h
