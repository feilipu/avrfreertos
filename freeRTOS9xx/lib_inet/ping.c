/**
 @file		ping.c
 @brief 	Implementation of Ping - To send 'ping-request' to peer & To receive 'ping-reply' from peer
 */

#include <stdlib.h> // For rand();
#include <string.h>
#include <util/delay.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "serial.h"

#include "socket.h"
#include "inet.h"

#if   defined(_WIZCHIP_)		// Definition in freeRTOSBoardDefs.h

static void SendPingReply(PING_MSG *pingrequest, uint32_t destaddr );

/**
 @brief 	Send ping-request to the specified peer and receive ping-reply from the specified peer.
 @return	1 - success, 0 - fail because free socket is not found or can't be opened.
 */
uint8_t ping(
	uint8_t count, 		/**< Ping request count. */
	uint16_t time, 		/**< wait ping reply time (unit : ms) */
	uint8_t* addr, 		/**< Peer IP Address string in dotted decimal format */
	PING_LOG* log		/**< result of ping */
	)
{

	PING_MSG* pPingRequest;		// pointer for Ping Request
	PING_MSG* pPingReply;		// pointer for Ping Reply
	uint32_t peerip;            // 32 bit Peer IP Address
	uint32_t tempip;			// IP address received from a destination
	uint16_t port;              // port number received from a destination

	SOCKET   s;					// socket variable for pinging

	uint16_t RandomSeqNum;		// Ping-Request ID

	uint16_t len;
	uint8_t  IsReceived;    	// Received packet = 1, not received packet = 0

	TickType_t xInitialTick;

	/* Initialise PingRequest */


	if( !(pPingRequest = (PING_MSG *) pvPortMalloc( sizeof(PING_MSG) )))
		return 0;

	if( !(pPingReply   = (PING_MSG *) pvPortMalloc( sizeof(PING_MSG) )))
	{
		vPortFree(pPingRequest);
		return 0;
	}

	RandomSeqNum = htons( (uint16_t)rand());		// set ping-request's sequence number to random integer value

	pPingRequest->type = 0x08;           			// Ping-Request - ICMP
	pPingRequest->code = 0x00;				// always 0
	pPingRequest->checksum = 0;				// value of checksum before calculating checksum of ping-request packet
	pPingRequest->id = htons(PING_ID);			// set ping-request ID

	for(uint16_t i = 0 ; i < PING_OPT_LEN; i++)
		pPingRequest->OPT[i] = 'a' + i % 23;		// fill 'a'~'w' characters into ping-request's data

	/* Initialise result of ping */
	memset((void*)log,0,sizeof(PING_LOG));

    /* Verify arguments */
	if(!count) count = 4;					// set count to default value (4 pings)
	if(!time) time = 1000;					// set response time to default value (1000ms)

	/* Create a ping socket */
	s = getSocket(SOCK_CLOSED,0);
	if(s == _WIZCHIP_MAX_SOC_NUM_)					// if it isn't exist free socket, Error
	{
		vPortFree(pPingRequest);
		vPortFree(pPingReply);
		return 0;
	}

	setSn_PROTO(s,IP_PROTO_ICMP);           	// Set upper-protocol of IP protocol
	if( socket( s, Sn_MR_IPRAW, 3000, 0) == 0)	// Open IP_RAW Mode , if fail then Error
	{
		vPortFree(pPingRequest);
		vPortFree(pPingReply);
		return 0;
	}
	peerip = htonl( inet_addr(addr));				// convert address string into 32bit address

	xSerialPrintf_P(PSTR("\r\nPinging %s with %d bytes of data:\r\n"), addr, (sizeof(PING_MSG) - 8));


	/* Ping Service */
	while( count-- != 0)
	{
		IsReceived = 0;
		pPingRequest->seqNum = htons( RandomSeqNum++);	// Increase Sequence number for next ping-request packet
		pPingRequest->checksum = 0;
		pPingRequest->checksum = htons( checksum( (uint8_t*)pPingRequest, sizeof(PING_MSG)));	// update checksum field

		(*log).PingRequest++;							// Increase PingRequest's value

		xInitialTick = xTaskGetTickCount();

		if( sendto( s, (const uint8_t *)pPingRequest, sizeof(PING_MSG), (uint8_t*)&peerip, 3000)== 0)	// Send Ping-Request to the specified peer. If fail, then it is occurred ARP Error.
		{
			(*log).ARPErr++;							// Increase ARPErr
			close(s);									// close the pinging socket

			/* Reopen pinging socket */
			setSn_PROTO(s,IP_PROTO_ICMP);
			if(socket( s, Sn_MR_IPRAW, 3000, 0)==0)
				{
				vPortFree(pPingRequest);
				vPortFree(pPingReply);
				return 0;
				}
			continue;
		}

		while( xTaskGetTickCount() < (xInitialTick + ( time / portTICK_PERIOD_MS )) )	// as long as time is remaining
		{

			if((len = getSn_RX_RSR(s)) > 0)		// Has pinging socket received a packet?
			{
				len = recvfrom( s, (uint8_t*)pPingReply, len, (uint8_t*)&tempip, &port);	// receive a packet from unknown peer

				xSerialPrintf_P(PSTR("\r\nReply from %s"), inet_ntoa( ntohl(tempip))); // convert 32 bit unknown peer IP address into string of IP Address.

				if( checksum((uint8_t*)pPingReply,len) != 0)		// if the packet's checksum value is correct
				{                                                   // not correct
					(*log).CheckSumErr++;                           // checksum error
					if(tempip == peerip) IsReceived = 1;
					xSerialPrint_P(PSTR(": Checksum Error"));
				}
				else if(pPingReply->type == 0)					// if the received packet is ping-reply
				{

					if((pPingReply->id!=pPingRequest->id) || (pPingReply->seqNum!=pPingRequest->seqNum) || (tempip!=peerip)) // verify id,sequence number, and ip address
					{
						xSerialPrint_P(PSTR(": Unmatched ID / SeqNum from peer"));			// fail to verify
						(*log).UnknownMSG++;
					}
					else                                                    // success
					{
						IsReceived = 1;
						xSerialPrintf_P(PSTR(": bytes=%d, time<=%dms"),len-8,(xTaskGetTickCount()-xInitialTick)*portTICK_PERIOD_MS );
						(*log).PingReply++;
					}
				}
				else if( pPingReply->type == 3)  					// If the packet is unreachable message
				{
					IsReceived = 1;
					xSerialPrint_P(PSTR(": Destination unreachable"));
					(*log).UnreachableMSG++;
				}
				else if( pPingReply->type == 11)                 	// If the packet is time exceeded message
				{
				        IsReceived = 1;
				        xSerialPrint_P(PSTR(": TTL expired in transit"));
					(*log).TimeExceedMSG++;
				}
				else if( pPingReply->type == 8)					// Send ping reply to a peer
				{
					xSerialPrint_P(PSTR(": Ping Request message"));
					SendPingReply(pPingReply,tempip);
				}
				else                                                            // if the packet is unknown message
				{
					xSerialPrintf_P(PSTR(": Unknown message (type = 0x%02X)"), pPingReply->type);
					(*log).UnknownMSG++;
				}
			}
			else if(getSn_SR(s)==SOCK_CLOSED) 				// if it is occurred to fail to send arp packet
			{
				(*log).ARPErr++;
				close(s);									// close the pinging socket

				setSn_PROTO( s, IP_PROTO_ICMP);             // reopen the pinging socket
				if(socket( s, Sn_MR_IPRAW, 3000, 0) == 0 )
					{
					vPortFree(pPingRequest);
					vPortFree(pPingReply);
					return 0;
					}
				break;
			}
			if((xTaskGetTickCount() >= (xInitialTick + ( time / portTICK_PERIOD_MS ))) && (IsReceived == 0))					// If it is not received packet from the specified peer during waiting ping-reply packet.
			{
				(*log).Loss++;
				xSerialPrint_P(PSTR("Request timed out\r\n"));
			}

		}
	}

	/* Release pinging socket */
	setSn_PROTO(s,0);
	close(s);
	vPortFree(pPingRequest);
	vPortFree(pPingReply);
	return 1;
}


/**
 @brief 	Display result of ping
 */
void DisplayPingStatistics(
	PING_LOG log		/**< result of ping */
	)
{
	xSerialPrintf_P(PSTR("\r\n    Packets: Sent = %d, Received = %d, Lost = %d\r\n"),
    		log.PingRequest,log.PingReply+log.CheckSumErr+log.UnknownMSG+log.UnreachableMSG+log.TimeExceedMSG,log.Loss+log.ARPErr);
	if(log.CheckSumErr > 0)
		xSerialPrintf_P(PSTR("    Checksum Error packets = %d\r\n"),log.CheckSumErr);
	if(log.UnreachableMSG > 0)
		xSerialPrintf_P(PSTR("    Unreachable Message packets = %d\r\n"),log.UnreachableMSG);
	if(log.TimeExceedMSG > 0)
		xSerialPrintf_P(PSTR("    Time Exceeded Message packets = %d\r\n"),log.TimeExceedMSG);
	if(log.UnknownMSG > 0)
		xSerialPrintf_P(PSTR("    Unknown Message packets = %d\r\n"),log.UnknownMSG);
	if(log.ARPErr > 0)
		xSerialPrintf_P(PSTR("    Fail To Send ARP packets = %d\r\n"),log.ARPErr);
	if(log.Loss > 0)
		xSerialPrintf_P(PSTR("    Request timed out = %d\r\n"),log.Loss);
	if(log.PingReply > 0)
		xSerialPrintf_P(PSTR("    Ping Reply packets = %d\r\n"),log.PingReply);
}

/**
 @brief 	Send ping reply packet to the specified peer

 If Error is occurred in sending ping reply packet, then ignored\n
 Because it is not need to send ping reply packet to the specified peer every time.

 */
static void SendPingReply(
	PING_MSG *pingrequest,	/**< received ping reply packet from the specified peer */
	uint32_t destaddr		/**< 32bit peer ip address (network ordering) */
	)
{
	SOCKET PingReplySocket;

	PingReplySocket = getSocket(SOCK_CLOSED,0);	// Find free socket number
	if(PingReplySocket != _WIZCHIP_MAX_SOC_NUM_ )        // Is there a free socket?
	{
		setSn_PROTO( PingReplySocket, IP_PROTO_ICMP); 	    		// Set upper protocol of IP_RAW mode
		if( socket( PingReplySocket, Sn_MR_IPRAW, 3001, 0) != 0)	// Open ICMP socket
		{
			(*pingrequest).type = 0;				// Ping-Reply
			(*pingrequest).code = 0;                // Always 0
  			(*pingrequest).checksum = 0;
			(*pingrequest).checksum = htons( checksum((uint8_t*)pingrequest, sizeof(PING_MSG)));	// Calculate checksum

			if(sendto( PingReplySocket, (uint8_t *)pingrequest, sizeof(PING_MSG), (uint8_t*)&destaddr, 3001) == 0)  // sent ping-reply packet to the specified peer
				xSerialPrintf_P(PSTR("Fail to send ping-reply packet to %s. Send failure"),inet_ntoa(ntohl(destaddr)));

			close(PingReplySocket);				// Close ICMP socket
		}

		setSn_PROTO(PingReplySocket,0);		// Clear IP protocol register.
	}
	else	xSerialPrintf_P(PSTR("Fail to send ping-reply packet to %s. NO FREE SOCKET"),inet_ntoa(ntohl(destaddr)));
}

#endif // #if   defined(_WIZCHIP_)		// Definition in freeRTOSBoardDefs.h
