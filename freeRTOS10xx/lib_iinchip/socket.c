/*
*
@file		socket.c
@brief	setting chip register for socket
		last update : 2008. Jan
*
*/

#include <avr/pgmspace.h>
#include <util/delay.h>

#include "socket.h"
#include "wizchip_conf.h"

#ifdef __DEF_WIZCHIP_DBG__
/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "serial.h"
#endif

#if   defined(_WIZCHIP_)		// Definition in freeRTOSBoardDefs.h

static uint16_t local_port;

/**
@brief	This Socket function initialise the channel in particular mode, and set the port and wait for W5200/W5100 to complete its state change.
@return 1 for success else 0.
*/
uint8_t socket(
	SOCKET s, 			/**< for socket number */
	uint8_t protocol,	/**< for socket protocol */
	uint16_t port,		/**< the source port for the socket */
	uint8_t flag		/**< the option for the socket */
	)
{
	uint8_t ret;
#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrint_P(PSTR(" socket()\r\n"));
#endif
	if (
		(protocol == Sn_MR_TCP) ||
		(protocol == Sn_MR_UDP) ||
		(protocol == Sn_MR_IPRAW) ||
		(protocol == Sn_MR_MACRAW) ||
		(protocol == Sn_MR_PPPOE)
		)
	{
		close(s);

		WIZCHIP_write(Sn_MR(s), (protocol | flag) );

		if (port != 0) {
			WIZCHIP_write(Sn_PORT0(s), (uint8_t)((port & 0xff00) >> 8));
			WIZCHIP_write(Sn_PORT1(s), (uint8_t)(port & 0x00ff));
		} else {
			local_port++; // if don't set the source port, set local_port number.
			WIZCHIP_write(Sn_PORT0(s), (uint8_t)((local_port & 0xff00) >> 8));
			WIZCHIP_write(Sn_PORT1(s), (uint8_t)(local_port & 0x00ff));
		}
		WIZCHIP_write(Sn_CR(s), Sn_CR_OPEN);

		/* wait to process the command... */
		while( WIZCHIP_read(Sn_CR(s)) ) ;

		switch (protocol)
		{
			case Sn_MR_TCP :
			/* wait to achieve the command... */
				while( WIZCHIP_read(Sn_SR(s)) != SOCK_INIT ) ;
				break;

			case Sn_MR_UDP :
			/* wait to achieve the command... */
				while( WIZCHIP_read(Sn_SR(s)) != SOCK_UDP ) ;
				break;

			case Sn_MR_IPRAW :
			/* wait to achieve the command... */
				while( WIZCHIP_read(Sn_SR(s)) != SOCK_IPRAW ) ;
				break;

			case Sn_MR_MACRAW :
			/* wait to achieve the command... */
				while( WIZCHIP_read(Sn_SR(s)) != SOCK_MACRAW ) ;
				break;

			case Sn_MR_PPPOE : //
			/* wait to achieve the command... */
				while( WIZCHIP_read(Sn_SR(s)) != SOCK_PPPOE ) ;
				break;

			default:
				break;
		}

		ret = 1;
	}
	else
	{
#ifdef __DEF_WIZCHIP_DBG__
		xSerialPrintf_P(PSTR(" Sn_SR = %.2x , Bad Protocol"), WIZCHIP_read(Sn_SR(s) ));
#endif
		ret = 0;
	}
#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrintf_P(PSTR(" Sn_SR = %.2x , Protocol = %.2x\r\n"), WIZCHIP_read(Sn_SR(s)), WIZCHIP_read(Sn_MR(s)));
#endif
	return ret;
}

/**
@brief	This function close the socket and parameter is "s" which represent the socket number
*/
void close(SOCKET s)
{
#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrint_P(PSTR(" close()\r\n"));
#endif

	WIZCHIP_write(Sn_CR(s), Sn_CR_CLOSE);

	/* wait to process the command... */
	while( WIZCHIP_read(Sn_CR(s)) ) ;

	/* wait to achieve the right state */
	while( WIZCHIP_read(Sn_SR(s)) != SOCK_CLOSED ) ;

	/* clear interrupt */
	#ifdef __DEF_WIZCHIP_INT__
      /* all clear */
	       putISR(s, 0x00);
	#else
      /* all clear */
		WIZCHIP_write(Sn_IR(s), 0xFF);
	#endif
}


/**
@brief	This function establishes the connection for the channel in passive (server) mode.
		This function waits for the request from the peer.
@return	1 for success else 0.
*/
uint8_t listen(
	SOCKET s	/**< the socket number */
	)
{
	uint8_t ret;

#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrint_P(PSTR(" listen()\r\n"));
#endif

	if (WIZCHIP_read(Sn_SR(s)) == SOCK_INIT)
	{
		WIZCHIP_write(Sn_CR(s), Sn_CR_LISTEN);

		/* wait to process the command... */
		while( WIZCHIP_read(Sn_CR(s)) ) ;

		/* wait to action the command... */
		while( WIZCHIP_read(Sn_SR(s)) != SOCK_LISTEN ) ;

		ret = 1;
	}
	else
	{
		ret = 0;
#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrint_P(PSTR(" ...fail [invalid ip, port]\r\n"));
#endif
	}
	return ret;
}


/**
@brief	This function established the connection for the channel in Active (client) mode.
		This function waits for the until the connection is established.

@return	1 for success else 0.
*/
uint8_t connect(SOCKET s, uint8_t * addr, uint16_t port)
{
	uint8_t ret;
#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrint_P(PSTR(" connect()\r\n"));
#endif
	if
		(
			((addr[0] == 0xFF) && (addr[1] == 0xFF) && (addr[2] == 0xFF) && (addr[3] == 0xFF)) ||
		 	((addr[0] == 0x00) && (addr[1] == 0x00) && (addr[2] == 0x00) && (addr[3] == 0x00)) ||
		 	(port == 0x00)
		)
 	{
 		ret = 0;
#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrint_P(PSTR(" ...fail [invalid ip, port]\r\n"));
#endif
	}
	else
	{
		ret = 1;
		// set destination IP
		WIZCHIP_write(Sn_DIPR0(s),  addr[0]);
		WIZCHIP_write(Sn_DIPR1(s),  addr[1]);
		WIZCHIP_write(Sn_DIPR2(s),  addr[2]);
		WIZCHIP_write(Sn_DIPR3(s),  addr[3]);
		WIZCHIP_write( Sn_DPORT0(s), (uint8_t)((port & 0xff00) >> 8));
		WIZCHIP_write( Sn_DPORT1(s), (uint8_t) (port & 0x00ff));

#if  (_WIZCHIP_ <= 5200)
		setSUBR();	 // set the subnet mask register because of the ARP errata
#endif

		WIZCHIP_write(Sn_CR(s), Sn_CR_CONNECT);

        /* wait for completion */
		while ( WIZCHIP_read(Sn_CR(s)) ) ;

		while ( WIZCHIP_read(Sn_SR(s)) != SOCK_SYNSENT )
		{
			if(WIZCHIP_read(Sn_SR(s)) == SOCK_ESTABLISHED)
			{
				break;
			}
			if (getSn_IR(s) & Sn_IR_TIMEOUT)
			{
				WIZCHIP_write(Sn_IR(s), (Sn_IR_TIMEOUT));  // clear TIMEOUT Interrupt
				ret = 0;
				break;
			}
		}
#if  (_WIZCHIP_ <= 5200)
		clearSUBR();	// clear the subnet mask again and keep it because of the ARP errata
#endif
	}
	return ret;
}



/**
@brief	This function used for disconnect the socket and parameter is "s" which represent the socket number

*/
void disconnect(SOCKET s)
{
#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrint_P(PSTR(" disconnect()\r\n"));
#endif
	WIZCHIP_write(Sn_CR(s), Sn_CR_DISCON);

	/* wait to process the command... */
	while( WIZCHIP_read(Sn_CR(s)) ) ;

	/* wait to action the command... */
	while( WIZCHIP_read(Sn_SR(s)) != SOCK_CLOSED ) ;
}


/**
@brief	This function used to send the data in TCP mode
@return	bytes transmitted length for success else 0 for failure.
*/
uint16_t send(
	SOCKET s, 				/**< the socket index */
	const uint8_t * buf, 	/**< a pointer to data */
	uint16_t len			/**< the data size to be send */
	)
{
	uint8_t status = 0;
	uint16_t ret = 0;
	uint16_t freesize = 0;
	uint16_t txrd, txrd_before_send;
#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrint_P(PSTR(" send()\r\n"));
#endif

	if (len > WIZCHIP_getTxMAX(s))
		ret = WIZCHIP_getTxMAX(s); // check size not to exceed MAX size.
	else
		ret = len;

	// if free buffer is available, start.
	do
	{
		freesize = getSn_TX_FSR(s);
		status = WIZCHIP_read(Sn_SR(s));
		if ((status != SOCK_ESTABLISHED) && (status != SOCK_CLOSE_WAIT))
		{
			ret = 0;
			break;
		}
#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrintf_P(PSTR(" socket %d freesize(%d) empty or error\r\n"), s, freesize);
#endif
	} while (freesize < ret);

	// copy data
	WIZCHIP_send_data_processing(s, (uint8_t *)buf, ret);

	if(ret != 0)
	{
		txrd_before_send = WIZCHIP_read(Sn_TX_RD0(s));
		txrd_before_send = (txrd_before_send << 8) + WIZCHIP_read(Sn_TX_RD1(s));

		WIZCHIP_write(Sn_CR(s), Sn_CR_SEND);

		/* wait to process the command... */
		while( WIZCHIP_read(Sn_CR(s)) ) ;

#ifdef __DEF_WIZCHIP_INT__
		while ( (getISR(s) & Sn_IR_SEND_OK) != Sn_IR_SEND_OK )
#else
		while ( (WIZCHIP_read(Sn_IR(s)) & Sn_IR_SEND_OK) != Sn_IR_SEND_OK )
#endif
        {
			if( WIZCHIP_read(Sn_SR(s)) == SOCK_CLOSED )
			{
#ifdef __DEF_WIZCHIP_DBG__
				xSerialPrint_P(PSTR(" SOCK_CLOSED\r\n"));
#endif
				close(s);
				return 0;
			}
		}
#ifdef __DEF_WIZCHIP_INT__
  		putISR(s, getISR(s) & (~Sn_IR_SEND_OK));
#else
		WIZCHIP_write(Sn_IR(s), Sn_IR_SEND_OK);
#endif
		txrd = WIZCHIP_read(Sn_TX_RD0(s));
		txrd = (txrd << 8) + WIZCHIP_read(Sn_TX_RD1(s));

		if(txrd > txrd_before_send) {
			ret = txrd - txrd_before_send;
		} else {
			ret = (0xffff - txrd_before_send) + txrd + 1;
		}
	}

	return ret;
}


/**
@brief	This function is an application I/F function which is used to receive the data in TCP mode.
		It continues to wait for data as much as the application wants to receive.

@return	received data size for success else 0.
*/
uint16_t recv(
	SOCKET s, 	/**< socket index */
	uint8_t * buf, 	/**< a pointer to copy the data to be received */
	uint16_t len	/**< the data size to be read */
	)
{
	uint16_t ret = 0;

#ifdef __DEF_WIZCHIP_DBG2__
	xSerialPrint_P(PSTR(" recv()\r\n"));
#endif

	if ( len > 0 )
	{
		WIZCHIP_recv_data_processing(s, buf, len);
		WIZCHIP_write(Sn_CR(s), Sn_CR_RECV);

		/* wait to process the command... */
		while( WIZCHIP_read(Sn_CR(s)) ) ;

		ret = len;
	}
	return ret;
}


/**
@brief	This function is an application I/F function which is used to send the data for other than TCP mode.
		Unlike TCP transmission, The peer's destination address and the port is needed.

@return	This function return send data size for success else 0.
*/
uint16_t sendto(
	SOCKET s, 				/**< socket index */
	const uint8_t * buf, 	/**< a pointer to the data */
	uint16_t len, 			/**< the data size to send */
	uint8_t * addr, 		/**< the peer's Destination IP address */
	uint16_t port			/**< the peer's destination port number */
	)
{
	uint16_t ret = 0;

#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrint_P(PSTR(" sendto()\r\n"));
#endif

	if (len > WIZCHIP_getTxMAX(s))
		ret = WIZCHIP_getTxMAX(s); // check size not to exceed MAX size.
   else
	   ret = len;

	if
		(
		 	((addr[0] == 0x00) && (addr[1] == 0x00) && (addr[2] == 0x00) && (addr[3] == 0x00)) ||
		 	(port == 0x00)
		)
 	{
#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrintf_P(PSTR(" %d Fail [%.2x.%.2x.%.2x.%.2x, %.d, %.4x]\r\n"),s, addr[0], addr[1], addr[2], addr[3] , port, len);
	xSerialPrint_P(PSTR(" Fail [invalid ip, port, len]\r\n"));
#endif
 	   /* added return value */
 	   ret = 0;
	}
	else
	{
		WIZCHIP_write(Sn_DIPR0(s), addr[0]);
		WIZCHIP_write(Sn_DIPR1(s), addr[1]);
		WIZCHIP_write(Sn_DIPR2(s), addr[2]);
		WIZCHIP_write(Sn_DIPR3(s), addr[3]);
		WIZCHIP_write(Sn_DPORT0(s), (uint8_t)((port & 0xff00) >> 8));
		WIZCHIP_write(Sn_DPORT1(s), (uint8_t)(port & 0x00ff));

  		// copy data
 		WIZCHIP_send_data_processing(s, (uint8_t *)buf, ret);

#if   (_WIZCHIP_ <= 5200)
		setSUBR();    // set the subnet mask register because of the ARP errata
#endif

		WIZCHIP_write(Sn_CR(s), Sn_CR_SEND);

		/* wait to process the command... */
		while( WIZCHIP_read(Sn_CR(s)) ) ;

#ifdef __DEF_WIZCHIP_INT__
   	while ( (getISR(s) & Sn_IR_SEND_OK) != Sn_IR_SEND_OK )
#else
	   while ( (WIZCHIP_read(Sn_IR(s)) & Sn_IR_SEND_OK) != Sn_IR_SEND_OK )
#endif
		{
#ifdef __DEF_WIZCHIP_INT__
      	if (getISR(s) & Sn_IR_TIMEOUT)
#else
	      if (WIZCHIP_read(Sn_IR(s)) & Sn_IR_TIMEOUT)
#endif
			{

#ifdef __DEF_WIZCHIP_DBG__
				xSerialPrint_P(PSTR(" ...sendto() fail.\r\n"));
#endif
			/* clear interrupt */
#ifdef __DEF_WIZCHIP_INT__
         	putISR(s, getISR(s) & ~(Sn_IR_SEND_OK | Sn_IR_TIMEOUT));  /* clear SEND_OK & TIMEOUT */
#else
         	WIZCHIP_write(Sn_IR(s), (Sn_IR_SEND_OK | Sn_IR_TIMEOUT)); /* clear SEND_OK & TIMEOUT */
#endif
			return 0;
			}
	   }
#if   (_WIZCHIP_ <= 5200)
	   clearSUBR();	   // clear the subnet mask again and keep it because of the ARP errata
#endif

#ifdef __DEF_WIZCHIP_INT__
     	putISR(s, getISR(s) & (~Sn_IR_SEND_OK));
#else
	   WIZCHIP_write(Sn_IR(s), Sn_IR_SEND_OK);
#endif

#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrintf_P(PSTR(" %d Pass [%.2x.%.2x.%.2x.%.2x, %.d, %.4x]\r\n"),s, addr[0], addr[1], addr[2], addr[3] , port, ret);
	xSerialPrint_P(PSTR(" ...sendto() end\r\n"));
#endif
	}
	return ret;
}


/**
@brief	This function is an application I/F function which is used to receive the data in other than
	TCP mode. This function is used to receive UDP, IP_RAW and MAC_RAW mode, and handle the header as well.

@return	This function return received data size for success else 0.
*/
uint16_t recvfrom(
	SOCKET s, 		/**< the socket number */
	uint8_t * buf, 	/**< a pointer to copy the data to be received */
	uint16_t len, 	/**< the data size to read */
	uint8_t * addr, /**< a pointer to store the peer's IP address */
	uint16_t *port	/**< a pointer to store the peer's port number. */
	)
{
	uint8_t head[8];
	uint16_t data_len = 0;
	uint16_t ptr = 0;
#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrint_P(PSTR(" recvfrom()\r\n"));
#endif

	if ( len > 0 )
	{
   	ptr = WIZCHIP_read(Sn_RX_RD0(s));
   	ptr = ((ptr & 0x00ff) << 8) + WIZCHIP_read(Sn_RX_RD1(s));
#ifdef __DEF_WIZCHIP_DBG__
   	xSerialPrintf_P(PSTR(" rd_ptr: %.4x rd_len: %.4x\r\n"), ptr, len);
#endif
   	switch (WIZCHIP_read(Sn_MR(s)) & 0x07)
   	{
   	case Sn_MR_UDP :
   			WIZCHIP_read_data(s, (uint8_t *)ptr, head, 0x08);
   			ptr += 8;
   			// read peer's IP address, port number.
    		addr[0] = head[0];
   			addr[1] = head[1];
   			addr[2] = head[2];
   			addr[3] = head[3];
   			*port = head[4];
   			*port = (*port << 8) + head[5];
   			data_len = head[6];
   			data_len = (data_len << 8) + head[7];

#ifdef __DEF_WIZCHIP_DBG__
   			xSerialPrint_P(PSTR(" UDP msg arrived\r\n"));
   			xSerialPrintf_P(PSTR(" source Port : %d\r\n"), *port);
   			xSerialPrintf_P(PSTR(" source IP : %d.%d.%d.%d\r\n"), addr[0], addr[1], addr[2], addr[3]);
#endif

			WIZCHIP_read_data(s, (uint8_t *)ptr, buf, data_len); // data copy.
			ptr += data_len;

			WIZCHIP_write(Sn_RX_RD0(s), (uint8_t)((ptr & 0xff00) >> 8));
			WIZCHIP_write(Sn_RX_RD1(s), (uint8_t)(ptr & 0x00ff));
   			break;

   	case Sn_MR_IPRAW :
   			WIZCHIP_read_data(s, (uint8_t *)ptr, head, 0x06);
   			ptr += 6;

   			addr[0] = head[0];
   			addr[1] = head[1];
   			addr[2] = head[2];
   			addr[3] = head[3];
   			data_len = head[4];
   			data_len = (data_len << 8) + head[5];

#ifdef __DEF_WIZCHIP_DBG__
   			xSerialPrint_P(PSTR(" IP RAW msg arrived\r\n"));
   			xSerialPrintf_P(PSTR(" source IP : %d.%d.%d.%d\r\n"), addr[0], addr[1], addr[2], addr[3]);
#endif
			WIZCHIP_read_data(s, (uint8_t *)ptr, buf, data_len); // data copy.
			ptr += data_len;

			WIZCHIP_write(Sn_RX_RD0(s), (uint8_t)((ptr & 0xff00) >> 8));
			WIZCHIP_write(Sn_RX_RD1(s), (uint8_t)(ptr & 0x00ff));
   			break;

   	case Sn_MR_MACRAW :
   			WIZCHIP_read_data(s,(uint8_t*)ptr,head, 0x02);
   			ptr+=2;
   			data_len = head[0];
   			data_len = (data_len<<8) + head[1] - 2;

#ifdef __DEF_WIZCHIP_DBG__
   				xSerialPrint_P(PSTR(" MAC RAW msg arrived "));
#endif
   			if(data_len > 1514)
   			{
#ifdef __DEF_WIZCHIP_DBG__
   				xSerialPrint_P(PSTR(" ...data_len over 1514"));
#endif
   				break;
   			}

   			WIZCHIP_read_data(s,(uint8_t*) ptr,buf,data_len);
   			ptr += data_len;
   			WIZCHIP_write(Sn_RX_RD0(s), (uint8_t)((ptr & 0xff00) >> 8));
   			WIZCHIP_write(Sn_RX_RD1(s), (uint8_t)(ptr & 0x00ff));

#ifdef __DEF_WIZCHIP_DBG__
			xSerialPrintf_P(PSTR("\r\n dest mac=%.2X.%.2X.%.2X.%.2X.%.2X.%.2X\r\n"),buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);
			xSerialPrintf_P(PSTR(" src  mac=%.2X.%.2X.%.2X.%.2X.%.2X.%.2X\r\n"),buf[6],buf[7],buf[8],buf[9],buf[10],buf[11]);
			xSerialPrintf_P(PSTR(" type    =%.2X%.2X\r\n"),buf[12],buf[13]);
#endif
			break;

   	default :
   			break;
   	}
		WIZCHIP_write(Sn_CR(s), Sn_CR_RECV);

		/* wait to process the command... */
		while( WIZCHIP_read(Sn_CR(s)) ) ;
	}
#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrint_P(PSTR(" ...recvfrom() end\r\n"));
#endif
 	return data_len;
}

/**
@brief	This function is an application I/F function which is used to send the data in MACRAW mode.
		There is no two byte length header on the send function.

@return	This function return send data size for success else 0.
*/

uint16_t macraw_send(
	uint8_t * buf,	/**< a pointer to copy the data to be sent */
	uint16_t len 			/**< the data size to read */
	)
{
	uint16_t ptr;
	uint16_t ret;

#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrint_P(PSTR(" macraw_send()\r\n"));
#endif

	if (len > WIZCHIP_getTxMAX(MACRAW_SOCKET))
		ret = WIZCHIP_getTxMAX(MACRAW_SOCKET); // check size not to exceed MAX size.

   else ret = len;

	if	(ret == 0)
 	{
#ifdef __DEF_WIZCHIP_DBG__
		xSerialPrintf_P(PSTR(" %d Fail[%d]\r\n"),len);
#endif
	}
	else
	{
		ptr = WIZCHIP_read(Sn_TX_WR0(MACRAW_SOCKET));
		ptr = (ptr << 8) + WIZCHIP_read(Sn_TX_WR1(MACRAW_SOCKET));

		WIZCHIP_write_data( MACRAW_SOCKET, buf, (uint8_t *)(ptr), len );
		ptr += len;

	#ifdef __DEF_WIZCHIP_DBG__
		xSerialPrintf_P(PSTR(" tx_ptr: %.4x tx_len: %.4x\r\n"), ptr, len);
	#endif

		WIZCHIP_write(Sn_TX_WR0(MACRAW_SOCKET), (uint8_t)((ptr & 0xff00) >> 8));
		WIZCHIP_write(Sn_TX_WR1(MACRAW_SOCKET), (uint8_t)(ptr & 0x00ff));

		WIZCHIP_write(Sn_CR(MACRAW_SOCKET), Sn_CR_SEND);

		/* wait to process the command... */
		while( WIZCHIP_read(Sn_CR(MACRAW_SOCKET)) ) ;

#ifdef __DEF_WIZCHIP_INT__
		while ( (getISR(MACRAW_SOCKET) & Sn_IR_SEND_OK) != Sn_IR_SEND_OK )
#else
   		while ( (WIZCHIP_read(Sn_IR(MACRAW_SOCKET)) & Sn_IR_SEND_OK) != Sn_IR_SEND_OK )
#endif
		{
#ifdef __DEF_WIZCHIP_INT__
      	if (getISR(MACRAW_SOCKET) & Sn_IR_TIMEOUT)
#else
	      if (WIZCHIP_read(Sn_IR(MACRAW_SOCKET)) & Sn_IR_TIMEOUT)
#endif
			{
#ifdef __DEF_WIZCHIP_DBG__
				xSerialPrint_P(PSTR(" ...macraw_send fail.\r\n"));
#endif
			   return 0;
			}
		}
#ifdef __DEF_WIZCHIP_INT__
		putISR(s, getISR(MACRAW_SOCKET) & (~Sn_IR_SEND_OK));
#else
		WIZCHIP_write(Sn_IR(MACRAW_SOCKET), Sn_IR_SEND_OK);
#endif
   }

#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrint_P(PSTR(" ...macraw_send() end\r\n"));
#endif
	return ret;
}


/**
@brief	This function is an application I/F function which is used to receive the data
 	 	MAC_RAW mode, and handle the 2 byte length header as well.

@return	This function return received data size for success else 0.
*/
uint16_t macraw_recv(
	uint8_t * buf, 	/**< a pointer to copy the data to be received */
	uint16_t len 	/**< the data size to read */
	)
{
	uint8_t head[2];
	uint16_t data_len = 0;
	uint16_t ptr;

#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrint_P(PSTR(" macraw_recv()\r\n"));
#endif

	if ( len > 0 )
	{
	   	ptr = WIZCHIP_read(Sn_RX_RD0(MACRAW_SOCKET));
	   	ptr = ((ptr & 0x00ff) << 8) + WIZCHIP_read(Sn_RX_RD1(MACRAW_SOCKET));

		WIZCHIP_read_data( MACRAW_SOCKET, (uint8_t*)ptr, head, 2 );
		ptr+=2;
		data_len = head[0];
		data_len = (data_len<<8) + head[1] - 2;

		if(data_len > 1514)
		{
#ifdef __DEF_WIZCHIP_DBG__
			xSerialPrintf_P(PSTR(" ...data_len over 1514: [%d]\r\n"), len);
#endif
	         /** Recommended : close and open **/
			close(MACRAW_SOCKET); // Close the MACRAW_SOCKET socket
			socket(MACRAW_SOCKET, Sn_MR_MACRAW, 0x00, 0x00);  // Open the MACRAW_SOCKET socket with MACRAW mode

			return 0;
		}

		WIZCHIP_read_data( MACRAW_SOCKET, (uint8_t*) ptr, buf, data_len );
		ptr += data_len;
		WIZCHIP_write(Sn_RX_RD0(MACRAW_SOCKET), (uint8_t)((ptr & 0xff00) >> 8));
		WIZCHIP_write(Sn_RX_RD1(MACRAW_SOCKET), (uint8_t)(ptr & 0x00ff));

#ifdef __DEF_WIZCHIP_DBG__
		xSerialPrintf_P(PSTR(" dest mac=%.2X.%.2X.%.2X.%.2X.%.2X.%.2X\r\n"),buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);
		xSerialPrintf_P(PSTR(" src  mac=%.2X.%.2X.%.2X.%.2X.%.2X.%.2X\r\n"),buf[6],buf[7],buf[8],buf[9],buf[10],buf[11]);
		xSerialPrintf_P(PSTR(" type    =%.2X%.2X\r\n"),buf[12],buf[13]);
#endif

   	}

	WIZCHIP_write(Sn_CR(MACRAW_SOCKET), Sn_CR_RECV);

	/* wait to process the command... */
	while( WIZCHIP_read(Sn_CR(MACRAW_SOCKET)) ) ;

#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrint_P(PSTR(" ...macraw_recv() end\r\n"));
#endif
 	return data_len;
}

#endif // #if   defined(_WIZCHIP_)		// Definition in freeRTOSBoardDefs.h
