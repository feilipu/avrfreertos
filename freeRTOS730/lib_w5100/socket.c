/*
*
@file		socket.c
@brief	setting chip register for socket
		last update : 2008. Jan
*
*/

#include <avr/pgmspace.h>

#include <w5100.h>
#include <socket.h>

#ifdef __DEF_W5100_DBG__
/* Scheduler include files. */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include <lib_serial.h>
#endif




static uint16_t local_port;

/**
@brief	This Socket function initialise the channel in particular mode, and set the port and wait for W5100 done it.
@return 	1 for success else 0.
*/
uint8_t socket(
	SOCKET s, 		/**< for socket number */
	uint8_t protocol, /**< for socket protocol */
	uint16_t port, 	/**< the source port for the socket */
	uint8_t flag		/**< the option for the socket */
	)
{
	uint8_t ret;
#ifdef __DEF_W5100_DBG__
	xSerialPrint_P(PSTR(" socket()\r\n"));
#endif
	if ((protocol == Sn_MR_TCP) || (protocol == Sn_MR_UDP) || (protocol == Sn_MR_IPRAW) || (protocol == Sn_MR_MACRAW) || (protocol == Sn_MR_PPPOE))
	{
		close(s);
		W5100_WRITE(Sn_MR(s), (protocol | flag) );
		if (port != 0) {
			W5100_WRITE(Sn_PORT0(s),(uint8_t)((port & 0xff00) >> 8));
			W5100_WRITE(Sn_PORT1(s),(uint8_t)(port & 0x00ff));
		} else {
			local_port++; // if don't set the source port, set local_port number.
			W5100_WRITE(Sn_PORT0(s),(uint8_t)((local_port & 0xff00) >> 8));
			W5100_WRITE(Sn_PORT1(s),(uint8_t)(local_port & 0x00ff));
		}
		W5100_WRITE(Sn_CR(s),Sn_CR_OPEN); // run sock init Sn_CR

		/* +20071122[chungs]:wait to process the command... */
		while( W5100_READ(Sn_CR(s)) );

		/* ------- */
		ret = 1;
	}
	else
	{
		ret = 0;
	}
#ifdef __DEF_W5100_DBG__
	xSerialPrintf_P(PSTR("Sn_SR = %.2x , Protocol = %.2x\r\n"), W5100_READ(Sn_SR(s)), W5100_READ(Sn_MR(s)));
#endif
	return ret;
}


/**
@brief	This function close the socket and parameter is "s" which represent the socket number
*/
void close(SOCKET s)
{
#ifdef __DEF_W5100_DBG__
	xSerialPrint_P(PSTR(" close()\r\n"));
#endif

	W5100_WRITE(Sn_CR(s),Sn_CR_CLOSE);

	/* +20071122[chungs]:wait to process the command... */
	while( W5100_READ(Sn_CR(s)) );
	/* ------- */

	/* +2008.01 [hwkim]: clear interrupt */
	#ifdef __DEF_W5100_INT__
      /* m2008.01 [bj] : all clear */
	       putISR(s, 0x00);
	#else
      /* m2008.01 [bj] : all clear */
		W5100_WRITE(Sn_IR(s), 0xFF);
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

#ifdef __DEF_W5100_DBG__
	xSerialPrint_P(PSTR(" listen()\r\n"));
#endif

	if (W5100_READ(Sn_SR(s)) == SOCK_INIT)
	{
		W5100_WRITE(Sn_CR(s),Sn_CR_LISTEN);

		/* +20071122[chungs]:wait to process the command... */
		while( W5100_READ(Sn_CR(s)) );
		/* ------- */

		ret = 1;
	}
	else
	{
		ret = 0;
#ifdef __DEF_W5100_DBG__
	xSerialPrint_P(PSTR("Fail[invalid ip,port]\r\n"));
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
#ifdef __DEF_W5100_DBG__
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
#ifdef __DEF_W5100_DBG__
	xSerialPrint_P(PSTR("Fail[invalid ip,port]\r\n"));
#endif
	}
	else
	{
		ret = 1;
		// set destination IP
		W5100_WRITE( Sn_DIPR0(s)     ,addr[0]);
		W5100_WRITE((Sn_DIPR0(s) + 1),addr[1]);
		W5100_WRITE((Sn_DIPR0(s) + 2),addr[2]);
		W5100_WRITE((Sn_DIPR0(s) + 3),addr[3]);
		W5100_WRITE( Sn_DPORT0(s),(uint8_t)((port & 0xff00) >> 8));
		W5100_WRITE((Sn_DPORT0(s) + 1),(uint8_t)(port & 0x00ff));

		// m2012.03.13 [ys]: set/clear subnet for ARP Errata #2 and #3
        applySUBR();

		W5100_WRITE( Sn_CR(s),Sn_CR_CONNECT);

		/* m2008.01 [bj] :  wait for completion */
		while ( W5100_READ(Sn_CR(s)) );

		// m2012.03.13 [ys]: clear subnet for ARP Errata #2 and #3
        clearSUBR();
	}
	return ret;
}



/**
@brief	This function used for disconnect the socket and parameter is "s" which represent the socket number
@return	1 for success else 0.
*/
void disconnect(SOCKET s)
{
#ifdef __DEF_W5100_DBG__
	xSerialPrint_P(PSTR("disconnect()\r\n"));
#endif
	W5100_WRITE(Sn_CR(s),Sn_CR_DISCON);

	/* +20071122[chungs]:wait to process the command... */
	while( W5100_READ(Sn_CR(s)) );
	/* ------- */
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
	uint8_t status;
	uint16_t ret;
	uint16_t freesize;
#ifdef __DEF_W5100_DBG__
	xSerialPrint_P(PSTR(" send()\r\n"));
#endif

   if (len > getW5100_TxMAX(s)) ret = getW5100_TxMAX(s); // check size not to exceed MAX size.
   else ret = len;

   // if free buffer is available, start.
	do
	{
		freesize = getSn_TX_FSR(s);
		status = W5100_READ(Sn_SR(s));
		if ((status != SOCK_ESTABLISHED) && (status != SOCK_CLOSE_WAIT))
		{
			ret = 0;
			break;
		}
#ifdef __DEF_W5100_DBG__
		xSerialPrintf_P(PSTR("socket %d freesize(%d) empty or error\r\n"), s, freesize);
#endif
	} while (freesize < ret);

    // copy data
	send_data_processing(s, (uint8_t *)buf, ret);
	W5100_WRITE(Sn_CR(s),Sn_CR_SEND);

	/* +20071122[chungs]:wait to process the command... */
		while( W5100_READ(Sn_CR(s)) );
	/* ------- */

/* +2008.01 bj */
#ifdef __DEF_W5100_INT__
	while ( (getISR(s) & Sn_IR_SEND_OK) != Sn_IR_SEND_OK )
#else
	while ( (W5100_READ(Sn_IR(s)) & Sn_IR_SEND_OK) != Sn_IR_SEND_OK )
#endif
	{
		/* m2008.01 [bj] : reduce code */
		if ( W5100_READ(Sn_SR(s)) == SOCK_CLOSED )
		{
#ifdef __DEF_W5100_DBG__
			xSerialPrint_P(PSTR("SOCK_CLOSED.\r\n"));
#endif
			close(s);
			return 0;
		}
  	}
/* +2008.01 bj */
#ifdef __DEF_W5100_INT__
  	putISR(s, getISR(s) & (~Sn_IR_SEND_OK));
#else
	W5100_WRITE(Sn_IR(s), Sn_IR_SEND_OK);
#endif
  	return ret;
}


/**
@brief	This function is an application I/F function which is used to receive the data in TCP mode.
		It continues to wait for data as much as the application wants to receive.

@return	received data size for success else -1.
*/
uint16_t recv(
	SOCKET s, 	/**< socket index */
	uint8_t * buf, 	/**< a pointer to copy the data to be received */
	uint16_t len	/**< the data size to be read */
	)
{
	uint16_t ret = 0;

#ifdef __DEF_W5100_DBG2__
	xSerialPrint_P(PSTR(" recv()\r\n"));
#endif

	if ( len > 0 )
	{
		recv_data_processing(s, buf, len);
		W5100_WRITE(Sn_CR(s),Sn_CR_RECV);

		// +20071122[chungs]:wait to process the command...
		// fixme removed by Phillip, as it doesn't seem to release.
//		while( W5100_READ(Sn_CR(s)) );

		/* ------- */
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
	SOCKET s, 		/**< socket index */
	const uint8_t * buf, 	/**< a pointer to the data */
	uint16_t len, 	/**< the data size to send */
	uint8_t * addr, 	/**< the peer's Destination IP address */
	uint16_t port		/**< the peer's destination port number */
	)
{

	uint16_t ret = 0;

#ifdef __DEF_W5100_DBG__
	xSerialPrint_P(PSTR(" sendto()\r\n"));
#endif
   if (len > getW5100_TxMAX(s)) ret = getW5100_TxMAX(s); // check size not to exceed MAX size.
   else ret = len;

	if
		(
		 	((addr[0] == 0x00) && (addr[1] == 0x00) && (addr[2] == 0x00) && (addr[3] == 0x00)) ||
		 	(port == 0x00) || (ret == 0)
		)
 	{
 	   /* +2008.01 [bj] : added return value */
 	   ret = 0;

#ifdef __DEF_W5100_DBG__
	xSerialPrintf_P(PSTR("%d Fail[%.2x.%.2x.%.2x.%.2x, %.d, %.4x]\r\n"),s, addr[0], addr[1], addr[2], addr[3] , port, len);
	xSerialPrint_P(PSTR("Fail[invalid ip,port]\r\n"));
#endif
	}
	else
	{
		W5100_WRITE((Sn_DIPR0(s)    ),addr[0]);
		W5100_WRITE((Sn_DIPR0(s) + 1),addr[1]);
		W5100_WRITE((Sn_DIPR0(s) + 2),addr[2]);
		W5100_WRITE((Sn_DIPR0(s) + 3),addr[3]);
		W5100_WRITE(Sn_DPORT0(s),(uint8_t)((port & 0xff00) >> 8));
		W5100_WRITE(Sn_DPORT1(s),(uint8_t)(port & 0x00ff));

  		// copy data
  		send_data_processing(s, (uint8_t *)buf, ret);

		// m2012.03.13 [ys]: set subnet for ARP Errata #2 and #3
  		applySUBR();

		W5100_WRITE(Sn_CR(s),Sn_CR_SEND);

		/* +20071122[chungs]:wait to process the command... */
		while( W5100_READ(Sn_CR(s)) );
		/* ------- */

		// m2012.03.13 [ys]: clear subnet for ARP Errata  #2 and #3
	    clearSUBR();

/* +2008.01 bj */
#ifdef __DEF_W5100_INT__
   	while ( (getISR(s) & Sn_IR_SEND_OK) != Sn_IR_SEND_OK )
#else
	   while ( (W5100_READ(Sn_IR(s)) & Sn_IR_SEND_OK) != Sn_IR_SEND_OK )
#endif
		{
#ifdef __DEF_W5100_INT__
      	if (getISR(s) & Sn_IR_TIMEOUT)
#else
	      if (W5100_READ(Sn_IR(s)) & Sn_IR_TIMEOUT)
#endif
			{
#ifdef __DEF_W5100_DBG__
				xSerialPrint_P(PSTR("send fail.\r\n"));
#endif
/* +2008.01 [bj]: clear interrupt */
#ifdef __DEF_W5100_INT__
         	putISR(s, getISR(s) & ~(Sn_IR_SEND_OK | Sn_IR_TIMEOUT));  /* clear SEND_OK & TIMEOUT */
#else
         	W5100_WRITE(Sn_IR(s), (Sn_IR_SEND_OK | Sn_IR_TIMEOUT)); /* clear SEND_OK & TIMEOUT */
#endif
			return 0;
			}
		}

/* +2008.01 bj */
#ifdef __DEF_W5100_INT__
     	putISR(s, getISR(s) & (~Sn_IR_SEND_OK));
#else
	   W5100_WRITE(Sn_IR(s), Sn_IR_SEND_OK);
#endif

#ifdef __DEF_W5100_DBG__
	xSerialPrintf_P(PSTR("%d Pass[%.2x.%.2x.%.2x.%.2x, %.d, %.4x]\r\n"),s, addr[0], addr[1], addr[2], addr[3] , port, ret);
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
	SOCKET s, 	/**< the socket number */
	uint8_t * buf, 	/**< a pointer to copy the data to be received */
	uint16_t len, 	/**< the data size to read */
	uint8_t * addr, 	/**< a pointer to store the peer's IP address */
	uint16_t *port	/**< a pointer to store the peer's port number. */
	)
{
	uint8_t head[8];
	uint16_t data_len = 0;
	uint16_t ptr;
#ifdef __DEF_W5100_DBG__
	xSerialPrint_P(PSTR(" recvfrom()\r\n"));
#endif

	if ( len > 0 )
	{
   	ptr = W5100_READ(Sn_RX_RD0(s));
   	ptr = ((ptr & 0x00ff) << 8) + W5100_READ(Sn_RX_RD1(s));
#ifdef __DEF_W5100_DBG__
   	xSerialPrintf_P(PSTR("ISR_RX: rd_ptr: %.4x rd_len: %.4x\r\n"), ptr, len);
#endif
   	switch (W5100_READ(Sn_MR(s)) & 0x07)
   	{
   	case Sn_MR_UDP :
   			read_data(s, (uint8_t *)ptr, head, 0x08);
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

#ifdef __DEF_W5100_DBG__
   			xSerialPrint_P(PSTR("UDP msg arrived\r\n"));
   			xSerialPrintf_P(PSTR("source Port : %d\r\n"), *port);
   			xSerialPrintf_P(PSTR("source IP : %d.%d.%d.%d\r\n"), addr[0], addr[1], addr[2], addr[3]);
#endif

			read_data(s, (uint8_t *)ptr, buf, data_len); // data copy.
			ptr += data_len;

			W5100_WRITE(Sn_RX_RD0(s),(uint8_t)((ptr & 0xff00) >> 8));
			W5100_WRITE(Sn_RX_RD1(s),(uint8_t)(ptr & 0x00ff));
   			break;

   	case Sn_MR_IPRAW :
   			read_data(s, (uint8_t *)ptr, head, 0x06);
   			ptr += 6;

   			addr[0] = head[0];
   			addr[1] = head[1];
   			addr[2] = head[2];
   			addr[3] = head[3];
   			data_len = head[4];
   			data_len = (data_len << 8) + head[5];

#ifdef __DEF_W5100_DBG__
   			xSerialPrint_P(PSTR("IP RAW msg arrived\r\n"));
   			xSerialPrintf_P(PSTR("source IP : %d.%d.%d.%d\r\n"), addr[0], addr[1], addr[2], addr[3]);
#endif
			read_data(s, (uint8_t *)ptr, buf, data_len); // data copy.
			ptr += data_len;

			W5100_WRITE(Sn_RX_RD0(s),(uint8_t)((ptr & 0xff00) >> 8));
			W5100_WRITE(Sn_RX_RD1(s),(uint8_t)(ptr & 0x00ff));
   			break;
   	case Sn_MR_MACRAW :
   			read_data(s,(uint8_t*)ptr,head,2);
   			ptr+=2;
   			data_len = head[0];
   			data_len = (data_len<<8) + head[1] - 2;

   			read_data(s,(uint8_t*) ptr,buf,data_len);
   			ptr += data_len;
   			W5100_WRITE(Sn_RX_RD0(s),(uint8_t)((ptr & 0xff00) >> 8));
   			W5100_WRITE(Sn_RX_RD1(s),(uint8_t)(ptr & 0x00ff));

#ifdef __DEF_W5100_DBG__
			xSerialPrint_P(PSTR("MAC RAW msg arrived\r\n"));
			xSerialPrintf_P(PSTR("dest mac=%.2X.%.2X.%.2X.%.2X.%.2X.%.2X\r\n"),buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);
			xSerialPrintf_P(PSTR("src  mac=%.2X.%.2X.%.2X.%.2X.%.2X.%.2X\r\n"),buf[6],buf[7],buf[8],buf[9],buf[10],buf[11]);
			xSerialPrintf_P(PSTR("type    =%.2X%.2X\r\n"),buf[12],buf[13]);
#endif
			break;

   	default :
   			break;
   	}
		W5100_WRITE(Sn_CR(s),Sn_CR_RECV);

		/* +20071122[chungs]:wait to process the command... */
		while( W5100_READ(Sn_CR(s)) );
		/* ------- */
	}
#ifdef __DEF_W5100_DBG__
	xSerialPrint_P(PSTR(" recvfrom() OK ..!\r\n"));
#endif
 	return data_len;
}


uint16_t igmpsend(SOCKET s, const uint8_t * buf, uint16_t len)
{
	uint8_t status;
	uint16_t ret;

#ifdef __DEF_W5100_DBG__
	xSerialPrint_P(PSTR(" igmpsend()\r\n"));
#endif
   if (len > getW5100_TxMAX(s)) ret = getW5100_TxMAX(s); // check size not to exceed MAX size.
   else ret = len;

	if	(ret == 0)
 	{
 	   ;
#ifdef __DEF_W5100_DBG__
	xSerialPrintf_P(PSTR("%d Fail[%d]\r\n"),len);
#endif
	}
	else
	{
		// copy data
		send_data_processing(s, (uint8_t *)buf, ret);
		W5100_WRITE(Sn_CR(s),Sn_CR_SEND);

		/* +20071122[chungs]:wait to process the command... */
		while( W5100_READ(Sn_CR(s)) );
		/* ------- */

/* +2008.01 bj */
#ifdef __DEF_W5100_INT__
   	while ( (getISR(s) & Sn_IR_SEND_OK) != Sn_IR_SEND_OK )
#else
	   while ( (W5100_READ(Sn_IR(s)) & Sn_IR_SEND_OK) != Sn_IR_SEND_OK )
#endif
		{
			status = W5100_READ(Sn_SR(s));
#ifdef __DEF_W5100_INT__
      	if (getISR(s) & Sn_IR_TIMEOUT)
#else
	      if (W5100_READ(Sn_IR(s)) & Sn_IR_TIMEOUT)
#endif
			{
#ifdef __DEF_W5100_DBG__
				xSerialPrint_P(PSTR(" igmpsend fail.\r\n"));
#endif
			   /* in case of igmp, if send fails, then socket closed */
			   /* if you want change, remove this code. */
			   close(s);
			   /* ----- */

				return 0;
			}
		}

/* +2008.01 bj */
#ifdef __DEF_W5100_INT__
     	putISR(s, getISR(s) & (~Sn_IR_SEND_OK));
#else
	   W5100_WRITE(Sn_IR(s), Sn_IR_SEND_OK);
#endif
   }
	return ret;
}

