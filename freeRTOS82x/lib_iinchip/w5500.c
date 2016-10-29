/*
 * (c)COPYRIGHT
 * ALL RIGHT RESERVED
 *
 * FileName : w5500.c
  * -----------------------------------------------------------------
 */
#include "wizchip_conf.h"

#if   (_WIZCHIP_ == 5500)		// Definition in freeRTOSBoardDefs.h

#include <stdio.h>
#include <string.h>

#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h> // for wait function

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "spi.h"

#include "w5500.h"

#ifdef __DEF_WIZCHIP_DBG__
/* serial interface include file. */
#include "serial.h"
#endif

#ifdef __DEF_WIZCHIP_PPP__
   #include "md5.h"
#endif

static uint8_t  I_STATUS[_WIZCHIP_MAX_SOC_NUM_];
static uint16_t SSIZE   [_WIZCHIP_MAX_SOC_NUM_]; /**< Max Tx buffer size by each channel */
static uint16_t RSIZE   [_WIZCHIP_MAX_SOC_NUM_]; /**< Max Rx buffer size by each channel */

uint8_t WIZCHIP_getISR(uint8_t s)
{
	return I_STATUS[s];
}
void WIZCHIP_putISR(uint8_t s, uint8_t val)
{
   I_STATUS[s] = val;
}
uint16_t WIZCHIP_getRxMAX(uint8_t s)
{
   return RSIZE[s];
}
uint16_t WIZCHIP_getTxMAX(uint8_t s)
{
   return SSIZE[s];
}


/**
@brief	This function writes the data into W5500 registers.
*/
void WIZCHIP_write( uint32_t addrbsb,  uint8_t data)
{
	uint8_t Byte;

	WIZCHIP_ISR_DISABLE();

	spiSelect(Wiznet);							// CS=0, get semaphore, SPI start

	spiSetDataMode(SPI_MODE0);					// Enable SPI function in mode 0, CPOL=0 CPHA=0
	spiSetClockDivider(_WIZCHIP_SPI_DIVIDER);	// SPI at maximum speed

	// If the SPI module has not been enabled yet, then return with nothing.
	if ( !(SPCR & _BV(SPE)) ) return;

	// The SPI module is enabled, but it is in slave mode, so we can not
	// transmit the byte.  This can happen if SSbar is an input and it went low.
	// We will try to recover by setting the MSTR bit.
	// Check this once only at the start. Assume that things don't change.
	if ( !(SPCR & _BV(MSTR)) ) SPCR |= _BV(MSTR);
	if ( !(SPCR & _BV(MSTR)) ) return;

	portENTER_CRITICAL();

	SPDR = (uint8_t)((addrbsb & 0x00FF0000)>>16);	// load the Address byte 1 to be transmitted
	Byte = (uint8_t)((addrbsb & 0x0000FF00)>> 8);	// pre-load the Address byte 2 to be transmitted
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;					// load the lower address to be transmitted
	Byte = (uint8_t)((addrbsb & 0x000000F8) + 0x04);// pre-load the Data Write command
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;					// load the Data Write command
	Byte = data;					// pre-load the Data (write 1 byte data) to be transmitted
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;					// load the Data (write 1 byte data)  to be transmitted
	while ( !(SPSR & _BV(SPIF)) );

	portEXIT_CRITICAL();

	spiDeselect(Wiznet);			// CS=1, SPI end, give semaphore

	WIZCHIP_ISR_ENABLE();			// Interrupt Service Routine Enable
}

/**
@brief	This function reads the value from W5500 registers.
*/
uint8_t WIZCHIP_read(uint32_t addrbsb)
{
	uint8_t Byte;

	WIZCHIP_ISR_DISABLE();

	spiSelect(Wiznet);							// CS=0, get semaphore, SPI start

	spiSetDataMode(SPI_MODE0);					// Enable SPI function in mode 0, CPOL=0 CPHA=0
	spiSetClockDivider(_WIZCHIP_SPI_DIVIDER);	// SPI at maximum speed

	// If the SPI module has not been enabled yet, then return with nothing.
	if ( !(SPCR & _BV(SPE)) ) return 0;

	// The SPI module is enabled, but it is in slave mode, so we can not
	// transmit the byte.  This can happen if SSbar is an input and it went low.
	// We will try to recover by setting the MSTR bit.
	// Check this once only at the start. Assume that things don't change.
	if ( !(SPCR & _BV(MSTR)) ) SPCR |= _BV(MSTR);
	if ( !(SPCR & _BV(MSTR)) ) return 0;

	portENTER_CRITICAL();

	SPDR = (uint8_t)((addrbsb & 0x00FF0000)>>16);	// load the Address byte 1 to be transmitted
	Byte = (uint8_t)((addrbsb & 0x0000FF00)>> 8); 	// pre-load the Address byte 2 to be transmitted
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;									// load the Address byte 2 to be transmitted
	Byte =  (uint8_t)(addrbsb & 0x000000F8); 		// pre-load the Data Read command
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;					// load the Data Read command
	Byte = 0x5A;					// pre-load a dummy byte to be transmitted
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;					// load a dummy byte to be transmitted
	while ( !(SPSR & _BV(SPIF)) );

	Byte = SPDR;					// copy received byte in case we get swapped out, and lose SPDR.

	portEXIT_CRITICAL();

	spiDeselect(Wiznet);			// CS=1, SPI end, give semaphore

	WIZCHIP_ISR_ENABLE();

	return Byte;
}

/**
@brief	This function writes into W5500 memory(Buffer)
*/
uint16_t WIZCHIP_write_buf(uint32_t addrbsb, uint8_t * buf, uint16_t len)
{
	uint16_t idx;
	uint8_t Byte;

	WIZCHIP_ISR_DISABLE();

	spiSelect(Wiznet);							// CS=0, get semaphore, SPI start

	spiSetDataMode(SPI_MODE0);					// Enable SPI function in mode 0, CPOL=0 CPHA=0
	spiSetClockDivider(_WIZCHIP_SPI_DIVIDER);	// SPI at maximum speed

	// If the SPI module has not been enabled yet, then return with nothing.
	if ( !(SPCR & _BV(SPE)) ) return 0;

	// The SPI module is enabled, but it is in slave mode, so we can not
	// transmit the byte.  This can happen if SSbar is an input and it went low.
	// We will try to recover by setting the MSTR bit.
	// Check this once only at the start. Assume that things don't change.
	if ( !(SPCR & _BV(MSTR)) ) SPCR |= _BV(MSTR);
	if ( !(SPCR & _BV(MSTR)) ) return 0;

	portENTER_CRITICAL();

	SPDR = (uint8_t)((addrbsb & 0x00FF0000)>>16);	// load the Address byte 1 to be transmitted
	Byte = (uint8_t)((addrbsb & 0x0000FF00)>> 8);	// pre-load the Address byte 2 to be transmitted
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;									// load the lower address to be transmitted
	Byte = (uint8_t)((addrbsb & 0x000000F8) + 0x04);// pre-load the Data Write command
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;					// load the Data Write command

	idx = 0;
	while(idx < len)							// Write data in loop
	{
		Byte = buf[ idx++ ];					// pre-load the byte to be transmitted
		while ( !(SPSR & _BV(SPIF)) );
		SPDR = Byte;							// load the byte to be transmitted
	}
	while ( !(SPSR & _BV(SPIF)) );

	portEXIT_CRITICAL();

	spiDeselect(Wiznet);			// CS=1, SPI end, give semaphore

    WIZCHIP_ISR_ENABLE();			// Interrupt Service Routine Enable

	return len;
}

/**
@brief	This function reads from W5500 memory(Buffer)
*/
uint16_t WIZCHIP_read_buf(uint32_t addrbsb, uint8_t* buf, uint16_t len)
{
	uint16_t idx;
	uint8_t Byte;

	WIZCHIP_ISR_DISABLE();

	spiSelect(Wiznet);							// CS=0, get semaphore, SPI start

	spiSetDataMode(SPI_MODE0);					// Enable SPI function in mode 0, CPOL=0 CPHA=0
	spiSetClockDivider(_WIZCHIP_SPI_DIVIDER);	// SPI at maximum speed

	// If the SPI module has not been enabled yet, then return with nothing.
	if ( !(SPCR & _BV(SPE)) ) return 0;

	// The SPI module is enabled, but it is in slave mode, so we can not
	// transmit the byte.  This can happen if SSbar is an input and it went low.
	// We will try to recover by setting the MSTR bit.
	// Check this once only at the start. Assume that things don't change.
	if ( !(SPCR & _BV(MSTR)) ) SPCR |= _BV(MSTR);
	if ( !(SPCR & _BV(MSTR)) ) return 0;

	portENTER_CRITICAL();

	SPDR = (uint8_t)((addrbsb & 0x00FF0000)>>16);	// load the Address byte 1 to be transmitted
	Byte = (uint8_t)((addrbsb & 0x0000FF00)>> 8); 	// pre-load the Address byte 2 to be transmitted
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;									// load the Address byte 2 to be transmitted
	Byte =  (uint8_t)(addrbsb & 0x000000F8); 		// pre-load the Data Read command
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;								// load the Data Read command
	Byte = 0x5A;								// pre-load the dummy byte.
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;								// Begin dummy transmission

	idx = 0;
	while (idx < len - 1)
	{
		while ( !(SPSR & _BV(SPIF)) );
		Byte = SPDR;							// copy received byte
		SPDR = 0x5A;							// Continue dummy transmission
		buf [ idx++ ] = Byte;
	}
	while ( !(SPSR & _BV(SPIF)) );

	buf [ idx ] = SPDR;							// store the last byte that was read

	portEXIT_CRITICAL();

	spiDeselect(Wiznet);						// CS=1, SPI end, give semaphore

	WIZCHIP_ISR_ENABLE();                       // Interrupt Service Routine Enable

	return len;
}


/**
@brief  This function is for resetting of the Wiz550io. Initializes the Wiznet W5500 via the MCU on the WizW5500io
*/
void WIZCHIP_init(void)
{

	_delay_ms(150); // AVR can't delay its boot long as the W5500 needs to, so add 150ms wait to allow the Wiz550io MCU to run its configuration.

	spiBegin(Wiznet);		// enable the EtherMega W5500

	spiSetDataMode(SPI_MODE0);					// Enable SPI function in mode 0, CPOL=0 CPHA=0
	spiSetClockDivider(_WIZCHIP_SPI_DIVIDER);	// SPI at maximum speed

	WIZCHIP_sw_reset(); // software reset the W5500 chip, and preserve only the MAC address.

#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrintf_P(PSTR("\r\nMR       (0x%02x)\r\nVERSIONR (0x%02x)\r\nPHYCFGR  (0x%02x)\r\n"), getMR(), WIZCHIP_read(VERSIONR), WIZCHIP_read(PHYCFGR) );
#endif
}

void WIZCHIP_sw_reset(void) // software reset the W5500 chip, and preserve only the MAC address.
{
//   uint8_t gw[4], sn[4], sip[4];
   uint8_t mac[6];
   getSHAR(mac);		// store the MAC address
//   getGAR(gw); getSUBR(sn); getSIPR(sip); // store the gateway IP, subnet mask, and source IP address off chip.

   setMR(MR_RST);			// RESET
   do{
	   _delay_ms(5); // Delay after reset for the W5500 needs to be (data sheet) 1ms, so wait 5ms before we address W5500.
   } while (WIZCHIP_read(VERSIONR) != 0x04);

   setSHAR(mac);		// write the MAC address back to the W5500
//   setGAR(gw);  setSUBR(sn); setSIPR(sip); // write the gateway IP, subnet mask, and source IP address back to the chip.
}


/**
@brief  This function set the transmit & receive buffer size as per the channels is used
Note for TMSR and RMSR bits are as follows\n
bit 1-0 : memory size of channel #0 \n
bit 3-2 : memory size of channel #1 \n
bit 5-4 : memory size of channel #2 \n
bit 7-6 : memory size of channel #3 \n
bit 9-8 : memory size of channel #4 \n
bit 11-10 : memory size of channel #5 \n
bit 12-12 : memory size of channel #6 \n
bit 15-14 : memory size of channel #7 \n
Maximum memory size for Tx, Rx in the W5500 is 16K Bytes,\n
In the range of 16KBytes, the memory size could be allocated dynamically by each channel.\n
Be attentive to sum of memory size shouldn't exceed 8Kbytes\n
and to data transmission and reception from non-allocated channel may cause some problems.\n
If the 16KBytes memory is already  assigned to certain channel, \n
other 3 channels couldn't be used, for there's no available memory.\n
If two 4KBytes memory are assigned to two each channels, \n
other 2 channels couldn't be used, for there's no available memory.\n
*/
void WIZCHIP_sysinit( uint8_t * tx_size, uint8_t * rx_size  )
{
  int16_t ssum,rsum;

#ifdef __DEF_WIZCHIP_DBG__
  xSerialPrint_P(PSTR("WIZCHIP_sysinit()\r\n"));
#endif

  ssum = 0;
  rsum = 0;

  for (uint8_t i = 0 ; i < _WIZCHIP_MAX_SOC_NUM_; ++i)       // Set the size, masking and base address of Tx & Rx memory by each channel
  {
	  WIZCHIP_write( (Sn_TXMEM_SIZE(i)), tx_size[i]);
	  WIZCHIP_write( (Sn_RXMEM_SIZE(i)), rx_size[i]);

#ifdef __DEF_WIZCHIP_DBG__
	  xSerialPrintf_P(PSTR("tx_size[%d]: %d, Sn_TXMEM_SIZE = %d\r\n"),i, tx_size[i], WIZCHIP_read(Sn_TXMEM_SIZE(i)));
	  xSerialPrintf_P(PSTR("rx_size[%d]: %d, Sn_RXMEM_SIZE = %d\r\n"),i, rx_size[i], WIZCHIP_read(Sn_RXMEM_SIZE(i)));
#endif
    SSIZE[i] = (uint16_t)(0);
    RSIZE[i] = (uint16_t)(0);


    if (ssum <= 16384)
    {
         switch( tx_size[i] )
      {
		case 1:
			SSIZE[i] = (uint16_t)(1024);
			break;
		case 2:
			SSIZE[i] = (uint16_t)(2048);
			break;
		case 4:
			SSIZE[i] = (uint16_t)(4096);
			break;
		case 8:
			SSIZE[i] = (uint16_t)(8192);
			break;
		case 16:
			SSIZE[i] = (uint16_t)(16384);
			break;
		default :
			SSIZE[i] = (uint16_t)(2048);
			break;
      }
    }

   if (rsum <= 16384)
    {
         switch( rx_size[i] )
      {
		case 1:
			RSIZE[i] = (uint16_t)(1024);
			break;
		case 2:
			RSIZE[i] = (uint16_t)(2048);
			break;
		case 4:
			RSIZE[i] = (uint16_t)(4096);
			break;
		case 8:
			RSIZE[i] = (uint16_t)(8192);
			break;
		case 16:
			RSIZE[i] = (uint16_t)(16384);
			break;
		default :
			RSIZE[i] = (uint16_t)(2048);
			break;
      }
    }
    ssum += SSIZE[i];
    rsum += RSIZE[i];
  }
}


void setMR(uint8_t val)
{
	WIZCHIP_write(MR,val);
}

uint8_t getMR(void)
{
	return WIZCHIP_read(MR);
}


/**
@brief  This function sets up MAC address.
*/
void setSHAR(
  uint8_t * addr  /**< a pointer to a 6 -byte array responsible to set the MAC address. */
  )
{
  WIZCHIP_write_buf(SHAR0, addr, 6);
}

/**
@brief	This function gets MAC address.
*/
void getSHAR(uint8_t * addr)
{
    WIZCHIP_read_buf(SHAR0, addr, 6);
}

/**
@brief  This function sets up gateway IP address.
*/
void setGAR(
  uint8_t * addr  /**< a pointer to a 4 -byte array responsible to set the Gateway IP address. */
  )
{
    WIZCHIP_write_buf(GAR0, addr, 4);
}

void getGAR(uint8_t * addr)
{
    WIZCHIP_read_buf(GAR0, addr, 4);
}


/**
@brief  It sets up SubnetMask address
*/
void setSUBR(uint8_t * addr)
{
    WIZCHIP_write_buf(SUBR0, addr, 4);
}

void getSUBR(uint8_t * addr)
{
    WIZCHIP_read_buf(SUBR0, addr, 4);
}


/**
@brief  This function sets up Source IP address.
*/
void setSIPR(
  uint8_t * addr  /**< a pointer to a 4 -byte array responsible to set the Source IP address. */
  )
{
    WIZCHIP_write_buf(SIPR0, addr, 4);
}

/**
@brief	This function gets the Source IP address.
*/
void getSIPR(uint8_t * addr)
{
    WIZCHIP_read_buf(SIPR0, addr, 4);
}


/**
@brief  This function gets Interrupt register in common register.
 */
uint8_t getIR( void )
{
   return WIZCHIP_read(IR);
}


/**
 Retransmission
 **/

/**
@brief  This function sets up Retransmission time.

If there is no response from the peer or delay in response then retransmission
will be there as per RTR (Retry Time-value Register)setting
*/
void setRTR(uint16_t timeout)
{
	WIZCHIP_write(RTR0,(uint8_t)((timeout & 0xff00) >> 8));
	WIZCHIP_write(RTR1,(uint8_t)(timeout & 0x00ff));
}

/**
@brief  This function set the number of Retransmission.

If there is no response from the peer or delay in response then recorded time
as per RTR & RCR register setting then time out will occur.
*/
void setRCR(uint8_t retry)
{
	WIZCHIP_write(RCR,retry);
}

/**
@brief  This function set the interrupt mask Enable/Disable appropriate Interrupt. ('1' : interrupt enable)

If any bit in IMR is set as '0' then there is not interrupt signal though the bit is
set in IR register.
*/
void clearIR(uint8_t mask)
{
	WIZCHIP_write(IR, ~mask | getIR() ); // must be set to 0x10.
}

/**
@brief  This sets the maximum segment size of TCP in Active Mode), while in Passive Mode this is set by peer
*/
void setSn_MSS(SOCKET s, uint16_t Sn_MSSR)
{
	WIZCHIP_write( Sn_MSSR0(s), (uint8_t)((Sn_MSSR & 0xff00) >> 8));
	WIZCHIP_write( Sn_MSSR1(s), (uint8_t)(Sn_MSSR & 0x00ff));
}

void setSn_TTL(SOCKET s, uint8_t ttl)
{
	WIZCHIP_write( Sn_TTL(s) , ttl);
}


/**
@brief	These below function is used to setup the Protocol Field of IP Header when
		executing the IP Layer RAW mode.
*/
void setSn_PROTO(SOCKET s, uint8_t proto)
{
	WIZCHIP_write(Sn_PROTO(s),proto);
}


/**
@brief  get socket interrupt status

These below functions are used to read the Interrupt & Socket Status register
*/
uint8_t getSn_IR(SOCKET s)
{
   return WIZCHIP_read(Sn_IR(s));
}


/**
@brief   get socket status
*/
uint8_t getSn_SR(SOCKET s)
{
   return WIZCHIP_read(Sn_SR(s));
}


/**
@brief  get socket TX transmit free buffer size

This gives free buffer size of transmit buffer. This is the data size that user can transmit.
User should check this value first and control the size of transmitted data.
*/
uint16_t
getSn_TX_FSR (SOCKET s)
{
  uint16_t val = 0;
  uint16_t val1 = 0;
  do
  {
    val1 = WIZCHIP_read(Sn_TX_FSR0(s));
    val1 = (val1 << 8) + WIZCHIP_read(Sn_TX_FSR1(s));
      if (val1 != 0)
    {
        val = WIZCHIP_read(Sn_TX_FSR0(s));
        val = (val << 8) + WIZCHIP_read(Sn_TX_FSR1(s));
    }
  } while (val != val1);
   return val;
}


/**
@brief	 get socket RX received buffer size

This gives size of received data in receive buffer.
*/
uint16_t getSn_RX_RSR(SOCKET s)
{
  uint16_t val=0,val1=0;
  do
  {
    val1 = WIZCHIP_read(Sn_RX_RSR0(s));
    val1 = (val1 << 8) + WIZCHIP_read(Sn_RX_RSR1(s));
    if(val1 != 0)
    {
        val = WIZCHIP_read(Sn_RX_RSR0(s));
        val = (val << 8) + WIZCHIP_read(Sn_RX_RSR1(s));
    }
  } while (val != val1);
   return val;
}


/**
@brief   This function is being called by send() and sendto() function also.

This function read the Tx write pointer register and after copy the data in buffer update the Tx write pointer
register. User should write upper byte first and lower byte later to get proper value.
*/
void WIZCHIP_send_data_processing(SOCKET s, uint8_t *data, uint16_t len)
{

  uint16_t ptr;
  uint32_t addrbsb;
  ptr = WIZCHIP_read( Sn_TX_WR0(s) );
  ptr = ((ptr & 0x00ff) << 8) + WIZCHIP_read(Sn_TX_WR1(s));

#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrintf_P(PSTR("ISR_TX: tx_ptr: %.4x tx_len: %.4x\r\n"), ptr, len);
#endif

  addrbsb = ((uint32_t)ptr<<8) + (s<<5) + 0x10;
  WIZCHIP_write_buf(addrbsb, data, len);
  ptr += len;

  WIZCHIP_write( Sn_TX_WR0(s) ,(uint8_t)((ptr & 0xff00) >> 8));
  WIZCHIP_write( Sn_TX_WR1(s),(uint8_t)(ptr & 0x00ff));
}


/**
@brief  This function is being called by recv() also.

This function read the Rx read pointer register
and after copy the data from receive buffer update the Rx write pointer register.
User should read upper byte first and lower byte later to get proper value.
*/
void WIZCHIP_recv_data_processing(SOCKET s, uint8_t *data, uint16_t len)
{
  uint16_t ptr;
  uint32_t addrbsb;

  ptr = WIZCHIP_read( Sn_RX_RD0(s) );
  ptr = ((ptr & 0x00ff) << 8) + WIZCHIP_read( Sn_RX_RD1(s) );

#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrintf_P(PSTR("ISR_RX: rd_ptr: %.4x rd_len: %.4x\r\n"), ptr, len);
#endif

  addrbsb = ((uint32_t)ptr<<8) + (s<<5) + 0x18;
  WIZCHIP_read_buf(addrbsb, data, len);
  ptr += len;

  WIZCHIP_write( Sn_RX_RD0(s), (uint8_t)((ptr & 0xff00) >> 8));
  WIZCHIP_write( Sn_RX_RD1(s), (uint8_t)(ptr & 0x00ff));
}



/**
@brief	for copy the data from application buffer to Transmit buffer of the chip.

This function is being used for copy the data from application buffer to Transmit
buffer of the chip. It calculates the actual physical address where one has to write
the data in transmit buffer. The Tx memory upper-bound of socket is wrapped automatically.
The entire 16bit address space is valid.
*/
void WIZCHIP_write_data(SOCKET s, uint8_t * src, uint8_t * dst, uint16_t len)
{
	uint32_t addrbsb;

	addrbsb = ((uint32_t)(uint16_t)dst<<8) + ((uint32_t)s<<5) + 0x10;
	WIZCHIP_write_buf(addrbsb, src, len);
}


/**
@brief	This function is being used for copy the data from Receive buffer of the chip to application buffer.

It calculate the actual physical address where one has to read
the data from Receive buffer.  The Rx memory upper-bound of socket is wrapped automatically.
The entire 16bit address space is valid.
*/
void WIZCHIP_read_data(SOCKET s, uint8_t * src, uint8_t * dst, uint16_t len)
{
	uint32_t addrbsb;

	addrbsb = ((uint32_t)(uint16_t)src<<8) + ((uint32_t)s<<5) + 0x18;
	WIZCHIP_read_buf(addrbsb, dst, len);
}


#endif // #if   (_WIZCHIP_ == 5500)		// Definition in freeRTOSBoardDefs.h

