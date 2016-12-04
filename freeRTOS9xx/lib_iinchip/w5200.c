/*
 * (c)COPYRIGHT
 * ALL RIGHT RESERVED
 *
 * FileName : w5200.c
  * -
  * ----------------------------------------------------------------
 */
#include "wizchip_conf.h"

#if   (_WIZCHIP_ == 5200)		// Definition in freeRTOSBoardDefs.h

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

#include "w5200.h"

#ifdef __DEF_WIZCHIP_DBG__
/* serial interface include file. */
#include "serial.h"
#endif

#ifdef __DEF_WIZCHIP_PPP__
   #include "md5.h"
#endif

static uint8_t 	I_STATUS[_WIZCHIP_MAX_SOC_NUM_];
static uint16_t SMASK[_WIZCHIP_MAX_SOC_NUM_]; /**< Variable for Tx buffer MASK in each channel */
static uint16_t RMASK[_WIZCHIP_MAX_SOC_NUM_]; /**< Variable for Rx buffer MASK in each channel */
static uint16_t SSIZE[_WIZCHIP_MAX_SOC_NUM_]; /**< Max Tx buffer size by each channel */
static uint16_t RSIZE[_WIZCHIP_MAX_SOC_NUM_]; /**< Max Rx buffer size by each channel */
static uint16_t SBUFBASEADDRESS[_WIZCHIP_MAX_SOC_NUM_]; /**< Tx buffer base address by each channel */
static uint16_t RBUFBASEADDRESS[_WIZCHIP_MAX_SOC_NUM_]; /**< Rx buffer base address by each channel */

//  the ARP errata fix
static un_l2cval SUBN_VAR; // off-chip subnet mask address - solve Errata 2 & 3 v1.6 - March 2012

uint16_t pre_sent_ptr, sent_ptr;

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
uint16_t WIZCHIP_getRxMASK(uint8_t s)
{
   return RMASK[s];
}
uint16_t WIZCHIP_getTxMASK(uint8_t s)
{
   return SMASK[s];
}
uint16_t WIZCHIP_getRxBASE(uint8_t s)
{
   return RBUFBASEADDRESS[s];
}
uint16_t WIZCHIP_getTxBASE(uint8_t s)
{
   return SBUFBASEADDRESS[s];
}



 /**
@brief	This function writes the data into W5200 registers.
*/
uint8_t WIZCHIP_write(uint16_t addr, uint8_t data)
{
	uint8_t Byte;

	WIZCHIP_ISR_DISABLE();

	spiSelect(Wiznet);							// CS=0, get semaphore, SPI start

	spiSetDataMode(SPI_MODE0);					// Enable SPI function in mode 0, CPOL=0 CPHA=0
	spiSetClockDivider(_WIZCHIP_SPI_DIVIDER);	// SPI at maximum speed

	// Make sure you manually pull slave select low to indicate start of transfer.
	// That is NOT done by this function..., because...
	// Some devices need to have their SS held low across multiple transfer calls.
	// Using spiSelect (SS_pin);

	// If the SPI module has not been enabled yet, then return with nothing.
	if ( !(SPCR & _BV(SPE)) ) return 0;

	// The SPI module is enabled, but it is in slave mode, so we can not
	// transmit the byte.  This can happen if SSbar is an input and it went low.
	// We will try to recover by setting the MSTR bit.
	// Check this once only at the start. Assume that things don't change.
	if ( !(SPCR & _BV(MSTR)) ) SPCR |= _BV(MSTR);
	if ( !(SPCR & _BV(MSTR)) ) return 0;

	portENTER_CRITICAL();

	SPDR = (uint8_t)((addr & 0xFF00) >> 8);	// load the upper address to be transmitted
	Byte = (uint8_t)(addr & 0x00FF);		// pre-load the lower address to be transmitted
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;					// load the lower address to be transmitted
	Byte = 0x80;					// pre-load the Data Write command and Write data length 14-8
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;					// load the Data Write command and Write data length 14-8
	Byte = 0x01;					// pre-load the Write data length 7-0
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;					// load the Write data length 7-0
	Byte = data;					// pre-load the byte to be transmitted
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;					// load the byte to be transmitted
	while ( !(SPSR & _BV(SPIF)) );

	portEXIT_CRITICAL();

	spiDeselect(Wiznet);			// CS=1, SPI end, give semaphore

	WIZCHIP_ISR_ENABLE();			// Interrupt Service Routine Enable

	return 1;
}


/**
@brief	This function reads the value from W5200 registers.
*/
uint8_t WIZCHIP_read(uint16_t addr)
{
	uint8_t Byte;

	WIZCHIP_ISR_DISABLE();

	spiSelect(Wiznet);							// CS=0, get semaphore, SPI start

	spiSetDataMode(SPI_MODE0);					// Enable SPI function in mode 0, CPOL=0 CPHA=0
	spiSetClockDivider(_WIZCHIP_SPI_DIVIDER);	// SPI at maximum speed

	// Make sure you manually pull slave select low to indicate start of transfer.
	// That is NOT done by this function..., because...
	// Some devices need to have their SS held low across multiple transfer calls.
	// Using spiSelect (SS_pin);

	// If the SPI module has not been enabled yet, then return with nothing.
	if ( !(SPCR & _BV(SPE)) ) return 0;

	// The SPI module is enabled, but it is in slave mode, so we can not
	// transmit the byte.  This can happen if SSbar is an input and it went low.
	// We will try to recover by setting the MSTR bit.
	// Check this once only at the start. Assume that things don't change.
	if ( !(SPCR & _BV(MSTR)) ) SPCR |= _BV(MSTR);
	if ( !(SPCR & _BV(MSTR)) ) return 0;

	portENTER_CRITICAL();

	SPDR = (uint8_t)((addr & 0xFF00) >> 8);	// load the upper address to be transmitted
	Byte = (uint8_t)(addr & 0x00FF); 		// pre-load the lower address to be transmitted
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;					// load the lower address to be transmitted
	Byte = 0x00;					// pre-load the Data Read command and Read data length 14-8
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;					// load the Data Read command and Read data length 14-8
	Byte = 0x01;					// pre-load the Data Read data length 7-0
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;					// load the Data Read data length 7-0
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
@brief	This function writes into W5200 memory(Buffer)
*/
uint16_t WIZCHIP_write_buf(uint16_t addr, uint8_t * buf, uint16_t len)
{
	uint16_t idx;
	uint8_t Byte;

	WIZCHIP_ISR_DISABLE();

	spiSelect(Wiznet);							// CS=0, get semaphore, SPI start

	spiSetDataMode(SPI_MODE0);					// Enable SPI function in mode 0, CPOL=0 CPHA=0
	spiSetClockDivider(_WIZCHIP_SPI_DIVIDER);	// SPI at maximum speed

	// Make sure you manually pull slave select low to indicate start of transfer.
	// That is NOT done by this function..., because...
	// Some devices need to have their SS held low across multiple transfer calls.
	// Using spiSelect (SS_pin);

	// If the SPI module has not been enabled yet, then return with nothing.
	if ( !(SPCR & _BV(SPE)) ) return 0;

	// The SPI module is enabled, but it is in slave mode, so we can not
	// transmit the byte.  This can happen if SSbar is an input and it went low.
	// We will try to recover by setting the MSTR bit.
	// Check this once only at the start. Assume that things don't change.
	if ( !(SPCR & _BV(MSTR)) ) SPCR |= _BV(MSTR);
	if ( !(SPCR & _BV(MSTR)) ) return 0;

	portENTER_CRITICAL();

	SPDR = (uint8_t)((addr & 0xFF00) >> 8);		// load the upper address to be transmitted
	Byte = (uint8_t)(addr & 0x00FF);			// pre-load the lower address to be transmitted
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;									// load the lower address to be transmitted
	Byte = 0x80 | (uint8_t)((len & 0x7F00) >> 8);	// pre-load the Data Write command and Write data length 14-8
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;								// load the Data Write command and Write data length 14-8
	Byte = (uint8_t)(len & 0x00FF);				// pre-load the Write data length 7-0
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;								// load the Write data length 7-0

	idx = 0;
	while(idx < len)							// Write data in loop
	{
		Byte = buf[ idx++ ];					// pre-load the byte to be transmitted
		while ( !(SPSR & _BV(SPIF)) );
		SPDR = Byte;							// load the byte to be transmitted
	}
	while ( !(SPSR & _BV(SPIF)) );

	portEXIT_CRITICAL();

	spiDeselect(Wiznet);						// CS=1, SPI end, give semaphore

    WIZCHIP_ISR_ENABLE();						// Interrupt Service Routine Enable

	return len;
}


/**
@brief	This function reads from W5200 memory(Buffer)
*/
uint16_t WIZCHIP_read_buf(uint16_t addr, uint8_t * buf, uint16_t len)
{
	uint16_t idx;
	uint8_t Byte;

	WIZCHIP_ISR_DISABLE();

	spiSelect(Wiznet);							// CS=0, get semaphore, SPI start

	spiSetDataMode(SPI_MODE0);					// Enable SPI function in mode 0, CPOL=0 CPHA=0
	spiSetClockDivider(_WIZCHIP_SPI_DIVIDER);	// SPI at maximum speed

	// Make sure you manually pull slave select low to indicate start of transfer.
	// That is NOT done by this function..., because...
	// Some devices need to have their SS held low across multiple transfer calls.
	// Using spiSelect (SS_pin);

	// If the SPI module has not been enabled yet, then return with nothing.
	if ( !(SPCR & _BV(SPE)) ) return 0;

	// The SPI module is enabled, but it is in slave mode, so we can not
	// transmit the byte.  This can happen if SSbar is an input and it went low.
	// We will try to recover by setting the MSTR bit.
	// Check this once only at the start. Assume that things don't change.
	if ( !(SPCR & _BV(MSTR)) ) SPCR |= _BV(MSTR);
	if ( !(SPCR & _BV(MSTR)) ) return 0;

	portENTER_CRITICAL();

	SPDR = (uint8_t)((addr & 0xFF00) >> 8);		// load the upper address to be transmitted
	Byte = (uint8_t)(addr & 0x00FF);			// pre-load the lower address to be transmitted
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;								// load the lower address to be transmitted
	Byte = (uint8_t)((len & 0x7F00) >> 8);		// pre-load the Data Read command and Read data length 14-8
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;								// load the Data Read command and Read data length 14-8
	Byte = (uint8_t)(len & 0x00FF);				// pre-load the Data Read data length 7-0
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;								// load the Data Read data length 7-0
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
@brief	This function is for resetting of the W5200. Initialises the WIZCHIP to work in SPI mode
*/
void WIZCHIP_init(void)
{

	_delay_ms(150); // AVR can't delay its boot long as the W5200 needs to (data sheet 150ms), so add 150ms wait before we fire up W5200.

	spiBegin(Wiznet);		// enable the EtherMega W5200

	spiSetDataMode(SPI_MODE0);					// Enable SPI function in mode 0, CPOL=0 CPHA=0
	spiSetClockDivider(_WIZCHIP_SPI_DIVIDER);	// SPI at maximum speed

	WIZCHIP_write(MR, MR_RST); // reset the W5200 chip.

	do{
	_delay_ms(150); // Delay after reset for the W5200 needs to be (data sheet) 150ms, so wait 150ms before we address W5200.
	} while  (WIZCHIP_read(VERSIONR) != 0x03); 	// VERSIONR always reads as 0x03 for W5200

#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrintf_P(PSTR("\r\nMR       (0x%02x)\r\nVERSIONR (0x%02x)\r\nPSTATUS  (0x%02x)\r\n"), WIZCHIP_read(MR), WIZCHIP_read(VERSIONR), WIZCHIP_read(PSTATUS));
#endif
}


/**
@brief	This function set the transmit & receive buffer size as per the channels is used

Maximum memory size for Tx, Rx in the W5200 is 16K Bytes,\n
In the range of 16KBytes, the memory size could be allocated dynamically by each channel.\n
Be attentive to sum of memory size shouldn't exceed 8Kbytes\n
and to data transmission and reception from non-allocated channel may cause some problems.\n
If the 16KBytes memory is already  assigned to certain channel, \n
other 3 channels couldn't be used, for there's no available memory.\n
If two 4KBytes memory are assigned to two each channels, \n
other 2 channels couldn't be used, for there's no available memory.\n
*/
void WIZCHIP_sysinit( uint8_t * tx_size, uint8_t * rx_size	)
{
	int16_t ssum,rsum;

#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrint_P(PSTR("WIZCHIP_sysinit()\r\n"));
#endif

	ssum = 0;
	rsum = 0;

	SBUFBASEADDRESS[0] = (uint16_t)(__DEF_WIZCHIP_MAP_TXBUF__);		/* Set base address of Tx memory for channel #0 */
	RBUFBASEADDRESS[0] = (uint16_t)(__DEF_WIZCHIP_MAP_RXBUF__);		/* Set base address of Rx memory for channel #0 */

#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrint_P(PSTR("Channel : TX ADDR - SIZE | RX ADDR - SIZE\r\n"));
#endif

   for (uint8_t i = 0 ; i < _WIZCHIP_MAX_SOC_NUM_; ++i)       // Set the size, masking and base address of Tx & Rx memory by each channel
	{
    WIZCHIP_write((Sn_TXMEM_SIZE(i)),tx_size[i]);
    WIZCHIP_write((Sn_RXMEM_SIZE(i)),rx_size[i]);

		SSIZE[i] = (uint16_t)(0);
		RSIZE[i] = (uint16_t)(0);

		if (ssum <= 16384)
		{
         switch( tx_size[i] )
			{
			case 1:
				SSIZE[i] = (uint16_t)(1024);
				SMASK[i] = (uint16_t)(0x03FF);
				break;
			case 2:
				SSIZE[i] = (uint16_t)(2048);
				SMASK[i] = (uint16_t)(0x07FF);
				break;
			case 4:
				SSIZE[i] = (uint16_t)(4096);
				SMASK[i] = (uint16_t)(0x0FFF);
				break;
			case 8:
				SSIZE[i] = (uint16_t)(8192);
				SMASK[i] = (uint16_t)(0x1FFF);
				break;
			case 16:
				SSIZE[i] = (uint16_t)(16384);
				SMASK[i] = (uint16_t)(0x3FFF);
				break;
		    default :
		        SSIZE[i] = (uint16_t)(2048);
				SMASK[i] = (uint16_t)(0x07FF);
		        break;
			}
		}

		if (rsum <= 16384)
		{
         switch( rx_size[i] )
			{
			case 1:
				RSIZE[i] = (uint16_t)(1024);
				RMASK[i] = (uint16_t)(0x03FF);
				break;
			case 2:
				RSIZE[i] = (uint16_t)(2048);
				RMASK[i] = (uint16_t)(0x07FF);
				break;
			case 4:
				RSIZE[i] = (uint16_t)(4096);
				RMASK[i] = (uint16_t)(0x0FFF);
				break;
			case 8:
				RSIZE[i] = (uint16_t)(8192);
				RMASK[i] = (uint16_t)(0x1FFF);
				break;
			case 16:
				RSIZE[i] = (uint16_t)(16384);
				RMASK[i] = (uint16_t)(0x3FFF);
				break;
		    default :
		        RSIZE[i] = (uint16_t)(2048);
				RMASK[i] = (uint16_t)(0x07FF);
		        break;
			}
		}
		ssum += SSIZE[i];
		rsum += RSIZE[i];

        if (i != 0)             // Sets base address of Tx and Rx memory for channel #1,#2,#3
		{
			SBUFBASEADDRESS[i] = SBUFBASEADDRESS[i-1] + SSIZE[i-1];
			RBUFBASEADDRESS[i] = RBUFBASEADDRESS[i-1] + RSIZE[i-1];
		}
#ifdef __DEF_WIZCHIP_DBG__
		xSerialPrintf_P(PSTR("     %d  :    %.4x - %.4x |    %.4x - %.4x\r\n"), i, (uint16_t)SBUFBASEADDRESS[i], SSIZE[i], (uint16_t)RBUFBASEADDRESS[i], RSIZE[i]);
		_delay_ms(5);
#endif
	}
}


void setMR(uint8_t val)
{
	/* 	DIRECT ACCESS	*/
	WIZCHIP_write(MR,val);
}

uint8_t getMR(void)
{
	return WIZCHIP_read(MR);
}


/**
@brief	This function sets up MAC address.
*/
void setSHAR(
	uint8_t * addr	/**< a pointer to a 6 -byte array responsible to set the MAC address. */
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
@brief	This function sets up gateway IP address.
*/
void setGAR(
	uint8_t * addr	/**< a pointer to a 4 -byte array responsible to set the Gateway IP address. */
	)
{
    WIZCHIP_write_buf(GAR0, addr, 4);
}

void getGAR(uint8_t * addr)
{
    WIZCHIP_read_buf(GAR0, addr, 4);
}


/**
@brief	It sets up SubnetMask address
*/
void saveSUBR(
		un_l2cval * addr	/**< a pointer to a 4 -byte array responsible to set the SubnetMask address */
	)
{
	// write to off-chip subnet mask address - solve Errata 2 & 3 v1.6
	// Basically the hardware ARP engine is broken, unless it is set to 0.0.0.0
	// so we have to keep it so, unless we're using TCP connect() or UDP sendto()
	SUBN_VAR.lVal = addr->lVal;
}

/**
@brief	It sets up SubnetMask address
*/
void setSUBR(
	void
	)
{
	// apply off-chip subnet mask address - solve Errata 2 & 3 v1.6
    WIZCHIP_write_buf(SUBR0, SUBN_VAR.cVal, 4);
}

/**
@brief	It sets up SubnetMask address
*/
void clearSUBR(
	void
	)
{
	// clear on-chip subnet mask address - solve Errata 2 & 3 v1.6
    WIZCHIP_write_buf(SUBR0, 0x00, 4);
}

void getSUBR(un_l2cval *addr)
{
	addr->lVal = SUBN_VAR.lVal;
}


/**
@brief	This function sets up Source IP address.
*/
void setSIPR(
	uint8_t * addr	/**< a pointer to a 4 -byte array responsible to set the Source IP address. */
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
@brief	This function gets Interrupt register in common register.
 */
uint8_t getIR( void )
{
   return WIZCHIP_read(IR);
}


/**
 Retransmission
 **/

/**
@brief	This function sets up Retransmission time.

If there is no response from the peer or delay in response then retransmission
will be there as per RTR (Retry Time-value Register) setting
*/
void setRTR(uint16_t timeout)
{
	WIZCHIP_write(RTR0,(uint8_t)((timeout & 0xff00) >> 8));
	WIZCHIP_write(RTR1,(uint8_t)(timeout & 0x00ff));
}


/**
@brief	This function set the number of Retransmission.

If there is no response from the peer or delay in response then recorded time
as per RTR & RCR register setting then time out will occur.
*/
void setRCR(uint8_t retry)
{
	WIZCHIP_write(RCR,retry);
}


/**
@brief	This function set the interrupt mask Enable/Disable appropriate Interrupt. ('1' : interrupt enable)

If any bit in IMR is set as '0' then there is not interrupt signal though the bit is
set in IR register.
*/
void setIMR(uint8_t mask)
{
	WIZCHIP_write(IMR,mask); // must be set to 0x10.
}


/**
 * @ingroup Common_register_access_function
 * @brief Get @ref PHYCFGR register
 * @return uint8_t. Value of @ref PHYCFGR register.
 * @sa setPHYCFGR()
 */
#define getPHYSTATUS()  	WIZCHIP_read(PSTATUS)


/**
@brief	This sets the maximum segment size of TCP in Active Mode), while in Passive Mode this is set by peer
*/
void setSn_MSS(SOCKET s, uint16_t Sn_MSSR)
{
	WIZCHIP_write(Sn_MSSR0(s),(uint8_t)((Sn_MSSR & 0xff00) >> 8));
	WIZCHIP_write(Sn_MSSR1(s),(uint8_t)(Sn_MSSR & 0x00ff));
}

void setSn_TTL(SOCKET s, uint8_t ttl)
{
   WIZCHIP_write(Sn_TTL(s), ttl);
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
@brief	get socket interrupt status

These below functions are used to read the Interrupt & Socket Status register
*/
uint8_t getSn_IR(SOCKET s)
{
   return WIZCHIP_read(Sn_IR(s));
}


/**
@brief	 get socket status
*/
uint8_t getSn_SR(SOCKET s)
{
   return WIZCHIP_read(Sn_SR(s));
}


/**
@brief	get socket TX transmit free buffer size

This gives free buffer size of transmit buffer. This is the data size that user can transmit.
User should check this value first and control the size of transmitted data.
*/
uint16_t getSn_TX_FSR(SOCKET s)
{
	uint16_t val  = 0;
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
	uint16_t val  = 0;
	uint16_t val1 = 0;
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
@brief	 This function is being called by TCP send() and UDP & IP_RAW sendto() function.

This function reads the Tx write pointer register and after copy the data in buffer update the Tx write pointer
register. User should read upper byte first and lower byte later to get proper value.
*/
void WIZCHIP_send_data_processing(SOCKET s, uint8_t *data, uint16_t len)
{
	uint16_t ptr;

	ptr = WIZCHIP_read(Sn_TX_WR0(s));
	ptr = ((ptr & 0x00ff) << 8) + WIZCHIP_read(Sn_TX_WR1(s));

#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrintf_P(PSTR("ISR_TX: tx_ptr: %.4x tx_len: %.4x\r\n"), ptr, len);
#endif

	WIZCHIP_write_data(s, data, (uint8_t *)(ptr), len);
	ptr += len;

	WIZCHIP_write(Sn_TX_WR0(s), (uint8_t)((ptr & 0xff00) >> 8));
	WIZCHIP_write(Sn_TX_WR1(s), (uint8_t)(ptr & 0x00ff));
}


/**
@brief	This function is being called by TCP recv().

This function read the Rx read pointer register
and after copy the data from receive buffer update the Rx write pointer register.
User should read upper byte first and lower byte later to get proper value.
*/
void WIZCHIP_recv_data_processing(SOCKET s, uint8_t *data, uint16_t len)
{
	uint16_t ptr;
	ptr = WIZCHIP_read(Sn_RX_RD0(s));
	ptr = ((ptr & 0x00ff) << 8) + WIZCHIP_read(Sn_RX_RD1(s));

#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrintf_P(PSTR("ISR_RX: rd_ptr: %.4x rd_len: %.4x\r\n"), ptr, len);
#endif

	WIZCHIP_read_data(s, (uint8_t *)ptr, data, len); // read data
	ptr += len;
	WIZCHIP_write(Sn_RX_RD0(s), (uint8_t)((ptr & 0xff00) >> 8));
	WIZCHIP_write(Sn_RX_RD1(s), (uint8_t)(ptr & 0x00ff));
}


/**
@brief	for copy the data from application buffer to Transmit buffer of the chip.

This function is being used for copy the data from application buffer to Transmit
buffer of the chip. It calculates the actual physical address where one has to write
the data in transmit buffer. Here also takes care of the condition that it exceeds
the Tx memory upper-bound of socket.
*/
void WIZCHIP_write_data(SOCKET s, uint8_t * src, uint8_t * dst, uint16_t len)
{
	uint16_t size;
	uint16_t dst_mask;
	uint8_t * dst_ptr;

	dst_mask = (uint16_t)dst & WIZCHIP_getTxMASK(s);
	dst_ptr = (uint8_t *)(WIZCHIP_getTxBASE(s) + dst_mask);

	if (dst_mask + len > WIZCHIP_getTxMAX(s))
	{
		size = WIZCHIP_getTxMAX(s) - dst_mask;
		WIZCHIP_write_buf((uint16_t)dst_ptr, (uint8_t *)src, size);
		src += size;
		size = len - size;
		dst_ptr = (uint8_t *)(WIZCHIP_getTxBASE(s));
		WIZCHIP_write_buf((uint16_t)dst_ptr, (uint8_t *)src, size);
	}
	else
	{
		WIZCHIP_write_buf((uint16_t)dst_ptr, (uint8_t *)src, len);
	}
}


/**
@brief	This function is being used for copy the data from Receive buffer of the chip to application buffer.

It calculate the actual physical address where one has to read
the data from Receive buffer. Here also take care of the condition that it exceeds
the Rx memory upper-bound of socket.
*/
void WIZCHIP_read_data(SOCKET s, uint8_t * src, uint8_t * dst, uint16_t len)
{
	uint16_t size;
	uint16_t src_mask;
	uint8_t * src_ptr;

	src_mask = (uint16_t)src & WIZCHIP_getRxMASK(s);
	src_ptr = (uint8_t *)(WIZCHIP_getRxBASE(s) + src_mask);

	if( (src_mask + len) > WIZCHIP_getRxMAX(s) )
	{
		size = WIZCHIP_getRxMAX(s) - src_mask;
		WIZCHIP_read_buf((uint16_t)src_ptr, (uint8_t *)dst,size);
		dst += size;
		size = len - size;
		src_ptr = (uint8_t *)(WIZCHIP_getRxBASE(s));
		WIZCHIP_read_buf((uint16_t)src_ptr, (uint8_t *) dst,size);
	}
	else
	{
		WIZCHIP_read_buf((uint16_t)src_ptr, (uint8_t *) dst,len);
	}
}

#endif // #if   (_WIZCHIP_ == 5200)		// Definition in freeRTOSBoardDefs.h
