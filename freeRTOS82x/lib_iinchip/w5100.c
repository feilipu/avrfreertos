/*
 * (c)COPYRIGHT
 * ALL RIGHT RESERVED
 *
 * FileName : w5100.c
 * Revision History :
 * ----------	-------		------------------------------------------------
 * 	Date			version	  	Description
 * ----------	-------  	------------------------------------------------
 * 01/25/2007	1.1			Bug is Fixed in the Indirect Mode
 *							: Memory mapping error
 * ----------	-------		------------------------------------------------
 * 01/08/2008	1.2			Modification of Socket Command Part
 *							: Check if the appropriately performed after writing Sn_CR
 *
 *							Modification of SPI Part
 *							: SPI code changed by adding 'spi.h'.
 *							: Change control type for SPI port from byte to bit.
 * ----------	-------		------------------------------------------------
 * 01/15/2008	1.3			Bug is Fixed in the pppinit() function.
 *							: do not clear interrupt value, so fixed.
 *
 *		                   			Modification of ISR
 *                   				: Do not exit ISR, if there is interrupt.
 * ----------	-------		------------------------------------------------
 * 03/21/2008	1.4			Modification of SetMR() function
 *                   				: Use WIZCHIP_write() function in Direct or SPI mode.
 * ----------	-------		------------------------------------------------
 * 03/21/2008	1.5			Bug is Fixed in the pppinit() function.
 *							: do not clear receive buffer, so fixed. +200903[bj] clear receive buffer
 * ----------	-------		------------------------------------------------
  * 03/13/2012	1.6.1		Added clearSUBR(), applySUBR() and modified setSUBR() functions
 *							      because of the ARP errata.
 *							Keep SUBR 0.0.0.0 unless using TCP connect() or UDP sendto()
 *							Use the SUBN_VAR variable to read the real subnet.
 * ----------	-------		------------------------------------------------
 */

#include "wizchip_conf.h"

#if   (_WIZCHIP_ == 5100)	// Definition in freeRTOSBoardDefs.h

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

#include "w5100.h"

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

// the ARP errata fix, only relevant to W5100
static un_l2cval SUBN_VAR; // off-chip subnet mask address - solve Errata 2 & 3 v1.6 - March 2012

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
@brief	This function writes the data into W5100 registers.
*/
uint8_t WIZCHIP_write(uint16_t addr, uint8_t data)
{
	uint8_t TxByte;

	WIZCHIP_ISR_DISABLE();

	spiSelect(Wiznet);    						// CS=0, get semaphore, SPI start

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

	SPDR = 0xF0; // Begin transmission with Write Op Code
	TxByte = (addr & 0xFF00) >> 8; // pre-load the upper address to be transmitted
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = TxByte; // Continue transmission
	TxByte = (addr & 0x00FF); // pre-load the lower address to be transmitted
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = TxByte; // Continue transmission
	TxByte = data; // pre-load the byte to be transmitted
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = TxByte; // Continue transmission
	while ( !(SPSR & _BV(SPIF)) );

	portEXIT_CRITICAL();

	spiDeselect(Wiznet);	// CS=1, SPI end, give semaphore

	WIZCHIP_ISR_ENABLE();

	return 1;
}


/**
@brief	This function reads the value from W5100 registers.
*/
uint8_t WIZCHIP_read(uint16_t addr)
{
	uint8_t RxByte;

	WIZCHIP_ISR_DISABLE();

	spiSelect(Wiznet);    						// CS=0, get semaphore, SPI start

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

	SPDR = 0x0F; // Begin transmission with Read Op Code
	RxByte = (addr & 0xFF00) >> 8; // pre-load the upper address to be transmitted
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = RxByte; // Continue transmission
	RxByte = (addr & 0x00FF); // pre-load the lower address to be transmitted
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = RxByte; // Continue transmission
	RxByte = 0xFF; // pre-load a dummy byte to be transmitted
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = RxByte; // Continue transmission
	while ( !(SPSR & _BV(SPIF)) );

	RxByte = SPDR; // copy received byte

	portEXIT_CRITICAL();

	spiDeselect(Wiznet);	// CS=1, SPI end, give semaphore

	WIZCHIP_ISR_ENABLE();

	return RxByte;
}


/**
@brief	This function writes into W5100 memory (Buffer)
*/
uint16_t WIZCHIP_write_buf(uint16_t addr, uint8_t *buf, uint16_t len)
{
	uint8_t TxByte;
	uint16_t i;

    if(len == 0)
    {
      return 0;
    }

	WIZCHIP_ISR_DISABLE();

	// Make sure you manually pull slave select low to indicate start of transfer.
	// That is NOT done by this function..., because...
	// Some devices need to have their SS held low across multiple transfer calls.
	// Using spiSelect (SS_pin);
	// But, this one needs the SS pin toggled to finish a transmission.
	// So call the fast macro to make it snappy. No need to release semaphore.

	spiSelect(Wiznet);    						// SS=0, SPI start, get semaphore

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

#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrintf_P(PSTR("WIZCHIP_write_buf: tx_ptr: %.4x "), addr);
#endif

	for( i=0; i<len; ++i)
	{

		portENTER_CRITICAL();

		SPI_PORT &= ~SPI_BIT_SS_WIZNET;	// SS=0, SPI start

		SPDR = 0xF0; // Begin transmission with Write Op Code
		TxByte = (((addr+i) & 0xFF00) >> 8); // pre-load the upper address to be transmitted
		while ( !(SPSR & _BV(SPIF)) );

		SPDR = TxByte; // Continue transmission
		TxByte = ((addr+i) & 0x00FF); // pre-load the lower address to be transmitted
		while ( !(SPSR & _BV(SPIF)) );

		SPDR = TxByte; // Continue transmission
		TxByte = buf[i]; // pre-load the byte to be transmitted
		while ( !(SPSR & _BV(SPIF)) );

		SPDR = TxByte; // Continue transmission
		while ( !(SPSR & _BV(SPIF)) );

		SPI_PORT |= SPI_BIT_SS_WIZNET;	// SS=1, SPI stop, but keep semaphore

		portEXIT_CRITICAL();
	}

	if ( !(SPCR & _BV(MSTR)) ) return 0; 	// The SPI module is enabled, but it is in slave mode.

	spiDeselect(Wiznet);	// SS=1, SPI end

	WIZCHIP_ISR_ENABLE();

#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrintf_P(PSTR(" %.4x tx_len: %.4x\r\n"), addr+i, len);
#endif

	return len;
}


/**
@brief	This function reads from W5100 memory (Buffer)
*/
uint16_t WIZCHIP_read_buf(uint16_t addr, uint8_t *buf, uint16_t len)
{
	uint8_t RxByte;
	uint16_t i;

	WIZCHIP_ISR_DISABLE();

	// Make sure you manually pull slave select low to indicate start of transfer.
	// That is NOT done by this function..., because...
	// Some devices need to have their SS held low across multiple transfer calls.
	// Using spiSelect (SS_pin);
	// But, this one needs the SS pin toggled to finish a transmission.
	// So call the fast macro to make it snappy. No need to release semaphore.

	spiSelect(Wiznet);    						// SS=0, SPI start

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

#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrintf_P(PSTR("WIZCHIP_read_buf: rx_ptr: %.4x "), addr);
#endif

	for ( i=0; i<len; ++i)
    {

		portENTER_CRITICAL();

		SPI_PORT &= ~SPI_BIT_SS_WIZNET;	// SS=0, SPI start

		SPDR = 0x0F; // Begin transmission with Read Op Code
		RxByte = (((addr+i) & 0xFF00) >> 8); // pre-load the upper address to be transmitted
		while ( !(SPSR & _BV(SPIF)) );

		SPDR = RxByte; // Continue transmission
		RxByte = ((addr+i) & 0x00FF); // pre-load the lower address to be transmitted
		while ( !(SPSR & _BV(SPIF)) );

		SPDR = RxByte; // Continue transmission
		RxByte = 0xFF; // pre-load a dummy byte to be transmitted
		while ( !(SPSR & _BV(SPIF)) );

		SPDR = RxByte; // Continue transmission
		while ( !(SPSR & _BV(SPIF)) );

		buf[i] = SPDR; // copy received byte

		SPI_PORT |= SPI_BIT_SS_WIZNET;	// SS=1, SPI stop, but keep semaphore

		portEXIT_CRITICAL();
	}

	if ( !(SPCR & _BV(MSTR)) ) return 0; 	// The SPI module is enabled, but it is in slave mode.

	spiDeselect(Wiznet);	// SS=1, SPI end

	WIZCHIP_ISR_ENABLE();

#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrintf_P(PSTR(" %.4x rx_len: %.4x\r\n"), addr+i, len);
#endif

	return len;
}


/**
@brief	Socket interrupt routine
*/
#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
ISR(INT4_vect)
#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega1284P__)
ISR(INT0_vect)
#endif
{
#ifdef __DEF_WIZCHIP_INT__
	uint8_t int_val;

	WIZCHIP_ISR_DISABLE();
	int_val = WIZCHIP_read(IR);

	/* process all of interrupt */
   do {
   /*---*/

   	if (int_val & IR_CONFLICT)
   	{
   		xSerialPrintf_P(PSTR("IP conflict : %.2x\r\n"), int_val);
   	}
   	if (int_val & IR_UNREACH)
   	{
   		xSerialPrintf_P(PSTR("INT Port Unreachable : %.2x\r\n"), int_val);
   		xSerialPrintf_P(PSTR("UIPR0 : %d.%d.%d.%d\r\n"), WIZCHIP_read(UIPR0), WIZCHIP_read(UIPR0+1), WIZCHIP_read(UIPR0+2), WIZCHIP_read(UIPR0+3));
   		xSerialPrintf_P(PSTR("UPORT0 : %.2x %.2x\r\n"), WIZCHIP_read(UPORT0), WIZCHIP_read(UPORT0+1));
   	}

   	/* interrupt clear */
   	WIZCHIP_write(IR, 0xf0);
      /*---*/

   	if (int_val & IR_SOCK(0))
   	{
   	/* save interrupt value*/
   		I_STATUS[0] |= WIZCHIP_read(Sn_IR(0)); // can be come to over two times.
   		WIZCHIP_write(Sn_IR(0), I_STATUS[0]);
      /*---*/
   	}
   	if (int_val & IR_SOCK(1))
   	{
   	/* save interrupt value*/
   		I_STATUS[1] |= WIZCHIP_read(Sn_IR(1));
   		WIZCHIP_write(Sn_IR(1), I_STATUS[1]);
      /*---*/
   	}
   	if (int_val & IR_SOCK(2))
   	{
   	/* save interrupt value*/
   		I_STATUS[2] |= WIZCHIP_read(Sn_IR(2));
   		WIZCHIP_write(Sn_IR(2), I_STATUS[2]);
      /*---*/
   	}
   	if (int_val & IR_SOCK(3))
   	{
   	/* save interrupt value*/
   		I_STATUS[3] |= WIZCHIP_read(Sn_IR(3));
   		WIZCHIP_write(Sn_IR(3), I_STATUS[3]);
      /*---*/
   	}

   	/* re-read interrupt value*/
   	int_val = WIZCHIP_read(IR);

	/* if exist, continue to process */
   } while (int_val != 0x00);
   /*---*/

	WIZCHIP_ISR_ENABLE();

#endif

}

/**
 * @brief	This function is for resetting of the W5100. Initialises the W5100 to work in SPI mode
 */
void WIZCHIP_init(void)
{

	_delay_ms(200); // AVR can't delay its boot long as the W5100 needs to, so add 200ms wait before we fire up W5100.

	spiBegin(Wiznet);							// enable the EtherMega W5100

	spiSetDataMode(SPI_MODE0);					// Enable SPI function in mode 0, CPOL=0 CPHA=0
	spiSetClockDivider(_WIZCHIP_SPI_DIVIDER);	// SPI at maximum speed

	WIZCHIP_write(MR, MR_RST); // reset the W5100 chip.

	_delay_ms(20);  // data sheet says 10ms after reset.

#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrintf_P(PSTR("\r\nMR       (0x%02x)\r\n"), WIZCHIP_read(MR));
#endif
}


/**
@brief	This function set the transmit & receive buffer size as per the channels is used

Note for TMSR and RMSR bits are as follows\n
bit 1-0 : memory size of channel #0 \n
bit 3-2 : memory size of channel #1 \n
bit 5-4 : memory size of channel #2 \n
bit 7-6 : memory size of channel #3 \n\n
Maximum memory size for Tx, Rx in the W5100 is 8K Bytes,\n
In the range of 8KBytes, the memory size could be allocated dynamically by each channel.\n
Be attentive to sum of memory size shouldn't exceed 8Kbytes\n
and to data transmission and reception from non-allocated channel may cause some problems.\n
If the 8KBytes memory is already  assigned to certain channel, \n
other 3 channels couldn't be used, for there's no available memory.\n
If two 4KBytes memory are assigned to two each channels, \n
other 2 channels couldn't be used, for there's no available memory.\n
*/
void WIZCHIP_sysinit(
	uint8_t tx_size, 	/**< tx_size Tx memory size (0x00 - 1KByte, 0x01- 2KByte, 0x10 - 4KByte, 0x11 - 8KByte) */
	uint8_t rx_size		/**< rx_size Rx memory size (0x00 - 1KByte, 0x01- 2KByte, 0x10 - 4KByte, 0x11 - 8KByte) */
	)
{
	int16_t ssum,rsum;

#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrint_P(PSTR(" sysinit()\r\n"));
#endif

	ssum = 0;
	rsum = 0;

	WIZCHIP_write(TMSR,tx_size);	/* Set Tx memory size for each channel */
	WIZCHIP_write(RMSR,rx_size);	/* Set Rx memory size for each channel */

	SBUFBASEADDRESS[0] = (uint16_t)(__DEF_WIZCHIP_MAP_TXBUF__);		/* Set base address of Tx memory for channel #0 */
	RBUFBASEADDRESS[0] = (uint16_t)(__DEF_WIZCHIP_MAP_RXBUF__);		/* Set base address of Rx memory for channel #0 */

#ifdef __DEF_WIZCHIP_DBG__
	xSerialPrint_P(PSTR("Channel : SEND MEM SIZE : RECV MEM SIZE\r\n"));
#endif

   for (uint8_t i = 0 ; i < _WIZCHIP_MAX_SOC_NUM_; ++i)       // Set the size, masking and base address of Tx & Rx memory by each channel
	{
		SSIZE[i] = (int16_t)(0);
		RSIZE[i] = (int16_t)(0);
		if (ssum <= 8192)
		{
         switch((tx_size >> i*2) & 0x03)  // Set Tx memory size
			{
			case 0:
				SSIZE[i] = (int16_t)(1024);
				SMASK[i] = (uint16_t)(0x03FF);
				break;
			case 1:
				SSIZE[i] = (int16_t)(2048);
				SMASK[i] = (uint16_t)(0x07FF);
				break;
			case 2:
				SSIZE[i] = (int16_t)(4096);
				SMASK[i] = (uint16_t)(0x0FFF);
				break;
			case 3:
				SSIZE[i] = (int16_t)(8192);
				SMASK[i] = (uint16_t)(0x1FFF);
				break;
			}
		}
		if (rsum <= 8192)
		{
         switch((rx_size >> i*2) & 0x03)     // Set Rx memory size
			{
			case 0:
				RSIZE[i] = (int16_t)(1024);
				RMASK[i] = (uint16_t)(0x03FF);
				break;
			case 1:
				RSIZE[i] = (int16_t)(2048);
				RMASK[i] = (uint16_t)(0x07FF);
				break;
			case 2:
				RSIZE[i] = (int16_t)(4096);
				RMASK[i] = (uint16_t)(0x0FFF);
				break;
			case 3:
				RSIZE[i] = (int16_t)(8192);
				RMASK[i] = (uint16_t)(0x1FFF);
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
		xSerialPrintf_P(PSTR("%d : %.4x : %.4x : %.4x : %.4x\r\n"), i, (uint16_t)SBUFBASEADDRESS[i], (uint16_t)RBUFBASEADDRESS[i], SSIZE[i], RSIZE[i]);
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
@brief	This function sets up gateway IP address.
*/
void setGAR(
	uint8_t * addr	/**< a pointer to a 4 -byte array responsible to set the Gateway IP address. */
	)
{
    WIZCHIP_write_buf(GAR0, addr, 4);
}

/**
@brief	These below functions are used to get the Gateway, SubnetMask
		and Source Hardware Address (MAC Address) and Source IP address
*/
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
	// get off-chip subnet mask address - solve Errata 2 & 3 v1.6
	addr->lVal = SUBN_VAR.lVal;
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
@brief	This function sets up Source IP address.
*/
void setSIPR(
	uint8_t * addr	/**< a pointer to a 4 -byte array responsible to set the Source IP address. */
	)
{
    WIZCHIP_write_buf(SIPR0, addr, 4);
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
	WIZCHIP_write((RTR1),(uint8_t)(timeout & 0x00ff));
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
@brief	This function sets up MAC address.
*/
void getSHAR(uint8_t * addr)
{
    WIZCHIP_read_buf(SHAR0, addr, 6);
}

void getSIPR(uint8_t * addr)
{
    WIZCHIP_read_buf(SIPR0, addr, 4);
}


/**
@brief	These below functions are used to get the Destination Hardware Address (MAC Address), Destination IP address and Destination Port.
*/
void getSn_DHAR(SOCKET s, uint8_t * addr)
{
    WIZCHIP_read_buf(Sn_DHAR0(s), addr, 6);
}

void setSn_DHAR(SOCKET s, uint8_t * addr)
{
    WIZCHIP_write_buf(Sn_DHAR0(s), addr, 6);
}

void getSn_DIPR(SOCKET s, uint8_t * addr)
{
    WIZCHIP_read_buf(Sn_DIPR0(s), addr, 4);
}

void setSn_DIPR(SOCKET s, uint8_t * addr)
{
    WIZCHIP_write_buf(Sn_DIPR0(s), addr, 4);
}

void getSn_DPORT(SOCKET s, uint8_t * addr)
{
	addr[0] = WIZCHIP_read(Sn_DPORT0(s));
	addr[1] = WIZCHIP_read(Sn_DPORT1(s));
}

void setSn_DPORT(SOCKET s, uint8_t * addr)
{
	WIZCHIP_write(Sn_DPORT0(s), addr[0]);
	WIZCHIP_write(Sn_DPORT1(s), addr[1]);
}

/**
@brief	This sets the maximum segment size of TCP in Active Mode), while in Passive Mode this is set by peer
*/
void setSn_MSS(SOCKET s, uint16_t mssr)
{
	WIZCHIP_write(Sn_MSSR0(s),(uint8_t)((mssr & 0xff00) >> 8));
	WIZCHIP_write(Sn_MSSR1(s),(uint8_t)(mssr & 0x00ff));
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
		val1 = ((val1 & 0x00ff) << 8) + WIZCHIP_read(Sn_TX_FSR1(s));
		if (val1 != 0)
		{
			val = WIZCHIP_read(Sn_TX_FSR0(s));
			val = ((val & 0x00ff) << 8) + WIZCHIP_read(Sn_TX_FSR1(s));
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
		val1 = ((val1 & 0x00ff) << 8) + WIZCHIP_read(Sn_RX_RSR1(s));
		if(val1 != 0)
		{
			val = WIZCHIP_read(Sn_RX_RSR0(s));
			val = ((val & 0x00ff) << 8) + WIZCHIP_read(Sn_RX_RSR1(s));
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

	// As long as the Tx read pointer and the Tx write pointer are not equal, W5100 is busy.
/*	while( !( (((uint16_t)((WIZCHIP_read(Sn_TX_RD0(s))) & 0x00ff) << 8) + WIZCHIP_read(Sn_TX_RD1(s))) == ( ptr = (((uint16_t)((WIZCHIP_read(Sn_TX_WR0(s))) & 0x00ff) << 8) + WIZCHIP_read(Sn_TX_WR1(s))))))
		_delay_us(40); // So, wait! -> Phillip 3.4.2012 // */

	ptr = WIZCHIP_read(Sn_TX_WR0(s)); // xxx Wrap this up in above test. Save a few read cycles.
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
void WIZCHIP_write_data(SOCKET s, volatile uint8_t * src, volatile uint8_t * dst, uint16_t len)
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
void WIZCHIP_read_data(SOCKET s, volatile uint8_t * src, volatile uint8_t * dst, uint16_t len)
{
	uint16_t size;
	uint16_t src_mask;
	uint8_t * src_ptr;

	src_mask = (uint16_t)src & WIZCHIP_getRxMASK(s);
	src_ptr = (uint8_t *)(WIZCHIP_getRxBASE(s) + src_mask);

	if( (src_mask + len) > WIZCHIP_getRxMAX(s) )
	{
		size = WIZCHIP_getRxMAX(s) - src_mask;
		WIZCHIP_read_buf((uint16_t)src_ptr, (uint8_t *)dst, size);
		dst += size;
		size = len - size;
		src_ptr = (uint8_t *)(WIZCHIP_getRxBASE(s));
		WIZCHIP_read_buf((uint16_t)src_ptr, (uint8_t *)dst, size);
	}
	else
	{
		WIZCHIP_read_buf((uint16_t)src_ptr, (uint8_t *)dst, len);
	}
}

#endif // #if   (_WIZCHIP_ == 5100)		// Definition in freeRTOSBoardDefs.h
