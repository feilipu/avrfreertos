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
 *                   				: Use W5100_WRITE() function in Direct or SPI mode.
 * ----------	-------		------------------------------------------------
 * 03/21/2008	1.5			Bug is Fixed in the pppinit() function.
 *							: do not clear receive buffer, so fixed. +200903[bj] clear receive buffer
 * ----------	-------		------------------------------------------------
  * 03/13/2012	1.6			Added clearSUBR(), applySUBR() and modified setSUBR() functions
 *							      because of the ARP errata.
 *							Keep SUBR 0.0.0.0 unless using TCP connect() or UDP sendto()
 *							Use the SUBN_VAR variable to read the real subnet.
 * ----------	-------		------------------------------------------------
 */

#include <stdio.h>
#include <string.h>

#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h> // for wait function

/* Scheduler include files. */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include <spi.h>

#include <w5100.h>
#include <socket.h>

#ifdef __DEF_W5100_DBG__
#include <lib_serial.h>
#endif

#ifdef __DEF_W5100_PPP__
   #include "md5.h"
#endif


static uint8_t 	I_STATUS[MAX_SOCK_NUM];
static uint16_t SMASK[MAX_SOCK_NUM]; /**< Variable for Tx buffer MASK in each channel */
static uint16_t RMASK[MAX_SOCK_NUM]; /**< Variable for Rx buffer MASK in each channel */
static uint16_t SSIZE[MAX_SOCK_NUM]; /**< Max Tx buffer size by each channel */
static uint16_t RSIZE[MAX_SOCK_NUM]; /**< Max Rx buffer size by each channel */
static uint16_t SBUFBASEADDRESS[MAX_SOCK_NUM]; /**< Tx buffer base address by each channel */
static uint16_t RBUFBASEADDRESS[MAX_SOCK_NUM]; /**< Rx buffer base address by each channel */

static uint8_t SUBN_VAR[4]; // off-chip subnet mask address - solve Errata 2 & 3 v1.6 - March 2012

uint8_t W5100_getISR(uint8_t s)
{
	return I_STATUS[s];
}
void W5100_putISR(uint8_t s, uint8_t val)
{
   I_STATUS[s] = val;
}
uint16_t getW5100_RxMAX(uint8_t s)
{
   return RSIZE[s];
}
uint16_t getW5100_TxMAX(uint8_t s)
{
   return SSIZE[s];
}
uint16_t getW5100_RxMASK(uint8_t s)
{
   return RMASK[s];
}
uint16_t getW5100_TxMASK(uint8_t s)
{
   return SMASK[s];
}
uint16_t getW5100_RxBASE(uint8_t s)
{
   return RBUFBASEADDRESS[s];
}
uint16_t getW5100_TxBASE(uint8_t s)
{
   return SBUFBASEADDRESS[s];
}

 /**
@brief	This function writes the data into W5100 registers.
*/
uint8_t W5100_WRITE(uint16_t addr, uint8_t data)
{
	register uint8_t TxByte;

	W5100_ISR_DISABLE();

#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
	spiSelect(SS_PB4);    // CS=0, SPI start, get semaphore
#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
	spiSelect(SS_PB2);
#endif

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

#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
	spiDeselect(SS_PB4);	// CS=1, SPI end, give semaphore
#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
	spiDeselect(SS_PB2);
#endif

	W5100_ISR_ENABLE();

	return 1;
}


/**
@brief	This function reads the value from W5100 registers.
*/
uint8_t W5100_READ(uint16_t addr)
{
	register uint8_t RxByte;

	W5100_ISR_DISABLE();

#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
	spiSelect(SS_PB4);    // CS=0, SPI start, get semaphore
#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
	spiSelect(SS_PB2);
#endif

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

#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
	spiDeselect(SS_PB4);	// CS=1, SPI end, give semaphore
#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
	spiDeselect(SS_PB2);
#endif

	W5100_ISR_ENABLE();

	return RxByte;
}


/**
@brief	This function writes into W5100 memory (Buffer)
*/
uint16_t wiz_write_buf(uint16_t addr, uint8_t *buf, uint16_t len)
{
	register uint8_t TxByte;
	uint16_t i;

	W5100_ISR_DISABLE();

	// Make sure you manually pull slave select low to indicate start of transfer.
	// That is NOT done by this function..., because...
	// Some devices need to have their SS held low across multiple transfer calls.
	// Using spiSelect (SS_pin);
	// But, this one needs the SS pin toggled to finish a transmission.
	// So call the fast macro to make it snappy. No need to release semaphore.
#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
	spiSelect(SS_PB4);    // SS=0, SPI start, get semaphore
#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
	spiSelect(SS_PB2);
#endif

	// If the SPI module has not been enabled yet, then return with nothing.
	if ( !(SPCR & _BV(SPE)) ) return 0;

	// The SPI module is enabled, but it is in slave mode, so we can not
	// transmit the byte.  This can happen if SSbar is an input and it went low.
	// We will try to recover by setting the MSTR bit.
	// Check this once only at the start. Assume that things don't change.
	if ( !(SPCR & _BV(MSTR)) ) SPCR |= _BV(MSTR);
	if ( !(SPCR & _BV(MSTR)) ) return 0;

#ifdef __DEF_W5100_DBG__
	xSerialPrintf_P(PSTR("wiz_write_buf: tx_ptr: %.4x "), addr);
#endif

	for( i=0; i<len; ++i)
	{

		portENTER_CRITICAL();

#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		SPI_SS_MEGA_WIZNET(0);	// SS=0, SPI start
#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
		SPI_SS(0);	// SS=0, SPI start
#endif

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

#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		SPI_SS_MEGA_WIZNET(1);	// SS=1, SPI stop, but keep semaphore
#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
		SPI_SS(1);	// SS=1, SPI stop
#endif

		portEXIT_CRITICAL();
	}

	if ( !(SPCR & _BV(MSTR)) ) return 0; 	// The SPI module is enabled, but it is in slave mode.

#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
	spiDeselect(SS_PB4);	// SS=1, SPI end
#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
	spiDeselect(SS_PB2);
#endif

	W5100_ISR_ENABLE();

#ifdef __DEF_W5100_DBG__
	xSerialPrintf_P(PSTR(" %.4x tx_len: %.4x\r\n"), addr+i, len);
#endif

	return len;
}


/**
@brief	This function reads from W5100 memory (Buffer)
*/
uint16_t wiz_read_buf(uint16_t addr, uint8_t *buf, uint16_t len)
{
	register uint8_t RxByte;
	uint16_t i;

	W5100_ISR_DISABLE();

	// Make sure you manually pull slave select low to indicate start of transfer.
	// That is NOT done by this function..., because...
	// Some devices need to have their SS held low across multiple transfer calls.
	// Using spiSelect (SS_pin);
	// But, this one needs the SS pin toggled to finish a transmission.
	// So call the fast macro to make it snappy. No need to release semaphore.
#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
	spiSelect(SS_PB4);    // SS=0, SPI start
#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
	spiSelect(SS_PB2);
#endif

	// If the SPI module has not been enabled yet, then return with nothing.
	if ( !(SPCR & _BV(SPE)) ) return 0;

	// The SPI module is enabled, but it is in slave mode, so we can not
	// transmit the byte.  This can happen if SSbar is an input and it went low.
	// We will try to recover by setting the MSTR bit.
	// Check this once only at the start. Assume that things don't change.
	if ( !(SPCR & _BV(MSTR)) ) SPCR |= _BV(MSTR);
	if ( !(SPCR & _BV(MSTR)) ) return 0;

#ifdef __DEF_W5100_DBG__
	xSerialPrintf_P(PSTR("wiz_read_buf: rx_ptr: %.4x "), addr);
#endif

	for ( i=0; i<len; ++i)
    {

		portENTER_CRITICAL();

#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		SPI_SS_MEGA_WIZNET(0);	// SS=0, SPI start
#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
		SPI_SS(0);	// SS=0, SPI start
#endif

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

#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		SPI_SS_MEGA_WIZNET(1);	// SS=1, SPI stop, but keep semaphore
#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
		SPI_SS(1);
#endif

		portEXIT_CRITICAL();
	}

	if ( !(SPCR & _BV(MSTR)) ) return 0; 	// The SPI module is enabled, but it is in slave mode.

#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
	spiDeselect(SS_PB4);	// SS=1, SPI end
#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
	spiDeselect(SS_PB2);
#endif

	W5100_ISR_ENABLE();

#ifdef __DEF_W5100_DBG__
	xSerialPrintf_P(PSTR(" %.4x rx_len: %.4x\r\n"), addr+i, len);
#endif

	return len;
}


/**
@brief	Socket interrupt routine
*/
#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
ISR(INT4_vect)
#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega1284P__) // added this || defined(__AVR_ATmega1284P__)
ISR(INT0_vect)
#endif
{
#ifdef __DEF_W5100_INT__
	uint8_t int_val;

	W5100_ISR_DISABLE();
	int_val = W5100_READ(IR);

	/* +200801[bj] process all of interrupt */
   do {
   /*---*/

   	if (int_val & IR_CONFLICT)
   	{
   		xSerialPrintf_P(PSTR("IP conflict : %.2x\r\n"), int_val);
   	}
   	if (int_val & IR_UNREACH)
   	{
   		xSerialPrintf_P(PSTR("INT Port Unreachable : %.2x\r\n"), int_val);
   		xSerialPrintf_P(PSTR("UIPR0 : %d.%d.%d.%d\r\n"), W5100_READ(UIPR0), W5100_READ(UIPR0+1), W5100_READ(UIPR0+2), W5100_READ(UIPR0+3));
   		xSerialPrintf_P(PSTR("UPORT0 : %.2x %.2x\r\n"), W5100_READ(UPORT0), W5100_READ(UPORT0+1));
   	}

   	/* +200801[bj] interrupt clear */
   	W5100_WRITE(IR, 0xf0);
      /*---*/

   	if (int_val & IR_SOCK(0))
   	{
   	/* +-200801[bj] save interrupt value*/
   		I_STATUS[0] |= W5100_READ(Sn_IR(0)); // can be come to over two times.
   		W5100_WRITE(Sn_IR(0), I_STATUS[0]);
      /*---*/
   	}
   	if (int_val & IR_SOCK(1))
   	{
   	/* +-200801[bj] save interrupt value*/
   		I_STATUS[1] |= W5100_READ(Sn_IR(1));
   		W5100_WRITE(Sn_IR(1), I_STATUS[1]);
      /*---*/
   	}
   	if (int_val & IR_SOCK(2))
   	{
   	/* +-200801[bj] save interrupt value*/
   		I_STATUS[2] |= W5100_READ(Sn_IR(2));
   		W5100_WRITE(Sn_IR(2), I_STATUS[2]);
      /*---*/
   	}
   	if (int_val & IR_SOCK(3))
   	{
   	/* +-200801[bj] save interrupt value*/
   		I_STATUS[3] |= W5100_READ(Sn_IR(3));
   		W5100_WRITE(Sn_IR(3), I_STATUS[3]);
      /*---*/
   	}

   	/* +-200801[bj] re-read interrupt value*/
   	int_val = W5100_READ(IR);

	/* +200801[bj] if exist, continue to process */
   } while (int_val != 0x00);
   /*---*/

	W5100_ISR_ENABLE();

#endif

}

/**
 * @brief	This function is for resetting of the W5100.
 * 			Initialises the W5100 to work in SPI mode
 */
void W5100_init(void)
{

	_delay_ms(100); // AVR can't delay its boot long as the W5100 needs to, so add 100ms wait before we fire up W5100.

#if  ( defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__))

	SPI_PORT |= SPI_BIT_SS_MEGA_WIZNET; // enable the EtherMega W5100 with SS on PB4
	SPI_SS_MEGA_WIZNET(1);
	spiBegin(SS_PB4);

#else // Assume standard Arduino. There will be others also required, but not just yet.

	SPI_PORT_DIR |= SPI_BIT_SS; // enable the Arduino UNO on Pin10 with SS on PB2
	SPI_SS(1);
	spiBegin(SS_PB2);

#endif
	/* Enable SPI function in mode 0 */
	spiSetDataMode(SPI_MODE0);

	/* SPI at maximum speed, half of CPU clock  */
	spiSetClockDivider(SPI_CLOCK_DIV2);

	setMR( MR_RST ); // reset the W5100 chip.
	_delay_ms(50);  // datasheet says 10ms.

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
void W5100_sysinit(
	uint8_t tx_size, 	/**< tx_size Tx memory size (0x00 - 1KByte, 0x01- 2KByte, 0x10 - 4KByte, 0x11 - 8KByte) */
	uint8_t rx_size		/**< rx_size Rx memory size (0x00 - 1KByte, 0x01- 2KByte, 0x10 - 4KByte, 0x11 - 8KByte) */
	)
{
	int16_t ssum,rsum;

#ifdef __DEF_W5100_DBG__
	xSerialPrint_P(PSTR(" sysinit()\r\n"));
#endif

	ssum = 0;
	rsum = 0;

	W5100_WRITE(TMSR,tx_size);	/* Set Tx memory size for each channel */
	W5100_WRITE(RMSR,rx_size);	/* Set Rx memory size for each channel */

	SBUFBASEADDRESS[0] = (uint16_t)(__DEF_W5100_MAP_TXBUF__);		/* Set base address of Tx memory for channel #0 */
	RBUFBASEADDRESS[0] = (uint16_t)(__DEF_W5100_MAP_RXBUF__);		/* Set base address of Rx memory for channel #0 */

#ifdef __DEF_W5100_DBG__
	xSerialPrint_P(PSTR("Channel : SEND MEM SIZE : RECV MEM SIZE\r\n"));
#endif

   for (uint8_t i = 0 ; i < MAX_SOCK_NUM; ++i)       // Set the size, masking and base address of Tx & Rx memory by each channel
	{
		SSIZE[i] = (int16_t)(0);
		RSIZE[i] = (int16_t)(0);
		if (ssum < 8192)
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
		if (rsum < 8192)
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
#ifdef __DEF_W5100_DBG__
		xSerialPrintf_P(PSTR("%d : %.4x : %.4x : %.4x : %.4x\r\n"), i, (uint16_t)SBUFBASEADDRESS[i], (uint16_t)RBUFBASEADDRESS[i], SSIZE[i], RSIZE[i]);
#endif
	}
}


void setMR(uint8_t val)
{
	/* 	DIRECT ACCESS	*/
	W5100_WRITE(MR,val);
}


/**
@brief	This function sets up gateway IP address.
*/
void setGAR(
	uint8_t * addr	/**< a pointer to a 4 -byte array responsible to set the Gateway IP address. */
	)
{
	W5100_WRITE((GAR0 + 0),addr[0]);
	W5100_WRITE((GAR0 + 1),addr[1]);
	W5100_WRITE((GAR0 + 2),addr[2]);
	W5100_WRITE((GAR0 + 3),addr[3]);
}

void getGWIP(uint8_t * addr)
{
	addr[0] = W5100_READ((GAR0 + 0));
	addr[1] = W5100_READ((GAR0 + 1));
	addr[2] = W5100_READ((GAR0 + 2));
	addr[3] = W5100_READ((GAR0 + 3));
}


/**
@brief	It sets up SubnetMask address
*/
void setSUBR(
	uint8_t * addr	/**< a pointer to a 4 -byte array responsible to set the SubnetMask address */
	)
{
	// write to off-chip subnet mask address - solve Errata 2 & 3 v1.6
	// Basically the hardware ARP engine is broken, unless it is set to 0.0.0.0
	// so we have to keep it so, unless we're using TCP connect() or UDP sendto()
	SUBN_VAR[0] = addr[0];
	SUBN_VAR[1] = addr[1];
	SUBN_VAR[2] = addr[2];
	SUBN_VAR[3] = addr[3];

//	W5100_WRITE((SUBR0 + 0),addr[0]);
//	W5100_WRITE((SUBR0 + 1),addr[1]);
//	W5100_WRITE((SUBR0 + 2),addr[2]);
//	W5100_WRITE((SUBR0 + 3),addr[3]);
}


/**
@brief	It sets up SubnetMask address
*/
void applySUBR(
	void
	)
{
	// apply off-chip subnet mask address - solve Errata 2 & 3 v1.6
	W5100_WRITE((SUBR0 + 0), SUBN_VAR[0]);
	W5100_WRITE((SUBR0 + 1), SUBN_VAR[1]);
	W5100_WRITE((SUBR0 + 2), SUBN_VAR[2]);
	W5100_WRITE((SUBR0 + 3), SUBN_VAR[3]);
}


/**
@brief	It sets up SubnetMask address
*/
void clearSUBR(
	void
	)
{
	// clear on-chip subnet mask address - solve Errata 2 & 3 v1.6
	W5100_WRITE((SUBR0 + 0), 0);
	W5100_WRITE((SUBR0 + 1), 0);
	W5100_WRITE((SUBR0 + 2), 0);
	W5100_WRITE((SUBR0 + 3), 0);
}


/**
@brief	This function sets up MAC address.
*/
void setSHAR(
	uint8_t * addr	/**< a pointer to a 6 -byte array responsible to set the MAC address. */
	)
{
	W5100_WRITE((SHAR0 + 0),addr[0]);
	W5100_WRITE((SHAR0 + 1),addr[1]);
	W5100_WRITE((SHAR0 + 2),addr[2]);
	W5100_WRITE((SHAR0 + 3),addr[3]);
	W5100_WRITE((SHAR0 + 4),addr[4]);
	W5100_WRITE((SHAR0 + 5),addr[5]);
}


/**
@brief	This function sets up Source IP address.
*/
void setSIPR(
	uint8_t * addr	/**< a pointer to a 4 -byte array responsible to set the Source IP address. */
	)
{
	W5100_WRITE((SIPR0 + 0),addr[0]);
	W5100_WRITE((SIPR0 + 1),addr[1]);
	W5100_WRITE((SIPR0 + 2),addr[2]);
	W5100_WRITE((SIPR0 + 3),addr[3]);
}


/**
@brief	This function gets Interrupt register in common register.
 */
uint8_t getIR( void )
{
   return W5100_READ(IR);
}


/**
@brief	This function sets up Retransmission time.

If there is no response from the peer or delay in response then retransmission
will be there as per RTR (Retry Time-value Register) setting
*/
void setRTR(uint16_t timeout)
{
	W5100_WRITE(RTR0,(uint8_t)((timeout & 0xff00) >> 8));
	W5100_WRITE((RTR0 + 1),(uint8_t)(timeout & 0x00ff));
}


/**
@brief	This function set the number of Retransmission.

If there is no response from the peer or delay in response then recorded time
as per RTR & RCR register setting then time out will occur.
*/
void setRCR(uint8_t retry)
{
	W5100_WRITE(RCR,retry);
}


/**
@brief	This function set the interrupt mask Enable/Disable appropriate Interrupt. ('1' : interrupt enable)

If any bit in IMR is set as '0' then there is not interrupt signal though the bit is
set in IR register.
*/
void setIMR(uint8_t mask)
{
	W5100_WRITE(IMR,mask); // must be set to 0x10.
}


/**
@brief	These below functions are used to get the Gateway, SubnetMask
		and Source Hardware Address (MAC Address) and Source IP address
*/
void getGAR(uint8_t * addr)
{
	addr[0] = W5100_READ(GAR0);
	addr[1] = W5100_READ(GAR0+1);
	addr[2] = W5100_READ(GAR0+2);
	addr[3] = W5100_READ(GAR0+3);
}
void getSUBR(uint8_t * addr)
{
	// retrieve off-chip subnet mask address - solve Errata 2 & 3 v1.6
	addr[0] = SUBN_VAR[0];
	addr[1] = SUBN_VAR[1];
	addr[2] = SUBN_VAR[2];
	addr[3] = SUBN_VAR[3];

//	addr[0] = W5100_READ(SUBR0);
//	addr[1] = W5100_READ(SUBR0+1);
//	addr[2] = W5100_READ(SUBR0+2);
//	addr[3] = W5100_READ(SUBR0+3);
}
void getSHAR(uint8_t * addr)
{
	addr[0] = W5100_READ(SHAR0);
	addr[1] = W5100_READ(SHAR0+1);
	addr[2] = W5100_READ(SHAR0+2);
	addr[3] = W5100_READ(SHAR0+3);
	addr[4] = W5100_READ(SHAR0+4);
	addr[5] = W5100_READ(SHAR0+5);
}
void getSIPR(uint8_t * addr)
{
	addr[0] = W5100_READ(SIPR0);
	addr[1] = W5100_READ(SIPR0+1);
	addr[2] = W5100_READ(SIPR0+2);
	addr[3] = W5100_READ(SIPR0+3);
}


/**
@brief	These below functions are used to get the Destination Hardware Address (MAC Address), Destination IP address and Destination Port.
*/
void getSn_DHAR(SOCKET s, uint8_t * addr)
{
	addr[0] = W5100_READ(Sn_DHAR0(s));
	addr[1] = W5100_READ(Sn_DHAR0(s)+1);
	addr[2] = W5100_READ(Sn_DHAR0(s)+2);
	addr[3] = W5100_READ(Sn_DHAR0(s)+3);
	addr[4] = W5100_READ(Sn_DHAR0(s)+4);
	addr[5] = W5100_READ(Sn_DHAR0(s)+5);
}
void setSn_DHAR(SOCKET s, uint8_t * addr)
{
	W5100_WRITE((Sn_DHAR0(s) + 0),addr[0]);
	W5100_WRITE((Sn_DHAR0(s) + 1),addr[1]);
	W5100_WRITE((Sn_DHAR0(s) + 2),addr[2]);
	W5100_WRITE((Sn_DHAR0(s) + 3),addr[3]);
	W5100_WRITE((Sn_DHAR0(s) + 4),addr[4]);
	W5100_WRITE((Sn_DHAR0(s) + 5),addr[5]);
}
void getSn_DIPR(SOCKET s, uint8_t * addr)
{
	addr[0] = W5100_READ(Sn_DIPR0(s));
	addr[1] = W5100_READ(Sn_DIPR0(s)+1);
	addr[2] = W5100_READ(Sn_DIPR0(s)+2);
	addr[3] = W5100_READ(Sn_DIPR0(s)+3);
}
void setSn_DIPR(SOCKET s, uint8_t * addr)
{
	W5100_WRITE((Sn_DIPR0(s) + 0),addr[0]);
	W5100_WRITE((Sn_DIPR0(s) + 1),addr[1]);
	W5100_WRITE((Sn_DIPR0(s) + 2),addr[2]);
	W5100_WRITE((Sn_DIPR0(s) + 3),addr[3]);
}
void getSn_DPORT(SOCKET s, uint8_t * addr)
{
	addr[0] = W5100_READ(Sn_DPORT0(s));
	addr[1] = W5100_READ(Sn_DPORT1(s));
}
void setSn_DPORT(SOCKET s, uint8_t * addr)
{
	W5100_WRITE(Sn_DPORT0(s), addr[0]);
	W5100_WRITE(Sn_DPORT1(s), addr[1]);
}


/**
@brief	This sets the maximum segment size of TCP in Active Mode), while in Passive Mode this is set by peer
*/
void setSn_MSS(SOCKET s, uint16_t mssr)
{
	W5100_WRITE(Sn_MSSR0(s),(uint8_t)((mssr & 0xff00) >> 8));
	W5100_WRITE(Sn_MSSR1(s),(uint8_t)(mssr & 0x00ff));
}

void setSn_TTL(SOCKET s, uint8_t ttl)
{
   W5100_WRITE(Sn_TTL(s), ttl);
}


/**
@brief	These below function is used to setup the Protocol Field of IP Header when
		executing the IP Layer RAW mode.
*/
void setSn_PROTO(SOCKET s, uint8_t proto)
{
	W5100_WRITE(Sn_PROTO(s),proto);
}


/**
@brief	get socket interrupt status

These below functions are used to read the Interrupt & Socket Status register
*/
uint8_t getSn_IR(SOCKET s)
{
   return W5100_READ(Sn_IR(s));
}


/**
@brief	 get socket status
*/
uint8_t getSn_SR(SOCKET s)
{
   return W5100_READ(Sn_SR(s));
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
		val1 = W5100_READ(Sn_TX_FSR0(s));
		val1 = ((val1 & 0x00ff) << 8) + W5100_READ(Sn_TX_FSR1(s));
		if (val1 != 0)
		{
			val = W5100_READ(Sn_TX_FSR0(s));
			val = ((val & 0x00ff) << 8) + W5100_READ(Sn_TX_FSR1(s));
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
		val1 = W5100_READ(Sn_RX_RSR0(s));
		val1 = ((val1 & 0x00ff) << 8) + W5100_READ(Sn_RX_RSR1(s));
		if(val1 != 0)
		{
			val = W5100_READ(Sn_RX_RSR0(s));
			val = ((val & 0x00ff) << 8) + W5100_READ(Sn_RX_RSR1(s));
		}
	} while (val != val1);
	return val;
}


/**
@brief	 This function is being called by TCP send() and UDP & IP_RAW sendto() function.

This function reads the Tx write pointer register and after copy the data in buffer update the Tx write pointer
register. User should read upper byte first and lower byte later to get proper value.
*/
void send_data_processing(SOCKET s, uint8_t *data, uint16_t len)
{
	uint16_t ptr;

	// As long as the Tx read pointer and the Tx write pointer are not equal, W5100 is busy.
	while( !( (((uint16_t)((W5100_READ(Sn_TX_RD0(s))) & 0x00ff) << 8) + W5100_READ(Sn_TX_RD1(s))) ==
			  ( ptr = (((uint16_t)((W5100_READ(Sn_TX_WR0(s))) & 0x00ff) << 8) + W5100_READ(Sn_TX_WR1(s))))))
		_delay_us(40); // So, wait! -> Phillip 3.4.2012

//	ptr = W5100_READ(Sn_TX_WR0(s)); // Wrap this up in above test. Save a few read cycles.
//	ptr = ((ptr & 0x00ff) << 8) + W5100_READ(Sn_TX_WR1(s));

#ifdef __DEF_W5100_DBG__
	xSerialPrintf_P(PSTR("ISR_TX: tx_ptr: %.4x tx_len: %.4x\r\n"), ptr, len);
#endif

	write_data(s, data, (uint8_t *)ptr, len);
	ptr += len;
	W5100_WRITE(Sn_TX_WR0(s), (uint8_t)((ptr & 0xff00) >> 8));
	W5100_WRITE(Sn_TX_WR1(s), (uint8_t)(ptr & 0x00ff));
}


/**
@brief	This function is being called by TCP recv().

This function read the Rx read pointer register
and after copy the data from receive buffer update the Rx write pointer register.
User should read upper byte first and lower byte later to get proper value.
*/
void recv_data_processing(SOCKET s, uint8_t *data, uint16_t len)
{
	uint16_t ptr;
	ptr = W5100_READ(Sn_RX_RD0(s));
	ptr = ((ptr & 0x00ff) << 8) + W5100_READ(Sn_RX_RD1(s));
#ifdef __DEF_W5100_DBG__
	xSerialPrintf_P(PSTR("ISR_RX: rd_ptr: %.4x rd_len: %.4x\r\n"), ptr, len);
#endif
	read_data(s, (uint8_t *)ptr, data, len); // read data
	ptr += len;
	W5100_WRITE(Sn_RX_RD0(s), (uint8_t)((ptr & 0xff00) >> 8));
	W5100_WRITE(Sn_RX_RD1(s), (uint8_t)(ptr & 0x00ff));
}


/**
@brief	for copy the data from application buffer to Transmit buffer of the chip.

This function is being used for copy the data from application buffer to Transmit
buffer of the chip. It calculates the actual physical address where one has to write
the data in transmit buffer. Here also takes care of the condition that it exceeds
the Tx memory upper-bound of socket.
*/
void write_data(SOCKET s, volatile uint8_t * src, volatile uint8_t * dst, uint16_t len)
{
	uint16_t size;
	uint16_t dst_mask;
	uint8_t * dst_ptr;

	dst_mask = (uint16_t)dst & getW5100_TxMASK(s);
	dst_ptr = (uint8_t *)(getW5100_TxBASE(s) + dst_mask);

	if (dst_mask + len > getW5100_TxMAX(s))
	{
		size = getW5100_TxMAX(s) - dst_mask;
		wiz_write_buf((uint16_t)dst_ptr, (uint8_t*)src, size);
		src += size;
		size = len - size;
		dst_ptr = (uint8_t *)(getW5100_TxBASE(s));
		wiz_write_buf((uint16_t)dst_ptr, (uint8_t*)src, size);
	}
	else
	{
		wiz_write_buf((uint16_t)dst_ptr, (uint8_t*)src, len);
	}
}


/**
@brief	This function is being used for copy the data from Receive buffer of the chip to application buffer.

It calculate the actual physical address where one has to read
the data from Receive buffer. Here also take care of the condition that it exceeds
the Rx memory upper-bound of socket.
*/
void read_data(SOCKET s, volatile uint8_t * src, volatile uint8_t * dst, uint16_t len)
{
	uint16_t size;
	uint16_t src_mask;
	uint8_t * src_ptr;

	src_mask = (uint16_t)src & getW5100_RxMASK(s);
	src_ptr = (uint8_t *)(getW5100_RxBASE(s) + src_mask);

	if( (src_mask + len) > getW5100_RxMAX(s) )
	{
		size = getW5100_RxMAX(s) - src_mask;
		wiz_read_buf((uint16_t)src_ptr, (uint8_t*)dst, size);
		dst += size;
		size = len - size;
		src_ptr = (uint8_t *)(getW5100_RxBASE(s));
		wiz_read_buf((uint16_t)src_ptr, (uint8_t*)dst, size);
	}
	else
	{
		wiz_read_buf((uint16_t)src_ptr, (uint8_t*)dst, len);
	}
}


#ifdef __DEF_W5100_PPP__
#define PPP_OPTION_BUF_LEN 64

uint8_t pppinit_in(uint8_t * id, uint8_t idlen, uint8_t * passwd, uint8_t passwdlen);


/**
@brief	make PPPoE connection
@return	1 => success to connect, 2 => Auth fail, 3 => timeout, 4 => Auth type not support

*/
uint8_t pppinit(uint8_t * id, uint8_t idlen, uint8_t * passwd, uint8_t passwdlen)
{
	uint8_t ret;
	uint8_t isr;

	// PHASE0. W5100 PPPoE(ADSL) setup
	// enable pppoe mode
	xSerialPrint_P(PSTR("-- PHASE 0. W5100 PPPoE(ADSL) setup process --\r\n\n"));

	W5100_WRITE(MR,W5100_READ(MR) | MR_PPPOE);

	// open socket in pppoe mode
	isr = W5100_READ(Sn_IR(0));// first clear isr(0), W5100 at present time
	W5100_WRITE(Sn_IR(0),isr);

	W5100_WRITE(PTIMER,200); // 5sec timeout
	W5100_WRITE(PMAGIC,0x01); // magic number
	W5100_WRITE(Sn_MR(0),Sn_MR_PPPOE);
	W5100_WRITE(Sn_CR(0),Sn_CR_OPEN);

	/* +20071122[chungs]:wait to process the command... */
	while( W5100_READ(Sn_CR(0)) )
		;
	/* ------- */

	ret = pppinit_in(id, idlen, passwd, passwdlen);

	// close ppp connection socket
	/* +200801 (hwkim) */
	close(0);
	/* ------- */

	return ret;
}


uint8_t pppinit_in(uint8_t * id, uint8_t idlen, uint8_t * passwd, uint8_t passwdlen)
{
	uint8_t loop_idx = 0;
	uint8_t isr = 0;
	uint8_t buf[PPP_OPTION_BUF_LEN];
	uint16_t len;
	uint8_t str[PPP_OPTION_BUF_LEN];
	uint8_t str_idx,dst_idx;

   // PHASE1. PPPoE Discovery
	// start to connect pppoe connection
	xSerialPrint_P(PSTR("-- PHASE 1. PPPoE Discovery process --"));
	xSerialPrint_P(PSTR(" ok\r\n\n"));
	W5100_WRITE(Sn_CR(0),Sn_CR_PCON);
	/* +20071122[chungs]:wait to process the command... */
	while( W5100_READ(Sn_CR(0)) )
		;
	/* ------- */
	_delay_ms(1000);

	loop_idx = 0;
	//check whether PPPoE discovery end or not
	while (!(W5100_READ(Sn_IR(0)) & Sn_IR_PNEXT))
	{
		xSerialPrintf(".");
		if (loop_idx++ == 10) // timeout
		{
			xSerialPrint_P(PSTR("timeout before LCP\r\n"));
			return 3;
		}
		_delay_ms(1000);
	}

   /* +200801[bj] clear interrupt value*/
   W5100_WRITE(Sn_IR(0), 0xff);
   /*---*/

   // PHASE2. LCP process
	xSerialPrint_P(PSTR("-- PHASE 2. LCP process --"));

	// send LCP Request
	{
		// Magic number option
		// option format (type value + length value + data)
	   // write magic number value
		buf[0] = 0x05; // type value
		buf[1] = 0x06; // length value
		buf[2] = 0x01; buf[3] = 0x01; buf[4] = 0x01; buf[5]= 0x01; // data
		// for MRU option, 1492 0x05d4
		// buf[6] = 0x01; buf[7] = 0x04; buf[8] = 0x05; buf[9] = 0xD4;
	}
	send_data_processing(0, buf, 0x06);
	W5100_WRITE(Sn_CR(0),Sn_CR_PCR); // send request
	/* +20071122[chungs]:wait to process the command... */
	while( W5100_READ(Sn_CR(0)) )
		;
	/* ------- */

	_delay_ms(1000);

	while (!((isr = W5100_READ(Sn_IR(0))) & Sn_IR_PNEXT))
	{
		if (isr & Sn_IR_PRECV) // Not support option
		{
   /* +200801[bj] clear interrupt value*/
         W5100_WRITE(Sn_IR(0), Sn_IR_PRECV);
   /*---*/
			len = getSn_RX_RSR(0);
			if ( len > 0 )
			{
				recv_data_processing(0, str, len);
				W5100_WRITE(Sn_CR(0),Sn_CR_RECV);
				/* +20071122[chungs]:wait to process the command... */
				while( W5100_READ(Sn_CR(0)) )
					;
				/* ------- */

				// for debug
				//xSerialPrintf("LCP proc len = %d\r\n", len); for (i = 0; i < len; i++) xSerialPrintf ("%02x ", str[i]); xSerialPrintf("\r\n");
				// get option length
				len = str[4]; len = ((len & 0x00ff) << 8) + str[5];
				len += 2;
				str_idx = 6; dst_idx = 0; // ppp header is 6 byte, so starts at 6.
				do
				{
					if ((str[str_idx] == 0x01) || (str[str_idx] == 0x02) || (str[str_idx] == 0x03) || (str[str_idx] == 0x05))
					{
						// skip as length of support option. str_idx+1 is option's length.
						str_idx += str[str_idx+1];
					}
					else
					{
						// not support option , REJECT
						memcpy((uint8_t *)(buf+dst_idx), (uint8_t *)(str+str_idx), str[str_idx+1]);
						dst_idx += str[str_idx+1]; str_idx += str[str_idx+1];
					}
				} while (str_idx != len);
	   			// for debug
	   			//xSerialPrintf("LCP dst proc\r\n"); for (i = 0; i < dst_idx; i++) xSerialPrintf ("%02x ", buf[i]); xSerialPrintf("\r\n");

	   			// send LCP REJECT packet
	   			send_data_processing(0, buf, dst_idx);
	   			W5100_WRITE(Sn_CR(0),Sn_CR_PCJ);
				/* +20071122[chungs]:wait to process the command... */
				while( W5100_READ(Sn_CR(0)) )
					;
				/* ------- */
  			}
		}
		xSerialPrintf(".");
		if (loop_idx++ == 10) // timeout
		{
			xSerialPrint_P(PSTR("timeout after LCP\r\n"));
			return 3;
		}
		_delay_ms(1000);
	}
	xSerialPrint_P(PSTR(" ok\r\n\n"));

   /* +200801[bj] clear interrupt value*/
   W5100_WRITE(Sn_IR(0), 0xff);

   /* +200903[bj] clear receive buffer */
	len = getSn_RX_RSR(0);
	if ( len > 0 )
	{
		recv_data_processing(0, str, len);
		//xSerialPrintf("dummy proc len = %d\r\n", len); for (i = 0; i < len; i++) xSerialPrintf ("%02x ", str[i]); xSerialPrintf("\r\n");
		W5100_WRITE(Sn_CR(0),Sn_CR_RECV);
		while( W5100_READ(Sn_CR(0)) )
			;
	}
   /*---*/

	xSerialPrint_P(PSTR("-- PHASE 3. PPPoE(ADSL) Authentication mode --\r\n"));
	xSerialPrintf_P(PSTR("Authentication protocol : %.2x %.2x, "), W5100_READ(PATR0), W5100_READ(PATR0+1));

	loop_idx = 0;
	if (W5100_READ(PATR0) == 0xc0 && W5100_READ(PATR0+1) == 0x23)
	{
		xSerialPrintf_P(PSTR("PAP\r\n")); // in case of adsl normally supports PAP.
		// send authentication data
		// copy (idlen + id + passwdlen + passwd)
		buf[loop_idx] = idlen; loop_idx++;
		memcpy((uint8_t *)(buf+loop_idx), (uint8_t *)(id), idlen); loop_idx += idlen;
		buf[loop_idx] = passwdlen; loop_idx++;
		memcpy((uint8_t *)(buf+loop_idx), (uint8_t *)(passwd), passwdlen); loop_idx += passwdlen;
		send_data_processing(0, buf, loop_idx);
		W5100_WRITE(Sn_CR(0),Sn_CR_PCR);
		/* +20071122[chungs]:wait to process the command... */
		while( W5100_READ(Sn_CR(0)) )
			;
		/* ------- */
		_delay_ms(1000);
	}
	else if (W5100_READ(PATR0) == 0xc2 && W5100_READ(PATR0+1) == 0x23)
	{
		uint8_t chal_len;
		md5_ctx context;
		uint8_t  digest[16];

		len = getSn_RX_RSR(0);
		if ( len > 0 )
		{
			recv_data_processing(0, str, len);
			W5100_WRITE(Sn_CR(0),Sn_CR_RECV);
			/* +20071122[chungs]:wait to process the command... */
			while( W5100_READ(Sn_CR(0)) )
				;
			/* ------- */
#ifdef __DEF_W5100_DBG__
			xSerialPrint_P(PSTR("recv CHAP\r\n"));
			{
				int16_t i;

				for (i = 0; i < 32; i++)
					xSerialPrintf_P(PSTR("%02x "), str[i]);
			}
			xSerialPrintf_P(PSTR("\r\n"));
#endif
// str is C2 23 xx CHAL_ID xx xx CHAP_LEN CHAP_DATA
// index  0  1  2  3       4  5  6        7 ...

			memset(buf,0x00,64);
			buf[loop_idx] = str[3]; loop_idx++; // chal_id
			memcpy((uint8_t *)(buf+loop_idx), (uint8_t *)(passwd), passwdlen); loop_idx += passwdlen; //passwd
			chal_len = str[6]; // chal_id
			memcpy((uint8_t *)(buf+loop_idx), (uint8_t *)(str+7), chal_len); loop_idx += chal_len; //challenge
			buf[loop_idx] = 0x80;
#ifdef __DEF_W5100_DBG__
			xSerialPrint_P(PSTR("CHAP proc d1\r\n"));
			{
				int16_t i;
				for (i = 0; i < 64; i++)
					xSerialPrintf_P(PSTR("%02x "), buf[i]);
			}
			xSerialPrint_P(PSTR("\r\n"));
#endif

			md5_init(&context);
			md5_update(&context, buf, loop_idx);
			md5_final(digest, &context);

#ifdef __DEF_W5100_DBG__
			xSerialPrint_P(PSTR("CHAP proc d1\r\n"));
			{
				int16_t i;
				for (i = 0; i < 16; i++)
					xSerialPrintf_P(PSTR("%02x"), digest[i]);
			}
			xSerialPrint_P(PSTR("\r\n"));
#endif
			loop_idx = 0;
			buf[loop_idx] = 16; loop_idx++; // hash_len
			memcpy((uint8_t *)(buf+loop_idx), (uint8_t *)(digest), 16); loop_idx += 16; // hashed value
			memcpy((uint8_t *)(buf+loop_idx), (uint8_t *)(id), idlen); loop_idx += idlen; // id
			send_data_processing(0, buf, loop_idx);
			W5100_WRITE(Sn_CR(0),Sn_CR_PCR);
			/* +20071122[chungs]:wait to process the command... */
			while( W5100_READ(Sn_CR(0)) )
				;
			/* ------- */
			_delay_ms(1000);
		}
	}
	else
	{
		xSerialPrint_P(PSTR("Not support\r\n"));
#ifdef __DEF_W5100_DBG__
		xSerialPrintf_P(PSTR("Not support PPP Auth type: %.2x%.2x\r\n"),W5100_READ(PATR0), W5100_READ(PATR0+1));
#endif
		return 4;
	}
	xSerialPrintf("\r\n");

	xSerialPrint_P(PSTR("-- Waiting for PPPoE server's admission --"));
	loop_idx = 0;
	while (!((isr = W5100_READ(Sn_IR(0))) & Sn_IR_PNEXT))
	{
		if (isr & Sn_IR_PFAIL)
		{
   /* +200801[bj] clear interrupt value*/
   W5100_WRITE(Sn_IR(0), 0xff);
   /*---*/
			xSerialPrint_P(PSTR("failed\r\nReinput id, password..\r\n"));
			return 2;
		}
		xSerialPrintf(".");
		if (loop_idx++ == 10) // timeout
		{
   /* +200801[bj] clear interrupt value*/
   W5100_WRITE(Sn_IR(0), 0xff);
   /*---*/
			xSerialPrint_P(PSTR("timeout after PAP\r\n"));
			return 3;
		}
		_delay_ms(1000);
	}
   /* +200801[bj] clear interrupt value*/
   W5100_WRITE(Sn_IR(0), 0xff);

   /* +200903[bj] clear receive buffer */
	len = getSn_RX_RSR(0);
	if ( len > 0 )
	{
		recv_data_processing(0, str, len);
		//xSerialPrintf("dummy proc len = %d\r\n", len); for (i = 0; i < len; i++) xSerialPrintf ("%02x ", str[i]); xSerialPrintf("\r\n");
		W5100_WRITE(Sn_CR(0),Sn_CR_RECV);
		while( W5100_READ(Sn_CR(0)) )
			;
	}
   /*---*/

	xSerialPrint_P(PSTR("ok\r\n\n-- PHASE 4. IPCP process --"));
	// IP Address
	buf[0] = 0x03; buf[1] = 0x06; buf[2] = 0x00; buf[3] = 0x00; buf[4] = 0x00; buf[5] = 0x00;
	send_data_processing(0, buf, 6);
	W5100_WRITE(Sn_CR(0),Sn_CR_PCR);
	/* +20071122[chungs]:wait to process the command... */
	while( W5100_READ(Sn_CR(0)) )
		;
	/* ------- */
	_delay_ms(1000);

	loop_idx = 0;
	while (1)
	{
		if (W5100_READ(Sn_IR(0)) & Sn_IR_PRECV)
		{
   /* +200801[bj] clear interrupt value*/
   W5100_WRITE(Sn_IR(0), 0xff);
   /*---*/
			len = getSn_RX_RSR(0);
			if ( len > 0 )
			{
				recv_data_processing(0, str, len);
				W5100_WRITE(Sn_CR(0),Sn_CR_RECV);
				/* +20071122[chungs]:wait to process the command... */
				while( W5100_READ(Sn_CR(0)) )
					;
				/* ------- */
	   			//for debug
	   			//xSerialPrintf("IPCP proc len = %d\r\n", len); for (i = 0; i < len; i++) xSerialPrintf ("%02x ", str[i]); xSerialPrintf("\r\n");
	   			str_idx = 6; dst_idx = 0;
	   			if (str[2] == 0x03) // in case of NAK
	   			{
	   				do
	   				{
	   					if (str[str_idx] == 0x03) // request only ip information
	   					{
	   						memcpy((uint8_t *)(buf+dst_idx), (uint8_t *)(str+str_idx), str[str_idx+1]);
	   						dst_idx += str[str_idx+1]; str_idx += str[str_idx+1];
	   					}
	   					else
	   					{
	   						// skip byte
	   						str_idx += str[str_idx+1];
	   					}
	   					// for debug
	   					//xSerialPrintf("s: %d, d: %d, l: %d", str_idx, dst_idx, len);
	   				} while (str_idx != len);
	   				send_data_processing(0, buf, dst_idx);
	   				W5100_WRITE(Sn_CR(0),Sn_CR_PCR); // send ipcp request
	   				/* +20071122[chungs]:wait to process the command... */
					while( W5100_READ(Sn_CR(0)) )
						;
					/* ------- */
	   				_delay_ms(1000);
	   				break;
	   			}
			}
		}
		xSerialPrintf(".");
		if (loop_idx++ == 10) // timeout
		{
			xSerialPrint_P(PSTR("timeout after IPCP\r\n"));
			return 3;
		}
		_delay_ms(1000);
		send_data_processing(0, buf, 6);
		W5100_WRITE(Sn_CR(0),Sn_CR_PCR); //ipcp re-request
		/* +20071122[chungs]:wait to process the command... */
		while( W5100_READ(Sn_CR(0)) )
			;
		/* ------- */
	}

	loop_idx = 0;
	while (!(W5100_READ(Sn_IR(0)) & Sn_IR_PNEXT))
	{
		xSerialPrint_P(PSTR("."));
		if (loop_idx++ == 10) // timeout
		{
			xSerialPrint_P(PSTR("timeout after IPCP NAK\r\n"));
			return 3;
		}
		_delay_ms(1000);
		W5100_WRITE(Sn_CR(0),Sn_CR_PCR); // send ipcp request
		/* +20071122[chungs]:wait to process the command... */
		while( W5100_READ(Sn_CR(0)) )
			;
		/* ------- */
	}
   /* +200801[bj] clear interrupt value*/
   W5100_WRITE(Sn_IR(0), 0xff);
   /*---*/
	xSerialPrint_P(PSTR("ok\r\n\n"));
	return 1;
	// after this function, User must save the pppoe server's mac address and pppoe session id in current connection
}


/**
@brief	terminate PPPoE connection
*/
uint8_t pppterm(uint8_t * mac, uint8_t * sessionid)
{
	uint16_t i;
	uint8_t isr;
#ifdef __DEF_W5100_DBG__
	xSerialPrint_P(PSTR("pppterm()\r\n"));
#endif
	/* Set PPPoE bit in MR(Common Mode Register) : enable socket0 pppoe */
	W5100_WRITE(MR,W5100_READ(MR) | MR_PPPOE);

	// write pppoe server's mac address and session id
	// must be setted these value.
	for (i = 0; i < 6; i++) W5100_WRITE((Sn_DHAR0(0)+i),mac[i]);
	for (i = 0; i < 2; i++) W5100_WRITE((Sn_DPORT0(0)+i),sessionid[i]);
	isr = W5100_READ(Sn_IR(0));
	W5100_WRITE(Sn_IR(0),isr);

	//open socket in pppoe mode
	W5100_WRITE(Sn_MR(0),Sn_MR_PPPOE);
	W5100_WRITE(Sn_CR(0),Sn_CR_OPEN);
	/* +20071122[chungs]:wait to process the command... */
	while( W5100_READ(Sn_CR(0)) )
		;
	/* ------- */
	_delay_us(1);
	// close pppoe connection
	W5100_WRITE(Sn_CR(0),Sn_CR_PDISCON);
	/* +20071122[chungs]:wait to process the command... */
	while( W5100_READ(Sn_CR(0)) )
		;
	/* ------- */
	_delay_ms(1000);
	// close socket
	/* +200801 (hwkim) */
	close(0);
	/* ------- */


#ifdef __DEF_W5100_DBG__
	xSerialPrint_P(PSTR("pppterm() end ..\r\n"));
#endif

	return 1;
}
#endif
