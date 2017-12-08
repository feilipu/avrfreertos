/*
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */

#include <stdlib.h>				// #include <stddef.h> to use size_t
#include <stdint.h>  			// has to be added to use uint8_t
#include <avr/io.h>
#include <avr/interrupt.h>		// Needed to use interrupts
#include <util/delay_basic.h>	// Needed for _delay_loop_1()


/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "spi.h"
#include "lib_util.h"


#if defined(portEXT_RAMFS)

/* serial interface include file. */
#include <serial.h>		// temporary while debugging

#include <ramfs.h>			// access to XRAM related functions
#include <diskio.h>			// emulate the diskio.c functions here, when portEXT_RAMFS is enabled.



#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)

/******************** FUNCTIONS FOR SUPERVISOR TASK ***********************/

void init16PCINTpins(void)
{
	taskENTER_CRITICAL();				// turn off interrupts, because we don't want false triggers.

#if defined (ARDUSAT_HARDWARE)			// Most of the PCINT1 pins are unavailable on the Mega2650
//	DDRE &= ~_BV(DDE0);					// Clear the PE0 pin.                 PE0 is RX0 (PCINT8) conflicts on PCINT1.
	DDRJ &=  _BV(DDJ7);					// Clear the PJ0 through PJ6 pins.    Note: no point to turn on PCIE1. Pins not routed on Mega, etc.
	// PE0, PJ0-PJ6 are now inputs

//	PORTE |= _BV(PE0);					// Set the PE0 pin.
	PORTJ |= 0xFF & ~_BV(PJ7);			// Set the PJ0 through PJ6 pins.
	// PE0, PJ0-PJ6 are now set to 1, which implies pulled up. This keeps all the Arduino clients (SPI Slaves) de-selected and off bus.

	PCMSK1 = 0xFE;						// set the masks to allow ALL the interrupts to trigger. PCINT15 - PCINT9  PE0 = RX0 = PCINT8 conflict.
	PCICR |= _BV(PCIE1);				// set PCIE1 and PCIE2 to enable PCMSK1 and PCMSK2 scan. Note: no point to turn on PCIE1. Pins not routed on Mega, etc.
#endif

	DDRK = 0x00;						// Clear the PK0 through PK7 pins.
	// PK0-PK7 are now inputs

	PORTK = 0xFF;						// Set the PK0 through PK7 pins.
	// PK0-PK7 are now set to 1, which implies pulled up. This keeps all the Arduino clients (SPI Slaves) de-selected and off bus.

	PCMSK2 = 0xFF;						// set the masks to allow ALL the interrupts to trigger. PCINT23 - PCINT16
	PCICR |= _BV(PCIE2);				// set PCIE2 to enable PCMSK2 scan.

	taskEXIT_CRITICAL();
}



#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)

/******************** FUNCTIONS FOR CLIENT TASKS ***********************/

/* Count how much memory is allocated in the RAMFS by vRAMFSMalloc() */
static size_t xNextFreeRAMFSByte = ( size_t ) 0;

/*-----------------Private Functions ----------------------------*/

// These could be shared, but they're kind of special to the RAMFS implementation, so better not.
void spiSlaveBegin(void) __attribute__ ((hot, flatten)); // New function for Arduino Uno Clients (SPI slaves).
void spiSlaveEnd(void) __attribute__ ((hot, flatten));

/*-----------------------------------------------------------*/

void spiSlaveBegin(void)
{
	//		When the SPI is configured as a Slave, the Slave Select (SS) pin is always input. When SS is
	//		held low, the SPI is activated, and MISO becomes an output if configured so by the user. All
	//		other pins are inputs. When SS is driven high, all pins are inputs, and the SPI is passive, which
	//		means that it will not receive incoming data. Note that the SPI logic will be reset once the SS pin
	//		is driven high.

	//		The SS pin is useful for packet/byte synchronisation to keep the slave bit counter synchronous
	//		with the master clock generator. When the SS pin is driven high, the SPI slave will immediately
	//		reset the send and receive logic, and drop any partially received data in the Shift Register.

	uint8_t tmp;

	// Set MISO as Output, all others default to input.
	SPI_PORT_DIR |= SPI_BIT_MISO ;

	// Enable (MOSI Pull-Up Resistor, SS is pulled
	// high to allow Master to drive low
	SPI_PORT |= SPI_BIT_MOSI | SPI_BIT_SS;

	// Set the control register to turn ON the SPI interface in slave mode.
	SPCR = _BV(SPE);

	// Reading SPI Status Register & SPI Data Register
	// has side effect of zeroing out both, and resetting the interrupt.
	tmp = SPSR;
	tmp = SPDR;
}

void spiSlaveEnd(void)
{
	// Set the control register to turn OFF the SPI interface in slave mode
	SPCR = 0x00;

	// Set MOSI, MISO, CLK, SS as Input.
	SPI_PORT_DIR &= ~(  SPI_BIT_SCK | SPI_BIT_MISO | SPI_BIT_MOSI | SPI_BIT_SS ) ;

	// Disable Pull-Up Resistors (high impedance).
	// Let them float so as not to load the SPI bus, as there might be up to 16 other Arduino devices on the bus.
	SPI_PORT &= ~( SPI_BIT_MISO | SPI_BIT_MOSI | SPI_BIT_SCK ) ;

	// Enable pull up to make sure that noise does not generate a false interrupt.
	SPI_PORT |= SPI_BIT_SS;
}

/*------------------------------------------------------------------------*/
/* STATIC FUNCTION DEFINED AS MACRO TO MINIMISE THE CALL DEPTH AND DELAY. */
/*------------------------------------------------------------------------*/
#define ramfs_transaction_init()																						\
{																														\
	_delay_loop_1((uint8_t) (F_CPU / 3e6 * CLIENT_CALL_US) );															\
								/* Wait  to make sure the Interrupt will be registered by the Supervisor. */			\
								/* If this is called immediately after predecessor, then PCINT may not be ready */		\
								/* Delay is 31us. */																	\
																														\
	SPI_PORT_DIR |= _BV(DDB2);	/* Set SS as output, to set up trigger Supervisor PCINT Pin Change Interrupt */			\
	SPI_PORT &= ~_BV(PORTB2);	/* Set SS low, to trigger Supervisor PCINT Pin Change Interrupt */						\
																														\
	_delay_loop_1((uint8_t) (F_CPU / 3e6 * CLIENT_CALL_US) );															\
								/* Wait just long enough to make sure the Interrupt is registered by the Supervisor. */	\
								/* Measured response time is 31us. */													\
																														\
	SPI_PORT_DIR &= ~_BV(DDB2);	/* Set SS as input */																	\
	SPI_PORT &= ~_BV(PORTB2);	/* Set SS high-impedance. */															\
																														\
	_delay_loop_1( (uint8_t) (F_CPU / 3e5) );																			\
								/* delay 10us. Wait just long enough to make sure the line has risen high again. */	 	\
								/* Only pull-ups holding it up. */														\
																														\
	/* we don't use the SPI interrupt, because that implies that the SPI slave interface must be turned on, */			\
	/* and we don't want to have Arduino slaves fighting for the bus. */												\
																														\
	/* Check if my SS line has been pulled low by the Master (Supervisor).)	*/											\
	/* tested interrupts and taskYIELD(), but it is generally only a 90us wait here. */									\
	/* So we just have to busy wait. Also, easier to implement in the Arduino environment. */							\
																														\
	/* Supervisor will only wait a maximum 1.6ms for us, from when it signals us. */									\
	/* So we have respond quickly.	*/																					\
	/* Also, For a loaded system, we need to allocate up to */															\
	/* 15 (maximum nodes - 1) x 4ms delay (effective maximum transfer size) = 60ms response time in worst case. */ 		\
																														\
	index = 0;																											\
	do																													\
	{																													\
		if( CHECK_FOR_MY_SS )			/* Check if my SS line has been pulled low by the Master (Supervisor). */		\
		_delay_loop_1( (uint8_t) (F_CPU / 3e6) );	/* delay 1us x 65535 = total of 65.535ms */							\
		else																											\
			break;																										\
	} while (index++ != 0xFFFF);																						\
																														\
	if (index == 0xFFFF)			/* If we were not called, then bail out. */											\
		return 0;																										\
																														\
	taskENTER_CRITICAL();	/* turn off interrupts, because we have to be ready for the Master (Supervisor) SPI. */		\
																														\
	/* enable the SPI interface, but only after we're sure that the SPI bus is free, because the Master */				\
	/* told us so, by setting our SS line low. */																		\
	/* Enabling the SPI bus before we're allowed, is BAD. Someone else might be using it. */							\
	spiSlaveBegin();																									\
																														\
	index = 0;																											\
																														\
	do								/* send a dummy byte so we get the Supervisor SPI Master to respond correctly. */	\
	{																													\
		SPDR = 0x5A;																									\
		if( !( ++index) ) break; 	/* try until index rolls over, then bail out. */									\
		while( WAIT_FOR_SPIF );																							\
																														\
		TxRxByte = SPDR;																								\
	} while( TxRxByte != 0xA5 ); 	/*  correct response means the SPI Master has responded to our PCINT interrupt. */	\
																														\
}

/*------------------------------------------------------------------------*/


uint8_t ramfs_transfer_block(pRAMFSarray pRAMFS_block, uint8_t *data)
{
	uint16_t index;
	uint8_t TxByte, RxByte, TxRxByte;
	uint8_t * pRAM;

	pRAM = (uint8_t *) pRAMFS_block; // make this cast to serialise the Command structure.

	ramfs_transaction_init();	// set up the SPI bus for the RAMFS transaction.
								// this is VERY time critical, so we do it in a MACRO to ensure there is no loss of time.
								// Note unpaired in this function: taskENTER_CRITICAL();

	index = 0;
	SPDR = pRAM[ index++ ]; 							// Begin transmission of the command structure (serialised)

	while( index < sizeof(xRAMFSarray) )
	{
		TxByte = pRAM[ index++ ]; 						// pre-load the byte to be transmitted
		if( CHECK_FOR_MY_SS ) break;
		while( WAIT_FOR_SPIF );

		SPDR = TxByte; 									// Continue transmission of the command structure
	}

	if( CHECK_FOR_MY_SS )
	{
		spiSlaveEnd();
		taskEXIT_CRITICAL();
		return 0;
	}
	while( WAIT_FOR_SPIF );

	index = 0;
	// now we have sent the command structure, the Supervisor will know what to do with the data.
	switch( pRAMFS_block->ram_cmd )
	{
		case Read :

			SPDR  = 0xFF;								// prepare a dummy byte.
			while( index < pRAMFS_block->ram_size - 1 )
			{
				if( CHECK_FOR_MY_SS ) break;
				while( WAIT_FOR_SPIF );

				RxByte = SPDR; 							// copy received byte
				SPDR = 0xFF;   							// Continue dummy byte transmission
				data [ index++ ] = RxByte;
			}
			if( CHECK_FOR_MY_SS ) break;
			while( WAIT_FOR_SPIF );

			RxByte = SPDR; 								// copy last received byte
			SPDR = 0xA5;								// make the check byte available
			data [ index ] = RxByte;					// store the last data byte that was read
			if( CHECK_FOR_MY_SS ) break;
			while( WAIT_FOR_SPIF );

			RxByte = SPDR;								// store the check byte
			break;

		case Write :

			SPDR = data[ index++ ]; 					// Begin transmission
			while( index < pRAMFS_block->ram_size )
			{
				TxByte = data[ index++ ]; 				// pre-load the byte to be transmitted
				if( CHECK_FOR_MY_SS ) break;
				while( WAIT_FOR_SPIF );

				SPDR = TxByte; 							// Continue transmission
			}
			if( CHECK_FOR_MY_SS ) break;
			while( WAIT_FOR_SPIF );

			SPDR = 0xA5;								// make the check byte available
			if( CHECK_FOR_MY_SS ) break;
			while( WAIT_FOR_SPIF );

			RxByte = SPDR;								// store the check byte
			break;

		case Swap :

			SPDR = data[ index ]; 						// Begin first byte transfer
			while( index < pRAMFS_block->ram_size - 1)
			{
				TxByte = data[ index + 1 ]; 			// pre-load the next byte to be transmitted, while transferring
				if( CHECK_FOR_MY_SS ) break;
				while( WAIT_FOR_SPIF );

				RxByte = SPDR; 							// copy received byte
				SPDR = TxByte; 							// Continue transmission
				data [ index++ ] = RxByte;				// store the byte that was read, while transferring
			}
			if( CHECK_FOR_MY_SS ) break;
			while( WAIT_FOR_SPIF );

			RxByte = SPDR; 								// copy last received byte
			SPDR = 0xA5;								// make the check byte available
			data [ index ] = RxByte;					// store the last data byte that was read
			if( CHECK_FOR_MY_SS ) break;
			while( WAIT_FOR_SPIF );

			RxByte = SPDR;								// store the check byte
			break;

		case Test :
		case Huh :
		default :
			break;
	}

	spiSlaveEnd();										// clean up the SPI bus for other Clients.
	taskEXIT_CRITICAL();								// turn on interrupts

	if( RxByte == 0x5A) 								// compare if the returned check byte is as expected?
		return 0;										// return success!

	return 1;
}

/*-----------------------------------------------------------*/

size_t vRAMFSMalloc( size_t xWantedSize )
{
size_t vReturn = 0;


	/* Check there is enough room left for the allocation. */
	if( ( ( xNextFreeRAMFSByte + xWantedSize ) < configTOTAL_RAMFS_SIZE ) &&
		( ( xNextFreeRAMFSByte + xWantedSize ) > xNextFreeRAMFSByte )	)/* Check for overflow. */
	{
		/* Return the next free byte then increment the index past this
		block. */
		vReturn = (size_t) XRAMSTART + xNextFreeRAMFSByte;
		xNextFreeRAMFSByte += xWantedSize;
	}

	#if( configUSE_MALLOC_FAILED_HOOK == 1 )
	{
		if( pvReturn == NULL )
		{
			extern void vApplicationMallocFailedHook( void );
			vApplicationMallocFailedHook();
		}
	}
	#endif

	return vReturn;
}
/*-----------------------------------------------------------*/

void vRAMFSFree( size_t v )
{
	/* Memory cannot be freed using this scheme. */
	v = 0;

	/* Force an assert as it is invalid to call this function. */
	configASSERT( pv == NULL );
}
/*-----------------------------------------------------------*/

void vRAMFSInitialiseBlocks( void )
{
	/* Only required when static memory is not cleared. */
	xNextFreeRAMFSByte = ( size_t ) 0;
}
/*-----------------------------------------------------------*/

size_t xRAMFSGetFreeSize( void )
{
	return ( configTOTAL_RAMFS_SIZE - xNextFreeRAMFSByte );
}
/*-----------------------------------------------------------*/



/*----------------------------------------------------------------------*/
/* Public disk control functions for use by ff.c - do not use directly  */
/*----------------------------------------------------------------------*/

DSTATUS disk_status (uint8_t pdrv)
{
	uint16_t index;
	uint8_t TxRxByte;
	DSTATUS diskStatus;
	xRAMFSarray xRAMFS_block;			// this is just an array for XRAMFS info.
	uint8_t * pRAM;

	if (pdrv) return STA_NOINIT;		// Supports only single drive, drive 0.

	xRAMFS_block.ram_cmd  = Disk_Status;
	xRAMFS_block.ram_addr = (size_t) (XRAMSTART + xNextFreeRAMFSByte); // Set to a valid address, not used.
	xRAMFS_block.ram_size = (uint16_t) 1;	// Set to a valid size, not used.

	pRAM = (uint8_t *) &xRAMFS_block;	// make this cast to serialise the Command structure.

	ramfs_transaction_init();	// set up the SPI bus for the RAMFS transaction.
								// this is VERY time critical, so we do it in a MACRO to ensure there is no loss of time.
								// Note unpaired in this function: taskENTER_CRITICAL();

	index = 0;
	SPDR = pRAM[ index++ ]; 				// Begin transmission of the command structure (serialised)

	while( index < sizeof(xRAMFSarray) )
	{
		TxRxByte = pRAM[ index++ ]; 		// pre-load the byte to be transmitted
		if( CHECK_FOR_MY_SS )
		{
			spiSlaveEnd();
			taskEXIT_CRITICAL();
			return STA_NOINIT;
		}
		while( WAIT_FOR_SPIF );

		SPDR = TxRxByte; 					// Continue transmission of the command structure
	}

	if( CHECK_FOR_MY_SS )
	{
		spiSlaveEnd();
		taskEXIT_CRITICAL();
		return STA_NOINIT;
	}
	while( WAIT_FOR_SPIF );

	// now we have sent the command structure, the Supervisor will know what to do for us.

	SPDR  = 0xFF;							// prepare a dummy byte.

	if( CHECK_FOR_MY_SS )
	{
		spiSlaveEnd();
		taskEXIT_CRITICAL();
		return STA_NOINIT;
	}
	while( WAIT_FOR_SPIF );
	TxRxByte = SPDR; 						// copy received byte

	SPDR = 0xA5;							// make the check byte available
	diskStatus = TxRxByte;					// store the disk status

	if( CHECK_FOR_MY_SS )
	{
		spiSlaveEnd();
		taskEXIT_CRITICAL();
		return STA_NOINIT;
	}
	while( WAIT_FOR_SPIF );
	TxRxByte = SPDR;							// store the check byte

	spiSlaveEnd();								// clean up the SPI bus for other Clients.
	taskEXIT_CRITICAL();						// turn on interrupts

	if( TxRxByte == 0x5A) 						// compare if the returned check byte is as expected?
		return diskStatus;						// return success in the form of the diskStatus!

	return STA_NOINIT;
}


DSTATUS disk_initialize (uint8_t pdrv)
{
	uint16_t index;
	uint8_t TxRxByte;
	DSTATUS diskStatus;
	xRAMFSarray xRAMFS_block;			// this is just an array for XRAMFS info.
	uint8_t * pRAM;

	if (pdrv) return STA_NOINIT;		// Supports only single drive, drive 0.

	xRAMFS_block.ram_cmd  = Disk_Init;
	xRAMFS_block.ram_addr = (size_t) (XRAMSTART + xNextFreeRAMFSByte); // Set to a valid address, not used.
	xRAMFS_block.ram_size = (uint16_t) 1;	// Set to a valid size, not used.

	pRAM = (uint8_t *) &xRAMFS_block; 	// make this cast to serialise the Command structure.

	ramfs_transaction_init();	// set up the SPI bus for the RAMFS transaction.
								// this is VERY time critical, so we do it in a MACRO to ensure there is no loss of time.
								// Note unpaired in this function: taskENTER_CRITICAL();

	index = 0;
	SPDR = pRAM[ index++ ]; 					// Begin transmission of the command structure (serialised)

	while( index < sizeof(xRAMFSarray) )
	{
		TxRxByte = pRAM[ index++ ]; 			// pre-load the byte to be transmitted
		if( CHECK_FOR_MY_SS )
		{
			spiSlaveEnd();
			taskEXIT_CRITICAL();
			return STA_NOINIT;
		}
		while( WAIT_FOR_SPIF );

		SPDR = TxRxByte; 						// Continue transmission of the command structure
	}

	if( CHECK_FOR_MY_SS )
	{
		spiSlaveEnd();
		taskEXIT_CRITICAL();
		return STA_NOINIT;
	}
	while( WAIT_FOR_SPIF );

	// now we have sent the command structure, the Supervisor will know what to do with the data.

	SPDR  = 0xFF;								// prepare a dummy byte.
	if( CHECK_FOR_MY_SS )
	{
		spiSlaveEnd();
		taskEXIT_CRITICAL();
		return STA_NOINIT;
	}
	while( WAIT_FOR_SPIF );
	TxRxByte = SPDR; 							// copy received byte

	SPDR = 0xA5;								// make the check byte available
	diskStatus = TxRxByte;						// store the data byte that was read
	if( CHECK_FOR_MY_SS )
	{
		spiSlaveEnd();
		taskEXIT_CRITICAL();
		return STA_NOINIT;
	}
	while( WAIT_FOR_SPIF );

	TxRxByte = SPDR;							// store the check byte

	spiSlaveEnd();								// clean up the SPI bus for other Clients.
	taskEXIT_CRITICAL();						// turn on interrupts

	if( TxRxByte != 0x5A) 						// compare if the returned check byte is as expected?
		return STA_NOINIT;

	return diskStatus;							// return the result!
}


DRESULT disk_read (uint8_t pdrv, uint8_t* buff, uint32_t sector, uint8_t count)
{
	uint16_t index;
	uint8_t TxRxByte;
	xRAMFSarray xRAMFS_block;			// this is just an array for XRAMFS info.
	uint8_t * pRAM;

	if (pdrv || !count) return RES_PARERR;	// Supports only single drive, drive 0.

	xRAMFS_block.ram_cmd  = Disk_Read;
	xRAMFS_block.ram_addr = (size_t) (XRAMSTART + xNextFreeRAMFSByte); // Set to a valid address
	xRAMFS_block.ram_size = count * 512;	// Set to a valid size
	xRAMFS_block.disk_sector = sector;
	xRAMFS_block.disk_sector_count = count;

	pRAM = (uint8_t *) &xRAMFS_block;	// make this cast to serialise the Command structure.


	do {
		ramfs_transaction_init();	// set up the SPI bus for the RAMFS transaction.
									// this is VERY time critical, so we do it in a MACRO to ensure there is no loss of time.
									// Note unpaired in this function: taskENTER_CRITICAL();
		index = 0;
		SPDR = pRAM[ index++ ]; 				// Begin transmission of the command structure (serialised)

		while( index < sizeof(xRAMFSarray) )
		{
			TxRxByte = pRAM[ index++ ]; 		// pre-load the byte to be transmitted
			if( CHECK_FOR_MY_SS )
			{
				spiSlaveEnd();
				taskEXIT_CRITICAL();
				return RES_NOTRDY;
			}
			while( WAIT_FOR_SPIF );

			SPDR = TxRxByte; 					// Continue transmission of the command structure
		}

		if( CHECK_FOR_MY_SS )
		{
			spiSlaveEnd();
			taskEXIT_CRITICAL();
			return RES_NOTRDY;
		}
		while( WAIT_FOR_SPIF );

		index = 0;
		// now we have sent the command structure, the Supervisor will know what to do with the data.

		SPDR  = 0xFF;							// prepare a dummy byte.
		if( CHECK_FOR_MY_SS )
		{
			spiSlaveEnd();
			taskEXIT_CRITICAL();
			return RES_ERROR;
		}
		while( WAIT_FOR_SPIF );

		TxRxByte = SPDR; 						// copy received byte

		if (TxRxByte & STA_NOINIT)				// If the disk_status is not initialised, then return error.
		{
			spiSlaveEnd();
			taskEXIT_CRITICAL();
			return RES_ERROR;
		}

		SPDR = 0xFF;   							// Continue dummy byte transmission
		if( CHECK_FOR_MY_SS )
		{
			spiSlaveEnd();
			taskEXIT_CRITICAL();
			return RES_ERROR;
		}
		while( WAIT_FOR_SPIF );

		TxRxByte = SPDR; 						// copy last_command_result byte

		if (TxRxByte == RES_PENDING) break;		// we're on the second loop, and the data is waiting for us to get from XRAM.

		SPDR = 0xA5;							// make the check byte available
		if( CHECK_FOR_MY_SS )
		{
			spiSlaveEnd();
			taskEXIT_CRITICAL();
			return RES_ERROR;
		}
		while( WAIT_FOR_SPIF );

		TxRxByte = SPDR;						// store the check byte

		spiSlaveEnd();							// clean up the SPI bus for other Clients.
		taskEXIT_CRITICAL();					// turn on interrupts

		if( TxRxByte != 0x5A) 					// compare if the returned check byte is as expected we can proceed.
			return RES_ERROR;

	}while (TxRxByte != RES_PENDING);

	SPDR  = 0xFF;								// prepare a dummy byte.
	while( index < xRAMFS_block.ram_size - 1 )
	{
		if( CHECK_FOR_MY_SS )
		{
			spiSlaveEnd();
			taskEXIT_CRITICAL();
			return RES_ERROR;
		}
		while( WAIT_FOR_SPIF );

		TxRxByte = SPDR; 						// copy received byte
		SPDR = 0xFF;   							// Continue dummy byte transmission

		buff[ index++ ] = TxRxByte;
	}
	if( CHECK_FOR_MY_SS )
	{
		spiSlaveEnd();
		taskEXIT_CRITICAL();
		return RES_ERROR;
	}
	while( WAIT_FOR_SPIF );

	TxRxByte = SPDR; 							// copy last received byte
	SPDR = 0xA5;								// make the check byte available

	buff[ index ] = TxRxByte;					// store the last data byte that was read

	if( CHECK_FOR_MY_SS )
	{
		spiSlaveEnd();
		taskEXIT_CRITICAL();
		return RES_ERROR;
	}
	while( WAIT_FOR_SPIF );

	TxRxByte = SPDR;							// store the check byte

	spiSlaveEnd();								// clean up the SPI bus for other Clients.
	taskEXIT_CRITICAL();						// turn on interrupts

	if( TxRxByte == 0x5A) 						// compare if the returned check byte is as expected?
		return RES_OK;							// return success!

	return RES_ERROR;
}

DRESULT disk_write (uint8_t pdrv, const uint8_t* buff, uint32_t sector, uint8_t count)
{
	uint16_t index;
	uint8_t TxRxByte;
	xRAMFSarray xRAMFS_block;			// this is just an array for XRAMFS info.
	uint8_t * pRAM;

	if (pdrv || !count) return RES_PARERR;	// Supports only single drive, drive 0.

	xRAMFS_block.ram_cmd  = Disk_Write;
	xRAMFS_block.ram_addr = (size_t) (XRAMSTART + xNextFreeRAMFSByte); // Set to a valid address
	xRAMFS_block.ram_size = count * 512;	// Set to a valid size
	xRAMFS_block.disk_sector = sector;
	xRAMFS_block.disk_sector_count = count;

	pRAM = (uint8_t *) &xRAMFS_block;	// make this cast to serialise the Command structure.

	ramfs_transaction_init();	// set up the SPI bus for the RAMFS transaction.
								// this is VERY time critical, so we do it in a MACRO to ensure there is no loss of time.
								// Note unpaired in this function: taskENTER_CRITICAL();

	index = 0;
	SPDR = pRAM[ index++ ]; 							// Begin transmission of the command structure (serialised)

	while( index < sizeof(xRAMFSarray) )
	{
		TxRxByte = pRAM[ index++ ]; 						// pre-load the byte to be transmitted
		if( CHECK_FOR_MY_SS )
		{
			spiSlaveEnd();
			taskEXIT_CRITICAL();
			return RES_NOTRDY;
		}
		while( WAIT_FOR_SPIF );

		SPDR = TxRxByte; 									// Continue transmission of the command structure
	}

	if( CHECK_FOR_MY_SS )
	{
		spiSlaveEnd();
		taskEXIT_CRITICAL();
		return RES_NOTRDY;
	}
	while( WAIT_FOR_SPIF );

	index = 0;
	// now we have sent the command structure, the Supervisor will know what to do with the data.

	SPDR = buff[ index++ ]; 					// Begin transmission
	while( index < xRAMFS_block.ram_size )
	{
		TxRxByte = buff[ index++ ]; 				// pre-load the byte to be transmitted
		if( CHECK_FOR_MY_SS ) return RES_ERROR;
		while( WAIT_FOR_SPIF );

		SPDR = TxRxByte; 							// Continue transmission
	}
	if( CHECK_FOR_MY_SS ) return RES_ERROR;
	while( WAIT_FOR_SPIF );

	SPDR = 0xA5;								// make the check byte available
	if( CHECK_FOR_MY_SS ) return RES_ERROR;
	while( WAIT_FOR_SPIF );

	TxRxByte = SPDR;								// store the check byte

	spiSlaveEnd();										// clean up the SPI bus for other Clients.
	taskEXIT_CRITICAL();								// turn on interrupts

	if( TxRxByte == 0x5A) 								// compare if the returned check byte is as expected?
		return RES_OK;									// return success!

	return RES_ERROR;
}

DRESULT disk_ioctl (uint8_t pdrv, uint8_t cmd, void* buff)
{
	uint16_t index;
	uint8_t TxRxByte;
	xRAMFSarray xRAMFS_block;			// this is just an array for XRAMFS info.
	uint8_t* pRAM;

	if (pdrv) return RES_PARERR;		// Supports only single drive, drive 0.

	xRAMFS_block.ram_cmd  = Disk_IOCtl;
	xRAMFS_block.ram_addr = (size_t) (XRAMSTART + xNextFreeRAMFSByte); // Set to a valid address
	xRAMFS_block.disk_sector_count = cmd;		// note REUSE of the disk_sector_count for carrying the specific IOCtl command.

	switch (cmd) {				/* Set the response size based on the type of cmd we're executing */

	case CTRL_POWER :			/* No Response Needed, Supervisor looks after the SD card. */
	case CTRL_ERASE_SECTOR :	/* No Response Needed, don't bother implement this. It is slow and unnecessary. */

		return RES_OK;
		break;

	case CTRL_SYNC :			/* No Response provided, so we're just going to set this to small as possible */

		xRAMFS_block.ram_size = sizeof(uint8_t);	// Set to a valid size of uint8_t
		break;

	case GET_SECTOR_SIZE :		/* Get R/W sector size (uint16_t) */

		xRAMFS_block.ram_size = sizeof(uint16_t);	// Set to a valid size of uint16_t
		break;

	case GET_SECTOR_COUNT :		/* Get number of sectors on the disk (uint32_t) */
	case GET_BLOCK_SIZE :		/* Get erase block size in unit of sector (uint32_t) */

		xRAMFS_block.ram_size = sizeof(uint32_t);	// Set to a valid size of uint32_t
		break;

	case MMC_GET_TYPE :			/* Get card type flags (1 byte) */

		xRAMFS_block.ram_size = sizeof(uint8_t);	// Set to a valid size of uint8_t
		break;

	case MMC_GET_CSD :			/* Receive CSD as a data block (16 bytes) */
	case MMC_GET_CID :			/* Receive CID as a data block (16 bytes) */

		xRAMFS_block.ram_size = 16 * sizeof(uint8_t);	// Set to a valid size of 16 bytes
		break;

	case MMC_GET_OCR :			/* Receive OCR as an R3 response (4 bytes) */

		xRAMFS_block.ram_size = 4 * sizeof(uint8_t);	// Set to a valid size of 4 bytes
		break;

	case MMC_GET_SDSTAT :		/* Receive SD status as a data block (64 bytes) */

		xRAMFS_block.ram_size = 64 * sizeof(uint8_t);	// Set to a valid size of 64 bytes
		break;

	default:
		return RES_PARERR;
		break;
	}

	pRAM = (uint8_t *) &xRAMFS_block;			// make this cast to serialise the Command structure.

	do {
		ramfs_transaction_init();	// set up the SPI bus for the RAMFS transaction.
									// this is VERY time critical, so we do it in a MACRO to ensure there is no loss of time.
									// Note unpaired in this function: taskENTER_CRITICAL();
		index = 0;
		SPDR = pRAM[ index++ ]; 				// Begin transmission of the command structure (serialised)

		while( index < sizeof(xRAMFSarray) )
		{
			TxRxByte = pRAM[ index++ ]; 		// pre-load the byte to be transmitted
			if( CHECK_FOR_MY_SS )
			{
				spiSlaveEnd();
				taskEXIT_CRITICAL();
				return RES_NOTRDY;
			}
			while( WAIT_FOR_SPIF );

			SPDR = TxRxByte; 					// Continue transmission of the command structure
		}

		if( CHECK_FOR_MY_SS )
		{
			spiSlaveEnd();
			taskEXIT_CRITICAL();
			return RES_NOTRDY;
		}
		while( WAIT_FOR_SPIF );

		index = 0;
		// now we have sent the command structure, the Supervisor will know what to do with the data.

		SPDR  = 0xFF;							// prepare a dummy byte.
		if( CHECK_FOR_MY_SS )
		{
			spiSlaveEnd();
			taskEXIT_CRITICAL();
			return RES_NOTRDY;
		}
		while( WAIT_FOR_SPIF );

		TxRxByte = SPDR; 						// copy received byte

		if (TxRxByte & STA_NOINIT)				// If the disk_status is not initialised, then return error.
		{
			spiSlaveEnd();
			taskEXIT_CRITICAL();
			return RES_NOTRDY;
		}

		SPDR = 0xFF;   							// Continue dummy byte transmission
		if( CHECK_FOR_MY_SS )
		{
			spiSlaveEnd();
			taskEXIT_CRITICAL();
			return RES_NOTRDY;
		}
		while( WAIT_FOR_SPIF );

		TxRxByte = SPDR; 						// copy last_command_result byte

		if (TxRxByte == RES_PENDING) break;		// we're on the second loop, and the data is waiting for us to get from XRAM.

		SPDR = 0xA5;							// make the check byte available
		if( CHECK_FOR_MY_SS )
		{
			spiSlaveEnd();
			taskEXIT_CRITICAL();
			return RES_NOTRDY;
		}
		while( WAIT_FOR_SPIF );

		TxRxByte = SPDR;						// store the check byte

		spiSlaveEnd();							// clean up the SPI bus for other Clients.
		taskEXIT_CRITICAL();					// turn on interrupts

		if( TxRxByte != 0x5A) 					// compare if the returned check byte is as expected we can proceed.
			return RES_NOTRDY;


	}while (TxRxByte != RES_PENDING);

	SPDR  = 0xFF;								// prepare a dummy byte.
	while( index < xRAMFS_block.ram_size - 1 )
	{
		if( CHECK_FOR_MY_SS )
		{
			spiSlaveEnd();
			taskEXIT_CRITICAL();
			return RES_NOTRDY;
		}
		while( WAIT_FOR_SPIF );

		TxRxByte = SPDR; 						// copy received byte
		SPDR = 0xFF;   							// Continue dummy byte transmission

		((uint8_t *)buff)[ index++ ] = TxRxByte;
	}
	if( CHECK_FOR_MY_SS )
	{
		spiSlaveEnd();
		taskEXIT_CRITICAL();
		return RES_NOTRDY;
	}
	while( WAIT_FOR_SPIF );

	TxRxByte = SPDR; 							// copy last received byte
	SPDR = 0xA5;								// make the check byte available

	((uint8_t *)buff)[ index ] = TxRxByte;		// store the last data byte that was read

	if( CHECK_FOR_MY_SS )
	{
		spiSlaveEnd();
		taskEXIT_CRITICAL();
		return RES_NOTRDY;
	}
	while( WAIT_FOR_SPIF );

	TxRxByte = SPDR;							// store the check byte

	spiSlaveEnd();								// clean up the SPI bus for other Clients.
	taskEXIT_CRITICAL();						// turn on interrupts

	if( TxRxByte == 0x5A) 						// compare if the returned check byte is as expected?
		return RES_OK;							// return success!

	return RES_ERROR;
}

/*-----------------------------------------------------------*/

// */
#endif



// ********** Supervisor Interrupt Handlers ********** //

#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
/****************************************************************************
This function is the Interrupt Service Routine (ISR), and called when a SS pin change interrupt is triggered;
indicating that a client device wants to be serviced.
This function should not be called directly from the main application.
****************************************************************************/

#if defined (ARDUSAT_HARDWARE) // Most of the PCINT1 pins are unavailable on the Mega2650

ISR (PCINT1_vect)
{
    uint16_t changedbits;
	signed portBASE_TYPE xHigherPriorityTaskWoken;

    changedbits = (uint16_t)(( PINJ<<1 | (PINE & 0x01) ) ^ HIGH_BITS);	// Check if any of the SS lines from Clients have been pulled low.

    if (changedbits != 0x0000)
		// Push changedbits onto the queue for later processing.
		// Don't wait if queue full.
		xQueueSendToBackFromISR( xRAMFSCallQueue, &changedbits, &xHigherPriorityTaskWoken );

	if( xHigherPriorityTaskWoken )
		taskYIELD ();
}
#endif

ISR (PCINT2_vect)
{
    uint16_t changedbits;
	signed portBASE_TYPE xHigherPriorityTaskWoken;

	// calculate which bit changed since last time
    changedbits = ((uint16_t)(PINK ^ HIGH_BITS)) << 8;	// Check if any of the SS lines from Clients have been pulled low.

    if (changedbits != 0x0000)
		// Push changedbits onto the queue for later processing.
		// Don't wait if queue full.
		xQueueSendToBackFromISR( xRAMFSCallQueue, &changedbits, &xHigherPriorityTaskWoken );

	if( xHigherPriorityTaskWoken )
		taskYIELD ();
}

#endif

#endif
