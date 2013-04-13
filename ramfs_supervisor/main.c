////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////    main.c
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include <avr/io.h>
#include <util/delay.h>			// Needed for _delay_us()

/* Scheduler include files. */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include <ext_ram.h>
#include <spi.h>
#include <lib_crc.h>

/* RAMFS RAM file system interface include file. */
#include <ramfs.h>

#include <diskio.h>

/*--------------------Global Variables---------------------------*/

// xComPortHandle xSerialPort;				// Create a handle for the serial port.

/*--------------Tasks ------------------------------*/

static void TaskBlinkRedLED(void *pvParameters);	// Main Arduino Mega 2560, Freetronics EtherMega (Red) LED Blink

static void TaskRAMFSManager(void *pvParameters);	// RAMFS Manager.

/*-----------------------------------------------------------*/

/* Main program loop */
int16_t main(void) __attribute__((OS_main));

int16_t main(void)
{

    // turn on the serial port for debugging or for other USART reasons.
//	xSerialPort = xSerialPortInitMinimal( 115200, portSERIAL_BUFFER, portSERIAL_BUFFER); //  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)

//	avrSerialPrint_P(PSTR("\r\n\n\nHello World!\r\n")); // Ok, so we're alive...

    xTaskCreate(
		TaskBlinkRedLED
		,  (const signed portCHAR *)"RedLED" // Main Arduino Mega 2560, Freetronics EtherMega (Red) LED Blink
		,  132
		,  NULL
		,  1
		,  NULL ); // */


    xTaskCreate(
    	TaskRAMFSManager
 		,  (const signed portCHAR *)"RAMFS" // RAMFS Manager
 		,  256
 		,  NULL
 		,  3
 		,  NULL ); // */

//	avrSerialPrintf_P(PSTR("Free Heap Size: %u\r\n"),xPortGetFreeHeapSize() ); // needs heap_1 or heap_2 for this function to succeed.

	vTaskStartScheduler();

//	avrSerialPrint_P(PSTR("\r\n\n\nGoodbye... no space for idle task!\r\n")); // Doh, so we're dead...

}

/*-----------------------------------------------------------*/


static void TaskBlinkRedLED(void *pvParameters) // Main Red LED Flash
{
    (void) pvParameters;

    portTickType xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

	DDRB |= _BV(DDB7);

    while(1)
    {

    	PORTB |=  _BV(PORTB7);       // main (red IO_B7) LED on. EtherMega LED on
		vTaskDelayUntil( &xLastWakeTime, ( 100 / portTICK_RATE_MS ) );

		PORTB &= ~_BV(PORTB7);       // main (red IO_B7) LED off. EtherMega LED off
		vTaskDelayUntil( &xLastWakeTime, ( 900 / portTICK_RATE_MS ) );

//		xSerialPrintf_P(PSTR("RedLED HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
    }

}


/*---------------------------------------------------------------------------*/

static void TaskRAMFSManager(void *pvParameters) // RAMFS Manager
{
    (void) pvParameters;

//    portTickType xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
//	xLastWakeTime = xTaskGetTickCount();

	xRAMFSarray activeRAMFSblock;	// Establish a CMD block so we know where to put the RAM we read in.
	uint8_t * pActiveRAMFSblock;	// and a pointer to it.

	pActiveRAMFSblock = (uint8_t *) &activeRAMFSblock; // make this cast to serialise the command structure.

	DRESULT disk_last_command_result[CLIENTS];		// hold the result of the last disk command here, for client to query.

	// create a queue for the PCINT pin results to be pushed onto by the Interrupt routines. uint16_t covers 16 clients.
	xRAMFSCallQueue = xQueueCreate( RAMFSCALLQUEUEDEPTH, sizeof( uint16_t) );  // queue for calls on RAMFS

	extRAMInitHeap(false);			// use heapInXmem_ false to ignore the heap state for portEXT_RAMFS usage.
	setMemoryBank(0, false);		// use switchHeap_ false to ignore the heap for portEXT_RAMFS usage.

	spiSetClockDivider(SPI_CLOCK_DIV8); // hopefully we can go faster than DIV8, later. But for now, it is robust.
	spiBegin(SS_PB0);				// standard SS for Arduino Mega.

	spiSelect (SS_PB0);				// Select that we're using the SPI bus (in case there are multiple SPI tasks)

	DDRB |= _BV(DDB6);				// Set a led between PORTB6 (Pin 12) and GND. So we have a visual idea of transactions.

	init16PCINTpins();				// set up PCINT pins, to allow Clients to signal their requests.

	while(1)
    {
    	uint8_t		arduinoBank;
    	uint16_t	ISRrequest;

    	arduinoBank = 0x00;
    	ISRrequest = 0x0000;

		PORTB &= ~_BV(PORTB6);       // activity (IO_B6) LED off.

		if( xQueueReceive( xRAMFSCallQueue, &ISRrequest, 1000 / portTICK_RATE_MS) == pdTRUE )// block, until there is a request on the queue to grab.
		{
			uint8_t i;
			uint8_t pin;

	    	PORTB |=  _BV(PORTB6);       // activity (IO_B6) LED on.

			i = 0;
			while( (ISRrequest >> i++ & 0x0001) == 0x0000) // shift through the RAM banks (one each Arduino) until we find a request match.
			{
				arduinoBank = i;
			}

//			xSerialPrintf_P(PSTR("Interrupt: %4x, Bank: %2u"), ISRrequest, arduinoBank);	// may see multiple signals here on Interrupts.
																							// But just the lowest Arduino bank request will be found.

			if( (ISRrequest ^ 0x0001<<arduinoBank) != 0x0000) // check if there are any more requests, that didn't get their own interrupt.
			{
				ISRrequest ^= 0x0001<<arduinoBank;
				xQueueSendToFront( xRAMFSCallQueue, &ISRrequest, ( portTickType ) 0 );	// push remainder of this request back onto the front of the queue,
																						// but don't wait about.
																						// This catches simultaneous Client requests.
			}

			setMemoryBank(arduinoBank, false);		// set the RAMFS bank for the Arduino for usage.

            // here goes the SPI bus stuff...

			// First job is to work out which is the SS pin we need to use.
			// But this is just the pin that the interrupt came in on, so it is easy.

			// Then disable PCINT for the relevant Client SS line (pin) while SPI bus is in use.
			// Don't want interrupts triggering because we're driving it.

			// then we set the relevant Client SS line output and LOW to select the Arduino SPI interface.

            if( (0x0001<<arduinoBank) && 0x00FF != 0x0000)
            {
            	pin = (uint8_t)(((0x0001<<arduinoBank) & 0xFF00)>>8); // use the upper 8 bits for Port K.

//            	xSerialPrintf_P(PSTR(", K Pin: %2x\r\n"), pin);

				PCMSK2 &= ~pin;			// set the Interrupt mask to exclude current SS pin. PCINT23 - PCINT16

            	DDRK = pin;				// one pin set as output (SS). All the rest as input.
            	PORTK = ~pin;				// output pin driven low. All other input pins pulled high,
            }

#if defined (ARDUSAT_HARDWARE) // Most of the PCINT1 pins are unavailable on the Mega2650.
            else
            {
            	pin = (uint8_t)((0x0001<<arduinoBank) & 0x00FF); // use the lower 8 bits for Port J and Port E

//            	xSerialPrintf_P(PSTR(", J Pin: %2x\r\n"), pin);

    			PCMSK1 &= ~(pin);				// set the Interrupt mask to exclude current SS pin. PCINT15 - PCINT8

            	DDRJ &= pin>>1 | _BV(DDJ7);		// one pin set as output (SS). the rest as input.
            	PORTJ &= ~(pin>>1) | _BV(PJ7);	// output pin driven low. All other input pins pulled high. Except PJ7. We might be using it elsewhere.
            	DDRE &= pin & _BV(DDE0);		// and same on PE0.
            	PORTE &= ~(pin & _BV(PE0));
            }
#endif

            _delay_us(25);						// put in a delay here. It seems the Slave can't be ready in time.
            									// Sometimes it takes 16us to respond to SS line and begin enabling SPI

            i = 0;
            while( (spiTransfer(0xA5) != 0x5A) )// repeatedly send a dummy byte until we get the Arduino client SPI Slave to respond correctly
            {
            	if (!( ++i) ) break;			// if we don't hear back from SPI slave, then bail out of this loop.
            }									// this means it has enabled SPI slave and responded to the PCINT interrupt.

            // Get the command structure, so we know what we're going to be doing.
            if( !spiMultiByteRx( pActiveRAMFSblock, (uint16_t) sizeof(xRAMFSarray) ) ) break; // command structure

            if( (activeRAMFSblock.ram_addr < (size_t)XRAMSTART) || (activeRAMFSblock.ram_size > (size_t)XRAMEND - activeRAMFSblock.ram_addr) ) // check we're not being fed a phony address, or size
            	activeRAMFSblock.ram_cmd = Huh;

            switch (activeRAMFSblock.ram_cmd)
            {

				case Read : // read from RAMFS - write to SPI bus
					if( !spiMultiByteTx( (uint8_t *)(activeRAMFSblock.ram_addr), activeRAMFSblock.ram_size )) break;
		            spiTransfer(0x5A);	// give back the check byte so the Arduino Client SPI Slave knows we finished OK.
		        	init16PCINTpins();				// reset PCINT pins, to allow Clients to signal their requests.
					break;

				case Write : // write to RAMFS - read on SPI bus
					if( !spiMultiByteRx( (uint8_t *)(activeRAMFSblock.ram_addr), activeRAMFSblock.ram_size )) break;
		            spiTransfer(0x5A);	// give back the check byte so the Arduino Client SPI Slave knows we finished OK.
		        	init16PCINTpins();				// reset PCINT pins, to allow Clients to signal their requests.
					break;

				case Swap :	// swap the contents of RAMFS - bidirectional transfer "FASTEST THROUGHPUT" (simultaneous Read & Write)
					if( !spiMultiByteTransfer( (uint8_t *)(activeRAMFSblock.ram_addr), activeRAMFSblock.ram_size )) break;
		            spiTransfer(0x5A);	// give back the check byte so the Arduino Client SPI Slave knows we finished OK.
		        	init16PCINTpins();				// reset PCINT pins, to allow Clients to signal their requests.
					break;


				case Disk_Status : // get remote disk status (from RAM)
					spiTransfer( (uint8_t)disk_status(0) ); 						// transfer the disk status
		            spiTransfer(0x5A);	// give back the check byte so the Arduino Client SPI Slave knows we finished OK.
		        	init16PCINTpins();				// reset PCINT pins, to allow Clients to signal their requests.
					break;

				case Disk_Init : // initialise the remote disk
					if ( disk_status(0) && (STA_NOINIT | STA_NODISK))
					{
						// initialise the disk if it needs to be so.
						spiTransfer( (uint8_t)disk_status(0) );						// transfer the disk status
			            spiTransfer(0x5A);	// give back the check byte so the Arduino Client SPI Slave knows we finished OK.
			        	init16PCINTpins();				// reset PCINT pins, to allow Clients to signal their requests.

			            // Start the Disk operations.
			            spiDeselect (SS_PB0);				// Deselect the SPI bus (to make sure we can take the semaphore in disk_initialise)

			            disk_last_command_result[arduinoBank] = disk_initialize((uint8_t) 0);

			        	spiSetClockDivider(SPI_CLOCK_DIV8); // hopefully we can go faster than DIV8, later. But for now, it is robust.
			        	spiSelect (SS_PB0);					// Select that we're using the SPI bus (in case there are multiple SPI tasks)

					}else{
						// otherwise just report its condition.
						spiTransfer( (uint8_t)disk_status(0) );						// transfer the disk status
			            spiTransfer(0x5A);	// give back the check byte so the Arduino Client SPI Slave knows we finished OK.
			        	init16PCINTpins();				// reset PCINT pins, to allow Clients to signal their requests.
					}
					break;

				case Disk_Read : // read from the disk
					spiTransfer( (uint8_t)disk_status(0) ); 						// transfer the disk status
					spiTransfer( (uint8_t)disk_last_command_result[arduinoBank]);	// send the status so that the Read can be properly handled

					if( disk_last_command_result[arduinoBank] == RES_PENDING )
					{
						if( !spiMultiByteTx( (uint8_t *)(activeRAMFSblock.ram_addr), activeRAMFSblock.ram_size )) break;
			            spiTransfer(0x5A);	// give back the check byte so the Arduino Client SPI Slave knows we finished OK.
			        	init16PCINTpins();				// reset PCINT pins, to allow Clients to signal their requests.

			            disk_last_command_result[arduinoBank] = RES_OK;
					}
					else
					{

			            spiTransfer(0x5A);	// give back the check byte so the Arduino Client SPI Slave knows we finished OK.
			        	init16PCINTpins();				// reset PCINT pins, to allow Clients to signal their requests.

			            // Start the Disk operations.
			            spiDeselect (SS_PB0);				// Deselect the SPI bus (to make sure we can take the semaphore in disk_read)
			        	spiSetClockDivider(SPI_CLOCK_DIV2); // SD Card can go at full speed.

			            disk_last_command_result[arduinoBank] = disk_read( (uint8_t) 0, (uint8_t *)(activeRAMFSblock.ram_addr), activeRAMFSblock.disk_sector , activeRAMFSblock.disk_sector_count );

			        	spiSetClockDivider(SPI_CLOCK_DIV8); // hopefully we can go faster than DIV8, later. But for now, it is robust.
			        	spiSelect (SS_PB0);					// Select that we're using the SPI bus (in case there are multiple SPI tasks)

			        	if (disk_last_command_result[arduinoBank] == RES_OK)
			        		disk_last_command_result[arduinoBank] = RES_PENDING;
					}

					break;

				case Disk_Write : // write to the disk
					if( !spiMultiByteRx( (uint8_t *)(activeRAMFSblock.ram_addr), activeRAMFSblock.ram_size )) break;
		            spiTransfer(0x5A);	// give back the check byte so the Arduino Client SPI Slave knows we finished OK.
		        	init16PCINTpins();				// reset PCINT pins, to allow Clients to signal their requests.

		            // Start the Disk operations.
		            spiDeselect (SS_PB0);				// Deselect the SPI bus (to make sure we can take the semaphore in disk_write)
		        	spiSetClockDivider(SPI_CLOCK_DIV2); // SD Card can go at full speed.

		            disk_last_command_result[arduinoBank] = disk_write( (uint8_t) 0, (const uint8_t *)(activeRAMFSblock.ram_addr), activeRAMFSblock.disk_sector , activeRAMFSblock.disk_sector_count );

		        	spiSetClockDivider(SPI_CLOCK_DIV8); // hopefully we can go faster than DIV8, later. But for now, it is robust.
		        	spiSelect (SS_PB0);					// Select that we're using the SPI bus (in case there are multiple SPI tasks)

					break;

				case Disk_IOCtl : // do some IO control on the disk.
					spiTransfer( (uint8_t)disk_status(0) ); 						// transfer the disk status
					spiTransfer( (uint8_t)disk_last_command_result[arduinoBank]);	// send the status so that the IOCtl can be properly handled

					if( disk_last_command_result[arduinoBank] == RES_PENDING )
					{
						if( !spiMultiByteTx( (uint8_t *)(activeRAMFSblock.ram_addr), activeRAMFSblock.ram_size )) break;
			            spiTransfer(0x5A);	// give back the check byte so the Arduino Client SPI Slave knows we finished OK.
			        	init16PCINTpins();				// reset PCINT pins, to allow Clients to signal their requests.

			            disk_last_command_result[arduinoBank] = RES_OK;
					}
					else
					{

			            spiTransfer(0x5A);	// give back the check byte so the Arduino Client SPI Slave knows we finished OK.
			        	init16PCINTpins();				// reset PCINT pins, to allow Clients to signal their requests.

			            // Start the Disk operations.
			            spiDeselect (SS_PB0);				// Deselect the SPI bus (to make sure we can take the semaphore in disk_ioctl)
			        	spiSetClockDivider(SPI_CLOCK_DIV2); // SD Card can go at full speed.

			            disk_last_command_result[arduinoBank] = disk_ioctl( (uint8_t) 0, activeRAMFSblock.disk_sector_count, (uint8_t *)(activeRAMFSblock.ram_addr) );
			            									// using activeRAMFSblock.disk_sector_count for the IOCtl cmd just to make things tricky.

			        	spiSetClockDivider(SPI_CLOCK_DIV8); // hopefully we can go faster than DIV8, later. But for now, it is robust.
			        	spiSelect (SS_PB0);					// Select that we're using the SPI bus (in case there are multiple SPI tasks)

			        	if (disk_last_command_result[arduinoBank] == RES_OK)
			        		disk_last_command_result[arduinoBank] = RES_PENDING;
					}
					break;

				case Test :
		            spiTransfer(0x5A);	// give back the check byte so the Arduino Client SPI Slave knows we're OK.
		        	init16PCINTpins();				// reset PCINT pins, to allow Clients to signal their requests.
					break;

				case Huh :	// just break out of here, if the Client doesn't really want us to talk to it.
		        	init16PCINTpins();				// reset PCINT pins, to allow Clients to signal their requests.
					break;

				default :
		        	init16PCINTpins();				// reset PCINT pins, to allow Clients to signal their requests.
					break;
			}
		}
//		xSerialPrintf_P(PSTR("RAMFS HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
//		vTaskDelayUntil( &xLastWakeTime, ( 50 / portTICK_RATE_MS ) );
    }
}



/*-----------------------------------------------------------*/
/* Additional helper functions */
/*-----------------------------------------------------------*/


void vApplicationStackOverflowHook( xTaskHandle xTask,
                                    signed portCHAR *pcTaskName )
{

	DDRB  |= _BV(DDB7);
	PORTB |= _BV(PORTB7);       // main (red PB7) LED on. Mega main LED on and die.
	while(1);
}

/*-----------------------------------------------------------*/

