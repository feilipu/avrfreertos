////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////    main.c
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include <avr/io.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "lib_crc.h"

/* serial interface include file. */
#include "serial.h"

/* extended string to integer */
#include "xatoi.h"

/* FatF interface include file. */
#include "ff.h"

/* RAMFS RAM file system interface include file. */
#include "ramfs.h"


/*--------------------  Definitions  ---------------------------*/

#define LINE_SIZE 			16			// size of Client command line (on heap)

/*-------------------- Global Variables ------------------------*/

extern xComPortHandle xSerialPort;				// Create a handle for the serial port.

uint8_t * LineBuffer;					// put line buffer on heap (with pvPortMalloc).

/*-----------------  Private Functions  ----------------------*/

static void get_line (uint8_t *buff, uint8_t len);

/*--------------------   Tasks   ------------------------------*/

// DO NOT FLASH THE LED on PB5. It hangs off the SPI bus and kills things for other clients. etc.

static void TaskMonitor(void *pvParameters);		// Serial monitor for XRAMFS Testing

/*-----------------------------------------------------------*/

/* Main program loop */
int main(void) __attribute__((OS_main));

int main(void)
{
    // turn on the serial port for debugging or for other USART reasons.
	xSerialPort = xSerialPortInitMinimal( USART0, 115200, portSERIAL_BUFFER, LINE_SIZE); //  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)

	avrSerialPrint_P(PSTR("\r\n\n\nHello World!\r\n")); // Ok, so we're alive...

    xTaskCreate(
 		TaskMonitor
 		,  (const char *)"Monitor" // Serial Monitor
 		,  208
 		,  NULL
 		,  3
 		,  NULL ); // */

	avrSerialPrintf_P(PSTR("\r\n\nFree Heap Size: %u\r\n"),xPortGetFreeHeapSize() ); // needs heap_1 or heap_2 or heap_4 for this function to succeed.

	vTaskStartScheduler();

	avrSerialPrint_P(PSTR("\r\n\n\nGoodbye... no space for idle task!\r\n")); // Doh, so we're dead...

}


/*---------------------------------------------------------------------------*/

static void TaskMonitor(void *pvParameters) // Monitor for Serial Interface
{
    (void) pvParameters;

	uint8_t *ptr;
	int32_t p1;

	// create the working buffers on the heap (so they can be moved later).

	if(LineBuffer == NULL) // if there is no Line buffer allocated (pointer is NULL), then allocate buffer.
		if( !(LineBuffer = (uint8_t *) pvPortMalloc( sizeof(uint8_t) * LINE_SIZE )))
			xSerialPrint_P(PSTR("pvPortMalloc for *LineBuffer fail..!\r\n"));

	xRAMFSarray testRAMFS;			// this is just a  test array for XRAMFS info.

	uint8_t RAMbyte;				// initial fill value

	uint8_t * pLocalRAM = NULL;		// pointer to local ram.

	xSerialPrint_P(PSTR("\r\nXRAMFS FatFs test monitor"));

	while(1)
    {
    	xSerialPutChar(&xSerialPort, '>');

		ptr = LineBuffer;
		get_line(ptr, (uint8_t)(sizeof(uint8_t)* LINE_SIZE));  //sizeof (Line)

		switch (*ptr++) {

		case 'c' :	// Create XRAMFS structure. Allocate and fill local RAM with
					// > c ram_size [fill_bytes]   allocate, fill and calculate CRC8
					// > c                         just calculate a CRC8

			if (xatoi(&ptr, &p1)) {
				testRAMFS.ram_size = (uint16_t)p1;

				// "create" the XRAMFS information.
				if( !(testRAMFS.ram_addr = (uint16_t ) vRAMFSMalloc( sizeof(uint8_t) * testRAMFS.ram_size )))
				{
					xSerialPrint_P(PSTR("vRAMFSMalloc for testRAMFS fail..!\r\n"));
					break;
				}

				// create the local RAM on the heap.
				if( !(pLocalRAM = (uint8_t *) pvPortMalloc( sizeof(uint8_t) * testRAMFS.ram_size )))
				{
					xSerialPrint_P(PSTR("pvPortMalloc for *pLocalRAM fail..!\r\n"));
					break;
				}

				if (xatoi(&ptr, &p1))
				{
					RAMbyte = (uint8_t)p1;
					for (uint16_t i = 0; i < testRAMFS.ram_size; i++)
						pLocalRAM[i] = RAMbyte + i;
				} else {
					for (uint16_t i = 0; i < testRAMFS.ram_size; i++)
						pLocalRAM[i] = (uint8_t)rand();
				}

				// calculate a CRC on the RAM.
				testRAMFS.ram_crc8 = crc8( pLocalRAM, ( sizeof(uint8_t) * testRAMFS.ram_size ) );

				xSerialPrintf_P(PSTR("Addr: %4x Size: %u crc: %2x\r\n"), testRAMFS.ram_addr, testRAMFS.ram_size, testRAMFS.ram_crc8 );
				xSerialPrintf_P(PSTR("xRAMFSGetFreeSize: %u\r\n"),xRAMFSGetFreeSize());
				xSerialPrintf_P(PSTR("Free Heap Size:    %u\r\n"),xPortGetFreeHeapSize() ); // needs heap_1 or heap_2 or heap_4 for this function to succeed.

			} else {

				// calculate a CRC on the RAM (to test).
				testRAMFS.ram_crc8 = crc8( pLocalRAM, ( sizeof(uint8_t) * testRAMFS.ram_size ) );

				xSerialPrintf_P(PSTR("Addr: %4x Size: %u crc: %2x\r\n"), testRAMFS.ram_addr, testRAMFS.ram_size, testRAMFS.ram_crc8 );
				xSerialPrintf_P(PSTR("xRAMFSGetFreeSize: %u\r\n"),xRAMFSGetFreeSize());
				xSerialPrintf_P(PSTR("Free Heap Size:    %u\r\n"),xPortGetFreeHeapSize() ); // needs heap_1 or heap_2 or heap_4 for this function to succeed.
			}
			break;

		case 'p' : // Print the RAM contents.

			if(pLocalRAM != NULL)
				for (uint16_t i = 0; i < testRAMFS.ram_size; i+=8)
				{
					xSerialPrintf_P(PSTR("%4x: %2x %2x %2x %2x %2x %2x %2x %2x\r\n"), testRAMFS.ram_addr + i, pLocalRAM[i],pLocalRAM[i+1],pLocalRAM[i+2],pLocalRAM[i+3],pLocalRAM[i+4],pLocalRAM[i+5],pLocalRAM[i+6],pLocalRAM[i+7]);
					vTaskDelay(  10 / portTICK_PERIOD_MS );
				}
			xSerialPrint_P(PSTR("\r\n"));
			break;


		case 'r' : // Read the XRAMFS contents into RAM.

			// Set the Command for Read (from XRAMFS)
			testRAMFS.ram_cmd = Read;

			if(ramfs_transfer_block( &testRAMFS, pLocalRAM ))
				xSerialPrint_P(PSTR("Read XRAMFS: Fail\r\n"));
			else
				xSerialPrint_P(PSTR("Read XRAMFS: Success\r\n"));
			break;


		case 'w' : // Write the RAM contents into XRAMFS.

			// Set the Command for Write (to XRAMFS)
			testRAMFS.ram_cmd = Write;

			if(ramfs_transfer_block( &testRAMFS, pLocalRAM ))
				xSerialPrint_P(PSTR("Write XRAMFS: Fail\r\n"));
			else
				xSerialPrint_P(PSTR("Write XRAMFS: Success\r\n"));
			break;


		case 's' : // Swap the RAM contents with the XRAMFS contents.

			// Set the Command for Write (to XRAMFS)
			testRAMFS.ram_cmd = Swap;

			if(ramfs_transfer_block( &testRAMFS, pLocalRAM ))
				xSerialPrint_P(PSTR("Swap XRAMFS: Fail\r\n"));
			else
				xSerialPrint_P(PSTR("Swap XRAMFS: Success\r\n"));
			break;


		case 'x' : // Write, Read, randomise, Swap, randomise, and repeat checking for errors.

			// Set the Command for Write (to XRAMFS)
			testRAMFS.ram_cmd = Write;

			if(ramfs_transfer_block( &testRAMFS, pLocalRAM ))
				xSerialPrint_P(PSTR("Write XRAMFS: Fail\r\n"));
			else
				xSerialPrint_P(PSTR("Write XRAMFS: Success\r\n"));

			// Set the Command for Read (from XRAMFS)
			testRAMFS.ram_cmd = Read;

			if(ramfs_transfer_block( &testRAMFS, pLocalRAM ))
				xSerialPrint_P(PSTR("Read XRAMFS: Fail\r\n"));
			else
				xSerialPrint_P(PSTR("Read XRAMFS: Success\r\n"));

			// Randomise the RAM contents
			for (uint16_t i = 0; i < testRAMFS.ram_size; i++)
				pLocalRAM[i] = (uint8_t)rand();

			// Set the Command for Swap (with XRAMFS)
			testRAMFS.ram_cmd = Swap;

			if(ramfs_transfer_block( &testRAMFS, pLocalRAM ))
				xSerialPrint_P(PSTR("Swap XRAMFS: Fail\r\n"));
			else
				xSerialPrint_P(PSTR("Swap XRAMFS: Success\r\n"));

			// Set the Command for Swap (with XRAMFS)
			testRAMFS.ram_cmd = Swap;

			if(ramfs_transfer_block( &testRAMFS, pLocalRAM ))
				xSerialPrint_P(PSTR("Swap XRAMFS: Fail\r\n"));
			else
				xSerialPrint_P(PSTR("Swap XRAMFS: Success\r\n"));
			break;


		case 'z' : // Randomise the RAM contents.

			srand((uint16_t)xTaskGetTickCount()); // seed a random number

			if( (pLocalRAM != NULL) && (testRAMFS.ram_size != 0) )
				for (uint16_t i = 0; i < testRAMFS.ram_size; i++)
					pLocalRAM[i] = (uint8_t)rand();

			// calculate a CRC on the RAM.
			testRAMFS.ram_crc8 = crc8( pLocalRAM, ( sizeof(uint8_t) * testRAMFS.ram_size ) );

			xSerialPrintf_P(PSTR("Addr: %4x Size: %u crc: %2x\r\n"), testRAMFS.ram_addr, testRAMFS.ram_size, testRAMFS.ram_crc8 );
			break;


		case 'l' : // Generate load.

			// Create RAMFS structure. Allocate and fill local RAM with
			// > l [ram_size]    allocate, fill, generate load
			// > l               just generate load (allocate, fill, default 256 Bytes transfer)

			if (xatoi(&ptr, &p1))
				testRAMFS.ram_size = (uint16_t)p1;
			else
				testRAMFS.ram_size = (uint16_t)256;


				// "create" the RAMFS information.
				if( !(testRAMFS.ram_addr = (uint16_t ) vRAMFSMalloc( sizeof(uint8_t) * testRAMFS.ram_size )))
				{
					xSerialPrint_P(PSTR("vRAMFSMalloc for testRAMFS fail..!\r\n"));
					break;
				}

				// create the local RAM on the heap.
				if( !(pLocalRAM = (uint8_t *) pvPortMalloc( sizeof(uint8_t) * testRAMFS.ram_size )))
				{
					xSerialPrint_P(PSTR("pvPortMalloc for *pLocalRAM fail..!\r\n"));
					break;
				}

			srand((uint16_t)xTaskGetTickCount()); // seed a random number

			while(1)
			{
				xSerialPrint_P(PSTR("XRAMFS Test: "));

				for (uint16_t i = 0; i < testRAMFS.ram_size; i++)
					pLocalRAM[i] = (uint8_t)rand();

				// calculate a CRC on the local RAM.
				testRAMFS.ram_crc8 = crc8( pLocalRAM, ( sizeof(uint8_t) * testRAMFS.ram_size ) );

				xSerialPrintf_P(PSTR("Addr: %4x Size: %u crc: %2x -> "), testRAMFS.ram_addr, testRAMFS.ram_size, testRAMFS.ram_crc8 );

				// Set the Command for Write (to XRAMFS)
				testRAMFS.ram_cmd = Write;

				if(ramfs_transfer_block( &testRAMFS, pLocalRAM ))
					xSerialPrint_P(PSTR("Write XRAMFS: Fail\r\n"));

				// Randomise the RAM contents
				for (uint16_t i = 0; i < testRAMFS.ram_size; i++)
					pLocalRAM[i] = (uint8_t)rand();

				// Set the Command for Swap (with XRAMFS)
				testRAMFS.ram_cmd = Swap;

				if(ramfs_transfer_block( &testRAMFS, pLocalRAM ))
					xSerialPrint_P(PSTR("Swap XRAMFS: Fail\r\n"));

				// Set the Command for Swap (with XRAMFS)
				testRAMFS.ram_cmd = Swap;

				if(ramfs_transfer_block( &testRAMFS, pLocalRAM ))
					xSerialPrint_P(PSTR("Swap XRAMFS: Fail\r\n"));

				// Set the Command for Read (from XRAMFS)
				testRAMFS.ram_cmd = Read;

				if(ramfs_transfer_block( &testRAMFS, pLocalRAM ))
					xSerialPrint_P(PSTR("Read XRAMFS: Fail\r\n"));

				// calculate a CRC on the local RAM.
				if(testRAMFS.ram_crc8 != crc8( pLocalRAM, ( sizeof(uint8_t) * testRAMFS.ram_size ) ))
					xSerialPrint_P(PSTR("CRC8: VERIFICATION FAIL\r\n"));
				else
					xSerialPrint_P(PSTR("Pass\r\n"));

				testRAMFS.ram_addr += testRAMFS.ram_size;

				if( (testRAMFS.ram_addr < (size_t)XRAMSTART) || (testRAMFS.ram_size > (size_t)XRAMEND - testRAMFS.ram_addr) )
					testRAMFS.ram_addr = (size_t)XRAMSTART;

				vTaskDelay( (TickType_t) testRAMFS.ram_crc8 / portTICK_PERIOD_MS );	// delay for a random amount of time (being the CRC8 of the random numbers)
			}
			break;

		default :
			break;

		}
//		vTaskDelay(  50 / portTICK_PERIOD_MS );
//		xSerialPrintf_P(PSTR("Monitor - Stack HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
		vTaskDelay(  10 / portTICK_PERIOD_MS );
    }

}

/*-----------------------------------------------------------*/
/* Monitor                                                   */
/*-----------------------------------------------------------*/
static
void get_line (uint8_t *buff, uint8_t len)
{
	uint8_t c;
	uint8_t i = 0;

	for (;;) {
		while ( ! xSerialGetChar( &xSerialPort, &c ))
			vTaskDelay( 1 );

		if (c == '\r') break;
		if ((c == '\b') && i) {
			--i;
			xSerialPutChar(&xSerialPort, c );
			continue;
		}
		if (c >= ' ' && i < len - 1) {	/* Visible chars */
			buff[i++] = c;
			xSerialPutChar(&xSerialPort, c );
		}
	}
	buff[i] = 0;
	xSerialPrint((uint8_t *)"\r\n");
}


/*-----------------------------------------------------------


void vApplicationStackOverflowHook( xTaskHandle xTask,
                                    signed char *pcTaskName )
{
	DDRB  |= _BV(DDB5);
	PORTB |= _BV(PORTB5);       // main (red PB5) LED on. Arduino LED on and die.
	while(1);
}

-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t xTask,
									char *pcTaskName )
{
	/*---------------------------------------------------------------------------*\
	Usage:
	   called by task system when a stack overflow is noticed
	Description:
	   Stack overflow handler -- Shut down all interrupts, send serious complaint
	    to command port.
	Arguments:
	   pxTask - pointer to task handle
	   pcTaskName - pointer to task name
	Results:
	   <none>
	Notes:
	   This routine will never return.
	   This routine is referenced in the task.c file of FreeRTOS as an extern.
	\*---------------------------------------------------------------------------*/

	uint8_t* pC;
	uint16_t baud;

	/* shut down all interrupts */
	portDISABLE_INTERRUPTS();

	/* take over the command line buffer to generate our error message */
	pC = (uint8_t*) LineBuffer;

	strcat( (char*) pC, "StackOverflow\r\n" );
	strcat( (char*) pC, (char*) pcTaskName );
	strcat( (char*) pC, "\r\n" );

	pC = (uint8_t*) LineBuffer;

	/* Force the UART control register to be the way we want, just in case */

	UCSR0C = ( _BV( UCSZ01 ) | _BV( UCSZ00 ) );		// 8 data bits
	UCSR0B = _BV( TXEN0 );							// only enable transmit
	UCSR0A = 0;

	/* Calculate the baud rate register value from the equation in the
	* data sheet.  This calculation rounds to the nearest factor, which
	* means the resulting rate may be either faster or slower than the
	* desired rate (the old calculation was always faster).
	*
	* If the system clock is one of the Magic Frequencies, this
	* computation will result in the exact baud rate
	*/
	baud = ( ( ( configCPU_CLOCK_HZ / ( ( 16UL * 115200 ) / 2UL ) ) + 1UL ) / 2UL ) - 1UL;
	UBRR0 = baud;

	/* Send out the message, without interrupts.  Hard wired to USART 0 */
	while ( *pC )
	{
		while (!(UCSR0A & (1 << UDRE0)));
		UDR0 = *pC;
		pC++;
	}

	DDRB  |= _BV(DDB5);
	PORTB |= _BV(PORTB5);       // main (red PB5) LED on. Arduino LED on and die.
	while(1);
}

