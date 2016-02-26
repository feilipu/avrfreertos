////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////    main.c
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include <avr/io.h>
#include <avr/interrupt.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* ring buffer include file. */
#include "ringBuffer.h"
#include "../lib_eefs/eefs_ringBuffer.h"

/* serial interface include file. */
#include "serial.h"

/* i2c Interface include file. */
#include "i2cMultiMaster.h"

/* SPI interface include file. */
#include "spi.h"

/* system time include file. */
#include "time.h"


/* Goldilocks Analogue and other DAC functions include file. */
#include "DAC.h"

/* Gadget Shield and its sliders */
#include "GadgetShield.h"
#include "g711.h"

/*-----------------------------------------------------------*/
/* Create a handle for the serial port. */
extern xComPortHandle xSerialPort;

static void TaskBlinkRedLED(void *pvParameters); // Main Arduino Mega 2560, Freetronics EtherMega (Red) LED Blink

static void TaskAnalogue(void *pvParameters);   // Manage Analogue


void audioCodec_dsp( uint16_t * ch_A, uint16_t * ch_B) __attribute__ ((hot, flatten));
	// prototype for the DSP function to be implemented.
	// needs to at least provide *ch_A and *ch_B
	// within Timer1 interrupt routine - time critical I/O. Keep it short and punchy!


uint16_t ch_A_out; // storage for the values to be written to MCP4822
uint16_t ch_B_out;

filter_t tx_filter;

/*-----------------------------------------------------------*/

/* Main program loop */
int main(void) __attribute__((OS_main));

int main(void)
{
	// turn on the serial port for debugging or for other USART reasons.
	xSerialPort = xSerialPortInitMinimal( USART0, 38400, portSERIAL_BUFFER_TX, portSERIAL_BUFFER_RX); //  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)

	//	avrSerialPrint... doesn't need the scheduler running, so we can see freeRTOS initiation issues
	avrSerialPrint_P(PSTR("\r\n\nHello World!\r\n")); // Ok, so we're alive...

    xTaskCreate(
		TaskBlinkRedLED
		,  (const portCHAR *)"RedLED" // Main Arduino Mega 2560, Freetronics EtherMega (Red) LED Blink
		,  256				// Tested 9 free @ 208
		,  NULL
		,  1
		,  NULL ); // */


    xTaskCreate(
		TaskAnalogue
		,  (const portCHAR *) "Analogue"
		,  256  // This stack size can be checked & adjusted by reading Highwater
		,  NULL
		,  1
		,  NULL ); // */

//	avrSerialPrintf_P(PSTR("\r\nFree Heap Size: %u\r\n"),xPortGetMinimumEverFreeHeapSize() ); // needs heap_1.c, heap_2.c or heap_4.c

    vTaskStartScheduler(); // start the task scheduler, which takes over control of individual tasks. Should never return to here.

//  avrSerialPrint_P(PSTR("\r\n\nGoodbye... no space for idle task!\r\n")); // Doh, so we're dead...

}

/*--------------------------------------------------*/
/*---------------Task Definitions-------------------*/
/*--------------------------------------------------*/


static void TaskBlinkRedLED(void *pvParameters) // Main Red LED Flash
{
    (void) pvParameters;;
    TickType_t xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
		vTaskDelayUntil( &xLastWakeTime, ( 1024 / portTICK_PERIOD_MS ) );

		xSerialPrintf_P(PSTR("\r\nFree Heap Size: %u"),xPortGetMinimumEverFreeHeapSize() ); // needs heap_1.c, heap_2.c or heap_4.c

		xSerialPrintf_P(PSTR("\r\nRedLED HighWater @ %u"), uxTaskGetStackHighWaterMark(NULL));
    }
}


static void TaskAnalogue(void *pvParameters) // Prepare the DAC
{
	(void) pvParameters;

#if  defined(USE_EEFS)
	/* Create the SPI SRAM ring-buffers used by audio delay loop. */
	eefs_ringBuffer_InitBuffer( &eefs_delayBuffer, (uint_farptr_t)(RAM0_ADDR + 0xFF), sizeof(uint8_t) * DELAY);
#else

	/* Create the ring-buffers used by audio delay loop. */
	if( (delayDataPtr = (uint8_t *)pvPortMalloc( sizeof(uint8_t) * DELAY )))
		ringBuffer_InitBuffer( &delayBuffer, delayDataPtr, sizeof(uint8_t) * DELAY);
#endif

	xSerialPrintf_P(PSTR("\r\nDAC_Codec_init:"));
	DAC_init();

	xSerialPrintf_P(PSTR(" will"));

	/* Initialise the sample interrupt timer. Exact multiples of 2000Hz are ok with 8 bit Timer0, otherwise use 16 bit Timer1 */
	AudioCodec_Timer0_init(SAMPLE_RATE);	// xxx set up the sampling Timer0 to 48000Hz (or lower), runs at audio sampling rate in Hz.
//	AudioCodec_Timer1_init(SAMPLE_RATE);	// xxx set up the sampling Timer0 to 44100Hz (or odd rates), runs at audio sampling rate in Hz.

	xSerialPrintf_P(PSTR(" soon"));

	tx_filter.cutoff = 0xc000;				// set filter to 3/8 of sample frequency.
	setIIRFilterLPF( &tx_filter );			// initialise transmit sample filter

	xSerialPrintf_P(PSTR(" be"));

	AudioCodec_ADC_init();					// set up ADC sampling on the ADC0, ADC1, ADC2 using Danger Shield to control.
											// or, Microphone on ADC7.

	AudioCodec_setHandler(audioCodec_dsp, &ch_A_out, &ch_B_out); // Set the call back function to do the audio processing.
																 //	Done this way so that we can change the audio handling depending on what we want to achieve.

	xSerialPrintf_P(PSTR(" done."));

//	xSerialPrintf_P(PSTR("\r\nFree Heap Size: %u"),xPortGetMinimumEverFreeHeapSize() ); // needs heap_1.c, heap_2.c or heap_4.c
//	xSerialPrintf_P(PSTR("\r\nAudio HighWater: %u\r\n"), uxTaskGetStackHighWaterMark(NULL));

//	vTaskSuspend(NULL);						// Well, we're pretty much done here.
//	vTaskEndScheduler();					// Rely on Timer0/1 Interrupt for regular output.

	for(;;)
	{
		xSerialPrintf_P(PSTR("\r\nAudio HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
		vTaskDelay( 1000 );
	}
}


/*--------------------------------------------------*/
/*------------Analogue Processing HERE--------------*/
/*--------------------------------------------------*/

#if  defined(USE_EEFS)
void audioCodec_dsp( uint16_t * ch_A,  uint16_t * ch_B)
{
	int16_t xn;
	uint8_t cn;

	if( eefs_ringBuffer_GetCount(&eefs_delayBuffer) >= DELAY )
	{
		cn = eefs_ringBuffer_Pop(&eefs_delayBuffer);
	}
	else
	{
		cn = 0x80 ^ 0x55; // put A-Law nulled signal on the output.
	}

	alaw_expand1(&cn, &xn);	// expand the A-Law compression

	*ch_A = *ch_B = (uint16_t)(xn + 0x7fff); // put signal out on A & B channel.

	AudioCodec_ADC(&mod7_value.u16);

	xn = mod7_value.u16 - 0x7fe0;	// centre the sample to 0 by subtracting 1/2 10bit range.

	IIRFilter( &tx_filter, &xn);	// filter sample train

	alaw_compress1(&xn, &cn);	// compress using A-Law

	if( eefs_ringBuffer_GetCount(&eefs_delayBuffer) <= DELAY )
	{
		eefs_ringBuffer_Poke(&eefs_delayBuffer, cn);
	}
}
#else

void audioCodec_dsp( uint16_t * ch_A,  uint16_t * ch_B)
{
	int16_t xn;
	uint8_t cn;

	if( ringBuffer_GetCount(&delayBuffer) >= DELAY )
	{
		cn = ringBuffer_Pop(&delayBuffer);
	}
	else
	{
		cn = 0x80 ^ 0x55; // put A-Law nulled signal on the output.
	}

	alaw_expand1(&cn, &xn);	// expand the A-Law compression

	*ch_A = *ch_B = (uint16_t)(xn + 0x7fff); // put signal out on A & B channel.

	AudioCodec_ADC( &mod7_value.u16 );	// sample is 10bits left justified.

	xn = mod7_value.u16 - 0x7fe0;	// centre the sample to 0 by subtracting 1/2 10bit range.

	IIRFilter( &tx_filter, &xn);	// filter sample train

	alaw_compress1(&xn, &cn);	// compress using A-Law

	if( ringBuffer_GetCount(&delayBuffer) <= DELAY )
	{
		ringBuffer_Poke(&delayBuffer, cn);
	}
}
#endif

