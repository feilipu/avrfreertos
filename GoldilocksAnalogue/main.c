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

/* serial interface include file. */
#include "serial.h"

/* i2c Interface include file. */
#include "i2cMultiMaster.h"

/* SPI interface include file. */
#include "spi.h"

/* system time include file. */
#include "time.h"

#ifdef portHD44780_LCD
/* LCD (Freetronics 16x2) interface include file. */
#include "hd44780.h"
#endif

#ifdef portRTC_DEFINED
/* RTC interface (using I2C) include file. */
#include "rtc.h"
#endif

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

filter_t rx_filter;
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

#ifdef	portHD44780_LCD
	lcd_Init();

	lcd_Print_P(PSTR("Hello World!"));
	lcd_Locate (0, 0);
#endif

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

//    avrSerialPrint_P(PSTR("\r\n\nGoodbye... no space for idle task!\r\n")); // Doh, so we're dead...

#ifdef portHD44780_LCD
	lcd_Locate (1, 0);
	lcd_Print_P(PSTR("DEAD BEEF!"));
#endif
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

#if defined (portRTC_DEFINED)
	tm CurrTimeDate; 			// set up an array for the RTC info.

	// initialise I2C master interface, need to do this once only.
	// If there are two I2C processes, then do it during the system initiation.
	I2C_Master_Initialise((ARDUINO<<I2C_ADR_BITS) | (pdTRUE<<I2C_GEN_BIT));

#endif

    while(1)
    {

//   	vTaskDelay( 0 );
		vTaskDelayUntil( &xLastWakeTime, ( 1024 / portTICK_PERIOD_MS ) );

#if defined (portHD44780_LCD)
		lcd_Locate (0, 0);
		lcd_Printf_P(PSTR("Min Heap:%7u"), xPortGetMinimumEverFreeHeapSize() ); // needs heap_4 for this function to succeed.

#if defined (portRTC_DEFINED)
		if (getDateTimeDS1307( (ptm)&CurrTimeDate ) == pdTRUE){
			lcd_Locate (1, 8);
			lcd_Printf_P(PSTR("%02u:%02u:%02u"), CurrTimeDate.tm_hour, CurrTimeDate.tm_min, CurrTimeDate.tm_sec);
		}
#else
		lcd_Locate (1, 0);
		lcd_Printf_P(PSTR("Sys Tick:%7lu"), time(NULL));
#endif // portRTC_DEFINED

#endif // portHD44780_LCD

//	   	vTaskDelay( 0 );
		vTaskDelayUntil( &xLastWakeTime, ( 1024 / portTICK_PERIOD_MS ) );

		xSerialPrintf_P(PSTR("\r\nFree Heap Size: %u"),xPortGetMinimumEverFreeHeapSize() ); // needs heap_1.c, heap_2.c or heap_4.c

		xSerialPrintf_P(PSTR("RedLED HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
    }
}


static void TaskAnalogue(void *pvParameters) // Prepare the DAC
{
	(void) pvParameters;

#if defined(FLANGER)
	// create the buffer on the heap (so it can be moved later).
	if(delaymem == NULL) // if there is no Line buffer allocated (pointer is NULL), then allocate buffer.
		if( !(delaymem = (int16_t *) pvPortMalloc( sizeof(int16_t) * SIZE )))
			xSerialPrint_P(PSTR("pvPortMalloc for *delaymem fail..!\r\n"));
#endif

#if defined(MIC)
	/* Create the ring-buffers used by audio delay loop. */
	if( (delayDataPtr = (uint8_t *)pvPortMalloc( sizeof(uint16_t) * DELAY )))
		ringBuffer_InitBuffer( &delayBuffer, delayDataPtr, sizeof(uint16_t) * DELAY);
#endif

	xSerialPrintf_P(PSTR("\r\nDAC_Codec_init:"));
	DAC_init();

	xSerialPrintf_P(PSTR(" will"));
	/* Initialise the sample interrupt timer. Exact multiples of 2000Hz are ok with 8 bit Timer0, otherwise use 16 bit Timer1 */
//	AudioCodec_Timer0_init(SAMPLE_RATE);	// xxx set up the sampling Timer0 to 48000Hz (or lower), runs at audio sampling rate in Hz.
	AudioCodec_Timer1_init(SAMPLE_RATE);	// xxx set up the sampling Timer0 to 44100Hz (or odd rates), runs at audio sampling rate in Hz.

	xSerialPrintf_P(PSTR(" soon"));

	rx_filter.cutoff = \
	tx_filter.cutoff = 0xc000;				// set both filters to 3/8 of sample frequency.

	setIIRFilterLPF( &rx_filter );			// initialise received sample filter
	setIIRFilterLPF( &tx_filter );			// initialise transmit sample filter

	xSerialPrintf_P(PSTR(" be"));
	AudioCodec_ADC_init();					// set up ADC sampling on the ADC0, ADC1, ADC2 using Danger Shield to control.
											// or, Microphone on ADC7.

	AudioCodec_setHandler(audioCodec_dsp, &ch_A_out, &ch_B_out); // Set the call back function to do the audio processing.
																 //	Done this way so that we can change the audio handling depending on what we want to achieve.
	xSerialPrintf_P(PSTR(" done."));

//	xSerialPrintf_P(PSTR("\r\nFree Heap Size: %u"),xPortGetMinimumEverFreeHeapSize() ); // needs heap_1.c, heap_2.c or heap_4.c
//	xSerialPrintf_P(PSTR("\r\nAudio HighWater: %u\r\n"), uxTaskGetStackHighWaterMark(NULL));

//	spiSelect (Analogue);
//	vTaskSuspend(NULL);						// Well, we're pretty much done here.
	vTaskEndScheduler();					// Rely on Timer1 Interrupt for regular output.

	for(;;)
	{
//		if (spiSelect (Analogue))
		{
//			while(1)
			{
//				uint16_t j ;// = 0x1234;
//				uint16_t k ;// = 0x5678;

//				DAC_out(&j, &j);
//				DAC_out(&k, &k);

//				audioCodec_dsp(&j, &k);
//				DAC_out( &j, &k);

//				vTaskDelay( 0 );
			}
		}
//		spiDeselect (Analogue);

		xSerialPrintf_P(PSTR("\r\nAudio HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
		vTaskDelay( 1000 );
	}
}


/*--------------------------------------------------*/
/*------------Analogue Processing HERE--------------*/
/*--------------------------------------------------*/

#if defined (SINE)

// create sinewave lookup table
// PROGMEM stores the values in the program memory
const int16_t sinewave[] PROGMEM =
{
	// this file is a 1024 value sinewave lookup table of signed 16bit integers
	// you can replace it with your own waveform if you like
#include "sinetable.inc"
};

void audioCodec_dsp( uint16_t * ch_A,  uint16_t * ch_B)
{
	static uint16_t i = 0;

	*ch_A = (uint16_t)((  (int16_t)( pgm_read_word(&sinewave[i])) ) + 0x7fff); // put sinusoid out on A channel, shifted to (+)ve values only.
	*ch_B = (uint16_t)(( -(int16_t)( pgm_read_word(&sinewave[i])) ) + 0x7fff); // put inverted sinusoid out on B channel, shifted to (+)ve values only.

	i += 0x01;		// increment through the array of sinewave values. 0x01 is 43Hz @ 44.1kHz
	i &= 0x03ff;	// make sure that i stays less than 1024 (0x03ff), within the array of sinewave values

//	*ch_A = i;
//	*ch_B = i;
//	i += 0x10;		// increment through staircase values. 0x10 is LSB for 12 bit output.

#if ADCS == 0
	AudioCodec_ADC();
#elif ADCS == 1
	AudioCodec_ADC(&mod0_value);
#elif ADCS == 2
	AudioCodec_ADC(&mod0_value, &mod1_value);
#elif ADCS == 3
	AudioCodec_ADC(&mod0_value, &mod1_value, &mod2_value);
#endif
}

#elif defined (MUSIC)

#define SCALE (2)				// implement an averaging filter

void audioCodec_dsp( uint16_t * ch_A,  uint16_t * ch_B)
{
	static uint32_t t = 0;

	uint16_t A_temp;
	uint16_t B_temp;

	A_temp = *ch_A >> SCALE;	// reduce the old value of A channel by the SCALE factor
	*ch_A -= A_temp;			// now subtract the SCALEd old value
								// finally SCALE the new value and add it in, then cast it to make sure it is 8bits for putchar().

	B_temp = *ch_B >> SCALE;	// repeat as above for B channel.
	*ch_B -= B_temp;

	// original programmes use putchar. So we cast to 8 bits then move to the upper byte of the output 16bit word (The MCP4822 discards the lowest 4 bits).
	// eg putchar( t*((42&t>>10)%14))

//	*ch_A += (uint16_t)( (uint8_t)( t*((42&t>>10)%14)) )<<(8 - SCALE); // 8kHz
//	*ch_B += (uint16_t)( (uint8_t)( t*((42&t>>10)%14)) )<<(8 - SCALE);


//	*ch_A = *ch_B += (uint16_t)( (uint8_t)((t * (( t>>9 | t>>13 ) & 15)) & 129) )<<(8 - SCALE);	// sounds good, at 8kHz
//	*ch_A = *ch_B += (uint16_t)( (uint8_t)(t * ((t>>3 | t>>9) & 82 & t>>9)) )<<(8 - SCALE);		// works at 8kHz
//	*ch_A = *ch_B += (uint16_t)( (uint8_t)(t * (((t>>12)|(t>>8))&(63&(t>>4)))) )<<(8 - SCALE);	// very metal, works at 8kHz
//	*ch_A = *ch_B += (uint16_t)( (uint8_t)((t * (t>>8 + t>>9) * 100) + (0x8000 + pgm_read_word(&sinewave[0x3ff & t]))) )<<(8 - SCALE);	// quite aggressive
//	*ch_A = *ch_B += (uint16_t)( (uint8_t)(((t>>4) * (13 & ( 0x8898a989 >> (t>>11 & 30))) & 255) + ((((t>>9|(t>>2)|t>>8) * 10 + 4 *((t>>2) & t >> 15 | t >> 8)) & 255)>> 1)) )<<(8 - SCALE);
//	*ch_A = *ch_B += (uint16_t)( (uint8_t)(((t*(t>>12)&(201*t/100)&(199*t/100))&(t*(t>>14)&(t*301/100)&(t*399/100)))+((t*(t>>16)&(t*202/100)&(t*198/100))-(t*(t>>17)&(t*302/100)&(t*298/100)))) )<<(8 - SCALE);

	*ch_A += (uint16_t)( (uint8_t) ( (t&4096) ? ((t*(t^t%255)|(t>>4))>>1) : (t>>3)|((t&8192) ? t<<2 : t)) ) << (8 - SCALE); //  stereo 8kHz
	*ch_B += (uint16_t)( (uint8_t) (t*(((t>>9)^((t>>9)-1)^1)%13)) ) << (8 - SCALE);

	++t;		// increment through the counter by ones

#if ADCS == 0
	AudioCodec_ADC();
#elif ADCS == 1
	AudioCodec_ADC(&mod0_value);
#elif ADCS == 2
	AudioCodec_ADC(&mod0_value, &mod1_value);
#elif ADCS == 3
	AudioCodec_ADC(&mod0_value, &mod1_value, &mod2_value);
#endif
}

#elif defined(VCO)
// lookup table value location
uint32_t location;	// this is a 32bit number
					// the lower 8bits are the subsample fraction
					// and the upper 24 bits contain the sample number

// create sinewave lookup table
// PROGMEM stores the values in the program memory
const int16_t sinewave[] PROGMEM =
{
	// this file is a 1024 value sinewave lookup table of signed 16bit integers
	// you can replace it with your own waveform if you like
#include "sinetable.inc"
};

void audioCodec_dsp( uint16_t * ch_A,  uint16_t * ch_B) // Voltage controlled oscillator
{
	// create some temporary variables
	uint16_t temp0;
	uint8_t frac;
	int16_t temp1;
	int16_t temp2;
	int16_t temp3;

	// create a variable frequency and amplitude sine wave.
	// since we will be moving through the lookup table at
	// a variable frequency, we won't always land directly
	// on a single sample.  so we will average between the
	// two samples closest to us. This is called interpolation.
	// step through the table at rate determined by mod1
	// use upper byte of mod1 value to set the rate
	// and have an offset of 1 so there is always an increment.
	location += 1 + (mod1_value << 3);
	// if we've gone over the table boundary -> loop back
	location &= 0x0003ffff; // this is a faster way doing the table
						  // wrap around, which is possible
						  // because our table is a multiple of 2^n.
						  // otherwise you would do something like:
						  // if (location >= 1024*256) {
						  //   location -= 1024*256;
						  // }
	temp0 = (location >> 8);
	// get first sample and store it in temp1
	temp1 = pgm_read_word_near(sinewave + temp0);
	++temp0; // go to next sample
	temp0 &= 0x03ff; // check if we've gone over the boundary.
				   // we can do this because its a multiple of 2^n,
				   // otherwise it would be:
				   // if (temp0 >= 1024) {
				   //   temp0 = 0; // reset to 0
				   // }
	// get second sample and put it in temp2
	temp2 = pgm_read_word_near(sinewave + temp0);

	// interpolate between samples
	// multiply each sample by the fractional distance
	// to the actual location value
	frac = (uint8_t)(location & 0x000000ff); // fetch the lower 8b
	MultiSU16X8toH16(temp3, temp2, frac);
	// scaled sample 2 is now in temp3, and since we are done with
	// temp2, we can reuse it for the next result
	MultiSU16X8toH16(temp2, temp1, 0xff - frac);
	// temp2 now has the scaled sample 1
	temp2 += temp3; // add samples together to get an average
	// our sine wave is now in temp2

	// set amplitude with mod0
	// multiply our sine wave by the mod0 value
	MultiSU16X16toH16(temp1, temp2, mod0_value << 3);
	// our sine wave is now in temp1
	*ch_A =  temp1 + 0x7fff; // put sinusoid out on A channel, shifted to (+)ve values only.
	*ch_B = -temp1 + 0x7fff; // put inverted version out on B channel, shifted to (+)ve values only.

	// adc sampling routine
	// sampling the potentiometers (no sound here) - only needed if the pots are used to modify sound
	// & is required before ADC variables

#if ADCS == 0
	AudioCodec_ADC();
#elif ADCS == 1
	AudioCodec_ADC(&mod0_value);
#elif ADCS == 2
	AudioCodec_ADC(&mod0_value, &mod1_value);
#elif ADCS == 3
	AudioCodec_ADC(&mod0_value, &mod1_value, &mod1_value);
#endif
}

#elif defined(MIC)

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

	IIRFilter( &rx_filter, &xn);	// filter received sample train

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

#elif defined (WALKIE_TALKIE)

void audioCodec_dsp( uint16_t * ch_A,  uint16_t * ch_B)
{
	int16_t xn;
	uint8_t cn;
	/*----- Audio Rx -----*/

	/* Get the next character from the ring buffer. */

	if( ringBuffer_IsEmpty( (ringBuffer_t*) &(xSerialPort.xRxedChars) ) )
	{
		cn = 0x80 ^ 0x55; // put A-Law nulled signal on the output.
	}
	else if (ringBuffer_GetCount( &(xSerialPort.xRxedChars) ) > (portSERIAL_BUFFER_RX>>1) ) // if the buffer is more than half full.
	{
		cn = ringBuffer_Pop( (ringBuffer_t*) &(xSerialPort.xRxedChars) ); // pop two samples to catch up, discard first one.
		cn = ringBuffer_Pop( (ringBuffer_t*) &(xSerialPort.xRxedChars) );
	}
	else
	{
		cn = ringBuffer_Pop( (ringBuffer_t*) &(xSerialPort.xRxedChars) ); // pop a sample
	}

	alaw_expand1(&cn, &xn);	// expand the A-Law compression

	*ch_A = *ch_B = (uint16_t)(xn + 0x7fff); // move the signal to positive values, and put signal out on A & B channel.

	/*----- Audio Tx -----*/

	AudioCodec_ADC( &mod7_value.u16 );	// sample is 10bits left justified.

	xn = mod7_value.u16 - 0x7fe0;	// centre the sample to 0 by subtracting 1/2 10bit range.

	IIRFilter( &tx_filter, &xn);	// filter transmitted sample train

	alaw_compress1(&xn, &cn);	// compress using A-Law

	xSerialPutChar( &xSerialPort, cn);	// transmit the sample
}
#endif



#if 0 // things that haven't found a use yet, but are in the file set.

// create logarithmic frequency lookup table
// PROGMEM stores the values in the program memory
const uint16_t logtable[] PROGMEM = {
  // this file is a 256 value logarithmic lookup table of unsigned 16bit integers
  // you can replace it with your own table if you like
  #include "logtable.inc"
};
#endif

