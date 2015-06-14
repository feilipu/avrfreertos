////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////    main.c
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////


#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <avr/io.h>
#include <avr/interrupt.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* serial interface include file. */
#include "serial.h"

#include "spi.h"

/* system time include file. */
#include "time.h"

#include "AudioCodec.h"

/*-----------------------------------------------------------*/

/* Create a handle for the serial port. */
extern xComPortHandle xSerialPort; // To enable serial port

static void TaskAudio(void *pvParameters);   // Manage Audio

/*-----------------------------------------------------------*/
/* Main program loop */
int main(void) __attribute__((OS_main));

int main(void)
{
	//	To enable serial port for debugging or other purposes.
	xSerialPort = xSerialPortInitMinimal( USART0, 115200, portSERIAL_BUFFER_TX, portSERIAL_BUFFER_RX); //  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)

	avrSerialPrint_P(PSTR("\r\n\nHello World!\r\n")); // Ok, so we're alive...

    xTaskCreate(
		TaskAudio
		,  (const portCHAR *) "Audio"
		,  256  // This stack size can be checked & adjusted by reading Highwater
		,  NULL
		,  1
		,  NULL );

	avrSerialPrintf_P(PSTR("\r\nFree Heap Size: %u\r\n"),xPortGetFreeHeapSize() ); // needs heap_1.c, heap_2.c or heap_4.c

    vTaskStartScheduler(); // start the task scheduler, which takes over control of individual tasks. Should never return to here.

    avrSerialPrint_P(PSTR("\r\n\nGoodbye... no space for idle task!\r\n")); // Doh, so we're dead...

}

/*--------------------------------------------------*/
/*-------------Audio Processing HERE----------------*/
/*--------------------------------------------------*/



#if defined(IO)
void AudioCodec_dsp(void) // straight through connection I -> O
{
	right_out =  right_in; // put in to out on right channel
	left_out  =  left_in; // put in to out on left channel
}

#elif defined(VOLUME)
void AudioCodec_dsp(void) // simple volume control, level on mod0
{
	MultiSU16X16toH16Round(right_out, right_in, mod0_value << 3); // put right_in multiplied by 13 bit mod0 pot to right_out channel
	MultiSU16X16toH16Round(left_out, left_in, mod0_value << 3);   // put left_in multiplied by 13 bit mod0 pot to left_out channel
}

#elif defined (SINE)

// create sinewave lookup table
// PROGMEM stores the values in the program memory
const int16_t sinewave[] PROGMEM =
{
	// this file is a 1024 value sinewave lookup table of signed 16bit integers
	// you can replace it with your own waveform if you like
#include "sinetable.inc"
};

void AudioCodec_dsp(void)
{
	static uint16_t i = 0;

	right_out = (int16_t)( pgm_read_word(&sinewave[i])); // put sinusoid out on A channel.
	left_out = -(int16_t)( pgm_read_word(&sinewave[i])); // put inverted sinusoid out on B channel.

	i += 0x01;		// increment through the array of sinewave values. 0x01 is 43Hz @ 44.1kHz
	i &= 0x03ff;	// make sure that i stays less than 1024 (0x03ff), within the array of sinewave values

//	*ch_A = i;
//	*ch_B = i;
//	i += 0x10;		// increment through staircase values. 0x10 is LSB for 12 bit output.
}

#elif defined (MUSIC)

// create sinewave lookup table
// PROGMEM stores the values in the program memory
const int16_t sinewave[] PROGMEM =
{
	// this file is a 1024 value sinewave lookup table of signed 16bit integers
	// you can replace it with your own waveform if you like
#include "sinetable.inc"
};

void AudioCodec_dsp(void)
{
	static uint32_t t = 0;

	// original programmes use putchar. So we cast to 8 bits then move to the upper byte of the output word.
	// eg putchar( t*((42&t>>10)%14))

//	left_out =  (uint16_t)( (uint8_t)( t*((42&t>>10)%14)) )<<8; //  stereo 8kHz
//	right_out = (uint16_t)( (uint8_t)( t*((42&t>>10)%14)) )<<8;

//	left_out = right_out = (uint16_t)( (uint8_t)((t * (( t>>9 | t>>13 ) & 15)) & 129) )<<8;	// sounds good but quite low, at 8,000Hz
//	left_out = right_out = (uint16_t)( (uint8_t)(t * ((t>>3 | t>>9) & 82 & t>>9)) )<<8;		// metallic drum kit, works at 8,000Hz
//	left_out = right_out = (uint16_t)( (uint8_t)(t * (((t>>12)|(t>>8))&(63&(t>>4)))) )<<8;	// very metallic, works at 8,000Hz
//	left_out = right_out = (uint16_t)( (uint8_t)((t * (t>>8 + t>>9) * 100) + (0x8000 + pgm_read_word(&sinewave[t]))) )<<8;	// quite aggressive
//	left_out = right_out = (uint16_t)( (uint8_t)(((t>>4) * (13 & ( 0x8898a989 >> (t>>11 & 30))) & 255) + ((((t>>9|(t>>2)|t>>8) * 10 + 4 *((t>>2) & t >> 15|t >> 8)) & 255)>> 1)) )<<8;
//	left_out = right_out = (uint16_t)( (uint8_t)(((t*(t>>12)&(201*t/100)&(199*t/100))&(t*(t>>14)&(t*301/100)&(t*399/100)))+((t*(t>>16)&(t*202/100)&(t*198/100))-(t*(t>>17)&(t*302/100)&(t*298/100)))) )<<8;

	left_out =  (uint16_t)( (uint8_t)( (t&4096) ? ((t*(t^t%255)|(t>>4))>>1) : (t>>3)|((t&8192) ? t<<2 : t) ) )<<8; //  stereo 11kHz
	right_out = (uint16_t)( (uint8_t)(t*(((t>>9)^((t>>9)-1)^1)%13)) )<<8;

	++t;		// increment through the counter by ones
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

void AudioCodec_dsp(void) // Voltage controlled oscillator
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
	temp1 = pgm_read_word(sinewave + temp0);
	++temp0; // go to next sample
	temp0 &= 0x03ff; // check if we've gone over the boundary.
				   // we can do this because its a multiple of 2^n,
				   // otherwise it would be:
				   // if (temp0 >= 1024) {
				   //   temp0 = 0; // reset to 0
				   // }
	// get second sample and put it in temp2
	temp2 = pgm_read_word(sinewave + temp0);

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
	right_out = temp1; // put  sinusoid out on right channel
	left_out  = -temp1; // put inverted version out on left channel
}

#elif defined(FLANGER)
uint16_t lookup;	// lookup table value location
uint16_t location = 0; // buffer location to read/write from

// create sinewave lookup table
// PROGMEM stores the values in the program memory
const int16_t sinewave[] PROGMEM =
{
	// this file is a 1024 value sinewave lookup table of signed 16bit integers
	// you can replace it with your own waveform if you like
#include "sinetable.inc"
};

void AudioCodec_dsp(void) // flanger
{
	// create some temporary variables
	// these tend to work faster than using the main data variables
	// as they aren't fetched and stored all the time
	int16_t temp1;
	int16_t temp2;
	int16_t temp3;
	int16_t temp4;

	// create a variable frequency and amplitude sine wave
	// fetch a sample from the lookup table
	temp1 = pgm_read_word(sinewave + lookup);

	// step through table at rate determined by mod1
	// use upper byte of mod1 value to set the rate.
	lookup += 1 + (mod1_value >> 8);

	// if we've gone over the table boundary -> loop back
	// around to the other side.
	lookup &= 0x03ff; // fast way of dealing with roll over
					  // only works for 2^n values

	// set amplitude with mod0
	// multiply our sinewave by the mod0 value
	MultiSU16X16toH16(temp2, temp1, mod0_value << 3);
	// our sine wave is now in temp2

	// create a flanger effect by moving through delayed data
	// store incoming data
	delaymem[location++] = (left_in >> 1) + (right_in >> 1);
	// post increment location to go to next memory location

	// check if location has gotten bigger than buffer size
	if (location >= SIZE) {
	  location = 0; // reset location
	}

	// fetch delayed data with sinusoidal offset (temp2)
	uint16_t x; // create a temporary buffer index
	x = (uint16_t)(location + (SIZE/2) + (temp2 >> 8));
	if (x >= SIZE) { // check for buffer overflow
	  x -= SIZE;
	}
	temp1 = delaymem[x];

	// fetch next delayed data for interpolation
	if (++x == SIZE) { // check for buffer overflow
	  x = 0;
	}
	temp3 = delaymem[x];

	// interpolate between values
	MultiSU16X8toH16(temp4, temp3, (uint8_t)(temp2 & 0xff) );
	MultiSU16X8toH16(temp3, temp1, (uint8_t)(0xff - (temp2 & 0xff)) );

	// mix delayed and current data at each output
	right_out = (right_out >> 1) + (temp3 >> 1) + (temp4 >> 1);
	left_out = (left_out >> 1) + (temp3 >> 1) + (temp4 >> 1);
}

#else // things that haven't found a use yet, but are in the file set.

// create logarithmic frequency lookup table
// PROGMEM stores the values in the program memory
const uint16_t logtable[] PROGMEM = {
  // this file is a 256 value logarithmic lookup table of unsigned 16bit integers
  // you can replace it with your own table if you like
  #include "logtable.inc"
};

#endif

/*--------------------------------------------------*/
/*---------------Task Definitions-------------------*/
/*--------------------------------------------------*/

static void TaskAudio(void *pvParameters) // Prepare the Audio Codec
{
	(void) pvParameters;

#if defined(FLANGER)
	// create the buffer on the heap (so it can be moved later).
	if(delaymem == NULL) // if there is no Line buffer allocated (pointer is NULL), then allocate buffer.
		if( !(delaymem = (int16_t *) pvPortMalloc( sizeof(int16_t) * SIZE )))
			xSerialPrint_P(PSTR("pvPortMalloc for *delaymem fail..!\r\n"));
#endif

	xSerialPrintf_P(PSTR("\r\nAudioCodec_init:"));
	AudioCodec_ADC_init();					// initialise the potentiometer sampling.
	xSerialPrintf_P(PSTR(" will"));
	AudioCodec_SPI_init();					// initialise the SPI bus for special purpose Audio Codec use.
	xSerialPrintf_P(PSTR(" soon"));
	AudioCodec_init();						// initialise the Audio Codec using I2C bus.
	xSerialPrintf_P(PSTR(" be"));
	AudioCodec_Timer1_init();				// set up the sampling Timer1, runs at audio sampling rate.
	xSerialPrintf_P(PSTR(" done."));

	xSerialPrintf_P(PSTR("\r\nFree Heap Size: %u"),xPortGetFreeHeapSize() ); // needs heap_1.c, heap_2.c or heap_4.c
	xSerialPrintf_P(PSTR("\r\nAudio HighWater: %u\r\n"), uxTaskGetStackHighWaterMark(NULL));

	vTaskDelay( 512 / portTICK_PERIOD_MS );
//	vTaskSuspend(NULL);						// Well, we're pretty much done here.
	vTaskEndScheduler();					// Rely on Timer1 Interrupt for regular output.

	for(;;)
	{
//		vTaskDelay( 400 / portTICK_PERIOD_MS );
//		xSerialPrintf_P(PSTR("\r\nAudio HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
	}
}


/*--------------------------------------------------*/
/*----------------Stack Overflow--------------------*/
/*--------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t xTask,
									portCHAR *pcTaskName )
{
	DDRB  |= _BV(DDB5);
	PORTB |= _BV(PORTB5);       // main (red PB5) LED on. Arduino LED on and die.
	while(1);
}

/*-------------------------------------------------*/
