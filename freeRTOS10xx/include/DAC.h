// DAC.h

#ifndef DAC_h // include guard
#define DAC_h

#include <math.h>

#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "FreeRTOS.h"
#include "spi.h"

#include "mult16x16.h"
#include "mult16x8.h"
#include "mult32x16.h"

#ifdef __cplusplus
extern "C" {
#endif

/*--------------------------------------------------*/
/*------------------Definitions---------------------*/
/*--------------------------------------------------*/

/*
 * XXX Define for using the normal SPI bus as in Prototype 1 and Prototype 2.
 * Un-define for using MSPI on USART1 in Prototype 3 and Production Goldilocks Analogue.
 */
// #define LEGACY_GA

/*
 * XXX Ping PD7 to check timing on the audio processing.
 * PD7 will be set high when the DAC interrupt is entered, and set low when exited.
 * This allows you to check that there is sufficient time for the rest of the system to operate,
 * and to ensure that the sample interrupts are not being missed or are overlapping.
 */
// #define DEBUG_PING

/*
 * Bytes used to control the MCP4822 DAC
 */
#define CH_A_OUT	0x10
#define CH_B_OUT	0x90

#define CH_A_OFF	0x00
#define CH_B_OFF	0x80

/*
 * The LDAC pin on the MCP4822 synchronises the two channels to produce a simultaneous sample output
 * (even though the samples are clocked in at slightly different times).
 * Added for support of LDAC pin on Goldilocks Analogue MCP4822
 */

#define DAC_PORT_LCAC		PORTC
#define DAC_PORT_DIR_LDAC	DDRC
#define DAC_PORT_PIN_LDAC	PINC
#define DAC_BIT_LDAC		_BV(PC3)



//==================================================
//****************** IIR Filter ******************//
//==================================================

#ifndef SAMPLE_RATE
#define SAMPLE_RATE		12000	// set up the sampling Timer1 to 48000Hz, 44100Hz (or lower), runs at audio sampling rate in samples per Second.
#endif							// 384 = 3 x 2^7 divisors 2k, 3k, 4k, 6k, 8k, 12k, 16k, 24k, 48k

// IIR filter coefficient scaling
// multiply the coefficients by 32, assume a(1) is 32 * (1 + alpha), to get better accuracy in fixed format.
#define IIRSCALEFACTOR		32
#define IIRSCALEFACTORSHIFT	5

// IIR Resonance (maximum) at the corner frequency.
#define Q_MAXIMUM 			(6.0f)
#define Q_LINEAR			(M_SQRT1_2)

// float-fix conversion macros
// assuming we're using 8.8 for the coefficients.
#define float2int(a) ((int16_t)((a)*256.0))
#define int2float(a) ((double)(a)/256.0)


/*--------------------------------------------------*/
/*--------------------Typedefs----------------------*/
/*--------------------------------------------------*/

typedef union _DAC_value_t{
	uint16_t u16;
	int16_t  i16;
	uint8_t  u8[2];
} DAC_value_t;

typedef struct __attribute__ ((packed)) _DAC_command_t {
	uint8_t command;
	DAC_value_t value;
} DAC_command_t;

/*
 * Prototype for the audio DSP function to be implemented.
 * Needs to at least provide *ch_A and *ch_B
 * Runs within Timer0/1 interrupt routine - time critical I/O.
 * Keep it very short and punchy!
 * Use the DEBUG pings to ensure you're not using too much time.
 */

typedef void (audioCodec_DSP)( uint16_t* ch_A, uint16_t* ch_B);

//==================================================
//****************** IIR Filter ******************//
//==================================================

typedef struct __attribute__ ((packed)) _filter_t {
	uint16_t sample_rate;	// sample rate in Hz
	uint16_t cutoff;		// normalised cutoff frequency, 0-65536. maximum is sample_rate/2
	uint16_t peak;			// normalised Q factor, 0-65536. maximum is Q_MAXIMUM
	int16_t b0,b1,b2,a1,a2; // Coefficients in 8.8 format
    int16_t xn_1, xn_2;	//IIR state variables
    int16_t yn_1, yn_2; //IIR state variables
} filter_t;


/*--------------------------------------------------*/
/*--------------------Local Variables---------------*/
/*--------------------------------------------------*/

#if defined(portANALOGUE) || defined(portANALOGSHIELD)

// DSP function callback pointer.
static audioCodec_DSP * audioHandler;

 // pointers to storage for the values to be written to MCP4822
uint16_t* ch_A_ptr;
uint16_t* ch_B_ptr;

#endif //defined(portANALOGUE) || defined(portANALOGSHIELD)

/*--------------------------------------------------*/
/*----------Public Function Definitions-------------*/
/*--------------------------------------------------*/

//========================================================
// Some info on the Timers.
// Timer 2 is a bit different to the other timers.

/*
 * Timer_2 / Counter_2 Prescale
 * CS22 CS21 CS20 Description
 * 0    0    0    No clock source (Timer/Counter stopped).
 * 0    0    1    clk T2S /1   (No prescaling).
 * 0    1    0    clk T2S /8   (From prescaler).
 * 0    1    1    clk T2S /32  (From prescaler).
 * 1    0    0    clk T2S /64  (From prescaler).
 * 1    0    1    clk T2S /128 (From prescaler).
 * 1    1    0    clk T2S /256 (From prescaler).
 */

/*
 * Timer_n / Counter_n Prescale
 * CSn2 CSn1 CSn0 Description
 * 0    0    0    No clock source (Timer/Counter stopped).
 * 0    0    1    clk I/O /1    (No prescaling).
 * 0    1    0    clk I/O /8    (From prescaler).
 * 0    1    1    clk I/O /64   (From prescaler).
 * 1    0    0    clk I/O /256  (From prescaler).
 * 1    0    1    clk I/O /1024 (From prescaler).
 * 1    1    0    External clock source on Tn pin. Clock on falling edge.
 * 1    1    1    External clock source on Tn pin. Clock on rising edge.
 */

void AudioCodec_Timer0_init(uint16_t samplesSecond);	// set up the sampling Timer0, runs at audio sampling rate in Hz.
                                                        // 8 bit timer useful for fractions of 48,000 Hz.
														// 32,000 Hz, 16,000 Hz, 8,000 Hz, 4,000 Hz, 2,000 Hz.
/*** Initialise one OR the other Timer (but not both) ***/
void AudioCodec_Timer1_init(uint16_t samplesSecond);	// set up the sampling Timer1, runs at audio sampling rate in Hz.
                                                        // 16 bit timer useful for any sampling rate.
                                                        // Including 44,100 Hz, 22,050 Hz, 11,025 Hz and any above rates too.

void AudioCodec_setHandler(audioCodec_DSP* handler, uint16_t* ch_A, uint16_t* ch_B);	// set up the DSP processing to prepare the samples.

void DAC_init(void);									// initialise the SPI or MSPIM bus specifically for DAC use.

// Assumes that the DAC SPI has already been selected, as the selection process is quite time consuming.

//	if (spiSelect (Analogue))
//	{
//		uint16_t j = 0x1234;
//		uint16_t k = 0x5678;
//		DAC_out(NULL, NULL); 	// the ch_A and ch_B are turned off by NULL, or
//		DAC_out(&j, &k);		// output the j and k values by reference
//		spiDeselect (Analogue);
//	}
void DAC_out(const uint16_t * ch_A, const uint16_t * ch_B) __attribute__ ((hot, flatten));

// Assumes that the DAC SPU has already been selected, as the selection process is quite time consuming.

//	if (spiSelect (Analogue))
//	{
//		DAC_out_P( (const uint16_t *)&sinewave[i], (const uint16_t*)&sinewave[i]); // for PROGMEM stored arrays of samples
//		spiDeselect (Analogue);
//	}
void DAC_out_P(const uint16_t * ch_A, const uint16_t * ch_B) __attribute__ ((hot, flatten));


//==================================================
//****************** IIR Filter ******************//
//==================================================
// second order IIR -- "Direct Form I Transposed"
//  a(0)*y(n) = b(0)*x(n) + b(1)*x(n-1) +  b(2)*x(n-2)
//                   - a(1)*y(n-1) -  a(2)*y(n-2)
// assumes a(0) = IIRSCALEFACTOR = 32

// Compute filter function coefficients
// http://en.wikipedia.org/wiki/Digital_biquad_filter
// https://www.hackster.io/bruceland/dsp-on-8-bit-microcontroller
// http://www.musicdsp.org/files/Audio-EQ-Cookbook.txt

void setIIRFilterLPF( filter_t *filter ); // Low Pass
void setIIRFilterBPF( filter_t *filter ); // Band Pass
void setIIRFilterHPF( filter_t *filter ); // High Pass

// returns y(n) filtered by the biquad IIR process in place of x(n)
void IIRFilter( filter_t *filter, int16_t * xn ) __attribute__ ((hot, flatten));


/*--------------------------------------------------*/
/*---------------Public Functions-------------------*/
/*--------------------------------------------------*/

#if defined(portANALOGUE) || defined(portANALOGSHIELD)
void DAC_init(void)
{

#if defined(portANALOGSHIELD)

	spiBegin(Analogue);							// initialise the SPI bus for DAC use.
	spiSetBitOrder(SPI_MSBFIRST);
	spiSetClockDivider(SPI_CLOCK_DIV2);

	spiSetDataMode(SPI_MODE1);

	DDRD |= _BV(DDD6) | _BV(DDD5) | _BV(DDD3);	// Set DAC _LDAC & DAC _SS & ADC _SS to be an output for the Analog Shield
	PORTD |= _BV(PORTD5) | _BV(DDD3);			// Pull high: the Analog Shield ADC _SS and DAC _SS.
	PORTD &= ~_BV(PORTD6);						// Pull low: the Analog Shield _LDAC.

#elif defined(portANALOGUE)

#if defined(LEGACY_GA)

	SPI_PORT_DIR_SS_DAC |= SPI_BIT_SS_DAC;		// Set _SS as output pin.
	SPI_PORT_SS_DAC |= SPI_BIT_SS_DAC;			// Pull _SS high to deselect the Goldilocks Analogue DAC.

	spiBegin(Analogue);							// initialise the SPI bus for DAC use.
	spiSetBitOrder(SPI_MSBFIRST);
	spiSetClockDivider(SPI_CLOCK_DIV2);
	spiSetDataMode(SPI_MODE0);

#else
	/* For Prototype 3 and onwards, we are using USART1 in MSPIM mode, rather than using the overloaded SPI interface, to drive the DAC.
	 * The SPI SCK is connected to XCK1, and the SPI MOSI is connected to TX1. SPI MISO is not connected, as it is not needed.
	 */

	SPI_PORT_DIR_SS_DAC |= SPI_BIT_SS_DAC;		// Set _SS as output pin.
	SPI_PORT_SS_DAC |= SPI_BIT_SS_DAC;			// Pull _SS high to deselect the Goldilocks Analogue DAC.

	DAC_PORT_DIR_LDAC |= DAC_BIT_LDAC;			// Set  MCP4822 _LDAC as output pin.
	DAC_PORT_LCAC |= DAC_BIT_LDAC;				// Pull  MCP4822 _LDAC high to disable the DAC latch. No output will be latched.

	UBRR1 = 0x0000;								// Set Baud Rate to maximum, to speed XCK1 setting time.

	DDRD |= _BV(PD4);							// Setting the XCK1 port pin as output, enables USART master SPI mode.
	UCSR1C = _BV(UMSEL11) | _BV(UMSEL10);		// Set USART MSPI mode of operation using SPI data mode 0,0. UCPHA1 = UCSZ10
	UCSR1B = _BV(TXEN1);						// Enable transmitter. Enable the TX1 (don't enable the RX1, and the rest of the interrupt enable bits leave set to 0 too).

	/* Set baud rate.  IMPORTANT: The actual Baud Rate must be set after the transmitter is enabled */
	UBRR1 = 0x0000;								// FCPU/2 = 0x0000

#endif // #if defined(LEGACY_GA)

#endif // #if defined(portANALOGUE) || defined(portANALOGSHIELD)
}

void AudioCodec_Timer0_init(uint16_t samplesSecond) // set up the sampling reconstruction Timer0, runs at audio sampling rate in Hz.
{
	if (TIMSK0 && _BV(OCIE0A))          // if the timer has already been set, then just adjust the sample rate.
	{
		OCR0A = (F_CPU / ((uint32_t)samplesSecond *64)) -1;
	}
	else                                // otherwise we have to set up the timer before enabling the interrupt routine.
	{
#if defined(DEBUG_PING)
		DDRD |= _BV(DDD7);              // set the debugging ping
		PORTD &= ~_BV(PORTD7);
#endif

		// setup Timer0 for codec clock division
		TCCR0A = _BV(WGM01);            // set to  CTC Mode 2.  TOP = OCR0A. Normal port operation, OC0A disconnected.
		TCCR0B = _BV(CS01) | _BV(CS00); // Clk I/O scaler 64(From prescaler) CTC Mode. TOP = OCR0A.  Clock on rising edge.
		TCNT0 =  0x00;                  // clear the counter.

		OCR0A = (F_CPU / ((uint32_t)samplesSecond *64)) -1;

		TIMSK0 = _BV(OCIE0A);           // turn on compare match interrupt
	}
}

void AudioCodec_Timer1_init(uint16_t samplesSecond) // set up the sampling reconstruction Timer1, runs at audio sampling rate in Hz.
{
	if( TIMSK1 && _BV(OCIE1A))          // if the timer has already been set, then just adjust the sample rate.
	{
		portENTER_CRITICAL(); // turn off interrupts
		OCR1A = (F_CPU / ((uint32_t)samplesSecond)) -1;
		portEXIT_CRITICAL();             // turn on interrupts
	}
	else                                 // otherwise we have to set up the timer before enabling the interrupt routine.
	{
#if defined(DEBUG_PING)
		DDRD |= _BV(DDD7);              // set the debugging ping
		PORTD &= ~_BV(PORTD7);
#endif

		// setup Timer1 for codec clock division
		TCCR1A = 0x00;                   // set to  CTC Mode.
		TCCR1B = _BV(WGM12) | _BV(CS10); // CTC Mode 4. TOP = OCR1A. No prescaling. Clock on rising edge.
		TCCR1C = 0x00;                   // not used

		portENTER_CRITICAL(); // turn off interrupts
		TCNT1 =  0x0000;                 // clear the counter, high byte first for 16bit writes. gcc compiler knows how to handle this.
		OCR1A = (F_CPU / ((uint32_t)samplesSecond)) -1;
		portEXIT_CRITICAL();             // turn on interrupts

		TIMSK1 = _BV(OCIE1A);            // turn on compare match interrupt
	}
}

/**
 * Sets the audio handler function.
 * This function will be called to process the audio at each sample period.
 *
 */
void AudioCodec_setHandler(audioCodec_DSP * handler, uint16_t *ch_A, uint16_t *ch_B)		// set up the DSP processing to prepare the samples.
{
	audioHandler = handler;
	ch_A_ptr = ch_A;
	ch_B_ptr = ch_B;
}

#endif // #if defined(portANALOGUE) || defined(portANALOGSHIELD)


//========================================================
//********************* IIR Filter *********************//
//========================================================
// second order IIR -- "Direct Form I Transposed"
//  a(0)*y(n) = b(0)*x(n) + b(1)*x(n-1) +  b(2)*x(n-2)
//                   - a(1)*y(n-1) -  a(2)*y(n-2)
// assumes a(0) = IIRSCALEFACTOR = 32

// Compute filter function coefficients
// http://en.wikipedia.org/wiki/Digital_biquad_filter
// https://www.hackster.io/bruceland/dsp-on-8-bit-microcontroller
// http://www.musicdsp.org/files/Audio-EQ-Cookbook.txt

void setIIRFilterLPF( filter_t *filter ) // Low Pass Filter Setting
{
	if ( !(filter->sample_rate) )
		filter->sample_rate = SAMPLE_RATE;

	if ( !(filter->cutoff) )
		filter->cutoff = UINT16_MAX >> 1; // 1/4 of sample rate = filter->sample_rate>>2

	if ( !(filter->peak) )
		filter->peak =  (uint16_t)(M_SQRT1_2 * UINT16_MAX / Q_MAXIMUM); // 1/sqrt(2) effectively

	double frequency = ((double)filter->cutoff * (filter->sample_rate>>1)) / UINT16_MAX;
	double q = (double)filter->peak * Q_MAXIMUM / UINT16_MAX;
	double w0 = (2.0 * M_PI * frequency) / filter->sample_rate;
	double sinW0 = sin(w0);
	double cosW0 = cos(w0);
	double alpha = sinW0 / (q * 2.0f);
	double scale = IIRSCALEFACTOR / (1 + alpha); // a0 = 1 + alpha

	filter->b0	= \
	filter->b2	= float2int( ((1.0 - cosW0) / 2.0) * scale );
	filter->b1	= float2int(  (1.0 - cosW0) * scale );

	filter->a1	= float2int( (-2.0 * cosW0) * scale );
	filter->a2	= float2int( (1.0 - alpha) * scale );
}

void setIIRFilterHPF( filter_t *filter ) // High Pass Filter Setting
{
	if ( !(filter->sample_rate) )
		filter->sample_rate = SAMPLE_RATE;

	if ( !(filter->cutoff) )
		filter->cutoff = UINT16_MAX >> 1; // 1/4 of sample rate = filter->sample_rate>>2

	if ( !(filter->peak) )
		filter->peak =  (uint16_t)(M_SQRT1_2 * UINT16_MAX / Q_MAXIMUM); // 1/sqrt(2) effectively

	double frequency = ((double)filter->cutoff * (filter->sample_rate>>1)) / UINT16_MAX;
	double q = (double)filter->peak * Q_MAXIMUM / UINT16_MAX;
	double w0 = (2.0 * M_PI * frequency) / filter->sample_rate;
	double sinW0 = sin(w0);
	double cosW0 = cos(w0);
	double alpha = sinW0 / (q * 2.0f);
	double scale = IIRSCALEFACTOR / (1 + alpha); // a0 = 1 + alpha

	filter->b0	= float2int( ((1.0 + cosW0) / 2.0) * scale );
	filter->b1	= float2int( -(1.0 + cosW0) * scale );
	filter->b2	= float2int( ((1.0 + cosW0) / 2.0) * scale );

	filter->a1	= float2int( (-2.0 * cosW0) * scale );
	filter->a2	= float2int( (1.0 - alpha) * scale );
}

void setIIRFilterBPF( filter_t *filter ) // Band Pass Filter Setting
{
	if ( !(filter->sample_rate) )
		filter->sample_rate = SAMPLE_RATE;

	if ( !(filter->cutoff) )
		filter->cutoff = UINT16_MAX >> 1; // 1/4 of sample rate = filter->sample_rate>>2

	if ( !(filter->peak) )
		filter->peak =  (uint16_t)(M_SQRT1_2 * UINT16_MAX / Q_MAXIMUM); // 1/sqrt(2) effectively

	double frequency = ((double)filter->cutoff * (filter->sample_rate>>1)) / UINT16_MAX;
	double q = (double)filter->peak * Q_MAXIMUM / UINT16_MAX;
	double w0 = (2.0 * M_PI * frequency) / filter->sample_rate;
	double sinW0 = sin(w0);
	double cosW0 = cos(w0);
	double alpha = sinW0 / (q * 2.0f);
	double scale = IIRSCALEFACTOR / (1 + alpha); // a0 = 1 + alpha

	filter->b0	= float2int( alpha * scale );
	filter->b1	= 0;
	filter->b2	= float2int( -alpha * scale );

	filter->a1	= float2int( (-2.0 * cosW0) * scale );
	filter->a2	= float2int( (1.0 - alpha) * scale );
}

// Coefficients in 8.8 format
// interim values in 24.8 format
// returns y(n) in place of x(n)
void IIRFilter( filter_t *filter, int16_t * xn )
{
    int32_t yn;			// current output
    int32_t  accum;		// temporary accumulator

    // sum the 5 terms of the biquad IIR filter
	// and update the state variables
	// as soon as possible
    MultiS16X16to32(yn,filter->xn_2,filter->b2);
    filter->xn_2 = filter->xn_1;

    MultiS16X16to32(accum,filter->xn_1,filter->b1);
    yn += accum;
    filter->xn_1 = *xn;

    MultiS16X16to32(accum,*xn,filter->b0);
    yn += accum;

    MultiS16X16to32(accum,filter->yn_2,filter->a2);
    yn -= accum;
    filter->yn_2 = filter->yn_1;

    MultiS16X16to32(accum,filter->yn_1,filter->a1);
    yn -= accum;

    filter->yn_1 = yn >> (IIRSCALEFACTORSHIFT + 8); // divide by a(0) = 32 & shift to 16.0 bit outcome from 24.8 interim steps

    *xn = filter->yn_1; // being 16 bit yn, so that's what we return.
}



/*--------------------------------------------------*/
/*-----------Time Critical Functions----------------*/
/*--------------------------------------------------*/

#if defined(portANALOGUE) || defined(portANALOGSHIELD)

void DAC_out(const uint16_t * ch_A, const uint16_t * ch_B)
{
	DAC_command_t write;

#if defined(portANALOGSHIELD)
	//control byte
	//A1 - Always 0
	//A0 - Always 0
	//LD1 - Load DAC selected
	//LD0 - Load DAC selected
	//NIL - NOT USED
	//DAC Sel 1 - Write DAC Buffer selected
	//DAC Sel 0 - Write DAC Buffer selected
	//PD0 - power down mode

	// If the SPI module has not been enabled yet, then return with nothing.
	if( !(SPCR & _BV(SPE)) ) return;

	// The SPI module is enabled, but it is in slave mode, so we can not
	// transmit the byte.  This can happen if SSbar is an input and it went low.
	// We will try to recover by setting the MSTR bit.
	// Check this once only at the start. Assume that things don't change.
	if( !(SPCR & _BV(MSTR)) )
		{
			SPCR |= _BV(MSTR);
			if( !(SPCR & _BV(MSTR)) ) return;
		}

	write.command = 0x00; // set to channel 0 '00'

	write.value.u16 = *ch_A;

	PORTD &= ~_BV(PORTD5);				// Pull _SS low to select the Analog Shield DAC
										// (because this might not be the first time through).

	SPDR = write.command;				// Begin transmission ch_A.
	while ( !(SPSR & _BV(SPIF)) );
	SPDR = write.value.u8[1];			// Begin transmission ch_A.
	while ( !(SPSR & _BV(SPIF)) );
	SPDR = write.value.u8[0];			// Continue transmission ch_A.

	write.value.u16 = *ch_B;
	write.command = 0x22; 				// set to channel 1 '01' Write to buffer with data and then load all DACs simultaneously from their corresponding buffers.

	while ( !(SPSR & _BV(SPIF)) );		// check we've finished ch_A.
	PORTD |= _BV(PORTD5);				// Pull _SS high to deselect the Analog Shield DAC.

	PORTD &= ~_BV(PORTD5); 				// Pull _SS low to select the Analog Shield DAC.
	SPDR = write.command;				// Begin transmission ch_B, and latch value into DAC.
	while ( !(SPSR & _BV(SPIF)) );
	SPDR = write.value.u8[1];			// Begin transmission ch_B.
	while ( !(SPSR & _BV(SPIF)) );
	SPDR = write.value.u8[0];			// Continue transmission ch_B.
	while ( !(SPSR & _BV(SPIF)) );		// check we've finished ch_B.
	PORTD |= _BV(PORTD5);				// Pull _SS high to deselect the Analog Shield DAC.

#elif defined(portANALOGUE)


#if defined(LEGACY_GA)

	if (ch_A != NULL)
	{
		write.value.u16 = (*ch_A) >> 4;
		write.value.u8[1] |= CH_A_OUT;
	}
	else								// ch_A is NULL so we turn off the DAC
	{
		write.value.u8[1] = CH_A_OFF;
	}

	SPI_PORT_SS_DAC &= ~SPI_BIT_SS_DAC;	// Pull SS low to select the Goldilocks Analogue DAC
										// (because this might not be the first time through).
	SPDR = write.value.u8[1];			// Begin transmission ch_A.
	while ( !(SPSR & _BV(SPIF)) );
	SPDR = write.value.u8[0];			// Continue transmission ch_A.

	if (ch_B != NULL)					// start processing ch_B while we're doing the ch_A transmission
	{
		write.value.u16 = (*ch_B) >> 4;
		write.value.u8[1] |= CH_B_OUT;
	}
	else								// ch_B is NULL so we turn off the DAC
	{
		write.value.u8[1] = CH_B_OFF;
	}

	while ( !(SPSR & _BV(SPIF)) );		// check we've finished ch_A.
	SPI_PORT_SS_DAC |= SPI_BIT_SS_DAC;	// Pull SS high to deselect the Goldilocks Analogue DAC, and latch value into DAC.

	SPI_PORT_SS_DAC &= ~SPI_BIT_SS_DAC; // Pull SS low to select the Goldilocks Analogue DAC.
	SPDR = write.value.u8[1];			// Begin transmission ch_B.
	while ( !(SPSR & _BV(SPIF)) );
	SPDR = write.value.u8[0];			// Continue transmission ch_B.
	while ( !(SPSR & _BV(SPIF)) );		// check we've finished ch_B.
	SPI_PORT_SS_DAC |= SPI_BIT_SS_DAC;	// Pull SS high to deselect the Goldilocks Analogue DAC, and latch value into DAC.

#else // For Prototype 3 and onwards
	DAC_PORT_LCAC &= ~DAC_BIT_LDAC;		// Pull MCP4822 _LDAC low to enable the DAC output to latch the previous sample input buffer values.
										// We do this first to minimise the sample jitter when latching output values.

//	if (ch_A != NULL)
//	{
		write.value.u16 = (*ch_A) >> 4;
		write.value.u8[1] |= CH_A_OUT;
//	}
//	else								// ch_A is NULL so we turn off the DAC
//	{
//		write.value.u8[1] = CH_A_OFF;
//	}

	DAC_PORT_LCAC |= DAC_BIT_LDAC;		// Pull MCP4822 _LDAC high to disable the DAC latch.
										// Values we write into the DAC buffer this time, will be latched into the DAC output next interrupt.

	UCSR1A = _BV(TXC1);					// Clear the Transmit complete flag.
	SPI_PORT_SS_DAC &= ~SPI_BIT_SS_DAC;	// Pull _SS low to select the Goldilocks Analogue DAC
	UDR1 = write.value.u8[1];			// Begin transmission ch_A.
	UDR1 = write.value.u8[0];			// Continue transmission ch_A. UDR1 is double buffered.

//	if (ch_B != NULL)					// start processing ch_B while we're doing the ch_A transmission
//	{
		write.value.u16 = (*ch_B) >> 4;
		write.value.u8[1] |= CH_B_OUT;
//	}
//	else								// ch_B is NULL so we turn off the DAC
//	{
//		write.value.u8[1] = CH_B_OFF;
//	}

	while ( !(UCSR1A & _BV(TXC1)) );	// Check we've finished ch_A.
	SPI_PORT_SS_DAC |= SPI_BIT_SS_DAC;	// Pull _SS high to deselect the Goldilocks Analogue DAC.

	UCSR1A = _BV(TXC1);					// Clear the Transmit complete flag.
	SPI_PORT_SS_DAC &= ~SPI_BIT_SS_DAC; // Pull _SS low to select the Goldilocks Analogue DAC.
	UDR1 = write.value.u8[1];			// Begin transmission ch_B.
	UDR1 = write.value.u8[0];			// Continue transmission ch_B.
	while ( !(UCSR1A & _BV(TXC1)) );	// Check we've finished ch_B.
	SPI_PORT_SS_DAC |= SPI_BIT_SS_DAC;	// Pull _SS high to deselect the Goldilocks Analogue DAC.

#endif

#endif
}

void DAC_out_P(const uint16_t * ch_A, const uint16_t * ch_B)
{
	DAC_command_t write;

#if defined(portANALOGSHIELD)
	//control byte
	//A1 - Always 0
	//A0 - Always 0
	//LD1 - Load DAC selected
	//LD0 - Load DAC selected
	//NIL - NOT USED
	//DAC Sel 1 - Write DAC Buffer selected
	//DAC Sel 0 - Write DAC Buffer selected
	//PD0 - power down mode

	// If the SPI module has not been enabled yet, then return with nothing.
	if( !(SPCR & _BV(SPE)) ) return;

	// The SPI module is enabled, but it is in slave mode, so we can not
	// transmit the byte.  This can happen if SSbar is an input and it went low.
	// We will try to recover by setting the MSTR bit.
	// Check this once only at the start. Assume that things don't change.
	if( !(SPCR & _BV(MSTR)) )
		{
			SPCR |= _BV(MSTR);
			if( !(SPCR & _BV(MSTR)) ) return;
		}

	write.command = 0x00; // Set to channel 0 '00'

	write.value.u16 = pgm_read_word(ch_A);

	PORTD &= ~_BV(PORTD5);				// Pull _SS low to select the Analog Shield DAC
										// (because this might not be the first time through).

	SPDR = write.command;				// Begin transmission ch_A.
	while ( !(SPSR & _BV(SPIF)) );
	SPDR = write.value.u8[1];			// Begin transmission ch_A.
	while ( !(SPSR & _BV(SPIF)) );
	SPDR = write.value.u8[0];			// Continue transmission ch_A.


	write.value.u16 = pgm_read_word(ch_B);
	write.command = 0x22; 				// Set to channel 1 '01' Write to buffer with data and then load all DACs simultaneously from their corresponding buffers.

	while ( !(SPSR & _BV(SPIF)) );		// check we've finished ch_A.
	PORTD |= _BV(PORTD5);				// Pull _SS high to deselect the Analog Shield DAC.

	PORTD &= ~_BV(PORTD5);				// Pull _SS low to select the Analog Shield DAC.
	SPDR = write.command;				// Begin transmission ch_B.
	while ( !(SPSR & _BV(SPIF)) );
	SPDR = write.value.u8[1];			// Begin transmission ch_B.
	while ( !(SPSR & _BV(SPIF)) );
	SPDR = write.value.u8[0];			// Continue transmission ch_B.
	while ( !(SPSR & _BV(SPIF)) );		// check we've finished ch_B.
	PORTD |= _BV(PORTD5);				// Pull _SS high to deselect the Analog Shield DAC, and latch value into DAC.

#elif defined(portANALOGUE)

#if defined(LEGACY_GA)
	if (ch_A != NULL)
	{
		write.value.u16 = pgm_read_word(ch_A) >> 4;
		write.value.u8[1] |= CH_A_OUT;
	}
	else								// ch_A is NULL so we turn off the DAC
	{
		write.value.u8[1] = CH_A_OFF;
	}

	SPI_PORT_SS_DAC &= ~SPI_BIT_SS_DAC;	// Pull SS low to select the Goldilocks Analogue DAC
										// (because this might not be the first time through).
	SPDR = write.value.u8[1];			// Begin transmission
	while ( !(SPSR & _BV(SPIF)) );
	SPDR = write.value.u8[0];			// Continue transmission

	if (ch_B != NULL)					// start ch_B while we're doing the transmission
	{
		write.value.u16 = pgm_read_word(ch_B) >> 4;
		write.value.u8[1] |= CH_B_OUT;
	}
	else								// ch_B is NULL so we turn off the DAC
	{
		write.value.u8[1] = CH_B_OFF;
	}

	while ( !(SPSR & _BV(SPIF)) );		// check we've finished ch_A.
	SPI_PORT_SS_DAC |= SPI_BIT_SS_DAC;	// Pull SS high to deselect the Goldilocks Analogue DAC, and latch value into DAC.

	SPI_PORT_SS_DAC &= ~SPI_BIT_SS_DAC;	// Pull SS low to select the Goldilocks Analogue DAC.
	SPDR = write.value.u8[1];			// Begin transmission
	while ( !(SPSR & _BV(SPIF)) );
	SPDR = write.value.u8[0];			// Continue transmission
	while ( !(SPSR & _BV(SPIF)) );		// check we've finished ch_B.
	SPI_PORT_SS_DAC |= SPI_BIT_SS_DAC;	// Pull SS high to deselect the Goldilocks Analogue DAC, and latch value into DAC.

#else // For Prototype 3 and onwards.
	DAC_PORT_LCAC &= ~DAC_BIT_LDAC;		// Pull MCP4822 _LDAC low to enable the DAC output to latch the previous sample input buffer values.
                                        // We do this first to minimise the sample jitter when latching output values.

//	if (ch_A != NULL)
//	{
		write.value.u16 = pgm_read_word(ch_A) >> 4;
		write.value.u8[1] |= CH_A_OUT;
//	}
//	else								// ch_A is NULL so we turn off the DAC
//	{
//		write.value.u8[1] = CH_A_OFF;
//	}

	DAC_PORT_LCAC |= DAC_BIT_LDAC;		// Pull MCP4822 _LDAC high to disable the DAC latch.
										// Values we write into the DAC buffer this time, will be latched into the DAC output next interrupt.

	UCSR1A = _BV(TXC1);					// Clear the Transmit complete flag.
	SPI_PORT_SS_DAC &= ~SPI_BIT_SS_DAC;	// Pull _SS low to select the Goldilocks Analogue DAC

	UDR1 = write.value.u8[1];			// Begin transmission
	UDR1 = write.value.u8[0];			// Continue transmission

	if (ch_B != NULL)					// start ch_B while we're doing the transmission
	{
		write.value.u16 = pgm_read_word(ch_B) >> 4;
		write.value.u8[1] |= CH_B_OUT;
	}
	else								// ch_B is NULL so we turn off the DAC
	{
		write.value.u8[1] = CH_B_OFF;
	}

	while ( !(UCSR1A & _BV(TXC1)) );	// Check we've finished ch_A.
	SPI_PORT_SS_DAC |= SPI_BIT_SS_DAC;	// Pull _SS high to deselect the Goldilocks Analogue DAC.

	UCSR1A = _BV(TXC1);					// Clear the Transmit complete flag.
	SPI_PORT_SS_DAC &= ~SPI_BIT_SS_DAC;	// Pull _SS low to select the Goldilocks Analogue DAC.
	UDR1 = write.value.u8[1];			// Begin transmission
	UDR1 = write.value.u8[0];			// Continue transmission
	while ( !(UCSR1A & _BV(TXC1)) );	// Check we've finished ch_B.
	SPI_PORT_SS_DAC |= SPI_BIT_SS_DAC;	// Pull _SS high to deselect the Goldilocks Analogue DAC.

#endif

#endif
}

#endif //#if defined(portANALOGUE) || defined(portANALOGSHIELD)

/*--------------------------------------------------*/
/*-------Interrupt (Loop at Sample Rate)------------*/
/*--------------------------------------------------*/

// This is an example interrupt process that will output values at the sample rate defined.
// Other processes may use the DAC_out_P to increment across a set of samples stored in PROGMEM

#if defined(portANALOGUE) || defined(portANALOGSHIELD)
ISR(TIMER1_COMPA_vect) __attribute__ ((hot, flatten));
ISR(TIMER1_COMPA_vect)
{
#if defined(DEBUG_PING)
	// start mark - check for start of interrupt - for debugging only
	PORTD |=  _BV(PORTD7);				// Ping IO line.
#endif

	// MCP4822 data transfer routine
	// move data to the MCP4822 - done first for regularity (reduced jitter).
	// &'s are necessary on data_in variables
	DAC_out  (ch_A_ptr, ch_B_ptr); // OR
//	DAC_out_P(ch_A_ptr, ch_B_ptr); // Where ch_A_ptr and ch_B_ptr are pointers to values stored in PROGMEM

	// audio processing routine - do whatever processing on input is required - prepare output for next sample.
	// Fire the global audio handler.
	if (audioHandler!=NULL)
		audioHandler(ch_A_ptr, ch_B_ptr);

#if defined(DEBUG_PING)
	// end mark - check for end of interrupt - for debugging only
	PORTD &= ~_BV(PORTD7);
#endif
}

ISR(TIMER0_COMPA_vect) __attribute__ ((hot, flatten));
ISR(TIMER0_COMPA_vect)
{
#if defined(DEBUG_PING)
	// start mark - check for start of interrupt - for debugging only
	PORTD |=  _BV(PORTD7);				// Ping IO line.
#endif

	// MCP4822 data transfer routine
	// move data to the MCP4822 - done first for regularity (reduced jitter).
	// &'s are necessary on data_in variables
	DAC_out  (ch_A_ptr, ch_B_ptr); // OR
//	DAC_out_P(ch_A_ptr, ch_B_ptr); // Where ch_A_ptr and ch_B_ptr are pointers to values stored in PROGMEM

	// audio processing routine - do whatever processing on input is required - prepare output for next sample.
	// Fire the global audio handler, if set.
	if (audioHandler!=NULL)
		audioHandler(ch_A_ptr, ch_B_ptr);

#if defined(DEBUG_PING)
	// end mark - check for end of interrupt - for debugging only
	PORTD &= ~_BV(PORTD7);
#endif
}

#endif //#if defined(portANALOGUE) || defined(portANALOGSHIELD)

#ifdef __cplusplus
}
#endif

#endif // DAC_h end include guard
