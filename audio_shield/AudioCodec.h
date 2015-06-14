// AudioCodec.h
/* guest openmusiclabs 7.28.11
// this is the library file for ARDUINO -> there is a different
// file for Maple, make sure you are using the right one.
// place this file in the libraries file of your Arduino sketches folder
// e.g. C:\Documents and Settings\user\My Documents\Arduino\libraries\ */
// you may have to create the \libraries folder
//*/


#ifndef AudioCodec_h // include guard
#define AudioCodec_h

#include <avr/io.h>
#include <avr/pgmspace.h>

#include "spi.h"
#include "i2cMultiMaster.h"

#include "mult16x16.h"
#include "mult16x8.h"
#include "mult32x16.h"

/*--------------------------------------------------*/
/*------------Often Configured Parameters-----------*/
/*--------------------------------------------------*/

// TODO Define the audio function that is desired by uncommenting one of below...
// #define IO
//#define VOLUME
//#define SINE
#define MUSIC
//#define VCO
//#define FLANGER

#if defined(IO)
#define ADCS 	0		// number of ADCs in use. 0, 1, or 2 of the rotary potentiometers.

#elif defined(SINE)
#define ADCS 	0		// number of ADCs in use. 0, 1, or 2 of the rotary potentiometers.

#elif defined(MUSIC)
#define ADCS 	0		// number of ADCs in use. 0, 1, or 2 of the linear potentiometers on a Danger Shield.

#elif defined(VOLUME)
#define ADCS 	1		// number of ADCs in use. 0, 1, or 2 of the rotary potentiometers.

#elif defined(VCO)
#define ADCS 	2		// number of ADCs in use. 0, 1, or 2 of the rotary potentiometers.

#elif defined(FLANGER)
#define ADCS 	2		// number of ADCs in use. 0, 1, or 2 of the rotary potentiometers.
// create a delay buffer in memory, 630 positions x 2 bytes = 1260 bytes of SRAM
#define SIZE 650 // buffer size is limited by micro-controller SRAM size
int16_t * delaymem = NULL; // Allocate later with pvPortMalloc()
#endif


// set up Left Input Volume
#ifndef LINVOL
  #define LINVOL 				23
#elif (LINVOL >= 0) && (LINVOL <= 0x1f)
  #error LINVOL value not properly defined
#endif

// set up Right Input Volume
#ifndef RINVOL
  #define RINVOL 				23
#elif (RINVOL >= 0) && (RINVOL <= 0x1f)
  #error RINVOL value not properly defined
#endif

// set up Left Headphone Volume
#ifndef LHPVOL
  #define LHPVOL 				121
#elif (LHPVOL == 0) || ((LHPVOL >= 0x30) && (LHPVOL <= 0x7f))
  #error LHPVOL value not properly defined
#endif

// set up Right Headphone Volume
#ifndef RHPVOL
  #define RHPVOL 				121
#elif (RHPVOL == 0) || ((RHPVOL >= 0x30) && (RHPVOL <= 0x7f))
  #error RHPVOL value not defined
#endif

// setup the sample rate 44.1kHz is the standard rate. 8kHz is good for Arduino
#ifndef SAMPLE_RATE
  #define SAMPLE_RATE 			8
#elif (SAMPLE_RATE == 88)||(SAMPLE_RATE == 44)||(SAMPLE_RATE == 22)||(SAMPLE_RATE == 8)||(SAMPLE_RATE == 2)
  #error SAMPLE_RATE value not properly defined
#endif

// setup variables for Rotary Controller ADC (on Arduino)
#ifndef ADCS
  #define ADCS 					2		// number of ADCs in use. 0, 1, or 2 of the rotary knobs.
#elif (ADCS < 0 ) || ((ADCS >= 3))
  #error ADCS value not properly defined
#endif

#ifndef DECIMATE
  #define DECIMATE 				2		// number of samples for decimation accuracy.  4^DECIMATE samples are taken before reporting ADC value.
#elif (DECIMATE < 1 ) || ((DECIMATE >= 3))
  #error DECIMATE value not properly defined
#endif

/*--------------------------------------------------*/
/*-----Define the Registers and their contents------*/
/*--------------------------------------------------*/

#define LEFT_LINE_IN			0x00
#define LINVOL0					0			// Left Channel Line Input Volume Control
#define LINVOL1					1			// 11111 = +12dB . . 1.5dB steps down to 00000 = -34.5dB
#define LINVOL2					2
#define LINVOL3					3
#define LINVOL4					4
// Bits 5 & 6 must be 0
#define LINMUTE					7			// Left Channel Line Input Mute to ADC
#define LRINBOTH				8			// Enable Simultaneous Load of Left and Right channel Mute

#define RIGHT_LINE_IN			0x01
#define RINVOL0					0			// Right Channel Line Input Volume Control
#define RINVOL1					1			// 11111 = +12dB . . 1.5dB steps down to 00000 = -34.5dB
#define RINVOL2					2
#define RINVOL3					3
#define RINVOL4					4
// Bits 5 & 6 must be 0
#define RINMUTE					7			// Right Channel Line Input Mute to ADC
#define RLINBOTH				8			// Enable Simultaneous Load of Left and Right channel Mute

#define LEFT_HEADPHONE_OUT		0x02
#define LHPVOL0					0			// Left Channel Headphone Output Volume Control
#define LHPVOL1					1			// 1111111 = +6dB, 1dB steps down to 0110000 = -73dB 0000000 to 0101111 = MUTE
#define LHPVOL2					2
#define LHPVOL3					3
#define LHPVOL4					4
#define LHPVOL5					5
#define LHPVOL6					6
#define LZCEN					7			// Left Channel Zero Cross detect Enable
#define LRHPBOTH				8			// Left to Right Channel Headphone Volume, Mute and Zero Cross Data Load Control

#define RIGHT_HEADPHONE_OUT		0x03
#define RHPVOL0					0			// Right Channel Headphone Output Volume Control
#define RHPVOL1					1			// 1111111 = +6dB, 1dB steps down to 0110000 = -73dB 0000000 to 0101111 = MUTE
#define RHPVOL2					2
#define RHPVOL3					3
#define RHPVOL4					4
#define RHPVOL5					5
#define RHPVOL6					6
#define RZCEN					7			// Right Channel Zero Cross detect Enable
#define RLHPBOTH				8			// Right to Left Channel Headphone Volume, Mute and Zero Cross Data Load Control

#define ANALOGUE_PATH_CONTROL	0x04
#define MICBOOST				0			// Microphone Input Level Boost
#define MUTEMIC					1			// Mic Input Mute to ADC
#define INSEL					2			// Microphone or Line Input Select to ADC, 1 = Microphone to ADC, 0 = Line to ADC
#define BYPASS					3			// Bypass Switch
#define DACSEL					4			// DAC Select
#define SIDETONE				5			// Side Tone Switch
#define SIDEATT0				6			// Side Tone Attenuation, 11 = -15dB, 10 = -12dB, 01 = -9dB, 00 = -6dB
#define SIDEATT1				7
// Bit 8 must be 0

#define DIGITAL_PATH_CONTROL	0x05
#define ADCHPD					0			// ADC High Pass Filter Enable
#define DEEMPH0					1			// De-emphasis Control, 11 = 48kHz, 10 = 44.1kHz, 01 = 32kHz, 00 = Disable
#define DEEMPH1					2
#define DACMU					3			// DAC Soft Mute Control
#define HPOR					4			// Store DC offset when High Pass Filter disabled
// Bits 5 through 8 must be 0

#define POWER_DOWN_CONTROL		0x06
#define	LINEINPD				0			// Line Input Power Down
#define MICPD					1			// Microphone Input an Bias Power Down
#define ADCPD					2			// ADC Power Down
#define DACPD					3			// DAC Power Down
#define OUTPD					4			// Outputs Power Down
#define OSCPD					5			// Oscillator Power Down
#define CLKOUTPD				6			// CLKOUT power down
#define POWEROFF				7			// POWEROFF mode
// Bit 8 must be 0

#define DIGITAL_AUDIO_INTERFACE	0x07
#define FORMAT0					0			// Audio Data Format Select, 11 = DSP (SPI) Mode, frame sync + 2 data packed words
#define FORMAT1					1
#define IWL0					2			// Input Audio Data Bit Length Select, 11 = 32 bits, 10 = 24 bits, 01 = 20 bits, 00 = 16 bits
#define IWL1					3
#define LRP						4			// DSP mode A/B select (in DSP mode only)
#define LRSWAP					5			// DAC Left Right Clock Swap
#define MS						6			// Master Slave Mode Control, for SCK, 1 = Enable Master Mode
#define BCLKINV					7			// Bit Clock Invert
// Bit 8 must be 0

#define SAMPLING_CONTROL		0x08
#define USB						0			// Mode Select, 1 = USB mode (250/272fs), 0 = Normal mode (256/384fs)
#define BOSR					1			// Base Over-Sampling Rate, Normal Mode 0 = 256fs 1 = 384fs
#define SR0						2			// ADC and DAC sample rate control
#define SR1						3
#define SR2						4
#define SR3						5
#define CLKIDIV2				6			// Core Clock divider select, 1 = Core Clock is MCLK divided by 2, 0 = Core Clock is MCLK
#define CLKODIV2				7			// CLKOUT divider select, 1 = CLOCKOUT is Core Clock divided by 2, 0 = CLOCKOUT is Core Clock
// Bit 8 must be 0

#define ACTIVE_CONTROL			0x09
#define ACTIVE					0			// Activate Interface
// Bit 1 through 8 must be 0

#define RESET_WM8731			0x0F		// Reset Register
// Writing 00000000 to register resets device


#define _REG(address)			((address) << 1) // Map the Register address into the correct part of the I2C command byte.

#if ADCS == 0
  // do nothing

#elif ADCS == 1
  uint8_t _i = (uint8_t)_BV(2 * DECIMATE);			// 4 ^ DECIMATE for one ADCS
  uint16_t _mod0temp = 0x0000;

#elif ADCS == 2
  uint8_t _i = (uint8_t) _BV(2 * DECIMATE) << 1;	// 4 ^ DECIMATE for two ADCS
  uint16_t _mod0temp = 0x0000;
  uint16_t _mod1temp = 0x0000;

#endif

/* Define the addresses of the AudioCodec device that we want to manage on i2c bus.*/
#define WM8731					0x1A      // device address of WM8731 AudioCodec is 0011010  (see data sheet)
#define ARDUINO					0x32	  // device address of Freetronics / Arduino (just making this up, not special)

/*--------------------------------------------------*/
/*--------------------Globals-----------------------*/
/*--------------------------------------------------*/

// create data variables for audio transfer
// the codec requires stereo data
int16_t left_in  =  0; // in from codec (LINE_IN)
int16_t right_in =  0;
int16_t left_out =  0; // out to codec (HP_OUT)
int16_t right_out = 0;

// create variables for ADC results
// it only has positive values -> unsigned
uint16_t mod0_value = 0;
uint16_t mod1_value = 0;

/*--------------------------------------------------*/
/*---------------Public Functions-------------------*/
/*--------------------------------------------------*/


static void AudioCodec_init(void) {

    uint8_t I2C_command_buf[ 3 ]; // WM8731 needs address (I2C) and 2 bytes command (register+command)

	// initialise I2C master interface, need to do this once only.
	I2C_Master_Initialise((ARDUINO<<I2C_ADR_BITS) | (pdTRUE<<I2C_GEN_BIT));

	I2C_command_buf[0] = (WM8731 << I2C_ADR_BITS) + I2C_WRITE; 			// set device address and write mode

	I2C_command_buf[1] = (uint8_t)_REG(POWER_DOWN_CONTROL);            	// power reduction register
	I2C_command_buf[2] = (uint8_t) 0x00;                      			// turn everything on
	I2C_Master_Start_Transceiver_With_Data( (uint8_t *)&I2C_command_buf, 3 );

	I2C_command_buf[1] = (uint8_t)_REG(DIGITAL_AUDIO_INTERFACE);		// digital data format
	I2C_command_buf[2] = (uint8_t) (_BV(FORMAT1)|_BV(FORMAT0));			// Slave, Mode A, 16bit SPI (DSP)
	I2C_Master_Start_Transceiver_With_Data( (uint8_t *)&I2C_command_buf, 3 );

	I2C_command_buf[1] = (uint8_t)_REG(LEFT_LINE_IN);					// left in setup register
	I2C_command_buf[2] = (uint8_t) LINVOL;
	I2C_Master_Start_Transceiver_With_Data( (uint8_t *)&I2C_command_buf, 3 );

	I2C_command_buf[1] = (uint8_t)_REG(RIGHT_LINE_IN);					// right in setup register
	I2C_command_buf[2] = (uint8_t) RINVOL;
	I2C_Master_Start_Transceiver_With_Data( (uint8_t *)&I2C_command_buf, 3 );

	I2C_command_buf[1] = (uint8_t)_REG(LEFT_HEADPHONE_OUT);				// left headphone out register
	I2C_command_buf[2] = (uint8_t) LHPVOL;
	I2C_Master_Start_Transceiver_With_Data( (uint8_t *)&I2C_command_buf, 3 );

	I2C_command_buf[1] = (uint8_t)_REG(RIGHT_HEADPHONE_OUT);			// right headphone out register
	I2C_command_buf[2] = (uint8_t) RHPVOL;
	I2C_Master_Start_Transceiver_With_Data( (uint8_t *)&I2C_command_buf, 3 );

	I2C_command_buf[1] = (uint8_t)_REG(ANALOGUE_PATH_CONTROL);			// analogue audio pathway configuration
	I2C_command_buf[2] = (uint8_t) (_BV(DACSEL)|_BV(MUTEMIC));
	I2C_Master_Start_Transceiver_With_Data( (uint8_t *)&I2C_command_buf, 3 );

	I2C_command_buf[1] = (uint8_t)_REG(DIGITAL_PATH_CONTROL); 			// digital audio path configuration
	I2C_command_buf[2] = (uint8_t) (_BV(DEEMPH1)|_BV(ADCHPD));
	I2C_Master_Start_Transceiver_With_Data( (uint8_t *)&I2C_command_buf, 3 );

	I2C_command_buf[1] = (uint8_t)_REG(SAMPLING_CONTROL);				// clock configuration
	#if SAMPLE_RATE == 88
	I2C_command_buf[2] = (uint8_t) (_BV(CLKODIV2)|_BV(SR3)|_BV(SR2)|_BV(SR1)|_BV(SR0));
	#elif SAMPLE_RATE == 44
	I2C_command_buf[2] = (uint8_t) (_BV(CLKODIV2)|_BV(SR3));
	#elif SAMPLE_RATE == 22
	I2C_command_buf[2] = (uint8_t) (_BV(CLKODIV2)|_BV(CLKIDIV2)|_BV(SR3));
	#elif SAMPLE_RATE == 8
	I2C_command_buf[2] = (uint8_t) (_BV(CLKODIV2)|_BV(SR3)|_BV(SR1)|_BV(SR0));
	#elif SAMPLE_RATE == 2
	I2C_command_buf[2] = (uint8_t) (_BV(CLKODIV2)|_BV(CLKIDIV2)|_BV(SR1)|_BV(SR0)|_BV(BOSR));
	#endif
	I2C_Master_Start_Transceiver_With_Data( (uint8_t *)&I2C_command_buf, 3 );

	I2C_command_buf[1] = (uint8_t)_REG(ACTIVE_CONTROL);					// WM8731 enable
	I2C_command_buf[2] = (uint8_t) _BV(ACTIVE);
	I2C_Master_Start_Transceiver_With_Data( (uint8_t *)&I2C_command_buf, 3 );

}

static void AudioCodec_ADC_init(void) __attribute__ ((flatten));

static void AudioCodec_ADC_init(void)
{
	// setup ADCs
#if ADCS == 0
	DIDR0  = _BV(ADC1D)|_BV(ADC0D); // turn off digital inputs for ADC0 and ADC1

#elif (ADCS == 1) || (ADCS == 2)
	ADMUX  = _BV(REFS0); // start with ADC0 - AVCC with external capacitor at AREF pin
	ADCSRA = _BV(ADEN)|_BV(ADSC)|_BV(ADATE)|_BV(ADPS2)|_BV(ADPS1)|_BV(ADPS0); // ADC enable, auto trigger, ck/128
	ADCSRB = 0x00; // free running mode
	DIDR0  = _BV(ADC5D)|_BV(ADC4D)|_BV(ADC1D)|_BV(ADC0D); // turn off digital inputs for pins ADC0 and ADC1, and SCL / SDA on A4 and A5

#endif
}

// adc sampling routine
#if ADCS == 0
static void AudioCodec_ADC() __attribute__((flatten));
static void AudioCodec_ADC()
{
    // do nothing
}

#elif ADCS == 1
static void AudioCodec_ADC(uint16_t* _mod0value) __attribute__((hot, flatten));
static void AudioCodec_ADC(uint16_t* _mod0value)
{
    if (ADCSRA & (1 << ADIF))				// check if sample ready
    {
      _mod0temp += ADCW;					// fetch ADCL first to freeze sample is done by the compiler
      ADCSRA = 0xf7;						// reset the interrupt flag

      if (--_i == 0)						// check if enough samples have been collected
      {
		_mod0temp >>= DECIMATE;				// Decimate the summed samples (to get better accuracy), see AVR8003.doc
		*_mod0value = _mod0temp;			// move temp value
		_mod0temp = 0x0000;					// reset temp value
		_i = _BV(2 * DECIMATE);				// reset loop counter
      }
   }
}

#elif ADCS == 2
static void AudioCodec_ADC(uint16_t * _mod0value, uint16_t * _mod1value) __attribute__((hot, flatten));
static void AudioCodec_ADC(uint16_t * _mod0value, uint16_t * _mod1value)
{
    if (ADCSRA & (1 << ADIF))				// check if sample ready
    {
      if (_i >= _BV(2 * DECIMATE))			// sample ADC0
      {
        _mod0temp += ADCW; 					// fetch ADCL first to freeze sample is done by the compiler

        if (_i == _BV(2 * DECIMATE))		// decimate and return the ADC0 result.
        {
        	_mod0temp >>= DECIMATE;			// Decimate the summed samples (to get better accuracy), see AVR8003.doc
            *_mod0value = _mod0temp;		// move temp value to return
			_mod0temp = 0x0000;				// reset temp value
			ADMUX = 0x41;					// switch to ADC1
        }
        ADCSRA = 0xf7;						// reset the interrupt flag
      }

      else if (_i < (_BV(2 * DECIMATE)) )	// sample ADC1
      {
        _mod1temp += ADCW;					// fetch ADCL first to freeze sample is done by the compiler

		if (_i == 0)						// decimate and return the ADC1 result.
		{
			_mod1temp >>= DECIMATE;			// Decimate the summed samples (to get better accuracy), see AVR8003.doc
			*_mod1value = _mod1temp;		// move temp value to return
			_mod1temp = 0x0000;				// reset temp value
			ADMUX = 0x40;					// switch to ADC0

			_i = _BV(2 * DECIMATE) << 1;	// reset loop counter
		}
        ADCSRA = 0xf7;						// reset the interrupt flag
      }

    --_i; // decrement the sample counter
   }
}
#endif

static void AudioCodec_SPI_init(void) __attribute__((flatten));
static void AudioCodec_SPI_init(void)
{
	uint8_t tmp __attribute__ ((unused));
    // setup spi peripheral
	spiSetBitOrder(SPI_MSBFIRST);
	spiSetDataMode(SPI_MODE3);
	spiSetClockDivider(SPI_CLOCK_DIV2);

	// Set up the SPI bus pins
	SPI_PORT_DIR |= SPI_BIT_MOSI | SPI_BIT_SCK | SPI_BIT_SS;
	SPI_PORT_DIR &= ~SPI_BIT_MISO;
	SPI_PORT |= SPI_BIT_MISO;
	SPI_PORT &= ~SPI_BIT_SS; // set the slave select low. Unusual, but what this interface needs.

	// Set the control register to turn on the SPI interface as Master.
	SPCR |= _BV(MSTR) | _BV(SPE);

	// Reading SPI Status Register & SPI Data Register
	// has side effect of zeroing out both
	tmp = SPSR;
	tmp = SPDR;

	DDRD |= _BV(DDD6);            // Disable the Audio Shield SPI buffer, just a precaution.
	PORTD &= ~_BV(PORTD6);
  }

static void AudioCodec_Timer1_init(void) __attribute__((flatten));
static void AudioCodec_Timer1_init(void)
{
	portENTER_CRITICAL(); // turn off interrupts

	// setup Timer1 for codec clock division
	TCCR1A = 0x00; // set to  CTC Mode.
	TCCR1B = _BV(WGM12) | _BV(CS12) | _BV(CS11) | _BV(CS10); // CTC Mode. TOP = OCR1A. External clock source on T1 pin. Clock on rising edge.
	TCCR1C = 0x00; // not used
	TCNT1 =  0x0000; // clear the counter, high byte first for 16bit writes. gcc compiler knows how to handle this.

	#if SAMPLE_RATE == 88
	OCR1A = 0x003f; // set the counter top to be 0x003F (63)

	#elif (SAMPLE_RATE == 44) || (SAMPLE_RATE == 22)
	OCR1A = 0x007f; // set the counter top to be 0x007F (127)

	#elif SAMPLE_RATE == 8
	OCR1A = 0x02bf; // set the counter top to be 0x02BF (703)

	#elif SAMPLE_RATE == 2
	OCR1A = 0x047f; // set the counter top to be 0x047F (1151)

	#endif

	TIMSK1 = _BV(OCIE1A); // turn on compare match interrupt

	// turn off all enabled interrupts (delay and wire) -- No we can't do that. Sorry.
	portEXIT_CRITICAL(); // turn on interrupts
}


/*--------------------------------------------------*/
/*-----------Time Critical Functions----------------*/
/*--------------------------------------------------*/

// WM8731 data transfer function
static void AudioCodec_data(int16_t* _lin, int16_t* _rin, int16_t _lout, int16_t _rout) __attribute__((hot, flatten));
static void AudioCodec_data(int16_t* _lin, int16_t* _rin, int16_t _lout, int16_t _rout)
{
	register int16_t _out_temp;

	PORTD |= _BV(PORTD6);			// Enable the Audio Shield buffer.

	_out_temp = _lout;
	SPI_PORT |= SPI_BIT_SS;	// toggle ss pin high
	asm volatile ("out %0, %B1" : : "I" (_SFR_IO_ADDR(SPDR)), "r" (_out_temp) );
	SPI_PORT &= ~SPI_BIT_SS;  // toggle ss pin low
	while(!(SPSR & (1<<SPIF))); // wait for data transfer to complete

	asm volatile ("out %0, %A1" : : "I" (_SFR_IO_ADDR(SPDR)), "r" (_out_temp) );
	asm volatile ("in r3, %0" : : "I" (_SFR_IO_ADDR(SPDR)) : "r3" );

	_out_temp = _rout;
	while(!(SPSR & (1<<SPIF))); // wait for data transfer to complete

	asm volatile ("out %0, %B1" : : "I" (_SFR_IO_ADDR(SPDR)), "r" (_out_temp) );
	asm volatile ("in r2, %0" : : "I" (_SFR_IO_ADDR(SPDR)) : "r2" );
	asm volatile ("movw %0, r2" : "=r" (*_lin) : : "r2", "r3" );
	while(!(SPSR & (1<<SPIF))); // wait for data transfer to complete

	asm volatile ("out %0, %A1" : : "I" (_SFR_IO_ADDR(SPDR)), "r" (_out_temp) );
	asm volatile ("in r3, %0" : : "I" (_SFR_IO_ADDR(SPDR)) : "r3" );
	while(!(SPSR & (1<<SPIF))); // wait for data transfer to complete

	asm volatile ("in r2, %0" : : "I" (_SFR_IO_ADDR(SPDR)) : "r2" );
	asm volatile ("movw %0, r2" : "=r" (*_rin) : : "r2", "r3" );

	PORTD &= ~_BV(PORTD6);           // Disable Audio Shield buffer
}

/*--------------------------------------------------------*/

void AudioCodec_dsp(void) __attribute__((hot, flatten));
		// prototype for the DSP function to be implemented.
		// needs to at least provide left_out and right_out
		// within Timer1 interrupt routine - time critical I/O. Keep it short and punchy!


/*--------------------------------------------------*/
/*-------Interrupt (Loop at Sample Rate)------------*/
/*--------------------------------------------------*/


ISR(TIMER1_COMPA_vect)
{
	// WM8731 data transfer routine
	// move data from and to the WM8731 - done first for regularity (reduced jitter).
	// &'s are necessary on data_in variables
	AudioCodec_data(&left_in, &right_in, left_out, right_out);

	// audio processing routine - do whatever processing on input is required - prepare output for next time.
	AudioCodec_dsp();

	// adc sampling routine
	// sampling the potentiometers (no sound here) - only needed if the pots are used to modify sound
	// & is required before ADC variables
#if ADCS == 0
	AudioCodec_ADC();
#elif ADCS == 1
	AudioCodec_ADC(&mod0_value);
#elif ADCS == 2
	AudioCodec_ADC(&mod0_value, &mod1_value);
#endif

//	xxx end mark - check for end of interrupt - for debugging only - delete when not wanted
	PORTD |= _BV(PORTD6);           // Ping Audio Shield buffer line.
	PORTD &= ~_BV(PORTD6);
}

#endif // end include guard

