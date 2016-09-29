// DAC.h

#ifndef GadgetShield_h // include guard
#define GadgetShield_h

#include <avr/io.h>

#include "spi.h"

/*--------------------------------------------------*/
/*------------Often Configured Parameters-----------*/
/*--------------------------------------------------*/

#define DELAY			6000
#define DECIMATE		1

// TODO Define the audio function that is desired by uncommenting one of below...
#define SINE
//#define MUSIC
//#define VCO
//#define MIC
//#define WALKIE_TALKIE


#if defined(SINE)
#define ADCS 	0		// number of ADCs in use. 0, 1, or 2 of the linear potentiometers on a Danger Shield.


#elif defined(MUSIC)
#define ADCS 	0		// number of ADCs in use. 0, 1, or 2 of the linear potentiometers on a Danger Shield.


#elif defined(VCO)
#define ADCS 	2		// number of ADCs in use. 0, 1, or 2 of the linear potentiometers on a Danger Shield.


#elif defined(MIC) || defined (WALKIE_TALKIE)
#define ADCS    1		// number of ADCs in use. 1 Microphone on ADC7.

#endif

// setup variables for Linear Controller ADC (on Arduino)
#ifndef ADCS
  #define ADCS 					2		// number of ADCs in use. 0, 1, 2, or 3 of the linear potentiometers on a Danger Shield.
#elif (ADCS < 0 ) || ((ADCS >= 4))
  #error ADCS value not properly defined
#endif

#ifndef DECIMATE
  #define DECIMATE 				2		// number of samples for decimation accuracy.  4^DECIMATE samples are taken before reporting ADC value.
#elif (DECIMATE < 1 ) || ((DECIMATE >= 3))
  #error DECIMATE value not properly defined
#endif


/*--------------------------------------------------*/
/*--------------------Globals-----------------------*/
/*--------------------------------------------------*/

#if ADCS == 0
  // do nothing

#elif ADCS == 1

#if defined(MIC) || defined (WALKIE_TALKIE)

  uint8_t * delayDataPtr;
  ringBuffer_t delayBuffer;

  DAC_value_t mod7_value;

#else
  uint8_t _i = (uint8_t)_BV(2 * DECIMATE);			// 4 ^ DECIMATE for one ADCS
  uint16_t _mod0temp;

  uint16_t mod0_value;
#endif

#elif ADCS == 2
  uint8_t _i = (uint8_t) _BV(2 * DECIMATE) * 2;	// 4 ^ DECIMATE for two ADCS
  uint16_t _mod0temp;
  uint16_t _mod1temp;

  uint16_t mod0value;
  uint16_t mod1value;

#elif ADCS == 3
  uint8_t _i = (uint8_t) _BV(2 * DECIMATE) * 3;	// 4 ^ DECIMATE for two ADCS
  uint16_t _mod0temp;
  uint16_t _mod1temp;
  uint16_t _mod2temp;

  uint16_t mod0value;
  uint16_t mod1value;
  uint16_t mod2value;

#endif

/*--------------------------------------------------*/
/*---------------Public Functions-------------------*/
/*--------------------------------------------------*/

void AudioCodec_ADC_init(void)
{
	 // turn off digital inputs for pins ADC0, ADC1, ADC2, ADC6, ADC7
	DIDR0 = _BV(ADC7D)|_BV(ADC6D)|_BV(ADC5D)|_BV(ADC4D)|_BV(ADC3D)|_BV(ADC2D)|_BV(ADC1D)|_BV(ADC0D); // turn off digital inputs
	DIDR1 = _BV(AIN1D)|_BV(AIN0D);

	// setup ADCs
#if (ADCS == 1) || (ADCS == 2) || (ADCS == 3)

#if defined(MIC) || defined (WALKIE_TALKIE)
	ADMUX  = _BV(REFS1)|_BV(REFS0)|_BV(ADLAR)|_BV(MUX2)|_BV(MUX1)|_BV(MUX0); // 2.56V reference with external capacitor at AREF pin - left justify - start with MIC input ADC7
	ADCSRA = _BV(ADEN)|_BV(ADSC)|_BV(ADATE)|_BV(ADPS2)|_BV(ADPS1)|_BV(ADPS0); // ADC enable, auto trigger, ck/128 = 192kHz
	ADCSRB =  0x00;			// free running mode

#else
	ADMUX  = _BV(REFS0); // start with ADC0 - AVCC with external capacitor at AREF pin
	ADCSRA = _BV(ADEN)|_BV(ADSC)|_BV(ADATE)|_BV(ADPS2)|_BV(ADPS1)|_BV(ADPS0); // ADC enable, auto trigger, ck/128
	ADCSRB = 0x00; // free running mode

#endif

#endif
}

// adc sampling routine
#if ADCS == 0
static void AudioCodec_ADC(void) __attribute__((flatten));
static void AudioCodec_ADC()
{
    // do nothing
}

#elif ADCS == 1
static void AudioCodec_ADC(uint16_t* _modvalue) __attribute__((flatten));
static void AudioCodec_ADC(uint16_t* _modvalue)
{

#if defined(MIC) || defined (WALKIE_TALKIE)
	if (ADCSRA & _BV(ADIF))				// check if sample ready
	{
    	*_modvalue = ADCW;					// fetch ADCL first to freeze sample, then ADCH. It is done by the compiler.
    	ADCSRA |= _BV(ADIF);				// reset the interrupt flag
	}

#else
    if (ADCSRA & _BV(ADIF))				// check if sample ready
    {
      _mod0temp += ADCW;					// fetch ADCL first to freeze sample is done by the compiler
      ADCSRA |= _BV(ADIF);					// reset the interrupt flag

      if (--_i == 0)						// check if enough samples have been collected
      {
		_mod0temp >>= DECIMATE;				// Decimate the summed samples (to get better accuracy), see AVR8003.doc
		*_modvalue = _mod0temp;				// move temp value
		_mod0temp = 0x0000;					// reset temp value
		_i = _BV(2 * DECIMATE);				// reset loop counter
      }
   }
#endif

}

#elif ADCS == 2
static void AudioCodec_ADC(uint16_t * _mod0value, uint16_t * _mod1value) __attribute__((flatten));
static void AudioCodec_ADC(uint16_t * _mod0value, uint16_t * _mod1value)
{
    if (ADCSRA & _BV(ADIF))				// check if sample ready
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

#elif ADCS == 3
static void AudioCodec_ADC(uint16_t * _mod0value, uint16_t * _mod1value, uint16_t * _mod2value) __attribute__((flatten));
static void AudioCodec_ADC(uint16_t * _mod0value, uint16_t * _mod1value, uint16_t * _mod2value)
{ //  FIXME haven't implemented for third linear potentiometer.
    if (ADCSRA & _BV(ADIF))				// check if sample ready
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

#endif // GadgetShield_h end include guard
