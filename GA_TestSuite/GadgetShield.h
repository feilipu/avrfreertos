// DAC.h

#ifndef GadgetShield_h // include guard
#define GadgetShield_h


#include <avr/io.h>

#include "../lib_eefs/eefs_ringBuffer.h"

/*--------------------------------------------------*/
/*------------Often Configured Parameters-----------*/
/*--------------------------------------------------*/

#define DELAY			128000

#define USE_EEFS

/*--------------------------------------------------*/
/*--------------------Globals-----------------------*/
/*--------------------------------------------------*/

#if defined(USE_EEFS)
eefs_ringBuffer_t eefs_delayBuffer;
#else

uint8_t * delayDataPtr;
ringBuffer_t delayBuffer;
#endif

DAC_value_t mod7_value;

/*--------------------------------------------------*/
/*---------------Public Functions-------------------*/
/*--------------------------------------------------*/

void AudioCodec_ADC_init(void)
{
	// setup ADCs
	ADMUX  = _BV(REFS1)|_BV(REFS0)|_BV(ADLAR)|_BV(MUX2)|_BV(MUX1)|_BV(MUX0); // xxx 2.56V reference with external capacitor at AREF pin - left justify - start with MIC input ADC7
	ADCSRA = _BV(ADEN)|_BV(ADSC)|_BV(ADATE)|_BV(ADPS2)|_BV(ADPS1)|_BV(ADPS0); // ADC enable, auto trigger, ck/128 = 192kHz
	ADCSRB =  0x00;			// free running mode
	DIDR0  = _BV(ADC7D)|_BV(ADC6D)|_BV(ADC2D)|_BV(ADC1D)|_BV(ADC0D);	// turn off digital input for pin ADC6 Line and ADC7 Mic input (and ADC2, ADC1, & ADC0)
}

// adc sampling routine
static void AudioCodec_ADC(uint16_t* _modvalue) __attribute__((flatten));

static void AudioCodec_ADC(uint16_t* _modvalue)
{
	if (ADCSRA & _BV(ADIF))				// check if sample ready
	{
    	*_modvalue = ADCW;					// fetch ADCL first to freeze sample, then ADCH. It is done by the compiler.
    	ADCSRA |= _BV(ADIF);				// reset the interrupt flag
	}
}


#endif // GadgetShield_h end include guard
