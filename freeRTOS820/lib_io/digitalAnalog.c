
/* Scheduler include files. */
#include "FreeRTOS.h"
#include "digitalAnalog.h"


/*--------------------------------------------------------------------*/


// set the ADC to run in either 8-bit mode (MODE_8_BIT) or
// 10-bit mode (MODE_10_BIT)
inline void setAnalogMode(uint8_t mode)
{
	if (mode == MODE_10_BIT)
		ADMUX &= ~(1 << ADLAR);	// right-adjust result (ADC has result)
	else
		ADMUX |= 1 << ADLAR;	// left-adjust result (ADCH has result)
}

// returns 0 if in 10-bit mode, otherwise returns non-zero.  The return
// value of this method can be directly compared against the macros
// MODE_8_BIT and MODE_10_BIT:
// For example: if (getMode() == MODE_8_BIT) ...

inline uint8_t getAnalogMode(void)
{
	return (ADMUX >> ADLAR) & 1;
}

// returns 1 if the ADC is in the middle of an conversion, otherwise
// returns 0
inline uint8_t analogIsConverting(void)
{
	return (ADCSRA >> ADSC) & 1;
}

// the following method can be used to initiate an ADC conversion
// that runs in the background, allowing the CPU to perform other tasks
// while the conversion is in progress.  The procedure is to start a
// conversion on a channel with startConversion(channel), and then
// poll isConverting in your main loop.  Once isConverting() returns
// a zero, the result can be obtained through a call to conversionResult().
// NOTE: Some Orangutans and 3pis have their AREF pin connected directly to VCC.
//  On these Orangutans, you must not use the internal voltage reference as
//  doing so will short the internal reference voltage to VCC and could damage
//  the AVR.  It is safe to use the internal reference voltage on the
//  Orangutan SVP.

inline void startAnalogConversion(uint8_t channel, uint8_t use_internal_reference)
{
	// Channel numbers greater than 15 on 328p or 1284p or 31 on 2560 are invalid.
	if (channel > 0x1F)
	{
		return;
	}

	ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
//	ADCSRA = 0x86;		// We're going to be running faster than Pololu suggests. Good for 8 bit conversions.
						// bit 7 set: ADC enabled
						// bit 6 clear: don't start conversion
						// bit 5 clear: disable autotrigger
						// bit 4: ADC interrupt flag
						// bit 3 clear: disable ADC interrupt
						// bits 0-2 set: ADC clock prescaler is 128
						//  128 prescaler required for 10-bit resolution when FCPU = 20 MHz

	// NOTE: it is important to make changes to a temporary variable and then set the ADMUX
	// register in a single atomic operation rather than incrementally changing bits of ADMUX.
	// Specifically, setting the ADC channel by first clearing the channel bits of ADMUX and
	// then setting the ones corresponding to the desired channel briefly connects the ADC
	// to channel 0, which can affect the ADC charge capacitor.  For example, if you have a
	// high output impedance voltage on channel 1 and a low output impedance voltage on channel
	// 0, the voltage on channel 0 be briefly applied to the ADC capacitor before every conversion,
	// which could prevent the capacitor from settling to the voltage on channel 1, even over
	// many reads.
	uint8_t tempADMUX = ADMUX;

	tempADMUX |= _BV(REFS0);		// Use external capacitor on ARef
	if(use_internal_reference)		// Note: internal reference should NOT be used on devices
	{								//  where AREF is connected to an external voltage!
		// use the internal voltage reference
		tempADMUX |= _BV(REFS1);	// Internal reference: 1.1 V on ATmega48/168/328; 2.56 V on ATmega324/644/1284
	}
	else
	{
		// use AVcc as a reference
		tempADMUX &= ~_BV(REFS1);	// External reference: on AVcc
	}

	tempADMUX &= ~0x1F;		 // clear channel selection bits of ADMUX
	tempADMUX |= channel;    // we only get this far if channel is less than 32
	ADMUX = tempADMUX;
	ADCSRA |= _BV(ADSC); // start the conversion
}

// returns the result of the previous ADC conversion.
inline uint16_t analogConversionResult(void)
{
	if (getAnalogMode())				// if left-adjusted (i.e. 8-bit mode)
	{
		return (uint16_t) ADCH;			// 8-bit result
	}
	else
	{
		return ADC;				// 10-bit result
	}
}
