/*
 * Derived from Ben Schmidel, August 11, 2009.
 * Copyright (c) 2009 Pololu Corporation. For more information, see
 *
 *   http://www.pololu.com
 *   http://forum.pololu.com
 *   http://www.pololu.com/docs/0J18
 *
 * You may freely modify and share this code, as long as you keep this
 * notice intact (including the two links above).  Licensed under the
 * Creative Commons BY-SA 3.0 license:
 *
 *   http://creativecommons.org/licenses/by-sa/3.0/
 *
 * Disclaimer: To the extent permitted by law, Pololu provides this work
 * without any warranty.  It might be defective, in which case you agree
 * to be responsible for all resulting costs and damages.
 */


#include <digitalAnalog.h>

/*--------------- declarations here ----------------------------------*/

struct IOStructure
{
	// if these aren't volatile, the compiler sometimes incorrectly optimises away operations involving these registers:
	volatile uint8_t* pinRegister;
	volatile uint8_t* portRegister;
	volatile uint8_t* ddrRegister;
	uint8_t bitmask;
};


inline void setDigitalOutput(uint8_t pin, uint8_t outputState);
inline void setDigitalInput(uint8_t pin, uint8_t inputState);
inline uint8_t isDigitalInputHigh(uint8_t pin);

inline static void getIORegisters(struct IOStructure* io, uint8_t pin);
inline static void setDataDirection(struct IOStructure* ioPin, uint8_t val);
inline static void setOutputValue(struct IOStructure* ioPin, uint8_t val);
inline static uint8_t getInputValue(struct IOStructure* ioPin);

inline void setAnalogMode(uint8_t mode);
inline uint8_t getAnalogMode();
inline uint8_t analogIsConverting();

void startAnalogConversion( uint8_t channel, uint8_t use_internal_reference);
inline uint16_t analogConversionResult();



/*--------------- inline functions here ----------------------------------*/


// gets a structure with pointers to the three digital I/O registers associated
// with the specified pin (DDR, PORT, and PIN) along with a bitmask with a
// 1 in the position of the specified pin and 0s everywhere else.
inline static void getIORegisters(struct IOStructure* io, uint8_t pin)
{
	io->pinRegister = 0;
	io->portRegister = 0;
	io->ddrRegister = 0;
	io->bitmask = 0;

#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
// I'm not sure why Ben went to the effort of doing this out of order for Orangutan SVP.
// Doesn't seem to match any physical reasoning...
// So here' I'm keeping it simple.
// If there is a pin that isn't connected... Well I guess it won't do anything.

	if (pin < 8)			// pin 0 = PA0, ..., 7 = PA7
	{
		io->pinRegister = (uint8_t*)&PINA;
		io->portRegister = (uint8_t*)&PORTA;
		io->ddrRegister = (uint8_t*)&DDRA;
		io->bitmask = 1 << (pin - 0);
	}

	else if (pin < 16)		// pin 8 = PB0, ..., 15 = PB7
	{
		io->pinRegister = (uint8_t*)&PINB;
		io->portRegister = (uint8_t*)&PORTB;
		io->ddrRegister = (uint8_t*)&DDRB;
		io->bitmask = 1 << (pin - 8);
	}
	else if (pin < 24)		// pin 16 = PC0, ..., 23 = PC7
	{
		io->pinRegister = (uint8_t*)&PINC;
		io->portRegister = (uint8_t*)&PORTC;
		io->ddrRegister = (uint8_t*)&DDRC;
		io->bitmask = 1 << (pin - 16);
	}
	else if (pin < 32)		// pin 24 = PD0, ..., 31 = PD7
	{
		io->pinRegister = (uint8_t*)&PIND;
		io->portRegister = (uint8_t*)&PORTD;
		io->ddrRegister = (uint8_t*)&DDRD;
		io->bitmask = 1 << (pin - 24);
	}
	else if (pin < 40)		// pin 32 = PE0, ..., 39 = PE7
	{
		io->pinRegister = (uint8_t*)&PINE;
		io->portRegister = (uint8_t*)&PORTE;
		io->ddrRegister = (uint8_t*)&DDRE;
		io->bitmask = 1 << (pin - 32);
	}
	else if (pin < 48)		// pin 40 = PF0, ..., 47 = PF7
	{
		io->pinRegister = (uint8_t*)&PINF;
		io->portRegister = (uint8_t*)&PORTF;
		io->ddrRegister = (uint8_t*)&DDRF;
		io->bitmask = 1 << (pin - 40);
	}
	else if (pin < 54)		// pin 48 = PG0, ..., 53 = PG5 PG6 & PG7 don't exist.
	{
		io->pinRegister = (uint8_t*)&PING;
		io->portRegister = (uint8_t*)&PORTG;
		io->ddrRegister = (uint8_t*)&DDRG;
		io->bitmask = 1 << (pin - 48);
	}
	else if (pin < 56) {}	// Do nothing. This pins don't exist.

	else if (pin < 64)		// pin 56 = PH0, ..., 63 = PH7
	{
		io->pinRegister = (uint8_t*)&PINH;
		io->portRegister = (uint8_t*)&PORTH;
		io->ddrRegister = (uint8_t*)&DDRH;
		io->bitmask = 1 << (pin - 56);
	}
	else if (pin < 72)		// pin 64 = PJ0, ..., 71 = PJ7
	{
		io->pinRegister = (uint8_t*)&PINJ;
		io->portRegister = (uint8_t*)&PORTJ;
		io->ddrRegister = (uint8_t*)&DDRJ;
		io->bitmask = 1 << (pin - 64);
	}
	else if (pin < 80)		// pin 72 = PK0, ..., 79 = PK7
	{
		io->pinRegister = (uint8_t*)&PINK;
		io->portRegister = (uint8_t*)&PORTK;
		io->ddrRegister = (uint8_t*)&DDRK;
		io->bitmask = 1 << (pin - 72);
	}
	else if (pin < 88)		// pin 80 = PL0, ..., 87 = PL7
	{
		io->pinRegister = (uint8_t*)&PINL;
		io->portRegister = (uint8_t*)&PORTL;
		io->ddrRegister = (uint8_t*)&DDRL;
		io->bitmask = 1 << (pin - 80);
	}

#elif defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__)  || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__)

	if (pin < 8)			// pin 0 = PD0, ..., 7 = PD7
	{
		io->pinRegister = (uint8_t*)&PIND;
		io->portRegister = (uint8_t*)&PORTD;
		io->ddrRegister = (uint8_t*)&DDRD;
		io->bitmask = 1 << (pin - 0);
	}

	else if (pin < 16)		// pin 8 = PB0, ..., 15 = PB7
	{
		io->pinRegister = (uint8_t*)&PINB;
		io->portRegister = (uint8_t*)&PORTB;
		io->ddrRegister = (uint8_t*)&DDRB;
		io->bitmask = 1 << (pin - 8);
	}
	else if (pin < 24)		// pin 16 = PC0, ..., 23 = PC7
	{
		io->pinRegister = (uint8_t*)&PINC;
		io->portRegister = (uint8_t*)&PORTC;
		io->ddrRegister = (uint8_t*)&DDRC;
		io->bitmask = 1 << (pin - 16);
	}
	else if (pin < 32)		// pin 24 = PA0, ..., 31 = PA7
	{
		io->pinRegister = (uint8_t*)&PINA;
		io->portRegister = (uint8_t*)&PORTA;
		io->ddrRegister = (uint8_t*)&DDRA;
		io->bitmask = 1 << (24 - pin);
	}

#else // Arduino Compatible notation, for whatever that is worth.

	if (pin < 8)			// pin 0 = PD0, ..., 7 = PD7
	{
		io->pinRegister = (uint8_t*)&PIND;
		io->portRegister = (uint8_t*)&PORTD;
		io->ddrRegister = (uint8_t*)&DDRD;
		io->bitmask = 1 << (pin - 0);
	}

	else if (pin < 14)		// pin 8 = PB0, ..., 13 = PB5 (PB6 and PB7 reserved for external clock)
	{
		io->pinRegister = (uint8_t*)&PINB;
		io->portRegister = (uint8_t*)&PORTB;
		io->ddrRegister = (uint8_t*)&DDRB;
		io->bitmask = 1 << (pin - 8);
	}
	else if (pin < 21)		// pin 14 = PC0, ..., 19 = PC5 (PC6 is reset, PC7 doesn't exist)
	{
		io->pinRegister = (uint8_t*)&PINC;
		io->portRegister = (uint8_t*)&PORTC;
		io->ddrRegister = (uint8_t*)&DDRC;
		io->bitmask = 1 << (pin - 14);
	}
#endif
}


// low-level method for setting the data direction (i.e. input or output) of an pin or set of pins
// described by an IOStruct pointer.
inline static void setDataDirection(struct IOStructure* ioPin, uint8_t val)
{
	if (val)
		*(ioPin->ddrRegister) |= ioPin->bitmask;
	else
		*(ioPin->ddrRegister) &= ~ioPin->bitmask;
}


// low-level method for setting the PORT register value of an pin or set of pins
// described by an IOStruct pointer.  If the pin is an input, this lets you choose between
// setting it as high-impedance (val = 0) or enabling the internal pull-up (val = 1).  If the pin is an
// output, this lets you choose between driving low (val = 0) and driving high (val = 1).
// NOTE: if val is 0xFF (255), this method will toggle the PORT register pin(s).
inline static void setOutputValue(struct IOStructure* ioPin, uint8_t val)
{
	if (val == 0xFF)
		*(ioPin->portRegister) ^= ioPin->bitmask;
	else if (val)
		*(ioPin->portRegister) |= ioPin->bitmask;
	else
		*(ioPin->portRegister) &= ~ioPin->bitmask;
}


// low-level method for reading the value of the PIN register for an pin or set of pins
// described by an IOStruct pointer.
inline static uint8_t getInputValue(struct IOStructure* ioPin)
{
	return *(ioPin->pinRegister) & ioPin->bitmask;
}


/*--------------------------------------------------------------------*/

// high-level method for setting the specified pin as an output with the specified output state.
// An outputState value of 0 will cause the pin to drive low; a value of 1 will cause the pin to
// drive high.  A value of 0xFF (255) will toggle the output state of the pin (i.e. high -> low and
// low -> high).
inline void setDigitalOutput(uint8_t pin, uint8_t outputState)
{
	struct IOStructure registers;
	getIORegisters(&registers, pin);
	setOutputValue(&registers, outputState);
	setDataDirection(&registers, 1);
}


// high-level method for setting the specified pin as an input with the specified input state.
// An inputState value of 0 will cause the pin to be a high-impedance input; a value of 1 will enable the
// pin's internal pull-up resistor, which weakly pulls it to Vcc.  A value of 0xFF (255) will toggle the
// input state.
inline void setDigitalInput(uint8_t pin, uint8_t inputState)
{
	struct IOStructure registers;
	getIORegisters(&registers, pin);
	setDataDirection(&registers, 0);
	setOutputValue(&registers, inputState);
}


// high-level method for reading the input value of the specified pin.  If the voltage on the pin is low,
// this method will return 0.  Otherwise, it will return a non-zero result that depends on the value of
// the pin.
inline uint8_t isDigitalInputHigh(uint8_t pin)
{
	struct IOStructure registers;
	getIORegisters(&registers, pin);
	return getInputValue(&registers);
}


/*--------------------------------------------------------------------*/


// set the ADC to run in either 8-bit mode (MODE_8_BIT) or
// 10-bit mode (MODE_10_BIT)
inline void setAnalogMode(uint8_t mode)
{
	if (mode == MODE_10_BIT)
		ADMUX &= ~(1 << ADLAR);	// right-adjust result (ADC has result)
	else
		ADMUX |= 1 << ADLAR;		// left-adjust result (ADCH has result)
}

// returns 0 if in 10-bit mode, otherwise returns non-zero.  The return
// value of this method can be directly compared against the macros
// MODE_8_BIT and MODE_10_BIT:
// For example: if (getMode() == MODE_8_BIT) ...

inline uint8_t getAnalogMode()
{
	return (ADMUX >> ADLAR) & 1;
}

// returns 1 if the ADC is in the middle of an conversion, otherwise
// returns 0
inline uint8_t analogIsConverting()
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

void startAnalogConversion(uint8_t channel, uint8_t use_internal_reference)
{
	// Channel numbers greater than 31 are invalid.
	if (channel > 31)
	{
		return;
	}

//	ADCSRA = 0x86;		// We're going to be running faster than Pololu suggests. Good for 8 bit conversions.
	ADCSRA = 0x87;		// bit 7 set: ADC enabled
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

	tempADMUX |= 1 << 6;
	if(use_internal_reference)	// Note: internal reference should NOT be used on devices
	{							//  where AREF is connected to an external voltage!
		// use the internal voltage reference
		tempADMUX |= 1 << 7;		// 1.1 V on ATmega48/168/328; 2.56 V on ATmega324/644/1284
	}
	else
	{
		// use AVCC as a reference
		tempADMUX &= ~(1 << 7);
	}

	tempADMUX &= ~0x1F;		 // clear channel selection bits of ADMUX
	tempADMUX |= channel;    // we only get this far if channel is less than 32
	ADMUX = tempADMUX;
	ADCSRA |= 1 << ADSC; // start the conversion
}

// returns the result of the previous ADC conversion.
inline uint16_t analogConversionResult()
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
