/*
  OrangutanDigital.h - Library for using the digital I/O lines on the
	Orangutan LV, SV, SVP, X2, Baby Orangutan B, or 3pi robot.  The code
	is all inline, which lets it compile to very small, fast, efficient
	assembly code if you use constants as your inputs.  For example,
	the line:

		setOutput(3, HIGH);

	compiles to the assembly:

		sbi 0x0b, 3  ;i.e. PORTD |= 1 << 3;
		sbi 0x0a, 3  ;i.e. DDRD  |= 1 << 3;

	In short, if your inputs are constants, you can use this library in
	place of raw digital I/O register manipulation without worrying
	about any significantly increased overhead or processing time.
	Using variables as inputs can increase overhead and processing time,
	but the functions in this library allow for simpler programmatic
	approaches to working with digital I/O, since you no longer have to
	deal with a multitude of pin-specific registers.

	The digital pins on the AVR default to high-impedance inputs after
	a power-up or reset.
*/



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


#ifndef digitalAnalog_h
#define digitalAnalog_h

#ifdef __cplusplus
extern "C" {
#endif

#include <avr/io.h>

#define INPUT 				0
#define OUTPUT				1
#define LOW					0
#define HIGH				1
#define TOGGLE				0xFF
#define HIGH_IMPEDANCE		0
#define PULL_UP_ENABLED		1


#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
// I'm not sure why Ben went to the effort of doing this out of order for Orangutan SVP.
// Doesn't seem to match any physical reasoning...
// So here, I'm keeping it simple.
// If there is a pin that isn't connected... Well, I guess it won't do anything.

// port A pins
#define IO_A0				0
#define IO_A1				1
#define IO_A2				2
#define IO_A3				3
#define IO_A4				4
#define IO_A5				5
#define IO_A6				6
#define IO_A7				7

// port B pins
#define IO_B0				8
#define IO_B1				9
#define IO_B2				10
#define IO_B3				11
#define IO_B4				12
#define IO_B5				13
#define IO_B6				14
#define IO_B7				15

// port C pins
#define IO_C0				16
#define IO_C1				17
#define IO_C2				18
#define IO_C3				19
#define IO_C4				20
#define IO_C5				21
#define IO_C6				22
#define IO_C7				23

// port D pins
#define IO_D0				24
#define IO_D1				25
#define IO_D2				26
#define IO_D3				27
#define IO_D4				28
#define IO_D5				29
#define IO_D6				30
#define IO_D7				31

// port E pins
#define IO_E0				32
#define IO_E1				33
#define IO_E2				34
#define IO_E3				35
#define IO_E4				36
#define IO_E5				37
#define IO_E6				38
#define IO_E7				39

// port F pins
#define IO_F0				40
#define IO_F1				41
#define IO_F2				42
#define IO_F3				43
#define IO_F4				44
#define IO_F5				45
#define IO_F6				46
#define IO_F7				47

// port G pins
#define IO_G0				48
#define IO_G1				49
#define IO_G2				50
#define IO_G3				51
#define IO_G4				52
#define IO_G5				53
//#define IO_G6				54 // doesn't exist
//#define IO_G7				55 // doesn't exist

// port H pins
#define IO_H0				56
#define IO_H1				57
#define IO_H2				58
#define IO_H3				59
#define IO_H4				60
#define IO_H5				61
#define IO_H6				62
#define IO_H7				63

// port J pins
#define IO_J0				64
#define IO_J1				65
#define IO_J2				66
#define IO_J3				67
#define IO_J4				68
#define IO_J5				69
#define IO_J6				70
#define IO_J7				71

// port K pins
#define IO_K0				72
#define IO_K1				73
#define IO_K2				74
#define IO_K3				75
#define IO_K4				76
#define IO_K5				77
#define IO_K6				78
#define IO_K7				79

// port L pins
#define IO_L0				80
#define IO_L1				81
#define IO_L2				82
#define IO_L3				83
#define IO_L4				84
#define IO_L5				85
#define IO_L6				86
#define IO_L7				87

#elif defined(__AVR_ATmega324P__)  || defined(__AVR_ATmega644P__)|| defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__)

// port D pins
#define IO_D0				0
#define IO_D1				1
#define IO_D2				2
#define IO_D3				3
#define IO_D4				4
#define IO_D5				5
#define IO_D6				6
#define IO_D7				7

// port B pins
#define IO_B0				8
#define IO_B1				9
#define IO_B2				10
#define IO_B3				11
#define IO_B4				12
#define IO_B5				13
#define IO_B6				14
#define IO_B7				15

// port C pins
#define IO_C0				16
#define IO_C1				17
#define IO_C2				18
#define IO_C3				19
#define IO_C4				20
#define IO_C5				21
#define IO_C6				22
#define IO_C7				23

// port A pins
#define IO_A0				24
#define IO_A1				25
#define IO_A2				26
#define IO_A3				27
#define IO_A4				28
#define IO_A5				29
#define IO_A6				30
#define IO_A7				31

#else // Arduino Compatible notation, for whatever that is worth.

// port D pins
#define IO_D0				0
#define IO_D1				1
#define IO_D2				2
#define IO_D3				3
#define IO_D4				4
#define IO_D5				5
#define IO_D6				6
#define IO_D7				7

// port B pins
#define IO_B0				8
#define IO_B1				9
#define IO_B2				10
#define IO_B3				11
#define IO_B4				12
#define IO_B5				13

// port C pins
#define IO_C0				14
#define IO_C1				15
#define IO_C2				16
#define IO_C3				17
#define IO_C4				18
#define IO_C5				19
#define IO_C6				20	// only used if RESET pin is changed to be a digital I/O

#endif

// Analogue

#define MODE_8_BIT		1
#define MODE_10_BIT		0

#define INTERNAL_REF    1
#define EXTERNAL_REF    0


// high-level method for setting the specified pin as an output with the specified output state.
// An outputState value of 0 will cause the pin to drive low; a value of 1 will cause the pin to
// drive high.  A value of 0xFF (255) will toggle the output state of the pin (i.e. high -> low and
// low -> high).
extern void setDigitalOutput(uint8_t pin, uint8_t outputState);

// high-level method for setting the specified pin as an input with the specified input state.
// An inputState value of 0 will cause the pin to be a high-impedance input; a value of 1 will enable the
// pin's internal pull-up resistor, which weakly pulls it to Vcc.  A value of 0xFF (255) will toggle the
// input state.
extern void setDigitalInput(uint8_t pin, uint8_t inputState);

// high-level method for reading the input value of the specified pin.  If the voltage on the pin is low,
// this method will return 0.  Otherwise, it will return a non-zero result that depends on the value of
// the pin.
extern uint8_t isDigitalInputHigh(uint8_t pin);

// set the ADC to run in either 8-bit mode (MODE_8_BIT) or
// 10-bit mode (MODE_10_BIT)
extern void setAnalogMode(uint8_t mode);

// returns 0 if in 10-bit mode, otherwise returns non-zero.  The return
// value of this method can be directly compared against the macros
// MODE_8_BIT and MODE_10_BIT:
// For example: if (getMode() == MODE_8_BIT) ...
extern uint8_t getAnalogMode();

// returns 1 if the ADC is in the middle of an conversion, otherwise
// returns 0
extern uint8_t analogIsConverting();

// the following method can be used to initiate an ADC conversion
// that runs in the background, allowing the CPU to perform other tasks
// while the conversion is in progress.  The procedure is to start a
// conversion on a channel with startConversion(channel), and then
// poll isConverting in your main loop.  Once isConverting() returns
// a zero, the result can be obtained through a call to conversionResult().
// If use_internal_reference is set to true, the function will use the
// internal 1.1V voltage reference on the ATmega48/168/328 or the internal
// 2.56V voltage reference on the ATmega324/644/1284; otherwise, it uses
// the AVCC pin as a reference.
// *** NOTE ***: Some Orangutans and 3pis have their AREF pin connected directly to VCC.
//  On these Orangutans, you must not use the internal voltage reference as
//  doing so will short the internal reference voltage to VCC and could damage
//  the AVR.  It is safe to use the internal reference voltage on the
//  Orangutan SVP.

extern void startAnalogConversion( uint8_t channel, uint8_t use_internal_reference);

// returns the result of the previous ADC conversion.
extern uint16_t analogConversionResult();



#ifdef __cplusplus
}
#endif

#endif // digitalAnalog_h
