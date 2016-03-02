

#ifndef digitalAnalog_h
#define digitalAnalog_h

#include <stdint.h>

#include <avr/io.h>

#ifdef __cplusplus
extern "C" {
#endif

// Analogue

#define MODE_8_BIT		1
#define MODE_10_BIT		0

#define INTERNAL_REF    1
#define EXTERNAL_REF    0


/************** ANALOGUE *****************/

// set the ADC to run in either 8-bit mode (MODE_8_BIT) or
// 10-bit mode (MODE_10_BIT)
void setAnalogMode(uint8_t mode);

// returns 0 if in 10-bit mode, otherwise returns non-zero.  The return
// value of this method can be directly compared against the macros
// MODE_8_BIT and MODE_10_BIT:
// For example: if (getMode() == MODE_8_BIT) ...
uint8_t getAnalogMode(void);

// returns 1 if the ADC is in the middle of an conversion, otherwise
// returns 0
uint8_t analogIsConverting(void);

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

void startAnalogConversion( uint8_t channel, uint8_t use_internal_reference);

// returns the result of the previous ADC conversion.
uint16_t analogConversionResult(void);


#ifdef __cplusplus
}
#endif

#endif // digitalAnalog_h
