/*
 * Copyright (C) 2018 Phillip Stevens  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * 1 tab == 4 spaces!
 *
 * This file is NOT part of the FreeRTOS distribution.
 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>

#include <util/delay.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "stack_macros.h"

/*-----------------------------------------------------------*/
#if ( configUSE_IDLE_HOOK == 1 )

void vApplicationIdleHook( void ) __attribute__((weak));

void vApplicationIdleHook( void )
{
	// Digital Input Disable on Analogue Pins
	// When this bit is written logic one, the digital input buffer on the corresponding ADC pin is disabled.
	// The corresponding PIN Register bit will always read as zero when this bit is set. When an
	// analogue signal is applied to the ADC7..0 pin and the digital input from this pin is not needed, this
	// bit should be written logic one to reduce power consumption in the digital input buffer.

#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) // Arduino Mega with 2560
	DIDR0 = 0xFF;
	DIDR2 = 0xFF;

#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284PA__) // with 1284p
	DIDR0 = 0xFF;

#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) // assume we're using an Arduino with 328p
	DIDR0 = 0x3F;

#elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__) // assume we're using an Arduino Leonardo with 32u4
	DIDR0 = 0xF3;
	DIDR2 = 0x3F;

#endif

	// Analogue Comparator Disable
	// When the ACD bit is written logic one, the power to the Analogue Comparator is switched off.
	// This bit can be set at any time to turn off the Analogue Comparator.
	// This will reduce power consumption in Active and Idle mode.
	// When changing the ACD bit, the Analogue Comparator Interrupt must be disabled by clearing the ACIE bit in ACSR.
	// Otherwise an interrupt can occur when the ACD bit is changed.
	ACSR &= ~_BV(ACIE);
	ACSR |=  _BV(ACD);

	// There are several macros provided in this header file to actually put the device into sleep mode.
	// The simplest way is to optionally set the desired sleep mode using set_sleep_mode()
	// (it usually defaults to idle mode where the CPU is put on sleep but all peripheral clocks are still running),
	// and then call sleep_mode(). This macro automatically sets the sleep enable bit,
	// goes to sleep, and clears the sleep enable bit.

	// SLEEP_MODE_IDLE         (0)
	// SLEEP_MODE_ADC          _BV(SM0)
	// SLEEP_MODE_PWR_DOWN     _BV(SM1)
	// SLEEP_MODE_PWR_SAVE     (_BV(SM0) | _BV(SM1))
	// SLEEP_MODE_STANDBY      (_BV(SM1) | _BV(SM2))
	// SLEEP_MODE_EXT_STANDBY  (_BV(SM0) | _BV(SM1) | _BV(SM2))

	set_sleep_mode( SLEEP_MODE_IDLE );

	portENTER_CRITICAL();
	sleep_enable();

#if defined(BODS) && defined(BODSE) // only if there is support to disable the BOD.
	sleep_bod_disable();
#endif

	portEXIT_CRITICAL();
	sleep_cpu();		// good night.

	sleep_reset();		// reset the sleep_mode() faster than sleep_disable();
}

#endif /* configUSE_IDLE_HOOK == 1 */
/*-----------------------------------------------------------*/


#if defined( configUSE_MALLOC_FAILED_HOOK)

void vApplicationMallocFailedHook( void ) __attribute__((weak));

void vApplicationMallocFailedHook( void )
{
	/*---------------------------------------------------------------------------*\
	Usage:
	   called by task system when a malloc failure is noticed
	Description:
	   Malloc failure handler -- Shut down all interrupts, send serious complaint
	    to command port. FAST Blink.
	Arguments:
	   pxTask - pointer to task handle
	   pcTaskName - pointer to task name
	Results:
	   <none>
	Notes:
	   This routine will never return.
	   This routine is referenced in the task.c file of FreeRTOS as an extern.
	\*---------------------------------------------------------------------------*/

#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) // Arduino Mega with 2560
	DDRB  |= _BV(DDB7);
	PORTB |= _BV(PORTB7);       // Main (red PB7) LED on. Main LED on.

#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284PA__) // Goldilocks with 1284p
	DDRB  |= _BV(DDB7);
	PORTB |= _BV(PORTB7);       // Main (red PB7) LED on. Main LED on.

#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) // assume we're using an Arduino Uno with 328p
	DDRB  |= _BV(DDB5);
	PORTB |= _BV(PORTB5);       // Main (red PB5) LED on. Main LED on.

#elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__) // assume we're using an Arduino Leonardo with 32u4
	DDRC  |= _BV(DDC7);
	PORTC |= _BV(PORTC7);       // Main (red PC7) LED on. Main LED on.

#endif

	for(;;)
	{
		_delay_ms(50);

#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)  // Arduino Mega with 2560
		PINB  |= _BV(PINB7);       // Main (red PB7) LED toggle. Main LED fast blink.

#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284PA__) // Goldilocks with 1284p
		PINB  |= _BV(PINB7);       // Main (red PB7) LED toggle. Main LED fast blink.

#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) // assume we're using an Arduino Uno with 328p
		PINB  |= _BV(PINB5);       // Main (red PB5) LED toggle. Main LED fast blink.

#elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__) // assume we're using an Arduino Leonardo with 32u4
		PINC  |= _BV(PINC7);       // Main (red PC7) LED toggle. Main LED fast blink.

#endif

	}
}

#endif /* configUSE_MALLOC_FAILED_HOOK == 1 */
/*-----------------------------------------------------------*/


#if ( configCHECK_FOR_STACK_OVERFLOW >= 1 )

void vApplicationStackOverflowHook( TaskHandle_t xTask, portCHAR *pcTaskName ) __attribute__((weak));

void vApplicationStackOverflowHook( TaskHandle_t xTask __attribute__((unused)), portCHAR *pcTaskName __attribute__((unused)) )
{
	/*---------------------------------------------------------------------------*\
	Usage:
	   called by task system when a stack overflow is noticed
	Description:
	   Stack overflow handler -- Shut down all interrupts, send serious complaint
	    to command port. SLOW Blink.
	Arguments:
	   pxTask - pointer to task handle
	   pcTaskName - pointer to task name
	Results:
	   <none>
	Notes:
	   This routine will never return.
	   This routine is referenced in the task.c file of FreeRTOS as an extern.
	\*---------------------------------------------------------------------------*/

#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)  // Arduino Mega with 2560
	DDRB  |= _BV(DDB7);
	PORTB |= _BV(PORTB7);       // Main (red PB7) LED on. Main LED on.

#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284PA__) // Goldilocks with 1284p
	DDRB  |= _BV(DDB7);
	PORTB |= _BV(PORTB7);       // Main (red PB7) LED on. Main LED on.

#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) // assume we're using an Arduino Uno with 328p
	DDRB  |= _BV(DDB5);
	PORTB |= _BV(PORTB5);       // Main (red PB5) LED on. Main LED on.

#elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__) // assume we're using an Arduino Leonardo with 32u4
	DDRC  |= _BV(DDC7);
	PORTC |= _BV(PORTC7);       // Main (red PC7) LED on. Main LED on.

#endif

	for(;;)
	{
		_delay_ms(2000);

#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)  // Arduino Mega with 2560
		PINB  |= _BV(PINB7);       // Main (red PB7) LED toggle. Main LED slow blink.

#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284PA__) // Goldilocks with 1284p
		PINB  |= _BV(PINB7);       // Main (red PB7) LED toggle. Main LED slow blink.

#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) // assume we're using an Arduino Uno with 328p
		PINB  |= _BV(PINB5);       // Main (red PB5) LED toggle. Main LED slow blink.

#elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__) // assume we're using an Arduino Leonardo with 32u4
		PINC  |= _BV(PINC7);       // Main (red PC7) LED toggle. Main LED slow blink.

#endif

	}
}

#else

void vApplicationStackOverflowHook( TaskHandle_t xTask __attribute__((unused)), portCHAR *pcTaskName __attribute__((unused)) )
{
	/*---------------------------------------------------------------------------*\
	Usage:
	   called by task system when a stack overflow is noticed
	Description:
	   Stack overflow handler -- Shut down all interrupts, send serious complaint
	    to command port.
	Arguments:
	   pxTask - pointer to task handle
	   pcTaskName - pointer to task name
	Results:
	   <none>
	Notes:
	   This routine will never return.
	   This routine is referenced in the task.c file of FreeRTOS as an extern.
	\*---------------------------------------------------------------------------*/

	uint8_t* pC;
	uint16_t baud;

	extern uint8_t * LineBuffer;	// line buffer on heap (with pvPortMalloc).

	/* shut down all interrupts */
	portDISABLE_INTERRUPTS();


	/* take over the command line buffer to generate our error message */
	pC = (uint8_t*) LineBuffer;

	strcat_P( (char*) pC, PSTR("\r\n"));
	strcat( (char*) pC, (char*) pcTaskName );
	strcat_P( (char*) pC, PSTR("\r\n"));

	pC = (uint8_t*) LineBuffer;

	/* Force the UART control register to be the way we want, just in case */

	UCSR0C = ( _BV( UCSZ01 ) | _BV( UCSZ00 ) );		// 8 data bits
	UCSR0B = _BV( TXEN0 );							// only enable transmit
	UCSR0A = 0;

	/* Calculate the baud rate register value from the equation in the
	* data sheet.  This calculation rounds to the nearest factor, which
	* means the resulting rate may be either faster or slower than the
	* desired rate (the old calculation was always faster).
	*
	* If the system clock is one of the Magic Frequencies, this
	* computation will result in the exact baud rate
	*/
	baud = ( ( ( configCPU_CLOCK_HZ / ( ( 16UL * 38400 ) / 2UL ) ) + 1UL ) / 2UL ) - 1UL;
	UBRR0 = baud;

	/* Send out the message, without interrupts.  Hard wired to USART 0 */
	while ( *pC )
	{
		while (!(UCSR0A & (1 << UDRE0)));
		UDR0 = *pC;
		pC++;
	}

	while(1){ PINB |= _BV(PINB7); _delay_ms(100); } // main (red PB7) LED flash and die.
}

#endif /* configCHECK_FOR_STACK_OVERFLOW >= 1 */
/*-----------------------------------------------------------*/

#if ( configSUPPORT_STATIC_ALLOCATION >= 1 )

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    configSTACK_DEPTH_TYPE *pulIdleTaskStackSize ) __attribute__((weak));

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    configSTACK_DEPTH_TYPE *pulIdleTaskStackSize )
{
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

#if ( configUSE_TIMERS >= 1 )

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     configSTACK_DEPTH_TYPE *pulTimerTaskStackSize ) __attribute__((weak));

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     configSTACK_DEPTH_TYPE *pulTimerTaskStackSize )
{
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

#endif /* configUSE_TIMERS >= 1 */

#endif /* configSUPPORT_STATIC_ALLOCATION >= 1 */
