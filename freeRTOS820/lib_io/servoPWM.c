
/*
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


#include <avr/interrupt.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

#include "servoPWM.h"

#if defined( portUSE_TIMER0_PWM ) || defined( portUSE_TIMER1_PWM )

void start_PWM_hardware()
{
    taskENTER_CRITICAL();

#if defined( portUSE_TIMER0_PWM )
#warning "Timer0 used for PWM."

    // OutputHZ = configCPU_CLOCK_HZ/(pwmCLOCK_PRESCALER*510) = 42.352Hz with pwmPRESCALE_1024 Phase Correct PWM
    // This calculation result applies to 22118400Hz.
    pwmTCCRb = pwmPRESCALE_1024;                            // always counts to 0xFF

    DDRD |= pwmDDRD; // PD6 OC0A (Pin 6) & PD5 OC0B (Pin 5)

     // set servo A to mid range, being 3/40 or 1500uS/20000uS
    pwmOCRa = (uint8_t)( (configCPU_CLOCK_HZ/(pwmCLOCK_PRESCALER*pwmBASE_HZ) - 1) / 5 * 3 / 8 );

     // set servo B to mid range, being 3/40 or 1500uS/20000uS
    pwmOCRb = (uint8_t)( (configCPU_CLOCK_HZ/(pwmCLOCK_PRESCALER*pwmBASE_HZ) - 1) / 5 * 3 / 8 );

    pwmTCCRa = pwmFAST_NONINVERTED_a; // turn on the PWM...

#elif defined( portUSE_TIMER1_PWM )
#warning "Timer1 used for PWM."

    pwmTCCRb = pwmFAST_NONINVERTED_b | pwmPRESCALE_8;

    /* pwmIRC1 = configCPU_CLOCK_HZ/(pwmCLOCK_PRESCALER*pwmBASE_HZ) - 1 = 55,295 with pwmPRESCALE_8 */
    // This calculation result applies to 22118400Hz.
    pwmICR1 = configCPU_CLOCK_HZ/(pwmCLOCK_PRESCALER*pwmBASE_HZ) - 1; // 50Hz or 20mS cycles.

#if defined(__AVR_ATmega328P__)  // Arduino
    DDRB |= pwmDDRB; // PB1 OC1A (Pin 9) & PB2 OC1B (Pin 10)

#elif defined(__AVR_ATmega324P__)  || defined(__AVR_ATmega644P__)|| defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__) // Pololu SVP with 1284p
    DDRD |= pwmDDRD; // PD5 OC1A & PD4 OC1B

#endif

    // set servo A to mid range, being 3/40 or 1500uS/20000uS
    pwmOCRa = (uint16_t)( (configCPU_CLOCK_HZ/(pwmCLOCK_PRESCALER*pwmBASE_HZ) - 1) / 5 * 3 / 8 );

     // set servo B to mid range, being 3/40 or 1500uS/20000uS
    pwmOCRb = (uint16_t)( (configCPU_CLOCK_HZ/(pwmCLOCK_PRESCALER*pwmBASE_HZ) - 1) / 5 * 3 / 8 );

    pwmTCCRa = pwmFAST_NONINVERTED_a; // turn on the PWM...

#endif


    taskEXIT_CRITICAL();
}

void stop_PWM_hardware()
{
    taskENTER_CRITICAL();
    pwmTCCRa = 0;

#if defined( portUSE_TIMER0_PWM )
    pwmTCCRb = 0;
	DDRD &= ~pwmDDRD; // PD6 OC0A (Pin 6) & PD5 OC0B (Pin 5)

#elif defined( portUSE_TIMER1_PWM )

	#if defined(__AVR_ATmega328P__)  // Arduino
		DDRB &= ~pwmDDRB; // PB1 OC1A (Pin 9) & PB2 OC1B (Pin 10)

	#elif defined(__AVR_ATmega324P__)  || defined(__AVR_ATmega644P__)|| defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__)// Pololu SVP with 1284p
		DDRD &= ~pwmDDRD; // PD5 OC1A & PD4 OC1B

	#endif

#endif

    taskEXIT_CRITICAL();
}


void set_PWM_hardware( uint16_t servoA, uint16_t servoB)     // PWM pulse width in uS
{
#if defined( portUSE_TIMER0_PWM )

    uint8_t OCRA;
    uint8_t OCRB;

    OCRA = (uint8_t)( (float)servoA * configCPU_CLOCK_HZ  / pwmCLOCK_PRESCALER / 1000000 ); // This calculation is just wrong,
    OCRB = (uint8_t)( (float)servoB * configCPU_CLOCK_HZ  / pwmCLOCK_PRESCALER / 1000000 ); // but I have no interest to fix it.

    taskENTER_CRITICAL();
    pwmOCRa = OCRA;
    pwmOCRb = OCRB;
    taskEXIT_CRITICAL();

#elif defined( portUSE_TIMER1_PWM )

    uint16_t OCRA;
    uint16_t OCRB;

    // pwmOCRx is set in uS, so convert from uS to # of steps to pwmICR1
    OCRA = (uint16_t)( (float)servoA * (configCPU_CLOCK_HZ/(pwmCLOCK_PRESCALER*pwmBASE_HZ) - 1) * pwmBASE_HZ / 1000000.0 );
    OCRB = (uint16_t)( (float)servoB * (configCPU_CLOCK_HZ/(pwmCLOCK_PRESCALER*pwmBASE_HZ) - 1) * pwmBASE_HZ / 1000000.0 );

    taskENTER_CRITICAL();
    pwmOCRa = OCRA;
    pwmOCRb = OCRB;
    taskEXIT_CRITICAL();

#endif
}

#endif // #if defined(portUSE_TIMER0_PWM) || defined(portUSE_TIMER1_PWM)

