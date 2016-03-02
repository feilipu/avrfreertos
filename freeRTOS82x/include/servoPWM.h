#ifndef SERVO_PWM
#define SERVO_PWM

/*
  ServoPWM - Hardware serial library based on ServoTimer1 by Jim Studt

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
#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined( portUSE_TIMER0_PWM )         /* Hardware constants for PWM Timer0. */

	#define pwmBASE_HZ                              ( ( uint16_t ) 50 )
    #define pwmCLOCK_PRESCALER				        ( ( uint16_t ) 1024 )
    #define pwmPRESCALE_8				            ( ( uint8_t ) (1<<CS01) )
    #define pwmPRESCALE_64				            ( ( uint8_t ) ((1<<CS01)|(1<<CS00)) )
    #define pwmPRESCALE_256				            ( ( uint8_t ) (1<<CS02) )
    #define pwmPRESCALE_1024     			        ( ( uint8_t ) ((1<<CS02)|(1<<CS00)) )

	#define pwmOCRa                                 OCR0A
    #define pwmOCRb                                 OCR0B

    #define pwmTCCRa                                TCCR0A
    #define pwmTCCRb                                TCCR0B

    #define pwmFAST_NONINVERTED_a                   ( ( uint8_t ) (1<<COM0A1)|(1<<COM0B1)|(1<<WGM00) ) // PWM Phase Correct
                                                        // PWM Phase Correct has 1/2 the period of Fast PWM at given clock divide.

    #define pwmDDRD                                 ( ( uint8_t ) ((1<<PD6)|(1<<PD5)) ) // PD6 OC0A & PD5 OC0B

#elif defined( portUSE_TIMER1_PWM ) /* Hardware constants for PWM Timer1. */

    #define pwmBASE_HZ                              ( ( uint16_t ) 50 )
    #define pwmCLOCK_PRESCALER				        ( ( uint16_t ) 8 )
    #define pwmPRESCALE_8				            ( ( uint8_t ) (1<<CS11) ) // for 22118400 Hz CPU 8 is the right prescale
    #define pwmPRESCALE_64				            ( ( uint8_t ) ((1<<CS11)|(1<<CS10)) )
    #define pwmPRESCALE_256				            ( ( uint8_t ) (1<<CS12) )
    #define pwmPRESCALE_1024     			        ( ( uint8_t ) ((1<<CS12)|(1<<CS10)) )

    #define pwmOCRa                                 OCR1A
    #define pwmOCRb                                 OCR1B

    #define pwmICR1                              	ICR1

    #define pwmTCCRa                                TCCR1A
    #define pwmTCCRb                              	TCCR1B

    #define pwmFAST_NONINVERTED_a                   ( ( uint8_t ) (1<<COM1A1)|(1<<COM1B1)|(1<<WGM11) ) // Fast PWM
    #define pwmFAST_NONINVERTED_b                   ( ( uint8_t ) (1<<WGM13)|(1<<WGM12) )

#if defined(__AVR_ATmega328P__)  // Arduino
    #define pwmDDRB                                 ( ( uint8_t ) ((1<<PB1)|(1<<PB2)) ) // PB1 OC1A & PB2 OC1B
#elif defined(__AVR_ATmega324P__)  || defined(__AVR_ATmega644P__)|| defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__) // Pololu SVP with 1284p
	#define pwmDDRD                                 ( ( uint8_t ) ((1<<PD5)|(1<<PD4)) ) // PD5 OC1A & PD4 OC1B
#endif

#endif

void start_PWM_hardware(void);									// Set PWM hardware to 20Ms fast PWM.
void stop_PWM_hardware(void);								 	// Set PWM hardware to 20Ms fast PWM.
void set_PWM_hardware( uint16_t servoA, uint16_t servoB);		// PWM pulse width in uS for both servos.

#ifdef __cplusplus
}
#endif

#endif // SERVO_PWM
