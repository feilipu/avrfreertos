/*
    FreeRTOS V7.1.1 - Copyright (C) 2012 Real Time Engineers Ltd.


    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/


#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <avr/io.h>

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

// Define portUSE_TIMER0 or portUSE_TIMER1 to use the Arduino 328p
// Otherwise for Arduino Mega Rev3 and Pololu SVP 1284P, Timer3 should be the default

#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
	#define portUSE_TIMER3											// portUSE_TIMER3 to use 16 bit Timer3
    #define configTICK_RATE_HZ		( ( portTickType ) 500 )		// Use 500Hz for TIMER3
                                                                    // Use 1000Hz to get mSec timing.

	#define configCPU_CLOCK_HZ		( ( uint32_t ) F_CPU)			// This F_CPU variable set by Eclipse environment
//	#define configCPU_CLOCK_HZ		( ( uint32_t ) 16000000 )		// Arduino Mega2560 Rev3


//	XRAM device options. Different methods of enabling and driving.    MegaRAM only implemented for two banks of 56kByte currently.
	#define portMEGA_RAM											// Use the Rugged Circuits External (128kByte) MegaRAM device. - OR -
	#define portQUAD_RAM											// Use the Rugged Circuits External (512kByte) QuadRAM device.

//	portQUAD_RAM device Options. NOT valid for use with portMEGA_RAM.  XRAM Memory is available as 8 banks of 56kByte, for heap. - OR -
//	#define portEXT_RAM_16_BANK										// XRAM Memory is available as 16 banks of 32kByte, for heap. - OR -
//	#define portEXT_RAMFS											// XRAM Memory is available as 16 banks of 32kByte for 16 Arduino clients (i.e. NOT used for heap).


#if defined (portQUAD_RAM) || defined (portMEGA_RAM)
	#define portEXT_RAM
#endif

#if defined (portMEGA_RAM) || (defined (portQUAD_RAM) && !defined (portEXT_RAMFS))
	// XRAM banks enabled. We have the linker to move the heap to XRAM.
	#define configTOTAL_HEAP_SIZE	( (size_t ) ((uint8_t *)(XRAMEND - 0x8000)) ) // Should be 0xffff - 0x8000 = 32767 for (non malloc) heap in XRAM.
																	// Used for heap_1.c, heap2.c, and heap4.c only, and maximum Array size possible for Heap is 32767.
#else
	// There is no XRAM available for the heap.
	#define configTOTAL_HEAP_SIZE	( (size_t ) 0x1800 )			// 0x1800 = 6144 used for heap_1.c, heap2.c, and heap4.c only, where heap is NOT in XRAM.
																	// Used for heap_1.c, heap2.c, and heap4.c only, and maximum Array size possible for Heap is 32767.
#endif


	#define portHD44780_LCD					// define the use of the Freetronics HD44780 LCD (or other). Check include for (flexible) pin assignments.
	#define portSD_CARD						// define the use of the SD Card for Arduino Mega2560 and Freetronics EtherMega
	#define portRTC_DEFINED					// RTC DS1307 implemented, therefore define.
	#define	portSERIAL_BUFFER		255		// Define the size of the serial buffer.



#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) // assume we're using an Arduino with 328p
//  THIS IS IMPORTANT TO DEFINE THE freeRTOS TICK TIMER, and also Tick Rate
//  Define either: portUSE_TIMER0 or portUSE_TIMER1 for the Arduino 328p
    #define portUSE_TIMER0                                          // portUSE_TIMER0 to use 8 bit Timer0
//  #define portUSE_TIMER1                                          // portUSE_TIMER1 to use 16 bit Timer1
    #define configTICK_RATE_HZ		( ( portTickType ) 200 )        // Use 200Hz for TIMER0 and 400Hz for TIMER1
                                                                    // Use 1000Hz to get mSec timing.

	#define configCPU_CLOCK_HZ		( ( uint32_t ) F_CPU)			// This F_CPU variable set by Eclipse environment
//	#define configCPU_CLOCK_HZ		( ( uint32_t ) 22118400 )  		// Modded Arduino with 328p device
//	#define configCPU_CLOCK_HZ		( ( uint32_t ) 16000000 )  		// Standard Arduino with 328p device

	#define portEXT_RAMFS											// XRAM Memory is available by a 2560 as 16 banks of 32kByte for 16x 328p Arduino clients.

	// Cannot emphasise how important it is to watch and massage this heap size number.
	// Greater than 100% memory usage. Subtle fail.
	// Less than 96%. Typically every byte counts for 328p.
	// Watch for the stack overflowing, if you use interrupts. Use configCHECK_FOR_STACK_OVERFLOW
    #define configTOTAL_HEAP_SIZE	( (size_t ) 1024 )				// used for heap_1.c, heap_2.c, and heap_4.c only

//	#define portHD44780_LCD					// define the use of the Freetronics HD44780 LCD (or other). Check include for (flexible) pin assignments.
//	#define portRTC_DEFINED					// RTC DS1307 implemented, therefore define.
	#define	portSERIAL_BUFFER		64		// Define the size of the serial buffer.




#elif (defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284PA__)) // Pololu SVP with 1284p
	#define portUSE_TIMER3											// portUSE_TIMER3 to use 16 bit Timer3 on 1284p device
    #define configTICK_RATE_HZ		( ( portTickType ) 500 )		// Use 500Hz for TIMER3
                                                                    // Use 1000Hz to get mSec timing.

	#define configCPU_CLOCK_HZ		( ( uint32_t ) F_CPU)			// This F_CPU variable set by Eclipse environment
//  #define configCPU_CLOCK_HZ		( ( uint32_t ) 20000000 )		// Pololu SVP
    #define configTOTAL_HEAP_SIZE	( (size_t )  14000  )			// used for heap_1.c and heap2.c, and heap_4.c only


//	#define portHD44780_LCD					// define the use of the Freetronics HD44780 LCD (or other). Check include for (flexible) pin assignments.
	#define portSD_CARD						// define the use of the SD Card for Arduino Mega2560 and Freetronics EtherMega
//	#define portRTC_DEFINED					// RTC DS1307 implemented, therefore define.
	#define	portSERIAL_BUFFER		255		// Define the size of the serial buffer.


#endif


// Define which Timer to use as the PWM Timer (not the tick timer).
#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
    #define portUSE_TIMER1_PWM

#elif defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284PA__)
    #define portUSE_TIMER1_PWM  // though it is better to use Pololu functions, as they support 8x multiplexed servos.

#elif defined( portUSE_TIMER0 )
    #define portUSE_TIMER1_PWM

#elif defined(__AVR_ATmega328P__) || defined(portUSE_TIMER1)
// It's pointless to use the 8bit Timer0 or Timer2 for Servo PWM, as these Timers can't set ICR1 as TOP, allowing the pulse width period to be adjusted.
    #define portUSE_TIMER0_PWM

#endif

// And on to the things the same no matter the AVR type...
#define configUSE_PREEMPTION		    1
#define configUSE_IDLE_HOOK		        0
#define configUSE_TICK_HOOK		        0
#define configMAX_PRIORITIES		    ( ( unsigned portBASE_TYPE ) 4 )
#define configMINIMAL_STACK_SIZE	    ( ( uint16_t ) 85 )
#define configMAX_TASK_NAME_LEN		    ( 16 )
#define configUSE_TRACE_FACILITY	    0
#define configUSE_16_BIT_TICKS		    1
#define configIDLE_SHOULD_YIELD		    1
#define configUSE_MUTEXES               1
#define configUSE_RECURSIVE_MUTEXES     0
#define configUSE_COUNTING_SEMAPHORES   0
#define configUSE_ALTERNATIVE_API       0
#define configCHECK_FOR_STACK_OVERFLOW  0
#define configQUEUE_REGISTRY_SIZE	    0

/* Timer definitions. */
#define configUSE_TIMERS				0
#define configTIMER_TASK_PRIORITY       ( ( unsigned portBASE_TYPE ) 7 )
#define configTIMER_QUEUE_LENGTH        ( ( unsigned portBASE_TYPE ) 10 )
#define configTIMER_TASK_STACK_DEPTH    configMINIMAL_STACK_SIZE

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES 		    0
#define configMAX_CO_ROUTINE_PRIORITIES ( 2 )

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */

#define INCLUDE_vTaskPrioritySet		        0
#define INCLUDE_uxTaskPriorityGet		        0
#define INCLUDE_vTaskDelete			            0
#define INCLUDE_vTaskCleanUpResources		    0
#define INCLUDE_vTaskSuspend			        1
#define INCLUDE_vResumeFromISR                  1
#define INCLUDE_vTaskDelayUntil			        1
#define INCLUDE_vTaskDelay			            1
#define INCLUDE_xTaskGetSchedulerState          0
#define INCLUDE_xTaskGetCurrentTaskHandle       0
#define INCLUDE_uxTaskGetStackHighWaterMark     1

#endif /* FREERTOS_CONFIG_H */
