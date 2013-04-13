/*
    FreeRTOS V7.3.0 - Copyright (C) 2012 Real Time Engineers Ltd.

    FEATURES AND PORTS ARE ADDED TO FREERTOS ALL THE TIME.  PLEASE VISIT
    http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

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

    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?"                                     *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************


    http://www.FreeRTOS.org - Documentation, training, latest versions, license
    and contact details.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool.

    Real Time Engineers ltd license FreeRTOS to High Integrity Systems, who sell
    the code with commercial support, indemnification, and middleware, under
    the OpenRTOS brand: http://www.OpenRTOS.com.  High Integrity Systems also
    provide a safety engineered and independently SIL3 certified version under
    the SafeRTOS brand: http://www.SafeRTOS.com.
*/

/*

Changes from V2.6.0

	+ AVR port - Replaced the inb() and outb() functions with direct memory
	  access.  This allows the port to be built with the 20050414 build of
	  WinAVR.
*/

/*
Changes for FiFiSMSer
    Made the Scheduler Timer selectable (Between Timer0, Timer1 & Timer3)
    Richard Barry
    http://o28.sischa.net/fimser/svn/branches/yfe_rtos/port.c
*/

#include <stdlib.h>
#include <avr/interrupt.h>

#include <FreeRTOS.h>
#include <task.h>

#include <ext_ram.h>

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the AVR port.
 *----------------------------------------------------------*/



/* Start tasks with interrupts enabled. */
#define portFLAGS_INT_ENABLED			( ( portSTACK_TYPE ) 0x80 )


#if defined( portUSE_TIMER0 )
/* Hardware constants for Timer0. */
    #define portCLEAR_COUNTER_ON_MATCH			        ( ( unsigned portCHAR ) (1<<WGM01) )
    #define portPRESCALE_1024     			            ( ( unsigned portCHAR ) ((1<<CS02)|(1<<CS00)) )
    #define portCLOCK_PRESCALER				            ( ( unsigned portLONG ) 1024 )
    #define portCOMPARE_MATCH_A_INTERRUPT_ENABLE	    ( ( unsigned portCHAR ) (1<<OCIE0A) )
    #define portOCRL                                	OCR0A
    #define portTCCRa                               	TCCR0A
    #define portTCCRb                               	TCCR0B
    #define portTIMSK                               	TIMSK0

#elif defined( portUSE_TIMER1 )
/* Hardware constants for Timer1. */
	#define portCLEAR_COUNTER_ON_MATCH			    ( ( unsigned portCHAR ) (1<<WGM12) )
	#define portPRESCALE_64				            ( ( unsigned portCHAR ) ((1<<CS11)|(1<<CS10)) )
	#define portCLOCK_PRESCALER				        ( ( unsigned portLONG ) 64 )
	#define portCOMPARE_MATCH_A_INTERRUPT_ENABLE	( ( unsigned portCHAR ) (1<<OCIE1A) )
	#define portOCRL                              	OCR1AL
	#define portOCRH                                OCR1AH
	#define portTCCRa                               TCCR1A
	#define portTCCRb                              	TCCR1B
	#define portTIMSK                               TIMSK1

#elif defined( portUSE_TIMER3 )
/* Hardware constants for Timer3. */
	#define portCLEAR_COUNTER_ON_MATCH			    ( ( unsigned portCHAR ) (1<<WGM32) )
	#define portPRESCALE_64				            ( ( unsigned portCHAR ) ((1<<CS31)|(1<<CS30)) )
	#define portCLOCK_PRESCALER				        ( ( unsigned portLONG ) 64 )
	#define portCOMPARE_MATCH_A_INTERRUPT_ENABLE	( ( unsigned portCHAR ) (1<<OCIE3A) )
	#define portOCRL                              	OCR3AL
	#define portOCRH                                OCR3AH
	#define portTCCRa                               TCCR3A
	#define portTCCRb                              	TCCR3B
	#define portTIMSK                               TIMSK3

#endif

/*-----------------------------------------------------------*/

/* We require the address of the pxCurrentTCB variable, but don't want to know
any details of its type. */
typedef void tskTCB;
extern volatile tskTCB * volatile pxCurrentTCB;

/*-----------------------------------------------------------*/

/*
 * Macro to save all the general purpose registers, the save the stack pointer
 * into the TCB.
 *
 * The first thing we do is save the flags then disable interrupts.  This is to
 * guard our stack against having a context switch interrupt after we have already
 * pushed the registers onto the stack - causing the 32 registers to be on the
 * stack twice.
 *
 * r1 is set to zero as the compiler expects it to be thus, however some
 * of the math routines make use of R1.
 *
 * The interrupts will have been disabled during the call to portSAVE_CONTEXT()
 * so we need not worry about reading/writing to the stack pointer.
 */
#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
/* 3-Byte PC Save */
#define portSAVE_CONTEXT()									\
	asm volatile (	"push	r0						\n\t"	\
					"in		r0, __SREG__			\n\t"	\
					"cli							\n\t"	\
					"push	r0						\n\t"	\
					"in		r0, 0x3b				\n\t"	\
					"push	r0						\n\t"	\
					"in		r0, 0x3c				\n\t"	\
					"push	r0						\n\t"	\
					"push	r1						\n\t"	\
					"clr	r1						\n\t"	\
					"push	r2						\n\t"	\
					"push	r3						\n\t"	\
					"push	r4						\n\t"	\
					"push	r5						\n\t"	\
					"push	r6						\n\t"	\
					"push	r7						\n\t"	\
					"push	r8						\n\t"	\
					"push	r9						\n\t"	\
					"push	r10						\n\t"	\
					"push	r11						\n\t"	\
					"push	r12						\n\t"	\
					"push	r13						\n\t"	\
					"push	r14						\n\t"	\
					"push	r15						\n\t"	\
					"push	r16						\n\t"	\
					"push	r17						\n\t"	\
					"push	r18						\n\t"	\
					"push	r19						\n\t"	\
					"push	r20						\n\t"	\
					"push	r21						\n\t"	\
					"push	r22						\n\t"	\
					"push	r23						\n\t"	\
					"push	r24						\n\t"	\
					"push	r25						\n\t"	\
					"push	r26						\n\t"	\
					"push	r27						\n\t"	\
					"push	r28						\n\t"	\
					"push	r29						\n\t"	\
					"push	r30						\n\t"	\
					"push	r31						\n\t"	\
					"lds	r26, pxCurrentTCB		\n\t"	\
					"lds	r27, pxCurrentTCB + 1	\n\t"	\
					"in		r0, 0x3d				\n\t"	\
					"st		x+, r0					\n\t"	\
					"in		r0, 0x3e				\n\t"	\
					"st		x+, r0					\n\t"	\
				);
#else
/* 2-Byte PC Save */
#define portSAVE_CONTEXT()									\
	asm volatile (	"push	r0						\n\t"	\
					"in		r0, __SREG__			\n\t"	\
					"cli							\n\t"	\
					"push	r0						\n\t"	\
					"push	r1						\n\t"	\
					"clr	r1						\n\t"	\
					"push	r2						\n\t"	\
					"push	r3						\n\t"	\
					"push	r4						\n\t"	\
					"push	r5						\n\t"	\
					"push	r6						\n\t"	\
					"push	r7						\n\t"	\
					"push	r8						\n\t"	\
					"push	r9						\n\t"	\
					"push	r10						\n\t"	\
					"push	r11						\n\t"	\
					"push	r12						\n\t"	\
					"push	r13						\n\t"	\
					"push	r14						\n\t"	\
					"push	r15						\n\t"	\
					"push	r16						\n\t"	\
					"push	r17						\n\t"	\
					"push	r18						\n\t"	\
					"push	r19						\n\t"	\
					"push	r20						\n\t"	\
					"push	r21						\n\t"	\
					"push	r22						\n\t"	\
					"push	r23						\n\t"	\
					"push	r24						\n\t"	\
					"push	r25						\n\t"	\
					"push	r26						\n\t"	\
					"push	r27						\n\t"	\
					"push	r28						\n\t"	\
					"push	r29						\n\t"	\
					"push	r30						\n\t"	\
					"push	r31						\n\t"	\
					"lds	r26, pxCurrentTCB		\n\t"	\
					"lds	r27, pxCurrentTCB + 1	\n\t"	\
					"in		r0, 0x3d				\n\t"	\
					"st		x+, r0					\n\t"	\
					"in		r0, 0x3e				\n\t"	\
					"st		x+, r0					\n\t"	\
				);
#endif




/*
 * Opposite to portSAVE_CONTEXT().  Interrupts will have been disabled during
 * the context save so we can write to the stack pointer.
 */
#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
/* 3-Byte PC Restore */
#define portRESTORE_CONTEXT()								\
	asm volatile (	"lds	r26, pxCurrentTCB		\n\t"	\
					"lds	r27, pxCurrentTCB + 1	\n\t"	\
					"ld		r28, x+					\n\t"	\
					"out	__SP_L__, r28			\n\t"	\
					"ld		r29, x+					\n\t"	\
					"out	__SP_H__, r29			\n\t"	\
					"pop	r31						\n\t"	\
					"pop	r30						\n\t"	\
					"pop	r29						\n\t"	\
					"pop	r28						\n\t"	\
					"pop	r27						\n\t"	\
					"pop	r26						\n\t"	\
					"pop	r25						\n\t"	\
					"pop	r24						\n\t"	\
					"pop	r23						\n\t"	\
					"pop	r22						\n\t"	\
					"pop	r21						\n\t"	\
					"pop	r20						\n\t"	\
					"pop	r19						\n\t"	\
					"pop	r18						\n\t"	\
					"pop	r17						\n\t"	\
					"pop	r16						\n\t"	\
					"pop	r15						\n\t"	\
					"pop	r14						\n\t"	\
					"pop	r13						\n\t"	\
					"pop	r12						\n\t"	\
					"pop	r11						\n\t"	\
					"pop	r10						\n\t"	\
					"pop	r9						\n\t"	\
					"pop	r8						\n\t"	\
					"pop	r7						\n\t"	\
					"pop	r6						\n\t"	\
					"pop	r5						\n\t"	\
					"pop	r4						\n\t"	\
					"pop	r3						\n\t"	\
					"pop	r2						\n\t"	\
					"pop	r1						\n\t"	\
					"pop	r0						\n\t"	\
					"out	0x3c, r0				\n\t"	\
					"pop	r0						\n\t"	\
					"out	0x3b, r0				\n\t"	\
					"pop	r0						\n\t"	\
					"out	__SREG__, r0			\n\t"	\
					"pop	r0						\n\t"	\
				);
#else
/* 2-Byte PC Restore */
#define portRESTORE_CONTEXT()								\
	asm volatile (	"lds	r26, pxCurrentTCB		\n\t"	\
					"lds	r27, pxCurrentTCB + 1	\n\t"	\
					"ld		r28, x+					\n\t"	\
					"out	__SP_L__, r28			\n\t"	\
					"ld		r29, x+					\n\t"	\
					"out	__SP_H__, r29			\n\t"	\
					"pop	r31						\n\t"	\
					"pop	r30						\n\t"	\
					"pop	r29						\n\t"	\
					"pop	r28						\n\t"	\
					"pop	r27						\n\t"	\
					"pop	r26						\n\t"	\
					"pop	r25						\n\t"	\
					"pop	r24						\n\t"	\
					"pop	r23						\n\t"	\
					"pop	r22						\n\t"	\
					"pop	r21						\n\t"	\
					"pop	r20						\n\t"	\
					"pop	r19						\n\t"	\
					"pop	r18						\n\t"	\
					"pop	r17						\n\t"	\
					"pop	r16						\n\t"	\
					"pop	r15						\n\t"	\
					"pop	r14						\n\t"	\
					"pop	r13						\n\t"	\
					"pop	r12						\n\t"	\
					"pop	r11						\n\t"	\
					"pop	r10						\n\t"	\
					"pop	r9						\n\t"	\
					"pop	r8						\n\t"	\
					"pop	r7						\n\t"	\
					"pop	r6						\n\t"	\
					"pop	r5						\n\t"	\
					"pop	r4						\n\t"	\
					"pop	r3						\n\t"	\
					"pop	r2						\n\t"	\
					"pop	r1						\n\t"	\
					"pop	r0						\n\t"	\
					"out	__SREG__, r0			\n\t"	\
					"pop	r0						\n\t"	\
				);
#endif
/*-----------------------------------------------------------*/

/*
 * Perform hardware setup to enable ticks from timer 0/3, compare match A.
 */
static void prvSetupTimerInterrupt( void );
/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
portSTACK_TYPE *pxPortInitialiseStack( portSTACK_TYPE *pxTopOfStack, pdTASK_CODE pxCode, void *pvParameters )
{
unsigned short usAddress;

#if defined (portQUAD_RAM) || defined (portMEGA_RAM)
	// This function just here to ensure the library is included.
	// Can go anywhere, as it is just to ensure that the .init3 code is included
	extRAMcheck(); // This function just returns XMCRA. SRE is set if extended RAM enabled.
#warning "Ext RAM Enabled."
#endif

	/* Place a few bytes of known values on the bottom of the stack.
	This is just useful for debugging. */

	*pxTopOfStack = 0x11;
	pxTopOfStack--;
	*pxTopOfStack = 0x22;
	pxTopOfStack--;
	*pxTopOfStack = 0x33;
	pxTopOfStack--;

	/* Simulate how the stack would look after a call to vPortYield() generated by
	the compiler. */

	/*lint -e950 -e611 -e923 Lint doesn't like this much - but nothing I can do about it. */

	/* The start of the task code will be popped off the stack last, so place
	it on first. */

#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
	/* The AVR ATmega2560/ATmega2561 have 256KBytes of program memory and a 17-bit
	 * program counter.  When a code address is stored on the stack, it takes 3 bytes
	 * instead of 2 for the other ATmega* chips.
	 *
	 * Store 0 as the top byte since we force all task routines to the bottom 128K
	 * of flash. We do this by adding the .task label in the linker script.
	 *
	 * In order to do this properly, we would need to get a full 3-byte pointer to
	 * pxCode.  That requires a change to GCC.  Not likely to happen any time soon.
	 */
	usAddress = ( unsigned portSHORT ) pxCode;
	*pxTopOfStack = ( portSTACK_TYPE ) ( usAddress & ( unsigned portSHORT ) 0x00ff );
	pxTopOfStack--;

	usAddress >>= 8;
	*pxTopOfStack = ( portSTACK_TYPE ) ( usAddress & ( unsigned portSHORT ) 0x00ff );
	pxTopOfStack--;

	*pxTopOfStack = 0;
	pxTopOfStack--;
#else
	usAddress = ( unsigned short ) pxCode;
	*pxTopOfStack = ( portSTACK_TYPE ) ( usAddress & ( unsigned short ) 0x00ff );
	pxTopOfStack--;

	usAddress >>= 8;
	*pxTopOfStack = ( portSTACK_TYPE ) ( usAddress & ( unsigned short ) 0x00ff );
	pxTopOfStack--;
#endif

	/* Next simulate the stack as if after a call to portSAVE_CONTEXT().
	portSAVE_CONTEXT places the flags on the stack immediately after r0
	to ensure the interrupts get disabled as soon as possible, and so ensuring
	the stack use is minimal should a context switch interrupt occur. */
	*pxTopOfStack = ( portSTACK_TYPE ) 0x00;	/* R0 */
	pxTopOfStack--;
	*pxTopOfStack = portFLAGS_INT_ENABLED;
	pxTopOfStack--;

#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)

	/* If we have an ATmega256x, we are also saving the RAMPZ and EIND registers.
	 * We should default those to 0.
	 */
	*pxTopOfStack = ( portSTACK_TYPE ) 0x00;	/* EIND */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x00;	/* RAMPZ */
	pxTopOfStack--;

#endif

	/* Now the remaining registers.   The compiler expects R1 to be 0. */
	*pxTopOfStack = ( portSTACK_TYPE ) 0x00;	/* R1 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x02;	/* R2 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x03;	/* R3 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x04;	/* R4 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x05;	/* R5 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x06;	/* R6 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x07;	/* R7 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x08;	/* R8 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x09;	/* R9 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x10;	/* R10 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x11;	/* R11 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x12;	/* R12 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x13;	/* R13 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x14;	/* R14 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x15;	/* R15 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x16;	/* R16 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x17;	/* R17 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x18;	/* R18 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x19;	/* R19 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x20;	/* R20 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x21;	/* R21 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x22;	/* R22 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x23;	/* R23 */
	pxTopOfStack--;

	/* Place the parameter on the stack in the expected location. */
	usAddress = ( unsigned short ) pvParameters;
	*pxTopOfStack = ( portSTACK_TYPE ) ( usAddress & ( unsigned short ) 0x00ff );
	pxTopOfStack--;

	usAddress >>= 8;
	*pxTopOfStack = ( portSTACK_TYPE ) ( usAddress & ( unsigned short ) 0x00ff );
	pxTopOfStack--;

	*pxTopOfStack = ( portSTACK_TYPE ) 0x26;	/* R26 X */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x27;	/* R27 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x28;	/* R28 Y */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x29;	/* R29 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x30;	/* R30 Z */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x031;	/* R31 */
	pxTopOfStack--;

	/*lint +e950 +e611 +e923 */

	return pxTopOfStack;
}
/*-----------------------------------------------------------*/

portBASE_TYPE xPortStartScheduler( void )
{
	/* Setup the hardware to generate the tick. */
	prvSetupTimerInterrupt();

	/* Restore the context of the first task that is going to run. */
	portRESTORE_CONTEXT();

	/* Simulate a function call end as generated by the compiler.  We will now
	jump to the start of the task the context of which we have just restored. */
	asm volatile ( "ret" );

	/* Should not get here. */
	return pdTRUE;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
	/* It is unlikely that the AVR port will get stopped.  If required simply
	disable the tick interrupt here. */
}
/*-----------------------------------------------------------*/

/*
 * Manual context switch.  The first thing we do is save the registers so we
 * can use a naked attribute.
 */
void vPortYield( void ) __attribute__ ( ( naked ) );
void vPortYield( void )
{
	portSAVE_CONTEXT();
	vTaskSwitchContext();
	portRESTORE_CONTEXT();

	asm volatile ( "ret" );
}
/*-----------------------------------------------------------*/

/*
 * Context switch function used by the tick.  This must be identical to
 * vPortYield() from the call to vTaskSwitchContext() onwards.  The only
 * difference from vPortYield() is the tick count is incremented as the
 * call comes from the tick ISR.
 */
void vPortYieldFromTick( void ) __attribute__ ( ( naked ) );
void vPortYieldFromTick( void )
{
	portSAVE_CONTEXT();
	vTaskIncrementTick();
	vTaskSwitchContext();
	portRESTORE_CONTEXT();

	asm volatile ( "ret" );
}
/*-----------------------------------------------------------*/

/*
 * Setup timer 0 or 3 or 1 compare match A to generate a tick interrupt.
 */
static void prvSetupTimerInterrupt( void )
{
unsigned portLONG ulCompareMatch;
#ifdef portOCRH
unsigned portCHAR ucHighByte;
#endif
unsigned portCHAR ucLowByte;

    /* Using 8bit Timer0 or 16bit Timer1 or Timer3 to generate the tick. */

    // ulCompareMatch 40,000 = 20,000,000 / 500; 20MHz
    // ulCompareMatch 110,592 = 22,118,400 / 200; 22.1184 MHz
    ulCompareMatch = configCPU_CLOCK_HZ / configTICK_RATE_HZ;

    /* We only have 8 or 16 bits so have to scale 64 or 256 to get our required tick rate. */
    //ulCompareMatch = 625 /= portCLOCK_PRESCALER; 20MHz with 64 prescale
    //ulCompareMatch = 108 /= portCLOCK_PRESCALER; 22.1184 MHz with 1024 prescale
    ulCompareMatch /= portCLOCK_PRESCALER;

    /* Adjust for correct value. */
    ulCompareMatch -= ( unsigned portLONG ) 1;

    /* Setup compare match value for compare match A.  Interrupts are disabled
    before this is called so we need not worry here. */
    ucLowByte = ( unsigned portCHAR ) ( ulCompareMatch & ( unsigned portLONG ) 0xff );

    //  OCR3AH = ucHighByte;
    //  OCR3AL = ucLowByte;

    // the HiByte is only needed, if a 16 Bit counter is being utilized
#ifdef portOCRH

    ulCompareMatch >>= 8;
    ucHighByte = ( unsigned portCHAR ) ( ulCompareMatch & ( unsigned portLONG ) 0xff );
    portOCRH = ucHighByte;

#endif

    portOCRL = ucLowByte;

#if defined( portUSE_TIMER0 )
   /* Setup clock source and compare match behaviour. Assuming 328p (no Timer3) */
   portTCCRa = portCLEAR_COUNTER_ON_MATCH;
   portTCCRb = portPRESCALE_1024;

#elif defined( portUSE_TIMER1 )
	/* Setup clock source and compare match behaviour. Assuming 328p (with Timer1) */
	ucLowByte = portCLEAR_COUNTER_ON_MATCH | portPRESCALE_64;
	portTCCRb = ucLowByte;

#elif defined( portUSE_TIMER3 )
	/* Setup clock source and compare match behaviour. Assuming  640 / 1280 /1281 / 1284p / 2560 / 2561 (with Timer3) */
	ucLowByte = portCLEAR_COUNTER_ON_MATCH | portPRESCALE_64;
	portTCCRb = ucLowByte;
#endif

    /* Enable the interrupt - this is okay as interrupt are currently globally
	disabled. */
    ucLowByte = portTIMSK;
    ucLowByte |= portCOMPARE_MATCH_A_INTERRUPT_ENABLE;
    portTIMSK = ucLowByte;

}

/*-----------------------------------------------------------*/

#if configUSE_PREEMPTION == 1

	/*
	 * Tick ISR for preemptive scheduler.  We can use a naked attribute as
	 * the context is saved at the start of vPortYieldFromTick().  The tick
	 * count is incremented after the context is saved.
	 */
	#if defined( portUSE_TIMER0 )
		#warning "Timer0 used for PRE-EMPTIVE scheduler."
		ISR(TIMER0_COMPA_vect, ISR_NAKED)
		{
			vPortYieldFromTick();
			asm volatile ( "reti" );
		}

	#elif defined( portUSE_TIMER1 )
		#warning "Timer1 used for PRE-EMPTIVE scheduler."
		ISR(TIMER1_COMPA_vect, ISR_NAKED)
		{
			vPortYieldFromTick();
			asm volatile ( "reti" );
		}

	#elif defined( portUSE_TIMER3 )
		#warning "Timer3 used for PRE-EMPTIVE scheduler."
		ISR(TIMER3_COMPA_vect, ISR_NAKED)
		{
			vPortYieldFromTick();
			asm volatile ( "reti" );
		}
	#endif

#else

	/*
	 * Tick ISR for the cooperative scheduler.  All this does is increment the
	 * tick count.  We don't need to switch context, this can only be done by
	 * manual calls to taskYIELD();
	 */

	#if defined( portUSE_TIMER0 )
        #warning "Timer0 used for COOPERATIVE scheduler."
        ISR(TIMER0_COMPA_vect, ISR_NAKED)
        {
	    	vTaskIncrementTick();
        }

	#elif defined( portUSE_TIMER1 )
		#warning "Timer1 used for COOPERATIVE scheduler."
		ISR(TIMER1_COMPA_vect, ISR_NAKED)
		{
			vTaskIncrementTick();
		}

	#elif defined( portUSE_TIMER3 )
		#warning "Timer3 used for COOPERATIVE scheduler."
		ISR(TIMER3_COMPA_vect, ISR_NAKED)
		{
			vTaskIncrementTick();
		}

	#endif // portUSE_TIMER1

#endif // portUSE_PREEMPTION




