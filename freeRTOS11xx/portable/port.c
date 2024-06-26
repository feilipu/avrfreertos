/*
 * FreeRTOS Kernel V11.1.0
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
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
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */


#include <stdlib.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include "FreeRTOS.h"
#include "task.h"

#include "time.h"         // Needed for system_tick();

#if defined (portQUAD_RAM) || defined (portMEGA_RAM)
#include "ext_ram.h"    // Needed for extRAMcheck();
#endif

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the AVR port.
 *----------------------------------------------------------*/

/* Start tasks with interrupts enabled. */
#define portFLAGS_INT_ENABLED           ( ( StackType_t ) 0x80 )

#if defined( portUSE_WDTO )
    #warning "Watchdog Timer used for scheduler."
    #define portSCHEDULER_ISR           WDT_vect

#elif defined( portUSE_TIMER0 )
/* Hardware constants for Timer0. */
    #warning "Timer0 used for scheduler."
    #define portSCHEDULER_ISR           TIMER0_COMPA_vect
    #define portCLEAR_COUNTER_ON_MATCH  ( (uint8_t) _BV(WGM01) )
    #define portPRESCALE_1024           ( (uint8_t) (_BV(CS02)|_BV(CS00)) )
    #define portCLOCK_PRESCALER         ( (uint32_t) 1024 )
    #define portCOMPARE_MATCH_A_INTERRUPT_ENABLE    ( (uint8_t) _BV(OCIE0A) )
    #define portOCRL                    OCR0A
    #define portTCCRa                   TCCR0A
    #define portTCCRb                   TCCR0B
    #define portTIMSK                   TIMSK0
    #define portTIFR                    TIFR0

#elif defined( portUSE_TIMER1 )
/* Hardware constants for Timer1. */
    #warning "Timer1 used for scheduler."
    #define    portSCHEDULER_ISR        TIMER1_COMPA_vect
    #define portCLEAR_COUNTER_ON_MATCH  ( (uint8_t) _BV(WGM12) )
    #define portPRESCALE_64             ( (uint8_t) (_BV(CS11)|_BV(CS10)) )
    #define portCLOCK_PRESCALER         ( (uint32_t) 64 )
    #define portCOMPARE_MATCH_A_INTERRUPT_ENABLE    ( (uint8_t) _BV(OCIE1A) )
    #define portOCRL                    OCR1AL
    #define portOCRH                    OCR1AH
    #define portTCCRa                   TCCR1A
    #define portTCCRb                   TCCR1B
    #define portTIMSK                   TIMSK1
    #define portTIFR                    TIFR1

#elif defined( portUSE_TIMER2 )
/* Hardware constants for Timer2. */
    #warning "Timer2 used for scheduler."
    #define    portSCHEDULER_ISR        TIMER2_COMPA_vect
    #define portCOMPARE_MATCH_A_INTERRUPT_ENABLE    ( (uint8_t) _BV(OCIE2A) )
    #define portOCRL                    OCR2A
    #define portTCCRa                   TCCR2A
    #define portTCCRb                   TCCR2B
    #define portTIMSK                   TIMSK2
    #define portTCNT                    TCNT2
    #define portTIFR                    TIFR2

#elif defined( portUSE_TIMER3 )
/* Hardware constants for Timer3. */
    #warning "Timer3 used for scheduler."
    #define    portSCHEDULER_ISR        TIMER3_COMPA_vect
    #define portCLEAR_COUNTER_ON_MATCH  ( (uint8_t) _BV(WGM32) )
    #define portPRESCALE_64             ( (uint8_t) (_BV(CS31)|_BV(CS30)) )
    #define portCLOCK_PRESCALER         ( (uint32_t) 64 )
    #define portCOMPARE_MATCH_A_INTERRUPT_ENABLE    ( (uint8_t) _BV(OCIE3A) )
    #define portOCRL                    OCR3AL
    #define portOCRH                    OCR3AH
    #define portTCCRa                   TCCR3A
    #define portTCCRb                   TCCR3B
    #define portTIMSK                   TIMSK3
    #define portTIFR                    TIFR3

#endif

/*-----------------------------------------------------------*/

/* We require the address of the pxCurrentTCB variable, but don't want to know
 * any details of its type. */
typedef void TCB_t;
extern volatile TCB_t * volatile pxCurrentTCB;

/*-----------------------------------------------------------*/

/**
 *  Enable the watchdog timer, configuring it for expire after
 *  (value) timeout (which is a combination of the WDP0
 *  through WDP3 bits).
 *
 *  This function is derived from <avr/wdt.h> but enables only
 *  the interrupt bit (WDIE), rather than the reset bit (WDE).
 *
 *  Can't find it documented but the WDT, once enabled,
 *  rolls over and fires a new interrupt each time.
 *
 *  See also the symbolic constants WDTO_15MS et al.
 *
 *  Updated to match avr-libc 2.0.0
 */

#if defined( portUSE_WDTO )

static __inline__
__attribute__ ((__always_inline__))
void wdt_interrupt_enable (const uint8_t value)
{
    if (_SFR_IO_REG_P (_WD_CONTROL_REG))
    {
        __asm__ __volatile__ (
                "in __tmp_reg__,__SREG__"   "\n\t"
                "cli"                       "\n\t"
                "wdr"                       "\n\t"
                "out %0, %1"                "\n\t"
                "out __SREG__,__tmp_reg__"  "\n\t"
                "out %0, %2"                "\n\t"
                : /* no outputs */
                : "I" (_SFR_IO_ADDR(_WD_CONTROL_REG)),
                "r" ((uint8_t)(_BV(_WD_CHANGE_BIT) | _BV(WDE))),
                "r" ((uint8_t) ((value & 0x08 ? _WD_PS3_MASK : 0x00) |
                        _BV(WDIF) | _BV(WDIE) | (value & 0x07)) )
                : "r0"
        );
    }
    else
    {
        __asm__ __volatile__ (
                "in __tmp_reg__,__SREG__"   "\n\t"
                "cli"                       "\n\t"
                "wdr"                       "\n\t"
                "sts %0, %1"                "\n\t"
                "out __SREG__,__tmp_reg__"  "\n\t"
                "sts %0, %2"                "\n\t"
                : /* no outputs */
                : "n" (_SFR_MEM_ADDR(_WD_CONTROL_REG)),
                "r" ((uint8_t)(_BV(_WD_CHANGE_BIT) | _BV(WDE))),
                "r" ((uint8_t) ((value & 0x08 ? _WD_PS3_MASK : 0x00) |
                        _BV(WDIF) | _BV(WDIE) | (value & 0x07)) )
                : "r0"
        );
    }
}
#endif /* if defined( portUSE_WDTO ) */

/*-----------------------------------------------------------*/

/**
 *  Enable the watchdog timer, configuring it for expire after
 *  (value) timeout (which is a combination of the WDP0
 *  through WDP3 bits).
 *
 *  This function is derived from <avr/wdt.h> but enables both
 *  the reset bit (WDE), and the interrupt bit (WDIE).
 *
 *  This will ensure that if the interrupt is not serviced
 *  before the second timeout, the AVR will reset.
 *
 *  Servicing the interrupt automatically clears it,
 *  and ensures the AVR does not reset.
 *
 *  Can't find it documented but the WDT, once enabled,
 *  rolls over and fires a new interrupt each time.
 *
 *  See also the symbolic constants WDTO_15MS et al.
 *
 *  Updated to match avr-libc 2.0.0
 */

#if defined( portUSE_WDTO )

static __inline__
__attribute__ ((__always_inline__))
void wdt_interrupt_reset_enable (const uint8_t value)
{
    if (_SFR_IO_REG_P (_WD_CONTROL_REG))
    {
        __asm__ __volatile__ (
                "in __tmp_reg__,__SREG__"   "\n\t"
                "cli"                       "\n\t"
                "wdr"                       "\n\t"
                "out %0, %1"                "\n\t"
                "out __SREG__,__tmp_reg__"  "\n\t"
                "out %0, %2"                "\n\t"
                : /* no outputs */
                : "I" (_SFR_IO_ADDR(_WD_CONTROL_REG)),
                "r" ((uint8_t)(_BV(_WD_CHANGE_BIT) | _BV(WDE))),
                "r" ((uint8_t) ((value & 0x08 ? _WD_PS3_MASK : 0x00) |
                        _BV(WDIF) | _BV(WDIE) | _BV(WDE) | (value & 0x07)) )
                : "r0"
        );
    }
    else
    {
        __asm__ __volatile__ (
                "in __tmp_reg__,__SREG__"   "\n\t"
                "cli"                       "\n\t"
                "wdr"                       "\n\t"
                "sts %0, %1"                "\n\t"
                "out __SREG__,__tmp_reg__"  "\n\t"
                "sts %0, %2"                "\n\t"
                : /* no outputs */
                : "n" (_SFR_MEM_ADDR(_WD_CONTROL_REG)),
                "r" ((uint8_t)(_BV(_WD_CHANGE_BIT) | _BV(WDE))),
                "r" ((uint8_t) ((value & 0x08 ? _WD_PS3_MASK : 0x00) |
                        _BV(WDIF) | _BV(WDIE) | _BV(WDE) | (value & 0x07)) )
                : "r0"
        );
    }
}
#endif /* if defined( portUSE_WDTO ) */

/*-----------------------------------------------------------*/
/* actual number of ticks per second, after configuration. Not for RTC, which has 1 tick/second. */
TickType_t portTickRateHz;

/* remaining ticks in each second, decremented to enable the system_tick. Not for RTC, which has 1 tick/second. */
volatile TickType_t ticksRemainingInSec;

/*-----------------------------------------------------------*/

/*
 * Macro to save all the general purpose registers, the save the stack pointer
 * into the TCB.
 *
 * The first thing we do is save the flags then disable interrupts. This is to
 * guard our stack against having a context switch interrupt after we have already
 * pushed the registers onto the stack - causing the 32 registers to be on the
 * stack twice.
 *
 * r1 is set to zero (__zero_reg__) as the compiler expects it to be thus, however
 * some of the math routines make use of R1.
 *
 * r0 is set to __tmp_reg__ as the compiler expects it to be thus.
 *
 * #if defined(__AVR_HAVE_RAMPZ__)
 * #define __RAMPZ__ 0x3B
 * #endif
 *
 * #if defined(__AVR_3_BYTE_PC__)
 * #define __EIND__ 0x3C
 * #endif
 *
 * The interrupts will have been disabled during the call to portSAVE_CONTEXT()
 * so we need not worry about reading/writing to the stack pointer.
 */
#if defined(__AVR_3_BYTE_PC__) && defined(__AVR_HAVE_RAMPZ__)
/* 3-Byte PC Save  with RAMPZ */
#define portSAVE_CONTEXT()                                                              \
        __asm__ __volatile__ (  "push   __tmp_reg__                             \n\t"   \
                                "in     __tmp_reg__, __SREG__                   \n\t"   \
                                "cli                                            \n\t"   \
                                "push   __tmp_reg__                             \n\t"   \
                                "in     __tmp_reg__, 0x3B                       \n\t"   \
                                "push   __tmp_reg__                             \n\t"   \
                                "in     __tmp_reg__, 0x3C                       \n\t"   \
                                "push   __tmp_reg__                             \n\t"   \
                                "push   __zero_reg__                            \n\t"   \
                                "clr    __zero_reg__                            \n\t"   \
                                "push   r2                                      \n\t"   \
                                "push   r3                                      \n\t"   \
                                "push   r4                                      \n\t"   \
                                "push   r5                                      \n\t"   \
                                "push   r6                                      \n\t"   \
                                "push   r7                                      \n\t"   \
                                "push   r8                                      \n\t"   \
                                "push   r9                                      \n\t"   \
                                "push   r10                                     \n\t"   \
                                "push   r11                                     \n\t"   \
                                "push   r12                                     \n\t"   \
                                "push   r13                                     \n\t"   \
                                "push   r14                                     \n\t"   \
                                "push   r15                                     \n\t"   \
                                "push   r16                                     \n\t"   \
                                "push   r17                                     \n\t"   \
                                "push   r18                                     \n\t"   \
                                "push   r19                                     \n\t"   \
                                "push   r20                                     \n\t"   \
                                "push   r21                                     \n\t"   \
                                "push   r22                                     \n\t"   \
                                "push   r23                                     \n\t"   \
                                "push   r24                                     \n\t"   \
                                "push   r25                                     \n\t"   \
                                "push   r26                                     \n\t"   \
                                "push   r27                                     \n\t"   \
                                "push   r28                                     \n\t"   \
                                "push   r29                                     \n\t"   \
                                "push   r30                                     \n\t"   \
                                "push   r31                                     \n\t"   \
                                "lds    r26, pxCurrentTCB                       \n\t"   \
                                "lds    r27, pxCurrentTCB + 1                   \n\t"   \
                                "in     __tmp_reg__, __SP_L__                   \n\t"   \
                                "st     x+, __tmp_reg__                         \n\t"   \
                                "in     __tmp_reg__, __SP_H__                   \n\t"   \
                                "st     x+, __tmp_reg__                         \n\t"   \
                             );
#elif defined(__AVR_HAVE_RAMPZ__)
/* 2-Byte PC Save  with RAMPZ */
#define portSAVE_CONTEXT()                                                              \
        __asm__ __volatile__ (  "push   __tmp_reg__                             \n\t"   \
                                "in     __tmp_reg__, __SREG__                   \n\t"   \
                                "cli                                            \n\t"   \
                                "push   __tmp_reg__                             \n\t"   \
                                "in     __tmp_reg__, 0x3B                       \n\t"   \
                                "push   __tmp_reg__                             \n\t"   \
                                "push   __zero_reg__                            \n\t"   \
                                "clr    __zero_reg__                            \n\t"   \
                                "push   r2                                      \n\t"   \
                                "push   r3                                      \n\t"   \
                                "push   r4                                      \n\t"   \
                                "push   r5                                      \n\t"   \
                                "push   r6                                      \n\t"   \
                                "push   r7                                      \n\t"   \
                                "push   r8                                      \n\t"   \
                                "push   r9                                      \n\t"   \
                                "push   r10                                     \n\t"   \
                                "push   r11                                     \n\t"   \
                                "push   r12                                     \n\t"   \
                                "push   r13                                     \n\t"   \
                                "push   r14                                     \n\t"   \
                                "push   r15                                     \n\t"   \
                                "push   r16                                     \n\t"   \
                                "push   r17                                     \n\t"   \
                                "push   r18                                     \n\t"   \
                                "push   r19                                     \n\t"   \
                                "push   r20                                     \n\t"   \
                                "push   r21                                     \n\t"   \
                                "push   r22                                     \n\t"   \
                                "push   r23                                     \n\t"   \
                                "push   r24                                     \n\t"   \
                                "push   r25                                     \n\t"   \
                                "push   r26                                     \n\t"   \
                                "push   r27                                     \n\t"   \
                                "push   r28                                     \n\t"   \
                                "push   r29                                     \n\t"   \
                                "push   r30                                     \n\t"   \
                                "push   r31                                     \n\t"   \
                                "lds    r26, pxCurrentTCB                       \n\t"   \
                                "lds    r27, pxCurrentTCB + 1                   \n\t"   \
                                "in     __tmp_reg__, __SP_L__                   \n\t"   \
                                "st     x+, __tmp_reg__                         \n\t"   \
                                "in     __tmp_reg__, __SP_H__                   \n\t"   \
                                "st     x+, __tmp_reg__                         \n\t"   \
                             );
#else /* if defined( __AVR_3_BYTE_PC__ ) && defined( __AVR_HAVE_RAMPZ__ ) */
/* 2-Byte PC Save */
#define portSAVE_CONTEXT()                                                              \
        __asm__ __volatile__ (  "push   __tmp_reg__                             \n\t"   \
                                "in     __tmp_reg__, __SREG__                   \n\t"   \
                                "cli                                            \n\t"   \
                                "push   __tmp_reg__                             \n\t"   \
                                "push   __zero_reg__                            \n\t"   \
                                "clr    __zero_reg__                            \n\t"   \
                                "push   r2                                      \n\t"   \
                                "push   r3                                      \n\t"   \
                                "push   r4                                      \n\t"   \
                                "push   r5                                      \n\t"   \
                                "push   r6                                      \n\t"   \
                                "push   r7                                      \n\t"   \
                                "push   r8                                      \n\t"   \
                                "push   r9                                      \n\t"   \
                                "push   r10                                     \n\t"   \
                                "push   r11                                     \n\t"   \
                                "push   r12                                     \n\t"   \
                                "push   r13                                     \n\t"   \
                                "push   r14                                     \n\t"   \
                                "push   r15                                     \n\t"   \
                                "push   r16                                     \n\t"   \
                                "push   r17                                     \n\t"   \
                                "push   r18                                     \n\t"   \
                                "push   r19                                     \n\t"   \
                                "push   r20                                     \n\t"   \
                                "push   r21                                     \n\t"   \
                                "push   r22                                     \n\t"   \
                                "push   r23                                     \n\t"   \
                                "push   r24                                     \n\t"   \
                                "push   r25                                     \n\t"   \
                                "push   r26                                     \n\t"   \
                                "push   r27                                     \n\t"   \
                                "push   r28                                     \n\t"   \
                                "push   r29                                     \n\t"   \
                                "push   r30                                     \n\t"   \
                                "push   r31                                     \n\t"   \
                                "lds    r26, pxCurrentTCB                       \n\t"   \
                                "lds    r27, pxCurrentTCB + 1                   \n\t"   \
                                "in     __tmp_reg__, __SP_L__                   \n\t"   \
                                "st     x+, __tmp_reg__                         \n\t"   \
                                "in     __tmp_reg__, __SP_H__                   \n\t"   \
                                "st     x+, __tmp_reg__                         \n\t"   \
                             );
#endif /* if defined( __AVR_3_BYTE_PC__ ) && defined( __AVR_HAVE_RAMPZ__ ) */

/*
 * Opposite to portSAVE_CONTEXT().  Interrupts will have been disabled during
 * the context save so we can write to the stack pointer.
 */
#if defined(__AVR_3_BYTE_PC__) && defined(__AVR_HAVE_RAMPZ__)
/* 3-Byte PC Restore with RAMPZ */
#define portRESTORE_CONTEXT()                                                           \
        __asm__ __volatile__ (  "lds    r26, pxCurrentTCB                       \n\t"   \
                                "lds    r27, pxCurrentTCB + 1                   \n\t"   \
                                "ld     r28, x+                                 \n\t"   \
                                "out    __SP_L__, r28                           \n\t"   \
                                "ld     r29, x+                                 \n\t"   \
                                "out    __SP_H__, r29                           \n\t"   \
                                "pop    r31                                     \n\t"   \
                                "pop    r30                                     \n\t"   \
                                "pop    r29                                     \n\t"   \
                                "pop    r28                                     \n\t"   \
                                "pop    r27                                     \n\t"   \
                                "pop    r26                                     \n\t"   \
                                "pop    r25                                     \n\t"   \
                                "pop    r24                                     \n\t"   \
                                "pop    r23                                     \n\t"   \
                                "pop    r22                                     \n\t"   \
                                "pop    r21                                     \n\t"   \
                                "pop    r20                                     \n\t"   \
                                "pop    r19                                     \n\t"   \
                                "pop    r18                                     \n\t"   \
                                "pop    r17                                     \n\t"   \
                                "pop    r16                                     \n\t"   \
                                "pop    r15                                     \n\t"   \
                                "pop    r14                                     \n\t"   \
                                "pop    r13                                     \n\t"   \
                                "pop    r12                                     \n\t"   \
                                "pop    r11                                     \n\t"   \
                                "pop    r10                                     \n\t"   \
                                "pop    r9                                      \n\t"   \
                                "pop    r8                                      \n\t"   \
                                "pop    r7                                      \n\t"   \
                                "pop    r6                                      \n\t"   \
                                "pop    r5                                      \n\t"   \
                                "pop    r4                                      \n\t"   \
                                "pop    r3                                      \n\t"   \
                                "pop    r2                                      \n\t"   \
                                "pop    __zero_reg__                            \n\t"   \
                                "pop    __tmp_reg__                             \n\t"   \
                                "out    0x3C, __tmp_reg__                       \n\t"   \
                                "pop    __tmp_reg__                             \n\t"   \
                                "out    0x3B, __tmp_reg__                       \n\t"   \
                                "pop    __tmp_reg__                             \n\t"   \
                                "out    __SREG__, __tmp_reg__                   \n\t"   \
                                "pop    __tmp_reg__                             \n\t"   \
                             );
#elif defined(__AVR_HAVE_RAMPZ__)
/* 2-Byte PC Restore with RAMPZ */
#define portRESTORE_CONTEXT()                                                           \
        __asm__ __volatile__ (  "lds    r26, pxCurrentTCB                       \n\t"   \
                                "lds    r27, pxCurrentTCB + 1                   \n\t"   \
                                "ld     r28, x+                                 \n\t"   \
                                "out    __SP_L__, r28                           \n\t"   \
                                "ld     r29, x+                                 \n\t"   \
                                "out    __SP_H__, r29                           \n\t"   \
                                "pop    r31                                     \n\t"   \
                                "pop    r30                                     \n\t"   \
                                "pop    r29                                     \n\t"   \
                                "pop    r28                                     \n\t"   \
                                "pop    r27                                     \n\t"   \
                                "pop    r26                                     \n\t"   \
                                "pop    r25                                     \n\t"   \
                                "pop    r24                                     \n\t"   \
                                "pop    r23                                     \n\t"   \
                                "pop    r22                                     \n\t"   \
                                "pop    r21                                     \n\t"   \
                                "pop    r20                                     \n\t"   \
                                "pop    r19                                     \n\t"   \
                                "pop    r18                                     \n\t"   \
                                "pop    r17                                     \n\t"   \
                                "pop    r16                                     \n\t"   \
                                "pop    r15                                     \n\t"   \
                                "pop    r14                                     \n\t"   \
                                "pop    r13                                     \n\t"   \
                                "pop    r12                                     \n\t"   \
                                "pop    r11                                     \n\t"   \
                                "pop    r10                                     \n\t"   \
                                "pop    r9                                      \n\t"   \
                                "pop    r8                                      \n\t"   \
                                "pop    r7                                      \n\t"   \
                                "pop    r6                                      \n\t"   \
                                "pop    r5                                      \n\t"   \
                                "pop    r4                                      \n\t"   \
                                "pop    r3                                      \n\t"   \
                                "pop    r2                                      \n\t"   \
                                "pop    __zero_reg__                            \n\t"   \
                                "pop    __tmp_reg__                             \n\t"   \
                                "out    0x3B, __tmp_reg__                       \n\t"   \
                                "pop    __tmp_reg__                             \n\t"   \
                                "out    __SREG__, __tmp_reg__                   \n\t"   \
                                "pop    __tmp_reg__                             \n\t"   \
                             );
#else /* if defined( __AVR_3_BYTE_PC__ ) && defined( __AVR_HAVE_RAMPZ__ ) */
/* 2-Byte PC Restore */
#define portRESTORE_CONTEXT()                                                           \
        __asm__ __volatile__ (  "lds    r26, pxCurrentTCB                       \n\t"   \
                                "lds    r27, pxCurrentTCB + 1                   \n\t"   \
                                "ld     r28, x+                                 \n\t"   \
                                "out    __SP_L__, r28                           \n\t"   \
                                "ld     r29, x+                                 \n\t"   \
                                "out    __SP_H__, r29                           \n\t"   \
                                "pop    r31                                     \n\t"   \
                                "pop    r30                                     \n\t"   \
                                "pop    r29                                     \n\t"   \
                                "pop    r28                                     \n\t"   \
                                "pop    r27                                     \n\t"   \
                                "pop    r26                                     \n\t"   \
                                "pop    r25                                     \n\t"   \
                                "pop    r24                                     \n\t"   \
                                "pop    r23                                     \n\t"   \
                                "pop    r22                                     \n\t"   \
                                "pop    r21                                     \n\t"   \
                                "pop    r20                                     \n\t"   \
                                "pop    r19                                     \n\t"   \
                                "pop    r18                                     \n\t"   \
                                "pop    r17                                     \n\t"   \
                                "pop    r16                                     \n\t"   \
                                "pop    r15                                     \n\t"   \
                                "pop    r14                                     \n\t"   \
                                "pop    r13                                     \n\t"   \
                                "pop    r12                                     \n\t"   \
                                "pop    r11                                     \n\t"   \
                                "pop    r10                                     \n\t"   \
                                "pop    r9                                      \n\t"   \
                                "pop    r8                                      \n\t"   \
                                "pop    r7                                      \n\t"   \
                                "pop    r6                                      \n\t"   \
                                "pop    r5                                      \n\t"   \
                                "pop    r4                                      \n\t"   \
                                "pop    r3                                      \n\t"   \
                                "pop    r2                                      \n\t"   \
                                "pop    __zero_reg__                            \n\t"   \
                                "pop    __tmp_reg__                             \n\t"   \
                                "out    __SREG__, __tmp_reg__                   \n\t"   \
                                "pop    __tmp_reg__                             \n\t"   \
                             );
#endif /* if defined( __AVR_3_BYTE_PC__ ) && defined( __AVR_HAVE_RAMPZ__ ) */
/*-----------------------------------------------------------*/

/*
 * Perform hardware setup to enable ticks from relevant Timer.
 */
void prvSetupTimerInterrupt( void );

#if defined(portUSE_TIMER2_RTC) && !defined(portUSE_TIMER2)
/*
 * Perform hardware setup to enable 1 second sys_ticks() from RTC Timer2.

 */
void prvSetupRTCInterrupt( void );
#endif
/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
StackType_t * pxPortInitialiseStack( StackType_t * pxTopOfStack,
                                     TaskFunction_t pxCode,
                                     void * pvParameters )
{
uint16_t usAddress;

#if defined (portQUAD_RAM) || defined (portMEGA_RAM)
    // This function just here to ensure the library is included.
    // Can go anywhere, as it is just to ensure that the .init3 code is included
    extRAMcheck(); // This function just returns XMCRA. SRE is set if extended RAM enabled.
#warning "Ext RAM Enabled."
#endif

    /* Simulate how the stack would look after a call to vPortYield() generated by
     * the compiler. */

    /* The start of the task code will be popped off the stack last, so place
     * it on first. */
    usAddress = ( uint16_t ) pxCode;
    *pxTopOfStack = ( StackType_t ) ( usAddress & ( uint16_t ) 0x00ff );
    pxTopOfStack--;

    usAddress >>= 8;
    *pxTopOfStack = ( StackType_t ) ( usAddress & ( uint16_t ) 0x00ff );
    pxTopOfStack--;

#if defined(__AVR_3_BYTE_PC__)

    /* The AVR ATmega2560/ATmega2561 have 256KBytes of program memory and a 17-bit
     * program counter. When a code address is stored on the stack, it takes 3 bytes
     * instead of 2 for the other ATmega* chips.
     *
     * Store 0 as the top byte since we force all task routines to the bottom 128K
     * of flash. We do this by using the .lowtext label in the linker script.
     *
     * In order to do this properly, we would need to get a full 3-byte pointer to
     * pxCode. That requires a change to GCC. Not likely to happen any time soon.
     */
    *pxTopOfStack = 0;
    pxTopOfStack--;
#endif

    /* Next simulate the stack as if after a call to portSAVE_CONTEXT().
     * portSAVE_CONTEXT places the flags on the stack immediately after r0
     * to ensure the interrupts get disabled as soon as possible, and so ensuring
     *  the stack use is minimal should a context switch interrupt occur. */
    *pxTopOfStack = ( StackType_t ) 0x00; /* R0 */
    pxTopOfStack--;
    *pxTopOfStack = portFLAGS_INT_ENABLED;
    pxTopOfStack--;

#if defined(__AVR_3_BYTE_PC__)

    /* If we have an ATmega256x, we are also saving the EIND register.
     * We should default to 0.
     */
    *pxTopOfStack = ( StackType_t ) 0x00;    /* EIND */
    pxTopOfStack--;
#endif

#if defined(__AVR_HAVE_RAMPZ__)

    /* We are saving the RAMPZ register.
     * We should default to 0.
     */
    *pxTopOfStack = ( StackType_t ) 0x00;    /* RAMPZ */
    pxTopOfStack--;
#endif

    /* Now the remaining registers. The compiler expects R1 to be 0. */
    *pxTopOfStack = ( StackType_t ) 0x00;    /* R1 */

    /* Leave R2 - R23 untouched */
    pxTopOfStack -= 23;

    /* Place the parameter on the stack in the expected location. */
    usAddress = ( uint16_t ) pvParameters;
    *pxTopOfStack = ( StackType_t ) ( usAddress & ( uint16_t ) 0x00ff );
    pxTopOfStack--;

    usAddress >>= 8;
    *pxTopOfStack = ( StackType_t ) ( usAddress & ( uint16_t ) 0x00ff );

    /* Leave register R26 - R31 untouched */
    pxTopOfStack -= 7;

    return pxTopOfStack;
}
/*-----------------------------------------------------------*/

BaseType_t xPortStartScheduler( void )
{

#if defined(DEBUG_PING)
    DDRD |= _BV(DDD7);        /* set the debugging ping */
    PORTD &= ~_BV(PORTD7);
#endif

#if defined( portUSE_TIMER2_RTC ) && !defined( portUSE_TIMER2 )
    /* Setup the timer hardware to generate the RTC sys_tick(), at 1 sec intervals. */
    prvSetupRTCInterrupt();
#endif

    /* Setup the relevant timer hardware to generate the tick. */
    prvSetupTimerInterrupt();

    /* Restore the context of the first task that is going to run. */
    portRESTORE_CONTEXT();

    /* Simulate a function call end as generated by the compiler. We will now
     * jump to the start of the task the context of which we have just restored. */
    __asm__ __volatile__ ( "ret" );

    /* Should not get here. */
    return pdTRUE;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
    /* It is unlikely that the ATmega port will get stopped.  If required simply
     * disable the tick interrupt here. */

#if defined (portUSE_WDTO)
    wdt_disable();                                          /* disable Watchdog Timer */

#elif defined( portUSE_TIMER0 )
    portTIMSK &= ~( _BV(OCIE0B)|_BV(OCIE0A)|_BV(TOIE0) );   /* disable all Timer0 interrupts */

#elif defined( portUSE_TIMER1 )
    portTIMSK &= ~( _BV(OCIE1B)|_BV(OCIE1A)|_BV(TOIE1) );   /* disable all Timer1 interrupts */

#elif defined( portUSE_TIMER2 )
    portTIMSK &= ~( _BV(OCIE2B)|_BV(OCIE2A)|_BV(TOIE2) );   /* disable all Timer2 interrupts */
    ASSR = 0x00;                                            /* set Timer/Counter2 to be off */

#elif defined( portUSE_TIMER3 )
    portTIMSK &= ~( _BV(OCIE3B)|_BV(OCIE3A)|_BV(TOIE3) );   /* disable all Timer3 interrupts */

#endif
}
/*-----------------------------------------------------------*/

/*
 * Manual context switch. The first thing we do is save the registers so we
 * can use a naked attribute.
 */
void vPortYield( void ) __attribute__( ( hot, flatten, naked ) );
void vPortYield( void )
{
    portSAVE_CONTEXT();
    vTaskSwitchContext();
    portRESTORE_CONTEXT();

    __asm__ __volatile__ ( "ret" );
}
/*-----------------------------------------------------------*/

/*
 * Manual context switch callable from ISRs. The first thing we do is save
 * the registers so we can use a naked attribute.
 */
void vPortYieldFromISR( void ) __attribute__( ( hot, flatten, naked ) );
void vPortYieldFromISR( void )
{
    portSAVE_CONTEXT();
    vTaskSwitchContext();
    portRESTORE_CONTEXT();

    __asm__ __volatile__ ( "reti" );
}
/*-----------------------------------------------------------*/

/*
 * Context switch function used by the tick. This must be identical to
 * vPortYield() from the call to vTaskSwitchContext() onwards. The only
 * difference from vPortYield() is the tick count is incremented as the
 * call comes from the tick ISR.
 */
void vPortYieldFromTick( void ) __attribute__( ( hot, flatten, naked ) );
void vPortYieldFromTick( void )
{
    portSAVE_CONTEXT();
    sleep_reset();        /* reset the sleep_mode() faster than sleep_disable(); */

#if defined(DEBUG_PING)
    /* start mark - check for start of interrupt - for debugging only */
    PORTD |=  _BV(PORTD7);                /* Ping IO line */
#endif

#if !defined(portUSE_TIMER2_RTC)
    if (--ticksRemainingInSec == 0)
    {
        system_tick();
        ticksRemainingInSec = portTickRateHz;
    }
#endif

    if( xTaskIncrementTick() != pdFALSE )
    {
        vTaskSwitchContext();
    }

#if defined(DEBUG_PING)
    /* end mark - check for end of interrupt - for debugging only */
    PORTD &= ~_BV(PORTD7);
#endif

    portRESTORE_CONTEXT();

    __asm__ __volatile__ ( "ret" );
}
/*-----------------------------------------------------------*/

#if defined( portUSE_WDTO )

    /*
     * Setup WDT to generate a tick interrupt.
     */
    void prvSetupTimerInterrupt( void )
    {
        /* reset watchdog */
        wdt_reset();

         /* actual port tick rate in Hz, calculated */
        portTickRateHz = configTICK_RATE_HZ;
        /* initialise first second of ticks */
        ticksRemainingInSec = portTickRateHz;

        /* set up WDT Interrupt (rather than the WDT Reset). */
        wdt_interrupt_enable( portUSE_WDTO );
    }

#elif defined (portUSE_TIMER0) || defined (portUSE_TIMER1) || defined (portUSE_TIMER3)

/*
 * Setup Timer 0 or 1 or 3 compare match A to generate a tick interrupt.
 */
void prvSetupTimerInterrupt( void )
{
uint32_t ulCompareMatch;
#ifdef portOCRH
uint8_t ucHighByte;
#endif
uint8_t ucLowByte;

    /* Using 8bit Timer0 or 16bit Timer1 or Timer3 to generate the tick. Correct fuses must be
    selected for the configCPU_CLOCK_HZ clock.*/

    // ulCompareMatch 40,000 = 20,000,000 / 500; 20MHz
    // ulCompareMatch 110,592 = 22,118,400 / 200; 22.1184 MHz
    ulCompareMatch = configCPU_CLOCK_HZ / configTICK_RATE_HZ;

    /* We only have 8 or 16 bits so have to scale 64 or 256 to get our required tick rate. */
    //ulCompareMatch = 625 /= portCLOCK_PRESCALER; 20MHz with 64 prescale
    //ulCompareMatch = 108 /= portCLOCK_PRESCALER; 22.1184 MHz with 1024 prescale
    ulCompareMatch /= portCLOCK_PRESCALER;

     /* actual port tick rate in Hz, calculated */
    portTickRateHz = (TickType_t) ((uint32_t) configCPU_CLOCK_HZ / ( portCLOCK_PRESCALER * ulCompareMatch ));
    /* initialise first second of ticks */
    ticksRemainingInSec = portTickRateHz;

    /* Adjust for correct value. */
    ulCompareMatch -= ( uint32_t ) 1;

    /* Setup compare match value for compare match A. Interrupts are disabled
    before this is called so we need not worry here. */
    ucLowByte = ( uint8_t ) ( ulCompareMatch & ( uint32_t ) 0xff );

    //  OCR3AH = ucHighByte;
    //  OCR3AL = ucLowByte;

    // the HiByte is only needed, if a 16 Bit counter is being utilized
#ifdef portOCRH
    ulCompareMatch >>= 8;
    ucHighByte = ( uint8_t ) ( ulCompareMatch & ( uint32_t) 0xff );
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

    /* Enable the interrupt - this is okay as interrupt are currently globally disabled. */
    ucLowByte = portTIMSK;
    ucLowByte |= portCOMPARE_MATCH_A_INTERRUPT_ENABLE;
    portTIMSK = ucLowByte;
}

#elif defined(portUSE_TIMER2)

/*
 * Setup Crystal-controlled timer2 compare match A to generate a tick interrupt.
 */

void prvSetupTimerInterrupt( void )
{
    uint16_t usCompareMatch;

    /* Using 8bit Timer2 to generate the tick.  A 32.768 KHz crystal
     * must be attached to the appropriate pins.  We then adjust the number
     * to a power of two so we can get EXACT seconds for the Real Time clock.
     */

    usCompareMatch = (uint16_t) ((uint32_t) 32768) / configTICK_RATE_HZ;

    if ( usCompareMatch > 192 )
    {
        usCompareMatch = 256;
    }
    else
    {
        for (uint8_t i = 7; i >= 1; --i)
        {
            if ( usCompareMatch & ((uint16_t)1 << i) )
            {
                /* found the power - now let's see if we round up or down */
                if ( usCompareMatch & ((uint16_t)1 << (i-1)) )
                {
                    usCompareMatch = ((uint16_t)1 << (i+1));
                }
                else
                {
                    usCompareMatch = ((uint16_t)1 << i);
                }
                break;
            }
        }
    }

    /* actual port tick rate in Hz, calculated */
    portTickRateHz = (TickType_t) ((uint32_t) 32768 / usCompareMatch );
    /* initialise first second of ticks */
    ticksRemainingInSec = portTickRateHz;

    /* Adjust for correct value. */
    usCompareMatch -= ( uint16_t ) 1;

    portTIMSK &= ~( _BV(OCIE2B)|_BV(OCIE2A)|_BV(TOIE2) );       // disable all Timer2 interrupts
    portTIFR |=  _BV(OCF2B)|_BV(OCF2A)|_BV(TOV2);               // clear all pending interrupts
    ASSR = _BV(AS2);                                            // set Timer/Counter2 to be asynchronous from the CPU clock
                                                                // with a second external clock (32,768kHz) driving it.
    portTCNT  = 0x00;                                           // zero out the counter
    portTCCRa = _BV(WGM21);                                     // mode CTC (clear on counter match)
    portTCCRb = _BV(CS20);                                      // divide timer clock by 1 (No prescaling)
    portOCRL  = usCompareMatch;                                 // set the counter

    while( ASSR & (_BV(TCN2UB)|_BV(OCR2AUB)|_BV(TCR2AUB)));     // Wait until Timer2 update complete

    /* Enable the interrupt - this is okay as interrupts are currently globally disabled. */
    portTIMSK |= portCOMPARE_MATCH_A_INTERRUPT_ENABLE;          // interrupt on Timer2 compare match
}
#endif


#if defined(portUSE_TIMER2_RTC) && !defined(portUSE_TIMER2)

/*
 * Setup Crystal-controlled timer2 compare match A to generate a tick interrupt.
 */
    #warning "Timer2 used for RTC. This is a 1 second clock."

static void prvSetupRTCInterrupt( void )
{

    /* Using 8bit Timer2 to generate the tick.
     * A 32.768 KHz crystal must be attached to the appropriate pins.
     * We then adjust the scale factor and counter to roll over at the top
     * so we can get EXACT seconds for the Real Time clock.
     */

    TIMSK2 &= ~( _BV(OCIE2B)|_BV(OCIE2A)|_BV(TOIE2) );          // disable all Timer2 interrupts
    TIFR2 |=  _BV(OCF2B)|_BV(OCF2A)|_BV(TOV2);                  // clear all pending interrupts
    ASSR = _BV(AS2);                                            // set Timer/Counter2 to be asynchronous from the CPU clock
                                                                // with a second external clock (32,768kHz) driving it.
    TCNT2  = 0x00;                                              // zero out the counter
    TCCR2A = 0x00;                                              // Normal mode
    TCCR2B = _BV(CS22) | _BV(CS20);                             // divide timer clock by 128 so counter will roll over at MAX

    while( ASSR & (_BV(TCN2UB)|_BV(OCR2AUB)|_BV(TCR2AUB)));     // Wait until Timer2 update complete

    /* Enable the interrupt - this is okay as interrupts are currently globally disabled. */
    TIMSK2 |= _BV(TOIE2);                                       // When the TOIE2 bit is written to one, the interrupt is enabled
}

#elif defined(portUSE_TIMER2_RTC) && defined(portUSE_TIMER2)
    #warning "Trying to configure Timer 2 for both sys_tick() and xTaskIncrementTick()."

#endif


/*-----------------------------------------------------------*/

#if configUSE_PREEMPTION == 1

    /*
     * Tick ISR for preemptive scheduler. We can use a naked attribute as
     * the context is saved at the start of vPortYieldFromTick(). The tick
     * count is incremented after the context is saved.
     *
     * use ISR_NOBLOCK where there is an important timer running, that should preempt the scheduler.
     *
     */
    ISR( portSCHEDULER_ISR, ISR_NAKED ) __attribute__ ( ( hot, flatten ) );

/*  ISR( portSCHEDULER_ISR, ISR_NAKED ISR_NOBLOCK ) __attribute__ ( ( hot, flatten ) );
 */
    ISR( portSCHEDULER_ISR )
    {
        vPortYieldFromTick();
        __asm__ __volatile__ ( "reti" );
    }
#else /* if configUSE_PREEMPTION == 1 */

    /*
     * Tick ISR for the cooperative scheduler. All this does is increment the
     * tick count. We don't need to switch context, this can only be done by
     * manual calls to taskYIELD();
     *
     * use ISR_NOBLOCK where there is an important timer running, that should preempt the scheduler.
     */
    ISR( portSCHEDULER_ISR ) __attribute__ ( ( hot, flatten ) );

/*  ISR( portSCHEDULER_ISR, ISR_NOBLOCK ) __attribute__ ( ( hot, flatten ) );
 */
    ISR( portSCHEDULER_ISR )
    {
#if !defined(portUSE_TIMER2_RTC)
        if (--ticksRemainingInSec == 0)
        {
            system_tick();
            ticksRemainingInSec = portTickRateHz;
        }
#endif
        xTaskIncrementTick();
    }
#endif /* if configUSE_PREEMPTION == 1 */

#if defined (portUSE_TIMER2_RTC) && !defined(portUSE_TIMER2)
    /*
     * Tick ISR for the RTC.  All this does is increment the RTC tick count, once per second.
     * Use ISR_NOBLOCK where there is an important timer running, that should preempt the RTC.
     * As long as it completes within one second, then there is no issue.
     *
     * use ISR_NOBLOCK where there is an important timer running, that should preempt the scheduler.
     */
    ISR( TIMER2_OVF_vect, ISR_NAKED ) __attribute__ ( ( hot, flatten ) );

/*  ISR( TIMER2_OVF_vect, ISR_NAKED ISR_NOBLOCK ) __attribute__ ( ( hot, flatten ) );
 */
    ISR( TIMER2_OVF_vect )
    {
        system_tick();
        reti();
    }

#endif

