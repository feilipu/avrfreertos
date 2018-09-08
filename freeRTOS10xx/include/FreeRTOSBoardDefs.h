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

/* freeRTOSBoardDefs.h
 *
 * Board (hardware) specific definitions for the AVR boards that I use regularly.
 * This includes
 * Arduino UNO with ATmega328p
 * Goldilocks with ATmega1284p
 * Arduino MEGA with ATmega2560
 *
 * And also Pololu SVP with ATmega1284p
 *
 * This file is NOT part of the FreeRTOS distribution.
 *
 */

#ifndef freeRTOSBoardDefs_h
#define freeRTOSBoardDefs_h

#include <avr/io.h>
#include <avr/wdt.h>

#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif
/*
 * XXX Ping PD7 to check timing on interrupts.
 * PD7 will be set high when the interrupt is entered, and set low when exited.
 * This allows you to check that there is sufficient time for the rest of the system to operate,
 * and to ensure that the interrupts are not being missed or are overlapping.
 */
//    #define DEBUG_PING

/*-----------------------------------------------------------
 * MCU and application specific definitions.
 *
 * These definitions should be adjusted for your particular
 * application requirements.
 *
 *----------------------------------------------------------*/

#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)

#ifndef _MEGA_
    #define _MEGA_
#endif
// System Tick  - Scheduler timer
// Prefer to use the Watchdog timer, but also Timer 0, 1, or 3 are ok.

//  #define portUSE_WDTO    WDTO_15MS    // portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick

/* Watchdog period options: WDTO_15MS
                            WDTO_30MS
                            WDTO_60MS
                            WDTO_120MS
                            WDTO_250MS
                            WDTO_500MS
*/

//  #define portUSE_TIMER0                                          // portUSE_TIMER0 to use 8 bit Timer0 for xTaskIncrementTick
//  #define portUSE_TIMER1                                          // portUSE_TIMER1 to use 16 bit Timer1 for xTaskIncrementTick
//  #define portUSE_TIMER2                                          // portUSE_TIMER2 to use 8 bit Timer2 using 32,768Hz for xTaskIncrementTick
    #define portUSE_TIMER3                                          // portUSE_TIMER3 to use 16 bit Timer3 for xTaskIncrementTick

// Use Timer 2 for a Real Time Clock, if you have a 32kHz watch crystal attached.
//  #define portUSE_TIMER2_RTC                                      // portUSE_TIMER2_RTC to use 8 bit RTC Timer2 for system_tick (not xTaskIncrementTick)


#if defined (portUSE_WDTO)
//  xxx Watchdog Timer is 128kHz nominal, but 120 kHz at 5V DC and 25 degrees is actually more accurate, from data sheet.
    #define configTICK_RATE_HZ  ( (TickType_t)( (uint32_t)128000 >> (portUSE_WDTO + 11) ) )  // 2^11 = 2048 WDT Scale-factor

#elif defined (portUSE_TIMER0) || defined (portUSE_TIMER1) || defined (portUSE_TIMER3)
    #define configTICK_RATE_HZ  ( (TickType_t) 200 )                // Use 1000Hz to get mSec timing using Timer1 or Timer3.

#elif defined( portUSE_TIMER2 ) && !defined( portUSE_TIMER2_RTC )
    #define configTICK_RATE_HZ  ( (TickType_t) 128 )                // MINIMUM for TIMER2 is 128 Hz because of fixed scale factor.

#endif


//  XRAM device options. Different methods of enabling and driving.    MegaRAM only implemented for two banks of 56kByte currently.
//  #define portMEGA_RAM                                            // Use the Rugged Circuits External (128kByte) MegaRAM device. - OR -
//  #define portQUAD_RAM                                            // Use the Rugged Circuits External (512kByte) QuadRAM device.

//  portQUAD_RAM device Options. NOT valid for use with portMEGA_RAM.  XRAM Memory is available as 8 banks of 56kByte, for heap. - OR -
//  #define portEXT_RAM_16_BANK                                     // XRAM Memory is available as 16 banks of 32kByte, for heap. - OR -
//  #define portEXT_RAMFS                                           // XRAM Memory is available as 16 banks of 32kByte for 16 Arduino clients (i.e. NOT used for heap).


#if defined (portQUAD_RAM) || defined (portMEGA_RAM)
    #define portEXT_RAM
#endif

#if defined (portMEGA_RAM) || (defined (portQUAD_RAM) && !defined (portEXT_RAMFS))
    // XRAM banks enabled. We have to set the linker to move the heap to XRAM. -> DON'T FORGET TO ADD THESE LINK OPTIONS
    #define configTOTAL_HEAP_SIZE    ( (size_t ) (XRAMEND - 0x8000))    // Should be 0xffff - 0x8000 = 32767 for (non malloc) heap in XRAM.
                                                                        // Used for heap_1.c, heap2.c, and heap4.c only, and maximum Array size possible for Heap is 32767.
#else
    // There is no XRAM available for the heap.
    #define configTOTAL_HEAP_SIZE    ( (size_t ) 0x1200 )
//  #define configTOTAL_HEAP_SIZE    ( (size_t ) 0x1800 )               // 0x1800 = 6144 used for heap_1.c, heap2.c, and heap4.c only, where heap is NOT in XRAM.
                                                                        // Used for heap_1.c, heap2.c, and heap4.c only, and maximum Array size possible for Heap is 32767.
#endif

/**
 * Select WIZCHIP.
 * You should select one, \b 5100, \b 5200 ,\b 5500 or etc.
 */
    #define _WIZCHIP_                      5500     // 5100, 5200, 5500

//  #define portHD44780_LCD                         // define the use of the Freetronics HD44780 LCD (or other). Check include hd44780.h for (flexible) pin assignments.
    #define portSD_CARD                             // define the use of the SD Card for Arduino Mega2560 and Freetronics EtherMega
//  #define portRTC_DEFINED                         // RTC DS1307 / DS3231 implemented, therefore define.

    #define portSERIAL_BUFFER_RX    255             // Define the size of the serial receive buffer.
    #define portSERIAL_BUFFER_TX    255             // Define the size of the serial transmit buffer, only as long as the longest line of text.
    #define portSERIAL_BUFFER       portSERIAL_BUFFER_TX // just for compatibility with older programmes.

//  #define portUSE_TIMER1_PWM                      // Define which Timer to use as the PWM Timer (not the tick timer).


#elif defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284PA__) // Goldilocks with 1284p

#ifndef _GOLDILOCKS_
    #define _GOLDILOCKS_
#endif

    // System Tick  - Scheduler timer
    // Prefer to use the Watchdog timer, but also Timer 0, 1, or 3 are ok.

// #define portUSE_WDTO     WDTO_15MS    // portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick

/* Watchdog period options: WDTO_15MS
                            WDTO_30MS
                            WDTO_60MS
                            WDTO_120MS
                            WDTO_250MS
                            WDTO_500MS
    */

//  #define portUSE_TIMER0                      // portUSE_TIMER0 to use 8 bit Timer0 for xTaskIncrementTick
//  #define portUSE_TIMER1                      // portUSE_TIMER1 to use 16 bit Timer1 for xTaskIncrementTick
    #define portUSE_TIMER2                      // portUSE_TIMER2 to use 8 bit Timer2 using 32,768Hz for xTaskIncrementTick
//  #define portUSE_TIMER3                      // portUSE_TIMER3 to use 16 bit Timer3 for xTaskIncrementTick

// Use Timer 2 for a Real Time Clock, if you have a 32kHz watch crystal attached.
//    #define portUSE_TIMER2_RTC                // portUSE_TIMER2_RTC to use 8 bit RTC Timer2 for system_tick (not xTaskIncrementTick)

#if defined (portUSE_WDTO)
//  xxx Watchdog Timer is 128kHz nominal, but 120 kHz at 5V DC and 25 degrees is actually more accurate, from data sheet.
#define configTICK_RATE_HZ      ( (TickType_t)( (uint32_t)128000 >> (portUSE_WDTO + 11) ) )  // 2^11 = 2048 WDT Scale-factor

#elif defined (portUSE_TIMER0) || defined (portUSE_TIMER1) || defined (portUSE_TIMER3)
    #define configTICK_RATE_HZ  ( (TickType_t) 200 )        // Use 1000Hz to get mSec timing using Timer1 or Timer3.

#elif defined( portUSE_TIMER2 ) && !defined( portUSE_TIMER2_RTC )
    #define configTICK_RATE_HZ  ( (TickType_t) 128 )        // MINIMUM for TIMER2 is 128 Hz because of fixed scale factor.

#endif


//  #define configTOTAL_HEAP_SIZE    ( (size_t )  15699  )  // used for heap_1.c and heap2.c, and heap_4.c only (measured for GA Synth)
    #define configTOTAL_HEAP_SIZE    ( (size_t )  12699  )  // used for heap_1.c and heap2.c, and heap_4.c only

/**
 * Select WIZCHIP.
 * You should select one, \b 5100, \b 5200 ,\b 5500 or etc. \n\n
 */
    #define _WIZCHIP_               5500                    // 5100, 5200, 5500

//  #define portHD44780_LCD                                 // define the use of the Freetronics HD44780 LCD (or other). Check include hd44780.h for (flexible) pin assignments.
    #define portSD_CARD                                     // define the use of the SD Card for Goldilocks 1284p
//  #define portRTC_DEFINED                                 // RTC DS1307 / DS3231 implemented, therefore define.
//  #define portANALOGUE                                    // Goldilocks Analogue Capabilities
//  #define portANALOGSHIELD                                // Digilent Analog Shield (only DAC implemented)

    #define portSERIAL_BUFFER_RX    255                     // Define the size of the serial receive buffer.
    #define portSERIAL_BUFFER_TX    255                     // Define the size of the serial transmit buffer, only as long as the longest text.
    #define portSERIAL_BUFFER       portSERIAL_BUFFER_TX    // just for compatibility with older programmes.

//  #define portUSE_TIMER1_PWM                              // Define which Timer to use as the PWM Timer (not the tick timer).
                                                            // though it is better to use Pololu functions, as they support 8x multiplexed servos.

#elif defined(__AVR_ATmega32U2__) || defined(__AVR_ATmega16U2__) || defined(__AVR_ATmega8U2__)
// Arduino Serial I/O MCU Compatible notation.
#ifndef _U2DUINO_
    #define _U2DUINO_
#endif

// System Tick timer
// Prefer to use the Watchdog timer, but also Timer 0 or 3 are ok.

#define portUSE_WDTO        WDTO_15MS    // portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick

/* Watchdog period options: WDTO_15MS
                            WDTO_30MS
                            WDTO_60MS
                            WDTO_120MS
                            WDTO_250MS
                            WDTO_500MS
*/

//  #define portUSE_TIMER0                                  // portUSE_TIMER0 to use 8 bit Timer0 for xTaskIncrementTick
//  #define portUSE_TIMER1                                  // portUSE_TIMER1 to use 16 bit Timer1 for xTaskIncrementTick


#if defined (portUSE_WDTO)
//  xxx Watchdog Timer is 128kHz nominal, but 120 kHz at 5V DC and 25 degrees is actually more accurate, from data sheet.
#define configTICK_RATE_HZ         ( (TickType_t)( (uint32_t)128000 >> (portUSE_WDTO + 11) ) )  // 2^11 = 2048 WDT Scale-factor

#elif defined (portUSE_TIMER0) || defined (portUSE_TIMER1)
    #define configTICK_RATE_HZ      ( (TickType_t) 200 )    // Use 1000Hz to get mSec timing using Timer1.
#endif


    // xxx Cannot emphasise how important it is to watch and massage this heap size number.
    // Greater than 100% memory usage. Subtle fail.
    // Less than 96%. Typically every byte counts for 32u2.
    // Watch for the stack overflowing, if you use interrupts. Use configCHECK_FOR_STACK_OVERFLOW
    #define configTOTAL_HEAP_SIZE   ( (size_t ) 830 )       // used for heap_1.c, heap_2.c, and heap_4.c only

    #define portSERIAL_BUFFER_RX     16                     // Define the size of the serial receive buffer.
    #define portSERIAL_BUFFER_TX     128                    // Define the size of the serial transmit buffer, only as long as the longest line of text.
    #define portSERIAL_BUFFER        portSERIAL_BUFFER_TX

#if  defined(portUSE_TIMER1)                                // Define which Timer to use as the PWM Timer (not the tick timer)
    #define portUSE_TIMER0_PWM                              // It's pointless to use the 8bit Timer0 for Servo PWM,
                                                            // as these Timers can't set ICR1 as TOP, allowing the pulse width period to be adjusted.
#elif defined( portUSE_TIMER0 )
    #define portUSE_TIMER1_PWM
#else
    #warning Missing definition: The PWM Timer is not defined.
#endif

#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) // assume we're using an Arduino with 328p

#ifndef _UNO_
    #define _UNO_
#endif

//  System Tick  - Scheduler timer
//  Prefer to use the Watchdog timer, but also Timer 0 or 1 are ok.

// #define portUSE_WDTO         WDTO_15MS                   // portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick

/* Watchdog period options:     WDTO_15MS
                                WDTO_30MS
                                WDTO_60MS
                                WDTO_120MS
                                WDTO_250MS
                                WDTO_500MS

*/

    #define portUSE_TIMER0                                  // portUSE_TIMER0 to use 8 bit Timer0 for xTaskIncrementTick
//  #define portUSE_TIMER1                                  // portUSE_TIMER1 to use 16 bit Timer1 for xTaskIncrementTick


#if defined (portUSE_WDTO)
//  xxx Watchdog Timer is 128kHz nominal, but 120 kHz at 5V DC and 25 degrees is actually more accurate, from data sheet.
    #define configTICK_RATE_HZ      ( (TickType_t)( (uint32_t)128000 >> (portUSE_WDTO + 11) ) )  // 2^11 = 2048 WDT Scale-factor

#elif defined (portUSE_TIMER0) || defined (portUSE_TIMER1)
    #define configTICK_RATE_HZ  ( (TickType_t) 200 )        // Use 1000Hz to get mSec timing using Timer1.

#endif


    // xxx Cannot emphasise how important it is to watch and massage this heap size number.
    // Greater than 100% memory usage. Subtle fail.
    // Less than 96%. Typically every byte counts for 328p.
    // Watch for the stack overflowing, if you use interrupts. Use configCHECK_FOR_STACK_OVERFLOW
    #define configTOTAL_HEAP_SIZE   ( (size_t ) 1530 )      // used for heap_1.c, heap_2.c, and heap_4.c only

//  #define portEXT_RAMFS                                   // XRAM Memory is available for 16x 328p ArduSat (Uno) clients as 16 banks of 32kByte from a 2560.

/**
 * Select WIZCHIP.
 * You should select one, \b 5100, \b 5200 ,\b 5500 or etc.
 */
    #define _WIZCHIP_               5200    // 5100, 5200, 5500

//  #define portHD44780_LCD                                 // define the use of the Freetronics HD44780 LCD (or other). Check include hd44780.h for (flexible) pin assignments.
//  #define portRTC_DEFINED                                 // RTC DS1307 / DS3231 implemented, therefore define.

    #define portSERIAL_BUFFER_RX    32                      // Define the size of the serial receive buffer.
    #define portSERIAL_BUFFER_TX    128                     // Define the size of the serial transmit buffer, only as long as the longest line of text.
    #define portSERIAL_BUFFER       portSERIAL_BUFFER_TX    // Set the default serial buffer to be the Tx size.

#if  defined(portUSE_TIMER1)                                // Define which Timer to use as the PWM Timer (not the tick timer)
    #define portUSE_TIMER0_PWM                              // It's pointless to use the 8bit Timer0 or Timer2 for Servo PWM,
                                                            // as these Timers can't set ICR1 as TOP, allowing the pulse width period to be adjusted.
#elif !defined( portUSE_TIMER1 )
    #define portUSE_TIMER1_PWM
#else
    #warning Missing definition: The PWM Timer is not defined.
#endif

#else
    #error Missing definition: The MCU type is not defined.
#endif


/*-----------------------------------------------------------
 * Board specific definitions.
 *
 * These definitions should probably not be adjusted for your
 * application requirements.
 *
 *----------------------------------------------------------*/

#if defined(_MEGA_)

// I2C pins
    #define I2C_PORT            PORTD
    #define I2C_PORT_DIR        DDRD
    #define I2C_PORT_STATUS     PIND
    #define I2C_BIT_SDA         _BV(PD1)
    #define I2C_BIT_SCL         _BV(PD0)

// SPI pins
    #define SPI_PORT            PORTB
    #define SPI_PORT_DIR        DDRB
    #define SPI_PORT_PIN        PINB
    #define SPI_BIT_SCK         _BV(PB1)
    #define SPI_BIT_MISO        _BV(PB3)
    #define SPI_BIT_MOSI        _BV(PB2)
    #define SPI_BIT_SS          _BV(PB0)

    #define SPI_BIT_SS_WIZNET   _BV(PB4)    // added for Wiznet 5100/5200 support with SS on PB4 (Pin 4)

    #define SPI_PORT_SS_G2      PORTH
    #define SPI_PORT_DIR_SS_G2  DDRH
    #define SPI_PORT_PIN_SS_G2  PINH
    #define SPI_BIT_SS_G2       _BV(PH5)    // added to for Gameduino2 using FTDI FT800 Graphics (Pin 8)

    #define SPI_PORT_SS_SD      PORTG
    #define SPI_PORT_DIR_SS_SD  DDRG
    #define SPI_PORT_PIN_SS_SD  PING
    #define SPI_BIT_SS_SD       _BV(PG5)    // added for SD Card support with SS on PG5 (Pin 10)


#elif defined(_GOLDILOCKS_)

// I2C pins
    #define I2C_PORT            PORTC
    #define I2C_PORT_DIR        DDRC
    #define I2C_PORT_STATUS     PINC
    #define I2C_BIT_SDA         _BV(PC1)
    #define I2C_BIT_SCL         _BV(PC0)

// SPI pins
    #define SPI_PORT            PORTB
    #define SPI_PORT_DIR        DDRB
    #define SPI_PORT_PIN        PINB
    #define SPI_BIT_SCK         _BV(PB7)
    #define SPI_BIT_MISO        _BV(PB6)
    #define SPI_BIT_MOSI        _BV(PB5)
    #define SPI_BIT_SS          _BV(PB4)

    #define SPI_BIT_SS_WIZNET    _BV(PB4)   // added for Wiznet 5100/5200 support with SS on PB4 (Pin 10)

    #define SPI_PORT_SS_G2      PORTB
    #define SPI_PORT_DIR_SS_G2  DDRB
    #define SPI_PORT_PIN_SS_G2  PINB
//  #define SPI_BIT_SS_G2       _BV(PB3)    // added to for 4DSystems using FTDI FT800 Graphics (Pin 9)
    #define SPI_BIT_SS_G2       _BV(PB2)    // added to for Gameduino2 using FTDI FT800 Graphics (Pin 8)

    #define SPI_PORT_SS_DAC     PORTB
    #define SPI_PORT_DIR_SS_DAC DDRB
    #define SPI_PORT_PIN_SS_DAC PINB
    #define SPI_BIT_SS_DAC      _BV(PB1)    // added for support of integrated DAC card on PB1 Goldilocks Analogue MCP4822

#if 1                                       // xxx 1 to use Goldilocks Analogue with SD card on PB0 | 0 to use Goldilocks v1.1
    #define SPI_PORT_SS_SD      PORTB
    #define SPI_PORT_DIR_SS_SD  DDRB
    #define SPI_PORT_PIN_SS_SD  PINB
    #define SPI_BIT_SS_SD       _BV(PB0)    // added for support of integrated SD card on PB0, on Goldilocks Analogue.

    #define SD_CARD_DETECT_PORT PORTC       // Card Detect on PC2
    #define SD_CARD_DETECT_DIR  DDRC        // Card Detect on PC2
    #define SD_CARD_DETECT_PIN  PINC        // Card Detect on PC2
    #define SD_CARD_DETECT_BIT  _BV(PC2)    // Card Detect on PC2

#else
    #define SPI_PORT_SS_SD      PORTD
    #define SPI_PORT_DIR_SS_SD  DDRD
    #define SPI_PORT_PIN_SS_SD  PIND
    #define SPI_BIT_SS_SD       _BV(PD4)    // added for SD Card support with Standard Arduino SS on PD4 (Pin 4) for SD cages. This is Goldilocks v1.1. SS is on PD4.
#endif


#elif defined(_U2DUINO_)

// I2C pins                    // not available

// SPI pins
    #define SPI_PORT            PORTB
    #define SPI_PORT_DIR        DDRB
    #define SPI_PORT_PIN        PINB
    #define SPI_BIT_SCK         _BV(PB1)
    #define SPI_BIT_MISO        _BV(PB3)
    #define SPI_BIT_MOSI        _BV(PB2)
    #define SPI_BIT_SS          _BV(PB0)


    #elif defined(_UNO_)

// I2C pins
    #define I2C_PORT            PORTC
    #define I2C_PORT_DIR        DDRC
    #define I2C_PORT_STATUS     PINC
    #define I2C_BIT_SDA         _BV(PC4)
    #define I2C_BIT_SCL         _BV(PC5)

// SPI pins
    #define SPI_PORT            PORTB
    #define SPI_PORT_DIR        DDRB
    #define SPI_PORT_PIN        PINB
    #define SPI_BIT_SCK         _BV(PB5)
    #define SPI_BIT_MISO        _BV(PB4)
    #define SPI_BIT_MOSI        _BV(PB3)
    #define SPI_BIT_SS          _BV(PB2)

    #define SPI_BIT_SS_WIZNET   _BV(PB2)    // added for Wiznet 5100/5200 support with SS on PB2 (Pin 10)

    #define SPI_PORT_SS_G2      PORTB
    #define SPI_PORT_DIR_SS_G2  DDRB
    #define SPI_PORT_PIN_SS_G2  PINB
    #define SPI_BIT_SS_G2       _BV(PB0)    // added for Gameduino2 using FTDI FT800 Graphics (Pin 8)

    #define SPI_PORT_SS_SD      PORTD
    #define SPI_PORT_DIR_SS_SD  DDRD
    #define SPI_PORT_PIN_SS_SD  PIND
    #define SPI_BIT_SS_SD       _BV(PD4)    // added for SD Card support with SS on PD4 (Pin 4)


#else
    #error Missing definition: The Board is not defined.
#endif

#ifdef __cplusplus
}
#endif

#endif // freeRTOSBoardDefs_h
