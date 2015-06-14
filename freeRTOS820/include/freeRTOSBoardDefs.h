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
 */

#ifndef freeRTOSBoardDefs_h
#define freeRTOSBoardDefs_h

#ifdef __cplusplus
extern "C" {
#endif

#include <avr/io.h>

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

	#define portUSE_TIMER3											// portUSE_TIMER3 to use 16 bit Timer3

    #define configTICK_RATE_HZ		( ( TickType_t ) 500 )			// Use 500Hz for TIMER3

//	XRAM device options. Different methods of enabling and driving.    MegaRAM only implemented for two banks of 56kByte currently.
	#define portMEGA_RAM											// Use the Rugged Circuits External (128kByte) MegaRAM device. - OR -
//	#define portQUAD_RAM											// Use the Rugged Circuits External (512kByte) QuadRAM device.

//	portQUAD_RAM device Options. NOT valid for use with portMEGA_RAM.  XRAM Memory is available as 8 banks of 56kByte, for heap. - OR -
//	#define portEXT_RAM_16_BANK										// XRAM Memory is available as 16 banks of 32kByte, for heap. - OR -
//	#define portEXT_RAMFS											// XRAM Memory is available as 16 banks of 32kByte for 16 Arduino clients (i.e. NOT used for heap).


#if defined (portQUAD_RAM) || defined (portMEGA_RAM)
	#define portEXT_RAM
#endif

#if defined (portMEGA_RAM) || (defined (portQUAD_RAM) && !defined (portEXT_RAMFS))
	// XRAM banks enabled. We have to set the linker to move the heap to XRAM. -> DON'T FORGET TO ADD THESE LINK OPTIONS
	#define configTOTAL_HEAP_SIZE	( (size_t ) (XRAMEND - 0x8000)) // Should be 0xffff - 0x8000 = 32767 for (non malloc) heap in XRAM.
																	// Used for heap_1.c, heap2.c, and heap4.c only, and maximum Array size possible for Heap is 32767.
#else
	// There is no XRAM available for the heap.
	#define configTOTAL_HEAP_SIZE	( (size_t ) 0x1200 )
//	#define configTOTAL_HEAP_SIZE	( (size_t ) 0x1800 )			// 0x1800 = 6144 used for heap_1.c, heap2.c, and heap4.c only, where heap is NOT in XRAM.
																	// Used for heap_1.c, heap2.c, and heap4.c only, and maximum Array size possible for Heap is 32767.
#endif

/**
 * Select WIZCHIP.
 * ou should select one, \b 5100, \b 5200 ,\b 5500 or etc.
 */
//	#define _WIZCHIP_                      5100   // 5100, 5200, 5500

	#define portHD44780_LCD					// define the use of the Freetronics HD44780 LCD (or other). Check include hd44780.h for (flexible) pin assignments.
//	#define portSD_CARD						// define the use of the SD Card for Arduino Mega2560 and Freetronics EtherMega
//	#define portRTC_DEFINED					// RTC DS1307 / DS3231 implemented, therefore define.

	#define	portSERIAL_BUFFER_RX	64		// Define the size of the serial receive buffer.
	#define	portSERIAL_BUFFER_TX	255		// Define the size of the serial transmit buffer, only as long as the longest line of text.
	#define portSERIAL_BUFFER		portSERIAL_BUFFER_TX

//  #define portUSE_TIMER1_PWM				// Define which Timer to use as the PWM Timer (not the tick timer).


#elif defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284PA__) // Goldilocks with 1284p

#ifndef _GOLDILOCKS_
	#define _GOLDILOCKS_
#endif

//	#define portUSE_TIMER0                                          // portUSE_TIMER0 to use 8 bit Timer0
	#define portUSE_TIMER2											// portUSE_TIMER2 to use 8 bit RTC Timer2 on 1284p device
//	#define portUSE_TIMER3											// portUSE_TIMER3 to use 16 bit Timer3 on 1284p device

    #define configTICK_RATE_HZ		( ( TickType_t ) 256 )			// Use 500Hz for TIMER3. MINIMUM of 128Hz for TIMER2.
//	#define configTICK_RATE_HZ		( ( TickType_t ) 500 )          // Use 1000Hz to get mSec timing using TIMER3.

    #define configTOTAL_HEAP_SIZE	( (size_t )  15000  )			// used for heap_1.c and heap2.c, and heap_4.c only

/**
 * Select WIZCHIP.
 * You should select one, \b 5100, \b 5200 ,\b 5500 or etc. \n\n
 */
//	#define _WIZCHIP_						5500   // 5100, 5200, 5500

//	#define portHD44780_LCD					// define the use of the Freetronics HD44780 LCD (or other). Check include hd44780.h for (flexible) pin assignments.
	#define portSD_CARD						// define the use of the SD Card for Goldilocks 1284p
	#define portRTC_DEFINED					// RTC DS1307 / DS3231 implemented, therefore define.
	#define portANALOGUE					// Goldilocks Analogue Capabilities
//	#define portANALOGSHIELD				// Digilent Analog Shield (only DAC implemented)

	#define	portSERIAL_BUFFER_RX	255		// Define the size of the serial receive buffer.
	#define	portSERIAL_BUFFER_TX	255		// Define the size of the serial transmit buffer, only as long as the longest text.
	#define portSERIAL_BUFFER		portSERIAL_BUFFER_TX // just for compatibility with older programmes.

//   #define portUSE_TIMER1_PWM				// Define which Timer to use as the PWM Timer (not the tick timer).
											// though it is better to use Pololu functions, as they support 8x multiplexed servos.

#elif defined(__AVR_ATmega32U2__) || defined(__AVR_ATmega16U2__) || defined(__AVR_ATmega8U2__)
// Arduino Serial I/O MCU Compatible notation.
#ifndef _U2DUINO_
	#define _U2DUINO_
#endif

//  THIS IS IMPORTANT TO DEFINE THE freeRTOS TICK TIMER, and also Tick Rate
//  Define either: portUSE_TIMER0 or portUSE_TIMER1 for the 32u2
    #define portUSE_TIMER0                                          // portUSE_TIMER0 to use 8 bit Timer0
//  #define portUSE_TIMER1                                          // portUSE_TIMER1 to use 16 bit Timer1

    #define configTICK_RATE_HZ		( ( TickType_t ) 200  )			// Use 200Hz for TIMER0 and 400Hz for TIMER1
                                                                    // Use 1000Hz to get mSec timing.

	// Cannot emphasise how important it is to watch and massage this heap size number.
	// Greater than 100% memory usage. Subtle fail.
	// Less than 96%. Typically every byte counts for 328p.
	// Watch for the stack overflowing, if you use interrupts. Use configCHECK_FOR_STACK_OVERFLOW
    #define configTOTAL_HEAP_SIZE	( (size_t ) 830 )				// used for heap_1.c, heap_2.c, and heap_4.c only

	#define	portSERIAL_BUFFER_RX	16		// Define the size of the serial receive buffer.
	#define	portSERIAL_BUFFER_TX	128		// Define the size of the serial transmit buffer, only as long as the longest line of text.
	#define portSERIAL_BUFFER		portSERIAL_BUFFER_TX

#if  defined(portUSE_TIMER1)				// Define which Timer to use as the PWM Timer (not the tick timer)
    #define portUSE_TIMER0_PWM				// It's pointless to use the 8bit Timer0 for Servo PWM,
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

//  THIS IS IMPORTANT TO DEFINE THE freeRTOS TICK TIMER, and also Tick Rate
//  Define either: portUSE_TIMER0 or portUSE_TIMER1 for the Arduino 328p
	#define portUSE_TIMER0                                          // portUSE_TIMER0 to use 8 bit Timer0
//	#define portUSE_TIMER1                                          // portUSE_TIMER1 to use 16 bit Timer1

    #define configTICK_RATE_HZ		( ( TickType_t ) 200  )			// Use 200Hz for TIMER0 and 400Hz for TIMER1
                                                                    // Use 1000Hz to get mSec timing.

	// Cannot emphasise how important it is to watch and massage this heap size number.
	// Greater than 100% memory usage. Subtle fail.
	// Less than 96%. Typically every byte counts for 328p.
	// Watch for the stack overflowing, if you use interrupts. Use configCHECK_FOR_STACK_OVERFLOW
    #define configTOTAL_HEAP_SIZE	( (size_t ) 1530 )				// used for heap_1.c, heap_2.c, and heap_4.c only

//	#define portEXT_RAMFS					// XRAM Memory is available from a 2560 as 16 banks of 32kByte for 16x 328p ArduSat (Uno) clients.

/**
 * Select WIZCHIP.
 * You should select one, \b 5100, \b 5200 ,\b 5500 or etc.
 */
//	#define _WIZCHIP_                      5200   // 5100, 5200, 5500

	#define portHD44780_LCD					// define the use of the Freetronics HD44780 LCD (or other). Check include hd44780.h for (flexible) pin assignments.
	#define portRTC_DEFINED					// RTC DS1307 / DS3231 implemented, therefore define.

	#define	portSERIAL_BUFFER_RX	32		// Define the size of the serial receive buffer.
	#define	portSERIAL_BUFFER_TX	128		// Define the size of the serial transmit buffer, only as long as the longest line of text.
	#define portSERIAL_BUFFER		portSERIAL_BUFFER_TX	// Set the default serial buffer to be the Tx size.

#if  defined(portUSE_TIMER1)				// Define which Timer to use as the PWM Timer (not the tick timer)
    #define portUSE_TIMER0_PWM				// It's pointless to use the 8bit Timer0 or Timer2 for Servo PWM,
											// as these Timers can't set ICR1 as TOP, allowing the pulse width period to be adjusted.
#elif defined( portUSE_TIMER0 )
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
#define I2C_PORT			PORTD
#define I2C_PORT_DIR		DDRD
#define I2C_PORT_STATUS		PIND
#define I2C_BIT_SDA			_BV(PD1)
#define I2C_BIT_SCL			_BV(PD0)

// SPI pins
#define SPI_PORT			PORTB
#define SPI_PORT_DIR		DDRB
#define SPI_PORT_PIN		PINB
#define SPI_BIT_SCK			_BV(PB1)
#define SPI_BIT_MISO		_BV(PB3)
#define SPI_BIT_MOSI		_BV(PB2)
#define SPI_BIT_SS			_BV(PB0)

#define SPI_BIT_SS_WIZNET	_BV(PB4)	// added for Wiznet 5100/5200 support with SS on PB4 (Pin 4)

#define SPI_PORT_SS_G2		PORTH
#define SPI_PORT_DIR_SS_G2	DDRH
#define SPI_PORT_PIN_SS_G2	PINH
#define SPI_BIT_SS_G2		_BV(PH5)	// added to for Gameduino2 using FTDI FT800 Graphics (Pin 8)

#define SPI_PORT_SS_SD		PORTG
#define SPI_PORT_DIR_SS_SD	DDRG
#define SPI_PORT_PIN_SS_SD	PING
#define SPI_BIT_SS_SD		_BV(PG5)	// added for SD Card support with SS on PG5 (Pin 10)


#elif defined(_GOLDILOCKS_)

// I2C pins
#define I2C_PORT			PORTC
#define I2C_PORT_DIR		DDRC
#define I2C_PORT_STATUS		PINC
#define I2C_BIT_SDA			_BV(PC1)
#define I2C_BIT_SCL			_BV(PC0)

// SPI pins
#define SPI_PORT			PORTB
#define SPI_PORT_DIR		DDRB
#define SPI_PORT_PIN		PINB
#define SPI_BIT_SCK			_BV(PB7)
#define SPI_BIT_MISO		_BV(PB6)
#define SPI_BIT_MOSI		_BV(PB5)
#define SPI_BIT_SS			_BV(PB4)

#define SPI_BIT_SS_WIZNET	_BV(PB4)	// added for Wiznet 5100/5200 support with SS on PB4 (Pin 10)

#define SPI_PORT_SS_G2		PORTB
#define SPI_PORT_DIR_SS_G2	DDRB
#define SPI_PORT_PIN_SS_G2	PINB
//#define SPI_BIT_SS_G2		_BV(PB3)	// added to for 4DSystems using FTDI FT800 Graphics (Pin 9)
#define SPI_BIT_SS_G2		_BV(PB2)	// added to for Gameduino2 using FTDI FT800 Graphics (Pin 8)

#define SPI_PORT_SS_DAC		PORTB
#define SPI_PORT_DIR_SS_DAC	DDRB
#define SPI_PORT_PIN_SS_DAC	PINB
#define SPI_BIT_SS_DAC		_BV(PB1)	// added for support of integrated DAC card on PB1 Goldilocks Analogue MCP4822

#if 1 									// xxx 1 to use Goldilocks Analogue with SD card on PB0
#define SPI_PORT_SS_SD		PORTB
#define SPI_PORT_DIR_SS_SD	DDRB
#define SPI_PORT_PIN_SS_SD	PINB
#define SPI_BIT_SS_SD		_BV(PB0)	// added for support of integrated SD card on PB0, on Goldilocks Analogue.
#else
#define SPI_PORT_SS_SD		PORTD
#define SPI_PORT_DIR_SS_SD	DDRD
#define SPI_PORT_PIN_SS_SD	PIND
#define SPI_BIT_SS_SD		_BV(PD4)	// added for SD Card support with Standard Arduino SS on PD4 (Pin 4) for SD cages. This is Goldilocks v1.1. SS is on PD4.
#endif


#elif defined(_U2DUINO_)

// I2C pins					// not available

// SPI pins
#define SPI_PORT			PORTB
#define SPI_PORT_DIR		DDRB
#define SPI_PORT_PIN		PINB
#define SPI_BIT_SCK			_BV(PB1)
#define SPI_BIT_MISO		_BV(PB3)
#define SPI_BIT_MOSI		_BV(PB2)
#define SPI_BIT_SS			_BV(PB0)


#elif defined(_UNO_)

// I2C pins
#define I2C_PORT			PORTC
#define I2C_PORT_DIR		DDRC
#define I2C_PORT_STATUS		PINC
#define I2C_BIT_SDA			_BV(PC4)
#define I2C_BIT_SCL			_BV(PC5)

// SPI pins
#define SPI_PORT			PORTB
#define SPI_PORT_DIR		DDRB
#define SPI_PORT_PIN		PINB
#define SPI_BIT_SCK			_BV(PB5)
#define SPI_BIT_MISO		_BV(PB4)
#define SPI_BIT_MOSI		_BV(PB3)
#define SPI_BIT_SS			_BV(PB2)

#define SPI_BIT_SS_WIZNET	_BV(PB2)	// added for Wiznet 5100/5200 support with SS on PB2 (Pin 10)

#define SPI_PORT_SS_G2		PORTB
#define SPI_PORT_DIR_SS_G2	DDRB
#define SPI_PORT_PIN_SS_G2	PINB
#define SPI_BIT_SS_G2		_BV(PB0)	// added for Gameduino2 using FTDI FT800 Graphics (Pin 8)

#define SPI_PORT_SS_SD		PORTD
#define SPI_PORT_DIR_SS_SD	DDRD
#define SPI_PORT_PIN_SS_SD	PIND
#define SPI_BIT_SS_SD		_BV(PD4)	// added for SD Card support with SS on PD4 (Pin 4)


#else
	#error Missing definition: The Board is not defined.
#endif

#ifdef __cplusplus
}
#endif

#endif // freeRTOSBoardDefs_h
