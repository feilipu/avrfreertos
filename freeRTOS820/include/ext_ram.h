/*
 * ext_ram.h
 *
 *  Created on: 17/02/2012
 *      Author: Phillip
 */

#ifndef EXT_RAM_H_
#define EXT_RAM_H_


#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

// AVR include files.
#include <avr/io.h>

#include "FreeRTOS.h"

// initialise the heap states (this is done by the linker line)
// For freeRTOS heap.1 & heap.2 memory management the task heap is preallocated, and ignores __heap_start.

// Do NOT edit your linker and objcopy commands if using portEXT_RAMFS. YOUR HEAP IS _NOT_ BEING MOVED!

// For 56kByte and 8 banks  (448kByte)
//	... -Wl,--section-start=.ext_ram_heap=0x802200 -Wl,--defsym=__heap_start=0x802200,--defsym=__heap_end=0x80ffff ...

// For 32kByte and 16 banks (512kByte)
//	... -Wl,--section-start=.ext_ram_heap=0x808000 -Wl,--defsym=__heap_start=0x808000,--defsym=__heap_end=0x80ffff ...

// Also add this into the -avr-objcopy command, otherwise the Flash image is too big.
// --remove-section=.ext_ram_heap


#if ( defined (portQUAD_RAM) || defined (portMEGA_RAM) ) && ( defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) )

/****************************************************************************
  Defines
****************************************************************************/

#if defined( portMEGA_RAM )
	// MEGA RAM Enable (select) pin PL7 must be HIGH to enable the external RAM.
#define EXT_RAM_SELECT(x) { DDRL |= _BV(DDL7); if (x) PORTL |= _BV(PL7); else PORTL &= ~_BV(PL7); }

// Bit PD7 selects the bank BANKSEL for 2 Banks.
// We're assuming with this initialisation we want to have 2 banks of 56kByte (+ 8kByte inbuilt unbanked)
#define EXT_RAM_ADDR16(x) { DDRD |= _BV(DDD7); if (x) PORTD |= _BV(PD7); else PORTD &= ~_BV(PD7); }
#define EXT_RAM_ADDR15(x) { DDRC |= _BV(DDC7); if (x) PORTC |= _BV(PC7); else PORTC &= ~_BV(PC7); }
#define EXT_RAM_ADDR14(x) { DDRC |= _BV(DDC6); if (x) PORTC |= _BV(PC6); else PORTC &= ~_BV(PC6); }

#define RAM_BANKS 2

/* Pointers to the start of extended memory. XRAMEND is a system define */
#define XRAMSTART  0x2200

#elif defined( portQUAD_RAM )
	// QUAD RAM Enable (select) pin PD7 must be LOW to enable the external RAM.
#define EXT_RAM_SELECT(x) { DDRD |= _BV(DDD7); if (x) PORTD |= _BV(PD7); else PORTD &= ~_BV(PD7); }

// Bits PL7, PL6, PL5, PC7 select the bank
#define EXT_RAM_ADDR18(x) { DDRL |= _BV(DDL7); if (x) PORTL |= _BV(PL7); else PORTL &= ~_BV(PL7); }
#define EXT_RAM_ADDR17(x) { DDRL |= _BV(DDL6); if (x) PORTL |= _BV(PL6); else PORTL &= ~_BV(PL6); }
#define EXT_RAM_ADDR16(x) { DDRL |= _BV(DDL5); if (x) PORTL |= _BV(PL5); else PORTL &= ~_BV(PL5); }
#define EXT_RAM_ADDR15(x) { DDRC |= _BV(DDC7); if (x) PORTC |= _BV(PC7); else PORTC &= ~_BV(PC7); }

#if defined( portEXT_RAM_16_BANK ) || defined( portEXT_RAMFS )

#define RAM_BANKS 16
/* Pointers to the start of extended memory. XRAMEND is a system define */
#define XRAMSTART  0x8000

#else

#define RAM_BANKS 8
/* Pointers to the start of extended memory. XRAMEND is a system define */
#define XRAMSTART  0x2200

#endif

#endif

#if defined (portQUAD_RAM) || defined (portMEGA_RAM)
/****************************************************************************
  Global definitions
****************************************************************************/

// put this C code into .init3 (assembler could go into .init1)
void extRAMinit (void) __attribute__ ((used, naked, section (".init3")));

// This must be called from main() or port.c in pxPortInitialiseStack()
// to ensure that this library is included in the build,
// if no other function from this library is being used. i.e. you're not using the banks.
// Would rather not do this, but don't know another method to force the library to be included.
uint8_t extRAMcheck (void);

/****************************************************************************
  Variable definitions
****************************************************************************/

/* State variables used by the heap */
typedef struct {
	char *__malloc_heap_start;
	char *__malloc_heap_end;
	void *__brkval;
	void *__flp;
} heapState;

/* Results of a self-test run */
typedef struct {
		bool succeeded;
		volatile uint8_t *failedAddress;
		uint8_t failedBank;
} extRAMSelfTestResults;


/****************************************************************************
  Global definitions
****************************************************************************/

void extRAMInitHeap(bool heapInXmem_);  // use heapInXmem_ false to ignore the heap state for portEXT_RAMFS usage.

void setMemoryBank(uint8_t bank_, bool switchHeap_);  // use switchHeap_ false to ignore the heap state for portEXT_RAMFS usage.

extRAMSelfTestResults extRAMSelfTest(void);


/* these symbols get us to the details of the stack and heap */
#define STACK_POINTER() ((char *)AVR_STACK_POINTER_REG)

/* References to the private heap variables */
// These are system provided variables. The don't need to be further defined.
extern char  __heap_start;
extern char  __heap_end;
extern char *__malloc_heap_start;
extern char *__malloc_heap_end;
extern void *__brkval;
extern void *__flp;

#endif


#endif

#ifdef __cplusplus
}
#endif

#endif /* EXT_RAM_H_ */
