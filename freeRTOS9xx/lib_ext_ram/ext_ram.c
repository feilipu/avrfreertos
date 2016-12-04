/*
 * ext_ram.c
 *
 *  Created on: 17/02/2012
 *      Author: Phillip
 */


#include <stdint.h>
#include <stddef.h>

// AVR include files.
#include <avr/io.h>

#include "FreeRTOS.h"

#if ( defined (portQUAD_RAM) || defined (portMEGA_RAM) ) && ( defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) )

#include "ext_ram.h"

// When linking this stuff, we need to add in the .init3 code below.
// void extRAMinit (void) __attribute__ ((used, naked, section (".init3")));

// And, initialise the heap state (this is done by the linker line) for freeRTOS heap_1.c, heap_2.c, or heap_4.c
// FreeRTOS heap_1, heap_2, and heap_4 memory management the task heap is preallocated, and ignores __heap_start.

// Do NOT edit your linker and objcopy commands if using portEXT_RAMFS. YOUR HEAP IS _NOT_ BEING MOVED!

// For 56kByte and 8 banks  (448kByte)
//	... -Wl,--section-start=.ext_ram_heap=0x802200 -Wl,--defsym=__heap_start=0x802200,--defsym=__heap_end=0x80ffff ...

// For 32kByte and 16 banks (512kByte)
//	... -Wl,--section-start=.ext_ram_heap=0x808000 -Wl,--defsym=__heap_start=0x808000,--defsym=__heap_end=0x80ffff ...

// Also add this into the avr-objcopy, otherwise the Flash image contains the .ext_ram_heap, which will be too
// big if using large heaps with heap_1.c or heap_2.c, which allocates heap independently of malloc().
// --remove-section=.ext_ram_heap

void extRAMinit (void)
{


#if defined(portMEGA_RAM)
	// We're assuming with this initialisation we want to have 2 banks of 56kByte (+ 8kByte inbuilt unbanked)
	// Bit PD7 selects the bank 0
	EXT_RAM_ADDR16(0);

	// MEGA RAM Enable (select) pin PL7 must be HIGH to enable the external MEGA RAM.
	EXT_RAM_SELECT(1);

	// Bus keeper OFF, Port C used for upper address bits.
	XMCRB = 0x00;

#elif defined (portQUAD_RAM) && !(defined(portEXT_RAM_16_BANK) || defined(portEXT_RAMFS))
	// We're assuming with this initialisation we want to have 8 banks of 56kByte (+ 8kByte inbuilt unbanked).
	// Bits PL7, PL6, PL5 select the bank 0
	EXT_RAM_ADDR18(0);
	EXT_RAM_ADDR17(0);
	EXT_RAM_ADDR16(0);

	// QUAD RAM Enable (select) pin PD7 must be LOW to enable the external QUAD RAM.
	EXT_RAM_SELECT(0);

	// Bus keeper OFF, Port C used for upper address bits.
	XMCRB = 0x00;

#elif defined (portQUAD_RAM) && (defined(portEXT_RAM_16_BANK) || defined(portEXT_RAMFS))
	// With this initialisation we want to have 16 banks of 32kByte (+ 8kByte inbuilt unbanked).
	// Bits PL7, PL6, PL5 select the bank 0
	EXT_RAM_ADDR18(0);
	EXT_RAM_ADDR17(0);
	EXT_RAM_ADDR16(0);
	EXT_RAM_ADDR15(0);

	// QUAD RAM Enable (select) pin PD7 must be LOW to enable the external QUAD RAM.
	EXT_RAM_SELECT(0);

	// Bus keeper ON, because we have to manage the address bits manually
	// Lower 7 bits of Port C used for upper address bits, bit 7 (EXT_RAM_ADDR15) is under PIO control.
	XMCRB = _BV(XMBK) | _BV(XMM0);

#endif

	/* Enable XMEM interface:
		 SRE    (7)   : Set to 1 to enable XMEM interface
		 SRL2-0 (6-4) : Set to 00x for wait state sector config: Low=N/A, High=0x2200-0xFFFF
		 SRW11:0 (3-2) : Set to 00 for no wait states in upper sector
		 SRW01:0 (1-0) : Set to 00 for no wait states in lower sector
	*/
	XMCRA = _BV(SRE);

}

uint8_t extRAMcheck (void) __attribute__((flatten));
uint8_t extRAMcheck (void)
{
	return XMCRA;
}


#if defined (portQUAD_RAM) || defined (portMEGA_RAM)

/* --------------------------------------------- */
// Private Function Declarations.

void saveHeap(uint8_t bank_);
void restoreHeap(uint8_t bank_);

/* --------------------------------------------- */
// Global Variables.

/* State for all 8 or 16 banks */
static heapState bankHeapStates[RAM_BANKS];
// this is not really needed, and can be commented out for portEXT_RAMFS

/* The currently selected bank */
static uint8_t currentBank;

/* --------------------------------------------- */

/* Initial setup. You must call this once */
void extRAMInitHeap(bool heapInXmem_)   // use heapInXmem_ false to ignore the heap state for portEXT_RAMFS usage.
{

	uint8_t bank;

	// set up the external RAM registers.
	// This is done in .init3 section. Don't need to do it here.

	// initialise the heap states (this is done by the linker line)
	// For freeRTOS heap.1 & heap.2 memory management the task heap is preallocated, and ignores __heap_start.
	// Do not change the linker and objcopy commands if using portEXT_RAMFS. The heap is not being moved.

	// For 56kByte and 8 banks  (448kByte)
	//	... -Wl,--section-start=.ext_ram_heap=0x802200 -Wl,--defsym=__heap_start=0x802200,--defsym=__heap_end=0x80ffff ...

	// For 32kByte and 16 banks (512kByte)
	//	... -Wl,--section-start=.ext_ram_heap=0x808000 -Wl,--defsym=__heap_start=0x808000,--defsym=__heap_end=0x80ffff ...

	// Also add this into the -avr-objcopy command, otherwise the Flash image is too big.
	// --remove-section=.ext_ram_heap

	if(heapInXmem_) {
		__malloc_heap_end   = (char *) XRAMEND;
		__malloc_heap_start = (char *) XRAMSTART;
		__brkval            = (void *) XRAMSTART;
	}

	for(bank=0;bank<RAM_BANKS;++bank)
		saveHeap(bank);

	// set the current bank to zero
	setMemoryBank(0,false);
}

/* Save the heap variables */

void saveHeap(uint8_t bank_)
{
	bankHeapStates[bank_].__malloc_heap_start = __malloc_heap_start;
	bankHeapStates[bank_].__malloc_heap_end = __malloc_heap_end;
	bankHeapStates[bank_].__brkval = __brkval;
	bankHeapStates[bank_].__flp = __flp;
}

/* Restore the heap variables */

void restoreHeap(uint8_t bank_)
{
	__malloc_heap_start = bankHeapStates[bank_].__malloc_heap_start;
	__malloc_heap_end = bankHeapStates[bank_].__malloc_heap_end;
	__brkval = bankHeapStates[bank_].__brkval;
	__flp = bankHeapStates[bank_].__flp;
}

/* --------------------------------------------- */

/* Set the memory bank */
void setMemoryBank(uint8_t bank_, bool switchHeap_) // use switchHeap_ false to ignore the heap for portEXT_RAMFS usage.
{
	// check, if there's nothing to do.
	if(bank_==currentBank)
		return;

	portENTER_CRITICAL();

	// save heap state if requested
	if(switchHeap_)
		saveHeap(currentBank);

	// switch in the new bank

#if defined(portQUAD_RAM)

	if (XMCRB == 0)
		// we are using 8 banks of 56kBytes
		// Write lower 3 bits of 'bank' to upper 3 bits of Port L
		PORTL = (PORTL & 0x1F) | ((bank_ & 0x7) << 5);
	else // Assume XMCRB = _BV(XMBK) | _BV(XMM0) and we are using 16 banks of 32kBytes
	{
		// The 16-bit 'bank_' parameter can be thought of as a 4-bit field, with the upper
		// 3 bits to be written to PL7:PL5 and the lowest 1 bit to PC7.
		if (bank_ & 0x01) // Take care of lowest 1 bit
			PORTC |= _BV(PC7);
		else
			PORTC &= ~_BV(PC7);

		// Now shift upper 3 bits (of 4-bit value in 'bank_') to PL7:PL5
		PORTL = (PORTL & 0x1F) | (((bank_>>1)&0x7) << 5);
	}

#elif defined(portMEGA_RAM)

	if (bank_ & 0x01) // Take care of lowest 1 bit only allowing for 2 Banks of 56 kByte.
		PORTD |= _BV(PD7);
	else
		PORTD &= ~_BV(PD7);

#endif

	// save state and restore the malloc settings for this bank
	currentBank=bank_;

	if(switchHeap_)
		restoreHeap(currentBank);

	portEXIT_CRITICAL();
}


/* --------------------------------------------- */


extRAMSelfTestResults extRAMSelfTest(void) {

	volatile uint8_t *ptr;
	uint8_t bank,writeValue,readValue;
	extRAMSelfTestResults results;

	// write an ascending sequence of 1..237 decrementing through
	// all memory banks

	writeValue=1;
	for(bank=0;bank<RAM_BANKS;++bank) {

		setMemoryBank(bank, true);

		for(ptr= (uint8_t *) XRAMEND; ptr >= (uint8_t *) XRAMSTART; --ptr) {
			*ptr=writeValue;

			if(writeValue++==237)
				writeValue=1;
		}
	}

	// verify the writes

	writeValue=1;
	for(bank=0;bank<RAM_BANKS;bank++) {

		setMemoryBank(bank, true);

		for(ptr = (uint8_t *) XRAMEND; ptr >= (uint8_t *) XRAMSTART; --ptr) {

			readValue=*ptr;

			if(readValue!=writeValue) {
				results.succeeded=false;
				results.failedAddress=ptr;
				results.failedBank=bank;
				return results;
			}

			if(writeValue++==237)
				writeValue=1;
		}
	}

	results.succeeded=true;
	return results;
}

/* -------------------------------------------- */

#endif

#endif
