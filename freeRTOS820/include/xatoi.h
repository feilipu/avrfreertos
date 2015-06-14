/*---------------------------------------------------------------------------
   Extended atoi,                                             (C)ChaN, 2011
-----------------------------------------------------------------------------*/

#ifndef XATOI
#define XATOI

#include <inttypes.h>
#include <avr/pgmspace.h>


/*-----------------------------------------------------------------------------*/
uint8_t xatoi( uint8_t **str, int32_t *ret );

/* Get value of the numeral string.

  str
    Pointer to pointer to source string

    "0b11001010" binary
    "0377" octal
    "0xff800" hexdecimal
    "1250000" decimal
    "-25000" decimal

  ret
    Pointer to return value
*/

#endif	/* XITOA */
