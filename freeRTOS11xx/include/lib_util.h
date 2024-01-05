#ifndef LIB_UTIL_H_
#define LIB_UTIL_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

uint8_t crc7(const uint8_t *data, uint16_t number_of_bytes_in_data) __attribute__ ((hot, flatten));

uint8_t crc8( const uint8_t *data, uint16_t number_of_bytes_in_data ) __attribute__ ((hot, flatten));

uint16_t crc16( const uint8_t *data, uint16_t number_of_bytes_in_data, uint16_t init ) __attribute__ ((hot, flatten));

#define crc16_calc	crc16

#define CRC16XMODEM_INIT 0x0000
#define CRC16CCITT_INIT 0xFFFF

/*---------------------------------------
 *
 * Utility functions
 *
 *---------------------------------------*/

#define sq(v)		({ __typeof__ (v) _v = (v); _v * _v; })
#define SQ(v)		sq(v)

#define ABS(x)		abs(x)

#define max(a,b)  	({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })
#define MAX(a,b)  	max(a,b)

#define min(a,b)  	({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })
#define MIN(a,b)  	min(a,b)


uint32_t _swaplong( const uint32_t l) __attribute__ ((hot, flatten));

#define swaplong	_swaplong
#define swap32		_swaplong
#define swapl		_swaplong

uint16_t _swapshort( const uint16_t i) __attribute__ ((hot, flatten));

#define swapshort	_swapshort
#define swapuint	_swapshort
#define swap16		_swapshort
#define swaps		_swapshort


/**
	@internal
	@brief
	Function similar to memcpy() but reverses byte order during copy.
	Copy \a count from \a src to \a dst while reversing the order.  Assumes
	that \a src and \a dst do not overlap.

	@param[out]	dst	destination buffer
	@param[in]	src	source buffer
	@param[in]	count number of bytes to copy
*/
void _swapcpy( void *dst, const void *src, uint_fast8_t count) __attribute__ ((hot, flatten));

#ifdef __cplusplus
}
#endif

#endif

/*
Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

