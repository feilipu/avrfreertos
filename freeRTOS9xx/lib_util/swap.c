/* please read copyright-notice at EOF */

#include <stdint.h>
#include <avr/pgmspace.h>

#include "lib_util.h"

uint32_t _swaplong( const uint32_t l)
{
	union {
		uint32_t x;
		struct {
			uint8_t a;
			uint8_t b;
			uint8_t c;
			uint8_t d;
		} s;
	} in, out;
	in.x = l;
	out.s.a = in.s.d;
	out.s.b = in.s.c;
	out.s.c = in.s.b;
	out.s.d = in.s.a;
	return out.x;
}

uint16_t _swapshort(uint16_t i)
{
	union {
		uint16_t x;
		struct {
			uint8_t a;
			uint8_t b;
		} s;
	} in, out;
	in.x = i;
	out.s.a = in.s.b;
	out.s.b = in.s.a;
	return out.x;
}

void _swapcpy( void *dst, const void *src, uint_fast8_t bytes)
{
	if (bytes)
	{
		src = (const uint8_t *)src + bytes;
		do
		{
			src = (const uint8_t *)src - 1;
			*(uint8_t *)dst = *(const uint8_t *)src;
			dst = (uint8_t *)dst + 1;
		} while (--bytes);
	}
}


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
