/*
 * (C)2012 Michael Duane Rice All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer. Redistributions in binary
 * form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials
 * provided with the distribution. Neither the name of the copyright holders
 * nor the names of contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* $Id$ */

/*
	Return a value suitable for use as a FAT file system time stamp.
*/

#include "time.h"

uint32_t
system_fatfs(const struct tm * timeptr)
{
	uint32_t        ret;
	uint32_t        n;

	n = timeptr->tm_year - 80;
	n <<= 25;
	ret = n;

	n = timeptr->tm_mon + 1;
	n <<= 21;
	ret |= n;

	n = timeptr->tm_mday;
	n <<= 16;
	ret |= n;

	n = timeptr->tm_hour;
	n <<= 11;
	ret |= n;

	n = timeptr->tm_min;
	n <<= 5;
	ret |= n;

	ret |= timeptr->tm_sec >> 1;

	return ret;
}


/**
    Convert a FAT file system time stamp into a Y2K time stamp.

	wFatDate [in]
	The MS-DOS date. The date is a packed value with the following format.
	Bits	Description
	0-4		Day of the month (1–31)
	5-8		Month (1 = January, 2 = February, and so on)
	9-15	Year offset from 1980, subtract 20 to get Y2K year, add 2000 for Gregorian year.

	wFatTime [in]
	The MS-DOS time. The time is a packed value with the following format.
	Bits	Description
	0-4		Second divided by 2
	5-10	Minute (0–59)
	11-15	Hour (0–23 on a 24-hour clock)
*/

uint32_t
fatfs_system( uint16_t fsdate, uint16_t fstime, struct tm * timeptr)
{
	timeptr->tm_year =          ((fsdate >> 9) & 0x007F) + 80;
	timeptr->tm_mon =  ((uint8_t)(fsdate >> 5 ) & 0x0F) - 1;
	timeptr->tm_mday =  (uint8_t) fsdate & 0x1F;

	timeptr->tm_hour =  (uint8_t)(fstime >> 11) & 0x1F;
	timeptr->tm_min =   (uint8_t)(fstime >> 5) & 0x3F;
	timeptr->tm_sec =  ((uint8_t) fstime & 0x001F) << 1;
return mktime( timeptr );
}

