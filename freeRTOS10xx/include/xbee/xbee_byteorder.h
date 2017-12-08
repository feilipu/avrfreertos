/*
 * Copyright (c) 2010-2012 Digi International Inc.,
 * All rights not expressly granted are reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Digi International Inc. 11001 Bren Road East, Minnetonka, MN 55343
 * =======================================================================
 */

/**
	@addtogroup util_byteorder
	@{
	@file xbee/byteorder.h
	Macros for converting between big/little endian and host byte order.

	@def be16toh(x)
	@brief Convert a uint16_t in big-endian byte order to host byte order.

	@def be32toh(x)
	@brief Convert a uint32_t in big-endian byte order to host byte order.

	@def htobe16(x)
	@brief Convert a uint16_t in host byte order to big-endian byte order.

	@def htobe32(x)
	@brief Convert a uint32_t in host byte order to big-endian byte order.

	@def htole16(x)
	@brief Convert a uint16_t in host byte order to little-endian byte order.

	@def htole32(x)
	@brief Convert a uint32_t in host byte order to little-endian byte order.

	@def le16toh(x)
	@brief Convert a uint16_t in little-endian byte order to host byte order.

	@def le32toh(x)
	@brief Convert a uint32_t in little-endian byte order to host byte order.

	@def memcpy_betoh( dst, src_be, count)
	@brief Copy \a count bytes in big-endian byte order from \a src_be to
			\a dst in host byte order (equivalent to memcpy() on big-endian
			platforms).

	@def memcpy_betole( dst_le, src_be, count)
	@brief Copy \a count bytes in big-endian byte order from \a src_be to
			\a dst_le in little-endian byte order (always swaps byte order).

	@def memcpy_htobe( dst_be, src, count)
	@brief Copy \a count bytes in host byte order from \a src to \a dst_be
			in big-endian byte order (equivalent to memcpy() on big-endian
			platforms).

	@def memcpy_htole( dst_le, src, count)
	@brief Copy \a count bytes in host byte order from \a src to \a dst_le
			in little-endian byte order (equivalent to memcpy() on little-endian
			platforms).

	@def memcpy_letobe( dst_be, src_le, count)
	@brief Copy \a count bytes in little-endian byte order from \a src_le to
			\a dst_be in big-endian byte order (always swaps byte order).

	@def memcpy_letoh( dst, src_le, count)
	@brief Copy \a count bytes in little-endian byte order from \a src_le to
			\a dst in host byte order (equivalent to memcpy() on little-endian
			platforms).
*/

#ifndef __XBEE_ENDIAN_H
#define __XBEE_ENDIAN_H

	#include <string.h>			// for memcpy
	#include "lib_util.h"		// for swap16, swap32, and _swapcpy

	// xbee/platform will load the platform's endian.h or at least define
	// the macros LITTLE_ENDIAN, BIG_ENDIAN and BYTE_ORDER.
	#include "xbee_platform.h"

	// On AVRfreeRTOS, swap16() and swap32() are already defined, so don't
	// define them here.
	#ifndef swap16
		uint16_t swap16( uint16_t value);
	#endif
	#ifndef swap32
		uint32_t swap32( uint32_t value);
	#endif



	#if BYTE_ORDER == LITTLE_ENDIAN
		#define memcpy_letoh( dst, src_le, count)	_f_memcpy( dst, src_le, count )
		#define memcpy_htole( dst_le, src, count)	_f_memcpy( dst_le, src, count )

		#define memcpy_betoh( dst, src_be, count)	_swapcpy( dst, src_be, count )
		#define memcpy_htobe( dst_be, src, count)	_swapcpy( dst_be, src, count )
	#else
		#define memcpy_letoh( dst, src_le, count)	_swapcpy( dst, src_le, count )
		#define memcpy_htole( dst_le, src, count)	_swapcpy( dst_le, src, count )

		#define memcpy_betoh( dst, src_be, count)	_f_memcpy( dst, src_be, count )
		#define memcpy_htobe( dst_be, src, count)	_f_memcpy( dst_be, src, count )
	#endif

	#define memcpy_betole( dst_le, src_be, count)	\
						_swapcpy( dst_le, src_be, count)
	#define memcpy_letobe( dst_be, src_le, count)	\
						_swapcpy( dst_be, src_le, count)

	// define byte-swapping macros if the platform hasn't already done so
	#ifndef htobe16
		#if BYTE_ORDER == LITTLE_ENDIAN
			// host to big-endian
			#define htobe16(x)	swap16(x)
			#define htobe32(x)	swap32(x)

			// big-endian to host
			#define be16toh(x)	swap16(x)
			#define be32toh(x)	swap32(x)

			// host to little-endian
			#define htole16(x)	(x)
			#define htole32(x)	(x)

			// little-endian to host
			#define le16toh(x)	(x)
			#define le32toh(x)	(x)
		#else
			// host to little-endian
			#define htole16(x)	swap16(x)
			#define htole32(x)	swap32(x)

			// little-endian to host
			#define le16toh(x)	swap16(x)
			#define le32toh(x)	swap32(x)

			// host to big-endian
			#define htobe16(x)	(x)
			#define htobe32(x)	(x)

			// big-endian to host
			#define be16toh(x)	(x)
			#define be32toh(x)	(x)
		#endif
	#endif

#endif
