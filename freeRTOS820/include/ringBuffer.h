/*
             LUFA Library
     Copyright (C) Dean Camera, 2013.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2013  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \ingroup Group_MiscDrivers
 *  \defgroup Group_RingBuff Generic Byte Ring Buffer - LUFA/Drivers/Misc/RingBuffer.h
 *  \brief Lightweight ring buffer, for fast insertion/deletion of bytes.
 *
 *  \section Sec_RingBuff_Dependencies Module Source Dependencies
 *  The following files must be built with any user project that uses this module:
 *    - None
 *
 *  \section Sec_RingBuff_ModDescription Module Description
 *  Lightweight ring buffer, for fast insertion/deletion. Multiple buffers can be created of
 *  different sizes to suit different needs.
 *
 *  Note that for each buffer, insertion and removal operations may occur at the same time (via
 *  a multi-threaded ISR based system) however the same kind of operation (two or more insertions
 *  or deletions) must not overlap. If there is possibility of two or more of the same kind of
 *  operating occurring at the same point in time, atomic (mutex) locking should be used.
 *
 *  \section Sec_RingBuff_ExampleUsage Example Usage
 *  The following snippet is an example of how this module may be used within a typical
 *  application.
 *
 *  \code
 *      // Create the buffer structure and its underlying storage array
 *      ringBuffer_t buffer;
 *      uint8_t      bufferData[128];
 *
 *      // Initialize the buffer with the created storage array
 *      ringBuffer_InitBuffer(&buffer, bufferData, sizeof(bufferData));
 *
 *      // Insert some data into the buffer
 *      ringBuffer_Poke(&buffer, 'H');
 *      ringBuffer_Poke(&buffer, 'E');
 *      ringBuffer_Poke(&buffer, 'L');
 *      ringBuffer_Poke(&buffer, 'L');
 *      ringBuffer_Poke(&buffer, 'O');
 *
 *      // Cache the number of stored bytes in the buffer
 *      uint16_t BufferCount = ringBuffer_GetCount(&buffer);
 *
 *      // Printer stored data length
 *      printf("buffer Length: %d, Buffer Data: \r\n", BufferCount);
 *
 *      // Print contents of the buffer one character at a time
 *      while (BufferCount--)
 *        putc(ringBuffer_Pop(&buffer));
 *  \endcode
 *
 *  @{
 */

#ifndef __RING_BUFFER_H__
#define __RING_BUFFER_H__


/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
	extern "C" {
#endif

#include "FreeRTOS.h"

/** Indicates that the function returns a value which should not be ignored by the user code. When
 *  applied, any ignored return value from calling the function will produce a compiler warning.
 */
#define ATTR_WARN_UNUSED_RESULT      __attribute__ ((warn_unused_result))

/** Indicates that the specified parameters of the function are pointers which should never be \c NULL.
 *  When applied as a 1-based comma separated list the compiler will emit a warning if the specified
 *  parameters are known at compiler time to be \c NULL at the point of calling the function.
 */
#define ATTR_NON_NULL_PTR_ARG(...)   __attribute__ ((nonnull (__VA_ARGS__)))

/** Forces the compiler to never inline the specified function. When applied, the given function will be
 *  always be called explicitly under all circumstances.
 */
#define ATTR_NEVER_INLINE            __attribute__ ((noinline))

/** Forces the compiler to inline the specified function. When applied, the given function will be
 *  in-lined under all circumstances. And make it an often used "hot" function.
 */
#define ATTR_ALWAYS_INLINE           __attribute__ ((hot, always_inline))

#if defined(__GNUC__)
	/** Forces GCC to use pointer indirection (via the device's pointer register pairs) when accessing the given
	 *  struct pointer. In some cases GCC will emit non-optimal assembly code when accessing a structure through
	 *  a pointer, resulting in a larger binary. When this macro is used on a (non \c const) structure pointer before
	 *  use, it will force GCC to use pointer indirection on the elements rather than direct store and load
	 *  instructions.
	 *
	 *  \param[in, out] StructPtr  Pointer to a structure which is to be forced into indirect access mode.
	 */
	#define GCC_FORCE_POINTER_ACCESS(StructPtr)   __asm__ __volatile__("" : "=b" (StructPtr) : "0" (StructPtr))

	/** Forces GCC to create a memory barrier, ensuring that memory accesses are not reordered past the barrier point.
	 *  This can be used before ordering-critical operations, to ensure that the compiler does not re-order the resulting
	 *  assembly output in an unexpected manner on sections of code that are ordering-specific.
	 */
	#define GCC_MEMORY_BARRIER()                  __asm__ __volatile__("" ::: "memory");

#else
	#define GCC_FORCE_POINTER_ACCESS(StructPtr)
	#define GCC_MEMORY_BARRIER()
	#define GCC_IS_COMPILE_CONST(x)               0
#endif

/************************** Type Defines: ***************************/
/** \brief Ring Buffer Management Structure.
 *
 *  Type define for a new ring buffer object. Buffers should be initialized via a call to
 *  \ref ringBuffer_InitBuffer() before use.
 */
typedef struct
{
	volatile uint16_t count;	/**< Number of bytes currently stored in the buffer. */
	volatile uint8_t * in;		/**< Current storage location in the circular buffer. */
	volatile uint8_t * out;		/**< Current retrieval location in the circular buffer. */
	uint8_t* start;				/**< Pointer to the start of the buffer's underlying storage array. */
	uint8_t* end;				/**< Pointer to the end of the buffer's underlying storage array. */
	uint16_t size;				/**< Size of the buffer's underlying storage array. */
} ringBuffer_t, * ringBufferPtr_t;

/************************* Inline Functions: *************************/

/** Initializes a ring buffer ready for use. Buffers must be initialized via this function
 *  before any operations are called upon them. Already initialized buffers may be reset
 *  by re-initializing them using this function.
 *
 *  \param[out] buffer   Pointer to a ring buffer structure to initialize.
 *  \param[out] dataPtr  Pointer to a global array that will hold the data stored into the ring buffer.
 *  \param[out] size     Maximum number of bytes that can be stored in the underlying data array.
 */
inline void
ringBuffer_InitBuffer(	ringBuffer_t* buffer,
						uint8_t* const dataPtr,
						const uint16_t size) ATTR_NON_NULL_PTR_ARG(1) ATTR_NON_NULL_PTR_ARG(2) ATTR_ALWAYS_INLINE;


/** Flushes the contents of a ring buffer.
 *
 *  \param[out] buffer   Pointer to a ring buffer structure to flush out.
 */
inline void
ringBuffer_Flush(ringBuffer_t* const buffer) ATTR_NON_NULL_PTR_ARG(1) ATTR_ALWAYS_INLINE;

/** Retrieves the current number of bytes stored in a particular buffer. This value is computed
 *  by entering an atomic lock on the buffer, so that the buffer cannot be modified while the
 *  computation takes place. This value should be cached when reading out the contents of the buffer,
 *  so that as small a time as possible is spent in an atomic lock.
 *
 *  \note The value returned by this function is guaranteed to only be the minimum number of bytes
 *        stored in the given buffer; this value may change as other threads write new data, thus
 *        the returned number should be used only to determine how many successive reads may safely
 *        be performed on the buffer.
 *
 *  \param[in] buffer  Pointer to a ring buffer structure whose count is to be computed.
 *
 *  \return Number of bytes currently stored in the buffer.
 */
inline uint16_t
ringBuffer_GetCount(ringBuffer_t* const buffer) ATTR_WARN_UNUSED_RESULT ATTR_NON_NULL_PTR_ARG(1) ATTR_ALWAYS_INLINE;

/** Retrieves the free space in a particular buffer. This value is computed by entering an atomic lock
 *  on the buffer, so that the buffer cannot be modified while the computation takes place.
 *
 *  \note The value returned by this function is guaranteed to only be the maximum number of bytes
 *        free in the given buffer; this value may change as other threads write new data, thus
 *        the returned number should be used only to determine how many successive writes may safely
 *        be performed on the buffer when there is a single writer thread.
 *
 *  \param[in] buffer  Pointer to a ring buffer structure whose free count is to be computed.
 *
 *  \return Number of free bytes in the buffer.
 */
inline uint16_t
ringBuffer_GetFreeCount(ringBuffer_t* const buffer) ATTR_WARN_UNUSED_RESULT ATTR_NON_NULL_PTR_ARG(1) ATTR_ALWAYS_INLINE;

/** Atomically determines if the specified ring buffer contains any data. This should
 *  be tested before removing data from the buffer, to ensure that the buffer does not
 *  underflow.
 *
 *  If the data is to be removed in a loop, store the total number of bytes stored in the
 *  buffer (via a call to the \ref ringBuffer_GetCount() function) in a temporary variable
 *  to reduce the time spent in atomicity locks.
 *
 *  \param[in,out] buffer  Pointer to a ring buffer structure to insert into.
 *
 *  \return Boolean \c true if the buffer contains no free space, \c false otherwise.
 */
inline uint8_t
ringBuffer_IsEmpty(ringBuffer_t* const buffer) ATTR_WARN_UNUSED_RESULT ATTR_NON_NULL_PTR_ARG(1) ATTR_ALWAYS_INLINE;

/** Atomically determines if the specified ring buffer contains any free space. This should
 *  be tested before storing data to the buffer, to ensure that no data is lost due to a
 *  buffer overrun.
 *
 *  \param[in,out] buffer  Pointer to a ring buffer structure to insert into.
 *
 *  \return Boolean \c true if the buffer contains no free space, \c false otherwise.
 */
inline uint8_t
ringBuffer_IsFull(ringBuffer_t* const buffer) ATTR_WARN_UNUSED_RESULT ATTR_NON_NULL_PTR_ARG(1) ATTR_ALWAYS_INLINE;

/** Inserts an element into the ring buffer.
 *
 *  \warning Only one execution thread (main program thread or an ISR) may insert into a single buffer
 *           otherwise data corruption may occur. Insertion and removal may occur from different execution
 *           threads.
 *
 *  \param[in,out] buffer  Pointer to a ring buffer structure to insert into.
 *  \param[in]     data    Data element to insert into the buffer.
 */
inline void
ringBuffer_Poke(ringBuffer_t* buffer, const uint8_t data) ATTR_NON_NULL_PTR_ARG(1) ATTR_ALWAYS_INLINE;

/** Removes an element from the ring buffer.
 *
 *  \warning Only one execution thread (main program thread or an ISR) may remove from a single buffer
 *           otherwise data corruption may occur. Insertion and removal may occur from different execution
 *           threads.
 *
 *  \param[in,out] buffer  Pointer to a ring buffer structure to retrieve from.
 *
 *  \return Next data element stored in the buffer.
 */
inline uint8_t
ringBuffer_Pop(ringBuffer_t* buffer) ATTR_NON_NULL_PTR_ARG(1) ATTR_ALWAYS_INLINE;

/** Returns the next element stored in the ring buffer, without removing it.
 *
 *  \param[in,out] buffer  Pointer to a ring buffer structure to retrieve from.
 *
 *  \return Next data element stored in the buffer.
 */
inline uint8_t
ringBuffer_Peek(ringBuffer_t* const buffer) ATTR_WARN_UNUSED_RESULT ATTR_NON_NULL_PTR_ARG(1) ATTR_ALWAYS_INLINE;




inline void
ringBuffer_InitBuffer(	ringBuffer_t* buffer,
						uint8_t* const dataPtr,
						const uint16_t size)
{
	GCC_FORCE_POINTER_ACCESS(buffer);

	portENTER_CRITICAL();

	buffer->count  = 0;
	buffer->in     = dataPtr;
	buffer->out    = dataPtr;
	buffer->start  = &dataPtr[0];
	buffer->end    = &dataPtr[size];
	buffer->size   = size;

	portEXIT_CRITICAL();
}

inline void
ringBuffer_Flush(ringBuffer_t* const buffer)
{
	portENTER_CRITICAL();

	buffer->count	= 0;
	buffer->in		= buffer->start;
	buffer->out		= buffer->start;

	portEXIT_CRITICAL();
}

inline uint16_t
ringBuffer_GetCount(ringBuffer_t* const buffer)
{
	portENTER_CRITICAL();

	uint16_t count = buffer->count;

	portEXIT_CRITICAL();
	return count;
}

inline uint16_t
ringBuffer_GetFreeCount(ringBuffer_t* const buffer)
{
	return (buffer->size - ringBuffer_GetCount(buffer));
}

inline uint8_t
ringBuffer_IsEmpty(ringBuffer_t* const buffer)
{
	return (ringBuffer_GetCount(buffer) == 0);
}

inline uint8_t
ringBuffer_IsFull(ringBuffer_t* const buffer)
{
	return (ringBuffer_GetCount(buffer) == buffer->size);
}

inline void
ringBuffer_Poke(ringBuffer_t* buffer, const uint8_t data)
{
	GCC_FORCE_POINTER_ACCESS(buffer);

	*buffer->in = data;

	portENTER_CRITICAL();

	if (++buffer->in == buffer->end)
	  buffer->in = buffer->start;

	buffer->count++;

	portEXIT_CRITICAL();
}

inline uint8_t
ringBuffer_Pop(ringBuffer_t* buffer)
{
	GCC_FORCE_POINTER_ACCESS(buffer);

	uint8_t data = *buffer->out;

	portENTER_CRITICAL();

	if (++buffer->out == buffer->end)
	  buffer->out = buffer->start;

	buffer->count--;

	portEXIT_CRITICAL();

	return data;
}

inline uint8_t
ringBuffer_Peek(ringBuffer_t* const buffer)
{
	return *buffer->out;
}

/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
	}
#endif

#endif

/** @} */

