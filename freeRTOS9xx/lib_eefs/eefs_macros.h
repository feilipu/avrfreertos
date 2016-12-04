/*
** Filename: eefs_macros.h
**
**      Copyright (c) 2010-2014, United States government as represented by the
**      administrator of the National Aeronautics Space Administration.
**      All rights reserved. This software was created at NASAs Goddard
**      Space Flight Center pursuant to government contracts.
**
**      This is governed by the NASA Open Source Agreement and may be used,
**      distributed and modified only pursuant to the terms of that agreement.
**
*/

/*
**
** Purpose: This file contains system dependent macros for eefs lower level functions.
**
** Design Notes:
**
** References:
**
*/

#ifndef _eefs_macros_
#define	_eefs_macros_


/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Serial IO for debugging purposes */
#include "serial.h"

/*
 * System Dependent Lower Level Functions
 */

/* These macros define the lock and unlock interface functions used to guarantee
 * exclusive access to shared resources.  Defaults to undefined since it is implementation dependent. */
#define EEFS_LIB_INIT								eefs_avrspi_begin()
#define EEFS_LIB_LOCK								eefs_avrspi_lock()
#define EEFS_LIB_UNLOCK								eefs_avrspi_unlock()

/* These macros define the lower level EEPROM interface functions. */
#define EEFS_LIB_EEPROM_READ(Dest, Src, Length)		(void)eefs_avrspi_read( (uint8_t *) Dest, (const addr_farptr_t)(const uint32_t) Src, (size_t) Length)
#define EEFS_LIB_EEPROM_WRITE(Dest, Src, Length)	(void)eefs_avrspi_write( (addr_farptr_t)(uint32_t) Dest, (const uint8_t *) Src, (size_t) Length)
#define EEFS_LIB_EEPROM_FLUSH

/* This macro defines the time interface function.  Defaults to time(NULL) */
#define EEFS_LIB_TIME                				time(NULL)

/* This macro defines the debug print function. Assume that the serial port has been set up already. */
#define EEFS_PRINTF									xSerialPrintf

/* This macro defines the file system write protection interface function.  If the file system
   is read-only then set this macro to TRUE.  If the file system is always write enabled then
   set this macro to FALSE.  If the eeprom has an external write protection interface then a custom
   function can be called to determine the write protect status. */
#define EEFS_LIB_IS_WRITE_PROTECTED					FALSE


/*
 * Utility Macro Definitions
 */

#define EEFS_MAX(a,b)  			({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })

#define EEFS_MIN(a,b)  			({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })
#define EEFS_ROUND_UP(x, align)	((uint32_t)(x + (align - 1)) & ~((uint32_t)(align - 1))) // assuming align is power of 2


#endif

/************************/
/*  End of File Comment */
/************************/

