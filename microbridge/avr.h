/*
Copyright 2011 Niels Brouwers

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.#include <string.h>
*/

/**
 * \brief Misc. header functions for the Atmel AVR series of micro controllers.
 */

#ifndef __avr_h__
#define __avr_h__


#ifdef __cplusplus
extern "C" {
#endif


#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>

// AVR include files.
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include <util/delay.h>

// Free RTOS include files.
#include <FreeRTOS.h>
#include <task.h>
#include <lib_serial.h>


// TODO: remove this
typedef uint8_t boolean;
typedef uint8_t byte;


// clear bit, set bit macros
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


#ifdef __cplusplus
}
#endif

#endif
