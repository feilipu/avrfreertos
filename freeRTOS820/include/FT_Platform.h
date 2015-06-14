#ifndef _FT_PLATFORM_H_
#define _FT_PLATFORM_H_

/*
File:   FT_Platform.h
*/

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

/* freeRTOS Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "spi.h"
#include "time.h"  // included to get a random seed for srandom(time(NULL)); and random();

#include "../lib_ft800/FT_DataTypes.h"
#include "../lib_ft800/FT_X11_RGB.h"
#include "../lib_ft800/FT_Gpu.h"
#include "../lib_ft800/FT_Gpu_Hal.h"
#include "../lib_ft800/FT_Hal_Utils.h"
#include "../lib_ft800/FT_CoPro_Cmds.h"
#include "../lib_ft800/FT_API.h"

#ifdef __cplusplus
}
#endif

#endif /*_FT_PLATFORM_H_*/
/* Nothing beyond this*/




