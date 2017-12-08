#ifndef _FT_PLATFORM_H_
#define _FT_PLATFORM_H_

/*
File:   FT_Platform.h
*/

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
#include "lib_util.h"

#include "../lib_ft800/FT_DataTypes.h"
#include "../lib_ft800/FT_X11_RGB.h"
#include "../lib_ft800/FT_Gpu.h"
#include "../lib_ft800/FT_Gpu_Hal.h"
#include "../lib_ft800/FT_CoPro_Cmds.h"
#include "../lib_ft800/FT_API.h"

#endif /*_FT_PLATFORM_H_*/
/* Nothing beyond this*/




