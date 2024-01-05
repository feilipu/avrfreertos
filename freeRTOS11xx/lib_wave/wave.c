/*
 * wave.c
 *
 *  Created on: 07/02/2015
 *      Author: phillip
 */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include <util/delay.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


#include "ringBuffer.h"
#include "DAC.h"

#include "wave.h"


uint8_t waveplayer_create(FIL f)
{return 0;}

void waveplayer_setSampleRate(uint32_t samplerate)
{}

void waveplayer_play(void)
{}

void waveplayer_stop(void)
{}

uint8_t waveplayer_isPaused(void)
{return 0;}

void waveplayer_pause(void)
{}

void waveplayer_resume(void)
{}

void waveplayer_seek(uint32_t pos)
{}

int16_t waveplayer_readWaveData(uint8_t *buff, uint16_t len)
{return 0;}

