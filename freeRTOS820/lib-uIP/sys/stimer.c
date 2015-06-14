/**
 * \addtogroup stimer
 * @{
 */

/**
 * \file
 * Timer of seconds library implementation.
 * \author
 * Adam Dunkels <adam@sics.se>, Nicolas Tsiftes <nvt@sics.se>
 */

/*
 * Copyright (c) 2004, 2008, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Author: Adam Dunkels <adam@sics.se>, Nicolas Tsiftes <nvt@sics.se>
 *
 */

#include "time.h"
#include "uIP/sys/stimer.h"

#define SCLOCK_GEQ(a, b)	((time_t)((a) - (b)) < \
				((time_t)(~((time_t)0)) >> 1))

/*---------------------------------------------------------------------------*/
/**
 * Set a timer.
 *
 * This function is used to set a timer for a time sometime in the
 * future. The function stimer_expired() will evaluate to true after
 * the timer has expired.
 *
 * \param t A pointer to the timer
 * \param interval The interval before the timer expires.
 *
 */
void
stimer_set( seconds_timer_ptr t, const time_t interval )
{
  t->interval = interval;
  t->start = time(NULL);
}
/*---------------------------------------------------------------------------*/
/**
 * Reset the timer with the same interval.
 *
 * This function resets the timer with the same interval that was
 * given to the stimer_set() function. The start point of the interval
 * is the exact time that the timer last expired. Therefore, this
 * function will cause the timer to be stable over time, unlike the
 * stimer_restart() function.
 *
 * \param t A pointer to the timer.
 *
 * \sa stimer_restart()
 */
void
stimer_reset( seconds_timer_ptr t )
{
  t->start += t->interval;
}
/*---------------------------------------------------------------------------*/
/**
 * Restart the timer from the current point in time
 *
 * This function restarts a timer with the same interval that was
 * given to the stimer_set() function. The timer will start at the
 * current time.
 *
 * \note A periodic timer will drift if this function is used to reset
 * it. For periodic timers, use the stimer_reset() function instead.
 *
 * \param t A pointer to the timer.
 *
 * \sa stimer_reset()
 */
void
stimer_restart( seconds_timer_ptr t )
{
  t->start = time(NULL);
}
/*---------------------------------------------------------------------------*/
/**
 * Check if a timer has expired.
 *
 * This function tests if a timer has expired and returns true or
 * false depending on its status.
 *
 * \param t A pointer to the timer
 *
 * \return Non-zero if the timer has expired, zero otherwise.
 *
 */
int16_t
stimer_expired( seconds_timer_ptr t )
{
  return SCLOCK_GEQ(time(NULL), t->start + t->interval);
}
/*---------------------------------------------------------------------------*/
/**
 * The time until the timer expires
 *
 * This function returns the time until the timer expires.
 *
 * \param t A pointer to the timer
 *
 * \return The time until the timer expires
 *
 */
time_t
stimer_remaining( seconds_timer_ptr t )
{
  return t->start + t->interval - time(NULL);
}
/*---------------------------------------------------------------------------*/
/**
 * The time elapsed since the timer started
 *
 * This function returns the time elapsed.
 *
 * \param t A pointer to the timer
 *
 * \return The time elapsed since the last start of the timer
 *
 */
time_t
stimer_elapsed( seconds_timer_ptr t )
{
  return time(NULL) - t->start;
}

/*---------------------------------------------------------------------------*/

/** @} */
