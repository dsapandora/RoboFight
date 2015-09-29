/*
 * clock.h - millisecond and microsecond clock
 * 
 * Version 0.4		30/09/2011
 * Written by Peter Lanius
 * Please send suggestions and bug fixes to PeterLanius@gmail.com
 *
 */

/*
 * You may freely modify and share this code, as long as you keep this
 * notice intact. Licensed under the Creative Commons BY-SA 3.0 license:
 *
 *   http://creativecommons.org/licenses/by-sa/3.0/
 *
 * Disclaimer: To the extent permitted by law, this work is provided
 * without any warranty. It might be defective, in which case you agree
 * to be responsible for all resulting costs and damages.
 */


#ifndef _CLOCK_H_
#define _CLOCK_H_

#include <avr/io.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include "global.h"


#ifdef __cplusplus
extern "C"{
#endif

// Interrupt enable/disable macros
#define ENABLE_TIMER0_INTERRUPT()	TIMSK0 = (1<<TOIE0)
#define DISABLE_TIMER0_INTERRUPT()	TIMSK0 = 0

// macros for conversions between F_CPU and microseconds
#define clockCyclesPerMicrosecond()  ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

// initialize the clock functions
void clock_init(void);

// return millisecond count
unsigned long millis(void);

// return microsecond count
unsigned long micros(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif

