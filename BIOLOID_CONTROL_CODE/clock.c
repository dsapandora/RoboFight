/*
 * clock.c - millisecond and microsecond clock
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

#include "clock.h"

// The prescaler is set so that TIMER0 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks.
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))

// The whole number of milliseconds per TIMER0 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)

// The fractional number of milliseconds per TIMER0 overflow. We shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_millis = 0;
static unsigned char timer0_fract = 0;

// TIMER0 overflow interrupt is triggered every 1024us at 16MHz
ISR (TIMER0_OVF_vect)
{
   // copy these to local variables so they can be stored in registers
   // (volatile variables must be read from memory on every access)
   unsigned long m = timer0_millis;
   unsigned char f = timer0_fract;

   m += MILLIS_INC;
   f += FRACT_INC;
   if (f >= FRACT_MAX) {
      f -= FRACT_MAX;
      m += 1;
   }

   timer0_fract = f;
   timer0_millis = m;
   timer0_overflow_count++;
}

// return the current millisecond count
unsigned long millis()
{
	unsigned long m;

	// disable interrupt while we read timer0_millis or we might get an
	// inconsistent value (e.g. in the middle of a write to timer0_millis)
	DISABLE_TIMER0_INTERRUPT();
	m = timer0_millis;
	
	// re-enable TIMER0 overflow interrupt
	ENABLE_TIMER0_INTERRUPT();
	
	// there is a small chance an overflow occurred when interrupt was disabled
	if ( TIFR0 & _BV(TOV0) ) 
	{
		// this should be safe since we have nearly 1ms left until the next overflow
		m++;
		timer0_millis++;
		bit_set_hi(TIFR0,TOV0); // clear the overflow flag since the interrupt didn't execute
	}	

	return m;
}

// return current microsecond count by reading TIMER0 value directly
unsigned long micros() 
{
	unsigned long m;
	uint8 t;

	// disable interrupts and read the timer
	DISABLE_TIMER0_INTERRUPT();
	m = timer0_overflow_count;
	t = TCNT0;
  
	// re-enable interrupts
	ENABLE_TIMER0_INTERRUPT();
   
	// there is a small chance an overflow occurred when interrupt was disabled
	if ((TIFR0 & _BV(TOV0)) && (t < 255))
	{
		// this should be safe since we have nearly 1ms until the next overflow
		m++;
		timer0_millis++;
		bit_set_hi(TIFR0,TOV0); // clear the overflow flag since the interrupt didn't execute
	}	

	return ((m << 8) + t) * (64 / clockCyclesPerMicrosecond());
}

// initializes the TIMER0 which we use for the clock
void clock_init()
{
	// TIMER0 is an 8-bit timer which we set up in fast PWM mode
	bit_set_hi(TCCR0A, WGM01);
	bit_set_hi(TCCR0A, WGM00);

	// set TIMER0 prescale factor to 64 = 250kHz
	bit_set_hi(TCCR0B, CS01);
	bit_set_hi(TCCR0B, CS00);

	// enable TIMER0 overflow interrupt
	bit_set_hi(TIMSK0, TOIE0);
}