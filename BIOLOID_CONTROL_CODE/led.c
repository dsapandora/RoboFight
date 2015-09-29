/*
 * Led.c - Functions for controlling the six LEDs 
 *	on the Robotis CM-510 controller. 
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


#include <avr/io.h>
#include "global.h"
#include "led.h"

// initialize the LED port
void led_init()
{
	// all LEDs are on PORTC0-PORTC6
	DDRC  = 0x7F;  // configure as output
	PORTC = 0x00;  // switch all LEDs on
}

// toggle the status of the specified LED
void led_toggle(uint8 ledIndex)
{
	PORTC ^= ledIndex;
}

// switch the specified LED on (pin low)
void led_on(uint8 ledIndex)
{
	PORTC &= ~ledIndex;
}

// switch the specified LED off (pin high)
void led_off(uint8 ledIndex)
{
	PORTC |= ledIndex;
}
