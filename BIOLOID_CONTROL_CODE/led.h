/*
 * Led.h - Functions and definitions for controlling 
 *	the six LEDs on the Robotis CM-510 controller. 
 *   
 * Version 0.4		30/09/2011
 * Written by Peter Lanius
 * Please send suggestions and bug fixes to PeterLanius@gmail.com
 *
*/

/*
 * You may freely modify and share this code, as long as you keep this
 * notice intact.  Licensed under the Creative Commons BY-SA 3.0 license:
 *
 *   http://creativecommons.org/licenses/by-sa/3.0/
 *
 * Disclaimer: To the extent permitted by law, this work is provided
 * without any warranty. It might be defective, in which case you agree
 * to be responsible for all resulting costs and damages.
 */

#ifndef LED_H_
#define LED_H_

// Set pin output low to turn LED on (inverted via transistor)
#define LED_POWER		0x01	// PORTC0
#define LED_TXD			0x02	// PORTC1
#define LED_RXD			0x04	// PORTC2
#define LED_AUX			0x08	// PORTC3
#define LED_MANAGE		0x10	// PORTC4
#define LED_PROGRAM		0x20	// PORTC5
#define LED_PLAY		0x40	// PORTC6
#define ALL_LED	(LED_TXD | LED_RXD | LED_AUX | LED_MANAGE | LED_PROGRAM | LED_PLAY )

// initialize the LED functions
void led_init(void);

// toggle specified LED
void led_toggle(uint8 ledIndex);

// switch specified LED on
void led_on(uint8 ledIndex);

// switch specified LED off
void led_off(uint8 ledIndex);

#endif /* LED_H_ */