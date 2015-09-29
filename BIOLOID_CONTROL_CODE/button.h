/*
 * Button.h - Functions and Interrupt Service Routines for controlling 
 *	the five push buttons on the Robotis CM-510 controller. 
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


#ifndef BUTTON_H_
#define BUTTON_H_

// PORTD
// Input goes high when button is pressed 
#define START_BUTTON		0x01	// PORTD0
#define START_BUTTON_DDR	DDRD
#define START_BUTTON_PORT	PORTD

// PORTE
// Input goes high when button is pressed
#define BUTTON_UP		0x10	// PORTE4
#define BUTTON_DOWN		0x20	// PORTE5
#define BUTTON_LEFT		0x40	// PORTE6
#define BUTTON_RIGHT	0x80	// PORTE7

#define BUTTON_DDR		DDRE
#define BUTTON_PORT		PORTE
#define BUTTON_PINS		PINE

#define BUTTONS_HIGH	BUTTON_PINS
#define BUTTONS_LOW		(~BUTTON_PINS)
#define ALL_BUTTONS		(BUTTON_UP | BUTTON_DOWN | BUTTON_LEFT | BUTTON_RIGHT)
#define ANY_BUTTON		ALL_BUTTONS

// function prototypes
void button_init(void);

#endif /* BUTTON_H_ */