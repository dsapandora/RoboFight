/*
 * Button.c - Functions and Interrupt Service Routines for controlling 
 *	the five push buttons on the Robotis CM-510 controller. 
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
#include <avr/interrupt.h>
#include <util/delay.h>
#include "global.h"
#include "button.h"
#include "buzzer.h"

// Bring in the global variables for use in the ISRs
// Button related variables
extern volatile bool button_up_pressed;
extern volatile bool button_down_pressed;
extern volatile bool button_left_pressed;
extern volatile bool button_right_pressed;
extern volatile bool start_button_pressed;


// define Interrupt Service Routine for START button on INT0 (PD0) pin
ISR(INT0_vect)      
{
	cli();	// disable interrupts
	
	// wait 10ms to make sure we debounce the button
	_delay_ms(10);
	
	// set the global variable
	start_button_pressed = TRUE;

	sei();	// re-enable interrupts
}

// define Interrupt Service Routine for UP button on INT4 (PE4) pin
ISR(INT4_vect) 
{
	cli();	// disable interrupts

	// wait 10ms to make sure we debounce the button
	_delay_ms(10);
	
	// set the global variable
	button_up_pressed = TRUE;
	
	sei();	// re-enable interrupts
}

// define Interrupt Service Routine for DOWN button on INT5 (PE5) pin
ISR(INT5_vect)      
{
	cli();	// disable interrupts

	// wait 10ms to make sure we debounce the button
	_delay_ms(10);
	
	// set the global variable
	button_down_pressed = TRUE;
	
	sei();	// re-enable interrupts
}

// define Interrupt Service Routine for LEFT button on INT6 (PE6) pin
ISR(INT6_vect)      
{
	cli();	// disable interrupts

	// wait 10ms to make sure we debounce the button
	_delay_ms(10);
	
	// set the global variable
	button_left_pressed = TRUE;
	
	sei();	// re-enable interrupts
}


// define Interrupt Service Routine for RIGHT button on INT7 (PE7) pin
ISR(INT7_vect)      
{
	cli();	// disable interrupts

	// wait 10ms to make sure we debounce the button
	_delay_ms(10);
	
	// set the global variable
	button_right_pressed = TRUE;
	
	sei();	// re-enable interrupts
}

// Initializations of Push Buttons as inputs
void button_init(void)
{
	// configure all five buttons as input
	BUTTON_DDR &= ~ALL_BUTTONS;		// set the pushbutton pins to be inputs
	START_BUTTON_DDR &= ~START_BUTTON;	// same for START button
	// activate the pull-up resistors for PORTE
	BUTTON_PORT |= 0xF0;
	
	// Set up the external interrupts
	// interrupt on INT0 pin rising edge (button push = HIGH) 
	EICRA =  (1<<ISC01 | 1<<ISC00);

	// interrupt on INT4/5/6/7 pin on LOW level (button push = LOW) 
	EICRB = 0x00;

	// turn on interrupts
	EIMSK = (1<<INT7) | (1<<INT6) | (1<<INT5) | (1<<INT4) | (1<<INT0);
}
