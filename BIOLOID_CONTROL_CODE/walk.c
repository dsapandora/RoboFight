/*
 * walk.c - Functions for walking
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

#include <stdio.h>
#include "global.h"
#include "motion_f.h"
#include "dynamixel.h"
#include "walk.h"

// Global variables related to the finite state machine that governs execution
extern volatile uint8 bioloid_command;			// current command
extern volatile uint8 last_bioloid_command;		// last command
extern volatile bool  new_command;				// flag that we got a new command
extern volatile uint8 next_motion_page;			// next motion page if we got new command

// global variables for distance sensors
extern volatile uint16 adc_ultrasonic_distance;	// ultrasonic distance sensor value
extern volatile uint16 adc_dms_distance;	    // DMS sensor distance value

// global variable that keeps the current motion page
extern uint8 current_motion_page;

// the functions share some variables to keep the walking state
static uint8 walk_command = 0;
static uint8 walk_state = 0;

// initialize for walking - assume walk ready pose
void walk_init()
{
	int commStatus = 0;
	
	// reset walk state and command
	walk_state = 0;
	walk_command = 0;
	
	// and get ready for walking!
	current_motion_page = COMMAND_WALK_READY_MP;
	executeMotion(current_motion_page);
	
	// experimental - increase punch for walking
	commStatus = dxl_write_word(BROADCAST_ID, DXL_PUNCH_L, 100);
	if(commStatus != COMM_RXSUCCESS) {
		printf("\nDXL_PUNCH Broadcast - ");
		dxl_printCommStatus(dxl_get_result());
	}	
}

// function to update the walk state
void walk_setWalkState(int command)
{
	// the walk state simply corresponds to the command
	walk_state = command;
}

// function to retrieve the walk state
// Returns (int) walk state
int walk_getWalkState()
{
	// return current walk state
	return walk_state;
}

// Function that allows 'seamless' transition between certain walk commands
// Handles transitions between 1. WFWD - WFLS - WFRS and
//                             2. WBWD - WBLS - WBRS
// All other transitions between walk commands have to go via their exit page
// Returns:	(int)	shift flag 0 - nothing happened
//							   1 - new motion page set
int walk_shift()
{
	// first check that the current command is a walk command
	if ( bioloid_command < COMMAND_WALK_FORWARD || bioloid_command > COMMAND_WALK_BWD_TURN_RIGHT )
	{
		// nothing to do here, return
		return 0;
	}
	
	// next we deal with the special cases - walk forward related first
	if ( walk_state == 1 && bioloid_command == COMMAND_WALK_FWD_LEFT_SIDE )
	{
		// Transition WFWD -> WFLS
		if ( current_motion_page == 35 || current_motion_page == 39 )
		{
			// nothing to do yet, return
			return 0;
		} else {
			// last step is finished, switch motion page
			current_motion_page = 108;
			walk_state = 7;
			return 1;
		} 
	} else if ( walk_state == 1 && bioloid_command == COMMAND_WALK_FWD_RIGHT_SIDE )
	{
		// Transition WFWD -> WFRS
		if ( current_motion_page == 33 || current_motion_page == 37 )
		{
			// nothing to do yet, return
			return 0;
		} else {
			// last step is finished, switch motion page
			current_motion_page = 122;
			walk_state = 8;
			return 1;
		} 
	} else if ( walk_state == 7 && (bioloid_command == COMMAND_WALK_FWD_RIGHT_SIDE || bioloid_command == COMMAND_WALK_FORWARD) )
	{
		// Transition WFLS -> WFRS or WFWD
		if ( current_motion_page == 111 )
		{
			// nothing to do yet, return
			return 0;
		} else {
			// last step is finished, switch motion page
			current_motion_page = 36;
			walk_state = 1;
			return 1;
		} 
	} else if ( walk_state == 8 && (bioloid_command == COMMAND_WALK_FWD_LEFT_SIDE || bioloid_command == COMMAND_WALK_FORWARD) )
	{
		// Transition WFRS -> WFLS or WFWD
		if ( current_motion_page == 121 )
		{
			// nothing to do yet, return
			return 0;
		} else {
			// last step is finished, switch motion page
			current_motion_page = 38;
			walk_state = 1;
			return 1;
		} 
	}		

	// now the walk backward related special cases 
	if ( walk_state == 2 && bioloid_command == COMMAND_WALK_BWD_LEFT_SIDE )
	{
		// Transition WBWD -> WBLS
		if ( current_motion_page == 45 || current_motion_page == 49 )
		{
			// nothing to do yet, return
			return 0;
		} else {
			// last step is finished, switch motion page
			current_motion_page = 134;
			walk_state = 9;
			return 1;
		} 
	} else if ( walk_state == 2 && bioloid_command == COMMAND_WALK_BWD_RIGHT_SIDE )
	{
		// Transition WBWD -> WBRS
		if ( current_motion_page == 47 || current_motion_page == 51 )
		{
			// nothing to do yet, return
			return 0;
		} else {
			// last step is finished, switch motion page
			current_motion_page = 144;
			walk_state = 10;
			return 1;
		} 
	} else if ( walk_state == 9 && (bioloid_command == COMMAND_WALK_BWD_RIGHT_SIDE || bioloid_command == COMMAND_WALK_BACKWARD) )
	{
		// Transition WBLS -> WBRS or WBWD
		if ( current_motion_page == 133 )
		{
			// nothing to do yet, return
			return 0;
		} else {
			// last step is finished, switch motion page
			current_motion_page = 50;
			walk_state = 2;
			return 1;
		} 
	} else if ( walk_state == 10 && (bioloid_command == COMMAND_WALK_BWD_LEFT_SIDE || bioloid_command == COMMAND_WALK_BACKWARD) )
	{
		// Transition WBRS -> WBLS or WBWD
		if ( current_motion_page == 147 )
		{
			// nothing to do yet, return
			return 0;
		} else {
			// last step is finished, switch motion page
			current_motion_page = 48;
			walk_state = 2;
			return 1;
		} 
	}		
	
	// in all other cases nothing happened
	return 0;
}

// function to avoid obstacles by turning left until path is clear
// Input:	obstacle flag from last execution
// Returns:	(int) obstacle flag 0 - no obstacle
//								1 - new obstacle, start avoiding
//								2 - currently avoiding obstacle
//							   -1 - finished avoiding
int walk_avoidObstacle(int obstacle_flag)
{
	// first check if we are currently in obstacle avoidance mode
	if ( obstacle_flag == 1 || obstacle_flag == 2 )
	{
		if ( adc_dms_distance > SAFE_DISTANCE && adc_ultrasonic_distance > SAFE_DISTANCE )
		{
			// have cleared the obstacle, return to walking forward
			last_bioloid_command = bioloid_command;
			bioloid_command = COMMAND_WALK_FORWARD;
			return -1;
		} else {
			// still avoiding, return
			return 2;
		}
	}
	
	// next check for a new obstacle
	if ( obstacle_flag == 0 || obstacle_flag == -1 )
	{
		if ( adc_dms_distance < MINIMUM_DISTANCE || adc_ultrasonic_distance < MINIMUM_DISTANCE )
		{
			// have found an obstacle, start turning left
			last_bioloid_command = bioloid_command;
			bioloid_command = COMMAND_WALK_TURN_LEFT;
			return 1;
		} else {
			// no obstacle, return
			return 0;
		}
	}
	// not needed, but avoids compiler warning :)
	return 0;
}
