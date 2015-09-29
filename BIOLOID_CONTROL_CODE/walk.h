/*
 * walk.h - functions for walking 
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


#ifndef WALK_H_
#define WALK_H_

// initialize for walking - assume walk ready pose
void walk_init();

// function to update the walk state
void walk_setWalkState(int command);

// function to retrieve the walk state
// Returns (int) walk state
int walk_getWalkState();

// Function that allows 'seamless' transition between certain walk commands
// Handles transitions between 1. WFWD - WFLS - WFRS and
//                             2. WBWD - WBLS - WBRS
// All other transitions between walk commands have to go via their exit page
// Returns:	(int)	shift flag 0 - nothing happened
//							   1 - new motion page set
int walk_shift();

// function to avoid obstacles by turning left until path is clear
// Input:	obstacle flag from last execution
// Returns:	(int) obstacle flag 0 - no obstacle
//								1 - new obstacle, start avoiding
//								2 - currently avoiding obstacle
//							   -1 - finished avoiding
int walk_avoidObstacle(int obstacle_flag);

#endif /* WALK_H_ */