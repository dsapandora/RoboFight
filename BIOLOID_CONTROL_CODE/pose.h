/*
 * pose.h - functions for assuming poses base on motion pages  
 *	also provides the interpolation to smooth out movement 
 * 
 * Version 0.5		31/10/2011
 * Written by Peter Lanius
 * Please send suggestions and bug fixes to PeterLanius@gmail.com
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


#ifndef POSE_H_
#define POSE_H_

// define more readable constants for moveToGoalPose wait_flag
#define WAIT_FOR_POSE_FINISH		1
#define DONT_WAIT_FOR_POSE_FINISH	0

// read in current servo positions to the pose. 
void readCurrentPose();

// Function to wait out any existing servo movement
void waitForPoseFinish();

// Calculate servo speeds to achieve desired pose timing
// We make the following assumptions:
// AX-12 speed is 59rpm @ 12V which corresponds to 0.170s/60deg
// The AX-12 manual states this as the 'no load speed' at 12V
// We ignore the Moving Speed entry which states that 0x3FF = 114rpm
// Instead we assume that Moving Speed 0x3FF = 59rpm
void calculatePoseServoSpeeds(uint16 time);

// Moves from the current pose to the goal pose
// using calculated servo speeds and delay between steps
// to achieve the required step timing (actual play time)
// Inputs:  (uint16)  allocated step time in ms
//          (uint16)  array of goal positions for the actuators
//          (uint8)   flag = 0 don't wait for motion to finish
//					  flag = 1 wait for motion to finish and check alarms
// Returns	(int)	  -1  - communication error
//					   0  - all ok
//					   1  - alarm
int moveToGoalPose(uint16 time, uint16 goal[], uint8 wait_flag);

// Assume default pose (Balance - MotionPage 224)
void moveToDefaultPose(void);

#endif /* POSE_H_ */