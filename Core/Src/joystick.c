/*
 * joystick.c
 *
 *  Created on: 26 Nov 2021
 *      Author: ray
 */

#include "joystick.h"
#include <math.h>
#include <stdlib.h>
#define EXPONENTIAL_ALPHA 0.85

static const int JoystickCenterX = 32000;
static const int JoystickCenterY = 32000;
static const int JoystickMagnitudeMax = 18000;


static const float JoyForwardAngle = 1.57;
static const float JoyLeftTurnAngle = 3.142;
static const float JoyRightTurnAngle = 0;
static const float JoyForwardAngleDeadzone = 0.3;
static const float JoyTurnAngleDeadzone = 0.2;

static JoystickHandle prev_joystick = {.x = -1, .y = -1};

void calculatePos(void)
{
	double angle;

	//Normalize joystick input
	//Depends on type of joystick use
	raw_hJoystick.x = (raw_hJoystick.x < 0)? raw_hJoystick.x + JoystickCenterX : raw_hJoystick.x - JoystickCenterX;
	raw_hJoystick.y = (raw_hJoystick.y > 0)? JoystickCenterY - raw_hJoystick.y:  -raw_hJoystick.y - JoystickCenterX;

	//Store in processed data for clarity
	processed_hjoystick.x = raw_hJoystick.x;
	processed_hjoystick.y = raw_hJoystick.y;

	//return if joystick just initialize
	if (prev_joystick.x == 0 || prev_joystick.y == 0){
		prev_joystick.x = processed_hjoystick.x;
		prev_joystick.y = processed_hjoystick.y;
		return;
	}

	//Smoothening joystick reading by using EMA as low pass
	processed_hjoystick.x = (float)processed_hjoystick.x * EXPONENTIAL_ALPHA + (1.0 - EXPONENTIAL_ALPHA) * (float)prev_joystick.x;
	processed_hjoystick.y = (float)processed_hjoystick.y * EXPONENTIAL_ALPHA + (1.0 - EXPONENTIAL_ALPHA) * (float)prev_joystick.y;

	// calculate joystick magnitude and angle
	angle = atan2((double)processed_hjoystick.y, (double)processed_hjoystick.x);

	// limit magnitude
	if (processed_hjoystick.x > JoystickMagnitudeMax)
		processed_hjoystick.x = JoystickMagnitudeMax;
	if (processed_hjoystick.y > JoystickMagnitudeMax)
		processed_hjoystick.y = JoystickMagnitudeMax;

	// filter joystick forward deadzone
	if (angle > JoyForwardAngle - JoyForwardAngleDeadzone
			&& angle < JoyForwardAngle + JoyForwardAngleDeadzone)
		processed_hjoystick.x = 0;

	// filter joystick backward deadzone
	if (angle > -(JoyForwardAngle + JoyForwardAngleDeadzone)
			&& angle < -(JoyForwardAngle - JoyForwardAngleDeadzone))
		processed_hjoystick.x = 0;

	// filter joystick right turn deadzone
	if (angle > JoyRightTurnAngle - JoyTurnAngleDeadzone
			&& angle < JoyRightTurnAngle + JoyTurnAngleDeadzone)
		processed_hjoystick.y = 0;

	// filter joystick left turn deadzone
	if (angle > JoyLeftTurnAngle - JoyTurnAngleDeadzone
			|| angle < -JoyLeftTurnAngle + JoyTurnAngleDeadzone)
		processed_hjoystick.y = 0;


}
