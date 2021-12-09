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

#define MAX(x,y) (((x) > (y)) ? (x) : (y))
#define MIN(x,y) (((x) < (y)) ? (x) : (y))

static const int JoystickCenterX = 32000;
static const int JoystickCenterY = 32000;
static const int JoystickMagnitudeMax = 18000;

static const float JoyForwardAngle = 1.57;
static const float JoyLeftTurnAngle = 3.142;
static const float JoyRightTurnAngle = 0;
static const float JoyTurnAngleDeadzone = 0.05;
static const float JoyForwardAngleDeadzone = 0.2;

static JoystickHandle prev_joystick = {.x = -1, .y = -1};

/*********************************************************************
 * @fn      		- clamp
 *
 * @brief          	- Clamp x between min and max
 *
 * @param[x]     		- input value
 * @param[min]     		- min value allowed
 * @param[max]     		- max value allowed
 *
 * @return            - None
 *
 * @Note              -
 */
static int clamp(int x, int min, int max)
{
  return MIN(MAX(min, x), max);
}

/*********************************************************************
 * @fn      		  - calculatePos
 *
 * @brief             - Calculate joystick x and y value by fitlering the raw data and considering the deadzone
 *
 * @return            - None
 *
 * @Note              -
 */
void calculatePos(JoystickHandle* joystick_handler)
{
	double angle;

	//Normalize joystick input
	//Depends on type of joystick use
	joystick_handler->x = (joystick_handler->x < 0)? joystick_handler->x + JoystickCenterX : joystick_handler->x - JoystickCenterX;
	joystick_handler->y = (joystick_handler->y > 0)? JoystickCenterY - joystick_handler->y:  -joystick_handler->y - JoystickCenterX;

	//return if joystick just initialize
	if (prev_joystick.x == 0 || prev_joystick.y == 0){
		prev_joystick.x = joystick_handler->x;
		prev_joystick.y = joystick_handler->y;
		return;
	}

	//Smoothening joystick reading by using EMA as low pass
	joystick_handler->x = (float)joystick_handler->x * EXPONENTIAL_ALPHA + (1.0 - EXPONENTIAL_ALPHA) * (float)prev_joystick.x;
	joystick_handler->y = (float)joystick_handler->y * EXPONENTIAL_ALPHA + (1.0 - EXPONENTIAL_ALPHA) * (float)prev_joystick.y;

	// calculate joystick magnitude and angle
	angle = atan2((double)joystick_handler->y, (double)joystick_handler->x);

	// limit magnitude
	joystick_handler->x = clamp(joystick_handler->x, -JoystickMagnitudeMax, JoystickMagnitudeMax);
	joystick_handler->y = clamp(joystick_handler->y, -JoystickMagnitudeMax, JoystickMagnitudeMax);

	// filter joystick forward angle deadzone
	if (angle > JoyForwardAngle - JoyForwardAngleDeadzone
			&& angle < JoyForwardAngle + JoyForwardAngleDeadzone)
		joystick_handler->x = 0;

	// filter joystick backward angle deadzone
	if (angle > -(JoyForwardAngle + JoyForwardAngleDeadzone)
			&& angle < -(JoyForwardAngle - JoyForwardAngleDeadzone))
		joystick_handler->x = 0;

	// filter joystick right turn deadzone
	if (angle > JoyRightTurnAngle - JoyTurnAngleDeadzone
			&& angle < JoyRightTurnAngle + JoyTurnAngleDeadzone)
		joystick_handler->y = 0;

	// filter joystick left turn deadzone
	if (angle > JoyLeftTurnAngle - JoyTurnAngleDeadzone
			|| angle < -JoyLeftTurnAngle + JoyTurnAngleDeadzone)
		joystick_handler->y = 0;
}
