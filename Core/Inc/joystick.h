/*
 * joystick.h
 *
 *  Created on: 26 Nov 2021
 *      Author: ray
 */

#ifndef INC_JOYSTICK_H_
#define INC_JOYSTICK_H_



typedef struct
{
  int  	x;
  int  	y;
}JoystickHandle;

extern JoystickHandle joystick_handler;

void calculatePos(JoystickHandle* joystick_handler);

#endif /* INC_JOYSTICK_H_ */
