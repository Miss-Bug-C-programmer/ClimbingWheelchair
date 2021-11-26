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

extern JoystickHandle processed_hjoystick;
extern JoystickHandle raw_hJoystick;

void calculatePos(void);

#endif /* INC_JOYSTICK_H_ */
