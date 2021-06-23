/** @file   joystick.h
 *  @brief  Header file of joystick control functions.
 *  @author Rehabilitation Research Institute of Singapore / MRBA Team
 */

#ifndef MRBA3_WHEELCHAIR_H
#define MRBA3_WHEELCHAIR_H

#include <math.h>
#include <string.h>
#include "main.h"
#include "X2_6010S.h"

#define MOTOR_TIM htim3
#define LEFT_MOTOR_CHANNEL CCR1
#define RIGHT_MOTOR_CHANNEL CCR2

typedef struct
{
  int16_t  x;
  int16_t  y;
  float     magnitude;
  float     angle;
}JoystickHandle;

typedef struct
{
  float  l;
  float  r;
}WheelSpeed;

extern JoystickHandle hJoystick;

//Define following in main.c
extern WheelSpeed wheelchr_wheelspeed_cur;
extern WheelSpeed wheelchr_wheelspeed_pre;
extern int tempJoyRawDataX;
extern int tempJoyRawDataY;

//Define in wheelchair.c
extern int speed_level; //change the speed level if need higher speed
extern bool start_from_stationary;

void WHEELCHR_Init(void);

void WHEELCHR_JoystickControl(void);

void WHEELCHR_JoystickCalculatePos(void);

void WHEELCHR_JoystickCalculateSpeed(void);

#endif // MRBA3_WHEELCHAIR_H
