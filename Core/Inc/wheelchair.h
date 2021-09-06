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
#include <stdbool.h>

#define MOTOR_TIM htim3
#define LEFT_MOTOR_CHANNEL CCR2
#define RIGHT_MOTOR_CHANNEL CCR1

typedef struct
{
  int16_t  	x;
  int16_t  	y;
  float     	magnitude;
  float     	angle;
  float		linear;
  float		angular;
}JoystickHandle;

typedef struct
{
  float  	cur_l;
  float  	cur_r;
  float		pre_l;
  float		pre_r;
  int		stable_cnt;
  float		max_linear_speed;
  float		max_angular_speed;
  bool		start_from_stationary;
  float		accel_loop;
  float		decel_loop;
  float		left_speed_step; //if not start from stationary
  float		right_speed_step; //if not start from stationary

}WheelSpeed;

extern JoystickHandle hJoystick;

//Define following in main.c
//Joystick Raw Data
extern int tempJoyRawDataX;
extern int tempJoyRawDataY;

//Base wheel speed
extern WheelSpeed baseWheelSpeed;

//Climb wheel speed
extern WheelSpeed climbWheelSpeed;


void joystick_Init(void);

void joystickCalculatePos(void);

void wheelSpeedControl_Init(WheelSpeed* wheel, float max_lin_speed, float max_ang_speed);

void wheel_Control(WheelSpeed* wheel);

void wheelCalculateSpeed(WheelSpeed* wheel);


#endif // MRBA3_WHEELCHAIR_H
