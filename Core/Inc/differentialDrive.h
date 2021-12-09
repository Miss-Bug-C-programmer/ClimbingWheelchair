/*
 * differentialDrive.h
 *
 *  Created on: 26 Nov 2021
 *      Author: ray
 *      Convert joystick data to velocity output
 */

#ifndef INC_DIFFERENTIALDRIVE_H_
#define INC_DIFFERENTIALDRIVE_H_

#define COMPUTERANGE 18000


#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "stm32f4xx.h"
//user need to input the following

typedef enum{
	GEAR1 = 40,
	GEAR2 = 60,
	GEAR3 = 80,
	GEAR4 = 100,
}Gear_Level;

typedef struct{
	float min_vel; 			/*!< Motor minimum velocity >*/
	float max_vel; 			/*!< Motor maximum speed >*/
	float min_acc; 			/*!< Motor minimum acceleration >*/
	float max_acc;			/*!< Motor maximum acceleration >*/
	float min_jerk; 		/*!< Motor minimum jerk >*/
	float max_jerk;			/*!< Motor maximum jerk >*/
}speedConfig;

typedef struct{
	float vel;
	float prev_vel;
	uint32_t prev_t;
}motorState;

typedef struct{
	float m_leftMotor;	/*!< Motor (left)  mixed output (-1..+1) >*/
	float m_rightMotor;	/*!< Motor (right) mixed output (-1..+1) >*/
	int m_fPivYLimit;	/*!<The threshold at which the pivot action starts
                		This threshold is measured in units on the Y-axis
                		away from the X-axis (Y=0). A greater value will assign
                		more of the joystick's range to pivot actions.
                		Allowable range: (0..+COMPUTERANGE)>*/
	motorState left_motor_state;	/*!< Motor (left) state >*/
	motorState right_motor_state;	/*!< Motor (right) state >*/
	uint32_t frequency; 			/*!< To smoothen output >*/
}differentialDrive_Handler;

//User configurable output speed characteristic
extern speedConfig speed_config;

void differentialDriveInit(differentialDrive_Handler* dd_handler, uint32_t frequency);
void speedConfiguration(speedConfig* speed_config);
void computeSpeed(differentialDrive_Handler* dd_handler, int XValue, int YValue, Gear_Level gear_level);
void speedLimiter(motorState* motor_state, uint32_t frequency, float velocity);

extern differentialDrive_Handler differential_drive_handler;


#endif /* INC_DIFFERENTIALDRIVE_H_ */
