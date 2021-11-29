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
#define FREQUENCY

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
//user need to input the following
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
	uint32_t prev_acc_t; 	/*!< Used to calculate acceleration>*/
	float acc;
	float prev_acc;
	uint32_t prev_jerk_t;	/*!< Used to calculate acceleration>*/
	float jerk;
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
	uint32_t frequency;
}differentialDrive_Handler;





void differentialDriveInit(differentialDrive_Handler* dd_handler, uint32_t frequency);
void computeSpeed(differentialDrive_Handler* dd_handler, int XValue, int YValue);



#endif /* INC_DIFFERENTIALDRIVE_H_ */
