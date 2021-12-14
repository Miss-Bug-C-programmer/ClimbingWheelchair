/*
 * DifferentialDrivetoSabertooth.c
 *
 *  Created on: 9 Dec 2021
 *      Author: ray
 */
#include "DifferentialDrivetoSabertooth.h"

void differentialDrivetoSabertoothOutputAdapter(differentialDrive_Handler* dd_handler, Sabertooth_Handler *st_handler){

	speedLimiter(&(dd_handler->left_motor_state), dd_handler->frequency, dd_handler->m_leftMotor);
	speedLimiter(&(dd_handler->right_motor_state), dd_handler->frequency, dd_handler->m_rightMotor);

	dd_handler->m_leftMotor = dd_handler->left_motor_state.vel;
	dd_handler->m_rightMotor = dd_handler->right_motor_state.vel;

	int motor_output_1 = dd_handler->m_leftMotor * SABERTOOTH_MAX_ALLOWABLE_VALUE;
	int motor_output_2 = dd_handler->m_rightMotor * SABERTOOTH_MAX_ALLOWABLE_VALUE;
	MotorThrottle(st_handler, TARGET_2, motor_output_1);
	MotorThrottle(st_handler, TARGET_1, motor_output_2);
}
