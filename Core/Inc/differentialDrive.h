/*
 * differentialDrive.h
 *
 *  Created on: 26 Nov 2021
 *      Author: ray
 */

#ifndef INC_DIFFERENTIALDRIVE_H_
#define INC_DIFFERENTIALDRIVE_H_

#define COMPUTERANGE 18000

typedef struct{
	int m_fPivYLimit;
	int m_leftMotor;
	int m_rightMotor;
}differentialDrive_Handler;

void differentialDriveInit(differentialDrive_Handler* );
void computeMotors(int XValue, int YValue);

extern differentialDrive_Handler* dd_handler;

#endif /* INC_DIFFERENTIALDRIVE_H_ */
