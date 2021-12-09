/*
 * DifferentialDrivetoSabertooth.h
 *
 *  Created on: 9 Dec 2021
 *      Author: ray
 */

#ifndef INC_DIFFERENTIALDRIVETOSABERTOOTH_H_
#define INC_DIFFERENTIALDRIVETOSABERTOOTH_H_

#include "Sabertooth.h"
#include "differentialDrive.h"

void differentialDrivetoSabertoothOutputAdapter(differentialDrive_Handler* dd_handler, Sabertooth_Handler *st_handler);

#endif /* INC_DIFFERENTIALDRIVETOSABERTOOTH_H_ */
