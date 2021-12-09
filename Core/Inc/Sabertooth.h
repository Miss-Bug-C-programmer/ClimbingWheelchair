/*
 * Sabertooth.h
 *
 * This file is to target communicate with sabertooth through Serial Communication
 *  Created on: Dec 3, 2021
 *      Author: ray
 */

#ifndef INC_SABERTOOTH_H_
#define INC_SABERTOOTH_H_

#include <stdint.h>
#include <stm32f4xx.h>

//range that acceptable by sabertooth for value byte
#define SABERTOOTH_MIN_ALLOWABLE_VALUE				-(0x07FF)
#define SABERTOOTH_MAX_ALLOWABLE_VALUE				+(0x07FF)

//Targeted output number associated with output type
#define TARGET_1				1
#define TARGET_2				2
#define TARGET_BOTH				(uint8_t)'*'

typedef struct{
	int16_t duty_cycle;
	int16_t battery;
	int16_t current;
	int16_t temp;
}Sabertooth_Motor_Handler;
typedef struct{
  uint8_t address;
  UART_HandleTypeDef *huart;
  Sabertooth_Motor_Handler motor1;
  Sabertooth_Motor_Handler motor2;
}Sabertooth_Handler;

extern uint8_t motor_receive_buf[9];
extern uint8_t motor_need_receive;		//A flag to indicate need to call DMA_RX

void MotorInit(Sabertooth_Handler* st_handler, uint8_t address, UART_HandleTypeDef* huart);

/*!
 * Sets the power of the specified motor.
 * param st_handler 	pointer to sabertooth.
 * param motor 	The motor number, 1 or 2.
 * param power 	The power, between -2047 and 2047.
 */
void MotorThrottle(Sabertooth_Handler* st_handler, uint8_t motor, int16_t power);

/*!
 * Stops.
 */
void MotorStop(Sabertooth_Handler* st_handler);

/*!
 * shutdown of the specified motor.
 * Put the motor in hard brake state
 * param st_handler 	pointer to sabertooth.
 */
void MotorShutdown(Sabertooth_Handler* st_handler);

/*!
 * Startup of the specified motor.
 * Return motor from shutdown state to normal op
 * param st_handler 	pointer to sabertooth.
 */
void MotorStartup(Sabertooth_Handler* st_handler);

/*!
 * Sets the serial timeout of the specified motor.
 * param st_handler 	pointer to sabertooth.
 * param value 		0 use DEScribe setting, -ve disable serial timeout, in millisecond
 */
void MotorTimeout(Sabertooth_Handler* st_handler, int16_t value);

/*!
 * Read battery of the motor. Get in tenths of a volts
 * Both motor share the battery voltage
 * param st_handler 	pointer to sabertooth.
 */
void MotorReadBattery(Sabertooth_Handler* st_handler);

/*!
 * Get the motor currents, in amps.
 * param st_handler 	pointer to sabertooth.
 * param motor 		The motor number, 1 or 2.
 */
void MotorReadCurrent(Sabertooth_Handler* st_handler, uint8_t motor);

/*!
 * Get the motor temperature, in degrees celcius.
 * param st_handler 	pointer to sabertooth.
 * param motor 		The motor number, 1 or 2.
 */
void MotorReadTemperature(Sabertooth_Handler* st_handler, uint8_t motor);

/*!
 * Get the motor duty cycle, -2047,2047.
 * param st_handler 	pointer to sabertooth.
 * param motor 		The motor number, 1 or 2.
 */
void MotorReadDutyCycle(Sabertooth_Handler* st_handler, uint8_t motor);

void MotorProcessReply(Sabertooth_Handler *st_handler, uint8_t *receive_buf, uint8_t size);

#endif /* INC_SABERTOOTH_H_ */
