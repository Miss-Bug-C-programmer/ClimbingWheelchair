/*
 * X2_6010S.c
 *
 *  Created on: May 22, 2021
 *      Author: angcx
 */

#include "X2_6010S.h"

void hubMotor_Init(){

	/**USART3 GPIO Configuration
	PD8     ------> USART3_TX
	PD9     ------> USART3_RX
	*/

	HAL_GPIO_WritePin(HubM_IO_ALM_GPIO_Port, HubM_IO_ALM_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HubM_IO_SON_GPIO_Port, HubM_IO_SON_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HubM_IO_NOT_GPIO_Port, HubM_IO_NOT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HubM_IO_POT_GPIO_Port, HubM_IO_POT_Pin, GPIO_PIN_RESET);

}

void send_HubMotor(Motor_ConfigTypeDef* motor, Motor_Command command, Motor_Feedback_Command feedbackCommand,
		uint16_t motor1_speed, uint16_t motor2_speed){
	motor->send_buf[0] = 0xAA;
	motor->send_buf[1] = motor->motorID;
	motor->send_buf[2] = 0x0E;
	motor->send_buf[3] = 0x00;
	motor->send_buf[4] = command;
	motor->send_buf[5] = feedbackCommand;

	//Set acceleration to constant by default
	//time taken from 0 to 1000rpm
	uint16_t acceleration = 51200;
	uint8_t msb_acce = (uint8_t)((acceleration & 0xFF00) >> 8);
	uint8_t lsb_acce = (uint8_t)(acceleration & 0xFF00);
	motor->send_buf[6] = lsb_acce;
	motor->send_buf[7] = msb_acce;

	//Set maximum torque
	//Value: 0 - 450 (300 by default)
	uint16_t max_torque = 0x2C01;
	uint8_t msb_max_torque = (uint8_t)((max_torque & 0xFF00) >> 8);
	uint8_t lsb_max_torque = (uint8_t)(max_torque & 0xFF00);
	motor->send_buf[8] = lsb_max_torque;
	motor->send_buf[9] = msb_max_torque;

	//Set motor1 speed
	uint8_t msb_motor1_speed = (uint8_t)((motor1_speed & 0xFF00) >> 8);
	uint8_t lsb_motor1_speed = (uint8_t)(motor1_speed & 0xFF00);
	motor->send_buf[10] = lsb_motor1_speed;
	motor->send_buf[11] = msb_motor1_speed;

	//Set motor2 speed
	uint8_t msb_motor2_speed = (uint8_t)((motor2_speed & 0xFF00) >> 8);
	uint8_t lsb_motor2_speed = (uint8_t)(motor2_speed & 0xFF00);
	motor->send_buf[12] = lsb_motor2_speed;
	motor->send_buf[13] = msb_motor2_speed;

	//checksum byte
	uint16_t sum = (uint16_t)motor->send_buf[0] + (uint16_t)motor->send_buf[1] + (uint16_t)motor->send_buf[2] + (uint16_t)motor->send_buf[3]
					+ (uint16_t)motor->send_buf[4] + (uint16_t)motor->send_buf[5] + (uint16_t)motor->send_buf[6]
					+ (uint16_t)motor->send_buf[7] + (uint16_t)motor->send_buf[8] + (uint16_t)motor->send_buf[9]
					+ (uint16_t)motor->send_buf[10] + (uint16_t)motor->send_buf[11] + (uint16_t)motor->send_buf[12]
					+ (uint16_t)motor->send_buf[13];

	motor->send_buf[14] = (uint8_t)(sum & 0xFF00);


	HAL_UART_Transmit(&huart3, motor->send_buf, 15, 20);
	HAL_UART_Receive_DMA(&huart3, motor->receive_buf, 15);
}




















