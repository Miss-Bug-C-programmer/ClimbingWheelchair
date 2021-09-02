/*
 * X2_6010S.h
 *	Motor Hub header file
 *  Created on: May 22, 2021
 *      Author: angcx
 */

#define HubM_UART3_TX_Pin 			GPIO_PIN_8
#define HubM_UART3_TX_GPIO_Port 	GPIOD
#define HubM_UART3_RX_Pin 			GPIO_PIN_9
#define HubM_UART3_RX_GPIO_Port 	GPIOD
#define HubM_IO_ALM_Pin 			GPIO_PIN_12
#define HubM_IO_ALM_GPIO_Port 		GPIOD
#define HubM_IO_SON_Pin 			GPIO_PIN_13
#define HubM_IO_SON_GPIO_Port 		GPIOD
#define HubM_IO_NOT_Pin 			GPIO_PIN_14
#define HubM_IO_NOT_GPIO_Port 		GPIOD
#define HubM_IO_POT_Pin 			GPIO_PIN_15
#define HubM_IO_POT_GPIO_Port 		GPIOD

#ifndef X2_6010S_H
#define X2_6010S_H

#include "stm32f4xx_hal.h"
#include <math.h>
#include "main.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

typedef struct{
  int32_t encoder_1;
  int32_t encoder_2;
}Encoder_Feedback;

extern Encoder_Feedback hub_encoder_feedback;

enum MOTOR_ADDRESS_ENUM{
	MOTOR_1 = 0xA1,
	MOTOR_2 = 0xA2,
	MOTOR_3 = 0xA3,
	MOTOR_4 = 0xA4
};

typedef enum{
	MOTOR_ENABLE = 0x00,
	MOTOR_BRAKE= 0x01,
	MOTOR_DISABLE = 0x02,
	MOTOR_CLR_ALARM = 0x03
}Motor_Command;

typedef enum{
	MOTOR_ENCODER_FEEDBACK = 0x80,
	MOTOR_CONTROLLER_FEEDBACK= 0x82,
	MOTOR_VELOCITY_CURRENT_FEEDBACK = 0x83,
}Motor_Feedback_Command;


/**
  * @brief Initialize motor by setting all IO pin to LOW
  * @param None
  * @retval None
  */
void hubMotor_Init();

void hubMotor_Stop();
/**
  * @brief send speed command to Hub motor
  * Need add HAL_DMA_RECEIVE_CPT_CALLBACK to receive message
  * @param ang velocity
  * @retval None
  */
void send_HubMotor(float m1_ang_speed, float m2_ang_speed);

//TODO:Received message process
/**
  * @brief receive speed command to Hub motor
  * Need add HAL_DMA_RECEIVE_CPT_CALLBACK to receive message
  * @param None
  * @retval None
  */
//Encoder_Feedback receiveHubMotor(uint8_t receive_buf[]);



#endif
