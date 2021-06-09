/** @file bd25l.h
 *  @brief Header for motor driver bd25l.
 *
 *  @author ANG CHIN XIAN
 */



/*GPIO Congiguration*/
//#define ClimbM_IO_FR2_Pin 			GPIO_PIN_0
//#define ClimbM_IO_FR2_GPIO_Port 	GPIOB
//#define ClimbM_IO_EN2_Pin 			GPIO_PIN_1
//#define ClimbM_IO_EN2_GPIO_Port 	GPIOB
//#define ClimbM_IO_BRK2_Pin 			GPIO_PIN_7
//#define ClimbM_IO_BRK2_GPIO_Port 	GPIOE
//#define ClimbM_IO_ALM2_Pin 			GPIO_PIN_8
//#define ClimbM_IO_ALM2_GPIO_Port 	GPIOE
//#define Climb_TIM1_CH1_Pin 			GPIO_PIN_9
//#define Climb_TIM1_CH1_GPIO_Port 	GPIOE
//#define Climb_TIM8_CH4_Pin 			GPIO_PIN_9
//#define Climb_TIM8_CH4_GPIO_Port 	GPIOC
//#define ClimbM_IO_FR1_Pin 			GPIO_PIN_12
//#define ClimbM_IO_FR1_GPIO_Port 	GPIOE
//#define ClimbM_IO_EN1_Pin 			GPIO_PIN_13
//#define ClimbM_IO_EN1_GPIO_Port 	GPIOE
//#define ClimbM_IO_BRK1_Pin 			GPIO_PIN_14
//#define ClimbM_IO_BRK1_GPIO_Port 	GPIOE
//#define ClimbM_IO_ALM1_Pin 			GPIO_PIN_15
#define ClimbM_IO_ALM1_GPIO_Port 	GPIOE
//#define ClimbSpeed_TIM2_CH3_Pin 		GPIO_PIN_10
//#define ClimbSpeed_TIM2_CH3_GPIO_Port 	GPIOB
//#define ClimbSpeed_TIM2_CH4_Pin 		GPIO_PIN_11
//#define ClimbSpeed_TIM2_CH4_GPIO_Port 	GPIOB

#ifndef _BD25L_H
#define _BD25L_H

#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "main.h"
#include "tim.h"

typedef struct{
	/*Input Signal*/
	GPIO_TypeDef* 			ALM_port;
	uint16_t			ALM_pin;
	TIM_HandleTypeDef* 		speedCapture; //Input Capture: Speed
	uint32_t			speedCapture_Channel; //High: Brake
	uint32_t			detectedSpeed ; //High: Brake

	/*Output Signal*/
	TIM_HandleTypeDef* 		outputPWM; //Output PWM
	uint32_t			PWM_channel;

	GPIO_TypeDef* 			BRK_port;
	uint16_t			BRK_pin; //High: Brake

	GPIO_TypeDef*  			EN_port;
	uint16_t			EN_pin; //High: Brake

	GPIO_TypeDef*  			DIR_port;
	uint16_t			DIR_pin; //High: Brake

	uint8_t				pole;
	uint8_t				ID;

}Motor_TypeDef;

/** @brief Enable motor to start or stop its operation. Stop does slowly unlike brake
 *  @params motor entity, state low enable, high stop motor slowly
 *  @return Void
 */
void enableMotor(Motor_TypeDef* motor, uint8_t state);

/** @brief Enable motor to start or stop its operation. Stop does slowly unlike brake
 *  @params motor entity, high stop motor immediately
 *  @return Void
 */
void brakeMotor(Motor_TypeDef* motor, uint8_t state);

/** @brief Control motor dir
 *  @params high forward
 *  @return Void
 */
void setMotorDir(Motor_TypeDef* motor, uint8_t dir);

/** @brief Set motor speed by varying frequency of PWM
 *  @params speed is the rpm
 *  @return Void
 */
void setMotorSpeed(Motor_TypeDef* motor, float speed);

/** @brief Read motor speed
 *  @params motor entity
 *  @return Void
 */
//readMotorSpeed(MotorTypeDef* motor);

/** @brief Read motor error status
 *  @params motor entity
 *  @return -1 if error occur
 */
uint8_t readErrorStatus(Motor_TypeDef* motor);

/** @brief Run motor
 *  @params motor entity
 *  	dir high forward,low reverse
 *
 *  @return
 */
void runMotor(Motor_TypeDef* motor, float speed, uint8_t dir);
#endif
