/** @file bd25l.h
 *  @brief source for motor driver bd25l.
 *
 *  @author ANG CHIN XIAN
 */

#include "bd25l.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;

Motor_TypeDef rearMotor =
    {
	.ALM_port	= ClimbM_IO_ALM1_GPIO_Port,
	.ALM_pin	= ClimbM_IO_ALM1_Pin,
//	.speedCapture	= &htim2,
//	.speedCapture_Channel = HAL_TIM_ACTIVE_CHANNEL_3,
	.outputPWM 	= &htim2,
	.PWM_channel	= TIM_CHANNEL_3,
//	.outputPWM 	= &htim8,
//	.PWM_channel	= TIM_CHANNEL_4,
	.BRK_port	= ClimbM_IO_BRK1_GPIO_Port,
	.BRK_pin	= ClimbM_IO_BRK1_Pin,
	.EN_port	= ClimbM_IO_EN1_GPIO_Port,
	.EN_pin		= ClimbM_IO_EN1_Pin,
	.DIR_port	= ClimbM_IO_FR1_GPIO_Port,
	.DIR_pin	= ClimbM_IO_FR1_Pin,
	.pole		= 4,
	.ID		= 1,
};

Motor_TypeDef backMotor =
    {
	.ALM_port	= ClimbM_IO_ALM2_GPIO_Port,
	.ALM_pin	= ClimbM_IO_ALM2_Pin,
	.speedCapture	= &htim2,
	.speedCapture_Channel = HAL_TIM_ACTIVE_CHANNEL_4,
//	.outputPWM 	= &htim8,
//	.PWM_channel	= TIM_CHANNEL_4,
	.outputPWM 	= &htim1,
	.PWM_channel	= TIM_CHANNEL_2,
	.BRK_port	= ClimbM_IO_BRK2_GPIO_Port,
	.BRK_pin	= ClimbM_IO_BRK2_Pin,
	.EN_port	= ClimbM_IO_EN2_GPIO_Port,
	.EN_pin		= ClimbM_IO_EN2_Pin,
	.DIR_port	= ClimbM_IO_FR2_GPIO_Port,
	.DIR_pin	= ClimbM_IO_FR2_Pin,
	.pole		= 4,
	.ID		= 2,
};

void bd25l_Init(Motor_TypeDef* motor){
	enableMotor(motor, 0);
	brakeMotor(motor, 0);
}

void bd25l_DeInit(Motor_TypeDef* motor){
	brakeMotor(motor, 0);
	enableMotor(motor, 1);
}

void bd25l_Brake(Motor_TypeDef* motor){
	brakeMotor(motor, 1);
	enableMotor(motor, 0);

}

void enableMotor(Motor_TypeDef* motor, uint8_t state){
	HAL_GPIO_WritePin(motor->EN_port, motor->EN_pin, state);
}

void emBrakeMotor(uint8_t state){
	HAL_GPIO_WritePin(Brake_Wheel_GPIO_Port, Brake_Wheel_Pin, state);
}

void brakeMotor(Motor_TypeDef* motor, uint8_t state){
	HAL_GPIO_WritePin(motor->BRK_port, motor->BRK_pin, state);
	HAL_TIM_PWM_Stop(motor->outputPWM, motor->PWM_channel);
}

void setMotorDir(Motor_TypeDef* motor, uint8_t dir){
	HAL_GPIO_WritePin(motor->DIR_port, motor->DIR_pin, dir);
}

void setMotorSpeed(Motor_TypeDef* motor, float speed){

	float frequency = 0;
	uint16_t period;
	uint32_t duty_cycle; //50%

	if (speed > 100) speed = 100.0;
	//Frequency equation derived from data sheet
	frequency = (uint16_t)((speed - 0.2597)/0.02494);
	period = (int)(1e6/frequency)+1;
	duty_cycle = period / 2;

//	if (speed<4){
//	    period = 1;
//	    duty_cycle = 0;
//	    HAL_TIM_PWM_Stop(motor->outputPWM, motor->PWM_channel);
//	    brakeMotor(motor, 1);
//	}
//
//
//	else{
//	    motor->outputPWM->Instance->ARR = period;
//	    if(motor->PWM_channel == TIM_CHANNEL_1)
//	      motor->outputPWM->Instance->CCR1 = duty_cycle;
//	    else if(motor->PWM_channel == TIM_CHANNEL_2)
//		      motor->outputPWM->Instance->CCR2 = duty_cycle;
//	    else if(motor->PWM_channel == TIM_CHANNEL_3)
//		      motor->outputPWM->Instance->CCR3 = duty_cycle;
//	    else if(motor->PWM_channel == TIM_CHANNEL_4)
//	      motor->outputPWM->Instance->CCR4 = duty_cycle;
//
//	    HAL_TIM_PWM_Start(motor->outputPWM, motor->PWM_channel);
//	}

	if (speed<4){
		brakeMotor(motor, 1);
	}



	motor->outputPWM->Instance->ARR = period;
	if(motor->PWM_channel == TIM_CHANNEL_1)
	  motor->outputPWM->Instance->CCR1 = duty_cycle;
	else if(motor->PWM_channel == TIM_CHANNEL_2)
		  motor->outputPWM->Instance->CCR2 = duty_cycle;
	else if(motor->PWM_channel == TIM_CHANNEL_3)
		  motor->outputPWM->Instance->CCR3 = duty_cycle;
	else if(motor->PWM_channel == TIM_CHANNEL_4)
	  motor->outputPWM->Instance->CCR4 = duty_cycle;

	HAL_TIM_PWM_Start(motor->outputPWM, motor->PWM_channel);



	motor->outputPWM->Instance->CNT = 0;
}

//weak readMotorSpeed(TIM_HandleTypeDef *htim){
//	static uint32_t IC_Val1 = 0;
//	static uint32_t IC_Val2 = 0;
//	static uint32_t difference1 = 0;
//	static uint8_t is_First_Captured1 = 0;
//
//	static uint32_t IC_Val3 = 0;
//	static uint32_t IC_Val4 = 0;
//	static uint32_t difference2 = 0;
//	static uint8_t is_First_Captured2 = 0;
//
//
//	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
//		if (is_First_Captured1 == 0){
//			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
//			is_First_Captured = 1;
//		}
//		else if (is_First_Captured1 == 1){
//			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
//			if (IC_Val2 > IC_Val1){
//				difference1 = IC_Val2 - IC_Val1;
//				frontMotor.detectedSpeed = (1/(difference1*frontMotor.pole)) * 60/3;
//			}
//			is_First_Captured1 = 0;
//		}
//	}
//
//	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){
//		if (is_First_Captured2 == 0){
//			IC_Val3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
//			is_First_Captured2 = 1;
//		}
//		else if (is_First_Captured2 == 1){
//			IC_Val4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
//			if (IC_Val4 > IC_Val3){
//				difference2 = IC_Val4 - IC_Val3;
//				backMotor.detectedSpeed = (1/(difference2*backMotor.pole)) * 60/3;
//			}
//			is_First_Captured2 = 0;
//		}
//	}
//}

//Might need to set this to no pull
uint8_t readErrorStatus(Motor_TypeDef* motor){
	if (HAL_GPIO_ReadPin(motor->ALM_port, motor->ALM_pin) == 0){
		return -1;
	}
	else return 1;
}

void runMotor(Motor_TypeDef* motor, float speed){
  //	    - positive speed Lift DOWN
  //	    - negative speed lift UP
    if (motor->ID == 1){
	if (fabs(speed)/speed  >= 0)
	      setMotorDir(motor, 0);
	else
	  setMotorDir(motor, 1);
    }
    else if (motor->ID == 2){
	if (fabs(speed)/speed  >= 0)
	  setMotorDir(motor, 1);
	else
	  setMotorDir(motor, 0);
    }

    HAL_Delay(50);
    brakeMotor(motor, 0);
    setMotorSpeed(motor, fabs(speed));

}


