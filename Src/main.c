/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include "adc.h"
//#include "encoder.h"
#include "button.h"
#include "mpu6050.h"
#include "bd25l.h"
#include "X2_6010S.h"
#include "wheelchair.h"
#include "PID.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
  double x;
  double y;
  const int16_t MAX_Y;
  const int16_t MID_Y;
  const int16_t MIN_Y;

  const int16_t MAX_X;
  const int16_t MIN_X;
  const int16_t MID_X;

}Joystick_Def;

//typedef Joystick_Def * joy;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#define ADC_CHANNEL hspi1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//Joystick/ADC
int16_t adc_rawData[8];
uint32_t prev_adc_time = 0;
int tempJoyRawDataX;
int tempJoyRawDataY;

//Wheelchair Base wheel control
WheelSpeed baseWheelSpeed = {.accel_loop = 100.0f, .decel_loop = 200.0f, .left_speed_step = 1.0, .right_speed_step = 1.0};
const float base_linSpeedLevel[3] = {200.0f, 300.0f, 400.0f};
const float base_angSpeedLevel[3] = {100.0f, 150.0f, 200.0f};
int base_speedLevel = 1; //change the speed level if need higher speed

//Lifting Mode
//State
uint8_t lifting_mode = false;
uint8_t front_touchdown = false; //Record the state of climbing wheel whether contact with ground
uint8_t back_touchdown = false;


//Climbing motor
extern Motor_TypeDef rearMotor, backMotor; //declare in bd25l.c
float speed[2] = {0}; //range: 0 - 100




//Hub Motor UART receive
uint8_t receive_buf[15];
int32_t hub_encoder_1 = 0;
int32_t hub_encoder_2 = 0;
WheelSpeed climbWheelSpeed = {.accel_loop = 100.0f, .decel_loop = 200.0f, .left_speed_step = 5.0, .right_speed_step = 5.0};
const float climb_linSpeedLevel[3] = {1000.0f, 3000.0f, 4000.0f};
const float climb_angSpeedLevel[3] = {500.0f, 1500.0f, 2000.0f};
int climb_speedLevel = 0; //change the speed level if need higher speed

//Limit Switches
Button_TypeDef rearLS1 = {
    .gpioPort = LimitSW1_GPIO_Port,
    .gpioPin = LimitSW1_Pin
};
Button_TypeDef rearLS2 = {
    .gpioPort = LimitSW2_GPIO_Port,
    .gpioPin = LimitSW2_Pin
};
Button_TypeDef backLS1 = {
    .gpioPort = LimitSW3_GPIO_Port,
    .gpioPin = LimitSW3_Pin
};
Button_TypeDef backLS2 = {
    .gpioPort = LimitSW4_GPIO_Port,
    .gpioPin = LimitSW4_Pin
};


//Tactile Switches
Button_TypeDef button1 = {
    .gpioPort = Button1_GPIO_Port,
    .gpioPin = Button1_Pin
};
Button_TypeDef button2 = {
    .gpioPort = Button2_GPIO_Port,
    .gpioPin = Button2_Pin
};
Button_TypeDef button3 = {
    .gpioPort = Button3_GPIO_Port,
    .gpioPin = Button3_Pin
};

//Sensor
MPU6050_t MPU6050;
float initial_angle;
uint8_t climb_first_iteration = true;

//Balance control
struct pid_controller balance_ctrl;
PID_t balance_pid;

// Control loop input,output and setpoint variables
float backClimb_input = 0, backClimb_output = 0;
float backClimb_setpoint = 0;

// Control loop gains
float backClimb_kp = 2.5, backClimb_ki = 0.0, backClimb_kd = 0.0;


int state;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void baseMotorCommand(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / FREQUENCY);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  //Initialize hardware communication
  joystick_Init();
  ADC_Init();
  ADC_DataRequest();
//  encoder_Init();
//  DWT_Init();
  while(MPU6050_Init(&hi2c1)==1);
  HAL_Delay(100);

  //Start base wheel pwm pin
  wheelSpeedControl_Init(&baseWheelSpeed, base_linSpeedLevel[base_speedLevel], base_angSpeedLevel[base_speedLevel]);
  HAL_TIM_Base_Start(&MOTOR_TIM);
  HAL_TIM_PWM_Start(&MOTOR_TIM, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&MOTOR_TIM, TIM_CHANNEL_2);
  MOTOR_TIM.Instance->RIGHT_MOTOR_CHANNEL = 1500;
  MOTOR_TIM.Instance->LEFT_MOTOR_CHANNEL = 1500;
  HAL_Delay(100);

  //Initialize rear and back motor
  bd25l_Init(&rearMotor);
  bd25l_Init(&backMotor);
  runMotor(&rearMotor, 0);
  runMotor(&backMotor, 0);
  emBrakeMotor(1);

  //Initialize hub motor
  hubMotor_Init();
  wheelSpeedControl_Init(&climbWheelSpeed, climb_linSpeedLevel[climb_speedLevel], climb_angSpeedLevel[climb_speedLevel]);

  //Initialize balance controller
  // Prepare PID controller for operation
  balance_pid = pid_create(&balance_ctrl, &backClimb_input, &backClimb_output, &backClimb_setpoint, backClimb_kp, backClimb_ki, backClimb_kd);
  // Set controler output limits from 0 to 200
  pid_limits(balance_pid, 0, 30);
  // Allow PID to compute and change output
  pid_auto(balance_pid);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t prev_time = HAL_GetTick();

  //debug variable
//  uint32_t debug_prev_time = HAL_GetTick();
//  uint8_t led_status = 0;
//  float speed = 0;
  while (1)
  {
	//Code to debug with blinking LED
//      if (HAL_GetTick() - debug_prev_time >= 1000){
//	  if (led_status == 0){
////	      count++;
//	      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
//	      led_status = 1;
//	  }
//	  else if (led_status == 1){
//	      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
//	      led_status = 0;
//	  }
//		//Read joystick value
//
//      }

	//Debug BD25L
//      if(speed>100){
//	bd25l_Brake(&rearMotor);
//	bd25l_Brake(&backMotor);
//	speed=0;
//	HAL_Delay(5000);
//      }
//      if (speed<=100){
//	  runMotor(&rearMotor, speed, 0);
//	  runMotor(&backMotor, speed, 0);
//	  speed+=10;
//	  HAL_Delay(1000);
//      }

	//Debug Limit switch
//      rearLS1 = HAL_GPIO_ReadPin(LimitSW1_GPIO_Port, LimitSW1_Pin);
//      rearLS2 = HAL_GPIO_ReadPin(LimitSW2_GPIO_Port, LimitSW2_Pin);
//      backLS1 = HAL_GPIO_ReadPin(LimitSW3_GPIO_Port, LimitSW3_Pin);
//      backLS2 = HAL_GPIO_ReadPin(LimitSW4_GPIO_Port, LimitSW4_Pin);
//      HAL_Delay(500);




//      //Test speed commands on bse motor
//      MOTOR_TIM.Instance->RIGHT_MOTOR_CHANNEL -= 50;
//      MOTOR_TIM.Instance->LEFT_MOTOR_CHANNEL -= 50;
//      runMotor(&backMotor, 100, 1);

//      runMotor(&backMotor, speed++, 1);

    //Loop should execute once every 1 tick
    if(HAL_GetTick() - prev_time >= 1)
    {
	ADC_DataRequest();

	//Get kamlan filtered angle from MPU6050
	MPU6050_Read_All(&hi2c1, &MPU6050);


	GPIO_Digital_Filtered_Input(&button1, 30);
	GPIO_Digital_Filtered_Input(&button2, 30);
	GPIO_Digital_Filtered_Input(&button3, 30);
//
	GPIO_Digital_Filtered_Input(&rearLS1, 5);
	GPIO_Digital_Filtered_Input(&rearLS2, 5);
	GPIO_Digital_Filtered_Input(&backLS1, 5);
	GPIO_Digital_Filtered_Input(&backLS2, 5);

	state = GPIO_Digital_Filtered_Input(&button3, 30);

//	//Climbing wheel start landing when button3 is pressed
//	if (button3.state == 1 && front_touchdown == false && back_touchdown == false && lifting_mode == false){
//	    lifting_mode = true;
//
//	    while(front_touchdown == false || back_touchdown == false){
//	      if (back_touchdown == false)
//		  runMotor(&backMotor, 10);
//	      else
//		runMotor(&backMotor, 0);
//
//	      if (front_touchdown == false)
//		  runMotor(&rearMotor, 10);
//	      else
//		runMotor(&rearMotor, 0);
//
//	      if (GPIO_Digital_Filtered_Input(&rearLS1, 5) || GPIO_Digital_Filtered_Input(&rearLS2, 5))
//		front_touchdown = 1;
//	      if (GPIO_Digital_Filtered_Input(&backLS1, 5) || GPIO_Digital_Filtered_Input(&backLS2, 5))
//		back_touchdown = 1;
//	    }
//	}
//
//	if (!lifting_mode){
//	    //carry out normal wheelchair operation
//	    wheel_Control(&baseWheelSpeed);
//	    baseMotorCommand();
//	}
//	else{
//	    //Lifting up and Down with button control
//	    if (button1.state == GPIO_PIN_SET && button3.state == GPIO_PIN_RESET)
//		speed[FRONT_INDEX] = 30;
//	    else if(button1.state == GPIO_PIN_SET && button3.state == GPIO_PIN_SET)
//		speed[FRONT_INDEX] = -30;
//	    else if (button1.state == GPIO_PIN_RESET)
//		speed[FRONT_INDEX] = 0;
//
//	    if(button2.state == GPIO_PIN_SET && button3.state == GPIO_PIN_RESET)
//		speed[BACK_INDEX] = 30;
//	    else if(button2.state == GPIO_PIN_SET && button3.state == GPIO_PIN_SET)
//		speed[BACK_INDEX] = -30;
//	    else if (button2.state == GPIO_PIN_RESET)
//		speed[BACK_INDEX] = 0;
//
//	    runMotor(&rearMotor, speed[FRONT_INDEX]);
//	    runMotor(&backMotor, speed[BACK_INDEX]);
//
//	    //moving forward and backward with joystick
//	    wheel_Control(&climbWheelSpeed);
//	    send_HubMotor((int)climbWheelSpeed.cur_l, (int)climbWheelSpeed.cur_r);

	    //Lifting with PID controller and single button control
//	    if (button1.state == GPIO_PIN_SET && button3.state == GPIO_PIN_RESET)
//		speed[FRONT_INDEX] = 30;
//	    else if(button1.state == GPIO_PIN_SET && button3.state == GPIO_PIN_SET)
//		speed[FRONT_INDEX] = -30;
//	    else if (button1.state == GPIO_PIN_RESET)
//		speed[FRONT_INDEX] = 0;
//	    runMotor(&rearMotor, speed[FRONT_INDEX]);
//
//	    if (climb_first_iteration){
//		initial_angle = MPU6050.KalmanAngleX;
//		climb_first_iteration = false;
//	    }
//
//
//	    // Check if need to compute PID
//	    if (pid_need_compute(balance_pid) && climb_first_iteration == false && fabs(initial_angle - MPU6050.KalmanAngleX) > 2.0){
//		    // Read process feedback
//		    backClimb_input = (MPU6050.KalmanAngleX - initial_angle) * 5;
//		    // Compute new PID output value
//		    pid_compute(balance_pid);
//		    //Change actuator value
//		    speed[BACK_INDEX] = backClimb_output;
//	    }
//
//
//


//
//	}
//	wheel_Control(&climbWheelSpeed);
//	send_HubMotor((int)climbWheelSpeed.cur_l, (int)climbWheelSpeed.cur_r);

	//Lifting up and Down with button control
	if (button1.state == GPIO_PIN_SET && button3.state == GPIO_PIN_RESET)
	    speed[FRONT_INDEX] = 30;
	else if(button1.state == GPIO_PIN_SET && button3.state == GPIO_PIN_SET)
	    speed[FRONT_INDEX] = -30;
	else if (button1.state == GPIO_PIN_RESET)
	    speed[FRONT_INDEX] = 0;

	if(button2.state == GPIO_PIN_SET && button3.state == GPIO_PIN_RESET)
	    speed[BACK_INDEX] = 30;
	else if(button2.state == GPIO_PIN_SET && button3.state == GPIO_PIN_SET)
	    speed[BACK_INDEX] = -30;
	else if (button2.state == GPIO_PIN_RESET)
	    speed[BACK_INDEX] = 0;

	runMotor(&rearMotor, speed[FRONT_INDEX]);
//	runMotor(&backMotor, speed[BACK_INDEX]);

	if (climb_first_iteration){
	    initial_angle = MPU6050.KalmanAngleX;
	    climb_first_iteration = false;
	}


	// Check if need to compute PID
	if (pid_need_compute(balance_pid) && climb_first_iteration == false && fabs(initial_angle - MPU6050.KalmanAngleX) > 2.0){
		// Read process feedback
		backClimb_input = (MPU6050.KalmanAngleX - initial_angle) * 5;
		// Compute new PID output value
		pid_compute(balance_pid);
		//Change actuator value
		speed[BACK_INDEX] = backClimb_output;
	}


//	if (rearLS1.state == GPIO_PIN_RESET || rearLS2.state == GPIO_PIN_RESET){
//	    front_touchdown = 0;
//	}
//	else if (rearLS1.state == GPIO_PIN_SET || rearLS2.state == GPIO_PIN_SET){
//	    front_touchdown = 1;
//	}
//
//	if (backLS1.state == GPIO_PIN_RESET || backLS2.state == GPIO_PIN_RESET){
//	    back_touchdown = 0;
//	}
//	else if (backLS1.state == GPIO_PIN_SET || backLS2.state == GPIO_PIN_SET){
//	    back_touchdown = 1;
//	}
//
//	//Climbing phase start



	prev_time = HAL_GetTick();

    }
//	HAL_Delay(10);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin){
    case AD_BUSY_Pin:{
      if (HAL_GetTick()-prev_adc_time > 1){
	  ADC_Read(&adc_rawData[0]);
	  tempJoyRawDataX = adc_rawData[0];
	  tempJoyRawDataY = adc_rawData[1];
	  prev_adc_time = HAL_GetTick();
      }
    }

      break;
    default:
      break;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  //Checksum, make sure that response is correct
  uint16_t sum = (uint16_t)receive_buf[0] + (uint16_t)receive_buf[1] + (uint16_t)receive_buf[2] + (uint16_t)receive_buf[3]
  		+ (uint16_t)receive_buf[4] + (uint16_t)receive_buf[5] + (uint16_t)receive_buf[6]
  		+ (uint16_t)receive_buf[7] + (uint16_t)receive_buf[8] + (uint16_t)receive_buf[9]
  		+ (uint16_t)receive_buf[10] + (uint16_t)receive_buf[11] + (uint16_t)receive_buf[12]
  		+ (uint16_t)receive_buf[13];
  if ((uint8_t)sum == receive_buf[14]){
//      && receive_buf[4] == 0x00
      if (receive_buf[0] == 0xAA && receive_buf[1] == 0xA4 ){
	  hub_encoder_1 = 	(receive_buf[9] << 24) + (receive_buf[8] << 16) +
			      (receive_buf[7] << 8)+ (receive_buf[6] );
	  hub_encoder_2 = 	(receive_buf[13] << 24) + (receive_buf[12] << 16) +
				      (receive_buf[11] << 8) + (receive_buf[10] );
	  hub_encoder_1 %= 4096;
	  hub_encoder_2 %= 4096;
      }
  }
  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
//  uint32_t uart_prev_time = HAL_GetTick();
//  while(HAL_GetTick() - uart_prev_time < 100);
//  HAL_Delay(250);
}

void baseMotorCommand(void){
  MOTOR_TIM.Instance->RIGHT_MOTOR_CHANNEL = (int)baseWheelSpeed.cur_r  + 1500;
  MOTOR_TIM.Instance->LEFT_MOTOR_CHANNEL = (int)baseWheelSpeed.cur_l + 1500;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
