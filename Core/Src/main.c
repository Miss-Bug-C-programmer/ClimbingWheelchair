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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "encoder.h"
#include <math.h>
#include <stdio.h>
#include "mpu6050.h"
#include "pid.h"

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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#define ADC_CHANNEL hspi1
#define MOTOR_TIM htim3
#define LEFT_MOTOR_CHANNEL CCR1
#define RIGHT_MOTOR_CHANNEL CCR2

#define MAX_JOYSTICK_X		100
#define MAX_JOYSTICK_Y		100
#define ADC_EXPONENTIAL_ALPHA 	0.85
#define MAX_LIN_VEL		1//Maximum allowable speed is 1m/s
#define MAX_ANG_VEL		1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
//Joystick/ADC
int16_t adc_rawData[8];
int16_t adc_rawData_prev[2];
uint32_t prev_adc_time = 0;
static Joystick_Def joystick = {.x = 0, .y =0.0,
				.MAX_X = 26000, .MID_X = 16200, .MIN_X = 5900,
				.MAX_Y = 27000, .MID_Y = 16000, .MIN_Y = 6800};

//Encoder, Base wheel control
uint16_t encoder[2];
double velocity[2];
double setpoint_vel[2];
int16_t motor_command[2] = {0};
uint16_t e_stop = 1; //Store the status of emergency stop button
uint8_t braked = 1; //Stores the brake status of left and right motors
uint32_t brake_timer = 0;
double engage_brakes_timeout = 5; //5s

//Velocity stuff
double target_heading, curr_heading;
double angular_output = 0;

//PIDstruct and their tunings. There's one PID controller for each motor
PID_Struct left_pid, right_pid, left_ramp_pid, right_ramp_pid, left_d_ramp_pid, right_d_ramp_pid;
double p = 450.0, i = 500.0, d = 0.0, f = 370, max_i_output = 30;

//Lifting mechanism
MPU6050_t MPU6050;


int count = 0;
int count1 = 0;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void setBrakes();
void calcVelFromJoystick(Joystick_Def *joystick, double *vel_setpoint);
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
  /* USER CODE BEGIN 2 */
  //Initialize hardware communication
//  ADC_Init();
//    HAL_Delay(500);
//  ADC_DataRequest();
//  HAL_Delay(500);
////  encoder_Init();
////  DWT_Init();
//  while(MPU6050_Init(&hi2c1)==1);

//  //Start wheel pwm pin
  HAL_TIM_Base_Start(&MOTOR_TIM);
  HAL_TIM_PWM_Start(&MOTOR_TIM, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&MOTOR_TIM, TIM_CHANNEL_2);
  MOTOR_TIM.Instance->RIGHT_MOTOR_CHANNEL = 1500;
    	MOTOR_TIM.Instance->LEFT_MOTOR_CHANNEL = 1500;
//
//  //********* WHEEL PID *********//
//  double base_left_ramp_rate = 100;
//  double base_right_ramp_rate = 100;
//  double base_left_d_ramp_rate = 200;
//  double base_right_d_ramp_rate = 100;
//
//  //Setup right wheel PID
//  PID_Init(&right_pid);
//  PID_setPIDF(&right_pid, p, i, d, f);
//  PID_setMaxIOutput(&right_pid, max_i_output);
//  PID_setOutputLimits(&right_pid, -500, 500);
//  PID_setFrequency(&right_pid, 1000);
//  PID_setOutputRampRate(&right_pid, base_right_ramp_rate);
//  PID_setOutputDescentRate(&right_pid, -base_right_d_ramp_rate);
//
//  //Setup left wheel PID
//  PID_Init(&left_pid);
//  PID_setPIDF(&left_pid, p, i, d, f);
//  PID_setMaxIOutput(&left_pid, max_i_output);
//  PID_setOutputLimits(&left_pid, -500, 500);
//  PID_setFrequency(&left_pid, 1000);
//  PID_setOutputRampRate(&left_pid, base_left_ramp_rate);
//  PID_setOutputDescentRate(&left_pid, -base_left_d_ramp_rate);
//
//  //********* WHEEL ACCEL RAMP PID *********//
//  double ramp_p = 300;
//  double ramp_d = 0;
//  double ramp_i = 0;
//  double max_ramp_rate_inc = 400;
//  //Setup right wheel ramp PID
//  PID_Init(&right_ramp_pid);
//  PID_setPIDF(&right_ramp_pid, ramp_p, ramp_i, ramp_d, 0);
//  PID_setOutputLimits(&right_ramp_pid, 0, 400);
//  PID_setOutputRampRate(&right_ramp_pid, max_ramp_rate_inc);
//  PID_setOutputDescentRate(&right_ramp_pid, -max_ramp_rate_inc);
//  PID_setFrequency(&right_ramp_pid, 1000);
//
//  //Setup left wheel ramp PID
//  PID_Init(&left_ramp_pid);
//  PID_setPIDF(&left_ramp_pid, ramp_p, ramp_i, ramp_d, 0);
//  PID_setOutputLimits(&left_ramp_pid, 0, 400);
//  PID_setOutputRampRate(&left_ramp_pid, max_ramp_rate_inc);
//  PID_setOutputDescentRate(&left_ramp_pid, -max_ramp_rate_inc);
//  PID_setFrequency(&left_ramp_pid, 1000);
//
//  //********* WHEEL DECEL RAMP PID *********//
//  double d_ramp_p = 600;
//  double max_d_ramp_rate_inc = 300;
//  double max_d_increase = 350;
//  //Setup right wheel d ramp PID
//  PID_Init(&right_d_ramp_pid);
//  PID_setPIDF(&right_d_ramp_pid, d_ramp_p, 0, 0, 0);
//  PID_setOutputLimits(&right_d_ramp_pid, 0, max_d_increase);
//  PID_setOutputRampRate(&right_d_ramp_pid, max_d_ramp_rate_inc);
//  PID_setOutputDescentRate(&right_d_ramp_pid, -max_d_ramp_rate_inc);
//  PID_setFrequency(&right_d_ramp_pid, 1000);
//
//  //Setup left wheel d ramp PID
//  PID_Init(&left_d_ramp_pid);
//  PID_setPIDF(&left_d_ramp_pid, d_ramp_p, 0, 0, 0);
//  PID_setOutputLimits(&left_d_ramp_pid, 0, max_d_increase);
//  PID_setOutputRampRate(&left_d_ramp_pid, max_d_ramp_rate_inc);
//  PID_setOutputDescentRate(&left_d_ramp_pid, -max_d_ramp_rate_inc);
//  PID_setFrequency(&left_d_ramp_pid, 1000);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t prev_time = HAL_GetTick();
  while (1)
  {
      count++;
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
      HAL_Delay(500);
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
//      HAL_Delay(500);
//      ADC_DataRequest();
//      HAL_Delay(1);

    //Loop should execute once every 1 tick
//    if(HAL_GetTick() - prev_time >= 1)
//    {
//	encoderRead(encoder);
//	calcVelFromEncoder(encoder, velocity);
//	e_stop = HAL_GPIO_ReadPin(Brake_Wheel_GPIO_Port, Brake_Wheel_Pin);
//
//	//Need a way to calculate velocity
//	calcVelFromJoystick(&joystick, setpoint_vel);
//	 //Heading is synonymous to radius of curvature for given velocity pair
//	double target_angular = (setpoint_vel[RIGHT_INDEX] - setpoint_vel[LEFT_INDEX]);
//	double curr_angular = (velocity[RIGHT_INDEX] - velocity[LEFT_INDEX]);
//	double target_linear = (setpoint_vel[RIGHT_INDEX] + setpoint_vel[LEFT_INDEX]) / 2.0;
//	double curr_linear = (velocity[RIGHT_INDEX] + velocity[LEFT_INDEX]) / 2.0;
//	target_heading = atan2(target_linear, target_angular);
//	curr_heading = atan2(curr_linear, curr_angular);
//
//	//This case might happen when curr_heading is M_PI and target_heading is -M_PI
//	//In this case, both values should be equal signs
//	if(target_heading == M_PI || curr_heading == M_PI)
//	{
//	  curr_heading = fabs(curr_heading);
//	  target_heading = fabs(target_heading);
//	}
//
//	//When angular_output negative, right wheel is slower
//	//When angular_output positive, left wheel is slower
//	angular_output = (target_heading - curr_heading) / M_PI;
//	int sign = angular_output / fabs(angular_output);
//
//	//Sigmoid curve to make angular_output more sensitive in mid range (~0.5)
//	//~0.5 is max value that occurs when going from pure rotation to pure forward
//	angular_output = 1/(1 + exp(-15*(fabs(angular_output) - 0.35))) * sign;
//
//	//Small velocities cause large changes to heading due to noise
//	//Set difference to 0 if below threshold, no correction
//	if(fabs(velocity[LEFT_INDEX]) < 0.1 && fabs(velocity[RIGHT_INDEX]) < 0.1)
//		angular_output = 0;
//
//	//Amount of penalty to setpoint depends on how far away from the target heading
//	//Scale may increase over 100%, but does not matter as the heading approaches target heading
//	//scale will approach 100%
//	if(setpoint_vel[LEFT_INDEX] != 0.0 || setpoint_vel[RIGHT_INDEX] != 0.0)
//	{
//		setpoint_vel[LEFT_INDEX] *= (1 + angular_output);
//		setpoint_vel[RIGHT_INDEX] *= (1 - angular_output);
//	}
//
//	 //If e stop engaged, override setpoints to 0
//	if(e_stop == 1)
//	{
//		setpoint_vel[LEFT_INDEX] = 0;
//		setpoint_vel[RIGHT_INDEX] = 0;
//	}
//	//If data is old, set setpoint to 0
//	else if((HAL_GetTick() - prev_adc_time) > FREQUENCY * 0.2)
//	{
//		setpoint_vel[LEFT_INDEX] = 0;
//		setpoint_vel[RIGHT_INDEX] = 0;
//	}
//
//	//TODO:understand how the brake works
//	//Unbrake motors if there is command, brake otherwise
//	setBrakes();
//
//	//Ensure there is a commanded velocity, otherwise reset PID
//	if(fabs(setpoint_vel[LEFT_INDEX]) == 0 && fabs(velocity[LEFT_INDEX]) < 0.1)
//	{
//	  motor_command[LEFT_INDEX] = 0;
//	  PID_reset(&left_pid);
//	}
//	else if(!braked)
//	{
//	    //ACCELERATE
//	    {
//		    double new_left_ramp = base_left_ramp_rate + PID_getOutput(&left_ramp_pid, fabs(velocity[LEFT_INDEX]), fabs(setpoint_vel[LEFT_INDEX]));
//		    PID_setOutputRampRate(&left_pid, new_left_ramp);
//	    }
//
//	    //DECELERATE
//	    {
//		    double new_left_ramp = base_left_d_ramp_rate + PID_getOutput(&left_d_ramp_pid, fabs(setpoint_vel[LEFT_INDEX]), fabs(velocity[LEFT_INDEX]));
//		    PID_setOutputDescentRate(&left_pid, -new_left_ramp);
//	    }
//
//	    motor_command[LEFT_INDEX] = PID_getOutput(&left_pid, velocity[LEFT_INDEX], setpoint_vel[LEFT_INDEX]);
//	}
//	//Ensure there is a commanded velocity, otherwise reset PID
//	if(fabs(setpoint_vel[RIGHT_INDEX]) == 0 && fabs(velocity[RIGHT_INDEX]) < 0.1)
//	{
//		motor_command[RIGHT_INDEX] = 0;
//		PID_reset(&right_pid);
//	}
//
//	//Ensure there is a commanded velocity, otherwise reset PID
//	if(fabs(setpoint_vel[RIGHT_INDEX]) == 0 && fabs(velocity[RIGHT_INDEX]) < 0.1)
//	{
//		motor_command[RIGHT_INDEX] = 0;
//		PID_reset(&right_pid);
//	}
//
//	else if(!braked)
//	{
//
//		//ACCELERATE
//		{
//			double new_right_ramp = base_right_ramp_rate + PID_getOutput(&right_ramp_pid, fabs(velocity[RIGHT_INDEX]), fabs(setpoint_vel[RIGHT_INDEX]));
//			PID_setOutputRampRate(&right_pid, new_right_ramp);
//		}
//
//		//DECELERATE
//		{
//			double new_right_ramp = base_right_d_ramp_rate + PID_getOutput(&right_d_ramp_pid, fabs(setpoint_vel[RIGHT_INDEX]), fabs(velocity[RIGHT_INDEX]));
//			PID_setOutputDescentRate(&right_pid, -new_right_ramp);
//		}
//
//		motor_command[RIGHT_INDEX] = PID_getOutput(&right_pid, velocity[RIGHT_INDEX], setpoint_vel[RIGHT_INDEX]);
//	}
//	//Send PID commands to motor
//	MOTOR_TIM.Instance->RIGHT_MOTOR_CHANNEL = motor_command[LEFT_INDEX] + 1500;
//	MOTOR_TIM.Instance->LEFT_MOTOR_CHANNEL = motor_command[RIGHT_INDEX] + 1500;
//
//    }
//
//    prev_time = HAL_GetTick();
//
//      if(HAL_GetTick() - prev_time >= 1){
//	  MPU6050_Read_All(&hi2c1, &MPU6050);
//	  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
//	  ADC_DataRequest();
////	 HAL_Delay(500);
////	  HAL_Delay (50);
//      }
//      prev_time = HAL_GetTick();
//      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  	//Send speed commands to motor
  	MOTOR_TIM.Instance->RIGHT_MOTOR_CHANNEL += 50;
  	MOTOR_TIM.Instance->LEFT_MOTOR_CHANNEL += 50;
  	HAL_Delay(500);
//  	MOTOR_TIM.Instance->RIGHT_MOTOR_CHANNEL = 1000;
//	MOTOR_TIM.Instance->LEFT_MOTOR_CHANNEL = 1000;
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 180;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 180;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65536-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 180-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, AD_RANGE_Pin|AD_OS2_Pin|AD_OS1_Pin|AD_OS0_Pin
                          |AD_RST_Pin|AD_CV_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AD_SPI1_CS_GPIO_Port, AD_SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ClimbM_IO_FR2_Pin|ClimbM_IO_EN2_Pin|LED1_Pin|LED2_Pin
                          |Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, ClimbM_IO_BRK2_Pin|ClimbM_IO_FR1_Pin|ClimbM_IO_EN1_Pin|ClimbM_IO_BRK1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, CUI_SPI2_CS1_Pin|CUI_SPI2_CS2_Pin|HubM_IO_ALM_Pin|HubM_IO_SON_Pin
                          |HubM_IO_NOT_Pin|HubM_IO_POT_Pin|Brake_Wheel_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Button3_Pin LimitSW1_Pin LimitSW2_Pin LimitSW3_Pin
                           LimitSW4_Pin ClimbM_IO_ALM2_Pin ClimbM_IO_ALM1_Pin Button1_Pin
                           Button2_Pin */
  GPIO_InitStruct.Pin = Button3_Pin|LimitSW1_Pin|LimitSW2_Pin|LimitSW3_Pin
                          |LimitSW4_Pin|ClimbM_IO_ALM2_Pin|ClimbM_IO_ALM1_Pin|Button1_Pin
                          |Button2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AD_RANGE_Pin AD_OS2_Pin AD_OS1_Pin AD_OS0_Pin
                           AD_RST_Pin AD_CV_Pin */
  GPIO_InitStruct.Pin = AD_RANGE_Pin|AD_OS2_Pin|AD_OS1_Pin|AD_OS0_Pin
                          |AD_RST_Pin|AD_CV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : AD_SPI1_CS_Pin */
  GPIO_InitStruct.Pin = AD_SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AD_SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AD_BUSY_Pin */
  GPIO_InitStruct.Pin = AD_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(AD_BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ClimbM_IO_FR2_Pin ClimbM_IO_EN2_Pin LED1_Pin LED2_Pin
                           Buzzer_Pin */
  GPIO_InitStruct.Pin = ClimbM_IO_FR2_Pin|ClimbM_IO_EN2_Pin|LED1_Pin|LED2_Pin
                          |Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ClimbM_IO_BRK2_Pin ClimbM_IO_FR1_Pin ClimbM_IO_EN1_Pin ClimbM_IO_BRK1_Pin */
  GPIO_InitStruct.Pin = ClimbM_IO_BRK2_Pin|ClimbM_IO_FR1_Pin|ClimbM_IO_EN1_Pin|ClimbM_IO_BRK1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CUI_SPI2_CS1_Pin CUI_SPI2_CS2_Pin HubM_IO_ALM_Pin HubM_IO_SON_Pin
                           HubM_IO_NOT_Pin HubM_IO_POT_Pin Brake_Wheel_Pin */
  GPIO_InitStruct.Pin = CUI_SPI2_CS1_Pin|CUI_SPI2_CS2_Pin|HubM_IO_ALM_Pin|HubM_IO_SON_Pin
                          |HubM_IO_NOT_Pin|HubM_IO_POT_Pin|Brake_Wheel_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin){
    case AD_BUSY_Pin:{

      ADC_Read(&adc_rawData[0]);
//      prev_adc_time = HAL_GetTick();
//      //TODO: Process Joystick Data
//      //Exponential filter on adc raw data
      adc_rawData[0] = adc_rawData[0] * (1 - ADC_EXPONENTIAL_ALPHA) + adc_rawData_prev[0] * ADC_EXPONENTIAL_ALPHA;
      adc_rawData[1] = adc_rawData[1] * (1 - ADC_EXPONENTIAL_ALPHA) + adc_rawData_prev[1] * ADC_EXPONENTIAL_ALPHA;
      adc_rawData_prev[0] = adc_rawData[0];
      adc_rawData_prev[1] = adc_rawData[1];
//
//      //normalise joystick reading between 0 and 1
      joystick.x = (double) adc_rawData[0] / MAX_JOYSTICK_X;
      joystick.y = (double) adc_rawData[1] / MAX_JOYSTICK_Y;
      joystick.x = MAX(-1,MIN(joystick.x, 1));
      joystick.y = MAX(-1,MIN(joystick.y, 1));
      count1++;

    }

      break;
    default:
      break;
  }

}

void setBrakes()
{
  if((setpoint_vel[LEFT_INDEX] != 0 || setpoint_vel[RIGHT_INDEX] != 0))
  {
    //BRAKE_TIM.Instance->BRAKE_CHANNEL = 2000;
    braked = 0;
    brake_timer = 0;
  }

  else if(setpoint_vel[LEFT_INDEX] == 0 && setpoint_vel[RIGHT_INDEX] == 0)
  {
    if(fabs(velocity[LEFT_INDEX]) < 0.05 && fabs(velocity[RIGHT_INDEX]) < 0.05)
    {
      //Start timer before braking
      if(brake_timer == 0)
	brake_timer = HAL_GetTick();

      else if(HAL_GetTick() - brake_timer > engage_brakes_timeout * FREQUENCY)
      {
	//BRAKE_TIM.Instance->BRAKE_CHANNEL = 1000;
	braked = 1;
	brake_timer = 0;
      }
    }
  }
}

void calcVelFromJoystick(Joystick_Def *joystick, double *vel_setpoint){
  double lin_vel = joystick->y * MAX_LIN_VEL;
  double ang_vel = joystick->x * MAX_ANG_VEL;

  vel_setpoint[LEFT_INDEX] = (2 * lin_vel - ang_vel * BASE_WIDTH) / (2 * WHEEL_DIA);
  vel_setpoint[RIGHT_INDEX] = (2 * lin_vel + ang_vel * BASE_WIDTH) / (2 * WHEEL_DIA);
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
