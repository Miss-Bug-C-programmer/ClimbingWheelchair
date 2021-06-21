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
#include "encoder.h"
#include "button.h"
#include "mpu6050.h"
#include "pid.h"
#include "bd25l.h"
#include "X2_6010S.h"

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

#define ADC_EXPONENTIAL_ALPHA 	0.85
#define ADC_TOLERANCE		1500 //Joystick value tolerance
#define MAX_LIN_VEL		1//Maximum allowable speed is 1m/s
#define MAX_ANG_VEL		1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//Joystick/ADC
int16_t adc_rawData[8];
int16_t adc_rawData_prev[2];
uint32_t prev_adc_time = 0;
static Joystick_Def joystick = {.x = 0, .y =0.0,
				.MAX_X = 26000, .MID_X = 16200, .MIN_X = 5900,
				.MAX_Y = 27000, .MID_Y = 16000, .MIN_Y = 6800};


//Wheelchair Base wheel control
const int JoystickCenterX = 16610;
const int JoystickCenterY = 16520;
const int JoystickMagnitudeMax = 13000;
const int JoystickMagnitudeMin = 2500;
const int JoyPosBufferSize = 5;
int joyPosBuffer[2][5] = {0};
int joy_pos_buffer_cnt = 0;
int wheelchr_stable_cnt = 0;
const float JoyForwardAngle = 1.57;
const float JoyForwardAngleDeadzone = 0.1;
const float LinearSpeedLevel[3] = {200.0f, 300.0f, 400.0f};
const float AngularSpeedLevel[3] = {100.0f, 150.0f, 200.0f};
int speed_level = 1;
//bool start_from_stationary = true;
//WheelSpeed wheelchr_wheelspeed_cur;
//WheelSpeed wheelchr_wheelspeed_pre;



//Lifting Mechanism
//Climbing motor
extern Motor_TypeDef rearMotor, backMotor; //declare in bd25l.c
float speed[2] = {0}; //range: 0 - 100

//Hub Motor UART receive
uint8_t receive_buf[15];
int32_t hub_encoder_1 = 0;
int32_t hub_encoder_2 = 0;

//Switches
uint8_t rearLS1 = 0, rearLS2 = 0;
uint8_t backLS1 = 0, backLS2 = 0;

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  //Initialize hardware communication
  ADC_Init();
  ADC_DataRequest();
////  encoder_Init();
////  DWT_Init();
//  while(MPU6050_Init(&hi2c1)==1);
    HAL_Delay(500);

//  //Start base wheel pwm pin
  HAL_TIM_Base_Start(&MOTOR_TIM);
  HAL_TIM_PWM_Start(&MOTOR_TIM, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&MOTOR_TIM, TIM_CHANNEL_2);
  MOTOR_TIM.Instance->RIGHT_MOTOR_CHANNEL = 1500;
  MOTOR_TIM.Instance->LEFT_MOTOR_CHANNEL = 1500;
  HAL_Delay(500);

  //Initialize rear and back motor
  bd25l_Init(&rearMotor);
  bd25l_Init(&backMotor);
  runMotor(&rearMotor, 0);
  runMotor(&backMotor, 0);
  emBrakeMotor(1);

  //Initialize hub motor
  hubMotor_Init();

  //Initalize button state

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t prev_time = HAL_GetTick();

  //debug variable
  uint32_t debug_prev_time = HAL_GetTick();
  uint8_t led_status = 0;
//  float speed = 0;
  while (1)
  {
	//Code to debug with blinking LED
//      if (HAL_GetTick() - debug_prev_time >= 1000){
//	  if (led_status == 0){
//	      count++;
//	      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
//	      led_status = 1;
//	  }
//	  else if (led_status == 1){
//	      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
//	      led_status = 0;
//	  }
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

      //Read joystick value
//      ADC_DataRequest();.

      //Get kamlan filtered angle from MPU6050
//      MPU6050_Read_All(&hi2c1, &MPU6050);

//      //Test speed commands on bse motor
//      MOTOR_TIM.Instance->RIGHT_MOTOR_CHANNEL -= 50;
//      MOTOR_TIM.Instance->LEFT_MOTOR_CHANNEL -= 50;
//      runMotor(&backMotor, 100, 1);

//      runMotor(&backMotor, speed++, 1);

    //Loop should execute once every 1 tick
    if(HAL_GetTick() - prev_time >= 1)
    {
	GPIO_Digital_Filtered_Input(&button1, 30);
	GPIO_Digital_Filtered_Input(&button2, 30);
	GPIO_Digital_Filtered_Input(&button3, 30);

	if (button1.state == GPIO_PIN_SET || button3.state == GPIO_PIN_RESET)
	    speed[FRONT_INDEX] = 30;
	else if(button1.state == GPIO_PIN_SET || button3.state == GPIO_PIN_SET)
	    speed[FRONT_INDEX] = -30;
	else if (button1.state == GPIO_PIN_RESET)
	  speed[FRONT_INDEX] = 0;

	if(button2.state == GPIO_PIN_SET || button3.state == GPIO_PIN_RESET)
	    speed[BACK_INDEX] = 30;
	else if(button2.state == GPIO_PIN_SET || button3.state == GPIO_PIN_SET)
	    speed[BACK_INDEX] = -30;
	else if (button2.state == GPIO_PIN_RESET)
	    speed[FRONT_INDEX] = 0;

	runMotor(&rearMotor, speed[FRONT_INDEX]);
	runMotor(&backMotor, speed[BACK_INDEX]);


    }
    prev_time = HAL_GetTick();
//
//      if(HAL_GetTick() - prev_time >= 1){
//
//	  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
//	  ADC_DataRequest();
////	 HAL_Delay(500);
////	  HAL_Delay (50);
//      }
//      prev_time = HAL_GetTick();
//      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

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

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin){
    case AD_BUSY_Pin:{

      ADC_Read(&adc_rawData[0]);
//      //Exponential filter on adc raw data
      adc_rawData[0] = adc_rawData[0] * (1 - ADC_EXPONENTIAL_ALPHA) + adc_rawData_prev[0] * ADC_EXPONENTIAL_ALPHA;
      adc_rawData[1] = adc_rawData[1] * (1 - ADC_EXPONENTIAL_ALPHA) + adc_rawData_prev[1] * ADC_EXPONENTIAL_ALPHA;
      adc_rawData_prev[0] = adc_rawData[0];
      adc_rawData_prev[1] = adc_rawData[1];
//
      //normalise joystick reading between 0 and 1
      //MAX_X - MID_X != |MIN_X - MID_X|
      if (adc_rawData[0] >= joystick.MID_X + ADC_TOLERANCE)
	  joystick.x = (double)(adc_rawData[0] - joystick.MID_X - ADC_TOLERANCE)/(joystick.MAX_X - joystick.MID_X- ADC_TOLERANCE);
      else if (adc_rawData[0] <= joystick.MID_X - ADC_TOLERANCE)
      	  joystick.x = -(double)(adc_rawData[0] - joystick.MID_X + ADC_TOLERANCE)/(joystick.MIN_X - joystick.MID_X + ADC_TOLERANCE);

      if (adc_rawData[1] >= joystick.MID_Y + ADC_TOLERANCE)
      	  joystick.y = (double)(adc_rawData[1] - joystick.MID_Y - ADC_TOLERANCE)/(joystick.MAX_Y - joystick.MID_Y- ADC_TOLERANCE);
      else if (adc_rawData[1] <= joystick.MID_Y - ADC_TOLERANCE)
	    joystick.y = -(double)(adc_rawData[1] - joystick.MID_Y + ADC_TOLERANCE)/(joystick.MIN_Y - joystick.MID_Y + ADC_TOLERANCE);

      joystick.x = MAX(-1,MIN(joystick.x, 1));
      joystick.y = MAX(-1,MIN(joystick.y, 1));
    }

      break;
    default:
      break;
  }

}

void calcVelFromJoystick(Joystick_Def *joystick, double *vel_setpoint){
  double lin_vel = joystick->y * MAX_LIN_VEL;
  double ang_vel = joystick->x * MAX_ANG_VEL;

  vel_setpoint[LEFT_INDEX] = (2 * lin_vel - ang_vel * BASE_WIDTH) / (2 * WHEEL_DIA);
  vel_setpoint[RIGHT_INDEX] = (2 * lin_vel + ang_vel * BASE_WIDTH) / (2 * WHEEL_DIA);
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
