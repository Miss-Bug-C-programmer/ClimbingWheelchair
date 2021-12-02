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
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "adc.h"
#include "encoder.h"
#include "button.h"
#include "mpu6050.h"
#include "bd25l.h"
#include "X2_6010S.h"
#include "wheelchair.h"
#include "PID.h"
#include "battery.h"
#include "usbd_cdc_if.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define TO_RAD(x) x * M_PI / 180
#define TO_DEG(x) x * 180 / M_PI
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
const float CLIMBING_LEG_LENGTH = 0.349; //in m, measured from the pivot to the center of hub motor
const float BASE_HEIGHT = 0.15;
const float BACK_BASE_HEIGHT = 0.15;

//ALLOWABLE is the maximum pos the climbing wheel can turn
//FRONT_CLIMBING is the pos that the base above the climbing wheel w
const uint32_t MAX_FRONT_ALLOWABLE_ENC = 3100;
const uint32_t MIN_FRONT_ALLOWABLE_ENC = 6600; //6600
const uint32_t MAX_FRONT_CLIMBING_ENC = 1950; //used for climbing up
const uint32_t MAX_BACK_ALLOWABLE_ENC = 3000;
const uint32_t MIN_BACK_ALLOWABLE_ENC = 7500;
const uint32_t MAX_BACK_CLIMBING_ENC = 1850; //used when climbing down
const uint32_t FRONT_FULL_ROTATION_ENC = 4096 * FRONT_GEAR_RATIO;
const uint32_t BACK_FULL_ROTATION_ENC = 4096 * BACK_GEAR_RATIO;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//Limit Switches & Tactile Switches
Button_TypeDef rearLS1 =
{ .gpioPort = LimitSW1_GPIO_Port, .gpioPin =
LimitSW1_Pin };
Button_TypeDef rearLS2 =
{ .gpioPort = LimitSW2_GPIO_Port, .gpioPin =
LimitSW2_Pin };
Button_TypeDef backLS1 =
{ .gpioPort = LimitSW3_GPIO_Port, .gpioPin =
LimitSW3_Pin };
Button_TypeDef backLS2 =
{ .gpioPort = LimitSW4_GPIO_Port, .gpioPin =
LimitSW4_Pin };
Button_TypeDef button1 =
{ .gpioPort = Button1_GPIO_Port, .gpioPin = Button1_Pin };
Button_TypeDef button2 =
{ .gpioPort = Button2_GPIO_Port, .gpioPin = Button2_Pin };
Button_TypeDef button3 =
{ .gpioPort = Button3_GPIO_Port, .gpioPin = Button3_Pin };

//Joystick/ADC
int16_t adc_rawData[8];
uint32_t prev_adc_time = 0;
int tempJoyRawDataX, tempJoyRawDataY;

//Wheelchair Base wheel control
WheelSpeed baseWheelSpeed =

{ .accel_loop = 100.0f, .decel_loop = 50.0f, .left_speed_step = 5,
		.right_speed_step = 5 };
const float base_linSpeedLevel[3] =
{ 200.0f, 300.0f, 400.0f };
const float base_angSpeedLevel[3] =
{ 100.0f, 150.0f, 200.0f };
int base_speedLevel = 0; //change the speed level if need higher speed

//Lifting Mode
//State
Operation_Mode lifting_mode = RETRACTION; //0 is normal operation, 1 is lifting up, 2 is lifitng down, -1 is retraction
bool front_touchdown = false; //Record the state of climbing wheel whether contact with ground
bool back_touchdown = false;

//-----------------------------------------------------------------------------------------------
//Balance Control
//-----------------------------------------------------------------------------------------------
MPU6050_t MPU6050;
double initial_angle = 0;
double exp_angle_filter = 0.8;
bool climb_first_iteration = true;
bool button_prev_state = false;
//-----------------------------------------------------------------------------------------------
//Climbing Position Control
//-----------------------------------------------------------------------------------------------
//Climbing landing motor
Motor_TypeDef rearMotor, backMotor; //declare in bd25l.c
float speed[2] =
{ 0 }; //range: 0 - 100
EncoderHandle encoderBack, encoderFront;
float prev_angle_tick = 0;
float prev_angle = 0;

//Front Climbing Position Control
struct pid_controller frontClimb_ctrl;
PID_t frontClimb_pid;
float frontClimb_input = 0, frontClimb_output = 0;
float frontClimb_setpoint = 0;
float frontClimb_kp = 0.35, frontClimb_ki = 0.003, frontClimb_kd = 0.00001;

//Back Climbing Position Control
struct pid_controller backClimb_ctrl;
PID_t backClimb_pid;
float backClimb_input = 0, backClimb_output = 0;
float backClimb_setpoint = 0;
float backClimb_kp = 0.3, backClimb_ki = 0.004, backClimb_kd = 0.00001;

float curb_height = 0; //store curb height
int front_climbDown_enc = 0; //used when climbing down stair, to leave a gap between the front wheel and ground
float back_lifting_height = 0;
float back_lifting_angle = 0;
int back_encoder_input = 0;
//-----------------------------------------------------------------------------------------------
//Climbing Forward Control
//-----------------------------------------------------------------------------------------------
//Hub Motor UART receive
uint8_t receive_buf[15];
Encoder_Feedback hub_encoder_feedback;

float climbForward_speed = 0; //rad/s`
float forward_distance = BASE_LENGTH; // (in meter) distance to travel during climbing process by hub
bool is_lifting = false;
bool finish_climbing_flag = false;
//-----------------------------------------------------------------------------------------------
//Debug Test
//-----------------------------------------------------------------------------------------------
enum Mode
{
	TEST = 0, NORMAL_DEBUG
};
enum Mode state = NORMAL_DEBUG;
int state_count = 0;

int motor_speed = 0;
//float dist = 0.386;

//Test battery
batteryHandler main_power_supply;
uint8_t battery_receive_buf[50];
extern DMA_HandleTypeDef hdma_usart2_rx;
extern UART_HandleTypeDef huart2;
char *data = "Hello World from USB CDC\n";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void baseMotorCommand(void);
bool climbingForward(float dist); //return true if in the process of moving forward
bool goto_pos(int enc, PID_t pid_t); //return true if still in the process of reaching the position
bool in_climb_process(int front_enc, int back_enc);


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
//	18.63 18.13 18.81 +19.00 17.95

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	//Initialize hardware communication
//	joystick_Init();
//	ADC_Init();
//	ADC_DataRequest();
//	ENCODER_Init();
//	BatteryInit(&main_power_supply, &huart2);

//	uint32_t state_count = HAL_GetTick();
//	while (MPU6050_Init(&hi2c1) == 1)
//	{
//		if (HAL_GetTick() - state_count > 5000)
//			Error_Handler();
//	}

	//Start base wheel PWM pin
//	wheelSpeedControl_Init(&baseWheelSpeed, base_linSpeedLevel[base_speedLevel],
//			base_angSpeedLevel[base_speedLevel]);
//	HAL_TIM_Base_Start(&MOTOR_TIM);
//	HAL_TIM_PWM_Start(&MOTOR_TIM, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&MOTOR_TIM, TIM_CHANNEL_2);
//	MOTOR_TIM.Instance->RIGHT_MOTOR_CHANNEL = 1500;
//	MOTOR_TIM.Instance->LEFT_MOTOR_CHANNEL = 1500;
//
////	//Initialize rear and back motor
//	bd25l_Init(&rearMotor);
//	bd25l_Init(&backMotor);
//	runMotor(&rearMotor, 0);
//	runMotor(&backMotor, 0);
//	HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_4);
//	emBrakeMotor(0);
////
//	//Initialize hub motor provided joystick control
//	hubMotor_Init();
//	send_HubMotor(0, 0);
//
//	//Initialize front and back climbing position controller
//	frontClimb_pid = pid_create(&frontClimb_ctrl, &frontClimb_input,
//			&frontClimb_output, &frontClimb_setpoint, frontClimb_kp,
//			frontClimb_ki, frontClimb_kd);
//	pid_limits(frontClimb_pid, -80, 80);
//	pid_sample(frontClimb_pid, 1);
//	pid_auto(frontClimb_pid);
//
//	backClimb_pid = pid_create(&backClimb_ctrl, &backClimb_input,
//			&backClimb_output, &backClimb_setpoint, backClimb_kp, backClimb_ki,
//			backClimb_kd);
//	pid_limits(backClimb_pid, -80, 80);
//	pid_sample(backClimb_pid, 1);
//	pid_auto(backClimb_pid);
//
//  /* USER CODE END 2 */
//
//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
//	uint32_t prev_time = HAL_GetTick();
//	(void)prev_time;
//	ENCODER_Get_Angle(&encoderBack);
//	ENCODER_Get_Angle(&encoderFront);
//
////	while (state_count++ < 1000)
////		MPU6050_Read_All(&hi2c1, &MPU6050);
////	initial_angle = MPU6050.KalmanAngleX;
//	state_count = 0;
//	emBrakeMotor(1);
	//Reset encoder position
//	ENCODER_Set_ZeroPosition(&encoderBack);
//	ENCODER_Set_ZeroPosition(&encoderFront);
	HAL_Delay(500);
	//debug variable
//	uint32_t debug_prev_time = HAL_GetTick();
//	uint8_t led_status = 0;
	//  float speed = 0;
//	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
//	getBatteryState(&main_power_supply);
	while (1)
	{
//		while(HAL_GPIO_ReadPin(Button1_GPIO_Port, Button1_Pin) == GPIO_PIN_RESET);
//		getBatteryState(&main_power_supply);
		CDC_Transmit_FS(data, strlen(data));
		HAL_Delay(500);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	}
//	HAL_TIM_PWM_Stop(&MOTOR_TIM, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Stop(&MOTOR_TIM, TIM_CHANNEL_2);
//	brakeMotor(&backMotor, 1);
//	brakeMotor(&backMotor, 1);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	switch (GPIO_Pin)
//	{
//	case AD_BUSY_Pin:
//	{
//		if (HAL_GetTick() - prev_adc_time > 1)
//		{
//			ADC_Read(adc_rawData);
//			tempJoyRawDataX = adc_rawData[2];
//			tempJoyRawDataY = adc_rawData[1];
//			prev_adc_time = HAL_GetTick();
//		}
//	}
//		break;
//	default:
//		break;
//	}
//}

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
//	if (huart->Instance == USART3){
//
//	}
//}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2){
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, battery_receive_buf, sizeof(battery_receive_buf));
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	}
}


//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	//Hub Encoder callback
//	if (huart->Instance == USART3)
//	{
//		//Checksum, make sure that response is correct
//		uint16_t sum = (uint16_t) receive_buf[0] + (uint16_t) receive_buf[1]
//				+ (uint16_t) receive_buf[2] + (uint16_t) receive_buf[3]
//				+ (uint16_t) receive_buf[4] + (uint16_t) receive_buf[5]
//				+ (uint16_t) receive_buf[6] + (uint16_t) receive_buf[7]
//				+ (uint16_t) receive_buf[8] + (uint16_t) receive_buf[9]
//				+ (uint16_t) receive_buf[10] + (uint16_t) receive_buf[11]
//				+ (uint16_t) receive_buf[12] + (uint16_t) receive_buf[13];
//		if ((uint8_t) sum == receive_buf[14])
//		{
//			//Encoder Feedback
//			if (receive_buf[0] == 0xAA && receive_buf[1] == 0xA4
//					&& receive_buf[3] == 0x00)
//			{
//				hub_encoder_feedback.encoder_1 = (receive_buf[9] << 24)
//						+ (receive_buf[8] << 16) + (receive_buf[7] << 8)
//						+ (receive_buf[6]);
//				hub_encoder_feedback.encoder_2 = (receive_buf[13] << 24)
//						+ (receive_buf[12] << 16) + (receive_buf[11] << 8)
//						+ (receive_buf[10]);
//			}
//		}
//	}
//
//}

//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//	//Left Encoder Callback
//	static CAN_RxHeaderTypeDef canRxHeader;
//	uint8_t incoming[8];
//	if (hcan == &hcan1)
//	{
//		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &canRxHeader, incoming);
//		if (incoming[1] == ENC_ADDR_LEFT)
//		{
//			ENCODER_Sort_Incoming(incoming, &encoderBack);
//			ENCODER_Get_Angle(&encoderBack);
//			//Process the angle and GR
//			//4096 is encoder single turn value
//			//Need to check the encoder value in the correct direction
//			encoderBack.encoder_pos = (uint32_t)((4096 * BACK_GEAR_RATIO)
//					- encoderBack.encoder_pos)
//							% (4096 * BACK_GEAR_RATIO);
//			encoderBack.angleDeg = (float) encoderBack.encoder_pos
//					/ (4096 * BACK_GEAR_RATIO) * 360 + 36.587;
//			if (encoderBack.angleDeg > 360)
//				encoderBack.angleDeg -= 360;
//			if (encoderBack.encoder_pos >= MAX_BACK_ALLOWABLE_ENC)
//				encoderBack.signed_encoder_pos = encoderBack.encoder_pos
//						- 4096 * BACK_GEAR_RATIO;
//		}
//		if (incoming[1] == ENC_ADDR_RIGHT)
//		{
//			ENCODER_Sort_Incoming(incoming, &encoderFront);
//			ENCODER_Get_Angle(&encoderFront);
//			if (4096 * 24 - encoderFront.encoder_pos < 30000)
//			{
//				encoderFront.encoder_pos =
//						(4096 * 24 - encoderFront.encoder_pos)
//								% (uint32_t) (4096 * FRONT_GEAR_RATIO);
//				encoderFront.angleDeg = (float) encoderFront.encoder_pos
//						/ (4096 * FRONT_GEAR_RATIO) * 360 + 36.587;
//			}
//			else
//			{
//				encoderFront.encoder_pos = (4096 * FRONT_GEAR_RATIO)
//						- encoderFront.encoder_pos;
//				encoderFront.angleDeg = (float) encoderFront.encoder_pos
//						/ (4096 * FRONT_GEAR_RATIO) * 360 + 36.587 - 360;
//			}
//			if (encoderFront.encoder_pos >= MAX_FRONT_ALLOWABLE_ENC)
//				encoderFront.signed_encoder_pos = encoderFront.encoder_pos
//						- 4096 * FRONT_GEAR_RATIO;
//		}
//	}
//}



void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART2)
	{
		ReadBatteryState(&main_power_supply, battery_receive_buf);
		memset(battery_receive_buf, 0, sizeof(battery_receive_buf));
		getBatteryState(&main_power_supply);
	}
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
//	HAL_TIM_PWM_Stop(&MOTOR_TIM, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Stop(&MOTOR_TIM, TIM_CHANNEL_2);
//	brakeMotor(&backMotor, 1);
//	brakeMotor(&backMotor, 1);
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
