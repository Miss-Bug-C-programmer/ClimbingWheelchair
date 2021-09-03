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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include "adc.h"
#include "encoder.h"
#include "button.h"
#include "mpu6050.h"
#include "bd25l.h"
#include "X2_6010S.h"
#include "wheelchair.h"
#include "PID.h"

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
const uint32_t MAX_FRONT_ALLOWABLE_ENC = 3000;
const uint32_t MIN_FRONT_ALLOWABLE_ENC = 8051;
const uint32_t MAX_FRONT_CLIMBING_ENC = 2200; //used for climbing up
const uint32_t MAX_BACK_ALLOWABLE_ENC = 3400;
const uint32_t MAX_BACK_CLIMBING_ENC = 2048; //used when climbing down

enum Mode {
	TEST = 0, NORMAL
};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//Limit Switches & Tactile Switches
Button_TypeDef rearLS1 = { .gpioPort = LimitSW1_GPIO_Port, .gpioPin =
LimitSW1_Pin };
Button_TypeDef rearLS2 = { .gpioPort = LimitSW2_GPIO_Port, .gpioPin =
LimitSW2_Pin };
Button_TypeDef backLS1 = { .gpioPort = LimitSW3_GPIO_Port, .gpioPin =
LimitSW3_Pin };
Button_TypeDef backLS2 = { .gpioPort = LimitSW4_GPIO_Port, .gpioPin =
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
WheelSpeed baseWheelSpeed = { .accel_loop = 100.0f, .decel_loop = 200.0f,
		.left_speed_step = 5.0, .right_speed_step = 5.0 };
const float base_linSpeedLevel[3] = { 200.0f, 300.0f, 400.0f };
const float base_angSpeedLevel[3] = { 100.0f, 150.0f, 200.0f };
int base_speedLevel = 1; //change the speed level if need higher speed

//Lifting Mode
//State
int8_t lifting_mode = 0; //0 is normal operation, 1 is lifting up, 2 is lifitng down
uint8_t retraction_mode = 0;
bool front_touchdown = false; //Record the state of climbing wheel whether contact with ground
bool back_touchdown = false;

//-----------------------------------------------------------------------------------------------
//Balance Control
//-----------------------------------------------------------------------------------------------
MPU6050_t MPU6050;
double initial_angle = 0;
double exp_angle_filter = 0.8;
bool climb_first_iteration = true;

//Front Balance control
struct pid_controller frontBalance_ctrl;
PID_t frontBalance_pid;
float frontBalance_input = 0, frontBalance_output = 0;
float frontBalance_setpoint = 0;
float frontBalance_kp = 0, frontBalance_ki = 0, frontBalance_kd = 0;

//Back Balance control
struct pid_controller backBalance_ctrl;
PID_t backBalance_pid;
float backBalance_input = 0, backBalance_output = 0;
float backBalance_setpoint = 0;
float backBalance_kp = 3, backBalance_ki = 0.0, backBalance_kd = 0.0;

//-----------------------------------------------------------------------------------------------
//Climbing Position Control
//-----------------------------------------------------------------------------------------------
//Climbing landing motor
Motor_TypeDef rearMotor, backMotor; //declare in bd25l.c
float speed[2] = { 0 }; //range: 0 - 100
EncoderHandle encoderBack, encoderFront;
float prev_angle_tick = 0;
float prev_angle = 0;

//Front Climbing Position Control
struct pid_controller frontClimb_ctrl;
PID_t frontClimb_pid;
float frontClimb_input = 0, frontClimb_output = 0;
float frontClimb_setpoint = 0;
float frontClimb_kp = 0.09, frontClimb_ki = 0.001, frontClimb_kd = 0.00008;

//Back Climbing Position Control
struct pid_controller backClimb_ctrl;
PID_t backClimb_pid;
float backClimb_input = 0, backClimb_output = 0;
float backClimb_setpoint = 0;
float backClimb_kp = 0.09, backClimb_ki = 0.001, backClimb_kd = 0.00008;

float curb_height = 0; //store curb height
float back_lifting_height = 0;
float back_lifting_angle = 0;
int back_encoder_input = 0;
//-----------------------------------------------------------------------------------------------
//Climbing Forward Control
//-----------------------------------------------------------------------------------------------
//Hub Motor UART receive
uint8_t receive_buf[15];
Encoder_Feedback hub_encoder_feedback;
Encoder_Feedback prev_hub_encoder_feedback;
bool first_encoder_callback = true;
//ClimbWheel need to be removed after fully automated
WheelSpeed climbWheelSpeed = { .accel_loop = 100.0f, .decel_loop = 200.0f,
		.left_speed_step = 5.0, .right_speed_step = 5.0 };
const float climb_linSpeedLevel[3] = { 1000.0f, 3000.0f, 4000.0f };
const float climb_angSpeedLevel[3] = { 500.0f, 1500.0f, 2000.0f };
int climb_speedLevel = 0; //change the speed level if need higher speed

float climbForward_speed = 0; //rad/s
float forward_distance = BASE_LENGTH; // (in meter) distance to travel during climbing process by hub

//-----------------------------------------------------------------------------------------------
//Debug Test
//-----------------------------------------------------------------------------------------------
enum Mode state = NORMAL;
int state_count = 0;
float dist = 0.386;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void baseMotorCommand(void);
void climbingForward(float dist);
void reinitialize(void);
void goto_pos(int enc, PID_t pid_t);
void goto_angle(float angle, PID_t pid_t);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_USART3_UART_Init();
	MX_TIM3_Init();
	MX_TIM8_Init();
	MX_CAN1_Init();
	MX_SPI1_Init();
	/* USER CODE BEGIN 2 */
	//Initialize hardware communication
//	joystick_Init();
//	ADC_Init();
//	ADC_DataRequest();
	ENCODER_Init();
//	  DWT_Init();

	uint32_t state_count = HAL_GetTick();
	while (MPU6050_Init(&hi2c1) == 1) {
		if (HAL_GetTick() - state_count > 5000)
			Error_Handler();
	}

	//Start base wheel PWM pin
	wheelSpeedControl_Init(&baseWheelSpeed, base_linSpeedLevel[base_speedLevel],
			base_angSpeedLevel[base_speedLevel]);
	HAL_TIM_Base_Start(&MOTOR_TIM);
	HAL_TIM_PWM_Start(&MOTOR_TIM, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&MOTOR_TIM, TIM_CHANNEL_2);
	MOTOR_TIM.Instance->RIGHT_MOTOR_CHANNEL = 1500;
	MOTOR_TIM.Instance->LEFT_MOTOR_CHANNEL = 1500;

//	//Initialize rear and back motor
	bd25l_Init(&rearMotor);
	bd25l_Init(&backMotor);
	runMotor(&rearMotor, 0);
	runMotor(&backMotor, 0);
	emBrakeMotor(0);
//
	//Initialize hub motor provided joystick control
	hubMotor_Init();
	send_HubMotor(0, 0);
	wheelSpeedControl_Init(&climbWheelSpeed,
			climb_linSpeedLevel[climb_speedLevel],
			climb_angSpeedLevel[climb_speedLevel]);

	//Initialize front and back balance controller
	frontBalance_pid = pid_create(&frontBalance_ctrl, &frontBalance_input,
			&frontBalance_output, &frontBalance_setpoint, frontBalance_kp,
			frontBalance_ki, frontBalance_kd);
	pid_limits(frontBalance_pid, -50, 50);
	pid_sample(frontBalance_pid, 1);
	pid_auto(frontBalance_pid);

	backBalance_pid = pid_create(&backBalance_ctrl, &backBalance_input,
			&backBalance_output, &backBalance_setpoint, backBalance_kp,
			backBalance_ki, backBalance_kd);
	pid_limits(backBalance_pid, -20, 20);
	pid_sample(backBalance_pid, 1);
	pid_auto(backBalance_pid);

	//Initialize front and back climbing position controller
	frontClimb_pid = pid_create(&frontClimb_ctrl, &frontClimb_input,
			&frontClimb_output, &frontClimb_setpoint, frontClimb_kp,
			frontClimb_ki, frontClimb_kd);
	pid_limits(frontClimb_pid, -50, 50);
	pid_sample(frontClimb_pid, 1);
	pid_auto(frontClimb_pid);

	backClimb_pid = pid_create(&backClimb_ctrl, &backClimb_input,
			&backClimb_output, &backClimb_setpoint, backClimb_kp, backClimb_ki,
			backClimb_kd);
	pid_limits(backClimb_pid, -50, 50);
	pid_sample(backClimb_pid, 1);
	pid_auto(backClimb_pid);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint32_t prev_time = HAL_GetTick();
	ENCODER_Get_Angle(&encoderBack);
	ENCODER_Get_Angle(&encoderFront);
	while (state_count++ < 1000)
		MPU6050_Read_All(&hi2c1, &MPU6050);
	initial_angle = MPU6050.KalmanAngleX;
	state_count = 0;
	emBrakeMotor(1);
	//Reset encoder position
//	ENCODER_Set_ZeroPosition(&encoderBack);
//	ENCODER_Set_ZeroPosition(&encoderFront);
	//debug variable
	uint32_t debug_prev_time = HAL_GetTick();
	uint8_t led_status = 0;
	//  float speed = 0;
	while (1) {
		//Code to debug with blinking LED
		if (HAL_GetTick() - debug_prev_time >= 1000) {
			if (led_status == 0) {
				//	      count++;
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
				led_status = 1;
			} else if (led_status == 1) {
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
				led_status = 0;
			}
			debug_prev_time = HAL_GetTick();
		}

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

		//      runMotor(&backMotor, speed++, 1);xia
//		ENCODER_Get_Angle(&encoderBack);
//		ENCODER_Get_Angle(&encoderFront);
		//Loop should execute once every 1 tick
		if (HAL_GetTick() - prev_time >= 1) {
			//	ADC_DataRequest();
			ENCODER_Read(&encoderBack);
			ENCODER_Read(&encoderFront);

			//Get kamlan filtered angle from MPU6050
			MPU6050_Read_All(&hi2c1, &MPU6050);
			GPIO_Digital_Filtered_Input(&button1, 30);
			GPIO_Digital_Filtered_Input(&button2, 30);
			GPIO_Digital_Filtered_Input(&button3, 30);

			GPIO_Digital_Filtered_Input(&rearLS1, 5);
			GPIO_Digital_Filtered_Input(&rearLS2, 5);
			GPIO_Digital_Filtered_Input(&backLS1, 5);
			GPIO_Digital_Filtered_Input(&backLS2, 5);

			//---------------------------------------------------------------------------------------------------
			//3-button control climbing mechanism
//			---------------------------------------------------------------------------------------------------
//			if (button1.state == GPIO_PIN_SET
//					&& button3.state == GPIO_PIN_RESET)
//				speed[FRONT_INDEX] = 5;
//			else if (button1.state == GPIO_PIN_SET
//					&& button3.state == GPIO_PIN_SET)
//				speed[FRONT_INDEX] = -5;
//			else if (button1.state == GPIO_PIN_RESET)
//				speed[FRONT_INDEX] = 0;
//
//			if (button2.state == GPIO_PIN_SET
//					&& button3.state == GPIO_PIN_RESET)
//				speed[BACK_INDEX] = 5;
//			else if (button2.state == GPIO_PIN_SET
//					&& button3.state == GPIO_PIN_SET)
//				speed[BACK_INDEX] = -5;
//			else if (button2.state == GPIO_PIN_RESET)
//				speed[BACK_INDEX] = 0;
//
//			runMotor(&rearMotor, speed[FRONT_INDEX]);
//			runMotor(&backMotor, speed[BACK_INDEX]);

			//Prevent wheelchair from being pulled by the hub motor during lifting up process

//			climbingForward(dist);
			//---------------------------------------------------------------------------------------------------
			//Testing Climbing Position Control
			//---------------------------------------------------------------------------------------------------
//			if (button2.state == GPIO_PIN_SET && state_count++ > 10) {
//				state_count = 0;
//				if (state == TEST) {
//					state = NORMAL;
//				} else if (state == NORMAL)
//					state = TEST;
//			}
//			if (state == TEST) {
//				goto_pos(0, backClimb_pid);
//				goto_pos(0, frontClimb_pid);
//			}
//
//			if (state == NORMAL) {
//				if (button1.state == GPIO_PIN_SET
//						&& button3.state == GPIO_PIN_RESET)
//					speed[FRONT_INDEX] = 30;
//				else if (button1.state == GPIO_PIN_SET
//						&& button3.state == GPIO_PIN_SET)
//					speed[FRONT_INDEX] = -30;
//				else if (button1.state == GPIO_PIN_RESET)
//					speed[FRONT_INDEX] = 0;
//				pid_reset(frontClimb_pid);
//				curb_height = CLIMBING_LEG_LENGTH * cos(TO_RAD(encoderFront.angleDeg)) + BASE_HEIGHT - FRONT_CLIMB_WHEEL_DIAMETER / 2.0;
//				curb_height -= 0.01;
//			}
//			runMotor(&rearMotor, speed[FRONT_INDEX]);

//			if (state == NORMAL) {
//				if (button1.state == GPIO_PIN_SET
//						&& button3.state == GPIO_PIN_RESET)
//					speed[BACK_INDEX] = 30;
//				else if (button1.state == GPIO_PIN_SET
//						&& button3.state == GPIO_PIN_SET)
//					speed[BACK_INDEX] = -30;
//				else if (button1.state == GPIO_PIN_RESET)
//					speed[BACK_INDEX] = 0;
//				pid_reset(backClimb_pid);
//			}
//			runMotor(&backMotor, speed[BACK_INDEX]);

			//---------------------------------------------------------------------------------------------------
			//Testing Climbing Balance Control
			//---------------------------------------------------------------------------------------------------
//			if (button2.state == GPIO_PIN_SET && state_count++ > 10) {
//				state_count = 0;
//				if (state == TEST) {
//					state = NORMAL;
//				} else if (state == NORMAL)
//					state = TEST;
//			}
//			if (state == TEST) {
//				if (button1.state == GPIO_PIN_SET
//						&& button3.state == GPIO_PIN_RESET)
//					speed[FRONT_INDEX] = 20;
//				else if (button1.state == GPIO_PIN_SET
//						&& button3.state == GPIO_PIN_SET)
//					speed[FRONT_INDEX] = -20;
//				else if (button1.state == GPIO_PIN_RESET)
//					speed[FRONT_INDEX] = 0;
////				runMotor(&rearMotor, speed[FRONT_INDEX]);
//				goto_angle(initial_angle, backBalance_pid);
//			}
//
//			if (state == NORMAL) {
//				if (button1.state == GPIO_PIN_SET
//						&& button3.state == GPIO_PIN_RESET)
//					speed[FRONT_INDEX] = 5;
//				else if (button1.state == GPIO_PIN_SET
//						&& button3.state == GPIO_PIN_SET)
//					speed[FRONT_INDEX] = -5;
//				else if (button1.state == GPIO_PIN_RESET)
//					speed[FRONT_INDEX] = 0;
//				pid_reset(backBalance_pid);
//
//			}

//			if (state == NORMAL) {
//				if (button1.state == GPIO_PIN_SET
//						&& button3.state == GPIO_PIN_RESET)
//					speed[BACK_INDEX] = 30;
//				else if (button1.state == GPIO_PIN_SET
//						&& button3.state == GPIO_PIN_SET)
//					speed[BACK_INDEX] = -30;
//				else if (button1.state == GPIO_PIN_RESET)
//					speed[BACK_INDEX] = 0;
//				pid_reset(backClimb_pid);
//			}
//			runMotor(&backMotor, speed[BACK_INDEX]);

			//---------------------------------------------------------------------------------------------------
			//Testing Climbing Balance Control
			//2-button control front wheel control
			//---------------------------------------------------------------------------------------------------
			//Need to put this inside the landing loop
			//if front leg depressed, climb 1st iteration false
			//	if (climb_first_iteration && speed[BACK_INDEX] != 0){
			//		initial_angle = exp_angle_filter * MPU6050.KalmanAngleX + (1-exp_angle_filter) * initial_angle ;
			//		if (GPIO_Digital_Filtered_Input(&backLS1, 5) || GPIO_Digital_Filtered_Input(&backLS2, 5))
			//			climb_first_iteration = false;
			//	}
			//
			//	//If one of the leg limit switch release, restart the climb first iteration
			//	if ((!GPIO_Digital_Filtered_Input(&backLS1, 5) || !GPIO_Digital_Filtered_Input(&backLS2, 5)) && climb_first_iteration == false){
			//		climb_first_iteration = true;
			//		initial_angle = 0;
			//	}
			//
			//	// Check if need to compute PID
			//	//Note for front control balance, angle need to reverse
			//	if (pid_need_compute(backBalance_pid) && climb_first_iteration == false && fabs(initial_angle - MPU6050.KalmanAngleX) > 1.0){
			//		// Read process feedback
			//		backBalance_input = -(MPU6050.KalmanAngleX - initial_angle);
			//		// Compute new PID output value
			//		pid_compute(backBalance_pid);
			//		//Change actuator value
			//		speed[FRONT_INDEX] = backBalance_output;
			//	}
			//
			//
			//	if (speed[FRONT_INDEX] == 0 && speed[BACK_INDEX] == 0)
			//		emBrakeMotor(0);
			//	else
			//		emBrakeMotor(1);
			//	runMotor(&rearMotor, speed[FRONT_INDEX]);
			//	runMotor(&backMotor, speed[BACK_INDEX]);
			//
			//	speed[FRONT_INDEX] = 0;
			//	speed[BACK_INDEX] = 0;

			//---------------------------------------------------------------------------------------------------
			//Final Code
			//1. Climbing wheel extension
			//2. Wheelchair lifting/dropping
			//3. Climbing wheel retraction
			//---------------------------------------------------------------------------------------------------
			//Climbing wheel start landing when button3 is pressed
			if (button3.state == 1 && front_touchdown == false
					&& back_touchdown == false && lifting_mode == 0) {
				emBrakeMotor(1);
				while (front_touchdown == false || back_touchdown == false) {
					if (GPIO_Digital_Filtered_Input(&rearLS1, 5)
							|| GPIO_Digital_Filtered_Input(&rearLS2, 5))
						front_touchdown = 1;
					if (GPIO_Digital_Filtered_Input(&backLS1, 5)
							|| GPIO_Digital_Filtered_Input(&backLS2, 5))
						back_touchdown = 1;

					//if front touch before back, climbing up process
					if (back_touchdown == 0 && front_touchdown == 1)
						lifting_mode = 1;
					//if back touch before front, climbing down process
					else if (back_touchdown == 1 && front_touchdown == 0)
						lifting_mode = 2;

					initial_angle = exp_angle_filter * MPU6050.KalmanAngleX
							+ (1 - exp_angle_filter) * initial_angle;

					if (back_touchdown == false)
						runMotor(&backMotor, 5);
					else
						runMotor(&backMotor, 0);

					if (front_touchdown == false)
						runMotor(&rearMotor, 5);
					else
						runMotor(&rearMotor, 0);
				}

				HAL_Delay(500);
				continue; //to refresh the loop and get the latest encoder reading

//				curb_height = CLIMBING_LEG_LENGTH
//						* cos(TO_RAD(encoderFront.angleDeg)) + BASE_HEIGHT
//						- FRONT_CLIMB_WHEEL_DIAMETER / 2.0;
//				curb_height -= 0.01; //Small error correction
//				back_lifting_height = BACK_BASE_HEIGHT + curb_height
//						- HUB_DIAMETER / 2;
//				back_lifting_angle = TO_DEG(
//						(float )acos(-back_lifting_height / 0.34));
//				back_encoder_input = TO_DEG(back_lifting_angle) / 360
//						* (4096 * BACK_GEAR_RATIO);

			}

			//Normal wheelchair mode, basic joystick control mode
			if (lifting_mode == 0) {
//				wheel_Control(&baseWheelSpeed);
//				baseMotorCommand();
				goto_pos(0, frontClimb_pid);
				goto_pos(0, backClimb_pid);
				front_touchdown = false;
				back_touchdown = false;
				climb_first_iteration = true;

			}

//			//Climbing up process
			if (lifting_mode == 1) {
				if (climb_first_iteration) {
					//If curb_height is positive, should be climbing up process and vice versa
					curb_height = CLIMBING_LEG_LENGTH
							* cos(TO_RAD(encoderFront.angleDeg)) + BASE_HEIGHT
							- FRONT_CLIMB_WHEEL_DIAMETER / 2.0;
					//curb_height -= 0.01; //Small error correction

					//First determine whether is the height climb-able
					back_lifting_height = BACK_BASE_HEIGHT + curb_height
							- HUB_DIAMETER / 2;
					back_lifting_angle = TO_DEG(
							(float )acos(-back_lifting_height / 0.34));
					back_encoder_input = back_lifting_angle / 360
							* (4096 * BACK_GEAR_RATIO);

					if (isnan(back_lifting_angle)
							|| back_encoder_input >= MAX_BACK_ALLOWABLE_ENC) {
						lifting_mode = 0;
						continue;
					}

					climb_first_iteration = false;
				}

				//1. lift the front climbing wheel up until it reach it maximum pos
				//The process is controlled by PID on the front climbing wheel
				//the maximum pos is when the climbing wheel is below the wheelchair base
				goto_pos(MAX_FRONT_CLIMBING_ENC, frontClimb_pid);

				//2. In the meanwhile, use another PID to make sure the wheelchair is balance
				//By controlling the back wheel
//				if (pid_need_compute(backBalance_pid) && fabs(initial_angle - MPU6050.KalmanAngleX) > 1.0 && fabs(MAX_BACK_ALLOWABLE_ENC - encoderFront.encoder_pos) > 10){
//					// Read process feedback
//					backBalance_input = (MPU6050.KalmanAngleX - initial_angle);
//					// Compute new PID output value
//					pid_compute(backBalance_pid);
//					//Change actuator value
//					speed[BACK_INDEX] = backBalance_output;
//				}
//				else speed[BACK_INDEX] = 0;
				goto_pos(back_encoder_input, backClimb_pid);

				if (fabs(speed[FRONT_INDEX]) < 4)
					speed[FRONT_INDEX] = 0;
				if (fabs(speed[BACK_INDEX]) < 4)
					speed[BACK_INDEX] = 0;

				runMotor(&rearMotor, speed[FRONT_INDEX]);
				runMotor(&backMotor, speed[BACK_INDEX]);

				//3. During lifting, due to fixed point at the back climbing wheel.
				//The wheelchair would be pulled back if the back wheel not traveling while the its lifting
				//Therefore, lifting of back wheel and hub motor need to work at the same time to make sure the wheelchair is not moving back.
				//Pull back of wheelchair would cause the front climbing wheel to slip from the curb
				if (speed[BACK_INDEX] != 0
						&& GPIO_Digital_Filtered_Input(&backLS1, 5)
						&& GPIO_Digital_Filtered_Input(&backLS2, 5)) {
					double dt = (HAL_GetTick() - prev_angle_tick)
							/ (float) FREQUENCY;
					climbForward_speed = CLIMBING_LEG_LENGTH
							* (sin(TO_RAD(prev_angle))
									- sin(TO_RAD(encoderBack.angleDeg))) / dt; //unit: m/s,
					climbForward_speed = climbForward_speed
							/ (HUB_DIAMETER / 2);
					//Convert hub speed into pulse/second
					send_HubMotor(climbForward_speed, climbForward_speed);
					prev_angle = encoderBack.angleDeg;
					prev_angle_tick = HAL_GetTick();
				} else
					send_HubMotor(0, 0);
			}
			//
			//	else if (lifting_mode == 2){
			//		//Climbing down process
			//		//1. Back lift until the wheel is below the base
			//		if(button2.state == GPIO_PIN_SET && button3.state == GPIO_PIN_RESET)
			//		    speed[BACK_INDEX] = 30;
			//		else if(button2.state == GPIO_PIN_SET && button3.state == GPIO_PIN_SET)
			//		    speed[BACK_INDEX] = -30;
			//		else if (button2.state == GPIO_PIN_RESET)
			//		    speed[BACK_INDEX] = 0;
			//
			//		//2. In the mean while, the back wheel will balance the robot
			//		// Check if need to compute PID
			//		if (pid_need_compute(backBalance_pid) && fabs(initial_angle - MPU6050.KalmanAngleX) > 1.0){
			//			// Read process feedback
			//			backBalance_input = (MPU6050.KalmanAngleX - initial_angle);
			//			// Compute new PID output value
			//			pid_compute(backBalance_pid);
			//			//Change actuator value
			//			speed[FRONT_INDEX] = backBalance_output;
			//		}
			//		else
			//			speed[FRONT_INDEX] = 0;
			//
			//		//Need a safety check before move forward, can be done using encoder
			//		//3. Then move forward
			//		wheel_Control(&climbWheelSpeed);
			////		if (climbWheelSpeed.cur_l >500 || climbWheelSpeed.cur_r >500){
			//////			climbingForward(forward_distance);
			////			retraction_mode = 1;
			////			lifting_mode = -1;
			////		}
			//
			//		if (button1.state == GPIO_PIN_SET && button3.state == GPIO_PIN_RESET){
			//					lifting_mode = -1;
			//					retraction_mode = 1;
			//				}
			//
			//		//4. Retract both to initial pos
			//	}
			//
			//	if (retraction_mode == 1){
			//		//retraction process
			//		//---------------------------------------------------------------------------------------------------
			//		//3-button control climbing mechanism
			//		//---------------------------------------------------------------------------------------------------
			//		if (button1.state == GPIO_PIN_SET && button3.state == GPIO_PIN_RESET)
			//			speed[FRONT_INDEX] = -30;
			//		else if (button1.state == GPIO_PIN_RESET)
			//			speed[FRONT_INDEX] = 0;
			//
			//		if(button2.state == GPIO_PIN_SET && button3.state == GPIO_PIN_RESET)
			//			speed[BACK_INDEX] = -30;
			//		else if (button2.state == GPIO_PIN_RESET)
			//			speed[BACK_INDEX] = 0;
			//
			//		if (button3.state == GPIO_PIN_SET)
			//			reinitialize();
			// back lifted: 2336
			//
			//	}

//			if ((speed[FRONT_INDEX] <= 5 || speed[FRONT_INDEX] >= -5) && (speed[BACK_INDEX] <= 5 ||speed[BACK_INDEX] >= -5 )
//

			runMotor(&rearMotor, speed[FRONT_INDEX]);
			runMotor(&backMotor, speed[BACK_INDEX]);

			if (speed[FRONT_INDEX] == 0 && speed[BACK_INDEX] == 0)
				emBrakeMotor(0);
			else

				//store prev_angle for climbing Up mechanism
//				prev_angle = encoderFront.angleDeg;
//				prev_angle_tick = HAL_GetTick();

//				wheel_Control(&climbWheelSpeed);
//				send_HubMotor(climbWheelSpeed.cur_l, climbWheelSpeed.cur_r);
//				climbingForward();
//				float speed = 6.0/60.0;
//				send_HubMotor(speed, speed);

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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	switch (GPIO_Pin) {
	case AD_BUSY_Pin: {
		if (HAL_GetTick() - prev_adc_time > 1) {
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	//Hub Encoder callback
	if (huart->Instance == USART3) {
		//Checksum, make sure that response is correct
		uint16_t sum = (uint16_t) receive_buf[0] + (uint16_t) receive_buf[1]
				+ (uint16_t) receive_buf[2] + (uint16_t) receive_buf[3]
				+ (uint16_t) receive_buf[4] + (uint16_t) receive_buf[5]
				+ (uint16_t) receive_buf[6] + (uint16_t) receive_buf[7]
				+ (uint16_t) receive_buf[8] + (uint16_t) receive_buf[9]
				+ (uint16_t) receive_buf[10] + (uint16_t) receive_buf[11]
				+ (uint16_t) receive_buf[12] + (uint16_t) receive_buf[13];
		if ((uint8_t) sum == receive_buf[14]) {
			//Encoder Feedback
			if (receive_buf[0] == 0xAA && receive_buf[1] == 0xA4
					&& receive_buf[3] == 0x00) {
				hub_encoder_feedback.encoder_1 = (receive_buf[9] << 24)
						+ (receive_buf[8] << 16) + (receive_buf[7] << 8)
						+ (receive_buf[6]);
				hub_encoder_feedback.encoder_2 = (receive_buf[13] << 24)
						+ (receive_buf[12] << 16) + (receive_buf[11] << 8)
						+ (receive_buf[10]);
				if (first_encoder_callback) {
					prev_hub_encoder_feedback.encoder_1 =
							hub_encoder_feedback.encoder_1;
					prev_hub_encoder_feedback.encoder_2 =
							hub_encoder_feedback.encoder_2;
					first_encoder_callback = false;
				}
			}
		}
	}

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	//Left Encoder Callback
	static CAN_RxHeaderTypeDef canRxHeader;
	uint8_t incoming[8];
	if (hcan == &hcan1) {
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &canRxHeader, incoming);
		if (incoming[1] == ENC_ADDR_LEFT) {
			ENCODER_Sort_Incoming(incoming, &encoderBack);
			ENCODER_Get_Angle(&encoderBack);
			//Process the angle and GR
			//4096 is encoder single turn value
			//Need to check the encoder value in the correct direction
			encoderBack.encoder_pos = (uint32_t) (4096 * BACK_GEAR_RATIO)
					- encoderBack.encoder_pos
							% (uint32_t) (4096 * BACK_GEAR_RATIO);
			encoderBack.angleDeg = (float) encoderBack.encoder_pos
					/ (4096 * BACK_GEAR_RATIO) * 360 + 36.587;
			if (encoderBack.angleDeg > 360)
				encoderBack.angleDeg -= 360;
		}
		if (incoming[1] == ENC_ADDR_RIGHT) {
			ENCODER_Sort_Incoming(incoming, &encoderFront);
			ENCODER_Get_Angle(&encoderFront);
			if (4096 * 24 - encoderFront.encoder_pos < 30000) {
				encoderFront.encoder_pos =
						(4096 * 24 - encoderFront.encoder_pos)
								% (uint32_t) (4096 * FRONT_GEAR_RATIO);
				encoderFront.angleDeg = (float) encoderFront.encoder_pos
						/ (4096 * FRONT_GEAR_RATIO) * 360 + 36.587;
			} else {
				encoderFront.encoder_pos = (4096 * FRONT_GEAR_RATIO)
						- encoderFront.encoder_pos;
				encoderFront.angleDeg = (float) encoderFront.encoder_pos
						/ (4096 * FRONT_GEAR_RATIO) * 360 + 36.587 - 360;
			}
		}
	}
}

void baseMotorCommand(void) {
	MOTOR_TIM.Instance->RIGHT_MOTOR_CHANNEL = (int) baseWheelSpeed.cur_r + 1500;
	MOTOR_TIM.Instance->LEFT_MOTOR_CHANNEL = (int) baseWheelSpeed.cur_l + 1500;
}

//Move forward during climbing process
void climbingForward(float dist) {
	static int prev_tick = 0;
	static int32_t prev_enc;
	static bool first_loop = true;
	static float dist_remaining;

	float rps = (dist >= 0) ? 0.5 : -0.5; //rad/s

	if (first_loop) {
		prev_enc = hub_encoder_feedback.encoder_2;
		prev_tick = HAL_GetTick();
		first_loop = false;
		dist_remaining = dist;
	}
	if (dist / dist_remaining >= 0 && first_loop == false) {
		send_HubMotor(rps, rps);
		if (HAL_GetTick() - prev_tick > 1) {
			float dt = (float) (HAL_GetTick() - prev_tick) / FREQUENCY;
			float rad_per_s = ((float) (hub_encoder_feedback.encoder_2
					- prev_enc) / dt) * 2 * M_PI / 4096;
			dist_remaining -= (HUB_DIAMETER * rad_per_s * dt) / 2;
			prev_tick = HAL_GetTick();
			prev_enc = hub_encoder_feedback.encoder_2;
		}
	} else {
		first_loop = 1;
		send_HubMotor(0, 0);
	}

}

void reinitialize(void) {
	front_touchdown = false;
	back_touchdown = false;
	lifting_mode = 0;
	retraction_mode = 0;
	forward_distance = BASE_LENGTH;
}

void goto_pos(int enc, PID_t pid_t) {
//	&& encoderFront.encoder_pos >= MIN_FRONT_ALLOWABLE_ENC 	&& cur_enc_pos <= MAX_FRONT_ALLOWABLE_ENC
	int cur_enc_pos;

	if (pid_t == frontClimb_pid) {
		cur_enc_pos = (int) encoderFront.encoder_pos;
		if (pid_need_compute(frontClimb_pid) && fabs(enc - cur_enc_pos) > 10) {
			// Read process feedback
			if (cur_enc_pos > MAX_FRONT_ALLOWABLE_ENC)
				cur_enc_pos -= 4096 * FRONT_GEAR_RATIO;
			if (enc >= MAX_FRONT_ALLOWABLE_ENC)
				enc -= 4096 * FRONT_GEAR_RATIO;
			frontClimb_setpoint = enc;
			frontClimb_input = cur_enc_pos;
			// Compute new PID output value
			pid_compute(frontClimb_pid);
			//Change actuator value
			speed[FRONT_INDEX] = frontClimb_output;

		} else {
			speed[FRONT_INDEX] = 0;
		}
	} else if (pid_t == backClimb_pid) {
		cur_enc_pos = (int) encoderBack.encoder_pos;
		if (pid_need_compute(backClimb_pid) && fabs(enc - cur_enc_pos) > 10
				&& encoderBack.encoder_pos <= MAX_BACK_ALLOWABLE_ENC) {
			// Read process feedback
			if (cur_enc_pos > MAX_BACK_ALLOWABLE_ENC)
				cur_enc_pos -= 4096 * BACK_GEAR_RATIO;
			if (enc >= MAX_BACK_ALLOWABLE_ENC)
				enc -= 4096 * BACK_GEAR_RATIO;
			backClimb_setpoint = enc;
			backClimb_input = cur_enc_pos;
			// Compute new PID output value
			pid_compute(backClimb_pid);
			//Change actuator value
			speed[BACK_INDEX] = backClimb_output;
		} else {
			speed[BACK_INDEX] = 0;
		}
	}
}

void goto_angle(float angle, PID_t pid_t) {
	static float cur_angle;
	static int cur_pos;
	static uint32_t prev_speed_t;
	if (pid_t == backBalance_pid) {

		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
//		cur_angle = exp_angle_filter * (float) MPU6050.KalmanAngleX + (1-exp_angle_filter) * cur_angle;
		cur_angle = (float) MPU6050.KalmanAngleX;
//		 && fabs(angle - cur_angle) > 1.0
		if (pid_need_compute(backBalance_pid)) {
			backBalance_setpoint = angle;
			backBalance_input = cur_angle;
			pid_compute(backBalance_pid);
			speed[BACK_INDEX] = backBalance_output;
			cur_pos = (int) encoderBack.encoder_pos;
			prev_speed_t = HAL_GetTick();
//			if (speed[BACK_INDEX] < 5 && speed[BACK_INDEX] >= 0) speed[BACK_INDEX] = 5;
//			else if (speed[BACK_INDEX] > -5 && speed[BACK_INDEX] < 0) speed[BACK_INDEX] = -5;
		} else {
			if (HAL_GetTick() - prev_speed_t > 50)
				speed[BACK_INDEX] = 0;
//			goto_pos(cur_pos,backClimb_pid);
//			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
//			pid_reset(pid_t);
		}
	} else if (pid_t == frontBalance_pid) {
		cur_angle = (float) MPU6050.KalmanAngleX;
		if (pid_need_compute(frontBalance_pid)
				&& fabs(angle - cur_angle) > 1.0) {
			frontBalance_setpoint = angle;
			frontBalance_input = cur_angle;
			pid_compute(frontBalance_pid);
			speed[FRONT_INDEX] = frontBalance_output;
		} else {
			speed[FRONT_INDEX] = 0;
//			pid_reset(pid_t);
		}
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
void assert_failed(uint8_t *file, uint32_t line) {
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
