/*
 * Sabertooth.c
 *
 *  Created on: Dec 3, 2021
 *      Author: ray
 */

#include "Sabertooth.h"
#include "string.h"

#define MAX(x,y) 	(((x) > (y)) ? (x) : (y))
#define MIN(x,y) 	(((x) < (y)) ? (x) : (y))

//Sabertooth Basic Command Number
#define SABERTOOTH_SET			0x28	/*!< Set a value on the motor driver >*/
#define SABERTOOTH_GET			0x29	/*!< Get a value on the motor driver >*/
#define SABERTOOTH_REPLY		0x49	/*!< Reply from motor driver >*/

//SET Command Value
#define SET_VALUE				0x00	/*!< Set the value >*/
#define SET_KEEP_ALIVE			0x10	/*!< A keep-alive will reset the serial timeout without taking any action. >*/
#define SET_SHUTDOWN			0x20	/*!< Set the value >*/
#define SET_TIMEOUT				0x30	/*!< Set the value >*/

//GET Command Value
#define GET_DUTY_CYCLE			0x00	/*!< Get the value >*/
#define GET_BATTERY				0x10	/*!< Get the battery voltage, in tenth >*/
#define GET_CURRENT				0x20	/*!< Get the current, in amps >*/
#define GET_TEMP				0x30	/*!< Get the temp, in celcius >*/

//Desired output type from the motor driver
#define TYPE_MOTOR				(uint8_t)'M'
#define TYPE_POWER				(uint8_t)'P'
#define TYPE_RAMP				(uint8_t)'E'
#define TYPE_AUXILIARY			(uint8_t)'Q'

//Targeted output number associated with output type
//#define TARGET_1				2
//#define TARGET_2				1
//#define TARGET_BOTH				(uint8_t)'*'

//Use to indicate motor state
#define MOTOR_SHUTDOWN			1
#define MOTOR_STARTUP			0



//Send and Receive byte array index
#define IDX_ADDRESS 			0
#define IDX_COMMAND				1
#define IDX_COMMAND_VALUE		2
#define IDX_CHECKSUM_1			3
#define IDX_VALUE_LOW			4
#define IDX_VALUE_HIGH			5
#define IDX_TARGET_TYPE(x)		((x == SABERTOOTH_SET) || (x == SABERTOOTH_REPLY)) ? 6 : 4
#define IDX_TARGET_ID(x)		((x == SABERTOOTH_SET) || (x == SABERTOOTH_REPLY)) ? 7 : 5
#define IDX_CHECKSUM_2(x)		((x == SABERTOOTH_SET) || (x == SABERTOOTH_REPLY)) ? 8 : 6

//Use to indicate buffer size of sending data since different size is used for get and set
#define SEND_BUF_SIZE_GET		7
#define SEND_BUF_SIZE_SET		9


static uint8_t send_buf[9] = { 0 };

/*********************************************************************
 * @brief          	- Clamp x between min and max
 *
 * @param[x]     		- input value
 * @param[min]     		- min value allowed
 * @param[max]     		- max value allowed
 *
 * @return            - None
 *
 * @Note              -
 */
static int clamp(int x, int min, int max);

/* Writes a Packet Serial command into a buffer.
 * param[address]	The address of the Sabertooth. By default, this is 128.
 * param[command] 	The command number.
 * param[value]   	The command value.
 * param[data]    	Extra data.
 * param[length]  	The number of bytes of extra data.
 * return        	How many bytes were written. This always equals 5 + length,
 * 			unless length is 0, in which case it equals 4.
 */
static void writeSabertoothCommand(Sabertooth_Handler *st_handler, uint8_t command, uint8_t value, const uint8_t *data, uint8_t length);

/* Writes a Set command into a buffer.
 * param [st_handler]	sabertooth handler
 * param [setType]	0 to set the value, 16 to send a keep-alive,
 * 			32 to set the shutdown state, or 64 to set the serialtimeout.
 * param [targetType]  	'M' for a motor output, 'P' for a power output, etc.
 * param [targetNumber]	1 or 2, or a Simplified Serial character like '1' or '2'.
 * param [value]       	The value to set to.
 * return		None.
 */
static void writeSabertoothSetCommand(Sabertooth_Handler *st_handler, uint8_t setType, uint8_t targetType, uint8_t targetNumber, int16_t value);

/* Writes a Get command into a buffer.
 * param [st_handler]	sabertooth handler
 * param [getType]	0 to get the value, 16 to get battery,
 * 			32 to get current, or 64 to get temp.
 * param [targetType]  	'M' for a motor output, 'P' for a power output, etc.
 * param [targetNumber]	1 or 2, or a Simplified Serial character like '1' or '2'.
 * return		None.
 */
static void writeSabertoothGetCommand(Sabertooth_Handler *st_handler, uint8_t getType, uint8_t targetType, uint8_t targetNumber);

void MotorInit(Sabertooth_Handler* st_handler, uint8_t address, UART_HandleTypeDef* huart){
	st_handler->address = address;
	st_handler->huart = huart;
	st_handler->motor1.battery = 0;
	st_handler->motor1.duty_cycle = 0;
	st_handler->motor1.current = 0;
	st_handler->motor1.temp = 0;
	st_handler->motor2.battery = 0;
	st_handler->motor2.duty_cycle = 0;
	st_handler->motor2.current = 0;
	st_handler->motor2.temp = 0;
}


void MotorThrottle(Sabertooth_Handler *st_handler, uint8_t motor, int16_t power) {
	if (motor < 1 || motor > 2)
		return;
	clamp(power, SABERTOOTH_MIN_ALLOWABLE_VALUE, SABERTOOTH_MAX_ALLOWABLE_VALUE);
	uint8_t target_number = (motor == 1) ? TARGET_1 : TARGET_2;
	if(target_number == 1)
		st_handler->motor1.duty_cycle = power;
	else
		st_handler->motor2.duty_cycle = power;
	writeSabertoothSetCommand(st_handler, SET_VALUE, TYPE_MOTOR, target_number, power);
}

void MotorStop(Sabertooth_Handler *st_handler) {
	MotorThrottle(st_handler, TARGET_1, 0);
	MotorThrottle(st_handler, TARGET_2, 0);
}

void MotorShutdown(Sabertooth_Handler *st_handler) {
	writeSabertoothSetCommand(st_handler, SET_SHUTDOWN, TYPE_MOTOR, TARGET_BOTH, MOTOR_SHUTDOWN);
}

void MotorStartup(Sabertooth_Handler *st_handler) {
	writeSabertoothSetCommand(st_handler, SET_SHUTDOWN, TYPE_MOTOR, TARGET_BOTH, MOTOR_STARTUP);
}

void MotorTimeout(Sabertooth_Handler *st_handler, int16_t value) {
	clamp(value, SABERTOOTH_MIN_ALLOWABLE_VALUE, SABERTOOTH_MAX_ALLOWABLE_VALUE);
	writeSabertoothSetCommand(st_handler, SET_VALUE, TYPE_MOTOR, TARGET_BOTH, value);
}

void MotorReadBattery(Sabertooth_Handler *st_handler) {
	writeSabertoothGetCommand(st_handler, GET_BATTERY, TYPE_MOTOR, TARGET_1);
	writeSabertoothGetCommand(st_handler, GET_BATTERY, TYPE_MOTOR, TARGET_2);
}

void MotorReadCurrent(Sabertooth_Handler *st_handler, uint8_t motor) {
	if (motor < 1 || motor > 2)
		return;
	uint8_t target_number = (motor == 1) ? TARGET_1 : TARGET_2;
	writeSabertoothGetCommand(st_handler, GET_CURRENT, TYPE_MOTOR, target_number);
}

void MotorReadTemperature(Sabertooth_Handler *st_handler, uint8_t motor) {
	if (motor < 1 || motor > 2)
		return;
	uint8_t target_number = (motor == 1) ? TARGET_1 : TARGET_2;
	writeSabertoothGetCommand(st_handler, GET_TEMP, TYPE_MOTOR, target_number);
}

void MotorReadDutyCycle(Sabertooth_Handler *st_handler, uint8_t motor) {
	if (motor < 1 || motor > 2)
		return;
	uint8_t target_number = (motor == 1) ? TARGET_1 : TARGET_2;
	writeSabertoothGetCommand(st_handler, GET_DUTY_CYCLE, TYPE_MOTOR, target_number);
}

void MotorProcessReply(Sabertooth_Handler *st_handler, uint8_t *receive_buf, uint8_t size) {
	//make sure the byte is have the right reply number
	if (receive_buf[IDX_COMMAND] != SABERTOOTH_REPLY)
		return;
	//Checksum to make sure data receive is in the corrent form
	uint8_t dataChecksum = 0;
	dataChecksum = (receive_buf[IDX_ADDRESS] + receive_buf[IDX_COMMAND] + receive_buf[IDX_COMMAND_VALUE]) & 127;
	if (dataChecksum != receive_buf[IDX_CHECKSUM_1])
		return;
	dataChecksum = 0;
	for (int i = 4; i < size - 1; i++)
		dataChecksum += receive_buf[i];
	dataChecksum &= 127;
	if (dataChecksum != receive_buf[IDX_CHECKSUM_2(SABERTOOTH_REPLY)])
		return;

	int16_t reply_value = (receive_buf[IDX_VALUE_LOW] & 0x7F) + ((receive_buf[IDX_VALUE_HIGH] & 0x7F) << 7);

	//Create a pointer to hold the motor handler
	Sabertooth_Motor_Handler* pMotor;
	if ((receive_buf[IDX_TARGET_TYPE(SABERTOOTH_REPLY)] == TYPE_MOTOR ) && (receive_buf[IDX_TARGET_ID(SABERTOOTH_REPLY)] == TARGET_1))
		pMotor = &(st_handler->motor1);
	if ((receive_buf[IDX_TARGET_TYPE(SABERTOOTH_REPLY)] == TYPE_MOTOR ) && (receive_buf[IDX_TARGET_ID(SABERTOOTH_REPLY)] == TARGET_2))
		pMotor = &(st_handler->motor2);

	//Check reply from which command
	switch (receive_buf[IDX_COMMAND_VALUE]) {
		case GET_BATTERY:
		case GET_BATTERY + 1:
			pMotor->battery = (receive_buf[IDX_COMMAND_VALUE] == GET_BATTERY) ? reply_value : -reply_value;
			break;
		case GET_CURRENT:
		case GET_CURRENT + 1:
			pMotor->current = (receive_buf[IDX_COMMAND_VALUE] == GET_CURRENT) ? reply_value : -reply_value;
			break;
		case GET_TEMP:
		case GET_TEMP + 1:
			pMotor->temp = (receive_buf[IDX_COMMAND_VALUE] == GET_TEMP) ? reply_value : -reply_value;
			break;
		case GET_DUTY_CYCLE:
		case GET_DUTY_CYCLE + 1:
			pMotor->temp = (receive_buf[IDX_COMMAND_VALUE] == GET_TEMP) ? reply_value : -reply_value;
			break;
		default:
			return;
		}
}


static int clamp(int x, int min, int max) {
	return MIN(MAX(min, x), max);
}

static void writeSabertoothCommand(Sabertooth_Handler *st_handler, uint8_t command, uint8_t value, const uint8_t *data, uint8_t length) {
	uint8_t i;
	uint8_t dataChecksum;
	send_buf[IDX_ADDRESS] = st_handler->address;
	send_buf[IDX_COMMAND] = command;
	send_buf[IDX_COMMAND_VALUE] = value;
	send_buf[IDX_CHECKSUM_1] = (st_handler->address + command + value) & 127;
	if (length == 0) {
		return;
	}

	dataChecksum = 0;
	for (i = 0; i < length; i++) {
		send_buf[4 + i] = data[i];
		dataChecksum += data[i];
	}
	send_buf[IDX_CHECKSUM_2(command)] = dataChecksum & 127;
	if (command == SABERTOOTH_SET) {
		HAL_UART_Transmit(st_handler->huart, send_buf, SEND_BUF_SIZE_SET,1);
	} else if (command == SABERTOOTH_GET) {
		HAL_UART_Transmit(st_handler->huart, send_buf, SEND_BUF_SIZE_GET,1);
		uint32_t t = HAL_GetTick();
		while(HAL_UART_Receive_DMA(st_handler->huart, motor_receive_buf, 9) != HAL_OK
				|| HAL_GetTick() - t > 5000);

	}
}

static void writeSabertoothSetCommand(Sabertooth_Handler *st_handler, uint8_t setType, uint8_t targetType, uint8_t targetNumber, int16_t value) {
	uint8_t data[4];
	data[2] = targetType;
	data[3] = targetNumber;
	if (value < 0) {
		value = -value;
		data[0] = (uint8_t) ((value >> 0) & 127);
		data[1] = (uint8_t) ((value >> 7) & 127);
		return writeSabertoothCommand(st_handler, SABERTOOTH_SET, setType + 1, data, sizeof(data));
	} else {
		data[0] = (uint8_t) ((value >> 0) & 127);
		data[1] = (uint8_t) ((value >> 7) & 127);
		return writeSabertoothCommand(st_handler, SABERTOOTH_SET, setType, data, sizeof(data));
	}
}

static void writeSabertoothGetCommand(Sabertooth_Handler *st_handler, uint8_t getType, uint8_t targetType, uint8_t targetNumber) {
	uint8_t data[2];
	data[0] = targetType;
	data[1] = targetNumber;
	return writeSabertoothCommand(st_handler, SABERTOOTH_GET, getType, data, sizeof(data));
}
