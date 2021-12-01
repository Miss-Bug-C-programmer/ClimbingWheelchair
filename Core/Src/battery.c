/*
 * battery.c
 *
 *  Created on: 30 Nov 2021
 *      Author: ray
 */


#include "battery.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/*
 * Send byte format
 */
#define SOI 			0xDD	/*!< Indicate start of transmission >*/
#define EOI 			0x77	/*!< Indicate end of transmission >*/
#define BATTERY_READ 	0xA5	/*!< Read battery state >*/
#define BATTERY_WRITE 	0x5A	/*!< Write battery state >*/

/*
 * Send array index
 */
#define SEND_COMMAND_IDX 	2
#define SEND_LENGTH_IDX		3


/*
 * Reply array index
 */
#define REPLY_FLAG_IDX 								2
#define REPLY_LENGTH_IDX 							3
#define REPLY_TOTAL_VOLTAGE_IDX						4
#define REPLY_CURRENT_IDX							6
#define REPLY_REMAINING_CAPACITY_IDX				8
#define REPLY_NOMINAL_CAPACITY_IDX					10
#define REPLY_CYCLES_IDX							12
#define REPLY_PRODUCTION_DATE_IDX					14
#define REPLY_BALANCE_LOW_IDX						16
#define REPLY_BALANCE_HIGH_IDX						18
#define REPLY_PROTECTION_IDX						20
#define REPLY_SOFTWARE_IDX							22
#define REPLY_REMAINING_CAPACITY_PERCENTAGE_IDX		23
#define REPLY_FET_CONTROL_IDX						24
#define REPLY_BATTERY_IDX							25
#define REPLY_NTC_IDX								26
#define REPLY_NTC_CONTENT_IDX						27

enum Command{				/*!< Indicate what reply want from hardware >*/
	BATTERY_BASIC_STATE = 0x03,
	SINGLE_BATTERY_VOLTAGE,
	BATTERY_SERIAL_NUMBER
};

static uint16_t CalculateCheckSum(uint8_t buf[], int start_pos, int end_pos);


uint8_t send_buf[7] = {0};


void BatteryInit(batteryHandler* battery_handler, UART_HandleTypeDef* huart){
	battery_handler->huart = huart;
	battery_handler->battery_info.total_voltage = 0;
	battery_handler->battery_info.current = 0;
	battery_handler->battery_info.remaining_capacity = 0;
	battery_handler->battery_info.nominal_capacity = 0;
	battery_handler->battery_info.cycles = 0;
	battery_handler->battery_info.production_date = 0;
	battery_handler->battery_info.balanced_state[0] = 0;
	battery_handler->battery_info.balanced_state[1] = 0;
	battery_handler->battery_info.protection_state = 0;
	battery_handler->battery_info.software_version = 0;
	battery_handler->battery_info.remaining_capacity_RSOC = 0;
	battery_handler->battery_info.FET_control_status = 0;
	battery_handler->battery_info.battery_number = 0;
	battery_handler->battery_info.NTC_number = 0;
	battery_handler->battery_info.NTC_content = NULL;
}

void BatteryDeInit(batteryHandler* battery_handler){
	free(battery_handler->huart);
	free(battery_handler->battery_info.NTC_content);

}

void getBatteryState(batteryHandler* battery_handler){
	send_buf[0] = SOI;
	send_buf[1] = BATTERY_READ;
	send_buf[2] = BATTERY_BASIC_STATE;
	send_buf[3] = 0x00; //Length of info desired, 0 if want to get all info
	uint16_t checksum = CalculateCheckSum(send_buf, SEND_COMMAND_IDX, SEND_LENGTH_IDX);
	send_buf[4] = (checksum >> 8) & 0xFF;
	send_buf[5] = checksum & 0xFF;
	send_buf[6] = EOI;
//	HAL_UART_Transmit(battery_handler->huart, send_buf, sizeof(send_buf),10);

	HAL_UART_Transmit_DMA(battery_handler->huart, send_buf, sizeof(send_buf));
//	uint8_t send_buf1 = 0xff;
//	HAL_UART_Transmit_DMA(battery_handler->huart, &send_buf1, 1);
//	HAL_UART_Transmit(battery_handler->huart, (uint8_t*)send_buf1, 1,10);
}

void ReadBatteryState(batteryHandler* battery_handler, uint8_t receive_buf[]){
	//Last index number in buffer
	uint8_t end_idx = REPLY_TOTAL_VOLTAGE_IDX + receive_buf[REPLY_LENGTH_IDX] + 2; //2 is account for checksum byte
	//check receive start, r/w state. flag and end buffer
	uint8_t tmp1 = (receive_buf[0] == SOI);
	uint8_t tmp2 = (receive_buf[end_idx] == EOI);
	uint8_t tmp3 = ((receive_buf[1] == BATTERY_BASIC_STATE || receive_buf[1] == BATTERY_READ));
	if (!(tmp1 && tmp2 && tmp3))
		return;

	//Checksum_end_idx need to consider the length received from reply,
	uint8_t checksum_end_idx = REPLY_TOTAL_VOLTAGE_IDX + receive_buf[REPLY_LENGTH_IDX] - 1;
	uint16_t checksum = CalculateCheckSum(receive_buf, REPLY_FLAG_IDX, checksum_end_idx);
	//Check the last second and third byte, if doesnt match the check sum, dispose the data
	if (	receive_buf[REPLY_TOTAL_VOLTAGE_IDX + receive_buf[REPLY_LENGTH_IDX] 	  ] != ((checksum >> 8) & 0xFF) ||
			receive_buf[REPLY_TOTAL_VOLTAGE_IDX + receive_buf[REPLY_LENGTH_IDX] +1 ] != (checksum & 0xFF)	)
			return;

	battery_handler->battery_info.total_voltage = (receive_buf[REPLY_TOTAL_VOLTAGE_IDX] << 8) | receive_buf[REPLY_TOTAL_VOLTAGE_IDX+1];
	battery_handler->battery_info.current = (receive_buf[REPLY_CURRENT_IDX] << 8) | receive_buf[REPLY_CURRENT_IDX+1];
	battery_handler->battery_info.remaining_capacity = (receive_buf[REPLY_REMAINING_CAPACITY_IDX] << 8) | receive_buf[REPLY_REMAINING_CAPACITY_IDX+1];
	battery_handler->battery_info.nominal_capacity = (receive_buf[REPLY_NOMINAL_CAPACITY_IDX] << 8) | receive_buf[REPLY_NOMINAL_CAPACITY_IDX+1];
	battery_handler->battery_info.cycles = (receive_buf[REPLY_CYCLES_IDX] << 8) | receive_buf[REPLY_CYCLES_IDX+1];
	battery_handler->battery_info.production_date = (receive_buf[REPLY_PRODUCTION_DATE_IDX] << 8) | receive_buf[REPLY_PRODUCTION_DATE_IDX+1];
	battery_handler->battery_info.balanced_state[0] = (receive_buf[REPLY_BALANCE_LOW_IDX] << 8) | receive_buf[REPLY_BALANCE_LOW_IDX+1];
	battery_handler->battery_info.balanced_state[1] = (receive_buf[REPLY_BALANCE_HIGH_IDX] << 8) | receive_buf[REPLY_BALANCE_HIGH_IDX+1];
	battery_handler->battery_info.protection_state = (receive_buf[REPLY_PROTECTION_IDX] << 8) | receive_buf[REPLY_PROTECTION_IDX+1];
	battery_handler->battery_info.software_version = receive_buf[REPLY_SOFTWARE_IDX];
	battery_handler->battery_info.remaining_capacity_RSOC = receive_buf[REPLY_REMAINING_CAPACITY_PERCENTAGE_IDX];
	battery_handler->battery_info.FET_control_status = receive_buf[REPLY_FET_CONTROL_IDX];
	battery_handler->battery_info.battery_number = receive_buf[REPLY_BATTERY_IDX];
	//Check if NTC number is the same
	//For first iteration
	if (battery_handler->battery_info.NTC_number == 0){
		battery_handler->battery_info.NTC_number = receive_buf[REPLY_NTC_IDX];
		battery_handler->battery_info.NTC_content = (uint16_t*)malloc(battery_handler->battery_info.NTC_number*sizeof(uint16_t));
	}
	//for second onward iteration
	//Check if NTC number is the same, do nothing
	else if (battery_handler->battery_info.NTC_number == receive_buf[REPLY_NTC_IDX]){
		battery_handler->battery_info.NTC_number = receive_buf[REPLY_NTC_IDX];
	}
	//Check if NTC is different
	//realloc memory to change the size of array
	else if (battery_handler->battery_info.NTC_number != receive_buf[REPLY_NTC_IDX]){
		if(battery_handler->battery_info.NTC_content != NULL){
			battery_handler->battery_info.NTC_number = receive_buf[REPLY_NTC_IDX];
			battery_handler->battery_info.NTC_content = realloc(battery_handler->battery_info.NTC_content, battery_handler->battery_info.NTC_number*sizeof(uint16_t));
		}
	}
	for(int i = 0; i <= battery_handler->battery_info.NTC_number; i++){
		battery_handler->battery_info.NTC_content[i] =  (receive_buf[REPLY_NTC_CONTENT_IDX + 2*i] << 8) & 0xFF00;
		battery_handler->battery_info.NTC_content[i] |=  receive_buf[REPLY_NTC_CONTENT_IDX + 2*i + 1] & 0x00FF;
	}
}

uint16_t CalculateCheckSum(uint8_t buf[], int start_pos, int end_pos){
	uint16_t checksum =0;
	for(int i = start_pos; i <= end_pos; i++){
		checksum += (uint16_t)buf[i];
	}
	//Formula given by datasheet
	checksum = 0xFFFF - checksum + 1;
	return checksum;
}


