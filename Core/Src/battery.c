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
#define EOI 			0x77	/*!< Indicate start of transmission >*/
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
#define REPLY_FLAG_IDX 		2
#define REPLY_LENGTH_IDX 	3
#define REPLY_INFO_IDX		4

enum Command{				/*!< Indicate what reply want from hardware >*/
	BATTERY_BASIC_STATE = 0x03,
	SINGLE_BATTERY_VOLTAGE,
	BATTERY_SERIAL_NUMBER
};

static uint16_t CalculateCheckSum(uint8_t buf[], int start_pos, int end_pos);

void BatteryInit(batteryHandler* battery_handler, UART_HandleTypeDef* huart){
	battery_handler->huart = huart;
	memset(&battery_handler->battery_info, 0, sizeof(battery_handler->battery_info));
	battery_handler->battery_info.NTC_content = NULL;
}

void BatteryDeInit(batteryHandler* battery_handler){
	free(battery_handler->huart);
	free(battery_handler->battery_info.NTC_content);

}

void getBatteryState(batteryHandler* battery_handler){
	uint8_t send_buf[7] = {0};
	send_buf[0] = SOI;
	send_buf[1] = BATTERY_READ;
	send_buf[2] = BATTERY_BASIC_STATE;
	send_buf[3] = 0x00; //Length of info desired, 0 if want to get all info
	uint16_t checksum = CalculateCheckSum(send_buf, SEND_COMMAND_IDX, SEND_LENGTH_IDX);
	send_buf[4] = (checksum >> 8) & 0xFF;
	send_buf[5] = checksum & 0xFF;
	send_buf[6] = EOI;
	HAL_UART_Transmit_DMA(battery_handler->huart, send_buf, sizeof(send_buf));
}

void ReadBatteryState(batteryHandler* battery_handler, uint8_t receive_buf[]){
	//Last index number in buffer
	uint8_t end_idx = REPLY_INFO_IDX + receive_buf[REPLY_LENGTH_IDX] + 2;
	//check receive start, r/w state. flag and end buffer
	if (	receive_buf[0] != SOI ||
			receive_buf[end_idx] != EOI ||
			(receive_buf[1] != BATTERY_BASIC_STATE || receive_buf[1] != BATTERY_READ)
			)
		return;

	//Checksum_end_idx need to consider the length received from reply,
	uint8_t checksum_end_idx = REPLY_INFO_IDX + receive_buf[REPLY_LENGTH_IDX] - 1;
	uint16_t checksum = CalculateCheckSum(receive_buf, REPLY_FLAG_IDX, checksum_end_idx);
	//Check the last second and third byte, if doesnt match the check sum, dispose the data
	if (	receive_buf[REPLY_INFO_IDX + receive_buf[REPLY_LENGTH_IDX] 	  ] != ((checksum >> 8) & 0xFF) ||
			receive_buf[REPLY_INFO_IDX + receive_buf[REPLY_LENGTH_IDX] +1 ] != (checksum & 0xFF)	)
			return;

	battery_handler->battery_info.total_voltage = (receive_buf[5] << 8) | receive_buf[6];
	battery_handler->battery_info.current = (receive_buf[7] << 8) | receive_buf[8];
	battery_handler->battery_info.remaining_capacity = (receive_buf[9] << 8) | receive_buf[10];
	battery_handler->battery_info.nominal_capacity = (receive_buf[11] << 8) | receive_buf[12];
	battery_handler->battery_info.cycles = (receive_buf[13] << 8) | receive_buf[14];
	battery_handler->battery_info.production_date = (receive_buf[15] << 8) | receive_buf[16];
	battery_handler->battery_info.balanced_state[0] = (receive_buf[17] << 8) | receive_buf[18];
	battery_handler->battery_info.balanced_state[1] = (receive_buf[19] << 8) | receive_buf[20];
	battery_handler->battery_info.protection_state = (receive_buf[21] << 8) | receive_buf[22];
	battery_handler->battery_info.software_version = receive_buf[23];
	battery_handler->battery_info.remaining_capacity_RSOC = receive_buf[24];
	battery_handler->battery_info.FET_control_status = receive_buf[25];
	battery_handler->battery_info.battery_number = receive_buf[26];
	//Check if NTC number is the same
	//For first iteration
	if (battery_handler->battery_info.NTC_number == 0){
		battery_handler->battery_info.NTC_number = receive_buf[27];
		battery_handler->battery_info.NTC_content = (uint16_t*)malloc(battery_handler->battery_info.NTC_number*sizeof(uint16_t));
	}
	//for second onward iteration
	//Check if NTC number is the same, do nothing
	else if (battery_handler->battery_info.NTC_number == receive_buf[27]){
		battery_handler->battery_info.NTC_number = receive_buf[27];
	}
	//Check if NTC is different
	//realloc memory to change the size of array
	else if (battery_handler->battery_info.NTC_number != receive_buf[27]){
		if(battery_handler->battery_info.NTC_content != NULL){
			battery_handler->battery_info.NTC_number = receive_buf[27];
			battery_handler->battery_info.NTC_content = realloc(battery_handler->battery_info.NTC_content, battery_handler->battery_info.NTC_number*sizeof(uint16_t));
		}
	}
	for(int i = 0; i <= battery_handler->battery_info.NTC_number; i++){
		battery_handler->battery_info.NTC_content[i] =  (receive_buf[27+2*i+1] << 8) & 0xFF00;
		battery_handler->battery_info.NTC_content[i] |=  receive_buf[27+2*i+2] & 0x00FF;
	}
}

uint16_t CalculateCheckSum(uint8_t buf[], int start_pos, int end_pos){
	uint16_t checksum;
	for(int i = start_pos; i <= end_pos; i++){
		checksum += (uint16_t)buf[i];
	}
	//Formula given by datasheet
	checksum = 0xFFFF - checksum + 1;
	return checksum;
}


