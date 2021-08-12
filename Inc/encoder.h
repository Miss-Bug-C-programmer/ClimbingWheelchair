/** @file   encoder.h
 *  @brief  Header file of Briter multi-turn CAN bus encoder.
 *  @author Rehabilitation Research Institute of Singapore / MRBA Team
 */

#ifndef MRBA3_ENCODER_H
#define MRBA3_ENCODER_H

#include "common.h"

#define ENC_ADDR_RIGHT_TURN		1U
#define ENC_ADDR_LEFT_TURN		2U
#define ENC_ADDR_RIGHT_PULL		3U
#define ENC_ADDR_LEFT_PULL		4U


typedef struct
{
  CAN_HandleTypeDef*    hcan;
  CAN_RxHeaderTypeDef canRxHeader;
  CAN_TxHeaderTypeDef canTxHeader;
  uint32_t canMailbox;
	union UInt32UInt8 angle32Bit;
	uint32_t angleDeg;
  uint8_t sendData[4];
  uint8_t rawRead[8];
	
}EncoderHandle;

extern uint8_t incoming[8];

extern CAN_RxHeaderTypeDef RxHeader;

void ENCODER_Sort_Incoming(uint8_t* incoming_array, EncoderHandle* Encoder_ptr);

void ENCODER_Init(void);

void ENCODER_Set_TxHeader(EncoderHandle* Encoder_ptr, uint32_t Encoder_Address);

void ENCODER_Read(EncoderHandle* Encoder_ptr);

void ENCODER_Get_Angle(EncoderHandle* Encoder_ptr);

void ENCODER_Set_ZeroPosition(EncoderHandle* Encoder_ptr);

void ENCODER_Get_AllAngles(void);

extern EncoderHandle hEncoderLeftPull, hEncoderLeftTurn, hEncoderRightPull, hEncoderRightTurn;
extern CAN_FilterTypeDef canfil_1;
extern CAN_FilterTypeDef canfil_2;


//Paste this under Rx interrupt function to sort incoming messages 
/*
HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, incoming); // change FIFO accordingly
switch(incoming[1]){
	case 0x04:
		ENCODER_Sort_Incoming(incoming, &hEncoderLeftPull);
		break;
	case 0x02:
		ENCODER_Sort_Incoming(incoming, &hEncoderLeftTurn);
		break;
	case 0x03:
		ENCODER_Sort_Incoming(incoming, &hEncoderRightPull);
		break;
	case 0x01:
		ENCODER_Sort_Incoming(incoming, &hEncoderRightTurn);
		break;
}
*/

#endif
