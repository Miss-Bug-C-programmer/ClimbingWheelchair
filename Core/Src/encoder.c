/** @file   encoder.c
 *  @brief  Source file of Briter multi-turn CAN bus encoder.
 *  @author Rehabilitation Research Institute of Singapore / MRBA Team
 */

#include "encoder.h"


EncoderHandle encoderBack, encoderFront;
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
//uint8_t incoming[8];
//CAN_RxHeaderTypeDef RxHeader;

void ENCODER_Init(void)
{
  //Assign each encoder to one of the two CAN buses
	encoderBack.hcan = &hcan1;
	encoderFront.hcan = &hcan1;
	
	//Set Tx header for each encoder handle
	ENCODER_Set_TxHeader(&encoderBack, ENC_ADDR_LEFT);
	ENCODER_Set_TxHeader(&encoderFront, ENC_ADDR_RIGHT);

}

void ENCODER_Sort_Incoming(uint8_t* incoming_array, EncoderHandle* Encoder_ptr){
	Encoder_ptr->rawRead[0] = incoming_array[0];
	Encoder_ptr->rawRead[1] = incoming_array[1];
	Encoder_ptr->rawRead[2] = incoming_array[2];
	Encoder_ptr->rawRead[3] = incoming_array[3];
	Encoder_ptr->rawRead[4] = incoming_array[4];
	Encoder_ptr->rawRead[5] = incoming_array[5];
	Encoder_ptr->rawRead[6] = incoming_array[6];
	Encoder_ptr->rawRead[7] = incoming_array[7];
}

void ENCODER_Set_TxHeader(EncoderHandle* Encoder_ptr, uint32_t Encoder_Address){
	Encoder_ptr->canTxHeader.DLC = 4;
	Encoder_ptr->canTxHeader.IDE = CAN_ID_STD;
	Encoder_ptr->canTxHeader.RTR = CAN_RTR_DATA;
	Encoder_ptr->canTxHeader.StdId = Encoder_Address;
	Encoder_ptr->canTxHeader.TransmitGlobalTime = DISABLE;
	Encoder_ptr->canTxHeader.ExtId = 0;
}

void ENCODER_Read(EncoderHandle* Encoder_ptr){
	Encoder_ptr->sendData[0] = Encoder_ptr->canTxHeader.DLC;
	Encoder_ptr->sendData[1] = Encoder_ptr->canTxHeader.StdId;
	Encoder_ptr->sendData[2] = 0x01;
	Encoder_ptr->sendData[3] = 0x00;
	
	HAL_CAN_AddTxMessage(Encoder_ptr->hcan, &(Encoder_ptr->canTxHeader), Encoder_ptr->sendData, &(Encoder_ptr->canMailbox));
}

void ENCODER_SetBaudRate(EncoderHandle* Encoder_ptr){
	Encoder_ptr->sendData[0] = Encoder_ptr->canTxHeader.DLC;
	Encoder_ptr->sendData[1] = Encoder_ptr->canTxHeader.StdId;
	Encoder_ptr->sendData[2] = 0x03;
	Encoder_ptr->sendData[3] = 0x01;

	HAL_CAN_AddTxMessage(Encoder_ptr->hcan, &(Encoder_ptr->canTxHeader), Encoder_ptr->sendData, &(Encoder_ptr->canMailbox));
}

void ENCODER_Get_Angle(EncoderHandle* Encoder_ptr){
	ENCODER_Read(Encoder_ptr);
	Encoder_ptr->angle32Bit.b8[0] = Encoder_ptr->rawRead[3];
	Encoder_ptr->angle32Bit.b8[1] = Encoder_ptr->rawRead[4];
	Encoder_ptr->angle32Bit.b8[2] = Encoder_ptr->rawRead[5];
	Encoder_ptr->angle32Bit.b8[3] = Encoder_ptr->rawRead[6];

	//Get the outer gear encoder position
	//Gear ration from inner to outer gear is 1:2. Therefore, (2*4096=)8192 is used
//	Encoder_ptr->encoder_pos = (Encoder_ptr->rawRead[3] + (Encoder_ptr->rawRead[4] << 8) + (Encoder_ptr->rawRead[5] << 16)) ; //Get single turn encoder reading
	Encoder_ptr->encoder_pos = (Encoder_ptr->rawRead[3] + (Encoder_ptr->rawRead[4] << 8) + (Encoder_ptr->rawRead[5] << 16)); //Get single turn encoder reading

	//Convert from encoder position to angle in degree
//	Encoder_ptr->angleDeg = (Encoder_ptr->encoder_pos * 360 /8192) ; //Get encoder angle

}

void ENCODER_Set_ZeroPosition(EncoderHandle* Encoder_ptr){
	Encoder_ptr->sendData[0] = Encoder_ptr->canTxHeader.DLC;
	Encoder_ptr->sendData[1] = Encoder_ptr->canTxHeader.StdId;
	Encoder_ptr->sendData[2] = 0x06;
	Encoder_ptr->sendData[3] = 0x00;
	
	HAL_CAN_AddTxMessage(Encoder_ptr->hcan, &(Encoder_ptr->canTxHeader), Encoder_ptr->sendData, &(Encoder_ptr->canMailbox));
	//HAL_CAN_GetRxMessage(Encoder_ptr->hcan, CAN_RX_FIFO0, &(Encoder_ptr->canRxHeader), Encoder_ptr->rawRead);
}

void ENCODER_Get_AllAngles(void){
	ENCODER_Get_Angle(&encoderBack);
	ENCODER_Get_Angle(&encoderFront);
}
