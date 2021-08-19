/** @file   encoder.c
 *  @brief  Source file of Briter multi-turn CAN bus encoder.
 *  @author Rehabilitation Research Institute of Singapore / MRBA Team
 */

#include "encoder.h"


EncoderHandle encoderLeft, encoderRight;
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
CAN_FilterTypeDef canfil_1;
CAN_FilterTypeDef canfil_2;
//uint8_t incoming[8];
//CAN_RxHeaderTypeDef RxHeader;

void ENCODER_Init(void)
{
  //Assign each encoder to one of the two CAN buses
	encoderLeft.hcan = &hcan1;
	encoderRight.hcan = &hcan1;
	
	//Set Tx header for each encoder handle
	ENCODER_Set_TxHeader(&encoderLeft, ENC_ADDR_LEFT);
	ENCODER_Set_TxHeader(&encoderRight, ENC_ADDR_RIGHT);
	 
	//Filter Config - FIFO1 is assigned to hcan1_right and FIFO1 is assigned to hcan2_left
	canfil_1.FilterBank = 0;
	canfil_1.FilterMode = CAN_FILTERMODE_IDMASK;
	canfil_1.FilterScale = CAN_FILTERSCALE_32BIT;
	canfil_1.FilterIdHigh = 0x0000;
	canfil_1.FilterIdLow = 0x0000;
	canfil_1.FilterMaskIdHigh = 0x0000;
	canfil_1.FilterMaskIdLow = 0x0000;
	canfil_1.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfil_1.FilterActivation = CAN_FILTER_ENABLE;
	canfil_1.SlaveStartFilterBank = 14;

	
	canfil_2.FilterBank = 14;
	canfil_2.FilterMode = CAN_FILTERMODE_IDMASK;
	canfil_2.FilterScale = CAN_FILTERSCALE_32BIT;
	canfil_2.FilterIdHigh = 0x0000;
	canfil_2.FilterIdLow = 0x0000;
	canfil_2.FilterMaskIdHigh = 0x0000;
	canfil_2.FilterMaskIdLow = 0x0000;
	canfil_2.FilterFIFOAssignment = CAN_RX_FIFO1;
	canfil_2.FilterActivation = CAN_FILTER_ENABLE;
	canfil_2.SlaveStartFilterBank = 14;

	if(HAL_CAN_ConfigFilter(&hcan1, &canfil_1) != HAL_OK )	Error_Handler();
	if(HAL_CAN_ConfigFilter(&hcan2, &canfil_2) != HAL_OK )	Error_Handler();

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
	Encoder_ptr->angleDeg = (Encoder_ptr->rawRead[3] + Encoder_ptr->rawRead[4]*0x100 + Encoder_ptr->rawRead[5]*0x10000)*360/0x1000;
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
	ENCODER_Get_Angle(&encoderLeft);
	ENCODER_Get_Angle(&encoderRight);
}
