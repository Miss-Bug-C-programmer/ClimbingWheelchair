/** @file   encoder.c
 *  @brief  Source file of Briter multi-turn CAN bus encoder.
 *  @author Rehabilitation Research Institute of Singapore / MRBA Team
 */

#include "encoder.h"

EncoderHandle hEncoderLeftPull, hEncoderLeftTurn, hEncoderRightPull, hEncoderRightTurn;
CAN_FilterTypeDef canfil_1;
CAN_FilterTypeDef canfil_2;
uint8_t incoming[8];
CAN_RxHeaderTypeDef RxHeader;

void ENCODER_Init(void)
{
  //Assign each encoder to one of the two CAN buses
  hEncoderLeftPull.hcan = &hcan2_left;
  hEncoderLeftTurn.hcan = &hcan2_left;
  hEncoderRightPull.hcan = &hcan1_right;
  hEncoderRightTurn.hcan = &hcan1_right;
	
	//Set Tx header for each encoder handle
	ENCODER_Set_TxHeader(&hEncoderLeftPull, ENC_ADDR_LEFT_PULL);
	ENCODER_Set_TxHeader(&hEncoderLeftTurn, ENC_ADDR_LEFT_TURN);
	ENCODER_Set_TxHeader(&hEncoderRightPull, ENC_ADDR_RIGHT_PULL);
	ENCODER_Set_TxHeader(&hEncoderRightTurn, ENC_ADDR_RIGHT_TURN);
	 
	//Filter Config - FIFO1 is assigned to hcan1_right and FIFO1 is assigned to hcan2_left
	canfil_1.FilterBank = 0;
	canfil_1.FilterMode = CAN_FILTERMODE_IDMASK;
	canfil_1.FilterScale = CAN_FILTERSCALE_32BIT;
	canfil_1.FilterIdHigh = 0x0000;
	canfil_1.FilterIdLow = 0x0000;
	canfil_1.FilterMaskIdHigh = 0x0000;
	canfil_1.FilterMaskIdLow = 0x0000;
	canfil_1.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfil_1.FilterActivation = ENABLE;
	canfil_1.SlaveStartFilterBank = 14;
	
	canfil_2.FilterBank = 14;
	canfil_2.FilterMode = CAN_FILTERMODE_IDMASK;
	canfil_2.FilterScale = CAN_FILTERSCALE_32BIT;
	canfil_2.FilterIdHigh = 0x0000;
	canfil_2.FilterIdLow = 0x0000;
	canfil_2.FilterMaskIdHigh = 0x0000;
	canfil_2.FilterMaskIdLow = 0x0000;
	canfil_2.FilterFIFOAssignment = CAN_RX_FIFO1;
	canfil_2.FilterActivation = ENABLE;
	canfil_2.SlaveStartFilterBank = 14;

	HAL_CAN_ConfigFilter(&hcan1_right, &canfil_1);
	HAL_CAN_ConfigFilter(&hcan2_left, &canfil_2);
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
}

void ENCODER_Read(EncoderHandle* Encoder_ptr){
	Encoder_ptr->sendData[0] = Encoder_ptr->canTxHeader.DLC;
	Encoder_ptr->sendData[1] = Encoder_ptr->canTxHeader.StdId;
	Encoder_ptr->sendData[2] = 0x01;
	Encoder_ptr->sendData[3] = 0x00;
	
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
	ENCODER_Get_Angle(&hEncoderLeftPull);
	ENCODER_Get_Angle(&hEncoderLeftTurn);
	ENCODER_Get_Angle(&hEncoderRightPull);
	ENCODER_Get_Angle(&hEncoderRightTurn);
}
