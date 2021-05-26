/** @file   encoder.c
 *  @brief  Source file of modular absolute encoder - CUI AMT22.
 *  @author: Ang Chin Xian
 *  @by: Rehabilitation Research Institute of Singapore
 *
 */

#include "encoder.h"


extern SPI_HandleTypeDef hspi2;
uint16_t encoder_vals_prev[2] = {-1, -1};
double velocities_prev[2] = {0, 0};
uint32_t prev_time = 0;


void encoder_Init(void){
	//Get the CS line high which is the default inactive state
	CS1_HIGH;
	CS2_HIGH;
}

void setCSLine (Encoder encoder, state csLine){
	if (encoder == 0)
		HAL_GPIO_WritePin(SPI_CS1_GPIO_PORT, SPI_CS1_PIN, csLine);
	else if (encoder == 1)
		HAL_GPIO_WritePin(SPI_CS2_GPIO_PORT, SPI_CS2_PIN, csLine);
}

uint8_t spiWriteRead(uint8_t sendByte, Encoder encoder, state releaseLine){

	//holder for the received over SPI
	uint8_t data;

	//Initiate transfer from encoder
	setCSLine(encoder, LOW);
	DWT_Delay(3);
	HAL_SPI_Transmit(&hspi2, (uint8_t*)sendByte, 1, 100);
	HAL_SPI_Receive(&hspi2, (uint8_t *)data, 1, 100);
	DWT_Delay(3);
	setCSLine(encoder, releaseLine);

	return data;
}

uint16_t getPositionSPI(Encoder encoder, uint8_t resolution){
	uint16_t currentPosition;       //16-bit response from encoder
	uint8_t binaryArray[16];           //after receiving the position we will populate this array and use it for calculating the checksum

	//get first byte which is the high byte, shift it 8 bits. don't release line for the first byte
	currentPosition = spiWriteRead(AMT22_NOP, encoder, LOW) << 8;

	DWT_Delay(3);

	//OR the low byte with the currentPosition variable. release line after second byte
	currentPosition |= spiWriteRead(AMT22_NOP, encoder, HIGH);

	//run through the 16 bits of position and put each bit into a slot in the array so we can do the checksum calculation
	for(int i = 0; i < 16; i++)
		binaryArray[i] = (0x01) & (currentPosition >> (i));

	//using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent
	if ((binaryArray[15] == !(binaryArray[13] ^ binaryArray[11] ^ binaryArray[9] ^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3] ^ binaryArray[1]))
		  && (binaryArray[14] == !(binaryArray[12] ^ binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2] ^ binaryArray[0])))
	{
	  //we got back a good position, so just mask away the checkbits
	  currentPosition &= 0x3FFF;
	}
	else
	{
	currentPosition = 0xFFFF; //bad position
	}

	//If the resolution is 12-bits, and wasn't 0xFFFF, then shift position, otherwise do nothing
	if ((resolution == RES12) && (currentPosition != 0xFFFF))
		currentPosition = currentPosition >> 2;

	return currentPosition;

}

void setZeroSPI(Encoder encoder){

	spiWriteRead(AMT22_NOP, encoder, LOW);
	DWT_Delay(3);
	spiWriteRead(AMT22_ZERO, encoder, HIGH);
	delay(250);
}

void resetPositionSPI(Encoder encoder){
	spiWriteRead(AMT22_NOP, encoder, LOW);
	DWT_Delay(3);
	spiWriteRead(AMT22_RESET, encoder, HIGH);
	delay(250);
}


void encoderRead(uint16_t *encoder_vals){
  encoder_vals[LEFT_INDEX] = getPositionSPI(ENC1, RES14);
  encoder_vals[RIGHT_INDEX] = getPositionSPI(ENC1, RES14);

}

void calcVelFromEncoder(uint16_t *encoder_vals, double *velocities)
{
	//If previous time is not set or no previous velocities yet, set prev_time and skip this round
	if(prev_time == 0 || (encoder_vals_prev[RIGHT_INDEX] == -1 && encoder_vals_prev[LEFT_INDEX] == -1))
	{
		encoder_vals_prev[RIGHT_INDEX] = encoder_vals[RIGHT_INDEX];
		encoder_vals_prev[LEFT_INDEX] = encoder_vals[LEFT_INDEX];
		prev_time = HAL_GetTick();
		return;
	}

	//Get difference in encoder
	int16_t diff_enc_left = encoder_vals[LEFT_INDEX] - encoder_vals_prev[LEFT_INDEX];
	int16_t diff_enc_right = encoder_vals[RIGHT_INDEX] - encoder_vals_prev[RIGHT_INDEX];

	//Get difference in time
	double dt = (HAL_GetTick() - prev_time) / (double)FREQUENCY;
	if(dt == 0)
		return;

	//Encoders will wrap around, offset the wrap around if it does happen
	//Wrap around is detected by  checking if the difference in encoder value exceeds half the max encoder value
	if (diff_enc_right < -ENCODER_MAX / 2.0)
		diff_enc_right += ENCODER_MAX;
	else if(diff_enc_right > ENCODER_MAX / 2.0)
		diff_enc_right -= ENCODER_MAX;

	if (diff_enc_left < -ENCODER_MAX / 2.0)
		diff_enc_left += ENCODER_MAX;
	else if (diff_enc_left > ENCODER_MAX / 2.0)
		diff_enc_left -= ENCODER_MAX;

	//unit: m/s
	velocities[RIGHT_INDEX] = (double)diff_enc_right / ENCODER_MAX * M_PI * WHEEL_DIA / dt;
	velocities[LEFT_INDEX] = -(double)diff_enc_left / ENCODER_MAX * M_PI * WHEEL_DIA / dt;

	if(fabs(velocities[RIGHT_INDEX]) < 0.05)
		velocities[RIGHT_INDEX] = 0.000;

	if(fabs(velocities[LEFT_INDEX]) < 0.05)
		velocities[LEFT_INDEX] = 0.000;

	// Sometimes data gets lost and spikes are seen in the velocity readouts.
	// This is solved by limiting the max difference between subsequent velocity readouts.
	// If acceleration is passed, just update velocity within acceleration limits
	double right_acc = (velocities[RIGHT_INDEX] - velocities_prev[RIGHT_INDEX]) / dt;
	double left_acc = (velocities[LEFT_INDEX] - velocities_prev[LEFT_INDEX]) / dt;

	if (fabs(right_acc) > WHEEL_ACC_LIMIT)
	{
		velocities[RIGHT_INDEX] = velocities_prev[RIGHT_INDEX] + WHEEL_ACC_LIMIT * dt * (right_acc / fabs(right_acc));
	}

	if (fabs(left_acc) > WHEEL_ACC_LIMIT)
	{
		velocities[LEFT_INDEX] = velocities_prev[LEFT_INDEX] + WHEEL_ACC_LIMIT * dt * (left_acc / fabs(left_acc));
	}

	//Exponential filter for each velocity10
	velocities[RIGHT_INDEX] = velocities[RIGHT_INDEX] * EXPONENTIAL_ALPHA + (1.0 - EXPONENTIAL_ALPHA) * velocities_prev[RIGHT_INDEX];
	velocities[LEFT_INDEX] = velocities[LEFT_INDEX] * EXPONENTIAL_ALPHA + (1.0 - EXPONENTIAL_ALPHA) * velocities_prev[LEFT_INDEX];

	//Set all previous values to current values
	encoder_vals_prev[RIGHT_INDEX] = encoder_vals[RIGHT_INDEX];
	encoder_vals_prev[LEFT_INDEX] = encoder_vals[LEFT_INDEX];
	velocities_prev[RIGHT_INDEX] = velocities[RIGHT_INDEX];
	velocities_prev[LEFT_INDEX] = velocities[LEFT_INDEX];
	prev_time = HAL_GetTick();
}

















