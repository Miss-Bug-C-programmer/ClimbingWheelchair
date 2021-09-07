/** @file   adc.h
 *  @brief  Source file of AD7606 external ADC chip.
 *					Channel allocation:
 *					-------------------------------------------
 *					|			Channel				|				Sensor			|
 *					|				 1				|			Joystick X axis		|
 *					|				 2				|			Joystick Y axis	  	|
 *					|				 3				|				None			|
 *					|				 4				|				None			|
 *					|				 5				|				None			|
 *					|				 6				|				None			|
 *					|				 7				|				None			|
 *					|				 8				|				None			|
 *					-------------------------------------------
 *  @author: Ang Chin Xian
 *  @by: Rehabilitation Research Institute of Singapore
 *
 */

#include "adc.h"


SPI_HandleTypeDef hspi1;

//ADCHandle         hADC;

void ADC_Init()
{
	//Copy I2C CubeMX handle to local library
//	memcpy(&hspi1, SPIChnd, sizeof(*SPIChnd));

	//Delay for SPI encoder to startup
//	HAL_Delay(100);

	/*Reset AD7606*/
	AD7606_CS_HIGH;
	AD7606_CV_LOW;

	AD7606_RST_LOW;
	HAL_Delay(1);
	AD7606_RST_HIGH;
	HAL_Delay(1);
	AD7606_RST_LOW;

	/*Set analog input range*/
	AD7606_RANGE_LOW_5V;
	HAL_Delay(1);

	/*Set oversampling ratio to 32*/
	HAL_GPIO_WritePin(AD7606_OS2_PIN_Port, AD7606_OS2_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(AD7606_OS1_PIN_Port, AD7606_OS1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(AD7606_OS0_PIN_Port, AD7606_OS0_PIN, GPIO_PIN_SET);
	HAL_Delay(1);

//	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
//	HAL_Delay(500);
//	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

}

void ADC_DataRequest(void)
{
	AD7606_CV_LOW;
	AD7606_CV_HIGH;
}


void ADC_Read(int16_t *data)
{
	AD7606_CS_LOW;
	HAL_SPI_Receive(&hspi1, (uint8_t *)data, CHANNEL_NUM, 2);
	AD7606_CS_HIGH;
}
