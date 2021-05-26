/** @file   adc.h
 *  @brief  Header file of AD7606 external ADC chip.
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

#ifndef ADC_H
#define ADC_H

#include "stm32f429xx.h"

//Number of channels used
#define CHANNEL_NUM		8

// GPIO definition of AD7606
#define AD7606_BUSY_EXTI_IRQn 			EXTI9_5_IRQn

#define AD7606_CS_PIN             		GPIO_PIN_4
#define AD7606_CS_GPIO_PORT           	GPIOA
#define AD7606_CLK_PIN                	GPIO_PIN_5
#define AD7606_CLK_GPIO_PORT          	GPIOA
#define AD7606_MISO_PIN              	GPIO_PIN_6
#define AD7606_MISO_GPIO_PORT        	GPIOA
#define AD7606_BUSY_PIN               	GPIO_PIN_7
#define AD7606_BUSY_GPIO_PORT        	GPIOA
#define AD7606_RST_PIN                	GPIO_PIN_4
#define AD7606_RST_GPIO_PORT          	GPIOC
#define AD7606_CV_PIN                 	GPIO_PIN_5
#define AD7606_CV_GPIO_PORT           	GPIOC

#define AD7606_RANGE_PIN				GPIO_PIN_0
#define AD7606_RANGE_PIN_Port 			GPIOC
#define AD7606_OS2_PIN					GPIO_PIN_1
#define AD7606_OS2_PIN_Port 			GPIOC
#define AD7606_OS1_PIN					GPIO_PIN_2
#define AD7606_OS1_PIN_Port 			GPIOC
#define AD7606_OS0_PIN					GPIO_PIN_3
#define AD7606_OS0_PIN_Port 			GPIOC

// Controlling definition of AD7606
#define AD7606_CS_HIGH					      	HAL_GPIO_WritePin(AD7606_CS_GPIO_PORT, AD7606_CS_PIN, GPIO_PIN_SET)
#define AD7606_CS_LOW							HAL_GPIO_WritePin(AD7606_CS_GPIO_PORT, AD7606_CS_PIN, GPIO_PIN_RESET)
#define AD7606_RST_HIGH					      	HAL_GPIO_WritePin(AD7606_RST_GPIO_PORT, AD7606_RST_PIN, GPIO_PIN_SET)
#define AD7606_RST_LOW					      	HAL_GPIO_WritePin(AD7606_RST_GPIO_PORT, AD7606_RST_PIN, GPIO_PIN_RESET)
#define AD7606_CV_HIGH							HAL_GPIO_WritePin(AD7606_CV_GPIO_PORT, AD7606_CV_PIN, GPIO_PIN_SET)
#define AD7606_CV_LOW						 	HAL_GPIO_WritePin(AD7606_CV_GPIO_PORT, AD7606_CV_PIN, GPIO_PIN_RESET)
#define AD7606_RANGE_HIGH_10V					HAL_GPIO_WritePin(AD7606_RANGE_PIN_Port, AD7606_RANGE_PIN, GPIO_PIN_SET)
#define AD7606_RANGE_LOW_5V						HAL_GPIO_WritePin(AD7606_RANGE_PIN_Port, AD7606_RANGE_PIN, GPIO_PIN_RESET)




/** @brief  Initializes AD7606.
  * @param  None.
  * @retval None.
  */
void ADC_Init(void);

/** @brief  Triggers one sample from AD7606.
	*	After completing the conversion, the busy pin will trigger a GPIO interrupt callback function,
	*	and then the data can be read in HAL_GPIO_EXTI_Callback().
  * @param  None.
  * @retval None.
  */
void ADC_DataRequest(void);

/** @brief  Read the value from AD7606 via SPI1.
  * Note: Normally use in HAL_GPIO_EXTI_Callback() after calling ADC_DataRequest().
  * @param  data: Pointer to data buffer.  int16_t AD7606_rawData[8];
  * @retval None.
  */
void ADC_Read(int16_t *data);

extern ADCHandle  hADC;

#endif
