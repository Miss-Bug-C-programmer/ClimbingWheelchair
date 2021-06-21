/** @file   common.h
 *  @brief  Header file of commonly used macros and functions.
 *  @author Rehabilitation Research Institute of Singapore / MRBA Team
 */

#ifndef   	MRBA3_COMMON_H
#define		MRBA3_COMMON_H

#include "stm32f4xx_hal.h"

typedef struct
{
	GPIO_TypeDef* 		gpioPort;
	uint16_t		gpioPin;
	uint8_t			curRead;
	uint8_t			preRead;
	uint8_t   		state;
	uint32_t	    	lastDebounceTime;
}Button_TypeDef;

uint8_t GPIO_Digital_Filtered_Input(Button_TypeDef* hgpio, uint32_t debounce_time);

#endif
