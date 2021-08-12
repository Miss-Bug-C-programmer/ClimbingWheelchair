/** @file   common.c
 *  @brief  Source file of commonly used macros and functions.
 *  @author Rehabilitation Research Institute of Singapore / MRBA Team
 */

#include "button.h"

uint8_t GPIO_Digital_Filtered_Input(Button_TypeDef* hgpio, uint32_t debounce_time)
{
	// read the state of the switch into a local variable (LOW, when the button is not pressed)
	hgpio->curRead = HAL_GPIO_ReadPin(hgpio->gpioPort, hgpio->gpioPin);
	//check if the button was pressed
	if (hgpio->curRead != hgpio->preRead)
		// reset the debouncing timer
		hgpio->lastDebounceTime = HAL_GetTick();
	// whatever the reading is at, it's been there for longer than the debounce delay, so the current value is safe
//	&& (hgpio->curRead != hgpio->state)
	if (((HAL_GetTick() - hgpio->lastDebounceTime) > debounce_time) )
	{
		hgpio->state = hgpio->curRead;
		
		if (hgpio->state == GPIO_PIN_SET) //Effective only when Pin is RESET
		{
			return GPIO_PIN_SET;
		}
	}
	// Update the last button read
	hgpio->preRead = hgpio->curRead;
	return GPIO_PIN_RESET;
}
