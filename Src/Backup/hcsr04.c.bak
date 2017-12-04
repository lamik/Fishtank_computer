/*
 * hcsr04.c
 *
 *  Created on: 4 gru 2017
 *      Author: Mateusz Salamon
 */

#include "main.h"
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "gpio.h"

#include "hcsr04.h"

void HCSR04_Delay(uint16_t time_us)
{
	_HCSR04_TIMER.Instance->CNT = 0;
	while(_HCSR04_TIMER.Instance->CNT <= time_us);
}

uint8_t HCSR04_Init()
{
	HAL_TIM_Base_Start(&_HCSR04_TIMER);
	//pins are preconfigured in CubeMX
	HAL_GPIO_WritePin(HCSR04_Trig_GPIO_Port, HCSR04_Trig_Pin, GPIO_PIN_RESET);

	if(HCSR04_Read() >= 0)
	{
		//sensor ok
		return 1;
	}
	//sensor error
	return 0;
}

float HCSR04_Read()
{
	uint32_t timeout;
	uint16_t time;
	float distance = 0;

	HAL_GPIO_WritePin(HCSR04_Trig_GPIO_Port, HCSR04_Trig_Pin, GPIO_PIN_RESET); // Trigger to low
	HCSR04_Delay(2);
	HAL_GPIO_WritePin(HCSR04_Trig_GPIO_Port, HCSR04_Trig_Pin, GPIO_PIN_SET); // Trigger to high - triggered
	HCSR04_Delay(10); // keep high per 10 us
	HAL_GPIO_WritePin(HCSR04_Trig_GPIO_Port, HCSR04_Trig_Pin, GPIO_PIN_RESET); // Trigger to low again

	timeout = _HCSR04_TIMEOUT; // define timeout

	while(!HAL_GPIO_ReadPin(HCSR04_Echo_GPIO_Port, HCSR04_Echo_Pin)) // wait for high state on Echo
	{
		if(timeout-- == 0x00)
			return -1; // error if timeout is reached
	}

	_HCSR04_TIMER.Instance->CNT = 0;
	while(HAL_GPIO_ReadPin(HCSR04_Echo_GPIO_Port, HCSR04_Echo_Pin)){} // wait while Echo is high
	time = _HCSR04_TIMER.Instance->CNT; // read us from timer

	distance = (float)time / 2.0 * 0.0343;

	return distance;
}
