/*
 * ds3231.c

 *
 *  Created on: 7 gru 2017
 *      Author: Mateusz Salamon
 */
#include "main.h"
#include "stm32f1xx_hal.h"
#include "i2c.h"
#include "gpio.h"

#include "ds3231.h"

void DS3231_Init(DS3231_t *rtc, I2C_HandleTypeDef *i2c, uint8_t rtc_addres)
{
	rtc->i2c_h = i2c;
	rtc->rtc_i2c_address = rtc_addres;

    DS3231_Write_Register(rtc, DS3231_CONTROL_ADDR, 0x04); // 1Hz square freq
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn); // Enable INT PB5 pin EXTI9_5 interrupt
}

void DS3231_Write_Register(DS3231_t *rtc, uint8_t reg, uint8_t val)
{
	HAL_I2C_Mem_Write(rtc->i2c_h, rtc->rtc_i2c_address, reg, 1, &val, 1, 10);
}

uint8_t DS3231_Read_Register(DS3231_t *rtc, uint8_t reg)
{
	uint8_t ret;
	HAL_I2C_Mem_Read(rtc->i2c_h, rtc->rtc_i2c_address, reg, 1, &ret, 1, 10);
	return ret;
}
