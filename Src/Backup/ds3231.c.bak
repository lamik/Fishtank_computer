/*
 * ds3231.c

 *
 *  Created on: 7 gru 2017
 *      Author: Mateusz Salamon
 */

#include <string.h>

#include "main.h"
#include "stm32f1xx_hal.h"
#include "i2c.h"
#include "gpio.h"

#include "ds3231.h"

char* week[7]={"Pon","Wto","Sro","Czw","Pia","Sob","Nie"};

uint8_t bcd2dec(uint8_t val)
{
    return ((val / 16 * 10) + (val % 16));
}

uint8_t dec2bcd(uint8_t val)
{
	return ((val / 10 * 16) + (val % 10));
}

void DS3231_Init(DS3231_t *rtc, I2C_HandleTypeDef *i2c, uint8_t rtc_addres)
{
	rtc->i2c_h = i2c;
	rtc->rtc_i2c_address = rtc_addres;

    DS3231_Write_Register(rtc, DS3231_CONTROL_ADDR, 0x04); // 1Hz square freq
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); // Enable INT PB5 pin EXTI9_5 interrupt
}

void DS3231_Write_Register(DS3231_t *rtc, uint8_t reg, uint8_t val)
{
	HAL_I2C_Mem_Write(rtc->i2c_h, rtc->rtc_i2c_address, reg, 1, &val, 1, 100);
}

uint8_t DS3231_Read_Register(DS3231_t *rtc, uint8_t reg)
{
	uint8_t ret;
	HAL_I2C_Mem_Read(rtc->i2c_h, rtc->rtc_i2c_address, reg, 1, &ret, 1, 100);
	return ret;
}

void DS3231_Set_Time(DS3231_t *rtc, uint8_t hour, uint8_t min, uint8_t sec)
{
	uint8_t Time[3];

	Time[0] = dec2bcd(sec);
	Time[1] = dec2bcd(min);
	Time[2] = dec2bcd(hour);

	HAL_I2C_Mem_Write(rtc->i2c_h, DS3231_I2C_ADDR, DS3231_TIME_CAL_ADDR, 1, Time, 3, 10);
}

void DS3231_Set_Date(DS3231_t *rtc, uint8_t mday, uint8_t mon, uint16_t year)
{
	uint8_t Date[3];

	int Y,C,M,N,D;
	M=1+(9+mon)%12;
	Y=year-(M>10);
	C=Y/100;
	D=Y%100;
	N=((13*M-1)/5+D+D/4+6*C+mday+5)%7;

	Date[0]= (7+N)%7;
	Date[1] = dec2bcd(mday);
	Date[2] = dec2bcd(mon);
	Date[3] = dec2bcd(year - 2000);

	HAL_I2C_Mem_Write(rtc->i2c_h, DS3231_I2C_ADDR, 0x03, 1, Date, 4, 10);
}

void DS3231_Get_RTC(DS3231_t *rtc)
{
	uint8_t TimeDate[7];

	HAL_I2C_Mem_Read(rtc->i2c_h, DS3231_I2C_ADDR, DS3231_TIME_CAL_ADDR, 1, TimeDate, 7, 10);

	TimeDate[5] &= 0x1F; //delete century

	rtc->sec = bcd2dec(TimeDate[0]);
	rtc->min = bcd2dec(TimeDate[1]);
	rtc->hour = bcd2dec(TimeDate[2]);
	rtc->wday = bcd2dec(TimeDate[3]);
	rtc->mday = bcd2dec(TimeDate[4]);
	rtc->mon = bcd2dec(TimeDate[5] & 0x1F); // Century is not useful
	rtc->year = bcd2dec(TimeDate[6]) + 2000; // Will we back to 1900?
	strcpy(rtc->week_day, week[rtc->wday]);
}

