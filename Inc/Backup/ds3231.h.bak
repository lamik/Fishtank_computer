/*
 * ds3231.h
 *
 *  Created on: 7 gru 2017
 *      Author: Mateusz Salamon
 */

#ifndef DS3231_H_
#define DS3231_H_

// i2c slave address of the DS3231 chip
#define DS3231_I2C_ADDR             0x68

// timekeeping registers
#define DS3231_TIME_CAL_ADDR        0x00
#define DS3231_ALARM1_ADDR          0x07
#define DS3231_ALARM2_ADDR          0x0B
#define DS3231_CONTROL_ADDR         0x0E
#define DS3231_STATUS_ADDR          0x0F
#define DS3231_AGING_OFFSET_ADDR    0x10
#define DS3231_TEMPERATURE_ADDR     0x11

// control register bits
#define DS3231_A1IE     0x1
#define DS3231_A2IE     0x2
#define DS3231_INTCN    0x4

// status register bits
#define DS3231_A1F      0x1
#define DS3231_A2F      0x2
#define DS3231_OSF      0x80

typedef struct
{
	I2C_HandleTypeDef *i2c_h; // I2C handler
	uint8_t rtc_i2c_address;

}DS3231_t;

DS3231_t rtc;

void DS3231_Init(DS3231_t *rtc, I2C_HandleTypeDef *i2c, uint8_t rtc_addres);

void DS3231_Write_Register(DS3231_t *rtc, uint8_t reg, uint8_t val);
uint8_t DS3231_Read_Register(DS3231_t *rtc, uint8_t reg);


#endif /* DS3231_H_ */
