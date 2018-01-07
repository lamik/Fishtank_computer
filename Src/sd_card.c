/*
 * sd_card.c
 *
 *  Created on: 4 sty 2018
 *      Author: Mateusz Salamon
 */

#include "main.h"
#include "stm32f1xx_hal.h"
#include "spi.h"
#include "integer.h"
#include "sd_card.h"

void SD_SelectCard() {
	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
}

void SD_DeselectCard() {
	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
}

void SD_WaitUntilReady()
{
	uint8_t ans = 1;
	while (ans != 0xFF) {
		HAL_SPI_Receive(&hspi2, &ans, 2, 10);
	}
}

uint8_t SD_Command(uint8_t command, uint32_t arg)
{

	BYTE n, res, tmp, arg_tmp[4];

	if (command & 0x80) {	/* ACMD<n> is the command sequense of CMD55-CMD<n> */
		command &= 0x7F;
		res = SD_Command(CMD55, 0);
		if (res > 1) return res;
	}

	/* Select the card */
	tmp = 0xFF;
	SD_DeselectCard();
	HAL_SPI_Transmit(&hspi2, &tmp, 1, 10);
	SD_SelectCard();
	HAL_SPI_Transmit(&hspi2, &tmp, 1, 10);

	/* Send a command packet */
	command |= 0x40;
	arg_tmp[0] = (BYTE)(arg >> 24);
	arg_tmp[1] = (BYTE)(arg >> 16);
	arg_tmp[2] = (BYTE)(arg >> 8);
	arg_tmp[3] = (BYTE)arg;
	HAL_SPI_Transmit(&hspi2, &command, 1, 10);	/* Start + Command index */
	HAL_SPI_Transmit(&hspi2, arg_tmp, 4, 10);		/* Argument[0..24] */
	n = 0x01;							/* Dummy CRC + Stop */
	if (command == (CMD0 | 0x40)) n = 0x95;			/* Valid CRC for CMD0(0) */
	if (command == (CMD8 | 0x40)) n = 0x87;			/* Valid CRC for CMD8(0x1AA) */
	HAL_SPI_Transmit(&hspi2, &n, 1, 10);

	/* Receive a command response */
	n = 20;								/* Wait for a valid response in timeout of 10 attempts */
	do {
		HAL_SPI_Receive(&hspi2, &res, 1, 10);
	} while ((res & 0x80) && --n);

	return res;			/* Return with the response value */
}


