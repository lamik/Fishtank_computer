/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "crc.h"
#include "fatfs.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "FT_Platform.h"
#include "onewire.h"
#include "ds18b20.h"
#include "hcsr04.h"
#include "ds3231.h"
#include "stm32_adafruit_sd.h"
#include "images/images.h"

#define SD_CARD_NOT_FORMATTED                    0
#define SD_CARD_FILE_NOT_SUPPORTED               1
#define SD_CARD_OPEN_FAIL                        2
#define FATFS_NOT_MOUNTED                        3
#define BSP_SD_INIT_FAILED                       4

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
Ft_Gpu_Hal_Context_t host;

ft_char8_t StringArray[20];
void(* menu_actual)(void);
uint16_t touch_tag = 0;
uint16_t x_data[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
uint16_t y_data[] = {10,43,38,32,28,22,30,31,32,33,10,15,18,13,17,20,24,35,28,29,50};
uint32_t track_data = 0;
uint16_t value = 50;
uint16_t value2 = 230;
uint8_t brightness = 128;
char bufor[60];
char file_buf[10];

OneWire_t OW;
uint8_t DS_ROM[8];
float temperature;

float distance;

Ft_Bitmap_header_t *p_bmhdr;
uint32_t ending_address;

uint8_t sd_ok = 0, fopen_ok = 0, fmount_ok = 0, fwrite_ok = 0, fclose_ok = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int32_t map(int32_t value, int32_t inMin, int32_t inMax, int32_t outMin, int32_t outMax);
void test_screen(void);
void menu1(void);
void menu2(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_CRC_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();

  /* USER CODE BEGIN 2 */
  MX_FATFS_Init();

  uint32_t wbytes; /* File write counts */
  uint8_t wtext[] = "text to write logical disk"; /* File write buffer */

  Ft_Gpu_Hal_Init_43CTP(&host);
  MX_SPI1_SpeedUp();	//Speed-up SPI for FT800

	HAL_Delay(100);
	Ft_Gpu_CoCmd_Track(&host, 50, 240, 400, 20, 10);
	menu_actual = menu1;
// OneWire configuration

	OneWire_Init(&OW, DS18B20_GPIO_Port, DS18B20_Pin);

	if (OneWire_First(&OW))
		OneWire_GetFullROM(&OW, DS_ROM);
	if (DS18B20_Is(DS_ROM))
	{
		/* Set resolution */
		DS18B20_SetResolution(&OW, DS_ROM, DS18B20_Resolution_12bits);

		/* Start conversion on all sensors */
		DS18B20_StartAll(&OW);
	}

// HCSR04 init
	HCSR04_Init();

// RTC Init
	DS3231_Init(&rtc, &hi2c2, DS3231_I2C_ADDR);

// Load Icon into FT800

			Ft_Gpu_Hal_WrCmd32(&host,CMD_INFLATE);
			//	specify starting address in graphics RAM
				IMG_Icon_back_header->Address = 0L;
				Ft_Gpu_Hal_WrCmd32(&host,IMG_Icon_back_header->Address);
			//	write the .binh contents to the FT800 FIFO command buffer, filesize=
				Ft_Gpu_Hal_WrCmdBuf(&host, IMG_Icon_back_data, 1492);
				while(Ft_Gpu_Hal_Rd16(&host,REG_CMD_WRITE)!=Ft_Gpu_Hal_Rd16(&host, REG_CMD_READ));//Wait till the compression was done
				uint16_t x = Ft_Gpu_Hal_Rd16(&host,REG_CMD_WRITE);
				Ft_Gpu_CoCmd_GetPtr(&host,0);
				IMG_Icon_settings_header->Address = Ft_Gpu_Hal_Rd32(&host,(RAM_CMD+x+4));

				Ft_Gpu_Hal_WrCmd32(&host,CMD_INFLATE);
			//	specify starting address in graphics RAM
				Ft_Gpu_Hal_WrCmd32(&host,IMG_Icon_settings_header->Address);
			//	write the .binh contents to the FT800 FIFO command buffer, filesize=
				Ft_Gpu_Hal_WrCmdBuf(&host, IMG_Icon_settings_data, 1486);
				while(Ft_Gpu_Hal_Rd16(&host,REG_CMD_WRITE)!=Ft_Gpu_Hal_Rd16(&host, REG_CMD_READ));//Wait till the compression was done
				x = Ft_Gpu_Hal_Rd16(&host,REG_CMD_WRITE);
				Ft_Gpu_CoCmd_GetPtr(&host,0);
				IMG_Phial_header->Address = Ft_Gpu_Hal_Rd32(&host,(RAM_CMD + x + 4));

				Ft_Gpu_Hal_WrCmd32(&host,CMD_INFLATE);
			//	specify starting address in graphics RAM
				Ft_Gpu_Hal_WrCmd32(&host,IMG_Phial_header->Address);
			//	write the .binh contents to the FT800 FIFO command buffer, filesize=
				Ft_Gpu_Hal_WrCmdBuf(&host, IMG_Phial_data, 1992);
				while(Ft_Gpu_Hal_Rd16(&host,REG_CMD_WRITE)!=Ft_Gpu_Hal_Rd16(&host, REG_CMD_READ));//Wait till the compression was done
				x = Ft_Gpu_Hal_Rd16(&host,REG_CMD_WRITE);
				Ft_Gpu_CoCmd_GetPtr(&host,0);
				Ft_Gpu_Hal_Rd32(&host,(RAM_CMD + x + 4));

				uint8_t i = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		/* Check if connected device is DS18B20 */
		if (DS18B20_Is(DS_ROM)) {
			/* Everything is done */
			if (DS18B20_AllDone(&OW)) {
				/* Read temperature from device */
				if (DS18B20_Read(&OW, DS_ROM, &temperature)) {
					/* Temp read OK, CRC is OK */

					/* Start again on all sensors */
					DS18B20_StartAll(&OW);

				}
			}
		}
//		DS3231_Get_RTC(&rtc);
		distance = HCSR04_Read();

		UINT tmp;
		if(FR_OK != f_open(&USERFile, "test.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE))
			if(FR_OK != f_lseek(&USERFile, f_size(&USERFile)))
					fopen_ok = 1;

		char buf_tmp[10];
		memset(buf_tmp, 0x00, sizeof(buf_tmp));
		sprintf(buf_tmp, "TEST%d ", i++);

		if(FR_OK != f_write(&USERFile, buf_tmp, 10, &tmp))
				fwrite_ok = 1;
		f_close(&USERFile);

			  menu_actual();

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == DS3231_INT_Pin)
	{
		DS3231_Get_RTC(&rtc);
	}
}

int32_t map(int32_t value, int32_t inMin, int32_t inMax, int32_t outMin, int32_t outMax)
{
	return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}


unsigned int incCMDOffset(unsigned int currentOffset, unsigned char commandSize)
{
    unsigned int newOffset;															// Used to hold new offset
    newOffset = currentOffset + commandSize;						// Calculate new offset
    if(newOffset > 4095)																// If new offset past boundary...
    {
        newOffset = (newOffset - 4096);									// ... roll over pointer
    }
    return newOffset;																		// Return new offset
}

void menu1()
{
	Ft_Gpu_CoCmd_Dlstart(&host);

			/*Setting first screen*/
			Ft_App_WrCoCmd_Buffer(&host,CLEAR_COLOR_RGB(0xFF,0xFF,0xFF));//white color
			Ft_App_WrCoCmd_Buffer(&host,CLEAR(1,1,1));//clear screen

			track_data = Ft_Gpu_Hal_Rd32(&host, REG_TRACKER);
			if((track_data & 0xff) == 10)
				value = (track_data >> 16);


			Ft_App_WrCoCmd_Buffer(&host,COLOR_RGB(0xFF,0xFF,0x20));//red color
			Ft_Gpu_CoCmd_FgColor(&host, COLOR_RGB(0x00, 0x06, 0x70));
			Ft_App_WrCoCmd_Buffer(&host,TAG(10));
			Ft_Gpu_CoCmd_Slider(&host, 50, 240, 400, 20, 0, value, 0xFFFFFF);

			Ft_App_WrCoCmd_Buffer(&host,COLOR_RGB(0x01,0x00,0x00));
			sprintf(bufor, "ID: %u", track_data & 0xFF);
			Ft_Gpu_CoCmd_Text(&host, 10, 180, 28, 0, bufor);
			sprintf(bufor, "Val: %u", value);
			Ft_Gpu_CoCmd_Text(&host, 10, 200, 28, 0, bufor);


			Ft_App_WrCoCmd_Buffer(&host,COLOR_RGB(0x01,0x00,0x00));
			sprintf(bufor, "%2d:%02d:%02d", rtc.hour, rtc.min, rtc.sec);
			Ft_Gpu_CoCmd_Text(&host, 10, 100, 28, 0, bufor);

			sprintf(bufor, "%s %2d.%02d.%04d", rtc.week_day, rtc.mday, rtc.mon, rtc.year);
			Ft_Gpu_CoCmd_Text(&host, 10, 120, 28, 0, bufor);

			if(!sd_ok)
			{
				sprintf(bufor, "SD OK");
				Ft_Gpu_CoCmd_Text(&host, 10, 140, 28, 0, bufor);
			}
			else
			{
				sprintf(bufor, "SD ERROR");
				Ft_Gpu_CoCmd_Text(&host, 10, 140, 28, 0, bufor);
			}

			if(!fmount_ok)
			{
				sprintf(bufor, "fmount OK");
				Ft_Gpu_CoCmd_Text(&host, 10, 160, 28, 0, bufor);
			}
			else
			{
				sprintf(bufor, "fmount ERROR");
				Ft_Gpu_CoCmd_Text(&host, 10, 160, 28, 0, bufor);
			}


			if(!fopen_ok)
			{
				sprintf(bufor, "fopen OK");
				Ft_Gpu_CoCmd_Text(&host, 100, 140, 28, 0, bufor);
			}
			else
			{
				sprintf(bufor, "fopen ERROR");
				Ft_Gpu_CoCmd_Text(&host, 100, 140, 28, 0, bufor);
			}

			if(!fwrite_ok)
			{
				sprintf(bufor, "fwrite OK");
				Ft_Gpu_CoCmd_Text(&host, 120, 160, 28, 0, bufor);
			}
			else
			{
				sprintf(bufor, "fwrite ERROR");
				Ft_Gpu_CoCmd_Text(&host, 120, 160, 28, 0, bufor);
			}


			Ft_Gpu_CoCmd_Text(&host, 100, 200, 28, 0, file_buf);


			// BUTTON TEST
			Ft_App_WrCoCmd_Buffer(&host,TAG(11));
			Ft_App_WrCoCmd_Buffer(&host,COLOR_RGB(0xFF,0xFF,0xFF));
			Ft_Gpu_CoCmd_GradColor(&host, COLOR_RGB(0xFF, 0xFF, 0xFF));
			Ft_Gpu_CoCmd_FgColor(&host, COLOR_RGB(0x00, 0x70, 0x12));
			Ft_Gpu_CoCmd_Button(&host, 31,10, 160,60, 27,0, "Menu1");

			Ft_App_WrCoCmd_Buffer(&host,TAG(12));
			Ft_Gpu_CoCmd_GradColor(&host, COLOR_RGB(0xFF, 0xFF, 0xFF));
			Ft_Gpu_CoCmd_FgColor(&host, COLOR_RGB(0x70, 0x41, 0x00));
			Ft_Gpu_CoCmd_Button(&host, 275,10, 160,60, 27,0, "Brightness");
			//

			// icon setting
//

//			//specify the starting address of the bitmap in graphics RAM

			Ft_App_WrCoCmd_Buffer(&host,TAG(13));
			Ft_App_WrCoCmd_Buffer(&host, BITMAP_SOURCE(0L));
			p_bmhdr = (Ft_Bitmap_header_t *)&IMG_Icon_settings_header[0];
			Ft_App_WrCoCmd_Buffer(&host,BITMAP_LAYOUT(p_bmhdr->Format, p_bmhdr->Stride, p_bmhdr->Height));
			Ft_App_WrCoCmd_Buffer(&host, BITMAP_SIZE(NEAREST, BORDER, BORDER, p_bmhdr->Width, p_bmhdr->Height));
			Ft_App_WrCoCmd_Buffer(&host,BEGIN(BITMAPS));
			Ft_App_WrCoCmd_Buffer(&host,VERTEX2II(200, 100, 0, 0));
			// icon back

			Ft_App_WrCoCmd_Buffer(&host,TAG(14));
			Ft_App_WrCoCmd_Buffer(&host, BITMAP_SOURCE(IMG_Icon_settings_header->Address));
			p_bmhdr = (Ft_Bitmap_header_t *)&IMG_Icon_back_header[0];
			Ft_App_WrCoCmd_Buffer(&host,BITMAP_LAYOUT(p_bmhdr->Format, p_bmhdr->Stride, p_bmhdr->Height));
			Ft_App_WrCoCmd_Buffer(&host, BITMAP_SIZE(NEAREST, BORDER, BORDER, p_bmhdr->Width, p_bmhdr->Height));
			Ft_App_WrCoCmd_Buffer(&host,BEGIN(BITMAPS));
			Ft_App_WrCoCmd_Buffer(&host,VERTEX2II(250, 100, 0, 0));

			Ft_App_WrCoCmd_Buffer(&host, BITMAP_SOURCE(IMG_Phial_header->Address));
			p_bmhdr = (Ft_Bitmap_header_t *)&IMG_Phial_header[0];
			Ft_App_WrCoCmd_Buffer(&host,BITMAP_LAYOUT(p_bmhdr->Format, p_bmhdr->Stride, p_bmhdr->Height));
			Ft_App_WrCoCmd_Buffer(&host, BITMAP_SIZE(NEAREST, BORDER, BORDER, p_bmhdr->Width, p_bmhdr->Height));
			Ft_App_WrCoCmd_Buffer(&host,BEGIN(BITMAPS));
			Ft_App_WrCoCmd_Buffer(&host,VERTEX2II(300, 100, 0, 0));

			uint8_t level = map(value, 0, 65535, 0, 100);
			if(level < 30)
			{
				Ft_App_WrCoCmd_Buffer(&host, COLOR_RGB(0xFF, 0x00, 0x00));
			}
			else if(level < 45)
			{
				Ft_App_WrCoCmd_Buffer(&host, COLOR_RGB(0xFF, 0xA5, 0x00));
			}
			else
			{
				Ft_App_WrCoCmd_Buffer(&host, COLOR_RGB(0x00, 0xFF, 0x00));
			}
			Ft_App_WrCoCmd_Buffer(&host, SCISSOR_XY(317, 109 + map(value, 0, 65535, 90, 0)));
			Ft_App_WrCoCmd_Buffer(&host, SCISSOR_SIZE(16, 90 - map(value, 0, 65535, 90, 0)));
			Ft_App_WrCoCmd_Buffer(&host, LINE_WIDTH(8 * 16));
			Ft_App_WrCoCmd_Buffer(&host, BEGIN(LINES));
			Ft_App_WrCoCmd_Buffer(&host, VERTEX2F(325 * 16, 105 * 16));
			Ft_App_WrCoCmd_Buffer(&host, VERTEX2F(325 * 16, 186 * 16));



			touch_tag = Ft_Gpu_Hal_Rd8(&host,REG_TOUCH_TAG);
			if(touch_tag == 11)
			{
				Ft_App_WrCoCmd_Buffer(&host,COLOR_RGB(0x00,0x80,0x00));
				Ft_Gpu_CoCmd_Text(&host,(LCD_WIDTH/2), LCD_HEIGHT*2/5, 28, OPT_CENTERX, "Tu jesteœ");
			}

			if(touch_tag == 12)
			{
				menu_actual = menu2;
				Ft_Gpu_CoCmd_Track(&host, 50, 240, 400, 20, 9);
				Ft_Gpu_CoCmd_Track(&host, 50, 240, 0, 0, 10);
				while(touch_tag == 12)touch_tag = Ft_Gpu_Hal_Rd8(&host,REG_TOUCH_TAG);;
			}

			if(touch_tag == 13)
			{
				Ft_App_WrCoCmd_Buffer(&host,COLOR_RGB(0x00,0x80,0x00));
				Ft_Gpu_CoCmd_Text(&host,(LCD_WIDTH/2), LCD_HEIGHT*2/5, 28, OPT_CENTERX, "Ustawienia");
			}

			if(touch_tag == 14)
			{
				Ft_App_WrCoCmd_Buffer(&host,COLOR_RGB(0x00,0x80,0x00));
				Ft_Gpu_CoCmd_Text(&host,(LCD_WIDTH/2), LCD_HEIGHT*2/5, 28, OPT_CENTERX, "Back");
			}

			Ft_App_WrCoCmd_Buffer(&host,DISPLAY());
			Ft_Gpu_CoCmd_Swap(&host);
			/* Download the commands into fifo */
			Ft_App_Flush_Co_Buffer(&host);

			/* Wait till coprocessor completes the operation */
			Ft_Gpu_Hal_WaitCmdfifo_empty(&host);
}

void menu2()
{
	Ft_Gpu_CoCmd_Dlstart(&host);

			/*Setting first screen*/
			Ft_App_WrCoCmd_Buffer(&host,CLEAR_COLOR_RGB(0xff,0xff,0xff));//white color
			Ft_App_WrCoCmd_Buffer(&host,CLEAR(1,1,1));//clear screen

			track_data = Ft_Gpu_Hal_Rd32(&host, REG_TRACKER);
			if((track_data & 0xff) == 9)
			{
				value2 = (track_data >> 16);
				brightness = map(value2, 0, 65535, 1, 128);
				Ft_Gpu_Hal_Wr32(&host, REG_PWM_DUTY, brightness);
			}

			Ft_App_WrCoCmd_Buffer(&host,COLOR_RGB(0xF0,0xFF,0x20));//red color
			Ft_Gpu_CoCmd_FgColor(&host, COLOR_RGB(0x00, 0x76, 0x70));
			Ft_App_WrCoCmd_Buffer(&host,TAG(9));
			Ft_Gpu_CoCmd_Slider(&host, 50, 240, 400, 20, 0, brightness, 128);

			Ft_App_WrCoCmd_Buffer(&host,COLOR_RGB(0x01,0x00,0x00));
			sprintf(bufor, "ID: %u", track_data & 0xFF);
			Ft_Gpu_CoCmd_Text(&host, 10, 180, 28, 0, bufor);
			sprintf(bufor, "Val: %u", value2);
			Ft_Gpu_CoCmd_Text(&host, 10, 200, 28, 0, bufor);
			sprintf(bufor, "Brightness: %u", brightness);
			Ft_Gpu_CoCmd_Text(&host, 10, 160, 28, 0, bufor);

			sprintf(bufor, "Dystans: %.1f cm", distance);
			Ft_Gpu_CoCmd_Text(&host, 10, 120, 28, 0, bufor);

			sprintf(bufor, "Temperatura: %.1f *C", temperature);
			Ft_Gpu_CoCmd_Text(&host, 10, 100, 28, 0, bufor);

			// BUTTON TEST
			Ft_App_WrCoCmd_Buffer(&host,TAG(11));
			Ft_App_WrCoCmd_Buffer(&host,COLOR_RGB(0xFF,0xFF,0xFF));
			Ft_Gpu_CoCmd_GradColor(&host, COLOR_RGB(0xFF, 0xFF, 0xFF));
			Ft_Gpu_CoCmd_FgColor(&host, COLOR_RGB(0x70, 0x41, 0x00));
			Ft_Gpu_CoCmd_Button(&host, 31,10, 160,60, 27,0, "Menu1");

			Ft_App_WrCoCmd_Buffer(&host,TAG(12));
			Ft_Gpu_CoCmd_GradColor(&host, COLOR_RGB(0xFF, 0xFF, 0xFF));
			Ft_Gpu_CoCmd_FgColor(&host, COLOR_RGB(0x00, 0x70, 0x12));
			Ft_Gpu_CoCmd_Button(&host, 275,10, 160,60, 27,0, "Brightness");
			//

			touch_tag = Ft_Gpu_Hal_Rd8(&host,REG_TOUCH_TAG);
			if(touch_tag == 11)
			{
				menu_actual = menu1;
				Ft_Gpu_CoCmd_Track(&host, 50, 240, 400, 20, 10);
				Ft_Gpu_CoCmd_Track(&host, 50, 240, 0, 0, 9);
				while(touch_tag == 11)touch_tag = Ft_Gpu_Hal_Rd8(&host,REG_TOUCH_TAG);;
			}

			if(touch_tag == 12)
			{
				Ft_App_WrCoCmd_Buffer(&host,COLOR_RGB(0x00,0x80,0x00));
				Ft_Gpu_CoCmd_Text(&host,(LCD_WIDTH/2), LCD_HEIGHT*2/5, 28, OPT_CENTERX, "Tu jesteœ");
			}

			Ft_App_WrCoCmd_Buffer(&host,DISPLAY());
			Ft_Gpu_CoCmd_Swap(&host);
			/* Download the commands into fifo */
			Ft_App_Flush_Co_Buffer(&host);

			/* Wait till coprocessor completes the operation */
			Ft_Gpu_Hal_WaitCmdfifo_empty(&host);
}

void test_screen(void)
{
	uint8_t touch_tag = 0;

	Ft_Gpu_CoCmd_Dlstart(&host);
	/*Setting first screen*/
	Ft_App_WrCoCmd_Buffer(&host,CLEAR_COLOR_RGB(0xff,0xff,0xff));//white color
	Ft_App_WrCoCmd_Buffer(&host,CLEAR(1,1,1));//clear screen

	Ft_App_WrCoCmd_Buffer(&host,COLOR_RGB(0x80,0x00,0x00));//red color
	Ft_Gpu_CoCmd_Text(&host,(LCD_WIDTH/2), LCD_HEIGHT/5, 28, OPT_CENTERX, "Touch test");

	Ft_App_WrCoCmd_Buffer(&host,COLOR_RGB(0xff,0xff,0xff));
	Ft_App_WrCoCmd_Buffer(&host,TAG(249));
	Ft_Gpu_CoCmd_Button(&host,LCD_WIDTH/2-50,LCD_HEIGHT/5*4,100,50,20,0, "Push me");

	touch_tag = Ft_Gpu_Hal_Rd8(&host,REG_TOUCH_TAG);
	if(touch_tag == 249)
	{
		Ft_App_WrCoCmd_Buffer(&host,COLOR_RGB(0x00,0x80,0x00));
		Ft_Gpu_CoCmd_Text(&host,(LCD_WIDTH/2), LCD_HEIGHT*2/5, 28, OPT_CENTERX, "TOUCHED");
	}

	Ft_App_WrCoCmd_Buffer(&host,DISPLAY());
	Ft_Gpu_CoCmd_Swap(&host);
	/* Download the commands into fifo */
	Ft_App_Flush_Co_Buffer(&host);

	/* Wait till coprocessor completes the operation */
	Ft_Gpu_Hal_WaitCmdfifo_empty(&host);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
