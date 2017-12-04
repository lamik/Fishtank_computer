/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "crc.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "FT_Platform.h"
#include "onewire.h"
#include "ds18b20.h"
#include "hcsr04.h"
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

OneWire_t OW;
uint8_t DS_ROM[8];
float temperature;

float distance;
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
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_CRC_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
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

		distance = HCSR04_Read();

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
			Ft_App_WrCoCmd_Buffer(&host,CLEAR_COLOR_RGB(0xff,0xff,0xff));//white color
			Ft_App_WrCoCmd_Buffer(&host,CLEAR(1,1,1));//clear screen

			track_data = Ft_Gpu_Hal_Rd32(&host, REG_TRACKER);
			if((track_data & 0xff) == 10)
				value = (track_data >> 16);


			Ft_App_WrCoCmd_Buffer(&host,COLOR_RGB(0xF0,0xFF,0x20));//red color
			Ft_Gpu_CoCmd_FgColor(&host, COLOR_RGB(0x00, 0x06, 0x70));
			Ft_App_WrCoCmd_Buffer(&host,TAG(10));
			Ft_Gpu_CoCmd_Slider(&host, 50, 240, 400, 20, 0, value, 0xFFFFFF);

			Ft_App_WrCoCmd_Buffer(&host,COLOR_RGB(0x01,0x00,0x00));
			sprintf(bufor, "ID: %u", track_data & 0xFF);
			Ft_Gpu_CoCmd_Text(&host, 10, 180, 28, 0, bufor);
			sprintf(bufor, "Val: %u", value);
			Ft_Gpu_CoCmd_Text(&host, 10, 200, 28, 0, bufor);


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
