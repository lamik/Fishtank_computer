#ifndef _FT_PLATFORM_H_
#define _FT_PLATFORM_H_

#include "main.h"
#include "stm32f1xx_hal.h"
#include "spi.h"
#include "gpio.h"

#include <string.h>

#include "FT_DataTypes.h"
#include "FT_Gpu_Hal.h"
#include "FT_Gpu.h"
#include "FT_CoPro_Cmds.h"

#include "SampleApp.h"

#define LCD_WIDTH		480	// Active width of LCD display
#define LCD_HEIGHT		272 // Active height of LCD display
#define LCD_H_CYCLE		548 // Total number of clocks per line
#define LCD_H_OFFSET	43 	// Start of active line
#define LCD_H_SYNC0		0	// Start of horizontal sync pulse
#define LCD_H_SYNC1		41	// End of horizontal sync pulse
#define LCD_V_CYCLE		292	// Total number of lines per screen
#define LCD_V_OFFSET	12	// Start of active screen
#define LCD_V_SYNC0		0	// Start of vertical sync pulse
#define LCD_V_SYNC1		10	// End of vertical sync pulse
#define LCD_PCLK		3	// Pixel Clock
#define LCD_SWIZZLE		0	// Define RGB output pins
#define LCD_PCLK_POL 	1	// Define active edge of PCLK

//#define BUFFER_OPTIMIZATION

extern Ft_Gpu_Hal_Context_t host;

#endif /*_FT_PLATFORM_H_*/
/* Nothing beyond this*/




