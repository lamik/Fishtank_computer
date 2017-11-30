#include "main.h"
#include "stm32f4xx_hal.h"
#include "spi.h"
#include "gpio.h"
#include <string.h>

#include "FT_Platform.h"

/* API to initialize the SPI interface */
ft_bool_t Ft_Gpu_Hal_Init( void )
{
	// PD, CS was inited in HAL
	// MX_GPIO_Init();

	HAL_GPIO_WritePin(FT800_PD_GPIO_Port, FT800_PD_Pin, GPIO_PIN_SET); 		// Initial state of PD_N - high
	HAL_GPIO_WritePin(FT800_CS_GPIO_Port, FT800_CS_Pin, GPIO_PIN_SET);

	return TRUE;
}

ft_bool_t Ft_Gpu_Hal_Open(Ft_Gpu_Hal_Context_t *host, SPI_HandleTypeDef *spi_handler )
{
	// SPI init with HAL

	// MX_SPI1_Init();
	host->hspi = spi_handler;

	return TRUE;
}

ft_bool_t Ft_Gpu_Hal_Init_43CTP(Ft_Gpu_Hal_Context_t *host)
{
	  Ft_Gpu_Hal_Init();
	  Ft_Gpu_Hal_Open(host, &hspi1);

	  /*Set Power Down Pin UP*/
	  Ft_Gpu_Hal_Powercycle(FT_TRUE);
		/* Access address 0 to wake up the FT800 */
		Ft_Gpu_HostCommand(host, FT_GPU_ACTIVE_M);
		Ft_Gpu_Hal_Sleep(20);
		/* Set the clk to external clock */
		Ft_Gpu_HostCommand(host, FT_GPU_INTERNAL_OSC);
		Ft_Gpu_Hal_Sleep(10);
		/* Switch PLL output to 48MHz */
		Ft_Gpu_HostCommand(host, FT_GPU_PLL_48M);
		Ft_Gpu_Hal_Sleep(10);
		/* Do a core reset for safer side */
		Ft_Gpu_HostCommand(host, FT_GPU_CORE_RESET);
		Ft_Gpu_Hal_Sleep(10);


		//Read Register ID to check if FT800 or FT801 is ready.
		uint8_t chipid = Ft_Gpu_Hal_Rd8(host, REG_ID);
		while(chipid != 0x7C)
		chipid = Ft_Gpu_Hal_Rd8(host, REG_ID);

		Ft_Gpu_Hal_Wr8(host, REG_GPIO_DIR,0x83 | Ft_Gpu_Hal_Rd8(host,REG_GPIO_DIR));
		Ft_Gpu_Hal_Wr8(host, REG_GPIO,0x083 | Ft_Gpu_Hal_Rd8(host,REG_GPIO));

		Ft_Gpu_Hal_Wr8(host, REG_GPIO,0x080 | Ft_Gpu_Hal_Rd8(host,REG_GPIO));

		/*Set requested registers*/
		Ft_Gpu_Hal_Wr16(host, REG_HCYCLE, LCD_H_CYCLE);
		Ft_Gpu_Hal_Wr16(host, REG_HOFFSET, LCD_H_OFFSET);
		Ft_Gpu_Hal_Wr16(host, REG_HSYNC0, LCD_H_SYNC0);
		Ft_Gpu_Hal_Wr16(host, REG_HSYNC1, LCD_H_SYNC1);
		Ft_Gpu_Hal_Wr16(host, REG_VCYCLE, LCD_V_CYCLE);
		Ft_Gpu_Hal_Wr16(host, REG_VOFFSET, LCD_V_OFFSET);
		Ft_Gpu_Hal_Wr16(host, REG_VSYNC0, LCD_V_SYNC0);
		Ft_Gpu_Hal_Wr16(host, REG_VSYNC1, LCD_V_SYNC1);
		Ft_Gpu_Hal_Wr8(host, REG_SWIZZLE, LCD_SWIZZLE);
		Ft_Gpu_Hal_Wr8(host, REG_PCLK_POL, LCD_PCLK_POL);
		Ft_Gpu_Hal_Wr8(host, REG_PCLK,LCD_PCLK);//after this display is visible on the LCD

		Ft_Gpu_Hal_Wr16(host, REG_HSIZE, LCD_WIDTH);
		Ft_Gpu_Hal_Wr16(host, REG_VSIZE, LCD_HEIGHT);

		Ft_Gpu_Hal_Wr8(host, REG_GPIO_DIR,0x83 | Ft_Gpu_Hal_Rd8(host,REG_GPIO_DIR));
		Ft_Gpu_Hal_Wr8(host, REG_GPIO,0x083 | Ft_Gpu_Hal_Rd8(host,REG_GPIO));
		Ft_Gpu_Hal_Wr16(host, REG_TOUCH_RZTHRESH,1200);
		Ft_Gpu_Hal_Wr8(host, REG_ROTATE, 0);//turn off rotation

		Ft_Gpu_Hal_Wr32(host,REG_TOUCH_TRANSFORM_A,0x0000640F);
		Ft_Gpu_Hal_Wr32(host,REG_TOUCH_TRANSFORM_B,0xFFFFFc25);
		Ft_Gpu_Hal_Wr32(host,REG_TOUCH_TRANSFORM_C,0xFFF73909);
		Ft_Gpu_Hal_Wr32(host,REG_TOUCH_TRANSFORM_D,0x0000014B);
		Ft_Gpu_Hal_Wr32(host,REG_TOUCH_TRANSFORM_E,0x00006125);
		Ft_Gpu_Hal_Wr32(host,REG_TOUCH_TRANSFORM_F,0xFFF56FDB);

		return TRUE;
}

ft_void_t Ft_Gpu_Hal_Close(Ft_Gpu_Hal_Context_t *host)
{
	host->status = FT_GPU_HAL_CLOSED;

	HAL_SPI_DeInit(&hspi1);

}

ft_void_t Ft_Gpu_Hal_DeInit()
{

}

/*The APIs for reading/writing transfer continuously only with small buffer system*/
ft_void_t  Ft_Gpu_Hal_StartTransfer(Ft_Gpu_Hal_Context_t *host,FT_GPU_TRANSFERDIR_T rw,ft_uint32_t addr)
{
	if (FT_GPU_READ == rw){

		uint8_t cTempAddr[4];												// FT800 Memory Address										// Dummy byte


		cTempAddr[3] = (char) (addr >> 16);		// Compose the command and address to send
		cTempAddr[2] = (char) ((addr & 0xFF00) >> 8);								// middle byte
		cTempAddr[1] = (char) ((addr & 0xFF));		// low byte
		cTempAddr[0] = 0x00;

		HAL_GPIO_WritePin(FT800_CS_GPIO_Port, FT800_CS_Pin, GPIO_PIN_RESET);

		for (int i = 3; i >= 1; i--)
		{
			HAL_SPI_Transmit(host->hspi, &cTempAddr[i], 1, 1); 			// Send Memory Write plus high address byte
		}

		HAL_SPI_Transmit(host->hspi, &cTempAddr[0], 1, 1);

		host->status = FT_GPU_HAL_READING;
	}else{


		uint8_t cTempAddr[3];

		cTempAddr[2] = (char) ((addr >> 16) & 0xBF )| MEM_WRITE;	// Compose the command and address to send
		cTempAddr[1] = (char) ((addr & 0xFF00) >> 8);			// middle byte
		cTempAddr[0] = (char) ((addr & 0xFF));		// low byte

		HAL_GPIO_WritePin(FT800_CS_GPIO_Port, FT800_CS_Pin, GPIO_PIN_RESET);

		for (int i = 2; i >= 0; i--)
		{
			HAL_SPI_Transmit(host->hspi, &cTempAddr[i], 1, 1); 			// Send Memory Write plus high address byte
		}
		host->status = FT_GPU_HAL_WRITING;
	}
}



/*The APIs for writing transfer continuously only*/
ft_void_t  Ft_Gpu_Hal_StartCmdTransfer(Ft_Gpu_Hal_Context_t *host,FT_GPU_TRANSFERDIR_T rw, ft_uint16_t count)
{
	Ft_Gpu_Hal_StartTransfer(host,rw,host->ft_cmd_fifo_wp + RAM_CMD);
}

ft_uint8_t    Ft_Gpu_Hal_TransferString(Ft_Gpu_Hal_Context_t *host,const ft_char8_t *string)
{
    ft_uint16_t length = strlen(string);
    while(length --){
       Ft_Gpu_Hal_Transfer8(host,*string);
       string ++;
    }
    //Append one null as ending flag
    Ft_Gpu_Hal_Transfer8(host,0);
}


ft_uint8_t    Ft_Gpu_Hal_Transfer8(Ft_Gpu_Hal_Context_t *host,ft_uint8_t value)
{
	uint8_t retVal = 0;
	HAL_SPI_TransmitReceive(host->hspi, &value, &retVal, 1, 10);
	return retVal;
}


ft_uint16_t  Ft_Gpu_Hal_Transfer16(Ft_Gpu_Hal_Context_t *host,ft_uint16_t value)
{
	ft_uint8_t retVal[2];

	HAL_SPI_TransmitReceive(host->hspi, &value, retVal, 2, 10);

	return ((retVal[1] << 8) | retVal[0]);
}
ft_uint32_t  Ft_Gpu_Hal_Transfer32(Ft_Gpu_Hal_Context_t *host,ft_uint32_t value)
{
	ft_uint8_t retVal[4] ;

	HAL_SPI_TransmitReceive(host->hspi, &value, retVal, 4, 10);

	return ((retVal[3] << 24) | (retVal[2] << 16) | (retVal[1] << 8) | retVal[0]);
}

ft_void_t   Ft_Gpu_Hal_EndTransfer(Ft_Gpu_Hal_Context_t *host)
{

	HAL_GPIO_WritePin(FT800_CS_GPIO_Port, FT800_CS_Pin, GPIO_PIN_SET);	// Set chip select low

	host->status = FT_GPU_HAL_OPENED;
}


ft_uint8_t  Ft_Gpu_Hal_Rd8(Ft_Gpu_Hal_Context_t *host,ft_uint32_t addr)
{
	ft_uint8_t value;
	Ft_Gpu_Hal_StartTransfer(host,FT_GPU_READ,addr);
	value = Ft_Gpu_Hal_Transfer8(host,0);
	Ft_Gpu_Hal_EndTransfer(host);
	return value;
}
ft_uint16_t Ft_Gpu_Hal_Rd16(Ft_Gpu_Hal_Context_t *host,ft_uint32_t addr)
{
	ft_uint16_t value;
	Ft_Gpu_Hal_StartTransfer(host,FT_GPU_READ,addr);
	value = Ft_Gpu_Hal_Transfer16(host,0);
	Ft_Gpu_Hal_EndTransfer(host);
	return value;
}
ft_uint32_t Ft_Gpu_Hal_Rd32(Ft_Gpu_Hal_Context_t *host,ft_uint32_t addr)
{
	ft_uint32_t value;
	Ft_Gpu_Hal_StartTransfer(host,FT_GPU_READ,addr);
	value = Ft_Gpu_Hal_Transfer32(host,0);
	Ft_Gpu_Hal_EndTransfer(host);
	return value;
}

ft_void_t Ft_Gpu_Hal_Wr8(Ft_Gpu_Hal_Context_t *host,ft_uint32_t addr, ft_uint8_t v)
{	
	Ft_Gpu_Hal_StartTransfer(host,FT_GPU_WRITE,addr);
	Ft_Gpu_Hal_Transfer8(host,v);
	Ft_Gpu_Hal_EndTransfer(host);
}
ft_void_t Ft_Gpu_Hal_Wr16(Ft_Gpu_Hal_Context_t *host,ft_uint32_t addr, ft_uint16_t v)
{
	Ft_Gpu_Hal_StartTransfer(host,FT_GPU_WRITE,addr);
	Ft_Gpu_Hal_Transfer16(host,v);
	Ft_Gpu_Hal_EndTransfer(host);
}
ft_void_t Ft_Gpu_Hal_Wr32(Ft_Gpu_Hal_Context_t *host,ft_uint32_t addr, ft_uint32_t v)
{
	Ft_Gpu_Hal_StartTransfer(host,FT_GPU_WRITE,addr);
	Ft_Gpu_Hal_Transfer32(host,v);
	Ft_Gpu_Hal_EndTransfer(host);
}

ft_void_t Ft_Gpu_HostCommand(Ft_Gpu_Hal_Context_t *host, ft_uint8_t cmd)
{

	uint8_t cTemp[3];														// FT800 Memory Address

	cTemp[0] = cmd;	// Compose the command and address to send
	cTemp[1] = 0x00;								// middle byte
	cTemp[2] = 0x00;

	HAL_GPIO_WritePin(FT800_CS_GPIO_Port, FT800_CS_Pin, GPIO_PIN_RESET);	// Set chip select low

	  for (int i = 2; i >= 0; i--)
		{
			HAL_SPI_Transmit(host->hspi, &cTemp[i], 1, 0); 			// Send Memory Write plus high address byte
		}
		HAL_GPIO_WritePin(FT800_CS_GPIO_Port, FT800_CS_Pin, GPIO_PIN_SET);	// Set chip select low

}

ft_void_t Ft_Gpu_Hal_Updatecmdfifo(Ft_Gpu_Hal_Context_t *host,ft_uint16_t count)
{
	host->ft_cmd_fifo_wp  = (host->ft_cmd_fifo_wp + count) & 4095;

	//4 byte alignment
	host->ft_cmd_fifo_wp = (host->ft_cmd_fifo_wp + 3) & 0xffc;
	Ft_Gpu_Hal_Wr16(host,REG_CMD_WRITE,host->ft_cmd_fifo_wp);
}


ft_uint16_t Ft_Gpu_Cmdfifo_Freespace(Ft_Gpu_Hal_Context_t *host)
{
	ft_uint16_t fullness,retval;

	fullness = (host->ft_cmd_fifo_wp - Ft_Gpu_Hal_Rd16(host,REG_CMD_READ)) & 4095;
	retval = (FT_CMD_FIFO_SIZE - 4) - fullness;
	return (retval);
}

ft_void_t Ft_Gpu_Hal_WrCmdBuf(Ft_Gpu_Hal_Context_t *host,ft_uint8_t *buffer,ft_uint16_t count)
{
	ft_uint32_t length =0, SizeTransfered = 0;   

#define MAX_CMD_FIFO_TRANSFER   Ft_Gpu_Cmdfifo_Freespace(host)  
	do {                
		length = count;
		if (length > MAX_CMD_FIFO_TRANSFER){
		    length = MAX_CMD_FIFO_TRANSFER;
		}
      	        Ft_Gpu_Hal_CheckCmdBuffer(host,length);

                Ft_Gpu_Hal_StartCmdTransfer(host,FT_GPU_WRITE,length);

                SizeTransfered = 0;
		while (length--) {
                    Ft_Gpu_Hal_Transfer8(host,*buffer);
		    buffer++;
                    SizeTransfered ++;
		}
                length = SizeTransfered;


		Ft_Gpu_Hal_EndTransfer(host);
		Ft_Gpu_Hal_Updatecmdfifo(host,length);

		Ft_Gpu_Hal_WaitCmdfifo_empty(host);

		count -= length;
	}while (count > 0);
}

ft_void_t Ft_Gpu_Hal_WrCmdBufFromFlash(Ft_Gpu_Hal_Context_t *host,FT_PROGMEM ft_prog_uchar8_t *buffer,ft_uint16_t count)
{
	ft_uint32_t length =0, SizeTransfered = 0;   

#define MAX_CMD_FIFO_TRANSFER   Ft_Gpu_Cmdfifo_Freespace(host)  
	do {                
		length = count;
		if (length > MAX_CMD_FIFO_TRANSFER){
		    length = MAX_CMD_FIFO_TRANSFER;
		}
      	        Ft_Gpu_Hal_CheckCmdBuffer(host,length);

                Ft_Gpu_Hal_StartCmdTransfer(host,FT_GPU_WRITE,length);


                SizeTransfered = 0;
		while (length--) {
                    Ft_Gpu_Hal_Transfer8(host,ft_pgm_read_byte_near(buffer));
		    buffer++;
                    SizeTransfered ++;
		}
                length = SizeTransfered;

    	        Ft_Gpu_Hal_EndTransfer(host);
		Ft_Gpu_Hal_Updatecmdfifo(host,length);

		Ft_Gpu_Hal_WaitCmdfifo_empty(host);

		count -= length;
	}while (count > 0);
}


ft_void_t Ft_Gpu_Hal_CheckCmdBuffer(Ft_Gpu_Hal_Context_t *host,ft_uint16_t count)
{
   ft_uint16_t getfreespace;
   do{
        getfreespace = Ft_Gpu_Cmdfifo_Freespace(host);
   }while(getfreespace < count);
}
ft_void_t Ft_Gpu_Hal_WaitCmdfifo_empty(Ft_Gpu_Hal_Context_t *host)
{
   while(Ft_Gpu_Hal_Rd16(host,REG_CMD_READ) != Ft_Gpu_Hal_Rd16(host,REG_CMD_WRITE));
   
   host->ft_cmd_fifo_wp = Ft_Gpu_Hal_Rd16(host,REG_CMD_WRITE);
}

ft_void_t Ft_Gpu_Hal_WaitLogo_Finish(Ft_Gpu_Hal_Context_t *host)
{
    ft_int16_t cmdrdptr,cmdwrptr;

    do{
         cmdrdptr = Ft_Gpu_Hal_Rd16(host,REG_CMD_READ);
         cmdwrptr = Ft_Gpu_Hal_Rd16(host,REG_CMD_WRITE);
    }while ((cmdwrptr != cmdrdptr) || (cmdrdptr != 0));
    host->ft_cmd_fifo_wp = 0;
}


ft_void_t Ft_Gpu_Hal_ResetCmdFifo(Ft_Gpu_Hal_Context_t *host)
{
   host->ft_cmd_fifo_wp = 0;
}


ft_void_t Ft_Gpu_Hal_WrCmd32(Ft_Gpu_Hal_Context_t *host,ft_uint32_t cmd)
{
         Ft_Gpu_Hal_CheckCmdBuffer(host,sizeof(cmd));
      
         Ft_Gpu_Hal_Wr32(host,RAM_CMD + host->ft_cmd_fifo_wp,cmd);
      
         Ft_Gpu_Hal_Updatecmdfifo(host,sizeof(cmd));
}


ft_void_t Ft_Gpu_Hal_ResetDLBuffer(Ft_Gpu_Hal_Context_t *host)
{
           host->ft_dl_buff_wp = 0;
}
/* Toggle PD_N pin of FT800 board for a power cycle*/
ft_void_t Ft_Gpu_Hal_Powercycle(ft_bool_t up)
{
	if (up)
	{
		HAL_GPIO_WritePin(FT800_PD_GPIO_Port, FT800_PD_Pin, GPIO_PIN_RESET);		// Set chip select high
        Ft_Gpu_Hal_Sleep(20);
		HAL_GPIO_WritePin(FT800_PD_GPIO_Port, FT800_PD_Pin, GPIO_PIN_SET);		// Set chip select high
        Ft_Gpu_Hal_Sleep(20);

	}else
	{
		HAL_GPIO_WritePin(FT800_PD_GPIO_Port, FT800_PD_Pin, GPIO_PIN_SET);		// Set chip select high
        Ft_Gpu_Hal_Sleep(20);
    	HAL_GPIO_WritePin(FT800_PD_GPIO_Port, FT800_PD_Pin, GPIO_PIN_RESET);		// Set chip select high
        Ft_Gpu_Hal_Sleep(20);
	}
}
ft_void_t Ft_Gpu_Hal_WrMemFromFlash(Ft_Gpu_Hal_Context_t *host,ft_uint32_t addr,const ft_prog_uchar8_t *buffer, ft_uint32_t length)
{
	Ft_Gpu_Hal_StartTransfer(host,FT_GPU_WRITE,addr);

	while (length--) {
            Ft_Gpu_Hal_Transfer8(host,ft_pgm_read_byte_near(buffer));
	    buffer++;
	}


	Ft_Gpu_Hal_EndTransfer(host);
}

ft_void_t Ft_Gpu_Hal_WrMem(Ft_Gpu_Hal_Context_t *host,ft_uint32_t addr,const ft_uint8_t *buffer, ft_uint32_t length)
{
	Ft_Gpu_Hal_StartTransfer(host,FT_GPU_WRITE,addr);

	while (length--) {
            Ft_Gpu_Hal_Transfer8(host,*buffer);
	    buffer++;
	}

	Ft_Gpu_Hal_EndTransfer(host);
}


ft_void_t Ft_Gpu_Hal_RdMem(Ft_Gpu_Hal_Context_t *host,ft_uint32_t addr, ft_uint8_t *buffer, ft_uint32_t length)
{
	Ft_Gpu_Hal_StartTransfer(host,FT_GPU_READ,addr);

	while (length--) {
	   *buffer = Ft_Gpu_Hal_Transfer8(host,0);
	   buffer++;
	}
	Ft_Gpu_Hal_EndTransfer(host);
}

ft_int32_t Ft_Gpu_Hal_Dec2Ascii(ft_char8_t *pSrc,ft_int32_t value)
{
	ft_int16_t Length;
	ft_char8_t *pdst,charval;
	ft_int32_t CurrVal = value,tmpval,i;
	ft_char8_t tmparray[16],idx = 0;

	Length = strlen(pSrc);
	pdst = pSrc + Length;

	if(0 == value)
	{
		*pdst++ = '0';
		*pdst++ = '\0';
		return 0;
	}

	if(CurrVal < 0)
	{
		*pdst++ = '-';
		CurrVal = - CurrVal;
	}
	/* insert the value */
	while(CurrVal > 0){
		tmpval = CurrVal;
		CurrVal /= 10;
		tmpval = tmpval - CurrVal*10;
		charval = '0' + tmpval;
		tmparray[idx++] = charval;
	}

	for(i=0;i<idx;i++)
	{
		*pdst++ = tmparray[idx - i - 1];
	}
	*pdst++ = '\0';

	return 0;
}



ft_void_t Ft_Gpu_Hal_Sleep(ft_uint16_t ms)
{
	HAL_Delay(ms);
}

void delay(uint32_t ms)
{
	HAL_Delay(ms);
}
