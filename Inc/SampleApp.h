#ifndef _SAMPLEAPP_H_
#define _SAMPLEAPP_H_

/* sample app structure definitions */
typedef struct Ft_Bitmap_header
{
	ft_uint8_t Format;
	ft_int16_t Width;
	ft_int16_t Height;
	ft_int16_t Stride;
	ft_int32_t Arrayoffset;
	uint32_t Address;
}Ft_Bitmap_header_t;

ft_void_t Ft_App_WrCoCmd_Buffer(Ft_Gpu_Hal_Context_t *phost,ft_uint32_t cmd);
ft_void_t Ft_App_WrDlCmd_Buffer(Ft_Gpu_Hal_Context_t *phost,ft_uint32_t cmd);
ft_void_t Ft_App_WrCoStr_Buffer(Ft_Gpu_Hal_Context_t *phost,const ft_char8_t *s);
ft_void_t Ft_App_Flush_DL_Buffer(Ft_Gpu_Hal_Context_t *phost);
ft_void_t Ft_App_Flush_Co_Buffer(Ft_Gpu_Hal_Context_t *phost);

#endif /* _SAMPLEAPP_H_ */

/* Nothing beyond this */









