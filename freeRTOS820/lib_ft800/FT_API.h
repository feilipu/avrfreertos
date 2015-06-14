#ifndef _FT_API_H_
#define _FT_API_H_

/* FT API structure definitions */
typedef struct FT_API_Bitmap_header
{
	ft_uint8_t Format;
	ft_int16_t Width;
	ft_int16_t Height;
	ft_int16_t Stride;
	ft_int32_t ArrayOffset;
}FT_API_Bitmap_header_t;

ft_void_t	FT_API_Boot_Config(ft_void_t);	// you must do this first.

ft_void_t	FT_API_Touch_Config(ft_void_t);	// you must do this before using touch.

ft_void_t	FT_API_Write_CoCmd(ft_const_uint32_t cmd) __attribute__ ((flatten)); // write a co-processor command

ft_void_t	FT_API_Write_DLCmd(ft_const_uint32_t cmd) __attribute__ ((flatten)); // write a Download List (DL) command

ft_void_t	FT_API_Reset_DLBuffer(ft_void_t) __attribute__ ((flatten)); // reset (0) the Download List (DL) command offset

/* API to check the status of previous DLSWAP and perform DLSWAP of new DL */
/* Check for the status of previous DLSWAP and if still not done wait for few ms and check again */
ft_void_t	FT_API_GPU_DLSwap(const ft_uint8_t DL_Swap_Type);

/* API to wait until the command buffer is empty, following CMD_SWAP */
ft_void_t	FT_API_WaitCmdfifo_empty(ft_void_t) __attribute__ ((flatten));

/********** utilities ********************/

ft_void_t	FT_API_fadeout(ft_void_t);
ft_void_t	FT_API_fadein(ft_void_t);
ft_int16_t	FT_API_qsin(ft_uint16_t a);
ft_int16_t	FT_API_qcos(ft_uint16_t a);
ft_uint16_t FT_API_qatan(int16_t y, int16_t x);

#endif /* _FT_API_H_ */

/* Nothing beyond this */








