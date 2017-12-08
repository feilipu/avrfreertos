
#include "FT_Platform.h"

ft_void_t FT_GPU_Copro_SendCmd(FT_GPU_HAL_Context_t *phost, ft_const_uint32_t cmd) __attribute__ ( ( hot, flatten ) );
ft_void_t FT_GPU_CoCmd_SendStr(FT_GPU_HAL_Context_t *phost, ft_const_char8_t *s) __attribute__ ( ( hot, flatten ) );
ft_void_t FT_GPU_CoCmd_SendStr_P(FT_GPU_HAL_Context_t *phost, ft_prog_char8_t *s) __attribute__ ( ( hot, flatten ) );
ft_void_t FT_GPU_CoCmd_StartFunc(FT_GPU_HAL_Context_t *phost, ft_const_uint16_t count) __attribute__ ( ( hot, flatten ) );
ft_void_t FT_GPU_CoCmd_EndFunc(FT_GPU_HAL_Context_t *phost, ft_const_uint16_t count) __attribute__ ( ( hot, flatten ) );

ft_void_t FT_GPU_Copro_SendCmd(FT_GPU_HAL_Context_t *phost, ft_const_uint32_t cmd)
{
	FT_GPU_HAL_TransferCmd(phost, cmd);
}

ft_void_t FT_GPU_CoCmd_SendStr(FT_GPU_HAL_Context_t *phost, ft_const_char8_t *s)
{
	FT_GPU_HAL_TransferString(phost, s);
}

ft_void_t FT_GPU_CoCmd_SendStr_P(FT_GPU_HAL_Context_t *phost, ft_prog_char8_t *s)
{
	FT_GPU_HAL_TransferString_P(phost, s);
}

ft_void_t FT_GPU_CoCmd_StartFunc(FT_GPU_HAL_Context_t *phost, ft_const_uint16_t count)
{
  FT_GPU_HAL_CheckCmdBuffer(phost, count);
  FT_GPU_HAL_StartCmdTransfer(phost, FT_GPU_WRITE );
}

ft_void_t FT_GPU_CoCmd_EndFunc(FT_GPU_HAL_Context_t *phost, ft_const_uint16_t count)
{
  FT_GPU_HAL_EndCmdTransfer(phost);
  FT_GPU_HAL_Updatecmdfifo(phost, count);
}

/************ Co-Processor Command Functions **************/

ft_void_t FT_GPU_CoCmd_Text(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t font, ft_uint16_t options, const ft_char8_t* s)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*3 + (ft_uint16_t)strlen((ft_const_char8_t *)s) + 1);
  FT_GPU_Copro_SendCmd(phost, CMD_TEXT);
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)y<<16)|(x&0xffff)));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)options<<16)|(font&0xffff)));
  FT_GPU_CoCmd_SendStr(phost, s);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*3 + (ft_uint16_t)strlen((ft_const_char8_t *)s) + 1));
}

ft_void_t FT_GPU_CoCmd_Text_P(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t font, ft_uint16_t options, ft_prog_char8_t* s)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*3 + (ft_uint16_t)strlen_P((ft_prog_char8_t *)s) + 1);
  FT_GPU_Copro_SendCmd(phost, CMD_TEXT);
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)y<<16)|(x&0xffff)));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)options<<16)|(font&0xffff)));
  FT_GPU_CoCmd_SendStr_P(phost, s);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*3 + (ft_uint16_t)strlen_P((ft_prog_char8_t *)s) + 1));
}

ft_void_t FT_GPU_CoCmd_Number(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t font, ft_uint16_t options, ft_int32_t n)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*4);
  FT_GPU_Copro_SendCmd(phost, CMD_NUMBER);
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)y<<16)|x));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)options<<16)|font));
  FT_GPU_Copro_SendCmd(phost, n);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*4));
}

ft_void_t FT_GPU_CoCmd_LoadIdentity(FT_GPU_HAL_Context_t *phost)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*1);
  FT_GPU_Copro_SendCmd(phost, CMD_LOADIDENTITY);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*1));
}

ft_void_t FT_GPU_CoCmd_Toggle(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t font, ft_uint16_t options, ft_uint16_t state, const ft_char8_t* s)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*4 + (ft_uint16_t)strlen((ft_const_char8_t *)s) + 1);
  FT_GPU_Copro_SendCmd(phost, CMD_TOGGLE);
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)y<<16)|x));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)font<<16)|w));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)state<<16)|options));
  FT_GPU_CoCmd_SendStr(phost, s);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*4 + (ft_uint16_t)strlen((ft_const_char8_t *)s) + 1));
}

ft_void_t FT_GPU_CoCmd_Toggle_P(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t font, ft_uint16_t options, ft_uint16_t state, ft_prog_char8_t* s)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*4 + (ft_uint16_t)strlen_P((ft_prog_char8_t *)s) + 1);
  FT_GPU_Copro_SendCmd(phost, CMD_TOGGLE);
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)y<<16)|x));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)font<<16)|w));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)state<<16)|options));
  FT_GPU_CoCmd_SendStr_P(phost, s);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*4 + (ft_uint16_t)strlen_P((ft_prog_char8_t *)s) + 1));
}

/* Error handling for val is not done, so better to always use range of 65535 in order that needle is drawn within display region */
ft_void_t FT_GPU_CoCmd_Gauge(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t r, ft_uint16_t options, ft_uint16_t major, ft_uint16_t minor, ft_uint16_t val, ft_uint16_t range)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*5);
  FT_GPU_Copro_SendCmd(phost, CMD_GAUGE);
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)y<<16)|x));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)options<<16)|r));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)minor<<16)|major));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)range<<16)|val));
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*5));
}

ft_void_t FT_GPU_CoCmd_RegRead(FT_GPU_HAL_Context_t *phost, ft_uint32_t ptr, ft_uint32_t result) // fixme there's no result returned...?
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*3);
  FT_GPU_Copro_SendCmd(phost, CMD_REGREAD);
  FT_GPU_Copro_SendCmd(phost, ptr);
  FT_GPU_Copro_SendCmd(phost, 0);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*3));
}

ft_void_t FT_GPU_CoCmd_GetProps(FT_GPU_HAL_Context_t *phost, ft_uint32_t ptr, ft_uint32_t w, ft_uint32_t h) // fixme not sure where the properties are returned..?
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*4);
  FT_GPU_Copro_SendCmd(phost, CMD_GETPROPS);
  FT_GPU_Copro_SendCmd(phost, ptr);
  FT_GPU_Copro_SendCmd(phost, w);
  FT_GPU_Copro_SendCmd(phost, h);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*4));
}

ft_void_t FT_GPU_CoCmd_Memcpy(FT_GPU_HAL_Context_t *phost, ft_uint32_t dest, ft_uint32_t src, ft_uint32_t num)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*4);
  FT_GPU_Copro_SendCmd(phost, CMD_MEMCPY);
  FT_GPU_Copro_SendCmd(phost, dest);
  FT_GPU_Copro_SendCmd(phost, src);
  FT_GPU_Copro_SendCmd(phost, num);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*4));
}

ft_void_t FT_GPU_CoCmd_Spinner(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_uint16_t style, ft_uint16_t scale)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*3);
  FT_GPU_Copro_SendCmd(phost, CMD_SPINNER);
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)y<<16)|x));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)scale<<16)|style));
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*3));
}

ft_void_t FT_GPU_CoCmd_BgColor(FT_GPU_HAL_Context_t *phost, ft_uint32_t c)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*2);
  FT_GPU_Copro_SendCmd(phost, CMD_BGCOLOR);
  FT_GPU_Copro_SendCmd(phost, c);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*2));
}

ft_void_t FT_GPU_CoCmd_Swap(FT_GPU_HAL_Context_t *phost)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*1);
  FT_GPU_Copro_SendCmd(phost, CMD_SWAP);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*1));
}

ft_void_t FT_GPU_CoCmd_Inflate(FT_GPU_HAL_Context_t *phost, ft_uint32_t ptr)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*2);
  FT_GPU_Copro_SendCmd(phost, CMD_INFLATE);
  FT_GPU_Copro_SendCmd(phost, ptr);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*2));
}

ft_void_t FT_GPU_CoCmd_Translate(FT_GPU_HAL_Context_t *phost, ft_int32_t tx, ft_int32_t ty)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*3);
  FT_GPU_Copro_SendCmd(phost, CMD_TRANSLATE);
  FT_GPU_Copro_SendCmd(phost, tx);
  FT_GPU_Copro_SendCmd(phost, ty);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*3));
}

ft_void_t FT_GPU_CoCmd_Stop(FT_GPU_HAL_Context_t *phost)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*1);
  FT_GPU_Copro_SendCmd(phost, CMD_STOP);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*1));
}

ft_void_t FT_GPU_CoCmd_Slider(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t h, ft_uint16_t options, ft_uint16_t val, ft_uint16_t range)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*5);
  FT_GPU_Copro_SendCmd(phost, CMD_SLIDER);
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)y<<16)|x));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)h<<16)|w));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)val<<16)|options));
  FT_GPU_Copro_SendCmd(phost, range);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*5));
}

ft_void_t FT_GPU_CoCmd_TouchTransform(FT_GPU_HAL_Context_t *phost, ft_int32_t x0, ft_int32_t y0, ft_int32_t x1, ft_int32_t y1, ft_int32_t x2, ft_int32_t y2, ft_int32_t tx0, ft_int32_t ty0, ft_int32_t tx1, ft_int32_t ty1, ft_int32_t tx2, ft_int32_t ty2, ft_uint16_t result)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*6*2+FT_CMD_SIZE*2);
  FT_GPU_Copro_SendCmd(phost, CMD_TOUCH_TRANSFORM);
  FT_GPU_Copro_SendCmd(phost, x0);
  FT_GPU_Copro_SendCmd(phost, y0);
  FT_GPU_Copro_SendCmd(phost, x1);
  FT_GPU_Copro_SendCmd(phost, y1);
  FT_GPU_Copro_SendCmd(phost, x2);
  FT_GPU_Copro_SendCmd(phost, y2);
  FT_GPU_Copro_SendCmd(phost, tx0);
  FT_GPU_Copro_SendCmd(phost, ty0);
  FT_GPU_Copro_SendCmd(phost, tx1);
  FT_GPU_Copro_SendCmd(phost, ty1);
  FT_GPU_Copro_SendCmd(phost, tx2);
  FT_GPU_Copro_SendCmd(phost, ty2);
  FT_GPU_Copro_SendCmd(phost, result);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*6*2+FT_CMD_SIZE*2));
}

ft_void_t FT_GPU_CoCmd_Interrupt(FT_GPU_HAL_Context_t *phost, ft_uint32_t ms)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*2);
  FT_GPU_Copro_SendCmd(phost, CMD_INTERRUPT);
  FT_GPU_Copro_SendCmd(phost, ms);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*2));
}

ft_void_t FT_GPU_CoCmd_FgColor(FT_GPU_HAL_Context_t *phost, ft_uint32_t c)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*2);
  FT_GPU_Copro_SendCmd(phost, CMD_FGCOLOR);
  FT_GPU_Copro_SendCmd(phost, c);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*2));
}

ft_void_t FT_GPU_CoCmd_Rotate(FT_GPU_HAL_Context_t *phost, ft_int32_t a)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*2);
  FT_GPU_Copro_SendCmd(phost, CMD_ROTATE);
  FT_GPU_Copro_SendCmd(phost, a);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*2));
}

ft_void_t FT_GPU_CoCmd_Button(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t h, ft_int16_t font, ft_uint16_t options, const ft_char8_t* s)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*4 + (ft_uint16_t)strlen((ft_const_char8_t *)s) + 1);
  FT_GPU_Copro_SendCmd(phost, CMD_BUTTON);
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)y<<16)|x));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)h<<16)|w));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)options<<16)|font));
  FT_GPU_CoCmd_SendStr(phost, s);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*4 + (ft_uint16_t)strlen((ft_const_char8_t *)s) + 1));
}

ft_void_t FT_GPU_CoCmd_Button_P(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t h, ft_int16_t font, ft_uint16_t options, ft_prog_char8_t* s)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*4 + (ft_uint16_t)strlen_P((ft_prog_char8_t *)s) + 1);
  FT_GPU_Copro_SendCmd(phost, CMD_BUTTON);
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)y<<16)|x));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)h<<16)|w));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)options<<16)|font));
  FT_GPU_CoCmd_SendStr_P(phost, s);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*4 + (ft_uint16_t)strlen_P((ft_prog_char8_t *)s) + 1));
}

ft_void_t FT_GPU_CoCmd_MemWrite(FT_GPU_HAL_Context_t *phost, ft_uint32_t ptr, ft_uint32_t num)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*3);
  FT_GPU_Copro_SendCmd(phost, CMD_MEMWRITE);
  FT_GPU_Copro_SendCmd(phost, ptr);
  FT_GPU_Copro_SendCmd(phost, num);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*3));
}

ft_void_t FT_GPU_CoCmd_Scrollbar(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t h, ft_uint16_t options, ft_uint16_t val, ft_uint16_t size, ft_uint16_t range)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*5);
  FT_GPU_Copro_SendCmd(phost, CMD_SCROLLBAR);
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)y<<16)|x));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)h<<16)|w));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)val<<16)|options));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)range<<16)|size));
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*5));
}

ft_void_t FT_GPU_CoCmd_GetMatrix(FT_GPU_HAL_Context_t *phost, ft_int32_t a, ft_int32_t b, ft_int32_t c, ft_int32_t d, ft_int32_t e, ft_int32_t f)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*7);
  FT_GPU_Copro_SendCmd(phost, CMD_GETMATRIX);
  FT_GPU_Copro_SendCmd(phost, a);
  FT_GPU_Copro_SendCmd(phost, b);
  FT_GPU_Copro_SendCmd(phost, c);
  FT_GPU_Copro_SendCmd(phost, d);
  FT_GPU_Copro_SendCmd(phost, e);
  FT_GPU_Copro_SendCmd(phost, f);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*7));
}

ft_void_t FT_GPU_CoCmd_Sketch(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_uint16_t w, ft_uint16_t h, ft_uint32_t ptr, ft_uint16_t format)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*5);
  FT_GPU_Copro_SendCmd(phost, CMD_SKETCH);
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)y<<16)|x));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)h<<16)|w));
  FT_GPU_Copro_SendCmd(phost, ptr);
  FT_GPU_Copro_SendCmd(phost, format);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*5));
}

ft_void_t FT_GPU_CoCmd_MemSet(FT_GPU_HAL_Context_t *phost, ft_uint32_t ptr, ft_uint32_t value, ft_uint32_t num)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*4);
  FT_GPU_Copro_SendCmd(phost, CMD_MEMSET);
  FT_GPU_Copro_SendCmd(phost, ptr);
  FT_GPU_Copro_SendCmd(phost, value);
  FT_GPU_Copro_SendCmd(phost, num);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*4));
}

ft_void_t FT_GPU_CoCmd_GradColor(FT_GPU_HAL_Context_t *phost, ft_uint32_t c)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*2);
  FT_GPU_Copro_SendCmd(phost, CMD_GRADCOLOR);
  FT_GPU_Copro_SendCmd(phost, c);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*2));
}

ft_void_t FT_GPU_CoCmd_BitmapTransform(FT_GPU_HAL_Context_t *phost, ft_int32_t x0, ft_int32_t y0, ft_int32_t x1, ft_int32_t y1, ft_int32_t x2, ft_int32_t y2, ft_int32_t tx0, ft_int32_t ty0, ft_int32_t tx1, ft_int32_t ty1, ft_int32_t tx2, ft_int32_t ty2, ft_uint16_t result)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*6*2+FT_CMD_SIZE*2);
  FT_GPU_Copro_SendCmd(phost, CMD_BITMAP_TRANSFORM);
  FT_GPU_Copro_SendCmd(phost, x0);
  FT_GPU_Copro_SendCmd(phost, y0);
  FT_GPU_Copro_SendCmd(phost, x1);
  FT_GPU_Copro_SendCmd(phost, y1);
  FT_GPU_Copro_SendCmd(phost, x2);
  FT_GPU_Copro_SendCmd(phost, y2);
  FT_GPU_Copro_SendCmd(phost, tx0);
  FT_GPU_Copro_SendCmd(phost, ty0);
  FT_GPU_Copro_SendCmd(phost, tx1);
  FT_GPU_Copro_SendCmd(phost, ty1);
  FT_GPU_Copro_SendCmd(phost, tx2);
  FT_GPU_Copro_SendCmd(phost, ty2);
  FT_GPU_Copro_SendCmd(phost, result);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*6*2+FT_CMD_SIZE*2));
}

ft_void_t FT_GPU_CoCmd_Calibrate(FT_GPU_HAL_Context_t *phost, ft_uint32_t result)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*2);
  FT_GPU_Copro_SendCmd(phost, CMD_CALIBRATE);
  FT_GPU_Copro_SendCmd(phost, result);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*2));
  FT_GPU_HAL_WaitCmdfifo_empty(phost);
}

ft_void_t FT_GPU_CoCmd_SetFont(FT_GPU_HAL_Context_t *phost, ft_uint32_t font, ft_uint32_t ptr)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*3);
  FT_GPU_Copro_SendCmd(phost, CMD_SETFONT);
  FT_GPU_Copro_SendCmd(phost, font);
  FT_GPU_Copro_SendCmd(phost, ptr);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*3));
}

ft_void_t FT_GPU_CoCmd_Logo(FT_GPU_HAL_Context_t *phost)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*1);
  FT_GPU_Copro_SendCmd(phost, CMD_LOGO);
  FT_GPU_CoCmd_EndFunc(phost,FT_CMD_SIZE*1);
}

ft_void_t FT_GPU_CoCmd_Append(FT_GPU_HAL_Context_t *phost, ft_uint32_t ptr, ft_uint32_t num)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*3);
  FT_GPU_Copro_SendCmd(phost, CMD_APPEND);
  FT_GPU_Copro_SendCmd(phost, ptr);
  FT_GPU_Copro_SendCmd(phost, num);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*3));
}

ft_void_t FT_GPU_CoCmd_MemZero(FT_GPU_HAL_Context_t *phost, ft_uint32_t ptr, ft_uint32_t num)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*3);
  FT_GPU_Copro_SendCmd(phost, CMD_MEMZERO);
  FT_GPU_Copro_SendCmd(phost, ptr);
  FT_GPU_Copro_SendCmd(phost, num);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*3));
}

ft_void_t FT_GPU_CoCmd_Scale(FT_GPU_HAL_Context_t *phost, ft_int32_t sx, ft_int32_t sy)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*3);
  FT_GPU_Copro_SendCmd(phost, CMD_SCALE);
  FT_GPU_Copro_SendCmd(phost, sx);
  FT_GPU_Copro_SendCmd(phost, sy);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*3));
}

ft_void_t FT_GPU_CoCmd_Clock(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t r, ft_uint16_t options, ft_uint16_t h, ft_uint16_t m, ft_uint16_t s, ft_uint16_t ms)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*5);
  FT_GPU_Copro_SendCmd(phost, CMD_CLOCK);
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)y<<16)|x));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)options<<16)|r));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)m<<16)|h));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)ms<<16)|s));
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*5));
}

ft_void_t FT_GPU_CoCmd_Gradient(FT_GPU_HAL_Context_t *phost, ft_int16_t x0, ft_int16_t y0, ft_uint32_t rgb0, ft_int16_t x1, ft_int16_t y1, ft_uint32_t rgb1)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*5);
  FT_GPU_Copro_SendCmd(phost, CMD_GRADIENT);
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)y0<<16)|x0));
  FT_GPU_Copro_SendCmd(phost, rgb0);
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)y1<<16)|x1));
  FT_GPU_Copro_SendCmd(phost, rgb1);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*5));
}

ft_void_t FT_GPU_CoCmd_SetMatrix(FT_GPU_HAL_Context_t *phost)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*1);
  FT_GPU_Copro_SendCmd(phost, CMD_SETMATRIX);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*1));
}

ft_void_t FT_GPU_CoCmd_Track(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t h, ft_int16_t tag)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*4);
  FT_GPU_Copro_SendCmd(phost, CMD_TRACK);
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)y<<16)|x));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)h<<16)|w));
  FT_GPU_Copro_SendCmd(phost, tag);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*4));
}

ft_void_t FT_GPU_CoCmd_GetPtr(FT_GPU_HAL_Context_t *phost,ft_uint32_t result)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*2);
  FT_GPU_Copro_SendCmd(phost, CMD_GETPTR);
  FT_GPU_Copro_SendCmd(phost, result);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*2));
}

ft_void_t FT_GPU_CoCmd_Progress(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t h, ft_uint16_t options, ft_uint16_t val, ft_uint16_t range)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*5);
  FT_GPU_Copro_SendCmd(phost, CMD_PROGRESS);
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)y<<16)|x));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)h<<16)|w));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)val<<16)|options));
  FT_GPU_Copro_SendCmd(phost, range);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*5));
}

ft_void_t FT_GPU_CoCmd_ColdStart(FT_GPU_HAL_Context_t *phost)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*1);
  FT_GPU_Copro_SendCmd(phost, CMD_COLDSTART);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*1));
}

ft_void_t FT_GPU_CoCmd_Keys(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t h, ft_int16_t font, ft_uint16_t options, const ft_char8_t* s)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*4 + (ft_uint16_t)strlen((ft_const_char8_t *)s) + 1);
  FT_GPU_Copro_SendCmd(phost, CMD_KEYS);
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)y<<16)|x));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)h<<16)|w));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)options<<16)|font));
  FT_GPU_CoCmd_SendStr(phost, s);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*4 + (ft_uint16_t)strlen((ft_const_char8_t *)s) + 1));
}

ft_void_t FT_GPU_CoCmd_Keys_P(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t h, ft_int16_t font, ft_uint16_t options, ft_prog_char8_t* s)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*4 + (ft_uint16_t)strlen_P((ft_prog_char8_t *)s) + 1);
  FT_GPU_Copro_SendCmd(phost, CMD_KEYS);
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)y<<16)|x));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)h<<16)|w));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)options<<16)|font));
  FT_GPU_CoCmd_SendStr_P(phost, s);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*4 + (ft_uint16_t)strlen_P((ft_prog_char8_t *)s) + 1));
}

ft_void_t FT_GPU_CoCmd_Dial(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t r, ft_uint16_t options, ft_uint16_t val)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*4);
  FT_GPU_Copro_SendCmd(phost, CMD_DIAL);
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)y<<16)|x));
  FT_GPU_Copro_SendCmd(phost, (((ft_uint32_t)options<<16)|r));
  FT_GPU_Copro_SendCmd(phost, val);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*4));
}

ft_void_t FT_GPU_CoCmd_LoadImage(FT_GPU_HAL_Context_t *phost, ft_uint32_t ptr, ft_uint32_t options)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*3);
  FT_GPU_Copro_SendCmd(phost, CMD_LOADIMAGE);
  FT_GPU_Copro_SendCmd(phost, ptr);
  FT_GPU_Copro_SendCmd(phost, options);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*3));
}

ft_void_t FT_GPU_CoCmd_Dlstart(FT_GPU_HAL_Context_t *phost)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*1);
  FT_GPU_Copro_SendCmd(phost, CMD_DLSTART);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*1));
}

ft_void_t FT_GPU_CoCmd_Snapshot(FT_GPU_HAL_Context_t *phost, ft_uint32_t ptr)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*2);
  FT_GPU_Copro_SendCmd(phost, CMD_SNAPSHOT);
  FT_GPU_Copro_SendCmd(phost, ptr);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*2));
}

ft_void_t FT_GPU_CoCmd_ScreenSaver(FT_GPU_HAL_Context_t *phost)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*1);
  FT_GPU_Copro_SendCmd(phost, CMD_SCREENSAVER);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*1));
}

ft_void_t FT_GPU_CoCmd_MemCrc(FT_GPU_HAL_Context_t *phost, ft_uint32_t ptr, ft_uint32_t num, ft_uint32_t result)
{
  FT_GPU_CoCmd_StartFunc(phost,FT_CMD_SIZE*4);
  FT_GPU_Copro_SendCmd(phost, CMD_MEMCRC);
  FT_GPU_Copro_SendCmd(phost, ptr);
  FT_GPU_Copro_SendCmd(phost, num);
  FT_GPU_Copro_SendCmd(phost, result);
  FT_GPU_CoCmd_EndFunc(phost,(FT_CMD_SIZE*4));
}

/* Nothing beyond this */

