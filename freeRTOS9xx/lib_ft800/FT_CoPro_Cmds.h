#ifndef _FT_COPRO_CMDS_H_
#define _FT_COPRO_CMDS_H_

/*
File:   FT_CoPro_CMds.h
*/

#ifdef __cplusplus
extern "C" {
#endif

ft_void_t FT_GPU_CoCmd_Text(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t font, ft_uint16_t options, const ft_char8_t* s);
ft_void_t FT_GPU_CoCmd_Text_P(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t font, ft_uint16_t options, ft_prog_char8_t* s);
ft_void_t FT_GPU_CoCmd_Number(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t font, ft_uint16_t options, ft_int32_t n);
ft_void_t FT_GPU_CoCmd_LoadIdentity(FT_GPU_HAL_Context_t *phost);
ft_void_t FT_GPU_CoCmd_Toggle(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t font, ft_uint16_t options, ft_uint16_t state, const ft_char8_t* s);
ft_void_t FT_GPU_CoCmd_Toggle_P(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t font, ft_uint16_t options, ft_uint16_t state, ft_prog_char8_t* s);
ft_void_t FT_GPU_CoCmd_Gauge(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t r, ft_uint16_t options, ft_uint16_t major, ft_uint16_t minor, ft_uint16_t val, ft_uint16_t range);
ft_void_t FT_GPU_CoCmd_RegRead(FT_GPU_HAL_Context_t *phost, ft_uint32_t ptr, ft_uint32_t result); // fixme but there's no result returned..?
ft_void_t FT_GPU_CoCmd_GetProps(FT_GPU_HAL_Context_t *phost, ft_uint32_t ptr, ft_uint32_t w, ft_uint32_t h); // fixme not sure where the properties are returned..?
ft_void_t FT_GPU_CoCmd_Memcpy(FT_GPU_HAL_Context_t *phost, ft_uint32_t dest, ft_uint32_t src, ft_uint32_t num);
ft_void_t FT_GPU_CoCmd_Spinner(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_uint16_t style, ft_uint16_t scale);
ft_void_t FT_GPU_CoCmd_BgColor(FT_GPU_HAL_Context_t *phost, ft_uint32_t c);
ft_void_t FT_GPU_CoCmd_Swap(FT_GPU_HAL_Context_t *phost);
ft_void_t FT_GPU_CoCmd_Inflate(FT_GPU_HAL_Context_t *phost, ft_uint32_t ptr);
ft_void_t FT_GPU_CoCmd_Translate(FT_GPU_HAL_Context_t *phost, ft_int32_t tx, ft_int32_t ty);
ft_void_t FT_GPU_CoCmd_Stop(FT_GPU_HAL_Context_t *phost);
ft_void_t FT_GPU_CoCmd_Slider(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t h, ft_uint16_t options, ft_uint16_t val, ft_uint16_t range);
ft_void_t FT_GPU_CoCmd_Interrupt(FT_GPU_HAL_Context_t *phost, ft_uint32_t ms);
ft_void_t FT_GPU_CoCmd_FgColor(FT_GPU_HAL_Context_t *phost, ft_uint32_t c);
ft_void_t FT_GPU_CoCmd_Rotate(FT_GPU_HAL_Context_t *phost, ft_int32_t a);
ft_void_t FT_GPU_CoCmd_Button(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t h, ft_int16_t font, ft_uint16_t options, const ft_char8_t* s);
ft_void_t FT_GPU_CoCmd_Button_P(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t h, ft_int16_t font, ft_uint16_t options, ft_prog_char8_t* s);
ft_void_t FT_GPU_CoCmd_MemWrite(FT_GPU_HAL_Context_t *phost, ft_uint32_t ptr, ft_uint32_t num);
ft_void_t FT_GPU_CoCmd_Scrollbar(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t h, ft_uint16_t options, ft_uint16_t val, ft_uint16_t size, ft_uint16_t range);
ft_void_t FT_GPU_CoCmd_GetMatrix(FT_GPU_HAL_Context_t *phost, ft_int32_t a, ft_int32_t b, ft_int32_t c, ft_int32_t d, ft_int32_t e, ft_int32_t f);
ft_void_t FT_GPU_CoCmd_Sketch(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_uint16_t w, ft_uint16_t h, ft_uint32_t ptr, ft_uint16_t format);
ft_void_t FT_GPU_CoCmd_MemSet(FT_GPU_HAL_Context_t *phost, ft_uint32_t ptr, ft_uint32_t value, ft_uint32_t num);
ft_void_t FT_GPU_CoCmd_Calibrate(FT_GPU_HAL_Context_t *phost, ft_uint32_t result);
ft_void_t FT_GPU_CoCmd_SetFont(FT_GPU_HAL_Context_t *phost, ft_uint32_t font, ft_uint32_t ptr);
ft_void_t FT_GPU_CoCmd_Bitmap_Transform(FT_GPU_HAL_Context_t *phost, ft_int32_t x0, ft_int32_t y0, ft_int32_t x1, ft_int32_t y1, ft_int32_t x2, ft_int32_t y2, ft_int32_t tx0, ft_int32_t ty0, ft_int32_t tx1, ft_int32_t ty1, ft_int32_t tx2, ft_int32_t ty2, ft_uint16_t result);
ft_void_t FT_GPU_CoCmd_GradColor(FT_GPU_HAL_Context_t *phost, ft_uint32_t c);
ft_void_t FT_GPU_CoCmd_Append(FT_GPU_HAL_Context_t *phost, ft_uint32_t ptr, ft_uint32_t num);
ft_void_t FT_GPU_CoCmd_MemZero(FT_GPU_HAL_Context_t *phost, ft_uint32_t ptr, ft_uint32_t num);
ft_void_t FT_GPU_CoCmd_Scale(FT_GPU_HAL_Context_t *phost, ft_int32_t sx, ft_int32_t sy);
ft_void_t FT_GPU_CoCmd_Clock(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t r, ft_uint16_t options, ft_uint16_t h, ft_uint16_t m, ft_uint16_t s, ft_uint16_t ms);
ft_void_t FT_GPU_CoCmd_Gradient(FT_GPU_HAL_Context_t *phost, ft_int16_t x0, ft_int16_t y0, ft_uint32_t rgb0, ft_int16_t x1, ft_int16_t y1, ft_uint32_t rgb1);
ft_void_t FT_GPU_CoCmd_SetMatrix(FT_GPU_HAL_Context_t *phost);
ft_void_t FT_GPU_CoCmd_Track(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t h, ft_int16_t tag);
ft_void_t FT_GPU_CoCmd_GetPtr(FT_GPU_HAL_Context_t *phost, ft_uint32_t result);
ft_void_t FT_GPU_CoCmd_Progress(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t h, ft_uint16_t options, ft_uint16_t val, ft_uint16_t range);
ft_void_t FT_GPU_CoCmd_ColdStart(FT_GPU_HAL_Context_t *phost);
ft_void_t FT_GPU_CoCmd_Keys(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t h, ft_int16_t font, ft_uint16_t options, const ft_char8_t* s);
ft_void_t FT_GPU_CoCmd_Keys_P(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t h, ft_int16_t font, ft_uint16_t options, ft_prog_char8_t* s);
ft_void_t FT_GPU_CoCmd_Dial(FT_GPU_HAL_Context_t *phost, ft_int16_t x, ft_int16_t y, ft_int16_t r, ft_uint16_t options, ft_uint16_t val);
ft_void_t FT_GPU_CoCmd_LoadImage(FT_GPU_HAL_Context_t *phost, ft_uint32_t ptr, ft_uint32_t options);
ft_void_t FT_GPU_CoCmd_Dlstart(FT_GPU_HAL_Context_t *phost);
ft_void_t FT_GPU_CoCmd_Snapshot(FT_GPU_HAL_Context_t *phost, ft_uint32_t ptr);
ft_void_t FT_GPU_CoCmd_ScreenSaver(FT_GPU_HAL_Context_t *phost);
ft_void_t FT_GPU_CoCmd_Memcrc(FT_GPU_HAL_Context_t *phost, ft_uint32_t ptr, ft_uint32_t num, ft_uint32_t result);
ft_void_t FT_GPU_CoCmd_Logo(FT_GPU_HAL_Context_t *phost);
ft_void_t FT_GPU_CoCmd_Calibrate(FT_GPU_HAL_Context_t *phost, ft_uint32_t result);

#ifdef __cplusplus
}
#endif


#endif  /* _FT_COPRO_CMDS_H_ */
