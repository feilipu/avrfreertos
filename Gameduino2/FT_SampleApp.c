/* Sample application to demonstrate FT800 primitives, widgets and customized screen shots */

#include "FT_Platform.h"
#include "./FT_SampleApp.h"

#define FT_APP_DELAY_BTW_APIS			(1000)
#define FT_APP_ENABLE_DELAY() 			(vTaskDelay( FT_APP_DELAY_BTW_APIS / portTICK_PERIOD_MS ))
#define FT_APP_ENABLE_DELAY_MS(x) 		(vTaskDelay( x / portTICK_PERIOD_MS ))

#if defined(FT_APP_ENABLE_APIS_SET0) || defined(FT_APP_ENABLE_APIS_SET1)|| defined(FT_APP_ENABLE_APIS_SET2) || defined(FT_APP_ENABLE_APIS_SET4)
extern FT_API_Bitmap_header_t  Lena_Bitmap_RawData_Header[];
extern ft_prog_uint8_t         Lena_Bitmap_RawData[] PROGMEM;
#endif

#ifdef FT_APP_ENABLE_APIS_SET2
extern ft_prog_uint8_t Lenaface40[]  PROGMEM;
extern ft_prog_uint8_t Mandrill256[] PROGMEM;
#endif

#ifdef FT_APP_ENABLE_APIS_SET3
extern ft_prog_uint8_t Roboto_BoldCondensed_12[] PROGMEM;
#endif

#ifdef FT_APP_ENABLE_APIS_SET0

/******************************************************************************/
/* Example code to display few points at various offsets with various colours */
/******************************************************************************/
ft_void_t	FT_APP_GPU_Points()
{
	/* Construct DL of points */
	FT_GPU_HAL_Wr32(phost, RAM_DL + 0 , CLEAR_COLOR_RGB(128,128,128));
	FT_GPU_HAL_Wr32(phost, RAM_DL + 4 , CLEAR(1,1,1));
	FT_GPU_HAL_Wr32(phost, RAM_DL + 8 , COLOR_RGB(128, 0, 0) );
	FT_GPU_HAL_Wr32(phost, RAM_DL + 12, POINT_SIZE(5 * 16) );
	FT_GPU_HAL_Wr32(phost, RAM_DL + 16, BEGIN(POINTS) );
	FT_GPU_HAL_Wr32(phost, RAM_DL + 20, VERTEX2F((FT_DispWidth/5) * 16, (FT_DispHeight/2) * 16) );
	FT_GPU_HAL_Wr32(phost, RAM_DL + 24, COLOR_RGB(0, 128, 0) );
	FT_GPU_HAL_Wr32(phost, RAM_DL + 28, POINT_SIZE(15 * 16) );
	FT_GPU_HAL_Wr32(phost, RAM_DL + 32, VERTEX2F((FT_DispWidth*2/5) * 16, (FT_DispHeight/2) * 16) );
	FT_GPU_HAL_Wr32(phost, RAM_DL + 36, COLOR_RGB(0, 0, 128) );
	FT_GPU_HAL_Wr32(phost, RAM_DL + 40, POINT_SIZE(25 * 16) );
	FT_GPU_HAL_Wr32(phost, RAM_DL + 44, VERTEX2F((FT_DispWidth*3/5) * 16, (FT_DispHeight/2) * 16) );
	FT_GPU_HAL_Wr32(phost, RAM_DL + 48, COLOR_RGB(128, 128, 0) );
	FT_GPU_HAL_Wr32(phost, RAM_DL + 52, POINT_SIZE(35 * 16) );
	FT_GPU_HAL_Wr32(phost, RAM_DL + 56, VERTEX2F((FT_DispWidth*4/5) * 16, (FT_DispHeight/2) * 16) );
	FT_GPU_HAL_Wr32(phost, RAM_DL + 60, DISPLAY()); // display the image

	/* Do a swap */
	FT_API_GPU_DLSwap(DLSWAP_FRAME);
	FT_APP_ENABLE_DELAY();
}

ft_void_t FT_APP_GPU_Lines()
{
	ft_int16_t LineHeight = 25;

	FT_API_Write_DLCmd(CLEAR(1, 1, 1)); // clear screen
	FT_API_Write_DLCmd(COLOR_RGB(128, 0, 0) );
	FT_API_Write_DLCmd(LINE_WIDTH(5 * 16) );
	FT_API_Write_DLCmd(BEGIN(LINES) );
	FT_API_Write_DLCmd(VERTEX2F((FT_DispWidth/4) * 16,((FT_DispHeight - LineHeight)/2) * 16) );
	FT_API_Write_DLCmd(VERTEX2F((FT_DispWidth/4) * 16,((FT_DispHeight + LineHeight)/2) * 16) );
	FT_API_Write_DLCmd(COLOR_RGB(0, 128, 0) );
	FT_API_Write_DLCmd(LINE_WIDTH(10 * 16) );
	LineHeight = 40;
	FT_API_Write_DLCmd(VERTEX2F((FT_DispWidth*2/4) * 16,((FT_DispHeight - LineHeight)/2) * 16) );
	FT_API_Write_DLCmd(VERTEX2F((FT_DispWidth*2/4) * 16,((FT_DispHeight + LineHeight)/2) * 16) );
	FT_API_Write_DLCmd(COLOR_RGB(128, 128, 0) );
	FT_API_Write_DLCmd(LINE_WIDTH(20 * 16) );
	LineHeight = 55;
	FT_API_Write_DLCmd(VERTEX2F((FT_DispWidth*3/4) * 16, ((FT_DispHeight - LineHeight)/2) * 16) );
	FT_API_Write_DLCmd(VERTEX2F((FT_DispWidth*3/4) * 16, ((FT_DispHeight + LineHeight)/2) * 16) );
	FT_API_Write_DLCmd(DISPLAY() );

	/* Reset the DL Buffer index, set for the next group of DL commands */
	FT_API_Reset_DLBuffer();

	/* Do a swap */
	FT_API_GPU_DLSwap(DLSWAP_FRAME);
	FT_APP_ENABLE_DELAY();
}
ft_void_t	FT_APP_GPU_Rectangles()
{
	ft_int16_t RectWidth,RectHeight;

	FT_API_Write_DLCmd(CLEAR(1, 1, 1)); // clear screen
	FT_API_Write_DLCmd(COLOR_RGB(0, 0, 128) );
	FT_API_Write_DLCmd(LINE_WIDTH(1 * 16) );//LINE_WIDTH is used for corner curvature
	FT_API_Write_DLCmd(BEGIN(RECTS) );
	RectWidth = 5;RectHeight = 25;
	FT_API_Write_DLCmd(VERTEX2F( ((FT_DispWidth/4) - (RectWidth/2)) * 16,((FT_DispHeight - RectHeight)/2) * 16) );
	FT_API_Write_DLCmd(VERTEX2F( ((FT_DispWidth/4) + (RectWidth/2)) * 16,((FT_DispHeight + RectHeight)/2) * 16) );
	FT_API_Write_DLCmd(COLOR_RGB(0, 128, 0) );
	FT_API_Write_DLCmd(LINE_WIDTH(5 * 16) );
	RectWidth = 10;RectHeight = 40;
	FT_API_Write_DLCmd(VERTEX2F( ((FT_DispWidth*2/4) - (RectWidth/2)) * 16,((FT_DispHeight - RectHeight)/2) * 16) );
	FT_API_Write_DLCmd(VERTEX2F( ((FT_DispWidth*2/4) + (RectWidth/2)) * 16,((FT_DispHeight + RectHeight)/2) * 16) );
	FT_API_Write_DLCmd(COLOR_RGB(128, 128, 0) );
	FT_API_Write_DLCmd(LINE_WIDTH(10 * 16) );
	RectWidth = 20;RectHeight = 55;
	FT_API_Write_DLCmd(VERTEX2F( ((FT_DispWidth*3/4) - (RectWidth/2)) * 16,((FT_DispHeight - RectHeight)/2) * 16) );
	FT_API_Write_DLCmd(VERTEX2F( ((FT_DispWidth*3/4) + (RectWidth/2)) * 16,((FT_DispHeight + RectHeight)/2) * 16) );
	FT_API_Write_DLCmd(DISPLAY() );

	/* Reset the DL Buffer index, set for the next group of DL commands */
	FT_API_Reset_DLBuffer();

	/* Do a swap */
	FT_API_GPU_DLSwap(DLSWAP_FRAME);
	FT_APP_ENABLE_DELAY();
}

/* Display lena face at different locations with various colours, -ve offsets, alpha blend etc */
ft_void_t	FT_APP_GPU_Bitmap()
{
	FT_API_Bitmap_header_t *p_bmhdr;
	ft_int16_t BMoffsetx,BMoffsety;

	p_bmhdr = (FT_API_Bitmap_header_t *)&Lena_Bitmap_RawData_Header[0];
	/* Copy raw data into address 0 followed by generation of bitmap */
	FT_GPU_HAL_WrMem_P(phost, RAM_G, &Lena_Bitmap_RawData[p_bmhdr->ArrayOffset], p_bmhdr->Stride*p_bmhdr->Height);

	FT_API_Write_DLCmd(CLEAR(1, 1, 1)); // clear screen
	FT_API_Write_DLCmd(COLOR_RGB(255,255,255));
	FT_API_Write_DLCmd(BITMAP_SOURCE(RAM_G));
	FT_API_Write_DLCmd(BITMAP_LAYOUT(p_bmhdr->Format, p_bmhdr->Stride, p_bmhdr->Height));
	FT_API_Write_DLCmd(BITMAP_SIZE(NEAREST, BORDER, BORDER, p_bmhdr->Width, p_bmhdr->Height));
	FT_API_Write_DLCmd(BEGIN(BITMAPS)); // start drawing bitmaps
	BMoffsetx = ((FT_DispWidth/4) - (p_bmhdr->Width/2));
	BMoffsety = ((FT_DispHeight/2) - (p_bmhdr->Height/2));
	FT_API_Write_DLCmd(VERTEX2II(BMoffsetx, BMoffsety, 0, 0));
	FT_API_Write_DLCmd(COLOR_RGB(255, 64, 64)); // red at (200, 120)
	BMoffsetx = ((FT_DispWidth*2/4) - (p_bmhdr->Width/2));
	BMoffsety = ((FT_DispHeight/2) - (p_bmhdr->Height/2));
	FT_API_Write_DLCmd(VERTEX2II(BMoffsetx, BMoffsety, 0, 0));
	FT_API_Write_DLCmd(COLOR_RGB(64, 180, 64)); // green at (216, 136)
	BMoffsetx += (p_bmhdr->Width/2);
	BMoffsety += (p_bmhdr->Height/2);
	FT_API_Write_DLCmd(VERTEX2II(BMoffsetx, BMoffsety, 0, 0));
	FT_API_Write_DLCmd(COLOR_RGB(255, 255, 64)); // transparent yellow at (232, 152)
	FT_API_Write_DLCmd(COLOR_A(150));
	BMoffsetx += (p_bmhdr->Width/2);
	BMoffsety += (p_bmhdr->Height/2);
	FT_API_Write_DLCmd(VERTEX2II(BMoffsetx, BMoffsety, 0, 0));
	FT_API_Write_DLCmd(COLOR_A(255));
	FT_API_Write_DLCmd(COLOR_RGB(255,255,255));
	FT_API_Write_DLCmd(VERTEX2F(-10*16, -10*16));//for -ve coordinates use vertex2f instruction
	FT_API_Write_DLCmd(END());
	FT_API_Write_DLCmd(DISPLAY() );

	/* Reset the DL Buffer index, set for the next group of DL commands */
	FT_API_Reset_DLBuffer();

	/* Do a swap */
	FT_API_GPU_DLSwap(DLSWAP_FRAME);
	FT_APP_ENABLE_DELAY();
}

/* inbuilt font example for proportional and non proportional text - hello world */
ft_void_t	FT_APP_GPU_Fonts()
{
	ft_int32_t i,j,hoffset,voffset,stringlen;
	ft_uint32_t FontTableAddress;
	const ft_uint8_t Display_string[] = "Hello World";
	FT_GPU_Fonts_t Display_fontstruct;

	hoffset = ((FT_DispWidth - 100)/2);voffset = FT_DispHeight/2;

	/* Read the font address from 0xFFFFC location */
	FontTableAddress = FT_GPU_HAL_Rd32(phost, 0xFFFFC);
        stringlen = sizeof(Display_string) - 1;
	for(i=0;i<16;i++)
	{
		/* Read the font table from hardware */
		FT_GPU_HAL_RdMem(phost,(FontTableAddress + i*FT_GPU_FONT_TABLE_SIZE),(ft_uint8_t *)&Display_fontstruct,FT_GPU_FONT_TABLE_SIZE);

		FT_API_Write_DLCmd(CLEAR(1, 1, 1)); // clear screen
		FT_API_Write_DLCmd(COLOR_RGB(255, 255, 255)); // clear screen

		/* Display string at the centre of display */
		FT_API_Write_DLCmd(BEGIN(BITMAPS));
		hoffset = ((FT_DispWidth - 120)/2);
		voffset = ((FT_DispHeight - Display_fontstruct.FontHeightInPixels)/2);

		/* Display hello world by off setting with respect to char size */
		for(j=0;j<stringlen;j++)
		{
			FT_API_Write_DLCmd(VERTEX2II(hoffset,voffset,(i+16),Display_string[j]));
			hoffset += Display_fontstruct.FontWidth[Display_string[j]];
		}
		FT_API_Write_DLCmd(END());
		FT_API_Write_DLCmd(DISPLAY() );

		/* Reset the DL Buffer index, set for the next group of DL commands */
		FT_API_Reset_DLBuffer();

		/* Do a swap */
		FT_API_GPU_DLSwap(DLSWAP_FRAME);
        FT_APP_ENABLE_DELAY_MS(500);
	}
}
/* display text8x8 of abcdefgh */
ft_void_t	FT_APP_GPU_Text8x8()
{
	/* Write the data into RAM_G */
	const ft_uint8_t Text_Array[] = "abcdefgh";
	ft_int32_t String_size,hoffset = 16,voffset = 16;


        String_size = sizeof(Text_Array) - 1;
	FT_GPU_HAL_WrMem(phost,RAM_G,(ft_uint8_t *)Text_Array, String_size);

	/*
	      abcdefgh
	      abcdefgh
	*/

	FT_API_Write_DLCmd(CLEAR(1, 1, 1)); // clear screen
	FT_API_Write_DLCmd(BITMAP_SOURCE(RAM_G));
	FT_API_Write_DLCmd(BITMAP_LAYOUT(TEXT8X8, 1*8,1));//L1 format, each input data element is in 1 byte size
	FT_API_Write_DLCmd(BITMAP_SIZE(NEAREST, BORDER, REPEAT, 8*8, 8*2));//output is 8x8 format - draw 8 characters in horizontal repeated in 2 line

	FT_API_Write_DLCmd(BEGIN(BITMAPS));
	/* Display text 8x8 at hoffset, voffset location */
	FT_API_Write_DLCmd(VERTEX2F(hoffset*16,voffset*16));

	/*
           abcdabcdabcdabcd
           efghefghefghefgh
	*/
	FT_API_Write_DLCmd(BITMAP_LAYOUT(TEXT8X8, 1*4,2));//L1 format and each datatype is 1 byte size
	FT_API_Write_DLCmd(BITMAP_SIZE(NEAREST, REPEAT, BORDER, 8*16, 8*2));//each character is 8x8 in size -  so draw 32 characters in horizontal and 32 characters in vertical
	hoffset = FT_DispWidth/2;
	voffset = FT_DispHeight/2;
	FT_API_Write_DLCmd(VERTEX2F(hoffset*16,voffset*16));
	FT_API_Write_DLCmd(END());

	FT_API_Write_DLCmd(DISPLAY() );

	/* Reset the DL Buffer index, set for the next group of DL commands */
	FT_API_Reset_DLBuffer();

	/* Do a swap */
	FT_API_GPU_DLSwap(DLSWAP_FRAME);
	FT_APP_ENABLE_DELAY();
}

/* display textVGA of random values */
ft_void_t	FT_APP_GPU_TextVGA()
{
	/* Write the data into RAM_G */
	ft_uint16_t Text_Array[160];
	ft_int32_t String_size,hoffset = 32,voffset = 32,i;

	srandom(time( (void *)0 )); // set the seed based on the current system time.

	for(i=0;i<160;i++)
	{
		Text_Array[i] =  (uint16_t)random();
	}

	String_size = 160*2;
	FT_GPU_HAL_WrMem(phost,RAM_G,(ft_uint8_t*)Text_Array, String_size);


	FT_API_Write_DLCmd(CLEAR(1, 1, 1)); // clear screen
	FT_API_Write_DLCmd(BITMAP_SOURCE(RAM_G));

	/* mandatory for textvga as background color is also one of the parameter in textvga format */
	FT_API_Write_DLCmd(BLEND_FUNC(ONE,ZERO));

	//draw 8x8
	FT_API_Write_DLCmd(BITMAP_LAYOUT(TEXTVGA, 2*4,8));//L1 format, but each input data element is of 2 bytes in size
	FT_API_Write_DLCmd(BITMAP_SIZE(NEAREST, BORDER, BORDER, 8*8, 8*8));//output is 8x8 format - draw 8 characters in horizontal and 8 vertical
	FT_API_Write_DLCmd(BEGIN(BITMAPS));
	FT_API_Write_DLCmd(VERTEX2F(hoffset*16,voffset*16));
	FT_API_Write_DLCmd(END());

	//draw textvga
	FT_API_Write_DLCmd(BITMAP_LAYOUT(TEXTVGA, 2*16,8));//L1 format but each datatype is 16bit size
	FT_API_Write_DLCmd(BITMAP_SIZE(NEAREST, BORDER, REPEAT, 8*32, 8*32));//8 pixels per character and 32 rows/columns
	FT_API_Write_DLCmd(BEGIN(BITMAPS));
	hoffset = FT_DispWidth/2;
	voffset = FT_DispHeight/2;
	/* Display textvga at hoffset, voffset location */
	FT_API_Write_DLCmd(VERTEX2F(hoffset*16,voffset*16));
	FT_API_Write_DLCmd(END());

	FT_API_Write_DLCmd(DISPLAY() );

	/* Reset the DL Buffer index, set for the next group of DL commands */
	FT_API_Reset_DLBuffer();

	/* Do a swap */
	FT_API_GPU_DLSwap(DLSWAP_FRAME);
	FT_APP_ENABLE_DELAY();
}


#ifdef FT_APP_ENABLE_APIS_SET0
#define GRAPH_STRING		512

ft_void_t	FT_APP_GPU_Bargraph()
{
	/* Write the data into RAM_G */
	ft_uint8_t Y_Array[GRAPH_STRING];
	ft_int32_t hoffset = 0, voffset = 0;

	hoffset = 0;
	voffset = 0;

	srandom(time( (void *)0 )); // set the seed based on the current system time.

	for(uint16_t i=0; i<GRAPH_STRING; i++)
	{
		Y_Array[i] = (ft_uint8_t)random() >> 1; // range from 0 to 128
	}

	FT_GPU_HAL_WrMem(phost, RAM_G, (ft_uint8_t *)&Y_Array[0], GRAPH_STRING);

	FT_API_Write_DLCmd(CLEAR_COLOR_RGB(255,255,255));
	FT_API_Write_DLCmd(CLEAR(1, 1, 1)); // clear screen
	FT_API_Write_DLCmd(BITMAP_SOURCE(RAM_G));
	FT_API_Write_DLCmd(BITMAP_LAYOUT(BARGRAPH, 256,1));
	FT_API_Write_DLCmd(COLOR_RGB(128, 0,0));
	FT_API_Write_DLCmd(BITMAP_SIZE(NEAREST, BORDER, BORDER, 256, 256));
	FT_API_Write_DLCmd(BEGIN(BITMAPS));
	/* Display text 8x8 at hoffset, voffset location */
	FT_API_Write_DLCmd(VERTEX2II(hoffset,voffset,0,0));
	hoffset = 256;
	FT_API_Write_DLCmd(VERTEX2II(hoffset,voffset,0,1));

	FT_API_Write_DLCmd(DISPLAY() );

	/* Reset the DL Buffer index, set for the next group of DL commands */
	FT_API_Reset_DLBuffer();

	/* Do a swap */
	FT_API_GPU_DLSwap(DLSWAP_FRAME);
    FT_APP_ENABLE_DELAY();

#if 0
	for(uint16_t i=0; i<GRAPH_STRING; i++)
	{
        ft_int32_t tmpval;
//      ft_int32_t tmpidx;

//		tmpidx = (i + 256)&(512 - 1);
//      tmpidx = i;
        tmpval = 128 + ((ft_int32_t)(i/3)*FT_API_qsin(-65536*i/48)/65536);//within range

        Y_Array[i] = tmpval&0xff;
	}

	FT_GPU_HAL_WrMem(phost,RAM_G,(ft_uint8_t *)Y_Array, GRAPH_STRING);

	FT_API_Write_DLCmd(DISPLAY() );

	/* Reset the DL Buffer index, set for the next group of DL commands */
	FT_API_Reset_DLBuffer();

	/* Do a swap */
	FT_API_GPU_DLSwap(DLSWAP_FRAME);
    FT_APP_ENABLE_DELAY();
#endif

	FT_API_Write_DLCmd(CLEAR(1, 1, 1)); // clear screen
	FT_API_Write_DLCmd(BITMAP_SOURCE(RAM_G));
	FT_API_Write_DLCmd(BITMAP_LAYOUT(BARGRAPH, 256,1));
	FT_API_Write_DLCmd(BITMAP_SIZE(NEAREST, BORDER, BORDER, 256, 256));
	FT_API_Write_DLCmd(BEGIN(BITMAPS));
	FT_API_Write_DLCmd(COLOR_RGB(0xff, 0, 0));
	/* Display bargraph at hoffset, voffset location */
	hoffset = 0;
	voffset = 0;
	FT_API_Write_DLCmd(VERTEX2II(hoffset,voffset,0,0));
	hoffset = 256;
	voffset = 0;
	FT_API_Write_DLCmd(VERTEX2II(hoffset,voffset,0,1));
	hoffset = 0;
	voffset += 4;
	FT_API_Write_DLCmd(COLOR_RGB(0, 0, 0));
	FT_API_Write_DLCmd(VERTEX2II(hoffset,voffset,0,0));
	hoffset = 256;
	FT_API_Write_DLCmd(VERTEX2II(hoffset,voffset,0,1));

	FT_API_Write_DLCmd(DISPLAY() );

	/* Reset the DL Buffer index, set for the next group of DL commands */
	FT_API_Reset_DLBuffer();

	/* Do a swap */
	FT_API_GPU_DLSwap(DLSWAP_FRAME);
	FT_APP_ENABLE_DELAY();

}
#endif

ft_void_t	FT_APP_GPU_LineStrips()
{

	FT_API_Write_DLCmd(CLEAR_COLOR_RGB(5, 45, 10) );
	FT_API_Write_DLCmd(COLOR_RGB(255, 168, 64) );
	FT_API_Write_DLCmd(CLEAR(1 ,1 ,1) );
	FT_API_Write_DLCmd(BEGIN(LINE_STRIP) );
	FT_API_Write_DLCmd(VERTEX2F(16 * 16,16 * 16) );
	FT_API_Write_DLCmd(VERTEX2F((FT_DispWidth*2/3) * 16,(FT_DispHeight*2/3) * 16) );
	FT_API_Write_DLCmd(VERTEX2F((FT_DispWidth - 80) * 16,(FT_DispHeight - 20) * 16) );
	FT_API_Write_DLCmd(DISPLAY() );

	/* Reset the DL Buffer index, set for the next group of DL commands */
	FT_API_Reset_DLBuffer();

	/* Do a swap */
	FT_API_GPU_DLSwap(DLSWAP_FRAME);
	FT_APP_ENABLE_DELAY();

}

ft_void_t	FT_APP_GPU_EdgeStrips()
{

	FT_API_Write_DLCmd(CLEAR_COLOR_RGB(5, 45, 10) );
	FT_API_Write_DLCmd(COLOR_RGB(255, 168, 64) );
	FT_API_Write_DLCmd(CLEAR(1 ,1 ,1) );
	FT_API_Write_DLCmd(BEGIN(EDGE_STRIP_R) );
	FT_API_Write_DLCmd(VERTEX2F(16 * 16,16 * 16) );
	FT_API_Write_DLCmd(VERTEX2F((FT_DispWidth*2/3) * 16,(FT_DispHeight*2/3) * 16) );
	FT_API_Write_DLCmd(VERTEX2F((FT_DispWidth - 80) * 16,(FT_DispHeight - 20) * 16) );
	FT_API_Write_DLCmd(DISPLAY() );

	/* Reset the DL Buffer index, set for the next group of DL commands */
	FT_API_Reset_DLBuffer();

	/* Do a swap */
	FT_API_GPU_DLSwap(DLSWAP_FRAME);
	FT_APP_ENABLE_DELAY();

}
ft_void_t	FT_APP_GPU_Scissor()
{

	FT_API_Write_DLCmd(CLEAR(1,1,1)); // Clear to black
	FT_API_Write_DLCmd(SCISSOR_XY(40, 20)); // Scissor rectangle top left at (40, 20)
	FT_API_Write_DLCmd(SCISSOR_SIZE(40, 40)); // Scissor rectangle is 40 x 40 pixels
	FT_API_Write_DLCmd(CLEAR_COLOR_RGB(255, 255, 0)); // Clear to yellow
	FT_API_Write_DLCmd(CLEAR(1, 1, 1));
	FT_API_Write_DLCmd(DISPLAY());

	/* Reset the DL Buffer index, set for the next group of DL commands */
	FT_API_Reset_DLBuffer();

	/* Do a swap */
	FT_API_GPU_DLSwap(DLSWAP_FRAME);
	FT_APP_ENABLE_DELAY();
}


ft_void_t FT_APP_GPU_Polygon()
{

	FT_API_Write_DLCmd(CLEAR(1, 1, 1)); // clear screen
	FT_API_Write_DLCmd(COLOR_RGB(255, 0, 0) );
	FT_API_Write_DLCmd(STENCIL_OP(INCR,INCR) );
	FT_API_Write_DLCmd(COLOR_MASK(0,0,0,0) );//mask all the colors
	FT_API_Write_DLCmd(BEGIN(EDGE_STRIP_L));
	FT_API_Write_DLCmd(VERTEX2II((FT_DispWidth/2),(FT_DispHeight/4),0,0));
	FT_API_Write_DLCmd(VERTEX2II((FT_DispWidth*4/5),(FT_DispHeight*4/5),0,0));
	FT_API_Write_DLCmd(VERTEX2II((FT_DispWidth/4),(FT_DispHeight/2),0,0));
	FT_API_Write_DLCmd(VERTEX2II((FT_DispWidth/2),(FT_DispHeight/4),0,0));
	FT_API_Write_DLCmd(END());
	FT_API_Write_DLCmd(COLOR_MASK(1,1,1,1) );//enable all the colors
	FT_API_Write_DLCmd(STENCIL_FUNC(EQUAL,1,255));
	FT_API_Write_DLCmd(BEGIN(EDGE_STRIP_L));
	FT_API_Write_DLCmd(VERTEX2II(FT_DispWidth,0,0,0));
	FT_API_Write_DLCmd(VERTEX2II(FT_DispWidth,FT_DispHeight,0,0));
	FT_API_Write_DLCmd(END());

	/* Draw lines at the borders to make sure anti aliazing is also done */
	FT_API_Write_DLCmd(STENCIL_FUNC(ALWAYS,0,255));
	FT_API_Write_DLCmd(LINE_WIDTH(1*16));
	FT_API_Write_DLCmd(COLOR_RGB(0, 0, 0) );
	FT_API_Write_DLCmd(BEGIN(LINES));
	FT_API_Write_DLCmd(VERTEX2II((FT_DispWidth/2),(FT_DispHeight/4),0,0));
	FT_API_Write_DLCmd(VERTEX2II((FT_DispWidth*4/5),(FT_DispHeight*4/5),0,0));
	FT_API_Write_DLCmd(VERTEX2II((FT_DispWidth*4/5),(FT_DispHeight*4/5),0,0));
	FT_API_Write_DLCmd(VERTEX2II((FT_DispWidth/4),(FT_DispHeight/2),0,0));
	FT_API_Write_DLCmd(VERTEX2II((FT_DispWidth/4),(FT_DispHeight/2),0,0));
	FT_API_Write_DLCmd(VERTEX2II((FT_DispWidth/2),(FT_DispHeight/4),0,0));
	FT_API_Write_DLCmd(END());
	FT_API_Write_DLCmd(DISPLAY() );

	/* Reset the DL Buffer index, set for the next group of DL commands */
	FT_API_Reset_DLBuffer();

	/* Do a swap */
	FT_API_GPU_DLSwap(DLSWAP_FRAME);
    FT_APP_ENABLE_DELAY();
}

ft_void_t FT_APP_GPU_Cube()
{
    ft_uint32_t points[6*5],x,y,i,z;
    ft_int16_t xoffset,yoffset,CubeEdgeSz;

// colour vertices
    ft_uint16_t colors[6][3] = {{255,0,0},
                                {255,0,150},
                                {0,255,0},
                                {110,120,110},
                                {0,0,255},
                                {128,128,0}
	                            };

// Cube dimension is of 100*100*100
	CubeEdgeSz = 100;
	xoffset = (FT_DispWidth/2 - CubeEdgeSz);yoffset=((FT_DispHeight - CubeEdgeSz)/2);


// xy plane(front)
       points[0] = VERTEX2F(xoffset*16,yoffset*16);
       points[1] = VERTEX2F((xoffset+CubeEdgeSz)*16,yoffset*16);
       points[2] = VERTEX2F((xoffset+CubeEdgeSz)*16,(yoffset+CubeEdgeSz)*16);
       points[3] = VERTEX2F(xoffset*16,(yoffset+CubeEdgeSz)*16);
       points[4] = points[0];

//yz plane (left)
       x = (xoffset+(CubeEdgeSz/2));            //     xoff+w/2
       y = (yoffset-(CubeEdgeSz/2));            //     yoff-h/2

       points[5] = points[0];
       points[6] = VERTEX2F(x*16,y*16);
       points[7] = VERTEX2F(x*16,(y+CubeEdgeSz)*16);
       points[8] = points[3];
       points[9] = points[5];

//xz plane(top)
       points[10] = points[0];
       points[11] = points[1];
       points[12] = VERTEX2F((x+CubeEdgeSz)*16,(y)*16);
       points[13] = points[6];
       points[14] = points[10];

//xz plane(bottom)
       points[15] = points[3];
       points[16] = points[2];
       points[17] = VERTEX2F((x+CubeEdgeSz)*16,(y+CubeEdgeSz)*16);
       points[18] = points[7];
       points[19] = points[15];

//yz plane (right)
       points[20] = points[2];
       points[21] = points[17];
       points[22] = points[12];
       points[23] = points[1];
       points[24] = points[20];

//yz plane (back)
       points[25] = points[6];
       points[26] = points[7];
       points[27] = points[17];
       points[28] = points[12];
       points[29] = points[25];


       FT_API_Write_DLCmd(CLEAR(1,1,1));
       FT_API_Write_DLCmd(LINE_WIDTH(16));
       FT_API_Write_DLCmd(CLEAR_COLOR_RGB(255,255,255) );
       FT_API_Write_DLCmd(CLEAR(1,1,1) );
       FT_API_Write_DLCmd(COLOR_RGB(255,255,255));

// Draw a cube
       FT_API_Write_DLCmd(STENCIL_OP(INCR,INCR));
       FT_API_Write_DLCmd(COLOR_A(192));
       for(z=0;z<6;z++)
       {
			FT_API_Write_DLCmd(CLEAR(0,1,1) );//clear stencil buffer
			FT_API_Write_DLCmd(COLOR_MASK(0,0,0,0));//mask all the colors and draw one surface
			FT_API_Write_DLCmd(STENCIL_FUNC(ALWAYS,0,255));//stencil function to increment all the values
            FT_API_Write_DLCmd(BEGIN(EDGE_STRIP_L));
            for(i = 0;i<5;i++)
            {
				FT_API_Write_DLCmd(points[z*5 + i]);
            }
            FT_API_Write_DLCmd(END());
			/* set the color and draw a strip */
			FT_API_Write_DLCmd(COLOR_MASK(1,1,1,1));
			FT_API_Write_DLCmd(STENCIL_FUNC(EQUAL,1,255));
			//FT_API_Write_DLCmd(STENCIL_FUNC(EQUAL,(z+1),255));
            FT_API_Write_DLCmd(COLOR_RGB(colors[z][0],colors[z][1],colors[z][2]));
			FT_API_Write_DLCmd(BEGIN(RECTS));
			FT_API_Write_DLCmd(VERTEX2II(xoffset,0,0,0));
			FT_API_Write_DLCmd(VERTEX2II(xoffset + CubeEdgeSz*2,yoffset + CubeEdgeSz*2,0,0));
			FT_API_Write_DLCmd(END());
       }
    FT_API_Write_DLCmd(DISPLAY());

	/* Reset the DL Buffer index, set for the next group of DL commands */
	FT_API_Reset_DLBuffer();

	/* Do a swap */
	FT_API_GPU_DLSwap(DLSWAP_FRAME);
	FT_APP_ENABLE_DELAY();
}


ft_void_t	FT_APP_GPU_Stencil()
{
	ft_int16_t PointSize = 50,DispBtwPoints = 60;

	FT_API_Write_DLCmd(CLEAR(1,1,1)); // Clear to black
	FT_API_Write_DLCmd(SCISSOR_XY(40, 20)); // Scissor rectangle top left at (40, 20)
	FT_API_Write_DLCmd(STENCIL_OP(INCR, INCR) );
	FT_API_Write_DLCmd(POINT_SIZE(PointSize * 16) );
	FT_API_Write_DLCmd(BEGIN(POINTS) );
	FT_API_Write_DLCmd(VERTEX2II(((FT_DispWidth - DispBtwPoints)/2), (FT_DispHeight/2), 0, 0) );
	FT_API_Write_DLCmd(VERTEX2II(((FT_DispWidth + DispBtwPoints)/2), (FT_DispHeight/2), 0, 0) );
	FT_API_Write_DLCmd(STENCIL_FUNC(EQUAL, 2, 255) );
	FT_API_Write_DLCmd(COLOR_RGB(128, 0, 0) );
	FT_API_Write_DLCmd(VERTEX2II((FT_DispWidth/2), (FT_DispHeight/2), 0, 0) );
	FT_API_Write_DLCmd(DISPLAY());

	/* Reset the DL Buffer index, set for the next group of DL commands */
	FT_API_Reset_DLBuffer();

	/* Do a swap */
	FT_API_GPU_DLSwap(DLSWAP_FRAME);
	FT_APP_ENABLE_DELAY();
}
/*****************************************************************************/
/* Example code to demonstrated display of point and text                    */
/*****************************************************************************/
ft_void_t	FT_APP_GPU_FTDIString()
{
	ft_int16_t hoffset,voffset,PointSz;

	voffset = ((FT_DispHeight - 49)/2);//49 is the max height of inbuilt font handle 31
	hoffset = ((FT_DispWidth - 4*36)/2);
	PointSz = 20;
	hoffset += PointSz;
	FT_GPU_HAL_Wr32(phost, RAM_DL + 0,  CLEAR(1, 1, 1)); // clear screen
	FT_GPU_HAL_Wr32(phost, RAM_DL + 4,  BEGIN(BITMAPS)); // start drawing bitmaps

	FT_GPU_HAL_Wr32(phost, RAM_DL + 8,  VERTEX2II(hoffset, voffset, 31, 'F')); // ascii F in font 31
	hoffset += 24;
	FT_GPU_HAL_Wr32(phost, RAM_DL + 12, VERTEX2II(hoffset, voffset, 31, 'T')); // ascii T
	hoffset += 26;
	FT_GPU_HAL_Wr32(phost, RAM_DL + 16, VERTEX2II(hoffset, voffset, 31, 'D')); // ascii D
	hoffset += 29;
	FT_GPU_HAL_Wr32(phost, RAM_DL + 20, VERTEX2II(hoffset, voffset, 31, 'I')); // ascii I
	FT_GPU_HAL_Wr32(phost, RAM_DL + 24, END());
	FT_GPU_HAL_Wr32(phost, RAM_DL + 28, COLOR_RGB(160, 22, 22)); // change color to red
	FT_GPU_HAL_Wr32(phost, RAM_DL + 32, POINT_SIZE(PointSz * 16)); // set point size
	hoffset = ((FT_DispWidth - 4*36)/2);
	FT_GPU_HAL_Wr32(phost, RAM_DL + 36, BEGIN(POINTS)); // start drawing points
	FT_GPU_HAL_Wr32(phost, RAM_DL + 40, VERTEX2II(hoffset, (FT_DispHeight/2), 0, 0)); // red point
	FT_GPU_HAL_Wr32(phost, RAM_DL + 44, END());
	FT_GPU_HAL_Wr32(phost, RAM_DL + 48, DISPLAY()); // display the image

	/* Do a swap */
	FT_API_GPU_DLSwap(DLSWAP_FRAME);
	FT_APP_ENABLE_DELAY();
}
/* Call and return example - simple graph */
ft_void_t	FT_APP_GPU_StreetMap()
{
	FT_API_Write_DLCmd(CLEAR_COLOR_RGB(236,232,224)); //light gray
	FT_API_Write_DLCmd(CLEAR(1,1,1));
	FT_API_Write_DLCmd(COLOR_RGB(170,157,136));//medium gray
	FT_API_Write_DLCmd(LINE_WIDTH(63));
	FT_API_Write_DLCmd(CALL(19));//draw the streets
	FT_API_Write_DLCmd(COLOR_RGB(250,250,250));//white
	FT_API_Write_DLCmd(LINE_WIDTH(48));
	FT_API_Write_DLCmd(CALL(19));//draw the streets
	FT_API_Write_DLCmd(COLOR_RGB(0,0,0));
	FT_API_Write_DLCmd(BEGIN(BITMAPS));
	FT_API_Write_DLCmd(VERTEX2II(240,91,27,77  ));//draw "Main st." at (240,91)
	FT_API_Write_DLCmd(VERTEX2II(252,91,27,97	));
	FT_API_Write_DLCmd(VERTEX2II(260,91,27,105	));
	FT_API_Write_DLCmd(VERTEX2II(263,91,27,110	));
	FT_API_Write_DLCmd(VERTEX2II(275,91,27,115	));
	FT_API_Write_DLCmd(VERTEX2II(282,91,27,116	));
	FT_API_Write_DLCmd(VERTEX2II(286,91,27,46	));
	FT_API_Write_DLCmd(END());
	FT_API_Write_DLCmd(DISPLAY());
	FT_API_Write_DLCmd(BEGIN(LINES));
	FT_API_Write_DLCmd(VERTEX2F(-160,-20  ));
	FT_API_Write_DLCmd(VERTEX2F(320,4160  ));
	FT_API_Write_DLCmd(VERTEX2F(800,-20   ));
	FT_API_Write_DLCmd(VERTEX2F(1280,4160 ));
	FT_API_Write_DLCmd(VERTEX2F(1920,-20  ));
	FT_API_Write_DLCmd(VERTEX2F(2400,4160 ));
	FT_API_Write_DLCmd(VERTEX2F(2560,-20  ));
	FT_API_Write_DLCmd(VERTEX2F(3040,4160 ));
	FT_API_Write_DLCmd(VERTEX2F(3200,-20  ));
	FT_API_Write_DLCmd(VERTEX2F(3680,4160 ));
	FT_API_Write_DLCmd(VERTEX2F(2880,-20  ));
	FT_API_Write_DLCmd(VERTEX2F(3360,4160 ));
	FT_API_Write_DLCmd(VERTEX2F(-20,0	   ));
	FT_API_Write_DLCmd(VERTEX2F(5440,-480 ));
	FT_API_Write_DLCmd(VERTEX2F(-20,960   ));
	FT_API_Write_DLCmd(VERTEX2F(5440,480  ));
	FT_API_Write_DLCmd(VERTEX2F(-20,1920  ));
	FT_API_Write_DLCmd(VERTEX2F(5440,1440 ));
	FT_API_Write_DLCmd(VERTEX2F(-20,2880  ));
	FT_API_Write_DLCmd(VERTEX2F(5440,2400 ));
	FT_API_Write_DLCmd(END());
	FT_API_Write_DLCmd(RETURN());

	/* Reset the DL Buffer index, set for the next group of DL commands */
	FT_API_Reset_DLBuffer();

	/* Do a swap */
	FT_API_GPU_DLSwap(DLSWAP_FRAME);
    FT_APP_ENABLE_DELAY();
}

/* usage of additive blending - draw 3 Gs*/
ft_void_t	FT_APP_GPU_AdditiveBlendText()
{

	FT_API_Write_DLCmd(CLEAR(1, 1, 1)); // clear screen
	FT_API_Write_DLCmd(BEGIN(BITMAPS) );
	FT_API_Write_DLCmd(VERTEX2II(50, 30, 31, 0x47) );
	FT_API_Write_DLCmd(COLOR_A( 128 ) );
	FT_API_Write_DLCmd(VERTEX2II(58, 38, 31, 0x47) );
	FT_API_Write_DLCmd(COLOR_A( 64 ) );
	FT_API_Write_DLCmd(VERTEX2II(66, 46, 31, 0x47) );
	FT_API_Write_DLCmd(END() );
	FT_API_Write_DLCmd(DISPLAY() );

	/* Reset the DL Buffer index, set for the next group of DL commands */
	FT_API_Reset_DLBuffer();

	/* Do a swap */
	FT_API_GPU_DLSwap(DLSWAP_FRAME);
    FT_APP_ENABLE_DELAY();
}

/* Usage of macro */
ft_void_t	FT_APP_GPU_MacroUsage()
{
	ft_int32_t xoffset,yoffset,xflag = 1,yflag = 1,flagloop = 1;
	FT_API_Bitmap_header_t *p_bmhdr;

	xoffset = FT_DispWidth/3;
	yoffset = FT_DispHeight/2;

	/* First write a valid macro instruction into macro0 */
	FT_GPU_HAL_Wr32(phost, REG_MACRO_0,VERTEX2F(xoffset*16,yoffset*16));
	/* update lena face as bitmap 0 */

	p_bmhdr = &Lena_Bitmap_RawData_Header[0];
	/* Copy raw data into address 0 followed by generation of bitmap */
	FT_GPU_HAL_WrMem_P(phost, RAM_G,&Lena_Bitmap_RawData[p_bmhdr->ArrayOffset], p_bmhdr->Stride*p_bmhdr->Height);


	FT_API_Write_DLCmd(CLEAR(1, 1, 1)); // clear screen
	FT_API_Write_DLCmd(BITMAP_SOURCE(RAM_G));
	FT_API_Write_DLCmd(BITMAP_LAYOUT(p_bmhdr->Format, p_bmhdr->Stride, p_bmhdr->Height));
	FT_API_Write_DLCmd(BITMAP_SIZE(NEAREST, BORDER, BORDER, p_bmhdr->Width, p_bmhdr->Height));
	FT_API_Write_DLCmd(BEGIN(BITMAPS)); // start drawing bitmaps
	FT_API_Write_DLCmd(MACRO(0)); // draw the image at (100,120)
	FT_API_Write_DLCmd(END());
	FT_API_Write_DLCmd(DISPLAY() );

	/* Reset the DL Buffer index, set for the next group of DL commands */
	FT_API_Reset_DLBuffer();

	/* Do a swap */
	FT_API_GPU_DLSwap(DLSWAP_FRAME);
        flagloop = 1000;
	while(flagloop-- > 0)
	{
		if(((xoffset + p_bmhdr->Width) >= FT_DispWidth) || (xoffset <= 0))
		{
			xflag ^= 1;
		}
		if(((yoffset + p_bmhdr->Height) >= FT_DispHeight) || (yoffset <= 0))
		{
			yflag ^= 1;
		}
		if(xflag)
		{
			xoffset++;
		}
		else
		{
			xoffset--;
		}
		if(yflag)
		{
			yoffset++;
		}
		else
		{
			yoffset--;
		}
		/*  update just the macro */
		FT_GPU_HAL_Wr32(phost, REG_MACRO_0,VERTEX2F(xoffset*16,yoffset*16));
        FT_APP_ENABLE_DELAY_MS(10);	//sleep for 10 ms;
	}
}


/* Additive blending of points - 1000 points */
ft_void_t	FT_APP_GPU_AdditiveBlendPoints()
{
	ft_int32_t i,hoffset,voffset,flagloop=1,j,hdiff,vdiff,PointSz;

#define MSVC_PI (3.141592653)
	PointSz = 4;
        flagloop = 10;
	while(flagloop-- > 0)
	{
  		/* Reset the DL Buffer index, set for the next group of DL commands */
		FT_API_Reset_DLBuffer();

		FT_API_Write_DLCmd(CLEAR(1, 1, 1)); // clear screen
		FT_API_Write_DLCmd(COLOR_RGB(20, 91,20)); // green color for additive blending
		FT_API_Write_DLCmd(BLEND_FUNC(SRC_ALPHA,ONE));//input is source alpha and destination is whole color
		FT_API_Write_DLCmd(POINT_SIZE(PointSz*16));
		FT_API_Write_DLCmd(BEGIN(POINTS));


		/* First 100 random values */
		for(i=0;i<100;i++)
		{
			hoffset = random();
			voffset = random();
			FT_API_Write_DLCmd(VERTEX2F(hoffset*16,voffset*16));
		}

		/* next 480 are sine values of two cycles */
		for(i=0;i<160;i++)
		{
			/* i is x offset, y is sinwave */
			hoffset = i*3;

			//voffset = 136 + 136*sin((-(360*i/80)*MSVC_PI)/180);
			voffset = (FT_DispHeight/2) + ((ft_int32_t)(FT_DispHeight/2)*FT_API_qsin(-65536*i/(FT_DispWidth/6))/65536);

			FT_API_Write_DLCmd(VERTEX2F(hoffset*16,voffset*16));
			//randomvertexins[i*5] = VERTEX2F(hoffset*16,voffset*16);
			for(j=0;j<4;j++)
			{
				hdiff = random();
				vdiff = random();
				FT_API_Write_DLCmd(VERTEX2F((hoffset + hdiff)*16,(voffset + vdiff)*16));
			}
		}

		FT_API_Write_DLCmd(END());
		FT_API_Write_DLCmd(DISPLAY() );

		/* Reset the DL Buffer index, set for the next group of DL commands */
		FT_API_Reset_DLBuffer();

		/* Do a swap */
		FT_API_GPU_DLSwap(DLSWAP_FRAME);
        FT_APP_ENABLE_DELAY_MS(100);	//sleep for 100 ms

	}
}
#endif


ft_void_t FT_APP_Screen_P(ft_prog_char8_t *str)
{

	FT_GPU_CoCmd_Dlstart(phost);
	FT_API_Write_CoCmd(CLEAR_COLOR_RGB(255,255,255));
	FT_API_Write_CoCmd(CLEAR(1,1,1));

	FT_API_Write_CoCmd(COLOR_RGB(0x80,0x80,0x00));
	FT_GPU_CoCmd_Text_P(phost,(FT_DispWidth/2), FT_DispHeight/2, 31, OPT_CENTERX, str);

	FT_API_Write_CoCmd(DISPLAY());
	FT_GPU_CoCmd_Swap(phost);

	/* Wait till coprocessor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
	FT_APP_ENABLE_DELAY_MS(2000);
}


ft_void_t FT_APP_CoPro_Widget_Logo()
{
	FT_GPU_CoCmd_Logo(phost);

	FT_GPU_HAL_WaitLogo_Finish(phost);
	FT_APP_ENABLE_DELAY();
}


/* API to demonstrate calibrate widget/functionality */
ft_void_t FT_APP_CoPro_Widget_Calibrate()
{
	ft_uint32_t TransMatrix[6];

	/*************************************************************************/
	/* Below code demonstrates the usage of calibrate function. Calibrate    */
	/* function will wait until user presses all the three dots. Only way to*/
	/* come out of this api is to reset the coprocessor bit.                 */
	/*************************************************************************/
	{

	FT_GPU_CoCmd_Dlstart(phost);

	FT_API_Write_CoCmd(CLEAR_COLOR_RGB(64,64,64));
	FT_API_Write_CoCmd(CLEAR(1,1,1));
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	/* Draw number at 0,0 location */
	//FT_API_Write_CoCmd(COLOR_A(30));
	FT_GPU_CoCmd_Text_P(phost,(FT_DispWidth/2), (FT_DispHeight/2), 27, OPT_CENTER, PSTR("Please Tap on the dot"));
	FT_GPU_CoCmd_Calibrate(phost,0);

	/* Wait till coprocessor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
	/* Print the configured values */
	FT_GPU_HAL_RdMem(phost,REG_TOUCH_TRANSFORM_A,(ft_uint8_t *)TransMatrix,4*6);//read all the 6 coefficients
	}

}

#ifdef FT_APP_ENABLE_APIS_SET1
/* API to demonstrate text widget */
ft_void_t FT_APP_CoPro_Widget_Text()
{

	/*************************************************************************/
	/* Below code demonstrates the usage of text function. Text function     */
	/* draws text with either in built or externally configured text. Text   */
	/* color can be changed by colorrgb and text function supports display of*/
	/* texts on left, right, top, bottom and center respectively             */
	/*************************************************************************/
	{

	FT_GPU_CoCmd_Dlstart(phost);
	FT_API_Write_CoCmd(CLEAR_COLOR_RGB(64,64,64));
	FT_API_Write_CoCmd(CLEAR(1,1,1));
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	/* Draw text at 0,0 location */
	FT_API_Write_CoCmd(COLOR_RGB(0x00,0x00,0x80));
	FT_GPU_CoCmd_Text_P(phost,0, 0, 29, 0, PSTR("FTDI!"));
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	FT_GPU_CoCmd_Text_P(phost,0, 40, 26, 0, PSTR("Text29 at 0,0"));//text info
	/* text with centerx */
	FT_API_Write_CoCmd(COLOR_RGB(0x80,0x00,0x00));
	FT_GPU_CoCmd_Text_P(phost,(FT_DispWidth/2), 50, 29, OPT_CENTERX, PSTR("FTDI!"));
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	FT_GPU_CoCmd_Text_P(phost,((FT_DispWidth/2) - 30), 90, 26, 0, PSTR("Text29 centerX"));//text info
	/* text with centery */
	FT_API_Write_CoCmd(COLOR_RGB(0x41,0x01,0x05));
	FT_GPU_CoCmd_Text_P(phost,(FT_DispWidth/5), 120, 29, OPT_CENTERY, PSTR("FTDI!"));
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	FT_GPU_CoCmd_Text_P(phost,(FT_DispWidth/5), 140, 26, 0, PSTR("Text29 centerY"));//text info
	/* text with center */
	FT_API_Write_CoCmd(COLOR_RGB(0x0b,0x07,0x21));
	FT_GPU_CoCmd_Text_P(phost,(FT_DispWidth/2), 180, 29, OPT_CENTER, PSTR("FTDI!"));
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	FT_GPU_CoCmd_Text_P(phost,((FT_DispWidth/2) - 50), 200, 26, 0, PSTR("Text29 center"));//text info
	/* text with rightx */
	FT_API_Write_CoCmd(COLOR_RGB(0x57,0x5e,0x1b));
	FT_GPU_CoCmd_Text_P(phost,FT_DispWidth, 60, 29, OPT_RIGHTX, PSTR("FTDI!"));
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	FT_GPU_CoCmd_Text_P(phost,(FT_DispWidth - 90), 100, 26, 0, PSTR("Text29 rightX"));//text info
	FT_API_Write_CoCmd(DISPLAY());
	FT_GPU_CoCmd_Swap(phost);

	/* Wait till coprocessor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
    FT_APP_ENABLE_DELAY();
	}
}

/* API to demonstrate number widget */
ft_void_t FT_APP_CoPro_Widget_Number()
{
	/*************************************************************************/
	/* Below code demonstrates the usage of number function. Number function */
	/* draws text with only 32bit decimal number, signed or unsigned can also*/
	/* be specified as input parameter. Options like centerx, centery, center*/
	/* and rightx are supported                                              */
	/*************************************************************************/
	{


	FT_GPU_CoCmd_Dlstart(phost);
	FT_API_Write_CoCmd(CLEAR_COLOR_RGB(64,64,64));
	FT_API_Write_CoCmd(CLEAR(1,1,1));
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	/* Draw number at 0,0 location */
	FT_API_Write_CoCmd(COLOR_RGB(0x00,0x00,0x80));
	FT_GPU_CoCmd_Number(phost,0, 0, 29, 0, 1234);
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	FT_GPU_CoCmd_Text_P(phost,0, 40, 26, 0, PSTR("Number29 at 0,0"));//text info
	/* number with centerx */
	FT_API_Write_CoCmd(COLOR_RGB(0x80,0x00,0x00));
	FT_GPU_CoCmd_Number(phost,(FT_DispWidth/2), 50, 29, OPT_CENTERX | OPT_SIGNED, -1234);
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	FT_GPU_CoCmd_Text_P(phost,((FT_DispWidth/2) - 30), 90, 26, 0, PSTR("Number29 centerX"));//text info
	/* number with centery */
	FT_API_Write_CoCmd(COLOR_RGB(0x41,0x01,0x05));
	FT_GPU_CoCmd_Number(phost,(FT_DispWidth/5), 120, 29, OPT_CENTERY, 1234);
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	FT_GPU_CoCmd_Text_P(phost,(FT_DispWidth/5), 140, 26, 0, PSTR("Number29 centerY"));//text info
	/* number with center */
	FT_API_Write_CoCmd(COLOR_RGB(0x0b,0x07,0x21));
	FT_GPU_CoCmd_Number(phost,(FT_DispWidth/2), 180, 29, OPT_CENTER | OPT_SIGNED, -1234);
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	FT_GPU_CoCmd_Text_P(phost,((FT_DispWidth/2) - 50), 200, 26, 0, PSTR("Number29 center"));//text info
	/* number with rightx */
	FT_API_Write_CoCmd(COLOR_RGB(0x57,0x5e,0x1b));
	FT_GPU_CoCmd_Number(phost,FT_DispWidth, 60, 29, OPT_RIGHTX, 1234);
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	FT_GPU_CoCmd_Text_P(phost,(FT_DispWidth - 100), 100, 26, 0, PSTR("Number29 rightX"));//text info

	FT_API_Write_CoCmd(DISPLAY());
	FT_GPU_CoCmd_Swap(phost);

	/* Wait till coprocessor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
    FT_APP_ENABLE_DELAY();
	}
}
/* Main entry point */
/* API to demonstrate button functionality */
ft_void_t FT_APP_CoPro_Widget_Button()
{

	/*************************************************************************/
	/* Below code demonstrates the usage of button function. Buttons can be  */
	/* constructed using flat or 3d effect. Button color can be changed by   */
	/* fgcolor command and text color is set by COLOR_RGB graphics command   */
	/*************************************************************************/
	{
	ft_int16_t xOffset,yOffset,bWidth,bHeight,bDistx,bDisty;

	bWidth = 60;
	bHeight = 30;
	bDistx = bWidth + ((FT_DispWidth - 4 * bWidth)/5);
	bDisty = bHeight + 5;
	xOffset = 10;
	yOffset = (FT_DispHeight/2 - 2*bDisty);
	/************ Construct a buttons with "ONE/TWO/THREE" text and default background *********************/
	/* Draw buttons 60x30 resolution at 10x40,10x75,10x110 */

	FT_GPU_CoCmd_Dlstart(phost);
	FT_API_Write_CoCmd(CLEAR_COLOR_RGB(64,64,64));
	FT_API_Write_CoCmd(CLEAR(1,1,1));
	FT_API_Write_CoCmd(COLOR_RGB(255,255,255));
	/* flat effect and default color background */
	FT_GPU_CoCmd_FgColor(phost,0x0000ff);
	yOffset = (FT_DispHeight/2 - 2*bDisty);
	FT_GPU_CoCmd_Button_P(phost,xOffset,yOffset,bWidth,bHeight,28,OPT_FLAT,PSTR("ABC"));
	yOffset += bDisty;
	FT_GPU_CoCmd_Button_P(phost,xOffset,yOffset,bWidth,bHeight,28,OPT_FLAT,PSTR("ABC"));
	yOffset += bDisty;
	FT_GPU_CoCmd_Button_P(phost,xOffset,yOffset,bWidth,bHeight,28,OPT_FLAT,PSTR("ABC"));
	FT_GPU_CoCmd_Text_P(phost,xOffset, (yOffset + 40), 26, 0, PSTR("Flat effect"));//text info
	/* 3D effect */
	xOffset += bDistx;
	yOffset = (FT_DispHeight/2 - 2*bDisty);
	FT_GPU_CoCmd_Button_P(phost,xOffset,yOffset,bWidth,bHeight,28,0,PSTR("ABC"));
	yOffset += bDisty;
	FT_GPU_CoCmd_Button_P(phost,xOffset,yOffset,bWidth,bHeight,28,0,PSTR("ABC"));
	yOffset += bDisty;
	FT_GPU_CoCmd_Button_P(phost,xOffset,yOffset,bWidth,bHeight,28,0,PSTR("ABC"));
	FT_GPU_CoCmd_Text_P(phost,xOffset, (yOffset + 40), 26, 0, PSTR("3D Effect"));//text info
	/* 3d effect with background color */
	FT_GPU_CoCmd_FgColor(phost,0xffff00);
	xOffset += bDistx;
	yOffset = (FT_DispHeight/2 - 2*bDisty);
	FT_GPU_CoCmd_Button_P(phost,xOffset,yOffset,bWidth,bHeight,28,0,PSTR("ABC"));
	yOffset += bDisty;
	FT_GPU_CoCmd_FgColor(phost,0x00ffff);
	FT_GPU_CoCmd_Button_P(phost,xOffset,yOffset,bWidth,bHeight,28,0,PSTR("ABC"));
	yOffset += bDisty;
	FT_GPU_CoCmd_FgColor(phost,0xff00ff);
	FT_GPU_CoCmd_Button_P(phost,xOffset,yOffset,bWidth,bHeight,28,0,PSTR("ABC"));
	FT_GPU_CoCmd_Text_P(phost,xOffset, (yOffset + 40), 26, 0, PSTR("3D Colour"));//text info
	/* 3d effect with gradient color */
	FT_GPU_CoCmd_FgColor(phost,0x101010);
	FT_GPU_CoCmd_GradColor(phost,0xff0000);
	xOffset += bDistx;
	yOffset = (FT_DispHeight/2 - 2*bDisty);
	FT_GPU_CoCmd_Button_P(phost,xOffset,yOffset,bWidth,bHeight,28,0,PSTR("ABC"));
	yOffset += bDisty;
	FT_GPU_CoCmd_GradColor(phost,0x00ff00);
	FT_GPU_CoCmd_Button_P(phost,xOffset,yOffset,bWidth,bHeight,28,0,PSTR("ABC"));
	yOffset += bDisty;
	FT_GPU_CoCmd_GradColor(phost,0x0000ff);
	FT_GPU_CoCmd_Button_P(phost,xOffset,yOffset,bWidth,bHeight,28,0,PSTR("ABC"));
	FT_GPU_CoCmd_Text_P(phost,xOffset, (yOffset + 40), 26, 0, PSTR("3D Gradient"));//text info
	FT_API_Write_CoCmd(DISPLAY());
	FT_GPU_CoCmd_Swap(phost);

	/* Wait till coprocessor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
    FT_APP_ENABLE_DELAY();
	}
}
#endif

#ifdef FT_APP_ENABLE_APIS_SET4
/* API to demonstrate the use of transfer commands */
ft_void_t FT_APP_CoPro_AppendCmds()
{
	ft_uint32_t AppendCmds[16];
	ft_int16_t xoffset,yoffset;
	/*************************************************************************/
	/* Below code demonstrates the usage of coprocessor append command - to append any*/
	/* mcu specific graphics commands to coprocessor generated graphics commands      */
	/*************************************************************************/

	/* Bitmap construction by MCU - display lena at 200x60 offset */
	/* Construct the bitmap data to be copied in the temp buffer then by using
	   coprocessor append command copy it into graphics processor DL memory */
	xoffset = ((FT_DispWidth - Lena_Bitmap_RawData_Header[0].Width)/2);
	yoffset = ((FT_DispHeight/3) - Lena_Bitmap_RawData_Header[0].Height/2);

	FT_API_Write_CoCmd( CMD_DLSTART);
	AppendCmds[0] = CLEAR_COLOR_RGB(255,0,0);
	AppendCmds[1] = CLEAR(1,1,1);
	AppendCmds[2] = COLOR_RGB(255,255,255);
	AppendCmds[3] = BEGIN(BITMAPS);
	AppendCmds[4] = BITMAP_SOURCE(0);
	AppendCmds[5] = BITMAP_LAYOUT(Lena_Bitmap_RawData_Header[0].Format,
		Lena_Bitmap_RawData_Header[0].Stride,Lena_Bitmap_RawData_Header[0].Height);
	AppendCmds[6] = BITMAP_SIZE(BILINEAR,BORDER,BORDER,
		Lena_Bitmap_RawData_Header[0].Width,Lena_Bitmap_RawData_Header[0].Height);
	AppendCmds[7] = VERTEX2F(xoffset * 16,yoffset * 16);
	AppendCmds[8] = END();

        /* Download the bitmap data*/
	FT_GPU_HAL_WrMem_P(phost, RAM_G,(ft_uint8_t *)&Lena_Bitmap_RawData[Lena_Bitmap_RawData_Header[0].ArrayOffset],
		Lena_Bitmap_RawData_Header[0].Stride*Lena_Bitmap_RawData_Header[0].Height);

	/* Download the DL data constructed by the MCU to location 40*40*2 in sram */
	FT_GPU_HAL_WrMem(phost,RAM_G + Lena_Bitmap_RawData_Header[0].Stride*Lena_Bitmap_RawData_Header[0].Height,(ft_uint8_t *)AppendCmds,9*4);

	/* Call the append api for copying the above generated data into graphics processor
	   DL commands are stored at location 40*40*2 offset from the starting of the sram */
	FT_GPU_CoCmd_Append(phost,RAM_G + Lena_Bitmap_RawData_Header[0].Stride*Lena_Bitmap_RawData_Header[0].Height, 9*4);
	/*  Display the text information */
	FT_GPU_CoCmd_FgColor(phost,0xffff00);
	xoffset -=50;
	yoffset += 40;
	FT_GPU_CoCmd_Text_P(phost,xoffset, yoffset, 26, 0, PSTR("Display bitmap by Append"));
	FT_API_Write_CoCmd(DISPLAY());
	FT_GPU_CoCmd_Swap(phost);
    FT_APP_ENABLE_DELAY();
}
#endif

#ifdef FT_APP_ENABLE_APIS_SET2
/* API to demonstrate the usage of inflate command - compression done via zlib */
ft_void_t FT_APP_CoPro_Inflate()
{
	const FT_API_Bitmap_header_t *pBitmapHdr = (void *)0;
	ft_int16_t xoffset,yoffset;
	/***************************************************************************/
	/* Below code demonstrates the usage of inflate function                   */
	/* Download the deflated data into command buffer and in turn co-processor */
	/* inflates the deflated data and outputs at 0 location                    */
	/***************************************************************************/
	pBitmapHdr = &Lena_Bitmap_RawData_Header[0];

	xoffset = ((FT_DispWidth - Lena_Bitmap_RawData_Header[0].Width)/2);
	yoffset = ((FT_DispHeight - Lena_Bitmap_RawData_Header[0].Height)/2);

	/* Clear the memory at location 0 - any previous bitmap data */

	FT_API_Write_CoCmd( CMD_MEMSET);
	FT_API_Write_CoCmd( RAM_G);//starting address of memset
	FT_API_Write_CoCmd( 255L);//value of memset
	FT_API_Write_CoCmd( 1L*pBitmapHdr->Stride*pBitmapHdr->Height);//number of elements to be changed

	/* Set the display list for graphics processor */
	/* Bitmap construction by MCU - display lena at 200x90 offset */
	/* Transfer the data into co-processor memory directly word by word */
	FT_API_Write_CoCmd( CMD_DLSTART);
	FT_API_Write_CoCmd( CLEAR_COLOR_RGB(0,0,255));
	FT_API_Write_CoCmd( CLEAR(1,1,1));
	FT_API_Write_CoCmd( COLOR_RGB(255,255,255));
	FT_API_Write_CoCmd( BEGIN(BITMAPS));
	FT_API_Write_CoCmd( BITMAP_SOURCE(0));
	FT_API_Write_CoCmd( BITMAP_LAYOUT(Lena_Bitmap_RawData_Header[0].Format,
		Lena_Bitmap_RawData_Header[0].Stride,Lena_Bitmap_RawData_Header[0].Height));
	FT_API_Write_CoCmd( BITMAP_SIZE(BILINEAR,BORDER,BORDER,
		Lena_Bitmap_RawData_Header[0].Width,Lena_Bitmap_RawData_Header[0].Height));
	FT_API_Write_CoCmd( VERTEX2F(xoffset*16,yoffset*16));
	FT_API_Write_CoCmd( END());

	/*  Display the text information */
	FT_API_Write_CoCmd(COLOR_A(255));
	xoffset -= 50;
	yoffset += 40;
	FT_GPU_CoCmd_Text_P(phost,xoffset, yoffset, 26, 0, PSTR("Display bitmap by inflate"));
	FT_API_Write_CoCmd(DISPLAY());

	FT_GPU_CoCmd_Swap(phost);
	/* Wait till co-processor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);

	/* Inflate the data into location 0 */
	FT_GPU_HAL_WrCmd32(phost,  CMD_INFLATE);
	FT_GPU_HAL_WrCmd32(phost,  RAM_G);//destination address if inflated

	/* Copy the deflated/jpeg encoded data into co-processor fifo */
	FT_GPU_HAL_WrCmdBuf_P(phost, Lenaface40, 1L*FT_APP_Lenaface40_SIZE);

	FT_APP_ENABLE_DELAY_MS(2000);
}


/* API to demonstrate jpeg decode functionality */
ft_void_t FT_APP_CoPro_Loadimage()
{
	ft_int16_t ImgW, ImgH, xoffset,yoffset;
	/*************************************************************************/
	/* Below code demonstrates the usage of loadimage function               */
	/* Download the JPEG data into command buffer and in turn co-processor   */
	/* decodes and dumps into location 0 with rgb565 format                  */
	/*************************************************************************/
	ImgW = 256;
	ImgH = 256;

	xoffset = ((FT_DispWidth - ImgW)/2);
	yoffset = ((FT_DispHeight - ImgH)/2);
	/* Clear the memory at location 0 - any previous bitmap data */

	FT_API_Write_CoCmd( CMD_MEMSET);
	FT_API_Write_CoCmd( RAM_G);//starting address of memset
	FT_API_Write_CoCmd( 255L);//value of memset
	FT_API_Write_CoCmd( 256L*2*256);//number of elements to be changed

	/* Set the display list for graphics processor */

	/* Bitmap construction by MCU - display Mandrill at 112x8 offset */
	/* Transfer the data into co-processor memory directly word by word */
	FT_API_Write_CoCmd( CMD_DLSTART);
	FT_API_Write_CoCmd( CLEAR_COLOR_RGB(0,255,255));
	FT_API_Write_CoCmd( CLEAR(1,1,1));
	FT_API_Write_CoCmd( COLOR_RGB(255,255,255));
	FT_API_Write_CoCmd( BEGIN(BITMAPS));
	FT_API_Write_CoCmd( BITMAP_SOURCE(0));
	FT_API_Write_CoCmd( BITMAP_LAYOUT(RGB565, ImgW*2, ImgH));
	FT_API_Write_CoCmd( BITMAP_SIZE(BILINEAR, BORDER, BORDER, ImgW, ImgH));
	FT_API_Write_CoCmd( VERTEX2F(xoffset*16,yoffset*16));
	FT_API_Write_CoCmd( END());

	/*  Display the text information */
	xoffset = ((FT_DispWidth)/2);
	yoffset = ((FT_DispHeight)/2);
	FT_GPU_CoCmd_Text_P(phost,xoffset, yoffset, 26, OPT_CENTER, PSTR("Display bitmap by JPEG decode RGB565"));
	FT_API_Write_CoCmd(DISPLAY());
	FT_GPU_CoCmd_Swap(phost);

	/* Wait till co-processor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);

	/******************* Decode JPEG output into location 0 and output colour format as RGB565 *********************/
	FT_GPU_HAL_WrCmd32(phost,  CMD_LOADIMAGE);
	FT_GPU_HAL_WrCmd32(phost,  RAM_G);//destination address of jpg decode
	FT_GPU_HAL_WrCmd32(phost,  OPT_RGB565);//output format of the bitmap

	/* Copy the deflated/jpeg encoded data into coprocessor fifo */
	FT_GPU_HAL_WrCmdBuf_P(phost, Mandrill256, FT_APP_Mandrill256_SIZE);

	FT_APP_ENABLE_DELAY_MS(2000);

	/******************** Decode JPEG output into location 0 & output as MONOCHROME ******************************/
	/* Clear the memory at location 0 - any previous bitmap data */
	xoffset = ((FT_DispWidth - ImgW)/2);
	yoffset = ((FT_DispHeight - ImgH)/2);


	FT_API_Write_CoCmd( CMD_MEMSET);
	FT_API_Write_CoCmd( RAM_G);//starting address of memset
	FT_API_Write_CoCmd( 255L);//value of memset
	FT_API_Write_CoCmd( 256L*2*256);//number of elements to be changed

	/* Set the display list for graphics processor */
	/* Bitmap construction by MCU - display Mandrill at 112x8 offset */
	/* Transfer the data into co-processor memory directly word by word */
	FT_API_Write_CoCmd( CMD_DLSTART);
	FT_API_Write_CoCmd( CLEAR_COLOR_RGB(0,0,0));
	FT_API_Write_CoCmd( CLEAR(1,1,1));
	FT_API_Write_CoCmd( COLOR_RGB(255,255,255));
	FT_API_Write_CoCmd( BEGIN(BITMAPS));
	FT_API_Write_CoCmd( BITMAP_SOURCE(0));
	FT_API_Write_CoCmd( BITMAP_LAYOUT(L8,ImgW,ImgH));//monochrome
	FT_API_Write_CoCmd( BITMAP_SIZE(BILINEAR,BORDER,BORDER,ImgW,ImgH));
	FT_API_Write_CoCmd( VERTEX2F(xoffset*16,yoffset*16));
	FT_API_Write_CoCmd( END());

	/*  Display the text information */
	xoffset = ((FT_DispWidth)/2);
	yoffset = ((FT_DispHeight)/2);
	FT_GPU_CoCmd_Text_P(phost,xoffset, yoffset, 26, OPT_CENTER, PSTR("Display bitmap by JPEG decode L8"));
	FT_API_Write_CoCmd(DISPLAY());
	FT_GPU_CoCmd_Swap(phost);

	/* Wait till coprocessor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
	FT_APP_ENABLE_DELAY();

	FT_GPU_HAL_WrCmd32(phost,  CMD_LOADIMAGE);
	FT_GPU_HAL_WrCmd32(phost,  RAM_G);//destination address of jpg decode
	FT_GPU_HAL_WrCmd32(phost,  OPT_MONO);//output format of the bitmap - default is rgb565

	/* Copy the deflated/jpeg encoded data into co-processor fifo */
	FT_GPU_HAL_WrCmdBuf_P(phost,Mandrill256,FT_APP_Mandrill256_SIZE);

	FT_APP_ENABLE_DELAY_MS(2000);
}
#endif

#ifdef FT_APP_ENABLE_APIS_SET1
/* API to demonstrate clock widget */
ft_void_t FT_APP_CoPro_Widget_Clock()
{

	/*************************************************************************/
	/* Below code demonstrates the usage of clock function. Clocks can be    */
	/* constructed using flat or 3d effect. Clock background and foreground  */
	/* colors can be set by gbcolor and colorrgb. Clock can be constructed   */
	/* with multiple options such as no background, no needles, no pointer.  */
	/*************************************************************************/
	{
	ft_int16_t xOffset,yOffset,cRadius,xDistBtwClocks;

	xDistBtwClocks = FT_DispWidth/5;
	cRadius = xDistBtwClocks/2 - FT_DispWidth/64;

	/* Download the bitmap data for lena faced clock */
	FT_GPU_HAL_WrMem_P(phost, RAM_G,(ft_uint8_t *)&Lena_Bitmap_RawData[Lena_Bitmap_RawData_Header[0].ArrayOffset],
		Lena_Bitmap_RawData_Header[0].Stride*Lena_Bitmap_RawData_Header[0].Height);

	/* Draw clock with blue as background and read as needle color */

	FT_GPU_CoCmd_Dlstart(phost);
	FT_API_Write_CoCmd(CLEAR_COLOR_RGB(64,64,64));
	FT_API_Write_CoCmd(CLEAR(1,1,1));
	FT_API_Write_CoCmd(COLOR_RGB(255,255,255));
	/* flat effect and default color background */
	xOffset = xDistBtwClocks/2;
	yOffset = cRadius + 5;
	FT_GPU_CoCmd_BgColor(phost, 0x0000ff);
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0x00,0x00));
	FT_GPU_CoCmd_Clock(phost, xOffset,yOffset,cRadius,OPT_FLAT,30,100,5,10);
	FT_GPU_CoCmd_Text_P(phost,xOffset, (yOffset + cRadius + 6), 26, OPT_CENTER, PSTR("Flat effect"));//text info
	/* no seconds needle */
	FT_GPU_CoCmd_BgColor(phost, 0x00ff00);
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0x00,0x00));
	FT_GPU_CoCmd_FgColor(phost,0xff0000);
	xOffset += xDistBtwClocks;
	FT_GPU_CoCmd_Clock(phost, xOffset,yOffset,cRadius,OPT_NOSECS,10,10,5,10);
	FT_API_Write_CoCmd(COLOR_A(255));
	FT_GPU_CoCmd_Text_P(phost,xOffset, (yOffset + cRadius + 6), 26, OPT_CENTER, PSTR("No Secs"));//text info
	/* no background color */
	FT_GPU_CoCmd_BgColor(phost, 0x00ffff);
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0x00));
	xOffset += xDistBtwClocks;
	FT_GPU_CoCmd_Clock(phost, xOffset,yOffset,cRadius,OPT_NOBACK,10,10,5,10);
	FT_GPU_CoCmd_Text_P(phost,xOffset, (yOffset + cRadius + 6), 26, OPT_CENTER, PSTR("No BG"));//text info
	/* No ticks */
	FT_GPU_CoCmd_BgColor(phost, 0xff00ff);
	FT_API_Write_CoCmd(COLOR_RGB(0x00,0xff,0xff));
	xOffset += xDistBtwClocks;
	FT_GPU_CoCmd_Clock(phost, xOffset,yOffset,cRadius,OPT_NOTICKS,10,10,5,10);
	FT_GPU_CoCmd_Text_P(phost,xOffset, (yOffset + cRadius + 6), 26, OPT_CENTER, PSTR("No Ticks"));//text info
	/* No hands */
	FT_GPU_CoCmd_BgColor(phost, 0x808080);
	FT_API_Write_CoCmd(COLOR_RGB(0x00,0xff,0x00));
	xOffset += xDistBtwClocks;
	FT_GPU_CoCmd_Clock(phost, xOffset,yOffset,cRadius,OPT_NOHANDS,10,10,5,10);
	FT_GPU_CoCmd_Text_P(phost,xOffset, (yOffset + cRadius + 6), 26, OPT_CENTER, PSTR("No Hands"));//text info
	/* Bigger clock */
	yOffset += (cRadius + 10);
	cRadius = FT_DispHeight - (2*cRadius + 5 + 10);//calculate radius based on remaining height
	cRadius = (cRadius - 5 - 20)/2;
	xOffset = cRadius + 10;
	yOffset += cRadius + 5;
	FT_API_Write_CoCmd(COLOR_RGB(0x00,0x00,0xff));
	FT_GPU_CoCmd_Clock(phost, xOffset,yOffset,cRadius,0,10,10,5,10);

	xOffset += 2 * cRadius + 10;
	/* Lena clock with no background and no ticks */
	FT_API_Write_CoCmd(LINE_WIDTH(10*16));
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	FT_API_Write_CoCmd(BEGIN(RECTS));
	FT_API_Write_CoCmd(VERTEX2F((xOffset - cRadius + 10)*16,(yOffset - cRadius + 10)*16));
	FT_API_Write_CoCmd(VERTEX2F((xOffset + cRadius - 10)*16,(yOffset + cRadius - 10)*16));
	FT_API_Write_CoCmd(END());
	FT_API_Write_CoCmd(COLOR_A(0xff));
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	FT_API_Write_CoCmd(COLOR_MASK(0,0,0,1));
	FT_API_Write_CoCmd(CLEAR(1,1,1));
	FT_API_Write_CoCmd(BEGIN(RECTS));
	FT_API_Write_CoCmd(VERTEX2F((xOffset - cRadius + 12)*16,(yOffset - cRadius + 12)*16));
	FT_API_Write_CoCmd(VERTEX2F((xOffset + cRadius - 12)*16,(yOffset + cRadius - 12)*16));
	FT_API_Write_CoCmd(END());
	FT_API_Write_CoCmd(COLOR_MASK(1,1,1,1));
	FT_API_Write_CoCmd(BLEND_FUNC(DST_ALPHA,ONE_MINUS_DST_ALPHA));
	/* Lena bitmap - scale proportionately wrt output resolution */
	FT_GPU_CoCmd_LoadIdentity(phost);
	FT_GPU_CoCmd_Scale(phost, 65536*2*cRadius/Lena_Bitmap_RawData_Header[0].Width,65536*2*cRadius/Lena_Bitmap_RawData_Header[0].Height);
	FT_GPU_CoCmd_SetMatrix(phost );
	FT_API_Write_CoCmd(BEGIN(BITMAPS));
	FT_API_Write_CoCmd(BITMAP_SOURCE(0));
	FT_API_Write_CoCmd(BITMAP_LAYOUT(Lena_Bitmap_RawData_Header[0].Format,
		Lena_Bitmap_RawData_Header[0].Stride,Lena_Bitmap_RawData_Header[0].Height));
	FT_API_Write_CoCmd(BITMAP_SIZE(BILINEAR,BORDER,BORDER,
		2*cRadius,2*cRadius));
	FT_API_Write_CoCmd(VERTEX2F((xOffset - cRadius)*16,(yOffset - cRadius)*16));
	FT_API_Write_CoCmd(END());
	FT_API_Write_CoCmd(BLEND_FUNC(SRC_ALPHA,ONE_MINUS_SRC_ALPHA));
	FT_GPU_CoCmd_LoadIdentity(phost);
	FT_GPU_CoCmd_SetMatrix(phost );
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	FT_GPU_CoCmd_Clock(phost, xOffset,yOffset,cRadius,OPT_NOTICKS | OPT_NOBACK,10,10,5,10);
	FT_API_Write_CoCmd(DISPLAY());
	FT_GPU_CoCmd_Swap(phost);

	/* Wait till coprocessor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
    FT_APP_ENABLE_DELAY();
	}
}

/* API to demonstrate gauge widget */
ft_void_t FT_APP_CoPro_Widget_Gauge()
{
	/*************************************************************************/
	/* Below code demonstrates the usage of gauge function. Gauge can be     */
	/* constructed using flat or 3d effect. Gauge background and foreground  */
	/* colors can be set by gbcolor and colorrgb. Gauge can be constructed   */
	/* with multiple options such as no background, no minors/majors and     */
	/* no pointer.                                                           */
	/*************************************************************************/
	{
	ft_int16_t xOffset,yOffset,cRadius,xDistBtwClocks;

	xDistBtwClocks = FT_DispWidth/5;
	cRadius = xDistBtwClocks/2 - FT_DispWidth/64;

	/* Download the bitmap data */
	FT_GPU_HAL_WrMem_P(phost, RAM_G,(ft_uint8_t *)&Lena_Bitmap_RawData[Lena_Bitmap_RawData_Header[0].ArrayOffset],
		Lena_Bitmap_RawData_Header[0].Stride*Lena_Bitmap_RawData_Header[0].Height);

	/* Draw gauge with blue as background and read as needle color */

	FT_GPU_CoCmd_Dlstart(phost);
	FT_API_Write_CoCmd(CLEAR_COLOR_RGB(64,64,64));
	FT_API_Write_CoCmd(CLEAR(1,1,1));
	FT_API_Write_CoCmd(COLOR_RGB(255,255,255));
	/* flat effect and default color background */
	xOffset = xDistBtwClocks/2;
	yOffset = cRadius + 5;
	FT_GPU_CoCmd_BgColor(phost, 0x0000ff);
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0x00,0x00));
	FT_GPU_CoCmd_Gauge(phost, xOffset,yOffset,cRadius,OPT_FLAT,5,4,45,100);
	FT_GPU_CoCmd_Text_P(phost,xOffset, (yOffset + cRadius + 6), 26, OPT_CENTER, PSTR("Flat effect"));//text info
	/* 3d effect */
	FT_GPU_CoCmd_BgColor(phost, 0x00ff00);
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0x00,0x00));
	FT_GPU_CoCmd_FgColor(phost,0xff0000);
	xOffset += xDistBtwClocks;
	FT_GPU_CoCmd_Gauge(phost, xOffset,yOffset,cRadius,0,5,1,60,100);
	FT_GPU_CoCmd_Text_P(phost,xOffset, (yOffset + cRadius + 6), 26, OPT_CENTER, PSTR("3d effect"));//text info
	/* no background color */
	FT_GPU_CoCmd_BgColor(phost, 0x00ffff);
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0x00));
	xOffset += xDistBtwClocks;
	FT_GPU_CoCmd_Gauge(phost, xOffset,yOffset,cRadius,OPT_NOBACK,1,6,90,100);
	FT_API_Write_CoCmd(COLOR_A(255));
	FT_GPU_CoCmd_Text_P(phost,xOffset, (yOffset + cRadius + 6), 26, OPT_CENTER, PSTR("No BG"));//text info
	/* No ticks */
	FT_GPU_CoCmd_BgColor(phost, 0xff00ff);
	FT_API_Write_CoCmd(COLOR_RGB(0x00,0xff,0xff));
	xOffset += xDistBtwClocks;
	FT_GPU_CoCmd_Gauge(phost, xOffset,yOffset,cRadius,OPT_NOTICKS,5,4,20,100);
	FT_GPU_CoCmd_Text_P(phost,xOffset, (yOffset + cRadius + 6), 26, OPT_CENTER, PSTR("No Ticks"));//text info
	/* No hands */
	FT_GPU_CoCmd_BgColor(phost, 0x808080);
	FT_API_Write_CoCmd(COLOR_RGB(0x00,0xff,0x00));
	xOffset += xDistBtwClocks;
	FT_GPU_CoCmd_Gauge(phost, xOffset,yOffset,cRadius,OPT_NOHANDS,5,4,55,100);
	FT_API_Write_CoCmd(COLOR_A(255));
	FT_GPU_CoCmd_Text_P(phost,xOffset, (yOffset + cRadius + 6), 26, OPT_CENTER, PSTR("No Hands"));//text info
	/* Bigger gauge */
	yOffset += cRadius + 10;
	cRadius = FT_DispHeight - (2*cRadius + 5 + 10);//calculate radius based on remaining height
	cRadius = (cRadius - 5 - 20)/2;
	xOffset = cRadius + 10;
	yOffset += cRadius + 5;
	FT_GPU_CoCmd_BgColor(phost, 0x808000);
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	FT_GPU_CoCmd_Gauge(phost, xOffset,yOffset,cRadius,OPT_NOPOINTER,5,4,80,100);
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0x00,0x00));
	FT_GPU_CoCmd_Gauge(phost, xOffset,yOffset,cRadius,OPT_NOTICKS | OPT_NOBACK,5,4,30,100);

	xOffset += 2*cRadius + 10;
	/* Lena guage with no background and no ticks */
	FT_API_Write_CoCmd(POINT_SIZE(cRadius*16));
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	FT_API_Write_CoCmd(BEGIN(POINTS));
	FT_API_Write_CoCmd(VERTEX2F(xOffset*16,yOffset*16));
	FT_API_Write_CoCmd(END());
	FT_API_Write_CoCmd(COLOR_A(0xff));
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	FT_API_Write_CoCmd(COLOR_MASK(0,0,0,1));
	FT_API_Write_CoCmd(CLEAR(1,1,1));
	FT_API_Write_CoCmd(BEGIN(POINTS));
	FT_API_Write_CoCmd(POINT_SIZE((cRadius - 2)*16));
	FT_API_Write_CoCmd(VERTEX2F(xOffset*16,yOffset*16));
	FT_API_Write_CoCmd(END());
	FT_API_Write_CoCmd(COLOR_MASK(1,1,1,1));
	FT_API_Write_CoCmd(BLEND_FUNC(DST_ALPHA,ONE_MINUS_DST_ALPHA));
	/* Lena bitmap - scale proportionately wrt output resolution */
	FT_GPU_CoCmd_LoadIdentity(phost);
	FT_GPU_CoCmd_Scale(phost, 65536*2*cRadius/Lena_Bitmap_RawData_Header[0].Width,65536*2*cRadius/Lena_Bitmap_RawData_Header[0].Height);
	FT_GPU_CoCmd_SetMatrix(phost );
	FT_API_Write_CoCmd(BEGIN(BITMAPS));
	FT_API_Write_CoCmd(BITMAP_SOURCE(0));
	FT_API_Write_CoCmd(BITMAP_LAYOUT(Lena_Bitmap_RawData_Header[0].Format,
		Lena_Bitmap_RawData_Header[0].Stride,Lena_Bitmap_RawData_Header[0].Height));
	FT_API_Write_CoCmd(BITMAP_SIZE(BILINEAR,BORDER,BORDER,
		2*cRadius,2*cRadius));
	FT_API_Write_CoCmd(VERTEX2F((xOffset - cRadius)*16,(yOffset - cRadius)*16));
	FT_API_Write_CoCmd(END());
	FT_API_Write_CoCmd(BLEND_FUNC(SRC_ALPHA,ONE_MINUS_SRC_ALPHA));
	FT_GPU_CoCmd_LoadIdentity(phost);
	FT_GPU_CoCmd_SetMatrix(phost );

	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	FT_GPU_CoCmd_Gauge(phost, xOffset,yOffset,cRadius,OPT_NOTICKS | OPT_NOBACK,5,4,30,100);
	FT_API_Write_CoCmd(DISPLAY());
	FT_GPU_CoCmd_Swap(phost);

	/* Wait till coprocessor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
    FT_APP_ENABLE_DELAY();
	}
}

/* API to demonstrate gradient widget */
ft_void_t FT_APP_CoPro_Widget_Gradient()
{
	/*************************************************************************/
	/* Below code demonstrates the usage of gradient function. Gradient func */
	/* can be used to construct three effects - horizontal, vertical and     */
	/* diagonal effects.                                                      */
	/*************************************************************************/
	{
	ft_int16_t wScissor,hScissor,xOffset,yOffset;

	wScissor = ((FT_DispWidth - 4*10)/3);
	hScissor = ((FT_DispHeight - 3*20)/2);
	xOffset = 10;
	yOffset = 20;
	/* Draw gradient  */

	FT_GPU_CoCmd_Dlstart(phost);
	FT_API_Write_CoCmd(CLEAR_COLOR_RGB(64,64,64));
	FT_API_Write_CoCmd(CLEAR(1,1,1));
	FT_API_Write_CoCmd(COLOR_RGB(255,255,255));
	FT_API_Write_CoCmd(SCISSOR_SIZE(wScissor,hScissor));
	/* Horizontal gradient effect */
	FT_API_Write_CoCmd(SCISSOR_XY(xOffset,yOffset));//clip the display
	FT_GPU_CoCmd_Gradient(phost, xOffset,yOffset,0x808080,(xOffset + wScissor),yOffset,0xffff00);
	/* Vertical gradient effect */
	xOffset += wScissor + 10;
	FT_API_Write_CoCmd(SCISSOR_XY(xOffset,yOffset));//clip the display
	FT_GPU_CoCmd_Gradient(phost, xOffset,yOffset,0xff0000,xOffset,(yOffset + hScissor),0x00ff00);
	/* diagonal gradient effect */
	xOffset += wScissor + 10;
	FT_API_Write_CoCmd(SCISSOR_XY(xOffset,yOffset));//clip the display
	FT_GPU_CoCmd_Gradient(phost, xOffset,yOffset,0x800000,(xOffset + wScissor),(yOffset + hScissor),0xffffff);
	/* Diagonal gradient with text info */
	xOffset = 10;
	yOffset += hScissor + 20;
	FT_API_Write_CoCmd(SCISSOR_SIZE(wScissor,30));
	FT_API_Write_CoCmd(SCISSOR_XY(xOffset,(yOffset + hScissor/2 - 15)));//clip the display
	FT_GPU_CoCmd_Gradient(phost, xOffset,(yOffset + hScissor/2 - 15),0x606060,(xOffset + wScissor),(yOffset + hScissor/2 + 15),0x404080);
	FT_GPU_CoCmd_Text_P(phost,(xOffset + wScissor/2), (yOffset + hScissor/2), 28, OPT_CENTER, PSTR("Heading 1"));//text info

	/* Draw horizontal, vertical and diagonal with alpha */
	xOffset += wScissor + 10;
	FT_API_Write_CoCmd(SCISSOR_SIZE(wScissor,hScissor));
	FT_API_Write_CoCmd(SCISSOR_XY(xOffset,yOffset));//clip the display
	FT_GPU_CoCmd_Gradient(phost, xOffset,yOffset,0x808080,(xOffset + wScissor),yOffset,0xffff00);
	wScissor -= 30; hScissor -= 30;
	FT_API_Write_CoCmd(SCISSOR_SIZE(wScissor,hScissor));
	xOffset += 15; yOffset += 15;
	FT_API_Write_CoCmd(SCISSOR_XY(xOffset,yOffset));//clip the display
	FT_GPU_CoCmd_Gradient(phost, xOffset,yOffset,0x800000,xOffset,(yOffset + hScissor),0xffffff);
	wScissor -= 30; hScissor -= 30;
	FT_API_Write_CoCmd(SCISSOR_SIZE(wScissor,hScissor));
	xOffset += 15; yOffset += 15;
	FT_API_Write_CoCmd(SCISSOR_XY(xOffset,yOffset));//clip the display
	FT_GPU_CoCmd_Gradient(phost, xOffset,yOffset,0x606060,(xOffset + wScissor),(yOffset + hScissor),0x404080);

	/* Display the text wrt gradient */
	wScissor = ((FT_DispWidth - 4*10)/3);
	hScissor = ((FT_DispHeight - 3*20)/2);
	xOffset = 10 + wScissor/2;
	yOffset = 20 + hScissor + 5;
	FT_API_Write_CoCmd(SCISSOR_XY(0,0));//set to default values
	FT_API_Write_CoCmd(SCISSOR_SIZE(512,512));
	FT_GPU_CoCmd_Text_P(phost,xOffset,yOffset, 26, OPT_CENTER, PSTR("Horizontal grad"));//text info
	xOffset += wScissor + 10;
	FT_GPU_CoCmd_Text_P(phost,xOffset,yOffset, 26, OPT_CENTER, PSTR("Vertical grad"));//text info
	xOffset += wScissor + 10;
	FT_GPU_CoCmd_Text_P(phost,xOffset,yOffset, 26, OPT_CENTER, PSTR("Diagonal grad"));//text info

	FT_API_Write_CoCmd(DISPLAY());
	FT_GPU_CoCmd_Swap(phost);

	/* Wait till coprocessor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
    FT_APP_ENABLE_DELAY();
	}
}

#define FT_API_COPRO_WIDGET_KEYS_INTERACTIVE_TEXTSIZE (128) // xxx don't make this too big in arduino world

ft_void_t FT_APP_CoPro_Widget_Keys_Interactive()
{
	/*************************************************************************/
	/* Below code demonstrates the usage of keys function. keys function     */
	/* draws buttons with characters given as input parameters. Keys support */
	/* Flat and 3D effects, draw at (x,y) coordinates or center of the display*/
	/* , inbuilt or custom fonts can be used for key display                 */
	/*************************************************************************/
	{
	  ft_int32_t loopflag = 600;
	  ft_int16_t TextFont = 29, ButtonW = 30, ButtonH = 30, yBtnDst = 5, yOffset; // xOffset; xxx
	  ft_char8_t DispText[FT_API_COPRO_WIDGET_KEYS_INTERACTIVE_TEXTSIZE], CurrChar = '|';
	  ft_uint8_t CurrTag = 0,PrevTag = 0,Pendown = 1;
	  ft_int32_t CurrTextIdx = 0;

       while(loopflag --)
       {


       /* Check the user input and then add the characters into array */
       CurrTag = FT_GPU_HAL_Rd8(phost,REG_TOUCH_TAG);
       Pendown = ((FT_GPU_HAL_Rd32(phost,REG_TOUCH_DIRECT_XY)>>31) & 0x01);

       CurrChar = CurrTag;
       if(0 == CurrTag)
       {
              CurrChar = '|';
       }

       /* check whether pwndown has happened */
       if(( 1 == Pendown) && (0 != PrevTag))
       {
              CurrTextIdx++;
              /* clear all the characters after 100 are pressed */
              if(CurrTextIdx > 24)
              {
                     CurrTextIdx = 0;
              }
       }

       FT_GPU_CoCmd_Dlstart(phost);
       FT_API_Write_CoCmd(CLEAR_COLOR_RGB(64,64,64));
       FT_API_Write_CoCmd(CLEAR(1,1,1));
       FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
       /* Draw text entered by user */
       /* make sure the array is a string */
       DispText[CurrTextIdx] = CurrChar;
       DispText[CurrTextIdx + 1] = '\0';

       FT_API_Write_CoCmd(TAG_MASK(0));
       FT_GPU_CoCmd_Text(phost,FT_DispWidth/2, 40, TextFont, OPT_CENTER, DispText);//text info
       FT_API_Write_CoCmd(TAG_MASK(1));


       yOffset = 80 + 10;
       /* Construct a simple keyboard - note that the tags associated with the keys are the character values given in the arguments */
       FT_GPU_CoCmd_FgColor(phost,0x404080);
       FT_GPU_CoCmd_GradColor(phost,0x00ff00);
       FT_GPU_CoCmd_Keys_P(phost, yBtnDst, yOffset, 10*ButtonW, ButtonH, TextFont, (OPT_CENTER | CurrTag), PSTR("qwertyuiop"));
       FT_GPU_CoCmd_GradColor(phost,0x00ffff);
       yOffset += ButtonH + yBtnDst;
       FT_GPU_CoCmd_Keys_P(phost, yBtnDst, yOffset, 10*ButtonW, ButtonH, TextFont, (OPT_CENTER | CurrTag), PSTR("asdfghijkl"));
       FT_GPU_CoCmd_GradColor(phost,0xffff00);
       yOffset += ButtonH + yBtnDst;
       FT_GPU_CoCmd_Keys_P(phost, yBtnDst, yOffset, 10*ButtonW, ButtonH, TextFont, (OPT_CENTER | CurrTag), PSTR("zxcvbnm"));//highlight button z
       yOffset += ButtonH + yBtnDst;
       FT_API_Write_CoCmd(TAG(' '));
       if(' ' == CurrTag)
       {
              FT_GPU_CoCmd_Button_P(phost,yBtnDst, yOffset, 10*ButtonW, ButtonH, TextFont, OPT_CENTER | OPT_FLAT, PSTR( " "));//mandatory to give '\0' at the end to make sure coprocessor understands the string end
       }
       else
       {
              FT_GPU_CoCmd_Button_P(phost,yBtnDst, yOffset, 10*ButtonW, ButtonH, TextFont, OPT_CENTER, PSTR(" "));//mandatory to give '\0' at the end to make sure coprocessor understands the string end
       }
       yOffset = 80 + 10;
       FT_GPU_CoCmd_Keys_P(phost, 11*ButtonW, yOffset, 3*ButtonW, ButtonH, TextFont, (0 | CurrTag), PSTR("789"));
       yOffset += ButtonH + yBtnDst;
       FT_GPU_CoCmd_Keys_P(phost, 11*ButtonW, yOffset, 3*ButtonW, ButtonH, TextFont, (0 | CurrTag), PSTR("456"));
       yOffset += ButtonH + yBtnDst;
       FT_GPU_CoCmd_Keys_P(phost, 11*ButtonW, yOffset, 3*ButtonW, ButtonH, TextFont, (0 | CurrTag), PSTR("123"));
       yOffset += ButtonH + yBtnDst;
       FT_API_Write_CoCmd(COLOR_A(255));
       FT_GPU_CoCmd_Keys(phost, 11*ButtonW, yOffset, 3*ButtonW, ButtonH, TextFont, (0 | CurrTag), "0.");//highlight button 0
       FT_API_Write_CoCmd(DISPLAY());
       FT_GPU_CoCmd_Swap(phost);

       /* Wait till coprocessor completes the operation */
       FT_GPU_HAL_WaitCmdfifo_empty(phost);
       FT_APP_ENABLE_DELAY_MS( 10 );//sleep for 10 ms
       PrevTag = CurrTag;
       }
    }
}

/* API to demonstrate keys widget */
ft_void_t FT_APP_CoPro_Widget_Keys()
{
	/*************************************************************************/
	/* Below code demonstrates the usage of keys function. keys function     */
	/* draws buttons with characters given as input parameters. Keys support */
	/* Flat and 3D effects, draw at (x,y) coordinates or center of the display*/
	/* , inbuilt or custom fonts can be used for key display                 */
	/*************************************************************************/
	{
		ft_int16_t TextFont = 29,ButtonW = 30,ButtonH = 30,yBtnDst = 5,yOffset,xOffset;
#ifdef FT_API_DISPLAY_QVGA
		TextFont = 27;
		ButtonW = 22;
		ButtonH = 22;
		yBtnDst = 3;
#endif

	FT_GPU_CoCmd_Dlstart(phost);
	FT_API_Write_CoCmd(CLEAR_COLOR_RGB(64,64,64));
	FT_API_Write_CoCmd(CLEAR(1,1,1));
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	/* Draw keys with flat effect */
	FT_GPU_CoCmd_FgColor(phost,0x404080);
	FT_GPU_CoCmd_Keys_P(phost, 10, 10, 4*ButtonW, 30, TextFont, OPT_FLAT, PSTR("ABCD"));
	FT_GPU_CoCmd_Text_P(phost,10, 40, 26, 0, PSTR("Flat effect"));//text info
	/* Draw keys with 3d effect */
	FT_GPU_CoCmd_FgColor(phost,0x800000);
	xOffset = 4*ButtonW + 20;
	FT_GPU_CoCmd_Keys_P(phost, xOffset, 10, 4*ButtonW, 30, TextFont, 0, PSTR("ABCD"));
	FT_GPU_CoCmd_Text_P(phost,xOffset, 40, 26, 0, PSTR("3D effect"));//text info
	/* Draw keys with center option */
	FT_GPU_CoCmd_FgColor(phost,0xffff000);
	xOffset += 4*ButtonW + 20;
	FT_GPU_CoCmd_Keys_P(phost, xOffset, 10, (FT_DispWidth - 230), 30, TextFont, OPT_CENTER, PSTR("ABCD"));
	FT_GPU_CoCmd_Text_P(phost,xOffset, 40, 26, 0, PSTR("Option Center"));//text info

	yOffset = 80 + 10;
	/* Construct a simple keyboard - note that the tags associated with the keys are the character values given in the arguments */
	FT_GPU_CoCmd_FgColor(phost,0x404080);
	FT_GPU_CoCmd_GradColor(phost,0x00ff00);
	FT_GPU_CoCmd_Keys_P(phost, yBtnDst, yOffset, 10*ButtonW, ButtonH, TextFont, OPT_CENTER, PSTR("qwertyuiop"));
	FT_GPU_CoCmd_GradColor(phost,0x00ffff);
	yOffset += ButtonH + yBtnDst;
	FT_GPU_CoCmd_Keys_P(phost, yBtnDst, yOffset, 10*ButtonW, ButtonH, TextFont, OPT_CENTER, PSTR("asdfghijkl"));
	FT_GPU_CoCmd_GradColor(phost,0xffff00);
	yOffset += ButtonH + yBtnDst;
	FT_GPU_CoCmd_Keys_P(phost, yBtnDst, yOffset, 10*ButtonW, ButtonH, TextFont, (OPT_CENTER | 'z'), PSTR( "zxcvbnm"));//highlight button z
	yOffset += ButtonH + yBtnDst;
	FT_GPU_CoCmd_Button_P(phost,yBtnDst, yOffset, 10*ButtonW, ButtonH, TextFont, OPT_CENTER, PSTR(" "));//mandatory to give '\0' at the end to make sure coprocessor understands the string end
	yOffset = 80 + 10;
	FT_GPU_CoCmd_Keys_P(phost, 11*ButtonW, yOffset, 3*ButtonW, ButtonH, TextFont, 0, PSTR("789"));
	yOffset += ButtonH + yBtnDst;
	FT_GPU_CoCmd_Keys_P(phost, 11*ButtonW, yOffset, 3*ButtonW, ButtonH, TextFont, 0, PSTR("456"));
	yOffset += ButtonH + yBtnDst;
	FT_GPU_CoCmd_Keys_P(phost, 11*ButtonW, yOffset, 3*ButtonW, ButtonH, TextFont, 0, PSTR("123"));
	yOffset += ButtonH + yBtnDst;
	FT_API_Write_CoCmd(COLOR_A(255));
	FT_GPU_CoCmd_Keys_P(phost, 11*ButtonW, yOffset, 3*ButtonW, ButtonH, TextFont, (0 | '0'), PSTR("0."));//highlight button 0
	FT_API_Write_CoCmd(DISPLAY());
	FT_GPU_CoCmd_Swap(phost);

	/* Wait till coprocessor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
    FT_APP_ENABLE_DELAY();
	}
}
/* API to demonstrate progress bar widget */
ft_void_t FT_APP_CoPro_Widget_Progressbar()
{
	/*************************************************************************/
	/* Below code demonstrates the usage of progress function. Progress func */
	/* draws process bar with fgcolor for the % completion and bgcolor for   */
	/* % remaining. Progress bar supports flat and 3d effets                 */
	/*************************************************************************/
	{
		ft_int16_t xOffset,yOffset,yDist = FT_DispWidth/12,ySz = FT_DispWidth/24;

	FT_GPU_CoCmd_Dlstart(phost);
	FT_API_Write_CoCmd(CLEAR_COLOR_RGB(64,64,64));
	FT_API_Write_CoCmd(CLEAR(1,1,1));
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	/* Draw progress bar with flat effect */
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	FT_GPU_CoCmd_BgColor(phost, 0x404080);
	FT_GPU_CoCmd_Progress(phost, 20, 10, 120, 20, OPT_FLAT, 50, 100);//note that h/2 will be added on both sides of the progress bar
	FT_API_Write_CoCmd(COLOR_A(255));
	FT_GPU_CoCmd_Text_P(phost,20, 40, 26, 0, PSTR("Flat effect"));//text info
	/* Draw progress bar with 3d effect */
	FT_API_Write_CoCmd(COLOR_RGB(0x00,0xff,0x00));
	FT_GPU_CoCmd_BgColor(phost, 0x800000);
	FT_GPU_CoCmd_Progress(phost, 180, 10, 120, 20, 0, 75, 100);
	FT_API_Write_CoCmd(COLOR_A(255));
	FT_GPU_CoCmd_Text_P(phost,180, 40, 26, 0, PSTR("3D effect"));//text info
	/* Draw progress bar with 3d effect and string on top */
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0x00,0x00));
	FT_GPU_CoCmd_BgColor(phost, 0x000080);
	FT_GPU_CoCmd_Progress(phost, 30, 60, 120, 30, 0, 19660, 65535);
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	FT_GPU_CoCmd_Text_P(phost,78, 68, 26, 0, PSTR("30 %"));//text info

	xOffset = 20;yOffset = 120;
	FT_API_Write_CoCmd(COLOR_RGB(0x00,0xa0,0x00));
	FT_GPU_CoCmd_BgColor(phost, 0x800000);
	FT_GPU_CoCmd_Progress(phost, xOffset, yOffset, 150, ySz, 0, 10, 100);
	FT_GPU_CoCmd_BgColor(phost, 0x000080);
	yOffset += yDist;
	FT_GPU_CoCmd_Progress(phost, xOffset, yOffset, 150, ySz, 0, 40, 100);
	FT_GPU_CoCmd_BgColor(phost, 0xffff00);
	yOffset += yDist;
	FT_GPU_CoCmd_Progress(phost, xOffset, yOffset, 150, ySz, 0, 70, 100);
	FT_GPU_CoCmd_BgColor(phost, 0x808080);
	yOffset += yDist;
	FT_GPU_CoCmd_Progress(phost, xOffset, yOffset, 150, ySz, 0, 90, 100);

	FT_GPU_CoCmd_Text_P(phost,xOffset + 180, 80, 26, 0, PSTR("40 % TopBottom"));//text info
	FT_GPU_CoCmd_Progress(phost, xOffset + 180, 100, ySz, 150, 0, 40, 100);

	FT_API_Write_CoCmd(DISPLAY());
	FT_GPU_CoCmd_Swap(phost);

	/* Wait till coprocessor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
    FT_APP_ENABLE_DELAY();
	}
}
/* API to demonstrate scroll widget */
ft_void_t FT_APP_CoPro_Widget_Scroll()
{
	/*************************************************************************/
	/* Below code demonstrates the usage of scroll function. Scroll function */
	/* draws scroll bar with fgcolor for inner color and current location and*/
	/* can be given by val parameter */
	/*************************************************************************/
	{
		ft_int16_t xOffset,yOffset,xDist = FT_DispWidth/12,yDist = FT_DispWidth/12,wSz;


	FT_GPU_CoCmd_Dlstart(phost);
	FT_API_Write_CoCmd(CLEAR_COLOR_RGB(64,64,64));
	FT_API_Write_CoCmd(CLEAR(1,1,1));
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	/* Draw scroll bar with flat effect */
	FT_GPU_CoCmd_FgColor(phost,0xffff00);
	FT_GPU_CoCmd_BgColor(phost, 0x404080);
	FT_GPU_CoCmd_Scrollbar(phost, 20, 10, 120, 8, OPT_FLAT, 20, 30, 100);//note that h/2 size will be added on both sides of the progress bar
	FT_API_Write_CoCmd(COLOR_A(255));
	FT_GPU_CoCmd_Text_P(phost,20, 40, 26, 0, PSTR("Flat effect"));//text info
	/* Draw scroll bar with 3d effect */
	FT_GPU_CoCmd_FgColor(phost,0x00ff00);
	FT_GPU_CoCmd_BgColor(phost, 0x800000);
	FT_GPU_CoCmd_Scrollbar(phost, 180, 10, 120, 8, 0, 20, 30, 100);
	FT_API_Write_CoCmd(COLOR_A(255));
	FT_GPU_CoCmd_Text_P(phost,180, 40, 26, 0, PSTR("3D effect"));//text info

	xOffset = 20;
	yOffset = 120;
	wSz = ((FT_DispWidth/2) - 40);
	/* Draw horizontal scroll bars */
	FT_GPU_CoCmd_FgColor(phost,0x00a000);
	FT_GPU_CoCmd_BgColor(phost, 0x800000);
	FT_GPU_CoCmd_Scrollbar(phost, xOffset, yOffset, wSz, 8, 0, 10, 30, 100);
	FT_GPU_CoCmd_BgColor(phost, 0x000080);
	yOffset += yDist;
	FT_GPU_CoCmd_Scrollbar(phost, xOffset, yOffset, wSz, 8, 0, 30, 30, 100);
	FT_API_Write_CoCmd(COLOR_A(255));
	FT_GPU_CoCmd_BgColor(phost, 0xffff00);
	yOffset += yDist;
	FT_GPU_CoCmd_Scrollbar(phost, xOffset, yOffset, wSz, 8, 0, 50, 30, 100);
	FT_GPU_CoCmd_BgColor(phost, 0x808080);
	yOffset += yDist;
	FT_GPU_CoCmd_Scrollbar(phost, xOffset, yOffset, wSz, 8, 0, 70, 30, 100);

	xOffset = (FT_DispWidth/2) + 40;
	yOffset = 80;
	wSz = (FT_DispHeight - 100);
	/* draw vertical scroll bars */
	FT_GPU_CoCmd_BgColor(phost, 0x800000);
	FT_GPU_CoCmd_Scrollbar(phost, xOffset, yOffset, 8, wSz, 0, 10, 30, 100);
	FT_GPU_CoCmd_BgColor(phost, 0x000080);
	xOffset += xDist;
	FT_GPU_CoCmd_Scrollbar(phost, xOffset, yOffset, 8, wSz, 0, 30, 30, 100);
	FT_API_Write_CoCmd(COLOR_A(255));
	FT_GPU_CoCmd_BgColor(phost, 0xffff00);
	xOffset += xDist;
	FT_GPU_CoCmd_Scrollbar(phost, xOffset, yOffset, 8, wSz, 0, 50, 30, 100);
	FT_GPU_CoCmd_BgColor(phost, 0x808080);
	xOffset += xDist;
	FT_GPU_CoCmd_Scrollbar(phost, xOffset, yOffset, 8, wSz, 0, 70, 30, 100);

	FT_API_Write_CoCmd(DISPLAY());
	FT_GPU_CoCmd_Swap(phost);

	/* Wait till coprocessor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
    FT_APP_ENABLE_DELAY();
	}
}
/* API to demonstrate slider widget */
ft_void_t FT_APP_CoPro_Widget_Slider()
{
	/*************************************************************************/
	/* Below code demonstrates the usage of slider function. Slider function */
	/* draws slider bar with fgcolor for inner color and bgcolor for the knob*/
	/* , contains input parameter for position of the knob                   */
	/*************************************************************************/
	{
		ft_int16_t xOffset,yOffset,xDist = FT_DispWidth/12,yDist = FT_DispWidth/12,wSz;

	FT_GPU_CoCmd_Dlstart(phost);
	FT_API_Write_CoCmd(CLEAR_COLOR_RGB(64,64,64));
	FT_API_Write_CoCmd(CLEAR(1,1,1));
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	/* Draw scroll bar with flat effect */
	FT_GPU_CoCmd_FgColor(phost,0xffff00);
	FT_GPU_CoCmd_BgColor(phost, 0x000080);
	FT_GPU_CoCmd_Slider(phost, 20, 10, 120, 10, OPT_FLAT, 30, 100);//note that h/2 size will be added on both sides of the progress bar
	FT_API_Write_CoCmd(COLOR_A(255));
	FT_GPU_CoCmd_Text_P(phost,20, 40, 26, 0, PSTR("Flat effect"));//text info
	/* Draw scroll bar with 3d effect */
	FT_GPU_CoCmd_FgColor(phost,0x00ff00);
	FT_GPU_CoCmd_BgColor(phost, 0x800000);
	FT_GPU_CoCmd_Slider(phost, 180, 10, 120, 10, 0, 50, 100);
	FT_API_Write_CoCmd(COLOR_A(255));
	FT_GPU_CoCmd_Text_P(phost,180, 40, 26, 0, PSTR("3D effect"));//text info

	xOffset = 20;
	yOffset = 120;
	wSz = ((FT_DispWidth/2) - 40);
	/* Draw horizontal slider bars */
	FT_GPU_CoCmd_FgColor(phost,0x00a000);
	FT_GPU_CoCmd_BgColor(phost, 0x800000);
	FT_API_Write_CoCmd(COLOR_RGB(41,1,5));
	FT_GPU_CoCmd_Slider(phost, xOffset, yOffset, wSz, 10, 0, 10, 100);
	FT_API_Write_CoCmd(COLOR_RGB(11,7,65));
	FT_GPU_CoCmd_BgColor(phost, 0x000080);
	yOffset += yDist;
	FT_GPU_CoCmd_Slider(phost, xOffset, yOffset, wSz, 10, 0, 30, 100);
	FT_API_Write_CoCmd(COLOR_A(255));
	FT_GPU_CoCmd_BgColor(phost, 0xffff00);
	FT_API_Write_CoCmd(COLOR_RGB(87,94,9));
	yOffset += yDist;
	FT_GPU_CoCmd_Slider(phost, xOffset, yOffset, wSz, 10, 0, 50, 100);
	FT_GPU_CoCmd_BgColor(phost, 0x808080);
	FT_API_Write_CoCmd(COLOR_RGB(51,50,52));
	yOffset += yDist;
	FT_GPU_CoCmd_Slider(phost, xOffset, yOffset, wSz, 10, 0, 70, 100);

	xOffset = (FT_DispWidth/2) + 40;
	yOffset = 80;
	wSz = (FT_DispHeight - 100);
	/* draw vertical slider bars */
	FT_GPU_CoCmd_BgColor(phost, 0x800000);
	FT_GPU_CoCmd_Slider(phost, xOffset, yOffset, 10, wSz, 0, 10, 100);
	FT_GPU_CoCmd_BgColor(phost, 0x000080);
	xOffset += xDist;
	FT_GPU_CoCmd_Slider(phost, xOffset, yOffset, 10, wSz, 0, 30, 100);
	FT_API_Write_CoCmd(COLOR_A(255));
	FT_GPU_CoCmd_BgColor(phost, 0xffff00);
	xOffset += xDist;
	FT_GPU_CoCmd_Slider(phost, xOffset, yOffset, 10, wSz, 0, 50, 100);
	FT_GPU_CoCmd_BgColor(phost, 0x808080);
	xOffset += xDist;
	FT_GPU_CoCmd_Slider(phost, xOffset, yOffset, 10, wSz, 0, 70, 100);

	FT_API_Write_CoCmd(DISPLAY());
	FT_GPU_CoCmd_Swap(phost);

	/* Wait till coprocessor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
    FT_APP_ENABLE_DELAY();
	}
}
/*API to demonstrate dial widget */
ft_void_t FT_APP_CoPro_Widget_Dial()
{
	/*************************************************************************/
	/* Below code demonstrates the usage of dial function. Dial function     */
	/* draws rounded bar with fgcolor for knob color and colorrgb for pointer*/
	/* , contains input parameter for angle of the pointer                   */
	/*************************************************************************/
	{

	FT_GPU_CoCmd_Dlstart(phost);
	FT_API_Write_CoCmd(CLEAR_COLOR_RGB(64,64,64));
	FT_API_Write_CoCmd(CLEAR(1,1,1));
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	/* Draw dial with flat effect */
	FT_GPU_CoCmd_FgColor(phost,0xffff00);
	FT_GPU_CoCmd_BgColor(phost, 0x000080);
	FT_GPU_CoCmd_Dial(phost, 50, 50, 40, OPT_FLAT, 0.2*65535);//20%
	FT_API_Write_CoCmd(COLOR_A(255));
	FT_GPU_CoCmd_Text_P(phost,15, 90, 26, 0, PSTR("Flat effect"));//text info
	/* Draw dial with 3d effect */
	FT_GPU_CoCmd_FgColor(phost,0x00ff00);
	FT_GPU_CoCmd_BgColor(phost, 0x800000);
	FT_GPU_CoCmd_Dial(phost, 140, 50, 40, 0, 0.45*65535);//45%
	FT_API_Write_CoCmd(COLOR_A(255));
	FT_GPU_CoCmd_Text_P(phost,105, 90, 26, 0, PSTR("3D effect"));//text info

	/* Draw increasing dials horizontally */
	FT_GPU_CoCmd_FgColor(phost,0x800000);
	FT_API_Write_CoCmd(COLOR_RGB(41,1,5));
	FT_GPU_CoCmd_Dial(phost, 30, 160, 20, 0, 0.30*65535);
	FT_GPU_CoCmd_Text_P(phost,20, 180, 26, 0, PSTR("30 %"));//text info
	FT_API_Write_CoCmd(COLOR_RGB(11,7,65));
	FT_GPU_CoCmd_FgColor(phost,0x000080);
	FT_GPU_CoCmd_Dial(phost, 100, 160, 40, 0, 0.45*65535);
	FT_API_Write_CoCmd(COLOR_A(255));
	FT_GPU_CoCmd_Text_P(phost,90, 200, 26, 0, PSTR("45 %"));//text info
	FT_GPU_CoCmd_FgColor(phost,0xffff00);
	FT_API_Write_CoCmd(COLOR_RGB(87,94,9));
	FT_GPU_CoCmd_Dial(phost, 210, 160, 60, 0, 0.60*65535);
	FT_GPU_CoCmd_Text_P(phost,200, 220, 26, 0, PSTR("60 %"));//text info
	FT_GPU_CoCmd_FgColor(phost,0x808080);

	FT_API_Write_CoCmd(DISPLAY());
	FT_GPU_CoCmd_Swap(phost);

	/* Wait till coprocessor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
    FT_APP_ENABLE_DELAY();
	}
}
/* API to demonstrate toggle widget */
ft_void_t FT_APP_CoPro_Widget_Toggle()
{
	/*************************************************************************/
	/* Below code demonstrates the usage of toggle function. Toggle function */
	/* draws line with inside knob to choose between left and right. Toggle  */
	/* inside color can be changed by bgcolor and knob color by fgcolor. Left*/
	/* right texts can be written and the color of the text can be changed by*/
	/* colorrgb graphics function                                            */
	/*************************************************************************/
	{
		ft_int16_t xOffset,yOffset,yDist = 40;

	FT_GPU_CoCmd_Dlstart(phost);
	FT_API_Write_CoCmd(CLEAR_COLOR_RGB(64,64,64));
	FT_API_Write_CoCmd(CLEAR(1,1,1));
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	/* Draw toggle with flat effect */
	FT_GPU_CoCmd_FgColor(phost,0xffff00);
	FT_GPU_CoCmd_BgColor(phost, 0x000080);

	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));

	FT_GPU_CoCmd_Toggle(phost, 40, 10, 30, 27, OPT_FLAT, 0.5*65535,"no""\xff""yes");//circle radius will be extended on both the horizontal sides
	FT_API_Write_CoCmd(COLOR_A(255));
	FT_GPU_CoCmd_Text_P(phost,40, 40, 26, 0, PSTR("Flat effect"));//text info
	/* Draw toggle bar with 3d effect */
	FT_GPU_CoCmd_FgColor(phost,0x00ff00);
	FT_GPU_CoCmd_BgColor(phost, 0x800000);
	FT_GPU_CoCmd_Toggle(phost, 140, 10, 30, 27, 0, 1*65535,"stop""\xff""run");
	FT_API_Write_CoCmd(COLOR_A(255));
	FT_GPU_CoCmd_Text_P(phost,140, 40, 26, 0, PSTR("3D effect"));//text info

	xOffset = 40;
	yOffset = 80;
	/* Draw horizontal toggle bars */
	FT_GPU_CoCmd_BgColor(phost, 0x800000);
	FT_GPU_CoCmd_FgColor(phost,0x410105);
	FT_GPU_CoCmd_Toggle(phost, xOffset, yOffset, 30, 27, 0, 0*65535,"-ve""\xff""+ve");
	FT_GPU_CoCmd_FgColor(phost,0x0b0721);
	FT_GPU_CoCmd_BgColor(phost, 0x000080);
	yOffset += yDist;
	FT_GPU_CoCmd_Toggle(phost, xOffset, yOffset, 30, 27, 0, 0.25*65535,"zero""\xff""one");
	FT_GPU_CoCmd_BgColor(phost, 0xffff00);
	FT_GPU_CoCmd_FgColor(phost,0x575e1b);
	FT_API_Write_CoCmd(COLOR_RGB(0,0,0));
	yOffset += yDist;
	FT_GPU_CoCmd_Toggle(phost, xOffset, yOffset, 30, 27, 0, 0.5*65535,"exit""\xff""init");
	FT_GPU_CoCmd_BgColor(phost, 0x808080);
	FT_GPU_CoCmd_FgColor(phost,0x333234);
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	yOffset += yDist;
	FT_GPU_CoCmd_Toggle(phost, xOffset, yOffset, 30, 27, 0, 0.75*65535,"off""\xff""on");

	FT_API_Write_CoCmd(DISPLAY());
	FT_GPU_CoCmd_Swap(phost);

	/* Wait till coprocessor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
    FT_APP_ENABLE_DELAY();
	}
}

/* API to demonstrate spinner widget */
ft_void_t FT_APP_CoPro_Widget_Spinner()
{
	/*************************************************************************/
	/* Below code demonstrates the usage of spinner function. Spinner func   */
	/* will wait untill stop command is sent from the mcu. Spinner has option*/
	/* for displaying points in circle fashion or in a line fashion.         */
	/*************************************************************************/

	FT_GPU_CoCmd_Dlstart(phost);
	FT_API_Write_CoCmd(CLEAR_COLOR_RGB(64,64,64));
	FT_API_Write_CoCmd(CLEAR(1,1,1));
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	FT_GPU_CoCmd_Text_P(phost,(FT_DispWidth/2), 20, 27, OPT_CENTER, PSTR("Spinner circle"));
	FT_GPU_CoCmd_Text_P(phost,(FT_DispWidth/2), 80, 27, OPT_CENTER, PSTR("Please Wait ..."));
	FT_GPU_CoCmd_Spinner(phost, (FT_DispWidth/2),(FT_DispHeight/2),0,1);//style 0 and scale 0

	/* Wait till coprocessor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
	FT_APP_ENABLE_DELAY();

	/**************************** spinner with style 1 and scale 1 *****************************************************/


	FT_GPU_CoCmd_Dlstart(phost);
	FT_API_Write_CoCmd(CLEAR_COLOR_RGB(64,64,64));
	FT_API_Write_CoCmd(CLEAR(1,1,1));
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	FT_GPU_CoCmd_Text_P(phost,(FT_DispWidth/2), 20, 27, OPT_CENTER, PSTR("Spinner line"));
	FT_GPU_CoCmd_Text_P(phost,(FT_DispWidth/2), 80, 27, OPT_CENTER, PSTR("Please Wait ..."));
	FT_API_Write_CoCmd(COLOR_RGB(0x00,0x00,0x80));
	FT_GPU_CoCmd_Spinner(phost, (FT_DispWidth/2),(FT_DispHeight/2),1,1);//style 1 and scale 1

	/* Wait till coprocessor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
	FT_APP_ENABLE_DELAY();

	FT_GPU_CoCmd_Dlstart(phost);
	FT_API_Write_CoCmd(CLEAR_COLOR_RGB(64,64,64));
	FT_API_Write_CoCmd(CLEAR(1,1,1));
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	FT_GPU_CoCmd_Text_P(phost,(FT_DispWidth/2), 20, 27, OPT_CENTER, PSTR("Spinner clockhand"));
	FT_GPU_CoCmd_Text_P(phost,(FT_DispWidth/2), 80, 27, OPT_CENTER, PSTR("Please Wait ..."));
	FT_API_Write_CoCmd(COLOR_RGB(0x80,0x00,0x00));
	FT_GPU_CoCmd_Spinner(phost, (FT_DispWidth/2),((FT_DispHeight/2) + 20),2,1);//style 2 scale 1

	/* Wait till coprocessor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
	FT_APP_ENABLE_DELAY();

	FT_GPU_CoCmd_Dlstart(phost);
	FT_API_Write_CoCmd(CLEAR_COLOR_RGB(64,64,64));
	FT_API_Write_CoCmd(CLEAR(1,1,1));
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	FT_GPU_CoCmd_Text_P(phost,(FT_DispWidth/2), 20, 27, OPT_CENTER, PSTR("Spinner dual dots"));
	FT_GPU_CoCmd_Text_P(phost,(FT_DispWidth/2), 80, 27, OPT_CENTER, PSTR("Please Wait ..."));
	FT_API_Write_CoCmd(COLOR_RGB(0x80,0x00,0x00));
	FT_GPU_CoCmd_Spinner(phost, (FT_DispWidth/2),((FT_DispHeight/2) + 20),3,1);//style 3 scale 0

	/* Wait till coprocessor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
	FT_APP_ENABLE_DELAY();

	/* Send the stop command */
	FT_GPU_HAL_WrCmd32(phost,  CMD_STOP);
	/* Update the command buffer pointers - both read and write pointers */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
	FT_APP_ENABLE_DELAY();
}
#endif

#ifdef FT_APP_ENABLE_APIS_SET4
/* API to demonstrate screen saver widget - playing of bitmap via macro0 */
ft_void_t FT_APP_CoPro_Screensaver()
{
	/*************************************************************************/
	/* Below code demonstrates the usage of screensaver function. Screen     */
	/* saver modifies macro0 with the vertex2f command.                      */
	/* MCU can display any static display list at the background with the    */
	/* changing bitmap                                                       */
	/*************************************************************************/

	/* Download the bitmap data */
	FT_GPU_HAL_WrMem_P(phost, RAM_G, &(Lena_Bitmap_RawData[Lena_Bitmap_RawData_Header[0].ArrayOffset]), \
	Lena_Bitmap_RawData_Header[0].Stride*Lena_Bitmap_RawData_Header[0].Height);

	/* Send command screen saver */

	FT_API_Write_CoCmd( CMD_SCREENSAVER);//screen saver command will continuously update the macro0 with vertex2f command
	/* Copy the display list */
	FT_GPU_CoCmd_Dlstart(phost);
	FT_API_Write_CoCmd(CLEAR_COLOR_RGB(0,0,0x80));
	FT_API_Write_CoCmd(CLEAR(1,1,1));
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	/* lena bitmap */
	FT_GPU_CoCmd_LoadIdentity(phost);
	FT_GPU_CoCmd_Scale(phost, 3*65536,3*65536);//scale the bitmap 3 times on both sides
	FT_GPU_CoCmd_SetMatrix(phost );
	FT_API_Write_CoCmd(BEGIN(BITMAPS));
	FT_API_Write_CoCmd(BITMAP_SOURCE(0));
	FT_API_Write_CoCmd(BITMAP_LAYOUT(Lena_Bitmap_RawData_Header[0].Format,
		Lena_Bitmap_RawData_Header[0].Stride,Lena_Bitmap_RawData_Header[0].Height));
	FT_API_Write_CoCmd(BITMAP_SIZE(BILINEAR,BORDER,BORDER,
		Lena_Bitmap_RawData_Header[0].Width*3,Lena_Bitmap_RawData_Header[0].Height*3));
	FT_API_Write_CoCmd(MACRO(0));
	FT_API_Write_CoCmd(END());
	/* Display the text */
	FT_GPU_CoCmd_LoadIdentity(phost);
	FT_GPU_CoCmd_SetMatrix(phost );
	FT_GPU_CoCmd_Text_P(phost,(FT_DispWidth/2), (FT_DispHeight/2), 27, OPT_CENTER, PSTR("Screen Saver ..."));
	FT_GPU_CoCmd_MemSet(phost, (RAM_G + 3200), 0xff, (160L*2*120));
	FT_API_Write_CoCmd(DISPLAY());
	FT_GPU_CoCmd_Swap(phost);

	/* Wait till co-processor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
	FT_APP_ENABLE_DELAY_MS(10000);

	/* Send the stop command */
	FT_GPU_HAL_WrCmd32(phost,  CMD_STOP);
	/* Update the command buffer pointers - both read and write pointers */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);

}
/* Sample app to demonstrate snapshot widget/functionality */
ft_void_t FT_APP_CoPro_Snapshot()
{
	/*************************************************************************/
	/* Below code demonstrates the usage of snapshot function. Snapshot      */
	/* captures the present screen and dumps into bitmap with colour formats */
	/* argb4.                                                                */
	/*************************************************************************/

	ft_uint16_t WriteByte = 0;

	/* fadeout before switching off the pclock */
	FT_API_fadeout();

	/* Switch off the lcd */
	FT_GPU_HAL_Wr8(phost, REG_GPIO, 0x7f);
	vTaskDelay( 100 / portTICK_PERIOD_MS );//sleep for 100 ms

	/* Disable the pclock */
	FT_GPU_HAL_Wr8(phost, REG_PCLK,WriteByte);
	/* Configure the resolution to 160x120 dimension */
	WriteByte = 160;
	FT_GPU_HAL_Wr16(phost, REG_HSIZE,WriteByte);
	WriteByte = 120;
	FT_GPU_HAL_Wr16(phost, REG_VSIZE,WriteByte);

	/* Construct screen shot for snapshot */

	FT_GPU_CoCmd_Dlstart(phost);
	FT_API_Write_CoCmd(CLEAR_COLOR_RGB(0,0,0));
	FT_API_Write_CoCmd(CLEAR(1,1,1));
	FT_API_Write_CoCmd(COLOR_RGB(28,20,99));
	/* captured snapshot */
	FT_API_Write_CoCmd(BEGIN(POINTS));
	FT_API_Write_CoCmd(COLOR_A(128));
	FT_API_Write_CoCmd(POINT_SIZE(20*16));
	FT_API_Write_CoCmd(VERTEX2F(0*16,0*16));
	FT_API_Write_CoCmd(POINT_SIZE(25*16));
	FT_API_Write_CoCmd(VERTEX2F(20*16,10*16));
	FT_API_Write_CoCmd(POINT_SIZE(30*16));
	FT_API_Write_CoCmd(VERTEX2F(40*16,20*16));
	FT_API_Write_CoCmd(POINT_SIZE(35*16));
	FT_API_Write_CoCmd(VERTEX2F(60*16,30*16));
	FT_API_Write_CoCmd(POINT_SIZE(40*16));
	FT_API_Write_CoCmd(VERTEX2F(80*16,40*16));
	FT_API_Write_CoCmd(POINT_SIZE(45*16));
	FT_API_Write_CoCmd(VERTEX2F(100*16,50*16));
	FT_API_Write_CoCmd(POINT_SIZE(50*16));
	FT_API_Write_CoCmd(VERTEX2F(120*16,60*16));
	FT_API_Write_CoCmd(POINT_SIZE(55*16));
	FT_API_Write_CoCmd(VERTEX2F(140*16,70*16));
	FT_API_Write_CoCmd(POINT_SIZE(60*16));
	FT_API_Write_CoCmd(VERTEX2F(160*16,80*16));
	FT_API_Write_CoCmd(POINT_SIZE(65*16));
	FT_API_Write_CoCmd(VERTEX2F(0*16,120*16));
	FT_API_Write_CoCmd(POINT_SIZE(70*16));
	FT_API_Write_CoCmd(VERTEX2F(160*16,0*16));
	FT_API_Write_CoCmd(END());//display the bitmap at the center of the display
	FT_API_Write_CoCmd(COLOR_A(255));
	FT_API_Write_CoCmd(COLOR_RGB(32,32,32));
	FT_GPU_CoCmd_Text_P(phost,80, 60, 26, OPT_CENTER, PSTR("Points"));

	FT_API_Write_CoCmd(DISPLAY());
	FT_GPU_CoCmd_Swap(phost);

	/* Wait till co-processor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);

	vTaskDelay( 100 / portTICK_PERIOD_MS ); //timeout for snapshot to be performed by co-processor

	/* Take snap shot of the current screen */
	FT_GPU_HAL_WrCmd32(phost, CMD_SNAPSHOT);
	FT_GPU_HAL_WrCmd32(phost, 3200);//store the rgb content at location 3200

	//timeout for snapshot to be performed by co-processor

	/* Wait till coprocessor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);

	vTaskDelay( 100 / portTICK_PERIOD_MS ); //timeout for snapshot to be performed by co-processor

	/* reconfigure the resolution wrt configuration */
	WriteByte = FT_DispWidth;
	FT_GPU_HAL_Wr16(phost, REG_HSIZE,WriteByte);
	WriteByte = FT_DispHeight;
	FT_GPU_HAL_Wr16(phost, REG_VSIZE,WriteByte);

	FT_GPU_CoCmd_Dlstart(phost);
	FT_API_Write_CoCmd(CLEAR_COLOR_RGB(0xff,0xff,0xff));
	FT_API_Write_CoCmd(CLEAR(1,1,1));
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	/* captured snapshot */
	FT_API_Write_CoCmd(BEGIN(BITMAPS));
	FT_API_Write_CoCmd(BITMAP_SOURCE(3200));
	FT_API_Write_CoCmd(BITMAP_LAYOUT(ARGB4,160*2,120));
	FT_API_Write_CoCmd(BITMAP_SIZE(BILINEAR,BORDER,BORDER,160,120));
	FT_API_Write_CoCmd(VERTEX2F(((FT_DispWidth - 160)/2)*16,((FT_DispHeight - 120)/2)*16));
	FT_API_Write_CoCmd(END());//display the bitmap at the center of the display
	/* Display the text info */
	FT_API_Write_CoCmd(COLOR_RGB(32,32,32));
	FT_GPU_CoCmd_Text_P(phost,(FT_DispWidth/2), 40, 27, OPT_CENTER, PSTR("Snap shot"));

	FT_API_Write_CoCmd(DISPLAY());
	FT_GPU_CoCmd_Swap(phost);

	/* Wait till coprocessor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);

	/* reenable the pclock */
	WriteByte = FT_DispPCLK;
	FT_GPU_HAL_Wr8(phost, REG_PCLK,WriteByte);
	vTaskDelay( 60 / portTICK_PERIOD_MS ); //sleep for 60 ms

	/* Power on the LCD */
	FT_GPU_HAL_Wr8(phost, REG_GPIO, 0xff);
	vTaskDelay( 200 / portTICK_PERIOD_MS ); // give some time for the lcd to switch on - hack for one particular panel

	/* set the display pwm back to 128 */
	FT_API_fadein();

    FT_APP_ENABLE_DELAY();
}
/* API to demonstrate sketch widget */
ft_void_t FT_APP_CoPro_Sketch()
{
	/*************************************************************************/
	/* Below code demonstrates the usage of sketch function. Sketch function */
	/* draws line for pen movement. Skecth supports L1 and L8 output formats */
	/* Sketch draws till stop command is executed.                           */
	/*************************************************************************/

	ft_int16_t BorderSz = 40;
    ft_uint32_t MemZeroSz;
	/* Send command sketch */

    MemZeroSz = 1L*(FT_DispWidth - 2*BorderSz)*(FT_DispHeight - 2*BorderSz);
	FT_GPU_CoCmd_MemZero(phost, RAM_G,MemZeroSz);
	FT_GPU_CoCmd_Sketch(phost, BorderSz,BorderSz,(FT_DispWidth - 2*BorderSz),(FT_DispHeight - 2*BorderSz),0,L1);//sketch in L1 format
	/* Display the sketch */
	FT_GPU_CoCmd_Dlstart(phost);
	FT_API_Write_CoCmd(CLEAR_COLOR_RGB(0x80,0,0x00));
	FT_API_Write_CoCmd(CLEAR(1,1,1));
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	FT_API_Write_CoCmd(SCISSOR_SIZE((FT_DispWidth - 2*BorderSz),(FT_DispHeight - 2*BorderSz)));
	FT_API_Write_CoCmd(SCISSOR_XY(BorderSz,BorderSz));
	FT_API_Write_CoCmd(CLEAR_COLOR_RGB(0xff,0xff,0xff));
	FT_API_Write_CoCmd(CLEAR(1,1,1));

	FT_API_Write_CoCmd(SCISSOR_SIZE(512,512));
	FT_API_Write_CoCmd(SCISSOR_XY(0,0));
	FT_API_Write_CoCmd(COLOR_RGB(0,0,0));
	/* L1 bitmap display */
	FT_API_Write_CoCmd(BEGIN(BITMAPS));
	FT_API_Write_CoCmd(BITMAP_SOURCE(0));
	FT_API_Write_CoCmd(BITMAP_LAYOUT(L1,(FT_DispWidth - 2*BorderSz)/8,(FT_DispHeight - 2*BorderSz)));
	FT_API_Write_CoCmd(BITMAP_SIZE(BILINEAR,BORDER,BORDER,(FT_DispWidth - 2*BorderSz),(FT_DispHeight - 2*BorderSz)));
	FT_API_Write_CoCmd(VERTEX2F(BorderSz*16,BorderSz*16));
	FT_API_Write_CoCmd(END());
	/* Display the text */
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	FT_GPU_CoCmd_Text_P(phost,(FT_DispWidth/2), 20, 27, OPT_CENTER, PSTR("Sketch L1"));

	FT_API_Write_CoCmd(DISPLAY());
	FT_GPU_CoCmd_Swap(phost);

	/* Wait till coprocessor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);

	FT_APP_ENABLE_DELAY_MS(5000);

	/* Send the stop command */
	FT_GPU_HAL_WrCmd32(phost,  CMD_STOP);
	/* Update the command buffer pointers - both read and write pointers */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);

	/* Sketch L8 format */

	/* Send command sketch */

	FT_GPU_CoCmd_MemZero(phost, RAM_G,MemZeroSz);
	FT_GPU_CoCmd_Sketch(phost, BorderSz,BorderSz,(FT_DispWidth - 2*BorderSz),(FT_DispHeight - 2*BorderSz),0,L8);//sketch in L8 format
	/* Display the sketch */
	FT_GPU_CoCmd_Dlstart(phost);
	FT_API_Write_CoCmd(CLEAR_COLOR_RGB(0x00,0,0x80));
	FT_API_Write_CoCmd(CLEAR(1,1,1));
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	FT_API_Write_CoCmd(SCISSOR_SIZE((FT_DispWidth - 2*BorderSz),(FT_DispHeight - 2*BorderSz)));
	FT_API_Write_CoCmd(SCISSOR_XY(BorderSz,BorderSz));
	FT_API_Write_CoCmd(CLEAR_COLOR_RGB(0xff,0xff,0xff));
	FT_API_Write_CoCmd(CLEAR(1,1,1));

	FT_API_Write_CoCmd(SCISSOR_SIZE(512,512));
	FT_API_Write_CoCmd(SCISSOR_XY(0,0));
	FT_API_Write_CoCmd(COLOR_RGB(0,0,0));
	/* L8 bitmap display */
	FT_API_Write_CoCmd(BEGIN(BITMAPS));
	FT_API_Write_CoCmd(BITMAP_SOURCE(0));
	FT_API_Write_CoCmd(BITMAP_LAYOUT(L8,(FT_DispWidth - 2*BorderSz),(FT_DispHeight - 2*BorderSz)));
	FT_API_Write_CoCmd(BITMAP_SIZE(BILINEAR,BORDER,BORDER,(FT_DispWidth - 2*BorderSz),(FT_DispHeight - 2*BorderSz)));
	FT_API_Write_CoCmd(VERTEX2F(BorderSz*16,BorderSz*16));
	FT_API_Write_CoCmd(END());
	/* Display the text */
	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	FT_GPU_CoCmd_Text_P(phost,(FT_DispWidth/2), 20, 27, OPT_CENTER, PSTR("Sketch L8"));

	FT_API_Write_CoCmd(DISPLAY());
	FT_GPU_CoCmd_Swap(phost);

	/* Wait till coprocessor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
	FT_APP_ENABLE_DELAY_MS(5000);

	/* Send the stop command */
	FT_GPU_HAL_WrCmd32(phost,  CMD_STOP);
	/* Update the command buffer pointers - both read and write pointers */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
}

/* API to demonstrate scale, rotate and translate functionality */
ft_void_t FT_APP_CoPro_Matrix()
{
	/*************************************************************************/
	/* Below code demonstrates the usage of bitmap matrix processing apis.   */
	/* Mainly matrix apis consists if scale, rotate and translate.           */
	/* Units of translation and scale are interms of 1/65536, rotation is in */
	/* degrees and in terms of 1/65536. +ve theta is anticlock wise, and -ve  */
	/* theta is clock wise rotation                                          */
	/*************************************************************************/

	/* Lena image with 40x40 rgb565 is used as an example */

	ft_int32_t imagewidth,imagestride,imageheight,imagexoffset,imageyoffset;

	/* Download the bitmap data */
	FT_GPU_HAL_WrMem_P(phost, RAM_G,(ft_uint8_t *)&Lena_Bitmap_RawData[Lena_Bitmap_RawData_Header[0].ArrayOffset],
		Lena_Bitmap_RawData_Header[0].Stride*Lena_Bitmap_RawData_Header[0].Height);


	FT_GPU_CoCmd_Dlstart(phost);
	FT_API_Write_CoCmd(CLEAR_COLOR_RGB(0xff,0xff,0xff));
	FT_API_Write_CoCmd(CLEAR(1,1,1));
	FT_API_Write_CoCmd(COLOR_RGB(32,32,32));
	FT_GPU_CoCmd_Text_P(phost,10, 5, 16, 0, PSTR("BM with rotation"));
	FT_GPU_CoCmd_Text_P(phost,10, 20 + 40 + 5, 16, 0, PSTR("BM with scaling"));
	FT_GPU_CoCmd_Text_P(phost,10, 20 + 40 + 20 + 80 + 5, 16, 0, PSTR("BM with flip"));

	imagewidth = Lena_Bitmap_RawData_Header[0].Width;
	imageheight = Lena_Bitmap_RawData_Header[0].Height;
	imagestride = Lena_Bitmap_RawData_Header[0].Stride;
	imagexoffset = 10*16;
	imageyoffset = 20*16;

	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
	FT_API_Write_CoCmd(BEGIN(BITMAPS));
	FT_API_Write_CoCmd(BITMAP_SOURCE(0));
	FT_API_Write_CoCmd(BITMAP_LAYOUT(Lena_Bitmap_RawData_Header[0].Format,imagestride,imageheight));
	FT_API_Write_CoCmd(BITMAP_SIZE(BILINEAR,BORDER,BORDER,imagewidth,imageheight));
	/******************************************* Perform display of plain bitmap ************************************/
	FT_API_Write_CoCmd(VERTEX2F(imagexoffset,imageyoffset));

	/* Perform display of plain bitmap with 45 degrees anti clock wise and the rotation is performed on top left coordinate */
	imagexoffset += (imagewidth + 10)*16;
	FT_GPU_CoCmd_LoadIdentity(phost);
	FT_GPU_CoCmd_Rotate(phost, (-45*65536/360));//rotate by 45 degrees anticlock wise
	FT_GPU_CoCmd_SetMatrix(phost );
	FT_API_Write_CoCmd(VERTEX2F(imagexoffset,imageyoffset));

	/* Perform display of plain bitmap with 30 degrees clock wise and the rotation is performed on top left coordinate */
	imagexoffset += (imagewidth*1.42 + 10)*16;//add the width*1.41 as diagonal is new width and extra 10 pixels
	FT_GPU_CoCmd_LoadIdentity(phost);
	FT_GPU_CoCmd_Rotate(phost, 30*65536/360);//rotate by 33 degrees clock wise
	FT_GPU_CoCmd_SetMatrix(phost );
	FT_API_Write_CoCmd(VERTEX2F(imagexoffset,imageyoffset));

	/* Perform display of plain bitmap with 45 degrees anti clock wise and the rotation is performed wrt centre of the bitmap */
	imagexoffset += (imagewidth*1.42 + 10)*16;//add the width*1.41 as diagonal is new width and extra 10 pixels
	FT_GPU_CoCmd_LoadIdentity(phost);
	FT_GPU_CoCmd_Translate(phost, 65536*imagewidth/2,65536*imageheight/2);//make the rotation coordinates at the center
	FT_GPU_CoCmd_Rotate(phost, -45*65536/360);//rotate by 45 degrees anticlock wise
	FT_GPU_CoCmd_Translate(phost, -65536*imagewidth/2,-65536*imageheight/2);
	FT_GPU_CoCmd_SetMatrix(phost );
	FT_API_Write_CoCmd(VERTEX2F(imagexoffset,imageyoffset));

	/* Perform display of plain bitmap with 45 degrees clock wise and the rotation is performed so that whole bitmap is viewable */
	imagexoffset += (imagewidth*1.42 + 10)*16;//add the width*1.41 as diagonal is new width and extra 10 pixels
	FT_GPU_CoCmd_LoadIdentity(phost);
	FT_GPU_CoCmd_Translate(phost, 65536*imagewidth/2,65536*imageheight/2);//make the rotation coordinates at the center
	FT_GPU_CoCmd_Rotate(phost, 45*65536/360);//rotate by 45 degrees clock wise
	FT_GPU_CoCmd_Translate(phost, -65536*imagewidth/10,-65536*imageheight/2);
	FT_GPU_CoCmd_SetMatrix(phost );
	FT_API_Write_CoCmd(BITMAP_SIZE(BILINEAR,BORDER,BORDER,imagewidth*2,imageheight*2));
	FT_API_Write_CoCmd(VERTEX2F(imagexoffset,imageyoffset));

	/* Perform display of plain bitmap with 90 degrees anti clock wise and the rotation is performed so that whole bitmap is viewable */
	imagexoffset += (imagewidth*2 + 10)*16;
	FT_GPU_CoCmd_LoadIdentity(phost);
	FT_GPU_CoCmd_Translate(phost, 65536*imagewidth/2,65536*imageheight/2);//make the rotation coordinates at the center
	FT_GPU_CoCmd_Rotate(phost, -90*65536/360);//rotate by 90 degrees anticlock wise
	FT_GPU_CoCmd_Translate(phost, -65536*imagewidth/2,-65536*imageheight/2);
	FT_GPU_CoCmd_SetMatrix(phost );
	FT_API_Write_CoCmd(BITMAP_SIZE(BILINEAR,BORDER,BORDER,imagewidth,imageheight));
	FT_API_Write_CoCmd(VERTEX2F(imagexoffset,imageyoffset));

	/* Perform display of plain bitmap with 180 degrees clock wise and the rotation is performed so that whole bitmap is viewable */
	imagexoffset += (imagewidth + 10)*16;
	FT_GPU_CoCmd_LoadIdentity(phost);
	FT_GPU_CoCmd_Translate(phost, 65536*imagewidth/2,65536*imageheight/2);//make the rotation coordinates at the center
	FT_GPU_CoCmd_Rotate(phost, -180*65536/360);//rotate by 180 degrees anticlock wise
	FT_GPU_CoCmd_Translate(phost, -65536*imagewidth/2,-65536*imageheight/2);
	FT_GPU_CoCmd_SetMatrix(phost );
	FT_API_Write_CoCmd(VERTEX2F(imagexoffset,imageyoffset));
	/******************************************* Perform display of bitmap with scale ************************************/
	/* Perform display of plain bitmap with scale factor of 2x2 in x & y direction */
	imagexoffset = (10)*16;
	imageyoffset += (imageheight + 20)*16;
	FT_GPU_CoCmd_LoadIdentity(phost);
	FT_GPU_CoCmd_Scale(phost, 2*65536,2*65536);//scale by 2x2
	FT_GPU_CoCmd_SetMatrix(phost );
	FT_API_Write_CoCmd(BITMAP_SIZE(BILINEAR,BORDER,BORDER,imagewidth*2,imageheight*2));
	FT_API_Write_CoCmd(VERTEX2F(imagexoffset,imageyoffset));

	/* Perform display of plain bitmap with scale factor of .5x.25 in x & y direction, rotate by 45 degrees clock wise wrt top left */
	imagexoffset += (imagewidth*2 + 10)*16;
	FT_GPU_CoCmd_LoadIdentity(phost);
	FT_GPU_CoCmd_Translate(phost, 65536*imagewidth/2,65536*imageheight/2);//make the rotation coordinates at the center

	FT_GPU_CoCmd_Rotate(phost, 45*65536/360);//rotate by 45 degrees clock wise
	FT_GPU_CoCmd_Scale(phost, 65536/2,65536/4);//scale by .5x.25
	FT_GPU_CoCmd_Translate(phost, -65536*imagewidth/2,-65536*imageheight/2);
	FT_GPU_CoCmd_SetMatrix(phost );
	FT_API_Write_CoCmd(VERTEX2F(imagexoffset,imageyoffset));

	/* Perform display of plain bitmap with scale factor of .5x2 in x & y direction, rotate by 75 degrees anticlock wise wrt center of the image */
	imagexoffset += (imagewidth + 10)*16;
	FT_GPU_CoCmd_LoadIdentity(phost);
	FT_GPU_CoCmd_Translate(phost, 65536*imagewidth/2,65536*imageheight/2);//make the rotation coordinates at the center
	FT_GPU_CoCmd_Rotate(phost, -75*65536/360);//rotate by 75 degrees anticlock wise
	FT_GPU_CoCmd_Scale(phost, 65536/2,2*65536);//scale by .5x2
	FT_GPU_CoCmd_Translate(phost, -65536*imagewidth/2,-65536*imageheight/8);
	FT_GPU_CoCmd_SetMatrix(phost );
	FT_API_Write_CoCmd(BITMAP_SIZE(BILINEAR,BORDER,BORDER,imagewidth*5/2,imageheight*5/2));
	FT_API_Write_CoCmd(VERTEX2F(imagexoffset,imageyoffset));
	/******************************************* Perform display of bitmap flip ************************************/
	/* perform display of plain bitmap with 1x1 and flip right */
	imagexoffset = (10)*16;
	imageyoffset += (imageheight*2 + 20)*16;
	FT_GPU_CoCmd_LoadIdentity(phost);
	FT_GPU_CoCmd_Translate(phost, 65536*imagewidth/2,65536*imageheight/2);//make the rotation coordinates at the center
	FT_GPU_CoCmd_Scale(phost, -1*65536,1*65536);//flip right
	FT_GPU_CoCmd_Translate(phost, -65536*imagewidth/2,-65536*imageheight/2);
	FT_GPU_CoCmd_SetMatrix(phost );
	FT_API_Write_CoCmd(BITMAP_SIZE(BILINEAR,BORDER,BORDER,imagewidth,imageheight));
	FT_API_Write_CoCmd(VERTEX2F(imagexoffset,imageyoffset));

	/* Perform display of plain bitmap with 2x2 scaling, flip bottom */
	imagexoffset += (imagewidth + 10)*16;
	FT_GPU_CoCmd_LoadIdentity(phost);
	FT_GPU_CoCmd_Translate(phost, 65536*imagewidth/2,65536*imageheight/2);//make the rotation coordinates at the center
	FT_GPU_CoCmd_Scale(phost, 2*65536,-2*65536);//flip bottom and scale by 2 on both sides
	FT_GPU_CoCmd_Translate(phost, -65536*imagewidth/4,-65536*imageheight/1.42);
	FT_GPU_CoCmd_SetMatrix(phost );
	FT_API_Write_CoCmd(BITMAP_SIZE(BILINEAR,BORDER,BORDER,imagewidth*4,imageheight*4));
	FT_API_Write_CoCmd(VERTEX2F(imagexoffset,imageyoffset));

	/* Perform display of plain bitmap with 2x1 scaling, rotation and flip right and make sure whole image is viewable */
	imagexoffset += (imagewidth*2 + 10)*16;
	FT_GPU_CoCmd_LoadIdentity(phost);
	FT_GPU_CoCmd_Translate(phost, 65536*imagewidth/2,65536*imageheight/2);//make the rotation coordinates at the center

	FT_GPU_CoCmd_Rotate(phost, -45*65536/360);//rotate by 45 degrees anticlock wise
	FT_GPU_CoCmd_Scale(phost, -2*65536,1*65536);//flip right and scale by 2 on x axis
	FT_GPU_CoCmd_Translate(phost, -65536*imagewidth/2,-65536*imageheight/8);
	FT_GPU_CoCmd_SetMatrix(phost );
	FT_API_Write_CoCmd(BITMAP_SIZE(BILINEAR,BORDER,BORDER,(imagewidth*5),(imageheight*5)));
	FT_API_Write_CoCmd(VERTEX2F(imagexoffset,imageyoffset));

	FT_API_Write_CoCmd(END());
	FT_API_Write_CoCmd(DISPLAY());
	FT_GPU_CoCmd_Swap(phost);

	/* Wait till coprocessor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
	FT_APP_ENABLE_DELAY_MS(5000);
}

/* Sample app api to demonstrate track widget functionality */
ft_void_t FT_APP_CoPro_Track(ft_void_t)
{

	/*************************************************************************/
	/* Below code demonstrates the usage of track function. Track function   */
	/* tracks the pen touch on any specific object. Track function supports  */
	/* rotary and horizontal/vertical tracks. Rotary is given by rotation    */
	/* angle and horizontal/vertical track is offset position.               */
	/*************************************************************************/
	{
	ft_int32_t LoopFlag = 0;
	ft_uint32_t TrackRegisterVal = 0;
	ft_uint16_t angleval = 0,slideval = 0,scrollval = 0;

	/* Set the tracker for 3 objects */

	FT_GPU_CoCmd_Track(phost, FT_DispWidth/2, FT_DispHeight/2, 1,1, 10);
	FT_GPU_CoCmd_Track(phost, 40, (FT_DispHeight - 40), (FT_DispWidth - 80),8, 11);
	FT_GPU_CoCmd_Track(phost, (FT_DispWidth - 40), 40, 8,(FT_DispHeight - 80), 12);

	/* Wait till coprocessor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);

	LoopFlag = 600;
	/* update the background colour continuously for the colour change in any of the trackers */
	while(LoopFlag--)
	{
		ft_uint8_t tagval = 0;
		TrackRegisterVal = FT_GPU_HAL_Rd32(phost, REG_TRACKER);
		tagval = TrackRegisterVal & 0xff;
		if(0 != tagval)
		{
			if(10 == tagval)
			{
				angleval = TrackRegisterVal>>16;
			}
			else if(11 == tagval)
			{
				slideval = TrackRegisterVal>>16;
			}
			else if(12 == tagval)
			{
				scrollval = TrackRegisterVal>>16;
				if((scrollval + 65535/10) > (9*65535/10))
				{
					scrollval = (8*65535/10);
				}
				else if(scrollval < (1*65535/10))
				{
					scrollval = 0;
				}
				else
				{
					scrollval -= (1*65535/10);
				}
			}
		}
		/* Display a rotary dial, horizontal slider and vertical scroll */

		FT_API_Write_CoCmd( CMD_DLSTART);

		{
			ft_int32_t tmpval0,tmpval1,tmpval2;
			ft_uint8_t angval,sldval,scrlval;

			tmpval0 = (ft_int32_t)angleval*255/65536;
			tmpval1 = (ft_int32_t)slideval*255/65536;
			tmpval2 = (ft_int32_t)scrollval*255/65536;

			angval = tmpval0&0xff;
			sldval = tmpval1&0xff;
			scrlval = tmpval2&0xff;
			FT_API_Write_CoCmd(CLEAR_COLOR_RGB(angval,sldval,scrlval));
		}
		FT_API_Write_CoCmd(CLEAR(1,1,1));
		FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));

		/* Draw dial with 3d effect */
		FT_GPU_CoCmd_FgColor(phost,0x00ff00);
		FT_GPU_CoCmd_BgColor(phost, 0x800000);
		FT_API_Write_CoCmd(TAG(10));
		FT_GPU_CoCmd_Dial(phost, (FT_DispWidth/2), (FT_DispHeight/2), (FT_DispWidth/8), 0, angleval);

		/* Draw slider with 3d effect */
		FT_GPU_CoCmd_FgColor(phost,0x00a000);
		FT_GPU_CoCmd_BgColor(phost, 0x800000);
		FT_API_Write_CoCmd(TAG(11));
		FT_GPU_CoCmd_Slider(phost, 40, (FT_DispHeight - 40), (FT_DispWidth - 80),8, 0, slideval, 65535);

		/* Draw scroll with 3d effect */
		FT_GPU_CoCmd_FgColor(phost,0x00a000);
		FT_GPU_CoCmd_BgColor(phost, 0x000080);
		FT_API_Write_CoCmd(TAG(12));
		FT_GPU_CoCmd_Scrollbar(phost, (FT_DispWidth - 40), 40, 8, (FT_DispHeight - 80), 0, scrollval, (65535*0.2), 65535);

		FT_GPU_CoCmd_FgColor(phost,TAG_MASK(0));
		FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
		FT_GPU_CoCmd_Text_P(phost,(FT_DispWidth/2), ((FT_DispHeight/2) + (FT_DispWidth/8) + 8), 26, OPT_CENTER, PSTR("Rotary track"));
		FT_GPU_CoCmd_Text_P(phost,((FT_DispWidth/2)), ((FT_DispHeight - 40) + 8 + 8), 26, OPT_CENTER, PSTR("Horizontal track"));
		FT_GPU_CoCmd_Text_P(phost,(FT_DispWidth - 40), 20, 26, OPT_CENTER, PSTR("Vertical track"));

		FT_API_Write_CoCmd(DISPLAY());
		FT_GPU_CoCmd_Swap(phost);

		/* Wait till coprocessor completes the operation */
		FT_GPU_HAL_WaitCmdfifo_empty(phost);

		vTaskDelay( 10 / portTICK_PERIOD_MS );//sleep for 10 ms
	}

	/* Set the tracker for 3 objects */

	FT_GPU_CoCmd_Track(phost, 240, 136, 0,0, 10);
	FT_GPU_CoCmd_Track(phost, 40, 232, 0,0, 11);
	FT_GPU_CoCmd_Track(phost, 400, 40, 0,0, 12);

	/* Wait till coprocessor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
	FT_APP_ENABLE_DELAY_MS(5000);
	}
}
#endif

#ifdef FT_APP_ENABLE_APIS_SET3
/* API to demonstrate custom font display */
ft_void_t FT_APP_CoPro_Setfont()
{
	ft_uint8_t *pbuff;
	ft_uint8_t FontIdxTable[148];
	/*************************************************************************/
	/* Below code demonstrates the usage of setfont. Setfont function draws  */
	/* custom configured fonts on screen. Download the font table and raw    */
	/* font data in L1/L4/L8 format and display text                          */
	/*************************************************************************/

	/* Display custom font by reading from the binary file - header of 148 bytes is at the starting followed by font data of 96 characters */
	/*Roboto-BoldCondensed.ttf*/
	{
	ft_uint32_t fontaddr = (128+5*4);//header size
	ft_uint16_t blocklen;

	fontaddr = RAM_G;
	blocklen = 128+5*4;//header size

	pbuff = FontIdxTable;
	/* Copy data from starting of the array into buffer */
	//hal_memcpy((ft_uint8_t*)pbuff,(ft_uint8_t*)Roboto_BoldCondensed_12,1L*blocklen);
	memcpy_P((ft_uint8_t*)pbuff, (ft_uint8_t*)Roboto_BoldCondensed_12, 1L*blocklen);

	{
		ft_uint32_t *ptemp = (ft_uint32_t *)&pbuff[128+4*4],i;
		*ptemp = 1024;//download the font data at location 1024+32*8*25
		//memset(pbuff,16,32);
		for(i=0;i<32;i++)
		{
		  pbuff[i] = 16;
		}
	}
	/* Modify the font data location */
	FT_GPU_HAL_WrMem(phost,fontaddr,(ft_uint8_t *)pbuff,blocklen);

	/* Next download the data at location 32*8*25 - skip the first 32 characters */
	/* each character is 8x25 bytes */
	fontaddr += (1024+32*8*25);//make sure space is left at the starting of the buffer for first 32 characters - TBD manager this buffer so that this buffer can be utilized by other module

    FT_GPU_HAL_WrMem_P(phost, fontaddr,&Roboto_BoldCondensed_12[blocklen],1L*FT_APP_Roboto_BoldCondensed_12_SIZE);

	FT_GPU_CoCmd_Dlstart(phost);
	FT_API_Write_CoCmd(CLEAR_COLOR_RGB(0xff,0xff,0xff));//set the background to white
	FT_API_Write_CoCmd(CLEAR(1,1,1));
	FT_API_Write_CoCmd(COLOR_RGB(32,32,32));//black colour text

	FT_GPU_CoCmd_Text_P(phost,(FT_DispWidth/2), 20, 27, OPT_CENTER, PSTR("SetFont - format L4"));
	FT_API_Write_CoCmd(BITMAP_HANDLE(6));//give index table 6
	FT_API_Write_CoCmd(BITMAP_SOURCE(1024));//make the address to 0
	FT_API_Write_CoCmd(BITMAP_LAYOUT(L4,8,25));//stride is 8 and height is 25
	FT_API_Write_CoCmd(BITMAP_SIZE(NEAREST,BORDER,BORDER,16,25));//width is 16 and height is 25

	FT_GPU_CoCmd_SetFont(phost, 6,0);
	FT_GPU_CoCmd_Text_P(phost,(FT_DispWidth/2), 80,  6, OPT_CENTER, PSTR("The quick brown fox jumps"));
	FT_GPU_CoCmd_Text_P(phost,(FT_DispWidth/2), 120, 6, OPT_CENTER, PSTR("over the lazy dog."));
	FT_GPU_CoCmd_Text_P(phost,(FT_DispWidth/2), 160, 6, OPT_CENTER, PSTR("1234567890"));

	FT_API_Write_CoCmd(DISPLAY());
	FT_GPU_CoCmd_Swap(phost);

	/* Wait till coprocessor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
    FT_APP_ENABLE_DELAY_MS(4000);
	}
}
#endif


float lerp(float t, float a, float b)
{
	return (float)((1 - t) * a + t * b);
}
float smoothlerp(float t, float a, float b)
{

    float lt = 3 * t * t - 2 * t * t * t;

    return lerp(lt, a, b);
}

#ifdef FT_APP_ENABLE_APIS_SET0
/* First draw points followed by lines to create 3d ball kind of effect */
ft_void_t FT_APP_GPU_Ball_Stencil()
{
	ft_int16_t xball = (FT_DispWidth/2),yball = 120,rball = (FT_DispWidth/8);
	ft_int16_t numpoints = 6,numlines = 8,i,asize,aradius,gridsize = 20;
	ft_int32_t asmooth,loopflag = 1,dispr = (FT_DispWidth - 10),displ = 10,dispa = 10,dispb = (FT_DispHeight - 10),xflag = 1,yflag = 1;

	dispr -= ((dispr - displ)%gridsize);
	dispb -= ((dispb - dispa)%gridsize);
	/* write the play sound */
	FT_GPU_HAL_Wr16(phost, REG_SOUND,0x50);
        loopflag = 100;
	while(loopflag-- >0 )
	{
		if(((xball + rball + 2) >= dispr) || ((xball - rball - 2) <= displ))
		{
			xflag ^= 1;
			FT_GPU_HAL_Wr8(phost, REG_PLAY,1);
		}
		if(((yball + rball + 8) >= dispb) || ((yball - rball - 8) <= dispa))
		{
			yflag ^= 1;
			FT_GPU_HAL_Wr8(phost, REG_PLAY,1);
		}
		if(xflag)
		{
			xball += 2;
		}
		else
		{
			xball -= 2;
		}
		if(yflag)
		{
			yball += 8 ;
		}
		else
		{
			yball -= 8;
		}


		FT_API_Write_DLCmd(CLEAR_COLOR_RGB(128, 128, 0) );
		FT_API_Write_DLCmd(CLEAR(1, 1, 1)); // clear screen
		FT_API_Write_DLCmd(STENCIL_OP(INCR,INCR) );
		FT_API_Write_DLCmd(COLOR_RGB(0, 0, 0) );
		/* draw grid */
		FT_API_Write_DLCmd(LINE_WIDTH(16));
		FT_API_Write_DLCmd(BEGIN(LINES));
		for(i=0;i<=((dispr - displ)/gridsize);i++)
		{
			FT_API_Write_DLCmd(VERTEX2F((displ + i*gridsize)*16,dispa*16));
			FT_API_Write_DLCmd(VERTEX2F((displ + i*gridsize)*16,dispb*16));
		}
		for(i=0;i<=((dispb - dispa)/gridsize);i++)
		{
			FT_API_Write_DLCmd(VERTEX2F(displ*16,(dispa + i*gridsize)*16));
			FT_API_Write_DLCmd(VERTEX2F(dispr*16,(dispa + i*gridsize)*16));
		}
		FT_API_Write_DLCmd(END());
		FT_API_Write_DLCmd(COLOR_MASK(0,0,0,0) );//mask all the colors
		FT_API_Write_DLCmd(POINT_SIZE(rball*16) );
		FT_API_Write_DLCmd(BEGIN(POINTS));
		FT_API_Write_DLCmd(VERTEX2F(xball*16,yball*16));
		FT_API_Write_DLCmd(STENCIL_OP(INCR,ZERO) );
		FT_API_Write_DLCmd(STENCIL_FUNC(GEQUAL,1,255));
		/* one side points */

		for(i=1;i<=numpoints;i++)
		{
			asize = i*rball*2/(numpoints + 1);
			asmooth = (ft_int16_t)smoothlerp((float)((float)(asize)/(2*(float)rball)),0,2*(float)rball);

			if(asmooth > rball)
			{
				//change the offset to -ve
				ft_int32_t tempsmooth;
				tempsmooth = asmooth - rball;
				aradius = (rball*rball + tempsmooth*tempsmooth)/(2*tempsmooth);
				FT_API_Write_DLCmd(POINT_SIZE(aradius*16) );
				FT_API_Write_DLCmd(VERTEX2F((xball - aradius + tempsmooth)*16,yball*16));
			}
			else
			{
				ft_int32_t tempsmooth;
				tempsmooth = rball - asmooth;
				aradius = (rball*rball + tempsmooth*tempsmooth)/(2*tempsmooth);
				FT_API_Write_DLCmd(POINT_SIZE(aradius*16) );
				FT_API_Write_DLCmd(VERTEX2F((xball+ aradius - tempsmooth)*16,yball*16));
			}
		}



		FT_API_Write_DLCmd(END());
		FT_API_Write_DLCmd(BEGIN(LINES));
		/* draw lines - line should be at least radius diameter */
		for(i=1;i<=numlines;i++)
		{
			asize = (i*rball*2/numlines);
			asmooth = (ft_int16_t)smoothlerp((float)((float)(asize)/(2*(float)rball)),0,2*(float)rball);
			FT_API_Write_DLCmd(LINE_WIDTH(asmooth * 16));
			FT_API_Write_DLCmd(VERTEX2F((xball - rball)*16,(yball - rball )*16));
			FT_API_Write_DLCmd(VERTEX2F((xball + rball)*16,(yball - rball )*16));
		}
		FT_API_Write_DLCmd(END());

		FT_API_Write_DLCmd(COLOR_MASK(1,1,1,1) );//enable all the colors
		FT_API_Write_DLCmd(STENCIL_FUNC(ALWAYS,1,255));
		FT_API_Write_DLCmd(STENCIL_OP(KEEP,KEEP));
		FT_API_Write_DLCmd(COLOR_RGB(255, 255, 255) );
		FT_API_Write_DLCmd(POINT_SIZE(rball*16) );
		FT_API_Write_DLCmd(BEGIN(POINTS));
		FT_API_Write_DLCmd(VERTEX2F((xball - 1)*16,(yball - 1)*16));
		FT_API_Write_DLCmd(COLOR_RGB(0, 0, 0) );//shadow
		FT_API_Write_DLCmd(COLOR_A(160) );
		FT_API_Write_DLCmd(VERTEX2F((xball+16)*16,(yball+8)*16));
		FT_API_Write_DLCmd(COLOR_A(255) );
		FT_API_Write_DLCmd(COLOR_RGB(255, 255, 255) );
		FT_API_Write_DLCmd(VERTEX2F(xball*16,yball*16));
		FT_API_Write_DLCmd(COLOR_RGB(255, 0, 0) );
		FT_API_Write_DLCmd(STENCIL_FUNC(GEQUAL,1,1));
		FT_API_Write_DLCmd(STENCIL_OP(KEEP,KEEP));
		FT_API_Write_DLCmd(VERTEX2F(xball*16,yball*16));

		FT_API_Write_DLCmd(END());

		FT_API_Write_DLCmd(DISPLAY());

		/* Reset the DL Buffer index, set for the next group of DL commands */
		FT_API_Reset_DLBuffer();

		/* Do a swap */
		FT_API_GPU_DLSwap(DLSWAP_FRAME);
	    FT_APP_ENABLE_DELAY_MS(10);

	}

}
#endif

#ifdef FT_APP_ENABLE_APIS_SET4
/* API to explain the usage of touch engine */
ft_void_t FT_APP_Touch()
{
	ft_int32_t LoopFlag = 0,wbutton,hbutton,tagval,tagoption;
	ft_char8_t StringArray[100];
	ft_uint32_t ReadWord;
	ft_int16_t xvalue,yvalue,pendown;

	/*************************************************************************/
	/* Below code demonstrates the usage of touch function. Display info     */
	/* touch raw, touch screen, touch tag, raw adc and resistance values     */
	/*************************************************************************/
	LoopFlag = 300;
	wbutton = FT_DispWidth/8;
	hbutton = FT_DispHeight/8;
	while(LoopFlag--)
	{

		FT_GPU_CoCmd_Dlstart(phost);
		FT_API_Write_CoCmd(CLEAR_COLOR_RGB(64,64,64));
		FT_API_Write_CoCmd(CLEAR(1,1,1));
		FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
		FT_API_Write_CoCmd(TAG_MASK(0));
		/* Draw informative text at width/2,20 location */
		StringArray[0] = '\0';
		strcat_P(StringArray,PSTR("Touch Raw XY ("));
		ReadWord = FT_GPU_HAL_Rd32(phost, REG_TOUCH_RAW_XY);
		yvalue = (ft_int16_t)(ReadWord & 0xffff);
		xvalue = (ft_int16_t)((ReadWord>>16) & 0xffff);
		FT_GPU_HAL_Dec2ASCII(StringArray,(ft_int32_t)xvalue);
		strcat(StringArray,",");
		FT_GPU_HAL_Dec2ASCII(StringArray,(ft_int32_t)yvalue);
		strcat(StringArray,")");
		FT_GPU_CoCmd_Text(phost,FT_DispWidth/2, 10, 26, OPT_CENTER, StringArray);

		StringArray[0] = '\0';
		strcat_P(StringArray,PSTR("Touch RZ ("));
		ReadWord = FT_GPU_HAL_Rd16(phost,REG_TOUCH_RZ);
		FT_GPU_HAL_Dec2ASCII(StringArray,ReadWord);
		strcat(StringArray,")");
		FT_GPU_CoCmd_Text(phost,FT_DispWidth/2, 25, 26, OPT_CENTER, StringArray);

		StringArray[0] = '\0';
		strcat_P(StringArray,PSTR("Touch Screen XY ("));
		ReadWord = FT_GPU_HAL_Rd32(phost, REG_TOUCH_SCREEN_XY);
		yvalue = (ft_int16_t)(ReadWord & 0xffff);
		xvalue = (ft_int16_t)((ReadWord>>16) & 0xffff);
		FT_GPU_HAL_Dec2ASCII(StringArray,(ft_int32_t)xvalue);
		strcat(StringArray,",");
		FT_GPU_HAL_Dec2ASCII(StringArray,(ft_int32_t)yvalue);
		strcat(StringArray,")");
		FT_GPU_CoCmd_Text(phost,FT_DispWidth/2, 40, 26, OPT_CENTER, StringArray);

		StringArray[0] = '\0';
		strcat_P(StringArray,PSTR("Touch TAG ("));
		ReadWord = FT_GPU_HAL_Rd8(phost, REG_TOUCH_TAG);
		FT_GPU_HAL_Dec2ASCII(StringArray,ReadWord);
		strcat(StringArray,")");
		FT_GPU_CoCmd_Text(phost,FT_DispWidth/2, 55, 26, OPT_CENTER, StringArray);
		tagval = ReadWord;
		StringArray[0] = '\0';
		strcat_P(StringArray,PSTR("Touch Direct XY ("));
		ReadWord = FT_GPU_HAL_Rd32(phost, REG_TOUCH_DIRECT_XY);
		yvalue = (ft_int16_t)(ReadWord & 0x03ff);
		xvalue = (ft_int16_t)((ReadWord>>16) & 0x03ff);
		FT_GPU_HAL_Dec2ASCII(StringArray,(ft_int32_t)xvalue);
		strcat(StringArray,",");
		FT_GPU_HAL_Dec2ASCII(StringArray,(ft_int32_t)yvalue);
		pendown = (ft_int16_t)((ReadWord>>31) & 0x01);
		strcat(StringArray,",");
		FT_GPU_HAL_Dec2ASCII(StringArray,(ft_int32_t)pendown);
		strcat(StringArray,")");
		FT_GPU_CoCmd_Text(phost,FT_DispWidth/2, 70, 26, OPT_CENTER, StringArray);

		StringArray[0] = '\0';
		strcat_P(StringArray,PSTR("Touch Direct Z1Z2 ("));
		ReadWord = FT_GPU_HAL_Rd32(phost, REG_TOUCH_DIRECT_Z1Z2);
		yvalue = (ft_int16_t)(ReadWord & 0x03ff);
		xvalue = (ft_int16_t)((ReadWord>>16) & 0x03ff);
		FT_GPU_HAL_Dec2ASCII(StringArray,(ft_int32_t)xvalue);
		strcat(StringArray,",");
		FT_GPU_HAL_Dec2ASCII(StringArray,(ft_int32_t)yvalue);
		strcat(StringArray,")");

		FT_GPU_CoCmd_Text(phost,FT_DispWidth/2, 85, 26, OPT_CENTER, StringArray);

		FT_GPU_CoCmd_FgColor(phost,0x008000);
		FT_API_Write_CoCmd(TAG_MASK(1));
		tagoption = 0;
		if(12 == tagval)
		{
			tagoption = OPT_FLAT;
		}

		FT_API_Write_CoCmd(TAG(12));
		FT_GPU_CoCmd_Button(phost,(FT_DispWidth/4) - (wbutton/2),(FT_DispHeight*2/4) - (hbutton/2),wbutton,hbutton,26,tagoption,"Tag12");
		FT_API_Write_CoCmd(TAG(13));
		tagoption = 0;
		if(13 == tagval)
		{
			tagoption = OPT_FLAT;
		}
		FT_GPU_CoCmd_Button(phost,(FT_DispWidth*3/4) - (wbutton/2),(FT_DispHeight*3/4) - (hbutton/2),wbutton,hbutton,26,tagoption,"Tag13");

		FT_API_Write_CoCmd(DISPLAY());
		FT_GPU_CoCmd_Swap(phost);

		/* Wait till co-processor completes the operation */
		FT_GPU_HAL_WaitCmdfifo_empty(phost);
		vTaskDelay( 30 / portTICK_PERIOD_MS );
	}
}

static ft_void_t FT_APP_playmutesound()
{
	FT_GPU_HAL_Wr16(phost,REG_SOUND,0x0060);
    FT_GPU_HAL_Wr8(phost,REG_PLAY,0x01);
}


/* APP to demonstrate the usage of sound engine of FT800 */

ft_prog_uint8_t FT_APP_Snd_Array[5*58] PROGMEM =
	"Slce\0Sqrq\0Sinw\0Saww\0Triw\0Beep\0Alrm\0Warb\0Crsl\0Pp01\0Pp02\0Pp03\0Pp04\0Pp05\0Pp06\0Pp07\0Pp08\0Pp09\0Pp10\0Pp11\0Pp12\0Pp13\0Pp14\0Pp15\0Pp16\0DMF#\0DMF*\0DMF0\0DMF1\0DMF2\0DMF3\0DMF4\0DMF5\0DMF6\0DMF7\0DMF8\0DMF9\0Harp\0Xyph\0Tuba\0Glok\0Orgn\0Trmp\0Pian\0Chim\0MBox\0Bell\0Clck\0Swth\0Cowb\0Noth\0Hiht\0Kick\0Pop \0Clak\0Chak\0Mute\0uMut\0";

ft_prog_uint8_t FT_APP_Snd_TagArray[58] PROGMEM = {
	0x63,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
	0x08,0x10,0x11,0x12,0x13,0x14,0x15,0x16,
	0x17,0x18,0x19,0x1a,0x1b,0x1c,0x1d,0x1e,
	0x1f,0x23,0x2C,0x30,0x31,0x32,0x33,0x34, // x2a -> x2C DTMF*
	0x35,0x36,0x37,0x38,0x39,0x40,0x41,0x42,
	0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x50,
	0x51,0x52,0x53,0x54,0x55,0x56,0x57,0x58,
	0x60,0x61
};

#define SOUND_TRACK_TAG			100
#define SOUND_LOOP_ITERATIONS  	1000

ft_void_t FT_APP_Sound()
{
	ft_uint16_t LoopFlag = SOUND_LOOP_ITERATIONS;
	ft_uint8_t numbtnrow, numbtncol;
	ft_uint16_t wbutton, hbutton;

	ft_int8_t tagval, tagvalsnd = -1;
	ft_int8_t prevtag = -1;
	ft_uint32_t freqtrack = 0, currfreq = 0, prevfreq = 0;

	/*************************************************************************/
	/* Below code demonstrates the usage of sound function. All the supported*/
	/* sounds and respective pitches are put as part of keys/buttons, by     */
	/* choosing particular key/button the sound is played                    */
	/*************************************************************************/

	numbtnrow = 7 /*16*/;//number of rows to be created - note that mute and unmute are not played in this application
	numbtncol = 8 /*13*/;//number of columns to be created
	wbutton = (FT_DispWidth - 40)/numbtncol;
	hbutton = FT_DispHeight/numbtnrow;

	/* set the volume to maximum */
	FT_GPU_HAL_Wr8(phost, REG_VOL_SOUND, 0xFF);
	/* set the tracker to track the slider for frequency */

	FT_GPU_CoCmd_Track(phost, FT_DispWidth - 15, 20, 8, (FT_DispHeight - 40), SOUND_TRACK_TAG);

	while(LoopFlag--)
	{
		tagval = FT_GPU_HAL_Rd8(phost, REG_TOUCH_TAG);
		freqtrack = FT_GPU_HAL_Rd32(phost, REG_TRACKER);

		if( (freqtrack & 0xff) == SOUND_TRACK_TAG)
		{
			currfreq = (ft_uint32_t)(freqtrack >> 16);
			currfreq = (ft_uint32_t)(88*currfreq)/65536;
			if(currfreq > 108)
				currfreq = 108;
		}
		if((tagval > 0))
		{
			if( tagval <= 99)
			{
				tagvalsnd = tagval;
			}
			if(0x63 == tagvalsnd)
			{
				tagvalsnd = 0;
			}
			if((prevtag != tagval) || (prevfreq != currfreq))
			{
				/* Play sound wrt pitch */
				FT_GPU_HAL_Wr16(phost, REG_SOUND, (((currfreq + 21) << 8) | tagvalsnd));
				FT_GPU_HAL_Wr8(phost, REG_PLAY, 1);
			}
			if( tagvalsnd == 0)
				tagvalsnd = 99;
		}

		/* start a new display list for construction of screen */
		FT_GPU_CoCmd_Dlstart(phost);
		FT_API_Write_CoCmd(CLEAR_COLOR_RGB(64,64,64));
		FT_API_Write_CoCmd(CLEAR(1,1,1));

		/* line width for the rectangles */
		FT_API_Write_CoCmd(LINE_WIDTH(1*16));

		/* custom keys for sound input */
		/* First draw all the rectangles followed by the font */
		/* yellow colour for background colour */
		FT_API_Write_CoCmd(COLOR_RGB(0x80,0x80,0x00));

		FT_API_Write_CoCmd(BEGIN(RECTS));
		for(ft_uint8_t i=0; i<numbtnrow; ++i)
		{
			for(ft_uint8_t j=0; j<numbtncol; ++j)
			{
				FT_API_Write_CoCmd(TAG(pgm_read_byte_near(&FT_APP_Snd_TagArray[(i * numbtncol) + j ])));
				FT_API_Write_CoCmd(VERTEX2II((j*wbutton + 2),(hbutton*i + 2),0,0));
				FT_API_Write_CoCmd(VERTEX2II(((j*wbutton) + wbutton - 2),((hbutton*i) + hbutton - 2),0,0));
			}
		}
		FT_API_Write_CoCmd(END());

		/* draw the highlight rectangle and text info */
		FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));

		for(ft_uint8_t i=0; i<numbtnrow; ++i)
		{
			for(ft_uint8_t j=0; j<numbtncol; ++j)
			{
				FT_API_Write_CoCmd((ft_uint32_t)TAG(pgm_read_byte_near(&FT_APP_Snd_TagArray[(i * numbtncol) + j ])));
				if(tagvalsnd == pgm_read_byte_near(&FT_APP_Snd_TagArray[(i * numbtncol) + j ]))
				{
					/* red colour for highlight effect */
					FT_API_Write_CoCmd(COLOR_RGB(0x80,0x00,0x00));
					FT_API_Write_CoCmd(BEGIN(RECTS));
					FT_API_Write_CoCmd(TAG(pgm_read_byte_near(&FT_APP_Snd_TagArray[(i * numbtncol) + j ])));
					FT_API_Write_CoCmd(VERTEX2II((j*wbutton + 2),(hbutton*i + 2),0,0));
					FT_API_Write_CoCmd(VERTEX2II(((j*wbutton) + wbutton - 2),((hbutton*i) + hbutton - 2),0,0));
					FT_API_Write_CoCmd(END());
					/* reset the colour to make sure font doesn't get impacted */
					FT_API_Write_CoCmd(COLOR_RGB(0xff,0xff,0xff));
				}
				/* to make sure that highlight rectangle as well as font to take the same tag values */

				FT_GPU_CoCmd_Text_P(phost, (wbutton/2) + j*wbutton,(hbutton/2) + hbutton*i, 26, OPT_CENTER, (ft_prog_char8_t*)&FT_APP_Snd_Array[((i * numbtncol) + j)*5]);
			}
		}

		/* Draw vertical slider bar for frequency control */
		FT_API_Write_CoCmd(TAG_MASK(0));
		FT_GPU_CoCmd_Text_P(phost,FT_DispWidth - 20,10,26,OPT_CENTER, PSTR("Freq"));
		FT_API_Write_CoCmd(TAG_MASK(1));
		FT_API_Write_CoCmd(TAG(100));
		FT_GPU_CoCmd_Slider(phost, FT_DispWidth - 15, 20, 8, (FT_DispHeight - 40), 0, currfreq, 88);

		FT_API_Write_CoCmd(DISPLAY());
		FT_GPU_CoCmd_Swap(phost);

		prevtag = tagval;
		prevfreq = currfreq;

		/* Wait till coprocessor completes the operation */
		FT_GPU_HAL_WaitCmdfifo_empty(phost);
	    FT_APP_ENABLE_DELAY_MS(10);
	}

	FT_GPU_HAL_Wr16(phost, REG_SOUND,0);
	FT_GPU_HAL_Wr8(phost, REG_PLAY,1);
}


ft_void_t FT_APP_PowerMode()
{
	/*************************************************
	Senario1:  Transition from Active mode to Standby mode.
	           Transition from Standby mode to Active Mode
	**************************************************/
    FT_APP_Screen_P(PSTR("Active to Standby Mode"));

	//Switch FT800 from Active to Standby mode
	FT_API_fadeout();
	FT_APP_playmutesound();//Play mute sound to avoid pop sound
	FT_GPU_PowerModeSwitch(phost,FT_GPU_STANDBY_M);
    FT_APP_ENABLE_DELAY();

	//Wake up from Standby first before accessing FT800 registers.
	FT_GPU_PowerModeSwitch(phost,FT_GPU_ACTIVE_M);
	FT_API_fadein();
    FT_APP_ENABLE_DELAY();

	/*************************************************
	Senario2:  Transition from Active mode to Sleep mode.
	           Transition from Sleep mode to Active Mode
	**************************************************/
    //Switch FT800 from Active to Sleep mode
	FT_APP_Screen_P(PSTR("Active to Sleep Mode"));
    FT_API_fadeout();
	FT_APP_playmutesound();//Play mute sound to avoid pop sound
    FT_GPU_PowerModeSwitch(phost,FT_GPU_SLEEP_M);
    FT_APP_ENABLE_DELAY();

	//Wake up from Sleep
	FT_GPU_PowerModeSwitch(phost,FT_GPU_ACTIVE_M);
    FT_API_fadein();
    FT_APP_ENABLE_DELAY();

	/*************************************************
	Senario3:  Transition from Active mode to PowerDown mode.
	           Transition from PowerDown mode to Active Mode via Standby mode.
	**************************************************/
    //Switch FT800 from Active to PowerDown mode by sending command
	FT_APP_Screen_P(PSTR("Active to PowerDown Mode"));
    FT_API_fadeout();
	FT_APP_playmutesound();//Play mute sound to avoid pop sound
    FT_GPU_PowerModeSwitch(phost,FT_GPU_POWERDOWN_M);
    FT_APP_ENABLE_DELAY();

    FT_API_Boot_Config();
	//Need to download display list again because power down mode lost all registers and memory
    FT_Home_Setup();
	FT_API_fadein();
    FT_APP_ENABLE_DELAY();
}
#endif

ft_prog_uint8_t home_star_icon[] PROGMEM = {0x78,0x9C,0xE5,0x94,0xBF,0x4E,0xC2,0x40,0x1C,0xC7,0x7F,0x2D,0x04,0x8B,0x20,0x45,0x76,0x14,0x67,0xA3,0xF1,0x0D,0x64,0x75,0xD2,0xD5,0x09,0x27,0x17,0x13,0xE1,0x0D,0xE4,0x0D,0x78,0x04,0x98,0x5D,0x30,0x26,0x0E,0x4A,0xA2,0x3E,0x82,0x0E,0x8E,0x82,0xC1,0x38,0x62,0x51,0x0C,0x0A,0x42,0x7F,0xDE,0xB5,0x77,0xB4,0x77,0x17,0x28,0x21,0x26,0x46,0xFD,0x26,0xCD,0xE5,0xD3,0x7C,0xFB,0xBB,0xFB,0xFD,0xB9,0x02,0xCC,0xA4,0xE8,0x99,0x80,0x61,0xC4,0x8A,0x9F,0xCB,0x6F,0x31,0x3B,0xE3,0x61,0x7A,0x98,0x84,0x7C,0x37,0xF6,0xFC,0xC8,0xDD,0x45,0x00,0xDD,0xBA,0xC4,0x77,0xE6,0xEE,0x40,0xEC,0x0E,0xE6,0x91,0xF1,0xD2,0x00,0x42,0x34,0x5E,0xCE,0xE5,0x08,0x16,0xA0,0x84,0x68,0x67,0xB4,0x86,0xC3,0xD5,0x26,0x2C,0x20,0x51,0x17,0xA2,0xB8,0x03,0xB0,0xFE,0x49,0xDD,0x54,0x15,0xD8,0xEE,0x73,0x37,0x95,0x9D,0xD4,0x1A,0xB7,0xA5,0x26,0xC4,0x91,0xA9,0x0B,0x06,0xEE,0x72,0xB7,0xFB,0xC5,0x16,0x80,0xE9,0xF1,0x07,0x8D,0x3F,0x15,0x5F,0x1C,0x0B,0xFC,0x0A,0x90,0xF0,0xF3,0x09,0xA9,0x90,0xC4,0xC6,0x37,0xB0,0x93,0xBF,0xE1,0x71,0xDB,0xA9,0xD7,0x41,0xAD,0x46,0xEA,0x19,0xA9,0xD5,0xCE,0x93,0xB3,0x35,0x73,0x0A,0x69,0x59,0x91,0xC3,0x0F,0x22,0x1B,0x1D,0x91,0x13,0x3D,0x91,0x73,0x43,0xF1,0x6C,0x55,0xDA,0x3A,0x4F,0xBA,0x25,0xCE,0x4F,0x04,0xF1,0xC5,0xCF,0x71,0xDA,0x3C,0xD7,0xB9,0xB2,0x48,0xB4,0x89,0x38,0x20,0x4B,0x2A,0x95,0x0C,0xD5,0xEF,0x5B,0xAD,0x96,0x45,0x8A,0x41,0x96,0x7A,0x1F,0x60,0x0D,0x7D,0x22,0x75,0x82,0x2B,0x0F,0xFB,0xCE,0x51,0x3D,0x2E,0x3A,0x21,0xF3,0x1C,0xD9,0x38,0x86,0x2C,0xC6,0x05,0xB6,0x7B,0x9A,0x8F,0x0F,0x97,0x1B,0x72,0x6F,0x1C,0xEB,0xAE,0xFF,0xDA,0x97,0x0D,0xBA,0x43,0x32,0xCA,0x66,0x34,0x3D,0x54,0xCB,0x24,0x9B,0x43,0xF2,0x70,0x3E,0x42,0xBB,0xA0,0x95,0x11,0x37,0x46,0xE1,0x4F,0x49,0xC5,0x1B,0xFC,0x3C,0x3A,0x3E,0xD1,0x65,0x0E,0x6F,0x58,0xF8,0x9E,0x5B,0xDB,0x55,0xB6,0x41,0x34,0xCB,0xBE,0xDB,0x87,0x5F,0xA9,0xD1,0x85,0x6B,0xB3,0x17,0x9C,0x61,0x0C,0x9B,0xA2,0x5D,0x61,0x10,0xED,0x2A,0x9B,0xA2,0x5D,0x61,0x10,0xED,0x2A,0x9B,0xA2,0x5D,0x61,0x10,0xED,0x2A,0x9B,0xED,0xC9,0xFC,0xDF,0x14,0x54,0x8F,0x80,0x7A,0x06,0xF5,0x23,0xA0,0x9F,0x41,0xF3,0x10,0x30,0x4F,0x41,0xF3,0x18,0x30,0xCF,0xCA,0xFC,0xFF,0x35,0xC9,0x79,0xC9,0x89,0xFA,0x33,0xD7,0x1D,0xF6,0x5E,0x84,0x5C,0x56,0x6E,0xA7,0xDA,0x1E,0xF9,0xFA,0xAB,0xF5,0x97,0xFF,0x2F,0xED,0x89,0x7E,0x29,0x9E,0xB4,0x9F,0x74,0x1E,0x69,0xDA,0xA4,0x9F,0x81,0x94,0xEF,0x4F,0xF6,0xF9,0x0B,0xF4,0x65,0x51,0x08};

ft_void_t FT_Home_Setup(ft_void_t)
{
	FT_GPU_HAL_WrCmd32(phost, CMD_INFLATE);
	FT_GPU_HAL_WrCmd32(phost, 250*1024L);
	FT_GPU_HAL_WrCmdBuf_P(phost, home_star_icon, sizeof(home_star_icon));

	FT_GPU_CoCmd_Dlstart(phost);        // start
	FT_API_Write_CoCmd( CLEAR(1,1,1) );
	FT_API_Write_CoCmd( COLOR_RGB(255, 255, 255) );
	FT_API_Write_CoCmd( BITMAP_HANDLE(13) );    // handle for background stars
	FT_API_Write_CoCmd( BITMAP_SOURCE(250*1024L) );   // Starting address in RAM_G
	FT_API_Write_CoCmd( BITMAP_LAYOUT(L4, 16, 32) );  // format
	FT_API_Write_CoCmd( BITMAP_SIZE(NEAREST, REPEAT, REPEAT, 512, 512) );
	FT_API_Write_CoCmd( BITMAP_HANDLE(14) );    // handle for background stars
	FT_API_Write_CoCmd( BITMAP_SOURCE(250*1024L) );   // Starting address in RAM_G
	FT_API_Write_CoCmd( BITMAP_LAYOUT(L4, 16, 32) );  // format
	FT_API_Write_CoCmd( BITMAP_SIZE(NEAREST, BORDER, BORDER, 32, 32) );
	FT_API_Write_CoCmd( DISPLAY() );
	FT_GPU_CoCmd_Swap(phost);

	FT_GPU_HAL_WaitCmdfifo_empty(phost);
}

ft_void_t FT_Info(ft_void_t)
{
	FT_GPU_CoCmd_Dlstart(phost);
	FT_API_Write_CoCmd(CLEAR(1,1,1));
	FT_API_Write_CoCmd(COLOR_RGB(255,255,255));
	FT_GPU_CoCmd_Text_P(phost,FT_DispWidth/2,FT_DispHeight/2,26,OPT_CENTERX|OPT_CENTERY, PSTR("Please tap on each dot"));
	FT_GPU_CoCmd_Calibrate(phost,0);
	FT_API_Write_CoCmd(DISPLAY());
	FT_GPU_CoCmd_Swap(phost);
	FT_GPU_HAL_WaitCmdfifo_empty(phost);

	FT_GPU_CoCmd_Logo(phost);
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
}

/* Nothing beyond this */
