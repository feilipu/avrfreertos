/* Sample API for the FTDI FT800 EVE */

#include "FT_Platform.h"

/* Global used for HAL management */
extern FT_GPU_HAL_Context_t host;
extern FT_GPU_HAL_Context_t * phost;

/* Index into the Display List Buffer */
static ft_uint32_t FT_DLBuffer_Index;

ft_void_t FT_API_Write_CoCmd(ft_const_uint32_t cmd)
{
	FT_GPU_HAL_WrCmd32( phost, cmd);
}

ft_void_t FT_API_Write_DLCmd(ft_const_uint32_t cmd)
{
	FT_GPU_HAL_Wr32( phost, (RAM_DL + FT_DLBuffer_Index), cmd);
	/* Increment the DL Buffer index */
	FT_DLBuffer_Index += FT_CMD_SIZE;
}

ft_void_t FT_API_Reset_DLBuffer( ft_void_t )
{
	/* Reset the DL Buffer index, start writing at RAM_DL first byte */
	FT_DLBuffer_Index = 0;
}

/* API to check the status of previous DLSWAP and perform DLSWAP of new DL */
/* Check for the status of previous DLSWAP and if still not done wait for few ms and check again */
ft_void_t FT_API_GPU_DLSwap(const ft_uint8_t DL_Swap_Type)
{
	ft_uint8_t Swap_Type = DLSWAP_FRAME;
	ft_uint8_t Swap_Done = DLSWAP_FRAME;

	if(DL_Swap_Type == DLSWAP_LINE)
	{
		Swap_Type = DLSWAP_LINE;
	}

	/* Perform a new DL swap */
	FT_GPU_HAL_Wr8(phost,REG_DLSWAP,Swap_Type);

	/* Wait till the swap is done */
	while(Swap_Done)
	{
		Swap_Done = FT_GPU_HAL_Rd8(phost,REG_DLSWAP);

		if(DLSWAP_DONE != Swap_Done)
			vTaskDelay( 1 );	//sleep for 1 system tick.
	}
}

/* API to wait until the command buffer is empty, following CMD_SWAP */
ft_void_t FT_API_WaitCmdfifo_empty(ft_void_t)
{
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
}

/* API to give fade out effect by changing the display PWM from 128 till 0 */
ft_void_t FT_API_fadeout(ft_void_t)
{
	for (ft_uint8_t i = 128; i >= 0; i -= 3)
	{
		FT_GPU_HAL_Wr8(phost,REG_PWM_DUTY,i);
		vTaskDelay( 1 );//sleep for 1 tick
	}
	/* Finally ensure the PWM is 0% */
	FT_GPU_HAL_Wr8(phost,REG_PWM_DUTY,0x00); // 0
}

/* API to perform display fade in effect by changing the display PWM from 0 till 100 and finally 128 */
ft_void_t FT_API_fadein(ft_void_t)
{
	for (ft_uint8_t i = 0; i <=128 ; i += 3)
	{
		FT_GPU_HAL_Wr8(phost,REG_PWM_DUTY,i);
		vTaskDelay( 1 );//sleep for 1 tick
	}
	/* Finally ensure the PWM is 100% */
	FT_GPU_HAL_Wr8(phost,REG_PWM_DUTY,0x80); // 128
}

/* Optimized implementation of sin and cos table - precision is 16 bit */
ft_prog_uint16_t sintab[] PROGMEM = {
0, 402, 804, 1206, 1607, 2009, 2410, 2811, 3211, 3611, 4011, 4409, 4807, 5205, 5601, 5997, 6392,
6786, 7179, 7571, 7961, 8351, 8739, 9126, 9511, 9895, 10278, 10659, 11038, 11416, 11792, 12166, 12539,
12909, 13278, 13645, 14009, 14372, 14732, 15090, 15446, 15799, 16150, 16499, 16845, 17189, 17530, 17868,
18204, 18537, 18867, 19194, 19519, 19840, 20159, 20474, 20787, 21096, 21402, 21705, 22004, 22301, 22594,
22883, 23169, 23452, 23731, 24006, 24278, 24546, 24811, 25072, 25329, 25582, 25831, 26077, 26318, 26556, 26789,
27019, 27244, 27466, 27683, 27896, 28105, 28309, 28510, 28706, 28897, 29085, 29268, 29446, 29621, 29790, 29955,
30116, 30272, 30424, 30571, 30713, 30851, 30984, 31113, 31236, 31356, 31470, 31580, 31684, 31785, 31880, 31970,
32056, 32137, 32213, 32284, 32350, 32412, 32468, 32520, 32567, 32609, 32646, 32678, 32705, 32727, 32744, 32757,
32764, 32767, 32764};

ft_int16_t FT_API_qsin(ft_uint16_t a)
{
  ft_uint8_t f;
  ft_int16_t s0,s1;

  if (a & 32768)
    return -FT_API_qsin(a & 32767);
  if (a & 16384)
      a = 32768 - a;
  f = a & 127;
  s0 = pgm_read_word(sintab + (a >> 7));
  s1 = pgm_read_word(sintab + (a >> 7) + 1);
  return (s0 + ((ft_int32_t)f * (s1 - s0) >> 7));
}

/* cos function */
ft_int16_t FT_API_qcos(ft_uint16_t a)
{
  return (FT_API_qsin(a + 16384));
}

// >>> [int16t(round(1024 * atan(i / 256.0) / pi)) for i in range(256)]
ft_prog_uint8_t atan8[] PROGMEM = {
0,1,3,4,5,6,8,9,10,11,13,14,15,17,18,19,20,22,23,24,25,27,28,29,30,32,33,34,36,    \
37,38,39,41,42,43,44,46,47,48,49,51,52,53,54,55,57,58,59,60,62,63,64,65,67,68,     \
69,70,71,73,74,75,76,77,79,80,81,82,83,85,86,87,88,89,91,92,93,94,95,96,98,99,100, \
101,102,103,104,106,107,108,109,110,111,112,114,115,116,117,118,119,120,121,122,   \
124,125,126,127,128,129,130,131,132,133,134,135,137,138,139,140,141,142,143,144,   \
145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,160,161,162,163,164,   \
165,166,167,168,169,170,171,172,173,174,175,176,177,177,178,179,180,181,182,183,   \
184,185,186,187,188,188,189,190,191,192,193,194,195,195,196,197,198,199,200,201,   \
201,202,203,204,205,206,206,207,208,209,210,211,211,212,213,214,215,215,216,217,   \
218,219,219,220,221,222,222,223,224,225,225,226,227,228,228,229,230,231,231,232,   \
233,234,234,235,236,236,237,238,239,239,240,241,241,242,243,243,244,245,245,246,   \
247,248,248,249,250,250,251,251,252,253,253,254,255,255                            \
};

/* atan function */
ft_uint16_t FT_API_qatan(int16_t y, int16_t x)
{
  ft_uint16_t a;
  ft_uint16_t xx = 0;

  if ((x <= 0) ^ (y > 0))
  {
    int16_t t;
    t = x;
    x = y;
    y = t;
    xx ^= 0x4000;
  }
  if (x <= 0) {
    x = -x;
  } else {
    xx ^= 0x8000;
  }
  y = abs(y);
  if (x > y)
  {
    int16_t t;
    t = x;
    x = y;
    y = t;
    xx ^= 0x3fff;
  }
  while ((x | y) & 0xff80)
  {
    x >>= 1;
    y >>= 1;
  }
  if (y == 0) {
    a = 0;
  } else if (x == y) {
    a = 0x2000;
  } else {
    // assert(x <= y);
    uint16_t r = ((x << 8) / y);
    // assert(0 <= r);
    // assert(r < 256);
    a = pgm_read_byte(atan8 + r) << 5;
  }
  a ^= xx;
  return a;
}


ft_void_t FT_API_Boot_Config(ft_void_t)  // you must do this first TO OPEN the API and initialise the Gameduino2
{
	/* Global used for HAL management */
	FT_GPU_HAL_Open(&host);
	phost = &host;

	/* Access address 0 to wake up the FT800 */
	FT_GPU_HostCommand(phost, FT_GPU_ACTIVE_M);
	vTaskDelay( 32 / portTICK_PERIOD_MS ); // assuming waking from POWERDOWN or SLEEP. From STANDBY the delay is unnecessary.

	/* Set the clk to internal clock (default anyway) */
	FT_GPU_HostCommand(phost, FT_GPU_INTERNAL_OSC);

	/* Switch PLL output to 48MHz */
	FT_GPU_HostCommand(phost, FT_GPU_PLL_48M);

	/* set the SPI bus to full speed, in case we're using a Goldilocks where SCK/2 is too fast for FT800 startup */
	FT_GPU_HAL_Fast(phost);

	/* Do a core reset for safety */
	FT_GPU_HostCommand(phost, FT_GPU_CORE_RESET);

    {
        ft_uint8_t chipid, i = 0;
        //Read Register ID to check if FT800 is ready.
        chipid = FT_GPU_HAL_Rd8(phost, REG_ID);
        while((chipid != 0x7C) && (++i != 0) )
            chipid = FT_GPU_HAL_Rd8(phost, REG_ID);
    }

    FT_GPU_HAL_Wr8(phost,  REG_GPIO_DIR, 0x80 | FT_GPU_HAL_Rd8(phost, REG_GPIO_DIR));
    FT_GPU_HAL_Wr8(phost,  REG_GPIO,     0x80 | FT_GPU_HAL_Rd8(phost, REG_GPIO));

    FT_GPU_HAL_Wr8(phost,  REG_PCLK,	 0);			//after this, the display is turned off

    /* Configuration of LCD display */
    FT_GPU_HAL_Wr8(phost,  REG_PCLK_POL, FT_DispPCLKPol);
    FT_GPU_HAL_Wr8(phost,  REG_SWIZZLE,  FT_DispSwizzle);
    FT_GPU_HAL_Wr16(phost, REG_HSIZE,    FT_DispWidth);
    FT_GPU_HAL_Wr16(phost, REG_VSIZE,    FT_DispHeight);
    FT_GPU_HAL_Wr16(phost, REG_HCYCLE,   FT_DispHCycle);
    FT_GPU_HAL_Wr16(phost, REG_HOFFSET,  FT_DispHOffset);
    FT_GPU_HAL_Wr16(phost, REG_HSYNC0,   FT_DispHSync0);
    FT_GPU_HAL_Wr16(phost, REG_HSYNC1,   FT_DispHSync1);
    FT_GPU_HAL_Wr16(phost, REG_VCYCLE,   FT_DispVCycle);
    FT_GPU_HAL_Wr16(phost, REG_VOFFSET,  FT_DispVOffset);
    FT_GPU_HAL_Wr16(phost, REG_VSYNC0,   FT_DispVSync0);
    FT_GPU_HAL_Wr16(phost, REG_VSYNC1,   FT_DispVSync1);

    FT_GPU_HAL_Wr8(phost,  REG_CSPREAD,  FT_TRUE);
    FT_GPU_HAL_Wr8(phost,  REG_DITHER,   FT_TRUE);
	FT_GPU_HAL_Wr8(phost,  REG_ROTATE,   FT_TRUE);		// xxx rotates the screen by 180 degrees


    FT_GPU_HAL_Wr8(phost,  REG_PCLK,     FT_DispPCLK);	//after this, the display is visible on the LCD

    FT_GPU_HAL_Wr8(phost,  REG_GPIO_DIR, 0x83 | FT_GPU_HAL_Rd8(phost,  REG_GPIO_DIR));
    FT_GPU_HAL_Wr8(phost,  REG_GPIO,     0x83 | FT_GPU_HAL_Rd8(phost,  REG_GPIO));

    /* Touch configuration - configure the resistance value to 1200 - this value is specific to customer requirement and derived by experiment */
    FT_GPU_HAL_Wr16(phost, REG_TOUCH_RZTHRESH, 1200);

    /* Set up Touch and other Interrupts */
    FT_GPU_HAL_Wr8(phost,  REG_INT_MASK, 0);    // Interrupt masks available: INT_CONVCOMPLETE, INT_CMDFLAG, INT_CMDEMPTY, INT_PLAYBACK, INT_SOUND, INT_TAG, INT_TOUCH, INT_SWAP
    FT_GPU_HAL_Wr8(phost,  REG_INT_EN, 0x01);   // Global enable for interrupts attached to INT0 on 328p or 1284p

	/* Reset the DL Buffer index, to start writing at RAM_DL first byte */
	FT_DLBuffer_Index = 0;

	/* start a new display list and clear the screen  */
	FT_GPU_HAL_WrCmd32(phost, CMD_DLSTART);						// initialise and start a display list
	FT_GPU_HAL_WrCmd32(phost, CLEAR(1,1,1));
	FT_GPU_HAL_WrCmd32(phost, CMD_SWAP);					 	// Do a DL swap to render the DL
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
}

ft_void_t FT_API_Touch_Config(ft_void_t)
{
	FT_GPU_HAL_WrCmd32(phost, CMD_DLSTART );
	FT_GPU_HAL_WrCmd32(phost, CLEAR_COLOR_X11(BLACK) );
	FT_GPU_HAL_WrCmd32(phost, CLEAR(1,1,1) );
	FT_GPU_HAL_WrCmd32(phost, COLOR_X11(GREEN) );
	FT_GPU_CoCmd_Text_P(phost, FT_DispWidth/2,FT_DispHeight/2,27,OPT_CENTERX|OPT_CENTERY, PSTR("Please tap on each dot!"));
	FT_GPU_HAL_WrCmd32(phost, CMD_CALIBRATE );
	FT_GPU_HAL_WrCmd32(phost, DISPLAY() );
	FT_GPU_HAL_WrCmd32(phost, CMD_SWAP );
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
}

/* Nothing beyond this */
