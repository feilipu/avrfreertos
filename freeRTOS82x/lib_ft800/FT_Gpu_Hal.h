/*!
 * \file FT_GPU_HAL.h
 *
 * \author FTDI
 * \date 2013.04.24
 *
 * Copyright 2013 Future Technology Devices International Limited
 *
 * Project: FT800 or EVE compatible silicon
 * File Description:
 *    This file defines the generic APIs of host access layer for the FT800 or EVE compatible silicon.
 *    Application shall access FT800 or EVE resources over these APIs,regardless of I2C or SPI protocol.
 *    I2C and SPI is selected by compiler switch "FT_I2C_MODE"  and "FT_SPI_MODE". In addition, there are
 *    some helper functions defined for FT800 co-processor engine as well as host commands.
 * Revision History:
 */
#ifndef _FT_GPU_HAL_H_
#define _FT_GPU_HAL_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	FT_GPU_I2C_MODE = 0,
	FT_GPU_SPI_MODE,

	FT_GPU_MODE_COUNT,
	FT_GPU_MODE_UNKNOWN = FT_GPU_MODE_COUNT
}FT_GPU_HAL_MODE_E;

typedef enum {
	FT_GPU_HAL_OPENED,
	FT_GPU_HAL_READING,
	FT_GPU_HAL_WRITING,
	FT_GPU_HAL_CLOSED,

	FT_GPU_HAL_STATUS_COUNT,
	FT_GPU_HAL_STATUS_ERROR = FT_GPU_HAL_STATUS_COUNT
}FT_GPU_HAL_STATUS_E;

/*APIs for Host Commands*/
typedef enum {
	FT_GPU_INTERNAL_OSC = 0x48, //default
	FT_GPU_EXTERNAL_OSC = 0x44
}FT_GPU_PLL_SOURCE_T;
typedef enum {
	FT_GPU_PLL_48M = 0x62,	//default
	FT_GPU_PLL_36M = 0x61,
	FT_GPU_PLL_24M = 0x64
}FT_GPU_PLL_FREQ_T;

typedef enum {
	FT_GPU_ACTIVE_M =	 0x00,
	FT_GPU_STANDBY_M =	 0x41,	//default
	FT_GPU_SLEEP_M =	 0x42,
	FT_GPU_POWERDOWN_M = 0x50
}FT_GPU_POWER_MODE_T;

#define FT_GPU_CORE_RESET  (0x68)

typedef struct {
	ft_uint8_t reserved;
}FT_GPU_App_Context_t;

typedef enum {
	FT_GPU_READ = 0,
	FT_GPU_WRITE
}FT_GPU_TRANSFERDIR_T;

typedef struct {
	ft_uint16_t length; //IN and OUT
	ft_uint32_t address;
	ft_uint8_t  *buffer;
}FT_GPU_App_Transfer_t;

typedef struct {
	FT_GPU_HAL_STATUS_E		status;
    ft_uint16_t				ft_cmd_fifo_wp; 	// co-processor fifo write pointer
	FT_GPU_App_Context_t	app_header;			// prototype for application uses
	SemaphoreHandle_t		xINT0Semaphore;		// Create a Semaphore binary flag for the interrupt pending, should read the REG_INT_FLAGS to see which one fired.
}FT_GPU_HAL_Context_t;


// The APIs for reading/writing transfer continuously only with small buffer system
ft_void_t		FT_GPU_HAL_StartTransfer(FT_GPU_HAL_Context_t *host, FT_GPU_TRANSFERDIR_T rw, ft_const_uint32_t addr);
ft_uint8_t		FT_GPU_HAL_Transfer8(FT_GPU_HAL_Context_t *host, ft_const_uint8_t value) __attribute__ ((flatten));
ft_uint16_t		FT_GPU_HAL_Transfer16(FT_GPU_HAL_Context_t *host, ft_const_uint16_t value) __attribute__ ((flatten));
ft_uint32_t		FT_GPU_HAL_Transfer32(FT_GPU_HAL_Context_t *host, ft_const_uint32_t value) __attribute__ ((flatten));
ft_void_t		FT_GPU_HAL_EndTransfer(FT_GPU_HAL_Context_t *host) __attribute__ ((flatten));

//Helper function APIs Read
ft_uint8_t		FT_GPU_HAL_Rd8 (FT_GPU_HAL_Context_t *host, ft_const_uint32_t addr) __attribute__ ((flatten));
ft_uint16_t		FT_GPU_HAL_Rd16(FT_GPU_HAL_Context_t *host, ft_const_uint32_t addr) __attribute__ ((flatten));
ft_uint32_t		FT_GPU_HAL_Rd32(FT_GPU_HAL_Context_t *host, ft_const_uint32_t addr) __attribute__ ((flatten));

//Helper function APIs Write
ft_void_t		FT_GPU_HAL_Wr8 (FT_GPU_HAL_Context_t *host, ft_const_uint32_t addr, ft_const_uint8_t v) __attribute__ ((flatten));
ft_void_t		FT_GPU_HAL_Wr16(FT_GPU_HAL_Context_t *host, ft_const_uint32_t addr, ft_const_uint16_t v) __attribute__ ((flatten));
ft_void_t		FT_GPU_HAL_Wr32(FT_GPU_HAL_Context_t *host, ft_const_uint32_t addr, ft_const_uint32_t v) __attribute__ ((flatten));


/*******************************************************************************/
/*******************************************************************************/

/*The basic APIs Level 1*/
ft_bool_t		FT_GPU_HAL_Open(FT_GPU_HAL_Context_t *host); // API to initialise the SPI interface and enable the Pin 2 Interrupt
ft_void_t		FT_GPU_HAL_Fast(FT_GPU_HAL_Context_t *host); // used because maximum SPI rate of Goldilocks 1284p at 22118400Hz is too fast for initialisation
ft_void_t		FT_GPU_HAL_Close(FT_GPU_HAL_Context_t *host);
//ft_void_t		FT_GPU_HAL_Powercycle(FT_GPU_HAL_Context_t *host,ft_bool_t up); // no reset on Gamdeduino2, unfortunately

/*Preferred public APIs for co-processor Fifo read/write and space management*/
ft_void_t		FT_GPU_HAL_StartCmdTransfer(FT_GPU_HAL_Context_t *host, FT_GPU_TRANSFERDIR_T rw) __attribute__ ((flatten));
ft_uint32_t		FT_GPU_HAL_TransferCmd(FT_GPU_HAL_Context_t *host, ft_const_uint32_t cmd) __attribute__ ((flatten));
ft_void_t		FT_GPU_HAL_EndCmdTransfer(FT_GPU_HAL_Context_t *host) __attribute__ ((flatten));

ft_void_t		FT_GPU_HAL_CheckCmdBuffer(FT_GPU_HAL_Context_t *host,ft_const_uint16_t count)  __attribute__ ((flatten));
ft_void_t		FT_GPU_HAL_Updatecmdfifo(FT_GPU_HAL_Context_t *host, ft_const_uint16_t count) __attribute__ ((flatten));
ft_void_t		FT_GPU_HAL_WrCmd32(FT_GPU_HAL_Context_t *host, ft_const_uint32_t cmd) __attribute__ ((flatten));
ft_void_t		FT_GPU_HAL_WrCmdBuf(FT_GPU_HAL_Context_t *host, ft_const_uint8_t *buffer, ft_uint16_t count) __attribute__ ((flatten));
ft_void_t		FT_GPU_HAL_WrCmdBuf_P(FT_GPU_HAL_Context_t *host, ft_prog_uint8_t *buffer, ft_uint16_t count) __attribute__ ((flatten));
ft_void_t		FT_GPU_HAL_WaitCmdfifo_empty(FT_GPU_HAL_Context_t *host) __attribute__ ((flatten));
ft_void_t		FT_GPU_HAL_ResetCmdFifo(FT_GPU_HAL_Context_t *host) __attribute__ ((flatten));


/*******************************************************************************/
/*******************************************************************************/

ft_void_t		FT_GPU_HostCommand(FT_GPU_HAL_Context_t *host, ft_const_uint8_t cmd);
ft_void_t		FT_GPU_ClockSelect(FT_GPU_HAL_Context_t *host, const FT_GPU_PLL_SOURCE_T pllsource) __attribute__ ((flatten));
ft_void_t		FT_GPU_PLL_FreqSelect(FT_GPU_HAL_Context_t *host, const FT_GPU_PLL_FREQ_T freq) __attribute__ ((flatten));
ft_void_t		FT_GPU_PowerModeSwitch(FT_GPU_HAL_Context_t *host, const FT_GPU_POWER_MODE_T pwrmode) __attribute__ ((flatten));
ft_void_t		FT_GPU_CoreReset(FT_GPU_HAL_Context_t *host) __attribute__ ((flatten));

ft_void_t		FT_GPU_HAL_RdMem(FT_GPU_HAL_Context_t *host, ft_uint32_t addr, ft_uint8_t *buffer, ft_const_uint16_t length) __attribute__ ((flatten));
ft_void_t		FT_GPU_HAL_WrMem(FT_GPU_HAL_Context_t *host, ft_uint32_t addr, ft_const_uint8_t *buffer, ft_const_uint16_t length) __attribute__ ((flatten));
ft_void_t		FT_GPU_HAL_WrMem_P(FT_GPU_HAL_Context_t *host, ft_uint32_t addr, ft_prog_uint8_t *buffer, ft_const_uint16_t length) __attribute__ ((flatten));

ft_uint8_t		FT_GPU_HAL_TransferString(FT_GPU_HAL_Context_t *host, ft_const_char8_t *string) __attribute__ ((flatten));
ft_uint8_t		FT_GPU_HAL_TransferString_P(FT_GPU_HAL_Context_t *host, ft_prog_char8_t *string) __attribute__ ((flatten));

ft_void_t		FT_GPU_HAL_WaitLogo_Finish(FT_GPU_HAL_Context_t *host) __attribute__ ((flatten));

ft_int32_t		FT_GPU_HAL_Dec2ASCII(ft_char8_t *pSrc,ft_int32_t value);

/*******************************************************************************/
/*******************************************************************************/

/* Global variables for display resolution to support various display panels */
extern ft_int16_t FT_DispWidth;
extern ft_int16_t FT_DispHeight;
extern ft_int16_t FT_DispHCycle;
extern ft_int16_t FT_DispHOffset;
extern ft_int16_t FT_DispHSync0;
extern ft_int16_t FT_DispHSync1;
extern ft_int16_t FT_DispVCycle;
extern ft_int16_t FT_DispVOffset;
extern ft_int16_t FT_DispVSync0;
extern ft_int16_t FT_DispVSync1;
extern ft_uint8_t FT_DispPCLK;
extern ft_uint8_t FT_DispSwizzle;
extern ft_uint8_t FT_DispPCLKPol;

/* Global used for HAL management */
extern FT_GPU_HAL_Context_t  host;
extern FT_GPU_HAL_Context_t* phost;

#ifdef __cplusplus
}
#endif

#endif  /* _FT_GPU_HAL_H_ */
