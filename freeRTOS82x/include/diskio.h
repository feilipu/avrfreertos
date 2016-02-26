/*-----------------------------------------------------------------------/
/  Low level disk interface module include file   (C)ChaN, 2014          /
/-----------------------------------------------------------------------*/

#ifndef _DISKIO_DEFINED
#define _DISKIO_DEFINED

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define SD_SPI_DIVIDER 	SPI_CLOCK_DIV2 // xxx single point to control SPI speed when using SD cards

/* Status of Disk Functions */
typedef uint8_t	DSTATUS;

/* Results of Disk Functions */
typedef enum {
	RES_OK = 0,		/* 0: Successful */
	RES_ERROR,		/* 1: R/W Error */
	RES_WRPRT,		/* 2: Write Protected */
	RES_NOTRDY,		/* 3: Not Ready */
	RES_PARERR,		/* 4: Invalid Parameter */
	RES_TRIM_ERROR, /* 5: Error executing Sector Erase (Trim)*/
	RES_PENDING		/* 6: Result pending (Used for ArduSat SPI NetworkFS where diskio.c is on different machine to ff.c) */
} DRESULT;

/*
 * Disk Status Bits (uint8_t)
 *
 */

#define STA_NOINIT		0x01	/* Drive not initialised */
#define STA_NODISK		0x02	/* No medium in the drive */
#define STA_PROTECT		0x04	/* Write protected */


/*
 * Command codes for disk_ioctrl function
 *
 */

/* Generic command (used by FatFs) */
#define CTRL_SYNC			0	/* Flush disk cache (for write functions) */
#define GET_SECTOR_COUNT	1	/* Get media size (for only f_mkfs()) */
#define GET_SECTOR_SIZE		2	/* Get sector size (for multiple sector size (_MAX_SS >= 1024)) */
#define GET_BLOCK_SIZE		3	/* Get erase block size (for only f_mkfs()) */
#define CTRL_TRIM			4	/* Force erased a block of sectors (for only _USE_TRIM) */

/* Generic command (not used by FatFs) */
#define CTRL_POWER			5	/* Get/Set power status */
#define CTRL_LOCK			6	/* Lock/Unlock media removal */
#define CTRL_EJECT			7	/* Eject media */
#define CTRL_FORMAT			8	/* Create physical format on the media */

/* MMC/SDC specific ioctl command */
#define MMC_GET_TYPE		10	/* Get card type */
#define MMC_GET_CSD			11	/* Get CSD */
#define MMC_GET_CID			12	/* Get CID */
#define MMC_GET_OCR			13	/* Get OCR */
#define MMC_GET_SDSTAT		14	/* Get SD status */

/* ATA/CF specific ioctl command */
#define ATA_GET_REV			20	/* Get F/W revision */
#define ATA_GET_MODEL		21	/* Get model name */
#define ATA_GET_SN			22	/* Get serial number */

/* NAND specific ioctl command */
#define NAND_FORMAT			30	/* Create physical format */

/* MMC card type flags (MMC_GET_TYPE) */
#define CT_MMC				0x01		/* MMC ver 3 */
#define CT_SD1				0x02		/* SD ver 1 */
#define CT_SD2				0x04		/* SD ver 2 */
#define CT_SDC				(CT_SD1|CT_SD2)	/* SD */
#define CT_BLOCK			0x08		/* Block addressing */


/*---------------------------------------
 *
 * Prototypes for public disk control functions
 *
 *---------------------------------------*/

DSTATUS disk_initialize (uint8_t pdrv);
DSTATUS disk_status (uint8_t pdrv);
DRESULT disk_read (uint8_t pdrv, uint8_t* buff, uint32_t sector, uint8_t count);
DRESULT disk_write (uint8_t pdrv, uint8_t const * buff, uint32_t sector, uint8_t count);
DRESULT disk_ioctl (uint8_t pdrv, uint8_t cmd, void* buff);



#ifdef __cplusplus
}
#endif

#endif
