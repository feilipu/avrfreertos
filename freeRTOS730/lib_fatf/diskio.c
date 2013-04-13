/*-----------------------------------------------------------------------*/
/* MMCv3/SDv1/SDv2 (in SPI mode) control module  (C)ChaN, 2010           */
/*-----------------------------------------------------------------------*/
/* Only spiTransfer(0xFF), spiTransfer(), disk_timerproc() and some macros         */
/* are platform dependent.                                               */
/*-----------------------------------------------------------------------*/


#include <avr/io.h>
#include <util/delay.h>
#include <util/crc16.h>

#include <FreeRTOS.h>
#include <task.h>
#include <spi.h>

#if defined( portSD_CARD) || defined(portEXT_RAMFS)

#include <diskio.h>


/*--------------------------------------------------------------------------

   Module Private Functions

---------------------------------------------------------------------------*/

/* Definitions for MMC/SDC command */
#define CMD0	(0)			/* GO_IDLE_STATE */
#define CMD1	(1)			/* SEND_OP_COND (MMC) */
#define	ACMD41	(0x80+41)	/* SEND_OP_COND (SDC) */
#define CMD8	(8)			/* SEND_IF_COND */
#define CMD9	(9)			/* SEND_CSD */
#define CMD10	(10)		/* SEND_CID */
#define CMD12	(12)		/* STOP_TRANSMISSION */
#define CMD13	(13)		/* SD_STATUS (SDC) */
#define ACMD13	(0x80+13)	/* SD_STATUS (SDC) */
#define CMD16	(16)		/* SET_BLOCKLEN */
#define CMD17	(17)		/* READ_SINGLE_BLOCK */
#define CMD18	(18)		/* READ_MULTIPLE_BLOCK */
#define CMD23	(23)		/* SET_BLOCK_COUNT (MMC) */
#define	ACMD23	(0x80+23)	/* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24	(24)		/* WRITE_BLOCK */
#define CMD25	(25)		/* WRITE_MULTIPLE_BLOCK */
#define CMD32	(32)		/* SET ERASE START BLOCK */
#define CMD33	(33)		/* SET ERASE FINISH BLOCK */
#define CMD38	(38)		/* EXECUTE ERASE BLOCK ON CMD32 & CMD33 BLOCKS */
#define CMD55	(55)		/* APP_CMD */
#define CMD58	(58)		/* READ_OCR */


#if defined(portEXT_RAMFS) && ( defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) )
// see the functions implemented in ramfs.c which replicate these functions on Arduino Uno 328p
// but using the SPI bus to reach the ArduSat Supervisor 2560 or 2561.
#else

static volatile
DSTATUS Stat = STA_NOINIT;	/* Disk status */

static
uint8_t CardType;			/* Card type flags */

/*-----------------------------------------------------------------------*/
/* Transmit a byte to MMC via SPI  (Platform dependent)                  */
/*-----------------------------------------------------------------------*/
// Deleted. Replaced by lib_spi functions.

/*-----------------------------------------------------------------------*/
/* Receive a byte from MMC via SPI  (Platform dependent)                 */
/*-----------------------------------------------------------------------*/
// Deleted. Replaced by lib_spi functions.

/*-----------------------------------------------------------------------*/
/* Power Control  (Platform dependent)                                   */
/*-----------------------------------------------------------------------*/
/* When the target system does not support socket power control, there   */
/* is nothing to do in these functions and chk_power always returns 1.   */

static
uint8_t power_status(void)		/* Socket power state: 0=off, 1=on */
{
	return 1;
}


static
void power_on (void)
{
	/* Delay at power on */
	vTaskDelay( 50 / portTICK_RATE_MS ); // wait 50mS at power on.

	/* Enable SPI function in mode 0 */
	spiSetDataMode(SPI_MODE0);

	/* SPI at maximum speed, half of CPU clock*/
	spiSetClockDivider(SPI_CLOCK_DIV2);

#if defined(portSD_CARD) && ( defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__))

	SPI_PORT_DIR_SS_MEGA_SD |= SPI_BIT_SS_MEGA_SD; // enable the EtherMega SD Card with SS on PG5
	SPI_SS_MEGA_SD(1);

	spiBegin(SS_PG5);

#else // Assume standard Arduino. There will be others required, but not just yet.

	SPI_PORT_DIR |= SPI_BIT_SS; // enable the EtherMega SD Card with SS on PG5
	SPI_SS(1);

	spiBegin(SS_PB0)

#endif

}


static
void power_off (void)
{
	/* Disable SPI function */
	spiEnd();
	Stat |= STA_NOINIT;
}


/*-----------------------------------------------------------------------*/
/* Receive a data packet from MMC                                        */
/*-----------------------------------------------------------------------*/

static uint8_t rcvr_datablock (
	uint8_t *buff,			/* Data buffer to store received data */
	uint16_t btr			/* Byte count (must be multiple of 4) */
)
{
	uint8_t token;

	/* Wait for data packet in timeout of 200ms */
	token = 0xFF;
	for (uint8_t i = 100; i && token == 0xFF; --i)
	{
		token = spiTransfer(0xFF);
		vTaskDelay( 2 / portTICK_RATE_MS );		// It is finishing up a prior command. Up to 4mS x 50 delay.
	}

	if(token != 0xFE) return 0;		/* If not valid data token, return with error */

	if (!spiMultiByteRx(buff, btr)) return 0; /* Receive the data block into buffer */

	spiTransfer(0xFF);				// Discard CRC bits, being 16 bits.
	spiTransfer(0xFF);

	return 1;						/* Return with success */
}



/*-----------------------------------------------------------------------*/
/* Send a data packet to MMC                                             */
/*-----------------------------------------------------------------------*/

static uint8_t xmit_datablock (
	const uint8_t *buff,	/* 512 byte data block to be transmitted */
	uint8_t token			/* Data/Stop token */
)
{
	uint8_t resp;

	resp = 0;
	while ((spiTransfer(0xFF) == 0x00) && (--resp != 0)) 	// Long wait while SD busy (0x00 signal).
		vTaskDelay( 2 / portTICK_RATE_MS );					// It is finishing up a prior write command. Busy 0x00. Up to 2mS x 255 delay.

	spiTransfer(token);					/* Xmit data token */
	if (token != 0xFD) {				/* Is data token */

		if (!spiMultiByteTx( buff, 512 )) return 0; /* Xmit the 512 byte data block to MMC */

		spiTransfer(0xDE);				/* CRC (Dummy) 16 bit*/
		spiTransfer(0xAD);

		resp = spiTransfer(0xFF);		/* Receive data response */
		if ((resp & 0x1F) != 0x05)		/* If not accepted (xxx00101), return with error */
			return 0;
	}

	return 1;							/* Return with success */
}



/*-----------------------------------------------------------------------*/
/* Send a command packet to MMC                                          */
/*-----------------------------------------------------------------------*/

static
uint8_t send_cmd (		/* Returns R1 resp (bit7==1:Send failed) */
	uint8_t cmd,		/* Command index */
	uint32_t arg		/* Argument */
)
{
	uint8_t resp;
	uint8_t i = 0;

	do {
		resp = spiTransfer(0xFF);			// Send some clocks for SD to prepare & finish up from last command.
		if (resp == 0x00) vTaskDelay( 2 / portTICK_RATE_MS );	// Busy 0x00. It is finishing up a write command. Up to 2mS x 255 delay.
	} while ((resp != 0xFF) && (--i != 0));

	if (cmd & 0x80) {	/* ACMD<n> is the command sequence of CMD55 + CMD<n> */

		cmd &= 0x7F;

		/* Send CMD55 command packet */
		spiTransfer(0x40 | CMD55);				/* Start + Command index */
		spiTransfer(0x00);		 				/* Argument[31..24] */
		spiTransfer(0x00);						/* Argument[23..16] */
		spiTransfer(0x00);						/* Argument[15..8] */
		spiTransfer(0x00);						/* Argument[7..0] */
		spiTransfer(0x55 | 0x01);				/* Dummy CRC + Stop */

		/* Receive command response */
		i = 8;									/* Wait for a valid response within 8 attempts */
		do resp = spiTransfer(0xFF);
		while ((resp & 0x80) && (--i != 0));

		if (resp > 1) return resp;				// something bad happened.

	}

	/* Send command packet */
	spiTransfer(0x40 | cmd);				/* Start + Command index */
	spiTransfer((uint8_t)(arg >> 24));		/* Argument[31..24] */
	spiTransfer((uint8_t)(arg >> 16));		/* Argument[23..16] */
	spiTransfer((uint8_t)(arg >> 8));		/* Argument[15..8] */
	spiTransfer((uint8_t)arg);				/* Argument[7..0] */
	i = 0xAA | 0x01;						/* Dummy CRC + Stop */
	if (cmd == CMD0) i = 0x95;				/* Valid CRC for CMD0(0) */
	if (cmd == CMD8) i = 0x87;				/* Valid CRC for CMD8(0x1AA) */
	spiTransfer(i);

	/* Receive command response */
	if (cmd == CMD12) spiTransfer(0xFF);	/* Skip a stuff byte when stop reading */

	i = 8;									/* Wait for a valid R1 response within 8 attempts */
	do resp = spiTransfer(0xFF);
	while ((resp & 0x80) && (--i != 0));

	if (cmd == CMD13) spiTransfer(0xFF);	/* Skip R2 response second byte from CMD13 (or ACMD13) */

	return resp;							/* Return with the response value */
}



/*--------------------------------------------------------------------------

   Public Functions

---------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
/* Initialise Disk Drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	uint8_t drv		/* Physical drive number (0) */
)
{
	uint8_t resp, type, ocr[4];

	if (drv) return STA_NOINIT;			// Supports only single drive
	if (Stat & STA_NODISK) return Stat;	// No card in the socket

	power_on();							// Force socket power on

	spiSetDataMode(SPI_MODE0);			// Enable SPI function in mode 0

	spiSetClockDivider(SPI_CLOCK_DIV64); // slow clock down to between 100kHz and 400kHz (250kHz @ 16MHz)

	for (uint8_t i = 10; i; --i) spiTransfer(0xFF);	// 80 dummy clocks; without SD card selected.

	type = 0;							// Set invalid SD card type.

	if (!spiSelect(SS_PG5)) return STA_NOINIT;

	for (uint8_t i = 100; i && ((resp = send_cmd(CMD0, 0)) != 0x01); --i) // try up to 100 times to initialise the SD card.
		vTaskDelay( 4 / portTICK_RATE_MS );

	if ( resp == 0x01)
	{			/* Entered Idle state */

		if ( (resp = send_cmd(CMD8, 0x01AA)) == 0x01)
		{	/* SDv2? */

			for (uint8_t i = 0; i < 4; ++i)
				ocr[i] = spiTransfer(0xFF);		// pop trailing return value of R7 resp

			if ( ocr[2] == 0x01 &&  ocr[3] == 0xAA )
			{	/* The card can work at vdd range of 2.7-3.6V */
				/* Wait for leaving idle state (ACMD41 with HCS bit) */

				for (uint8_t i = 250; i && send_cmd(ACMD41, (uint32_t)1 << 30); --i) /* Initialisation timeout of 1000 msec */
					vTaskDelay( 4 / portTICK_RATE_MS );

				if ( send_cmd(CMD58, 0) == 0x00 )
				{		/* Check CCS bit in the OCR */
					for (uint8_t i = 0; i < 4; ++i) ocr[i] = spiTransfer(0xFF);
					type = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;	/* SDv2 */
				}
			}
		}
		else if (resp == 0x05)
		{	/* SDv1 or MMCv3 */

			for (uint8_t i = 250; i && (resp = send_cmd(ACMD41, 0)) == 0x01; --i) // initialise for 1000mS
				vTaskDelay( 4 / portTICK_RATE_MS );
			if (resp == 0x00) /* SDv1 */
				type = CT_SD1;
			else /* MMCv3 ?? */
			{
				for (uint8_t i = 250; i && (resp = send_cmd(CMD1, 0)) == 0x01; --i) // initialise for 1000mS
					vTaskDelay( 4 / portTICK_RATE_MS );
				if (resp == 0x00) /* MMCv3 */
					type = CT_MMC;
				else
					type = 0;
			}
		}
		else // some kind of unknown error in initialisation.
			type = 0;

		if (type != 0 && type != CT_BLOCK) // If it is NOT a Block Address SD Version 2 device.
			if ( send_cmd(CMD16, 512) != 0x00)	/* Try to set R/W block length to 512 */
				type = 0;
	}

	CardType = type;

	if (type) {			/* Initialisation succeeded */
		Stat &= ~STA_NOINIT;		/* Clear STA_NOINIT */
		spiSetClockDivider(SPI_CLOCK_DIV2); // Maximum speed clock, for maximum performance.
	} else {			/* Initialisation failed */
		power_off();
	}

	spiDeselect(SS_PG5);	// deselect the SD card

	return Stat;
}



/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	uint8_t drv		/* Physical drive number (0) */
)
{
	if (drv != 0) return STA_NOINIT;		/* Supports only single drive */
	return Stat;
}


/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	uint8_t drv,			/* Physical drive number (0) */
	uint8_t *buff,			/* Pointer to the data buffer to store read data */
	uint32_t sector,		/* Start sector number (LBA) */
	uint8_t count			/* Sector count (1..255) */
)
{
	if (drv || !count) return RES_PARERR;
	if (Stat & STA_NOINIT) return RES_NOTRDY;

	if (!spiSelect(SS_PG5)) return RES_NOTRDY;

	if (!(CardType & CT_BLOCK)) sector *= 512;	/* Convert to byte address if needed */

	if (count == 1) {	/* Single block read */
		if ((send_cmd(CMD17, sector) == 0)	/* READ_SINGLE_BLOCK */
			&& rcvr_datablock(buff, 512))
			count = 0;
	}
	else {				/* Multiple block read */
		if (send_cmd(CMD18, sector) == 0) {	/* READ_MULTIPLE_BLOCK */
			do {
				if (!rcvr_datablock(buff, 512)) break;
				buff += 512;
			} while (--count);
			send_cmd(CMD12, 0);				/* STOP_TRANSMISSION */
		}
	}
	spiDeselect(SS_PG5);
	return count ? RES_ERROR : RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

DRESULT disk_write (
	uint8_t drv,			/* Physical drive number (0) */
	const uint8_t *buff,	/* Pointer to the data to be written */
	uint32_t sector,		/* Start sector number (LBA) */
	uint8_t count			/* Sector count (1..255) */
)
{
	if (drv || !count) return RES_PARERR;
	if (Stat & STA_NOINIT) return RES_NOTRDY;
	if (Stat & STA_PROTECT) return RES_WRPRT;

	if (!spiSelect(SS_PG5)) return RES_NOTRDY;

	if (!(CardType & CT_BLOCK)) sector *= 512;	/* Convert to byte address if needed */

	if (count == 1) {	/* Single block write */
		if ((send_cmd(CMD24, sector) == 0)	/* WRITE_BLOCK */
			&& xmit_datablock(buff, 0xFE))
			count = 0;
	}
	else {				/* Multiple block write */
		if (CardType & CT_SDC) send_cmd(ACMD23, count);
		if (send_cmd(CMD25, sector) == 0) {	/* WRITE_MULTIPLE_BLOCK */
			do {
				if (!xmit_datablock(buff, 0xFC)) break;
				buff += 512;
			} while (--count);
			if (!xmit_datablock(0, 0xFD))	/* STOP_TRAN token */
				count = 1;
		}
	}

	spiDeselect(SS_PG5);
	return count ? RES_ERROR : RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	uint8_t drv,		/* Physical drive number (0) */
	uint8_t ctrl,	/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT resp;
	uint8_t n, csd[16], *ptr = buff; // 8 bit char for normal buffer
	uint32_t csize;
	uint32_t *erasePtr = buff; // 32 bit integers for the erase sector (or byte) addresses


	if (drv) return RES_PARERR;

	resp = RES_ERROR;

	if (ctrl == CTRL_POWER) {
		switch (ptr[0]) {
		case 0:		/* Sub control code (POWER_OFF) */
			power_off();		/* Power off */
			resp = RES_OK;
			break;
		case 1:		/* Sub control code (POWER_GET) */
			ptr[1] = (uint8_t)power_status();
			resp = RES_OK;
			break;
		default :
			resp = RES_PARERR;
			break;
		}
	}
	else {
		if (Stat & STA_NOINIT) return RES_NOTRDY;

		switch (ctrl) {
		case CTRL_SYNC :		/* Make sure that no pending write process. Do not remove this or written sector might be left not updated. */
			if (spiSelect(SS_PG5) ){
				spiDeselect(SS_PG5);	// deselect the SD card
				resp = RES_OK;
			}
			break;

		case GET_SECTOR_COUNT :	/* Get number of sectors on the disk (uint32_t) */
			if (!spiSelect(SS_PG5)) resp = RES_NOTRDY;
			if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {
				if ((csd[0] >> 6) == 1) {	/* SDC ver 2.00 */
					csize = csd[9] + ((uint16_t)csd[8] << 8) + ((uint32_t)(csd[7] & 63) << 16) + 1;
					*(uint32_t*)buff = csize << 10;
				} else {					/* SDC ver 1.XX or MMC*/
					n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
					csize = (csd[8] >> 6) + ((uint16_t)csd[7] << 2) + ((uint16_t)(csd[6] & 3) << 10) + 1;
					*(uint32_t*)buff = csize << (n - 9);
				}
				resp = RES_OK;
			}
			spiDeselect(SS_PG5);	// deselect the SD card
			break;

		case GET_SECTOR_SIZE :	/* Get R/W sector size (uint16_t) */
			*(uint16_t*)buff = 512;
			resp = RES_OK;
			break;

		case GET_BLOCK_SIZE :	/* Get erase block size in unit of sector (uint32_t) */
			if (!spiSelect(SS_PG5)) resp = RES_NOTRDY;
			if (CardType & CT_SD2) {	/* SDv2? */
				if ((send_cmd(ACMD13, 0) == 0) 	/* Read SD status */
					&& rcvr_datablock(csd, 16)) {				/* Read partial block */
						for (n = 64 - 16; n; --n) spiTransfer(0xFF);	/* Purge trailing data */
						*(uint32_t*)buff = 16UL << (csd[10] >> 4);
						resp = RES_OK;
					}
			} else {					/* SDv1 or MMCv3 */
				if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {	/* Read CSD */
					if (CardType & CT_SD1) {	/* SDv1 */
						*(uint32_t*)buff = (((csd[10] & 63) << 1) + ((uint16_t)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
					} else {					/* MMCv3 */
						*(uint32_t*)buff = ((uint16_t)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
					}
					resp = RES_OK;
				}
			}
			spiDeselect(SS_PG5);	// deselect the SD card
			break;

		case CTRL_ERASE_SECTOR : // Erase sectors
			if (!spiSelect(SS_PG5)) resp = RES_NOTRDY;

			if (!(CardType & CT_SDC)) break;				/* Check if the card is SDC */
			if (disk_ioctl(drv, MMC_GET_CSD, csd)) break;	/* Get CSD */
			if (!(csd[0] >> 6) && !(csd[10] & 0x40)) break;	/* Check if sector erase can be applied to the card */

			// Check to see if we have a BLOCK card; if not we calculate byte address.
			if (!(CardType & CT_BLOCK)){
				erasePtr[0] *= 512;
				erasePtr[1] *= 512;
			}
			// Set the start and end sectors (or bytes) for erasing.
			if ( (send_cmd(CMD32, erasePtr[0]) == 0) && (send_cmd(CMD33, erasePtr[1]) == 0) ){
				// Erase the nominated sectors. Response is R1b = R1 + 0x00 bytes while busy.
				if (send_cmd(CMD38, 0) == 0) {
					resp = 0;
					while ((spiTransfer(0xFF) == 0x00) && (--resp != 0)) 	// Long wait while SD busy (0x00 signal).
						vTaskDelay( 4 / portTICK_RATE_MS );					// Finishing up a erase command. Busy 0x00. Up to 4ms x 255 delay.
					resp = RES_OK;
				} else {
					resp = RES_ERASE_ERROR;
				}
			} else {
				resp = RES_ERASE_ERROR;
			}
			spiDeselect(SS_PG5);	// deselect the SD card
			break;

		case MMC_GET_TYPE :		/* Get card type flags (1 byte) */
			*ptr = CardType;
			resp = RES_OK;
			break;

		case MMC_GET_CSD :		/* Receive CSD as a data block (16 bytes) */
			if (!spiSelect(SS_PG5)) resp = RES_NOTRDY;
			if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(ptr, 16))	/* READ_CSD */
				resp = RES_OK;
			spiDeselect(SS_PG5);	// deselect the SD card
			break;

		case MMC_GET_CID :		/* Receive CID as a data block (16 bytes) */
			if (!spiSelect(SS_PG5)) resp = RES_NOTRDY;
			if ((send_cmd(CMD10, 0) == 0) && rcvr_datablock(ptr, 16))	/* READ_CID */
				resp = RES_OK;
			spiDeselect(SS_PG5);	// deselect the SD card
			break;

		case MMC_GET_OCR :		/* Receive OCR as an R3 response (4 bytes) */
			if (!spiSelect(SS_PG5)) resp = RES_NOTRDY;
			if (send_cmd(CMD58, 0) == 0) {	/* READ_OCR */
				for (n = 4; n; --n) *ptr++ = spiTransfer(0xFF);
				resp = RES_OK;
			}
			spiDeselect(SS_PG5);	// deselect the SD card
			break;

		case MMC_GET_SDSTAT :	/* Receive SD status as a data block (64 bytes) */
			if (!spiSelect(SS_PG5)) resp = RES_NOTRDY;
			if ((send_cmd(ACMD13, 0) == 0) && rcvr_datablock(ptr, 64))	/* SD_STATUS */
				resp = RES_OK;
			spiDeselect(SS_PG5);	// deselect the SD card
			break;

		default:
			resp = RES_PARERR;
			break;
		}

	}

	return resp;
}

#endif

#endif
