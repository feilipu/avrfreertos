/*-----------------------------------------------------------------------*/
/* MMCv3/SDv1/SDv2 (in SPI mode) control module  (C)ChaN, 2015           */
/*-----------------------------------------------------------------------*/
/* Only spiTransfer(0xFF), spiTransfer(), disk_timerproc() and some macros         */
/* are platform dependent.                                               */
/*-----------------------------------------------------------------------*/

#include <avr/io.h>
#include <util/delay.h>
#include <util/crc16.h>

#include "FreeRTOS.h"
#include "task.h"
#include "spi.h"

#if defined( portSD_CARD) || defined(portEXT_RAMFS)

#include "diskio.h"

/* serial interface include file. */
#include "serial.h"

/* CRC and utility byte swaps */
#include "lib_util.h"


/*--------------------------------------------------------------------------

   Module Private Definitions

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
#define CMD59   (59)		/* CRC_ON_OFF */

// Return from send_cmd with two bytes to allow for R2 responses.
// R1 responses are contained in the MSB of the word.
// R2 responses consist of R1 followed by the LSB of the word.

#define R2_CARD_LOCKED			0x0001  /** status bit for card locked by user */
#define R2_WRITE_VIOLATION		0x0002  /** status bit for write protect or lock / unlock sequence error */
#define R2_UNKNOWN_ERROR		0x0004  /** status bit for generalised fubar */
#define R2_CC_ERROR				0x0008  /** status bit for internal card controller error */
#define R2_CARD_ECC_FAIL		0x0010  /** status bit for internal ECC application failed, therefore invalid data */
#define R2_WRITE_PROTECT		0x0020  /** status bit for attempted write of write protected block */
#define R2_ERASE_PARAM_ERROR	0x0040  /** status bit for invalid selection for erase, sectors, or groups */
#define R2_RANGE_ERROR			0x0080  /** status bit for out of range or attempted CSD overwrite error */


#define R1_READY_STATE			0x0000	/** status for card in the ready state */

#define R1_IDLE_STATE			0x0100	/** status for card in the idle state */
#define R1_ERASE_RESET			0x0200  /** status for erase sequence cleared before executing, out of sequence command */
#define R1_ILLEGAL_COMMAND		0x0400	/** status bit for illegal command */
#define R1_CRC_ERROR			0x0800  /** status bit for CRC check for last command failed */
#define R1_ERASE_SEQ_ERROR		0x1000  /** status bit for erase command sequence error */
#define R1_ADDRESS_ERROR		0x2000  /** status bit for misaligned address not matching block length */
#define R1_PARAMETER_ERROR		0x4000  /** status bit for parameter error in command's argument outside this card range */

#define WRITE_MULTIPLE_TOKEN	0xFC	/** start data token for write multiple blocks*/
#define STOP_TRAN_TOKEN			0xFD	/** stop token for write multiple blocks*/

#define DATA_START_BLOCK		0xFE	/** start data token for read or write single block*/

#define DATA_RES_ACCEPTED		0x05	/** write data accepted token */
#define DATA_RES_CRC_ERROR		0x0B	/** write data failed CRC error token */
#define DATA_RES_WRITE_ERROR	0x0D	/** write data failed write error */

#define DATA_RES_MASK			0x1F	/** mask for data response tokens after a write block operation */
#define R1_RES_MASK				0x7F	/** mask for R1 result mask */

#define HIGH_CAPACITY_SUPPORT (0x40000000) // High Capacity Support argument used in ACMD41 during initialisation.
#define CRC_ON                (0x00000001)
#define CRC_OFF               (0x00000000)
#define CRC_INIT				(0x0000)

#define READ_ATTEMPTS			(3)		/** Attempts to read block. */
#define WRITE_ATTEMPTS			(3)		/** Attempts to write block. */

#if !defined(portEXT_RAMFS) && !( defined(_UNO_) )
// see the functions implemented in ramfs.c which replicate these functions on Arduino Uno 328p
// but using the SPI bus to reach the ArduSat Supervisor 2560 or 2561.

static volatile
DSTATUS Stat = STA_NOINIT;	/* Disk status */

static
uint8_t CardType;			/* Card type flags */

typedef struct
  {
	  uint8_t cmd;
	  uint32_t arg;
	  uint8_t crc;
  } sdCommand;


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
	vTaskDelay( 64 / portTICK_PERIOD_MS ); // wait 50mS at power on.

	/* Enable SPI function in mode 0 */
	spiSetDataMode(SPI_MODE0);

	/* SPI at maximum speed */
	spiSetClockDivider(SD_SPI_DIVIDER);

	spiBegin(SDCard);
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
) __attribute__ ((flatten));

static uint8_t rcvr_datablock (
	uint8_t *buff,			/* Data buffer to store received data */
	uint16_t btr			/* Byte count (must be multiple of 4) */
)
{
	uint16_t i = 0;
	uint8_t token;
	uint16_t crc;

	/* Wait for data packet*/
	while ( (--i != 0) && ((token = spiTransfer(0xFF)) == 0xFF) )	// Long wait while SD not ready (0xFF signal).
		vTaskDelay( 0 );											// It is finishing up a prior command, so swap out till next tick.

	if(token != DATA_START_BLOCK) return 0;							/* If not valid data token, return with error */

	if (!spiMultiByteRx(buff, btr)) return 0;						/* Receive the data block into buffer */

	crc = spiTransfer(0xFF) << 8;									// Capture CRC bits, being 16 bits.
	crc |= spiTransfer(0xFF);

	return crc == crc16(buff, btr, CRC_INIT);						/* Return with success if CRC16 is correct */
}



/*-----------------------------------------------------------------------*/
/* Send a data packet to MMC                                             */
/*-----------------------------------------------------------------------*/

static uint8_t xmit_datablock (
	const uint8_t *buff,	/* 512 byte data block to be transmitted */
	uint8_t token			/* Data/Stop token */
) __attribute__ ((flatten));

static uint8_t xmit_datablock (
	const uint8_t *buff,	/* 512 byte data block to be transmitted */
	uint8_t token			/* Data/Stop token */
)
{
	uint16_t i = 0;
	uint16_t crc;

	crc = crc16(buff, 512, CRC_INIT);								// prepare a CRC16 on the data.

	while ( (--i != 0) && (spiTransfer(0xFF) == 0x00) )				// Long wait while SD busy (0x00 signal).
		vTaskDelay( 0 );											// It is finishing up a prior command, so swap out till next tick.

	spiTransfer(token);												/* Xmit data token */
	if (token != STOP_TRAN_TOKEN) {									/* Is not a stop data transmission token? */

		if (!spiMultiByteTx( buff, 512 )) return 0;					/* Xmit the 512 byte data block to MMC */

		spiTransfer( (uint8_t)(crc >> 8) );							/* CRC 16 bit*/
		spiTransfer( (uint8_t)crc );

		return (spiTransfer(0xFF) & DATA_RES_MASK);					/* Receive data response. If accepted then xxx00101 */
	}
	return DATA_RES_ACCEPTED;										/* STOP_TRAN_TOKEN received so return with success */
}



/*-----------------------------------------------------------------------*/
/* Send a command packet to MMC                                          */
/*-----------------------------------------------------------------------*/

static
uint16_t send_cmd (		/* Returns R1/R2 resp (bit7==1:Send failed) */
	uint8_t cmd,		/* Command index */
	const uint32_t arg	/* Argument */
)
{
	sdCommand send;
	uint16_t resp;
	uint16_t i;

	if (cmd & 0x80)							/* ACMD<n> is the command sequence of CMD55 + CMD<n> */
	{
		/* Prepare the special CMD55 precursor command */
		send.cmd = 0x40 | CMD55;			/* Start Bit + Command index */
		send.arg = 0x00;					/* Stuff Bits: Null argument for CMD55 */
		send.crc = (crc7((uint8_t *)&send, sizeof(send) -1 ) << 1) | 0x01; /* CRC + Stop */

//		send.crc = (0x55 | 0x01);			/* Dummy CRC + Stop */

		i = 0;
		while ( (--i != 0) && (spiTransfer(0xFF) != 0xFF) ) 	// Long wait while SD busy (0x00 signal).
			vTaskDelay( 0 );				// It is finishing up a prior command, so swap out.
											// Busy signal 0x00, up to 65536 system ticks.

		/* Send special CMD55 as precursor to Alternate Command packet */
		spiMultiByteTx((uint8_t *)&send, sizeof(send)); // send the Command and a Null Argument.

		/* Receive command response */
		i = 8;								/* Wait for a valid response within 8 attempts */
		do
			resp = spiTransfer(0xFF);
		while ( (--i != 0) && (resp & 0x80) );

		if (resp > 0x01) return resp;		// something bad happened so we didn't see R1 = 0x01
	}

	/* Prepare the CMD packet */
	send.cmd = 0x40 | (cmd & 0x7f);			/* Start Bit + Command index */
	send.arg = swaplong(arg);				/* Command Argument, swapped around */
	send.crc = (crc7((uint8_t *)&send, sizeof(send) -1 ) << 1) | 0x01; /* CRC + Stop */

//	send.crc = 0xAA | 0x01;					/* Dummy CRC + Stop */
											/* Special case Commands needing valid CRC + Stop */
//	if (cmd == CMD0) send.crc = 0x95;		/* Revise with a Valid CRC + Stop for CMD0(0) */
//	if (cmd == CMD8) send.crc = 0x87;		/* Revise with a Valid CRC + Stop for CMD8(0x1AA) */

	if (cmd != CMD0)
	{
		i = 0;
		while ( (--i != 0) && (spiTransfer(0xFF) != 0xFF) ) 	// Long wait while SD busy (0x00 signal).
			vTaskDelay( 0 );				// It is finishing up a prior command, so swap out.
											// Busy signal 0x00, up to 65536 system ticks.
	}

	/* Send normal Command packet */
	spiMultiByteTx((uint8_t *)&send, sizeof(send)); // send a normal Command and a valid Argument.

	/* Receive command response */
	if ( cmd == CMD12 )						/* Skip a stuff byte for command CMD12 */
		spiTransfer(0xFF);

	i = 8;									/* Wait for a valid R1 response within 8 attempts */
	do
		resp = spiTransfer(0xFF);
	while ( (--i != 0) && (resp & 0x80) );

	resp <<= 8;								/* shift the first response byte up */

	if ( cmd == CMD12 || cmd == CMD38 )		/* Skip busy signal for command CMD12 (stop transmission) and CMD38 (erase) */
	{
		i = 0;
		while ( (--i != 0) && (spiTransfer(0xFF) == 0x00) )
			;								// Wait while SD busy (0x00 signal).
	}

	if (cmd == CMD13 || cmd == ACMD13)		/* Capture R2 response second byte from CMD13 (or ACMD13),*/
		resp |= spiTransfer(0xFF);			/* collect a R2 second byte response*/

	return resp;							/* Return with the R1 (and R2) response value in uint16 (two bytes) */
}



/*--------------------------------------------------------------------------

   Public Functions

---------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
/* Initialise Disk Drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE drv		/* Physical drive number (default is 0) */
)
{
	uint8_t type, ocr[4];
	uint16_t resp;

	Stat = STA_NOINIT;								// Set uninitialised, initially.

	if (drv)
		return Stat;								// Supports only single drive

#if defined(_GOLDILOCKS_)
	SD_CARD_DETECT_DIR &= ~SD_CARD_DETECT_BIT;		// set CD for input
	SD_CARD_DETECT_PORT |= SD_CARD_DETECT_BIT;		// set CD pull up
	__asm__ __volatile__("nop");					// no operation for synchronisation
	if(SD_CARD_DETECT_PIN & SD_CARD_DETECT_BIT)		// check whether the CD line is held high (SD card absent)
		Stat |= STA_NODISK;
#endif

	if (Stat & STA_NODISK)
		return Stat;								// No card in the socket

	power_on();										// Force socket power on

	spiSetDataMode(SPI_MODE0);						// Set Mode 0
	spiSetClockDivider(SPI_CLOCK_DIV128);			// Slow clock down to between 100kHz and 400kHz (125kHz @ 16MHz)

	for (uint8_t i = 100; i; --i) spiTransfer(0xFF);	// 800 (minimum 74) dummy clocks on SPI bus; without SD card selected.
														// Some SD cards are really slow to get started.

	type = 0;										// Set invalid SD card type, initially.

	if (!spiSelect(SDCard)) return STA_NOINIT;

	for (uint8_t i = 250; i && ((resp = send_cmd(CMD0, 0)) != R1_IDLE_STATE); --i) // try up to four seconds to initialise the SD card.
	{
		spiTransfer(0xFF);									/* Give SD Card 8 Clocks to complete command. */
		vTaskDelay( 16 / portTICK_PERIOD_MS );
	}

	if ( resp == R1_IDLE_STATE)
	{			/* Entered Idle state */

		send_cmd(CMD59, CRC_ON);				// Set the Card to use the command CRC7 and data CRC16 capability.

		if ( (resp = send_cmd(CMD8, 0x01AA)) == R1_IDLE_STATE) // 0x01 is 2.7-3.6V supply. 0xAA is to provide echo source.
		{	/* SDv2 */

			for (uint8_t i = 0; i < 4; ++i)
				ocr[i] = spiTransfer(0xFF);		// pop trailing return value of R7 response

			if ( ocr[2] == 0x01 &&  ocr[3] == 0xAA )
			{	/* The card can work at vdd range of 2.7-3.6V. And the echo bits were correctly replied */

				/* Wait for leaving idle state (ACMD41 with HCS bit) */
				for (uint8_t i = 250; i && (send_cmd(ACMD41, HIGH_CAPACITY_SUPPORT) == R1_IDLE_STATE) ; --i) /* Initialisation timeout of 2 second (High Capacity Support) Bit 30*/
				{
					spiTransfer(0xFF);								/* Give SD Card 8 Clocks to complete command. */
					vTaskDelay( 32 / portTICK_PERIOD_MS );
				}

				if ( send_cmd(CMD58, 0) == R1_READY_STATE )
				{		/* Check CCS bit in the OCR */
					for (uint8_t i = 0; i < 4; ++i) ocr[i] = spiTransfer(0xFF);
					type = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;	/* SDv2 Card Capacity Status check. CCS=1 means SDHC or SDXC */
				}
			}
		}
		else if ( resp == (R1_ILLEGAL_COMMAND | R1_IDLE_STATE) )
		{	/* SDv1 or MMCv3 legacy card*/

			for (uint8_t i = 250; i && ((resp = send_cmd(ACMD41, 0x00)) == R1_IDLE_STATE); --i) // initialise for 2 second
			{
				spiTransfer(0xFF);									/* Give SD Card 8 Clocks to complete command. */
				vTaskDelay( 32 / portTICK_PERIOD_MS );
			}
			if (resp == 0x00) /* SDv1 */
				type = CT_SD1;
			else /* MMCv3 ?? */
			{
				for (uint8_t i = 250; i && ((resp = send_cmd(CMD1, 0x00)) == R1_IDLE_STATE) ; --i) // initialise for 2 second
				{
					spiTransfer(0xFF);								/* Give SD Card 8 Clocks to complete command. */
					vTaskDelay( 32 / portTICK_PERIOD_MS );
				}
				if (resp == 0x00) /* MMCv3 */
					type = CT_MMC;
				else
					type = 0;
			}
		}
		else // some kind of unknown error in initialisation.
			type = 0;

		if (type != 0 && type != CT_BLOCK) // If it is NOT a Block Address SD Version 2 device.
			if ( send_cmd(CMD16, 512) != R1_READY_STATE)	/* Try to set R/W block length to 512 */
				type = 0;
	}

	CardType = type;

	spiSetClockDivider(SD_SPI_DIVIDER);	// Reset maximum speed clock, for maximum performance.
	spiTransfer(0xFF);					/* Give SD Card 8 Clocks to complete command. */
	spiDeselect(SDCard);				// Deselect the SD card (before power_off(), if card not initialised).

	if (type) {					/* Initialisation succeeded */
		Stat &= ~STA_NOINIT;	/* Clear STA_NOINIT */
	} else {					/* Initialisation failed */
		power_off();
	}

	return Stat;
}



/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE drv		/* Physical drive number (0) */
)
{
	if (drv != 0)
		Stat |= STA_NOINIT;		/* Supports only single drive */

#if defined(_GOLDILOCKS_)
	SD_CARD_DETECT_DIR &= ~SD_CARD_DETECT_BIT;		// set CD for input
	SD_CARD_DETECT_PORT |= SD_CARD_DETECT_BIT;		// set CD pull up
	__asm__ __volatile__("nop");					// no operation for synchronisation
	if(SD_CARD_DETECT_PIN & SD_CARD_DETECT_BIT)		// check whether the CD line is held high (SD card absent)
		Stat |= STA_NODISK;
#endif

	return Stat;
}


/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE drv,			/* Physical drive number (0) */
	BYTE *buff,			/* Pointer to the data buffer to store read data */
	DWORD sector,		/* Start sector number (LBA) */
	UINT count			/* Sector count (1..255) */
)
{
	uint8_t rattempt = READ_ATTEMPTS;		/* Read attempts */
	uint8_t resp = 0;

	if (drv || !count) return RES_PARERR; // only single drive supported, and sectors can't be null.
	if (Stat & STA_NOINIT) return RES_NOTRDY;

	if (!(CardType & CT_BLOCK)) sector *= 512;	/* Convert to byte address if needed */

	do {
		uint8_t *lbuff = buff;				/* Local Pointer to the data to be written */
		uint32_t lsector = sector;			/* Start local sector number (LBA) */
		uint8_t lcount = count;				/* Local sector count (1..255) */

		if (!spiSelect(SDCard)) return RES_NOTRDY;
		spiSetDataMode(SPI_MODE0);			// Enable SPI function in mode 0
		spiSetClockDivider(SD_SPI_DIVIDER);	// SPI at maximum speed


		if (lcount == 1)					/* Single block read */
		{

			if (( send_cmd(CMD17, lsector) == R1_READY_STATE)
				&& (rcvr_datablock(lbuff, 512)))/* READ_SINGLE_BLOCK */
					resp = DATA_RES_ACCEPTED;

		} else {							/* Multiple block read */

			if (CardType & CT_MMC)
				send_cmd(CMD23, lcount);	// prepare with the size of the data to be read for MMC cards.

			if ( send_cmd(CMD18, lsector) == R1_READY_STATE)
			{	/* READ_MULTIPLE_BLOCK */
				do {
					if (rcvr_datablock(lbuff, 512)) resp = DATA_RES_ACCEPTED;
					lbuff += 512;
				} while (--lcount && resp == DATA_RES_ACCEPTED);

				if (CardType & CT_SDC)
					send_cmd(CMD12, 0x00);				/* STOP_TRANSMISSION for SD cards*/
			}
			else										/* An error with CMD18, so we do multiple single block reads */
			{											/* Read multiple READ_SINGLE_BLOCK */
				if (CardType & CT_SDC)
					send_cmd(CMD12, 0x00);				/* STOP_TRANSMISSION for SD cards, to reset*/

				do {
					if ( send_cmd(CMD17, lsector++) == R1_READY_STATE)
					{									/* READ_SINGLE_BLOCK */
						if (rcvr_datablock(lbuff, 512)) resp = DATA_RES_ACCEPTED;
						lbuff += 512;
					}
					if (!(CardType & CT_BLOCK))	lsector += 511;	/* increment to sector byte address if needed */
				} while (--lcount && resp == DATA_RES_ACCEPTED);
			}
		}

		spiTransfer(0xFF);							/* Give SD Card 8 Clocks to complete command. */
		spiDeselect(SDCard);

	} while ((--rattempt != 0) && (resp != DATA_RES_ACCEPTED) );

	return resp == DATA_RES_ACCEPTED ? RES_OK : RES_ERROR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

DRESULT disk_write (
	BYTE drv,			/* Physical drive number (0) */
	BYTE const *buff,	/* Pointer to the data to be written */
	DWORD sector,		/* Start sector number (LBA) */
	UINT count			/* Sector count (1..255) */

)
{
	uint8_t wattempt = WRITE_ATTEMPTS;		/* Write attempts */
	uint8_t multiWrite = 1;					/* flag to check for CMD25 WRITE_MULTIPLE */
	uint8_t resp = 0;

	if (drv || !count) return RES_PARERR;
	if (Stat & STA_NOINIT) return RES_NOTRDY;
	if (Stat & STA_PROTECT) return RES_WRPRT;

	if (!(CardType & CT_BLOCK)) sector *= 512;	/* Convert to byte address if needed */

	do {
		uint8_t const *lbuff = buff;		/* Local Pointer to the data to be written */
		uint32_t lsector = sector;			/* Start local sector number (LBA) */
		uint8_t lcount = count;				/* Local sector count (1..255) */

		if (!spiSelect(SDCard)) return RES_NOTRDY;
		spiSetDataMode(SPI_MODE0);			// Enable SPI function in mode 0
		spiSetClockDivider(SD_SPI_DIVIDER);	// SPI at maximum speed

		if (lcount == 1)
		{	/* Single block write */

			if (( send_cmd(CMD24, lsector) == R1_READY_STATE))		/* WRITE_BLOCK */
				resp = (xmit_datablock(lbuff, DATA_START_BLOCK) & DATA_RES_MASK);

		} else { /* Multiple block write */

			if ( multiWrite == 1)									/* WRITE_MULTIPLE_BLOCK */
			{
				if (CardType & CT_MMC)
					send_cmd(CMD23, lcount);						// prepare with the size of the data to be written for MMC.

				if (CardType & CT_SDC)
					send_cmd(ACMD23, lcount);						// prepare with the size of the data to be erased for SDC.

				if( send_cmd(CMD25, lsector) == R1_READY_STATE )	/* Send WRITE_MULTIPLE_BLOCK */
				{
					do {
						if ( (resp = (xmit_datablock(lbuff, WRITE_MULTIPLE_TOKEN) & DATA_RES_MASK)) != DATA_RES_ACCEPTED )
						{
							send_cmd(CMD12, 0x00);					/* STOP_TRANSMISSION there's an error */
							if (resp == DATA_RES_WRITE_ERROR)		/* SD card is not capable of CMD25, multiple sector write */
								multiWrite = 0;
							break;
						}
						lbuff += 512;
					} while (--lcount);

					if ( (CardType & CT_SDC) && resp == DATA_RES_ACCEPTED)
						xmit_datablock(NULL, STOP_TRAN_TOKEN);		/* Send STOP_TRAN token for SD cards. */
				}

			} else { 												/* Multiple Single block writes */

				do {/* Send WRITE_BLOCK */
					if ( send_cmd(CMD24, lsector) != R1_READY_STATE ) break;

					if ( (resp = (xmit_datablock(lbuff, DATA_START_BLOCK) & DATA_RES_MASK)) != DATA_RES_ACCEPTED ) break;
					spiTransfer(0xFF);								/* Give SD Card 8 Clocks to complete command. */

					if (!(CardType & CT_BLOCK)) lsector += 512;		/* increment to sector byte address if needed */
					else ++lsector;									/* otherwise increment to sector address */
					lbuff += 512;

				} while (--lcount);
			}
		}

		if (resp == DATA_RES_ACCEPTED)
			count = 0;

		spiTransfer(0xFF);											/* Give SD Card 8 Clocks to complete command. */
		spiDeselect(SDCard);

	} while ((--wattempt != 0) && (resp != DATA_RES_ACCEPTED) );



	return count ? RES_ERROR : RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE drv,		/* Physical drive number (0) */
	BYTE ctrl,	    /* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT resp;
	uint8_t n, csd[16], *ptr = buff; // 8 bit char for normal buffer
	uint32_t csize;
	uint32_t *erasePtr = buff; // 32 bit integers for the erase sector (or byte) addresses


	if (drv) return RES_PARERR;

	resp = RES_ERROR;

	if (ctrl == CTRL_POWER)
	{
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
	} else {
		if (Stat & STA_NOINIT) return RES_NOTRDY;

		if (!spiSelect(SDCard)) return RES_NOTRDY;

		spiSetDataMode(SPI_MODE0);			// Enable SPI function in mode 0
		spiSetClockDivider(SD_SPI_DIVIDER);	// SPI at maximum speed

		switch (ctrl)
		{
			case CTRL_SYNC :		/* Make sure that no pending write process. Do not remove this or written sector might be left not updated. */
				{
					uint16_t i = 0;											// It takes longer than 256 cycles for large SD cards, therefore 16 bit decrement
					while ( (--i != 0) && (spiTransfer(0xFF) != 0xFF) );	// While SD busy (0x00 signal or other), waiting for 0xFF.
					if( i )
						resp = RES_OK;
				}
				break;

			case GET_SECTOR_COUNT :	/* Get number of sectors on the disk (uint32_t) */

				if ((send_cmd(CMD9, 0) == R1_READY_STATE) && rcvr_datablock(csd, 16)) {
					if ((csd[0] >> 6) == 1) {	/* SDC ver 2.00 */
						csize = csd[9] + ((uint16_t)csd[8] << 8) + ((uint32_t)(csd[7] & 63) << 16) + 1;
						*(uint32_t*)buff = csize << 10;
					} else {					/* SDC ver 1.XX or MMC */
						n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
						csize = (csd[8] >> 6) + ((uint16_t)csd[7] << 2) + ((uint16_t)(csd[6] & 3) << 10) + 1;
						*(uint32_t*)buff = csize << (n - 9);
					}
					resp = RES_OK;
				}
				break;

			case GET_SECTOR_SIZE :	/* Get R/W sector size (uint16_t) */
				*(uint16_t*)buff = 512;
				resp = RES_OK;
				break;

			case GET_BLOCK_SIZE :	/* Get erase block size in unit of sector (uint32_t) */
				if (CardType & CT_SD2) {	/* SDv2? */
					if ((send_cmd(ACMD13, 0) == R1_READY_STATE) && rcvr_datablock(csd, 16))
					{				/* Read partial block */
						for (n = 64 - 16; n; --n) spiTransfer(0xFF);	/* Purge trailing data */
						*(uint32_t*)buff = 16UL << (csd[10] >> 4);
						resp = RES_OK;
					}
				} else {					/* SDv1 or MMCv3 */
					if ((send_cmd(CMD9, 0) == R1_READY_STATE) && rcvr_datablock(csd, 16))
					{	/* Read CSD */
						if (CardType & CT_SD1) {	/* SDv1 */
							*(uint32_t*)buff = (((csd[10] & 63) << 1) + ((uint16_t)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
						} else {					/* MMCv3 */
							*(uint32_t*)buff = ((uint16_t)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
						}
						resp = RES_OK;
					}
				}
				break;

			case CTRL_TRIM : // Trim (erase) sectors
				if (!(CardType & CT_SDC)) break;				/* Check if the card is SDC */

				if ((send_cmd(CMD9, 0) == R1_READY_STATE) && rcvr_datablock(csd, 16))	/* READ_CSD */
					resp = RES_OK;
				else
					break;

				if (!(csd[0] >> 6) && !(csd[10] & 0x40)) break;	/* Check if sector erase can be applied to the card */

				// Check to see if we have a BLOCK card; if not we calculate byte address.
				if (!(CardType & CT_BLOCK))
				{
					erasePtr[0] *= 512;
					erasePtr[1] *= 512;
				}
				// Set the start and end sectors (or bytes) for erasing.
				if ( (send_cmd(CMD32, erasePtr[0]) == R1_READY_STATE) && (send_cmd(CMD33, erasePtr[1]) == R1_READY_STATE)
					&& (send_cmd(CMD38, 0) == R1_READY_STATE)) // Erase the nominated sectors. Response is R1b = R1 + 0x00 bytes while busy.
						resp = RES_OK;
				break;

			case MMC_GET_TYPE :		/* Get card type flags (1 byte) */
				*ptr = CardType;
				resp = RES_OK;
				break;

			case MMC_GET_CSD :		/* Receive CSD as a data block (16 bytes) */
				if ((send_cmd(CMD9, 0) == R1_READY_STATE) && rcvr_datablock(ptr, 16))	/* READ_CSD */
					resp = RES_OK;
				break;

			case MMC_GET_CID :		/* Receive CID as a data block (16 bytes) */
				if ((send_cmd(CMD10, 0) == R1_READY_STATE) && rcvr_datablock(ptr, 16))	/* READ_CID */
					resp = RES_OK;
				break;

			case MMC_GET_OCR :		/* Receive OCR as an R3 response (4 bytes) */
				if (send_cmd(CMD58, 0) == R1_READY_STATE) {	/* READ_OCR */
					for (n = 4; n; --n) *ptr++ = spiTransfer(0xFF);
					resp = RES_OK;
				}
				break;

			case MMC_GET_SDSTAT :	/* Receive SD status as a data block (64 bytes) */
				if ((send_cmd(ACMD13, 0) == R1_READY_STATE) && rcvr_datablock(ptr, 64))	/* SD_STATUS */
					resp = RES_OK;
				break;

			default:
				resp = RES_PARERR;
				break;
		}
		spiTransfer(0xFF);			/* Give SD Card 8 Clocks to complete command. */
		spiDeselect(SDCard);		// deselect the SD card
	}
	return resp;
}

#endif

#endif
