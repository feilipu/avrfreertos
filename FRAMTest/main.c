////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////    main.c
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include <util/delay.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* External RAM include file. */
#include "ext_ram.h"

/* i2c Interface include file. */
#include "i2cMultiMaster.h"

/* serial interface include file. */
#include "serial.h"

/* extended string to integer */
#include "xatoi.h"

/* SPI interface include file. */
#include "spi.h"

/* RTC interface (using I2C) include file. */
#include "rtc.h"
#include "time.h"

/* FatF interface include file. ffconf.h holds configuration options, and is auto included. */
#include "ff.h"

#if defined (portHD44780_LCD)
/* HD44780 interface include file. */
#include "hd44780.h"
#endif

/* FRAM interface include file. */

#include "../lib_eefs/eefs_filesys.h"


/*-----------------------------------------------------------*/
// DEFINES

#if defined(portEXT_RAM) && !defined(portEXT_RAMFS)
#define CMD_BUFFER_SIZE 8192	// size of working buffer (on heap) with extended RAM EtherMega
#else
#define CMD_BUFFER_SIZE 1024	// size of working buffer (on heap) for standard EtherMega & Goldilocks
#endif

#define LINE_SIZE 128			// size of command line (on heap)

/* Working buffer */
static uint8_t * Buff = NULL;	/* Put working buffer on heap later (with pvPortMalloc). */

/* Console input buffer */
uint8_t * LineBuffer = NULL;	// put line buffer on heap (with pvPortMalloc).

/* Create a handle for the serial port. */
extern xComPortHandle xSerialPort;

/*-----------------------------------------------------------*/
/* Create files for the SD card and FatFS. */
static uint32_t AccSize;				/* Work register for fs command */
static uint16_t AccFiles, AccDirs;
static FILINFO Finfo;

static FATFS Fatfs[_VOLUMES];		/* File system object for each logical drive */
static FIL File[_FS_LOCK];			/* File object. there are _FS_LOCK file objects available,  >= 2. */


/*-----------------------------------------------------------*/
// Task Definitions. Only two tasks here currently.

static void TaskBlinkRedLED(void *pvParameters);	// Main Arduino Mega 2560, Freetronics EtherMega (Red) LED Blink
static void TaskSDMonitor(void *pvParameters);		// SD task monitor for EtherMega

/* Private helper functions */

static void put_dump (const uint8_t *buff, uint32_t ofs, uint8_t cnt);
static void get_line (uint8_t *buff, uint8_t len);
static FRESULT scan_files (	uint8_t* path );
static void put_rc (FRESULT rc);

static uint8_t test_diskio (
    uint8_t pdrv,      /* Physical drive number to be checked (all data on the drive will be lost) */
    uint16_t ncyc,      /* Number of test cycles */
    uint8_t* pbuff,    /* Pointer to the working buffer */
    uint16_t sz_buff    /* Size of the working buffer in unit of byte */
);

/*-----------------------------------------------------------*/
int main(void) __attribute__((OS_main));

/* Main program loop */
int main(void)
{

    // turn on the serial port for debugging or for other USART reasons.
	xSerialPort = xSerialPortInitMinimal( USART0, 38400, portSERIAL_BUFFER_TX, portSERIAL_BUFFER_RX ); //  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)

	avrSerialPrint_P(PSTR("\r\nHello World!\r\n")); // Ok, so we're alive... (using polling serial access, pre-scheduler)

#if defined (portHD44780_LCD)
	lcd_Init();
	lcd_Print_P(PSTR("Hello!"));
	lcd_Locate (1, 0);
#endif


    xTaskCreate(
		TaskBlinkRedLED
		,  (const char *)"RedLED" // Arduino Mega 2560, Freetronics EtherMega (Red) LED Blink
		,  256				// Tested ok at 188 for LED blink only.
		,  NULL
		,  3
		,  NULL ); // */

    xTaskCreate(
		TaskSDMonitor
		,  (const char *)"SDMonitor" // Arduino Mega 2560, Freetronics EtherMega SD Monitor
		,  1768				// Tested x free
		,  NULL
		,  2
		,  NULL ); // */

//	avrSerialPrintf_P(PSTR("\r\nFree Heap Size: %u\r\n"),xPortGetFreeHeapSize() ); // needs heap_1 or heap_2 for this function to succeed.

    vTaskStartScheduler();

	avrSerialPrint_P(PSTR("\r\nGoodbye... no space for idle task!\r\n")); // Doh, so we're dead...

#if defined (portHD44780_LCD)
	lcd_Locate (1, 0);
	lcd_Print_P(PSTR("DEAD BEEF!"));
#endif

}

/*-----------------------------------------------------------*/


static void TaskSDMonitor(void *pvParameters) // Monitor for SD Card
{
    (void) pvParameters;

    TickType_t Timer;

    tm CurrTimeDate; 			// set up an array for the RTC info.

	uint8_t *ptr, *ptr2;
	int32_t p1, p2, p3;
	uint8_t res, b1, *bp;
	uint16_t s1, s2, cnt;
	uint32_t ofs = 0, sect = 0;
	FATFS *fs;
	DIR dir;

	uint_farptr_t				FarAddress;					// only used to copy FatFS file to FRAM file system.
    EEFS_DirectoryDescriptor_t *DirDescriptor		= NULL; // will point to EEFS_DirectoryDescriptor in eefs_fileapi.c
    EEFS_DirectoryEntry_t	   *DirEntry			= NULL; // will point to EEFS_DirectoryEntry in eefs_fileapi.c
	EEFS_Stat_t                 StatBuffer;

	int8_t						ReturnCode;					// usually error returns.
	int8_t                     	FileDescriptor = NULL;		// identifier for the file opened in the EEFS file system.
	int32_t						total_used_space;
	int32_t						total_free_space;

	// create the working buffers on the heap (so they can be moved later).

	if(LineBuffer == NULL) // if there is no LineBuffer buffer allocated (pointer is NULL), then allocate buffer.
		if( !(LineBuffer = (uint8_t *) pvPortMalloc( sizeof(uint8_t) * LINE_SIZE )))
			xSerialPrint_P(PSTR("pvPortMalloc for *LineBuffer fail..!\r\n"));


#if defined (portRTC_DEFINED)

	// initialise I2C master interface, need to do this once only.
	// If there are two I2C processes, then do it during the system initiation.
	I2C_Master_Initialise((ARDUINO<<I2C_ADR_BITS) | (pdTRUE<<I2C_GEN_BIT));

#endif

	xSerialPrint_P(PSTR("\r\nFatFs module test monitor for AVR"));
	xSerialPrint_P(_USE_LFN ? PSTR("\r\nLFN Enabled") : PSTR("\r\nLFN Disabled"));
	xSerialPrintf_P(PSTR(", Code page: %u\r\n"), _CODE_PAGE);

    for(;;)
    {
    	time((time_t *)&p1);
    	xSerialPrintf_P(PSTR("\r\n%s >"), ctime( (time_t *)&p1));

		ptr = LineBuffer;
		get_line(ptr, (sizeof(uint8_t)* LINE_SIZE)); //sizeof (LineBuffer);

		switch (*ptr++) {

		case 'd' :
			switch (*ptr++) {
			case 'd' :	/* dd <phy_drv#> [<sector>] - Dump sector */
				if (!xatoi(&ptr, &p1)) break;
				if (!xatoi(&ptr, &p2)) p2 = sect;
				sect = p2;

				res = disk_read((uint8_t)p1, (uint8_t *)Buff, p2, 1);
				if (res) {
					xSerialPrintf_P(PSTR("D:%2d\r\n"), res);
					break;
				} else ++sect;

				xSerialPrintf_P(PSTR("Sector:%lu\r\n"), p2);
				for (bp=Buff, ofs = 0; ofs < 0x200; bp+=16, ofs+=16)
					put_dump(bp, ofs, 16);
				break;

			case 'i' :	/* di [<phy_drv#>] - Initialise disk */
				if (!xatoi(&ptr, &p1)) p1 = 0;

				if(Buff == NULL) // if there is no Buff buffer allocated (pointer is NULL), then allocate buffer.
					if( !(Buff = (uint8_t *) pvPortMalloc( sizeof(uint8_t) * CMD_BUFFER_SIZE )))
					{
						xSerialPrint_P(PSTR("pvPortMalloc for *Buff fail..!\r\n"));
						break;
					}

				xSerialPrintf_P(PSTR("D:%2d\r\n"), disk_initialize((uint8_t)p1));
				break;

			case 's' :	/* ds [<phy_drv#>] - Show disk status */
				if (!xatoi(&ptr, &p1)) p1 = 0;
				if (disk_ioctl((uint8_t)p1, GET_SECTOR_COUNT, &p2) == RES_OK)
					{ xSerialPrintf_P(PSTR("Drive size: %lu sectors\r\n"), p2); }
				if (disk_ioctl((uint8_t)p1, GET_BLOCK_SIZE, &p2) == RES_OK)
					{ xSerialPrintf_P(PSTR("Erase block: %lu sectors\r\n"), p2); }
				if (disk_ioctl((uint8_t)p1, MMC_GET_TYPE, &b1) == RES_OK)
					{ xSerialPrintf_P(PSTR("Card type: %u\r\n"), b1); }
				if (disk_ioctl((uint8_t)p1, MMC_GET_CSD, Buff) == RES_OK)
					{ xSerialPrint_P(PSTR("CSD:\r\n")); put_dump(Buff, 0, 16); }
				if (disk_ioctl((uint8_t)p1, MMC_GET_CID, Buff) == RES_OK)
					{ xSerialPrint_P(PSTR("CID:\r\n")); put_dump(Buff, 0, 16); }
				if (disk_ioctl((uint8_t)p1, MMC_GET_OCR, Buff) == RES_OK)
					{ xSerialPrint_P(PSTR("OCR:\r\n")); put_dump(Buff, 0, 4); }
				if (disk_ioctl((uint8_t)p1, MMC_GET_SDSTAT, Buff) == RES_OK) {
					xSerialPrint_P(PSTR("SD Status:\r\n"));
					for (s1 = 0; s1 < 64; s1 += 16) put_dump(Buff+s1, s1, 16);
						break;
				}
				break;

			case 'x' : /* dx <iterations> - Destructive Testing to prove the DISKIO functions. */
				if (!xatoi(&ptr, &p1)) break;
			    /* Check function/compatibility of the physical drive #0 */
			    res = test_diskio(0, p1, (uint32_t *)Buff, CMD_BUFFER_SIZE);
			    if (res) {
			    	xSerialPrintf_P(PSTR("Sorry the function/compatibility test failed.\r\nFatFs will not work on this disk driver.\r\nErrors: %u\n"), res);
			    } else {
			    	xSerialPrintf_P(PSTR("Congratulations! The disk I/O layer works well.\r\n"));
			    }
				break;

			default :
				break;
			}
			break;

		case 'b' :
			switch (*ptr++) {
			case 'd' :	/* bd <addr> - Dump R/W buffer */
				if (!xatoi(&ptr, &p1)) break;
				for (bp=&Buff[p1], ofs = p1, cnt = 32; cnt; cnt--, bp+=16, ofs+=16)
					put_dump(bp, ofs, 16);
				break;

			case 'e' :	/* be <addr> [<data>] ... - Edit R/W buffer */
				if (!xatoi(&ptr, &p1)) break;
				if (xatoi(&ptr, &p2)) {
					do {
						Buff[p1++] = (uint8_t)p2;
					} while (xatoi(&ptr, &p2));
					break;
				}
				for (;;) {
					xSerialPrintf_P(PSTR("%04X %02X-"), (uint16_t)p1, Buff[p1]);
					get_line(LineBuffer, (sizeof(uint8_t)* LINE_SIZE) );
					ptr = LineBuffer;
					if (*ptr == '.') break;
					if (*ptr < ' ') { p1++; continue; }
					if (xatoi(&ptr, &p2))
						Buff[p1++] = (uint8_t)p2;
					else
						xSerialPrint_P(PSTR("???\r\n"));
				}
				break;

			case 'r' :	/* br <phy_drv#> <sector> [<n>] - Read disk into R/W buffer */
				if (!xatoi(&ptr, &p1)) break;
				if (!xatoi(&ptr, &p2)) break;
				if (!xatoi(&ptr, &p3)) p3 = 1;
				xSerialPrintf_P(PSTR("B:%2u\r\n"), disk_read((uint8_t)p1, (uint8_t *)Buff, p2, p3));
				break;

			case 'w' :	/* bw <phy_drv#> <sector> [<n>] - Write R/W buffer into disk */
				if (!xatoi(&ptr, &p1)) break;
				if (!xatoi(&ptr, &p2)) break;
				if (!xatoi(&ptr, &p3)) p3 = 1;
				xSerialPrintf_P(PSTR("B:%2u\r\n"), disk_write((uint8_t)p1, (uint8_t *)Buff, p2, p3));
				break;

			case 'f' :	/* bf <n> - Fill working buffer */
				if (!xatoi(&ptr, &p1)) break;
				memset(Buff, (uint8_t)p1, (sizeof(uint8_t)* CMD_BUFFER_SIZE));
				break;

			default :
				break;
			}
			break;


		case 'f' :
			switch (*ptr++) {

			case 'i' :	/* fi <log drv#> - Initialise logical drive */
				if (!xatoi(&ptr, &p1)) break;

				if(Buff == NULL) // if there is no Buff buffer allocated (pointer is NULL), then allocate buffer.
					if( !(Buff = (uint8_t *) pvPortMalloc( sizeof(uint8_t) * CMD_BUFFER_SIZE )))
					{
						xSerialPrint_P(PSTR("pvPortMalloc for *Buff fail..!\r\n"));
						break;
					}

#if _USE_LFN
				if(Finfo.lfname == NULL) // if there is no Long File Name buffer allocated (pointer is NULL), then allocate buffer.
				{
					if( !(Finfo.lfname = (TCHAR *) pvPortMalloc( sizeof(TCHAR) * (_MAX_LFN + 1) )))
					{
						xSerialPrint_P(PSTR("pvPortMalloc for Finfo.lfname fail..!\r\n"));
						break;
					}
					Finfo.lfsize = _MAX_LFN + 1;
				}
#endif

				put_rc(f_mount(&Fatfs[p1], (const TCHAR*)&p1, 1));

				break;

#if _FS_MINIMIZE < 1
			case 's' :	/* fs [<path>] - Show logical drive status */
				while (*ptr == ' ') ptr++;
				res = f_getfree(ptr, (uint32_t*)&p2, &fs);
				if (res) { put_rc(res); break; }
				xSerialPrintf_P(PSTR("FAT type = %u\r\nBytes/Cluster = %lu\r\nNumber of FATs = %u\r\n"
							 "Root DIR entries = %u\r\nSectors/FAT = %lu\r\nNumber of clusters = %lu\r\n"),
						fs->fs_type, (uint32_t)fs->csize * 512, fs->n_fats,
						fs->n_rootdir, fs->fsize, fs->n_fatent - 2	);
				vTaskDelay( 32 / portTICK_PERIOD_MS ); // Whoa... too fast.
				xSerialPrintf_P(PSTR("FAT start (lba) = %lu\r\nDIR start (lba,cluster) = %lu\r\nData start (lba) = %lu\r\n...\r\n"),
										fs->fatbase, fs->dirbase, fs->database	);
				vTaskDelay( 32 / portTICK_PERIOD_MS ); // Whoa... too fast.
				AccSize = AccFiles = AccDirs = 0;
				res = scan_files(ptr);
				if (res) { put_rc(res); break; }
				xSerialPrintf_P(PSTR("\r%u files, %lu bytes.\r\n%u folders.\r\n"
							 "%lu KB total disk space.\r\n%lu KB available.\r\n"),
						AccFiles, AccSize, AccDirs,
						(fs->n_fatent - 2) * (fs->csize / 2), p2 * (fs->csize / 2)
				);
				break;
#endif

#if _FS_MINIMIZE < 2
			case 'l' :	/* fl [<path>] - Directory listing */
				while (*ptr == ' ') ptr++;
				res = f_opendir(&dir, ptr);
				if (res) { put_rc(res); break; }
				p1 = s1 = s2 = 0;
				for(;;) {
					res = f_readdir(&dir, &Finfo);
					if ((res != FR_OK) || !Finfo.fname[0]) break;
					if (Finfo.fattrib & AM_DIR) {
						s2++;
					} else {
						s1++; p1 += Finfo.fsize;
					}
					xSerialPrintf_P(PSTR("%c%c%c%c%c %u/%02u/%02u %02u:%02u %9lu  %s"),
								(Finfo.fattrib & AM_DIR) ? 'D' : '-',
								(Finfo.fattrib & AM_RDO) ? 'R' : '-',
								(Finfo.fattrib & AM_HID) ? 'H' : '-',
								(Finfo.fattrib & AM_SYS) ? 'S' : '-',
								(Finfo.fattrib & AM_ARC) ? 'A' : '-',
								(Finfo.fdate >> 9) + 1980, (Finfo.fdate >> 5) & 15, Finfo.fdate & 31,
								(Finfo.ftime >> 11), (Finfo.ftime >> 5) & 63,
								Finfo.fsize, &(Finfo.fname[0]));
#if _USE_LFN
					for (p2 = strlen((char *)Finfo.fname); p2 < 14; p2++)
						xSerialPutChar( &xSerialPort, ' ' );
					xSerialPrintf_P(PSTR("%s\r\n"), Finfo.lfname);
#else
					xSerialPrint_P(PSTR("\r\n"));
#endif

					vTaskDelay( 32 / portTICK_PERIOD_MS ); // Whoa... too fast.
				}
				xSerialPrintf_P(PSTR("%4u File(s),%10lu bytes total\r\n%4u Dir(s)"), s1, p1, s2);
#if _FS_MINIMIZE < 1
				if (f_getfree(ptr, (uint32_t*)&p1, &fs) == FR_OK)
					xSerialPrintf_P(PSTR(", %10luK bytes free\r\n"), p1 * fs->csize / 2);
#else
				xSerialPrint_P(PSTR("\r\n"));
#endif
				break;
#endif

			case 'o' :	/* fo <mode> <name> - Open a file */
				if (!xatoi(&ptr, &p1)) break;
				while (*ptr == ' ') ptr++;
				put_rc(f_open(&File[0], ptr, (uint8_t)p1));
				break;

			case 'y' :	/* fy - Sync a file */
				put_rc(f_sync(&File[0]));
				break;

			case 'c' :	/* fc - Close a file */
				put_rc(f_close(&File[0]));
				break;

#if _FS_MINIMIZE < 3
			case 'e' :	/* fe <ptr> - Seek (set) file pointer */
				if (!xatoi(&ptr, &p1)) break;
				res = f_lseek(&File[0], p1);
				put_rc(res);
				if (res == FR_OK)
					xSerialPrintf_P(PSTR("fptr = %lu(0x%lX)\r\n"), File[0].fptr, File[0].fptr);
				break;
#endif

			case 'r' :	/* fr <len> - read file */
				if (!xatoi(&ptr, &p1)) break;
				p2 = 0;
				Timer = xTaskGetTickCount();
				while (p1) {
					if (p1 >= (sizeof(uint8_t)* CMD_BUFFER_SIZE)) 	{ cnt = (sizeof(uint8_t)* CMD_BUFFER_SIZE); p1 -= (sizeof(uint8_t)* CMD_BUFFER_SIZE); }
					else 			{ cnt = (uint16_t)p1; p1 = 0; }
					res = f_read(&File[0], Buff, cnt, &s2);
					if (res != FR_OK) { put_rc(res); break; }
					p2 += s2;
					if (cnt != s2) break;
				}
				s2 = xTaskGetTickCount() - Timer; //  portTICK_RATE_MS
				xSerialPrintf_P(PSTR("%lu Bytes read at %lu Bytes/sec.\r\n"), p2, s2 ? (p2 * 1000 / s2 / portTICK_PERIOD_MS) : 0 );
				break;

			case 'd' :	/* fd <len> - read and dump file from current fp */
				if (!xatoi(&ptr, &p1)) break;
				ofs = File[0].fptr;
				while (p1) {
					if (p1 >= 16)	{ cnt = 16; p1 -= 16; }
					else 			{ cnt = (uint16_t)p1; p1 = 0; }
					res = f_read(&File[0], Buff, cnt, &cnt);
					if (res != FR_OK) { put_rc(res); break; }
					if (!cnt) break;
					put_dump(Buff, ofs, cnt);
					ofs += 16;
				}
				break;

			case 'w' :	/* fw <len> <val> - write file */
				if (!xatoi(&ptr, &p1) || !xatoi(&ptr, &p2)) break;
				memset(Buff, (uint8_t)p2, (sizeof(uint8_t)* CMD_BUFFER_SIZE));
				p2 = 0;
				Timer = xTaskGetTickCount();
				while (p1) {
					if (p1 >= (sizeof(uint8_t)* CMD_BUFFER_SIZE))	{ cnt = (sizeof(uint8_t)* CMD_BUFFER_SIZE); p1 -= (sizeof(uint8_t)* CMD_BUFFER_SIZE); }
					else 			{ cnt = (uint16_t)p1; p1 = 0; }
					res = f_write(&File[0], Buff, cnt, &s2);
					if (res != FR_OK) { put_rc(res); break; }
					p2 += s2;
					if (cnt != s2) break;
				}
				s2 = xTaskGetTickCount() - Timer;
				xSerialPrintf_P(PSTR("%lu Bytes written at %lu Bytes/Sec.\r\n"), p2, s2 ? (p2 * 1000 / s2 / portTICK_PERIOD_MS) : 0 );
				break;

#if _FS_MINIMIZE < 1
			case 'v' :	/* fv - Truncate file at current file pointer */
				put_rc(f_truncate(&File[0]));
				break;

			case 'n' :	/* fn <old_name> <new_name> - Change file/dir name */
				while (*ptr == ' ') ptr++;
				ptr2 = (uint8_t *) strchr((char *)ptr, ' ');
				if (!ptr2) break;
				*ptr2++ = 0;
				while (*ptr2 == ' ') ptr2++;
				put_rc(f_rename(ptr, ptr2));
				break;

			case 'u' :	/* fu <name> - Unlink a file or dir */
				while (*ptr == ' ') ptr++;
				put_rc(f_unlink(ptr));
				break;

			case 'k' :	/* fk <name> - Create a directory */
				while (*ptr == ' ') ptr++;
				put_rc(f_mkdir(ptr));
				break;

			case 'a' :	/* fa <atrr> <mask> <name> - Change file/dir attribute */
				if (!xatoi(&ptr, &p1) || !xatoi(&ptr, &p2)) break;
				while (*ptr == ' ') ptr++;
				put_rc(f_chmod(ptr, p1, p2));
				break;


			case 'f' : /* ff <base_address> <src_name>  - Copy FatFS file to EEPROM / SRAM */
				if (!xatoi(&ptr, &p1)) break;
				while (*ptr == ' ') ptr++;

				xSerialPrintf_P(PSTR("Opening \"%s\""), ptr);
				res = f_open(&File[0], ptr, FA_OPEN_EXISTING | FA_READ);
				if (res) {
					put_rc(res);
					break;
				}
				xSerialPrintf_P(PSTR("\r\nPreparing EEPROM at 0x%08lx"), (uint_farptr_t)p1);

				ReturnCode = eefs_avrspi_begin();
				if (ReturnCode)	break; // problem with opening the EEPROM / SRAM

				xSerialPrintf_P(PSTR("\r\nWriting EEPROM..."));
				Timer = xTaskGetTickCount();
				FarAddress = (uint_farptr_t)p1;
				EEFS_LIB_LOCK;

				for (;;) {
					res = f_read(&File[0], Buff, (sizeof(uint8_t) * CMD_BUFFER_SIZE), &s1);
					if (res || s1 == 0) break;   /* error or eof */
					ReturnCode = eefs_avrspi_write( (addr_farptr_t)FarAddress, Buff, s1 );
					FarAddress += (uint_farptr_t)(uint32_t)s1;
					if (ReturnCode) break;   /* error or disk full */
					xSerialPrintf_P(PSTR("."));
				}

				EEFS_LIB_UNLOCK;
				s2 = xTaskGetTickCount() - Timer;

				if (res) put_rc(res);
				xSerialPrintf_P(PSTR(" %lu Bytes at %lu Bytes/Sec\r\n"), FarAddress - p1, s2 ? ((FarAddress - p1) * 1000 / s2 / portTICK_PERIOD_MS) : 0 );
				f_close(&File[0]);
				break;


			case 't' :	/* ft <year> <month> <day> <hour> <min> <sec> <name> */
				if (!xatoi(&ptr, &p1) || !xatoi(&ptr, &p2) || !xatoi(&ptr, &p3)) break;
				Finfo.fdate = ((p1 - 1980) << 9) | ((p2 & 15) << 5) | (p3 & 31);
				if (!xatoi(&ptr, &p1) || !xatoi(&ptr, &p2) || !xatoi(&ptr, &p3)) break;
				Finfo.ftime = ((p1 & 31) << 11) | ((p2 & 63) << 5) | ((p3 >> 1) & 31);
				while (*ptr == ' ') ptr++;
				put_rc(f_utime(ptr, &Finfo));
				break;

#endif

#if !_FS_READONLY
			case 'x' : /* fx <src_name> <dst_name> - Copy file */
				while (*ptr == ' ') ptr++;
				ptr2 = (uint8_t *) strchr( (char *)ptr, ' ');
				if (!ptr2) break;
				*ptr2++ = 0;
				while (*ptr2 == ' ') ptr2++;
				xSerialPrintf_P(PSTR("Opening \"%s\""), ptr);
				res = f_open(&File[0], ptr, FA_OPEN_EXISTING | FA_READ);
				if (res) {
					put_rc(res);
					break;
				}
				xSerialPrintf_P(PSTR("\r\nCreating \"%s\""), ptr2);
				res = f_open(&File[1], ptr2, FA_CREATE_ALWAYS | FA_WRITE);
				if (res) {
					put_rc(res);
					f_close(&File[0]);
					break;
				}
				xSerialPrintf_P(PSTR("\r\nCopying..."));
				Timer = xTaskGetTickCount();
				p1 = 0;
				for (;;) {
					res = f_read(&File[0], Buff, (sizeof(uint8_t)* CMD_BUFFER_SIZE), &s1);
					if (res || s1 == 0) break;   /* error or eof */
					res = f_write(&File[1], Buff, s1, &s2);
					p1 += s2;
					if (res || s2 < s1) break;   /* error or disk full */
					xSerialPrintf_P(PSTR("."));
				}
				s2 = xTaskGetTickCount() - Timer;
				if (res) put_rc(res);
				xSerialPrintf_P(PSTR(" %lu Bytes at %lu Bytes/Sec\r\n"),	p1, s2 ? (p1 * 1000 / s2 / portTICK_PERIOD_MS) : 0 );
				f_close(&File[0]);
				f_close(&File[1]);
				break;
#endif

#if _FS_RPATH
			case 'g' :	/* fg <path> - Change current directory */
				while (*ptr == ' ') ptr++;
				put_rc(f_chdir(ptr));
				break;

			case 'j' :	/* fj <drive#> - Change current drive */
				if (xatoi(&ptr, &p1)) {
					put_rc(f_chdrive((uint8_t*)&p1));
				}
				break;
#if _FS_RPATH >= 2
			case 'q' :	/* fq - Show current dir path */
				res = f_getcwd(LineBuffer, (sizeof(uint8_t)* CMD_BUFFER_SIZE));
				if (res)
					put_rc(res);
				else
					xSerialPrintf_P(PSTR("%s\r\n"), LineBuffer);
				break;
#endif
#endif

#if _USE_MKFS
			case 'm' :	/* fm <logi drv#> <part type: 0:FDISK 1:SFD> <bytes/cluster> - Create file system */
				if (!xatoi(&ptr, &p1) || !xatoi(&ptr, &p2) || !xatoi(&ptr, &p3)) break;
				xSerialPrintf_P(PSTR("The drive %u will be formatted. Are you sure? (Y/n)="), (uint16_t)p1);
				get_line(ptr, (sizeof(uint8_t)* LINE_SIZE) );
				if (*ptr == 'Y') put_rc(f_mkfs((uint8_t*)&p1, (uint8_t)p2, (uint16_t)p3));
				break;
#endif

			default :
				break;

			}
			break;


			// xxx start of EEFS functions

			case 'r' :
				switch (*ptr++) {

				case 'i' :	/* ri <address> </path> - Initialise SPI RAM */

					if(Buff == NULL) // if there is no Buff buffer allocated (pointer is NULL), then allocate buffer.
						if( !(Buff = (uint8_t *) pvPortMalloc( sizeof(uint8_t) * CMD_BUFFER_SIZE )))
						{
							xSerialPrint_P(PSTR("pvPortMalloc for *Buff fail..!\r\n"));
							break;
						}
					xSerialPrintf_P(PSTR("Congratulations! The FRAM I/O Command Buffer is initialised.\r\n"));

					ReturnCode = EEFS_Init();
				    if ( ReturnCode == EEFS_SUCCESS )
				    {
				    	xSerialPrint_P(PSTR(" - OK: Initialized EEFS_Init\r\n - Global data reset.\r\n"));
				    }
				    else
				    {
				    	xSerialPrintf_P(PSTR(" - Error %i: Failed to initialize EEFS_Init\r\n"), ReturnCode);
				    }

					break;

				case 'j' :	/* rj <base_address> </device_name> </path> - Mount and open a volume */

					if (!xatoi(&ptr, &p1)) break;

					while (*ptr == ' ') ptr++;

					ptr2 = (uint8_t *) strchr( (char *)ptr, ' ');
					if (!ptr2) break;
					*ptr2++ = 0;
					while (*ptr2 == ' ') ptr2++;
					xSerialPrintf_P(PSTR("Mounting 0x%08lx \"%s\" on \"%s\"\r\n"), (uint_farptr_t)p1, ptr, ptr2);

					if(Buff == NULL) // if there is no Buff buffer allocated (pointer is NULL)
						{
							xSerialPrint_P(PSTR(" - Command Buffer not allocated..! ( > ri )\r\n"));
							break;
						}

				    /*
				    ** Mount the EEPROM disk volumes
				    */
					ReturnCode = EEFS_InitFS(ptr, p1);
				    if ( ReturnCode == EEFS_SUCCESS )
				    {
				    	xSerialPrint_P(PSTR(" - OK: Initialized EEFS_InitFS\n"));

				    	ReturnCode = EEFS_Mount(ptr, ptr2);
				    	if ( ReturnCode == EEFS_SUCCESS )
				    		xSerialPrintf_P(PSTR(" - OK: Mounted File System \"%s\"\n"), ptr2);
				    	else
				    		xSerialPrintf_P(PSTR(" - Error %i: Failed to EEFS_Mount File System\n"), ReturnCode);
				    }
				    else
				    {
				    	xSerialPrintf_P(PSTR(" - Error %i: Failed to initialize EEFS_InitFS\n"), ReturnCode);
				    }

					break;

				case 'x' :	/* rx <base_address> <number of bytes> */

					if (!xatoi(&ptr, &p1)) break;

					while (*ptr == ' ') ptr++;

					if (!xatoi(&ptr, &p2)) break;

					if (p2 >= CMD_BUFFER_SIZE) p2 = CMD_BUFFER_SIZE;

					xSerialPrintf_P(PSTR("Testing 0x%08lX for %6d bytes\r\n"), (uint_farptr_t)p1, (uint32_t)p2);

					if(Buff == NULL) // if there is no Buff buffer allocated (pointer is NULL)
						{
							xSerialPrint_P(PSTR(" - Command Buffer not allocated..! ( > ri )\r\n"));
							break;
						}


					ReturnCode = eefs_avrspi_begin();
					if (ReturnCode)	break; // problem with opening the EEPROM / SRAM

					xSerialPrintf_P(PSTR("\r\nTesting EEPROM...\r\n"));

					FarAddress = (uint_farptr_t)p1;

					EEFS_LIB_LOCK;

					ReturnCode = eefs_avrspi_write( (addr_farptr_t)FarAddress, Buff, (size_t)p2);
					if (ReturnCode) break;   /* error or disk full */

					for(uint16_t i = 0; i < p2; ++i)
					{
						uint8_t read_result;

						ReturnCode = eefs_avrspi_read( &read_result, (addr_farptr_t)(FarAddress +i), (size_t)1);
						if (ReturnCode) break;   /* error or disk full */

//						xSerialPrintf_P(PSTR("Written 0x%02x Read 0x%02x\r\n"), Buff[i], read_result);

						if( Buff[i] != read_result)
							xSerialPrintf_P(PSTR("Error at 0x%08lX with 0x%02X\r\n"), (uint_farptr_t)(FarAddress +i), read_result);
					}

					EEFS_LIB_UNLOCK;
					break;

				case 'y' :	/* ry </path> - Unmount a volume */

					while (*ptr == ' ') ptr++;
					xSerialPrintf_P(PSTR("Unmounting \"%s\"\r\n"), ptr);

				    /*
				    ** Unmount the EEPROM disk volume
				    */

					ReturnCode = EEFS_UnMount(ptr);
					if ( ReturnCode == EEFS_SUCCESS )
						xSerialPrintf_P(PSTR(" - Done\n"));
					else
						xSerialPrintf_P(PSTR(" - Error %i: Failed to EEFS_UnMount EEPROM File System\r\n"), ReturnCode);
					break;


				case 'l' :	/* rl </path> - Volume listing [ll]*/

					while (*ptr == ' ') ptr++;
					xSerialPrintf_P(PSTR("Listing \"%s\""), ptr);

					DirDescriptor = EEFS_OpenDir(ptr);
					if ( DirDescriptor != NULL )
					{
						DirEntry = NULL;
						xSerialPrint_P(PSTR("--> EEFS Directory:\n"));
						xSerialPrintf_P(PSTR("%32s      %10s\n"), "Filename", "Size");
						xSerialPrint_P(PSTR("-------------------------------------------------\n"));

						while ( (DirEntry = EEFS_ReadDir(DirDescriptor)) != NULL )
						{
							if ( DirEntry->InUse != 0 )
							{
								xSerialPrintf_P(PSTR("%32s      %10ld\n"), DirEntry->Filename, DirEntry->MaxFileSize);
							}
						}
						xSerialPrint_P(PSTR("-------------------------------------------------\n"));
						ReturnCode = EEFS_CloseDir(DirDescriptor);
					    if (ReturnCode != EEFS_SUCCESS) {
					    	xSerialPrintf_P(PSTR("Sorry directory close failure. \nError %i\n"), ReturnCode);
					    }
					}
					break;

				case 's' :	/* rs </path> - Show logical drive status */

					while (*ptr == ' ') ptr++;
					xSerialPrintf_P(PSTR("Listing \"%s\""), ptr);

					total_used_space = 0;
					total_free_space = 0;
					DirDescriptor = EEFS_OpenDir(ptr);
					if ( DirDescriptor != NULL )
					{
						DirEntry = NULL;
						xSerialPrint_P(PSTR("--> EEFS Usage Statistics:\n"));
						xSerialPrintf_P(PSTR("%32s      %10s    %10s\n"), "Filename", "Size", "Max Size");
						xSerialPrint_P(PSTR("---------------------------------------------------------------\n"));

						while ( (DirEntry = EEFS_ReadDir(DirDescriptor)) != NULL )
						{
							if ( DirEntry->InUse != 0 )
							{
								strcpy((char *)Buff, (const char *)ptr);
								strcat((char *)Buff,"/");
								strcat((char *)Buff, (const char *)DirEntry->Filename);
								ReturnCode = EEFS_Stat(Buff, &StatBuffer);

								if ( ReturnCode == EEFS_SUCCESS )
								{
									xSerialPrintf_P(PSTR("%32s      %10ld    %10ld\n"),DirEntry->Filename, StatBuffer.FileSize,
																	 DirEntry->MaxFileSize);
									total_used_space = total_used_space + StatBuffer.FileSize;
									total_free_space = total_free_space +
													  ( DirEntry->MaxFileSize - StatBuffer.FileSize);
								}
								else
								{
									xSerialPrintf_P(PSTR("Error: Cannot get Statistics buffer for file: %s\n"),Buff);
								}
							}
						}
						xSerialPrint_P(PSTR("---------------------------------------------------------------\n"));
						xSerialPrintf_P(PSTR("Total Used space = %7ld bytes\n"),total_used_space);
						xSerialPrintf_P(PSTR("Total Free space = %7ld bytes\n"),total_free_space);
						xSerialPrintf_P(PSTR("Total Space =      %7ld bytes\n"),total_used_space + total_free_space);
						ReturnCode = EEFS_CloseDir(DirDescriptor);
						if (ReturnCode != EEFS_SUCCESS)
						{
							xSerialPrintf_P(PSTR("Sorry directory close failure. \nError %i\n"), ReturnCode);
						}
				    	}
					break;


				case 'o' :	/* ro <mode> <path> - Open a FRAM file */
					if (!xatoi(&ptr, &p1)) break;
					while (*ptr == ' ') ptr++;
					xSerialPrintf_P(PSTR("Opening \"%s\" "), ptr);
					FileDescriptor = EEFS_Open(ptr, p1);
					if ( FileDescriptor == EEFS_ERROR )	{
						xSerialPrintf_P(PSTR(" - Error %i: Failed to EEFS_Open EEPROM file. \r\n"), FileDescriptor);
					} else {
						ofs = 0;
						xSerialPrintf_P(PSTR(" - Done\n"));
					}
					break;

				case 'c' :	/* rc - Close a file */
					ReturnCode = EEFS_Close( FileDescriptor );
				    if (ReturnCode != EEFS_SUCCESS) {
				    	xSerialPrintf_P(PSTR("Sorry file close failure. \nError %i\n"), ReturnCode);
				    }
				    break;

				case 'e' :	/* re <ptr> - Seek (set) FRAM file pointer */
					if (!xatoi(&ptr, &p1)) break;
					p3 = EEFS_LSeek( FileDescriptor, p1, SEEK_SET);
				    if (p3 == EEFS_ERROR) {
				    	xSerialPrintf_P(PSTR("Sorry file seek failure. \nError %i\n"), p3);
				    }
				    else
				    	ofs = p3;
					break;


				case 'r' :	/* rr <len> - read FRAM file */
					if (!xatoi(&ptr, &p1)) break;
					p2 = 0;
					Timer = xTaskGetTickCount();
					while (p1) {
						if (p1 >= (sizeof(uint8_t)* CMD_BUFFER_SIZE)) 	{ cnt = (sizeof(uint8_t)* CMD_BUFFER_SIZE); p1 -= (sizeof(uint8_t)* CMD_BUFFER_SIZE); }
						else 			{ cnt = p1; p1 = 0; }
						p3 = EEFS_Read( FileDescriptor, Buff, cnt);
						if (p3 == EEFS_ERROR ) break;
						p2 += p3;
						if (cnt != p3) break;
					}
					s2 = xTaskGetTickCount() - Timer; //  portTICK_RATE_MS
					xSerialPrintf_P(PSTR("%lu Bytes read at %lu Bytes/sec.\r\n"), p2, s2 ? (p2 * 1000 / s2 / portTICK_PERIOD_MS) : 0 );
					break;

				case 'd' :	/* rd <len> - read and dump FRAM file from current fp */
					if (!xatoi(&ptr, &p1)) break;

					while (p1) {
						if (p1 >= 16)	{ cnt = 16; p1 -= 16; }
						else 			{ cnt = (uint16_t)p1; p1 = 0; }
						p3 = EEFS_Read( FileDescriptor, Buff, cnt);
						if (p3 == EEFS_ERROR ) break;
						p2 += p3;
						if (cnt != p3) break;
						put_dump(Buff, ofs, cnt);
						ofs += 16;
					}
					break;

				case 'w' :	/* rw <len> <val> - write FRAM file */
					if (!xatoi(&ptr, &p1) || !xatoi(&ptr, &p2)) break;
					memset(Buff, (uint8_t)p2, (sizeof(uint8_t)* CMD_BUFFER_SIZE));
					p2 = 0;
					Timer = xTaskGetTickCount();
					while (p1) {
						if (p1 >= (sizeof(uint8_t)* CMD_BUFFER_SIZE))	{ cnt = (sizeof(uint8_t)* CMD_BUFFER_SIZE); p1 -= (sizeof(uint8_t)* CMD_BUFFER_SIZE); }
						else 			{ cnt = p1; p1 = 0; }
						p3 = EEFS_Write( FileDescriptor, Buff, cnt);
						if (p3 == EEFS_ERROR ) break;
						p2 += p3;
						if (cnt != p3) break;
					}
					s2 = xTaskGetTickCount() - Timer;
					xSerialPrintf_P(PSTR("%lu Bytes written at %lu Bytes/Sec.\r\n"), p2, s2 ? (p2 * 1000 / s2 / portTICK_PERIOD_MS) : 0 );
					break;

				case 'n' :	/* rn <old_path> <new_path> - Change FRAM file name */
					while (*ptr == ' ') ptr++;
					ptr2 = (uint8_t *) strchr((char *)ptr, ' ');
					if (!ptr2) break;
					*ptr2++ = 0;
					while (*ptr2 == ' ') ptr2++;
					xSerialPrintf_P(PSTR("Renaming \"%s\" to \"%s\" "), ptr, ptr2);

					ReturnCode = EEFS_Rename( ptr, ptr2 );
				    if (ReturnCode != EEFS_SUCCESS ) {
				    	xSerialPrintf_P(PSTR("Sorry file rename failure. \nError %i\n"), ReturnCode);
				    }
					break;

				case 'u' :	/* ru <path> - Unlink a FRAM file */
					while (*ptr == ' ') ptr++;
					xSerialPrintf_P(PSTR("Deleting \"%s\" "), ptr);
					ReturnCode = EEFS_Remove( ptr );
				    if (ReturnCode != EEFS_SUCCESS ) {
				    	xSerialPrintf_P(PSTR("Sorry file delete failure. \nError %i\n"), ReturnCode);
				    }
					break;

				case 'f' : /* rf <base_address> <bytes> <src_name>  - Copy EEPROM / SRAM to FatFS file */

					if (!xatoi(&ptr, &p1) || !xatoi(&ptr, &p2)) break;
					while (*ptr == ' ') ptr++;

					xSerialPrintf_P(PSTR("\r\nCreating \"%s\""), ptr);
					res = f_open(&File[0], ptr, FA_CREATE_ALWAYS | FA_WRITE);
					if (res) {
						put_rc(res);
						break;
					}

					xSerialPrintf_P(PSTR("\r\nPreparing EEPROM from 0x%08lx to 0x%08lx"), (uint_farptr_t)p1, (uint_farptr_t)(p1 + p2) );

					ReturnCode = eefs_avrspi_begin();
					if (ReturnCode)	break; // problem with opening the EEPROM / SRAM

					xSerialPrintf_P(PSTR("\r\nReading EEPROM..."));
					Timer = xTaskGetTickCount();
					FarAddress = (uint_farptr_t)p1;
					EEFS_LIB_LOCK;

					for (;;) {
						if ( (uint16_t)p2 < (sizeof(uint8_t)* CMD_BUFFER_SIZE) )
							s1 = (uint16_t)p2;
						else
							s1 = (uint16_t)(sizeof(uint8_t)* CMD_BUFFER_SIZE);
						ReturnCode = eefs_avrspi_read( Buff, (addr_farptr_t)FarAddress, s1 );
						if (ReturnCode || s1 == 0) break;   /* error or eof */

						res = f_write(&File[0], Buff, s1, &s2);
						FarAddress += (uint_farptr_t)s2;
						p2 -= s2;
						if (res || s2 < s1) break;   /* error or disk full */

						xSerialPrintf_P(PSTR("."));
					}

					EEFS_LIB_UNLOCK;
					s2 = xTaskGetTickCount() - Timer;

					if (res) put_rc(res);
					xSerialPrintf_P(PSTR(" %lu Bytes at %lu Bytes/Sec\r\n"), FarAddress - p1, s2 ? ((FarAddress - p1) * 1000 / s2 / portTICK_PERIOD_MS) : 0 );
					f_close(&File[0]);
					break;

				default :
					break;
				}
				break;

		case 't' :	/* t [<year yy> <month mm> <date dd> <day: Sun=1> <hour hh> <minute mm> <second ss>] */

			if (xatoi(&ptr, &p1)) {
				if (p1 < 100) CurrTimeDate.tm_year = (uint8_t)p1 ;
				else CurrTimeDate.tm_year = (uint8_t)(p1 -1900);
				xatoi(&ptr, &p1); CurrTimeDate.tm_mon = (uint8_t)(p1 -1);
				xatoi(&ptr, &p1); CurrTimeDate.tm_mday = (uint8_t)p1;
				xatoi(&ptr, &p1); CurrTimeDate.tm_wday = (uint8_t)(p1 -1);
				xatoi(&ptr, &p1); CurrTimeDate.tm_hour = (uint8_t)p1;
				xatoi(&ptr, &p1); CurrTimeDate.tm_min = (uint8_t)p1;
				if (!xatoi(&ptr, &p1))
					break;
				CurrTimeDate.tm_sec = (uint8_t)p1;

		    	xSerialPrintf_P(PSTR("Setting Local Time: %s\r\n"), asctime( (ptm)&CurrTimeDate ));

				set_system_time( mktime( (ptm)&CurrTimeDate));

#ifdef portRTC_DEFINED

				if (setDateTimeDS1307( (ptm)&CurrTimeDate ) == pdTRUE)
					xSerialPrint_P( PSTR("RTC Setting successful\r\n") );
#endif

			} else {
				time((time_t *)&p1);
				xSerialPrintf_P(PSTR("SYSTEM Time: %s\r\n"), ctime( (time_t *)&p1) );

#ifdef portRTC_DEFINED

				if (getDateTimeDS1307( (ptm)&CurrTimeDate ) == pdTRUE)
				{
					xSerialPrintf_P(PSTR("RTC    Time: %s\r\n"), asctime( (ptm)&CurrTimeDate ));

				}

				if ( mktime( (ptm)&CurrTimeDate) != time(NULL))
				{
					xSerialPrintf_P(PSTR("Clock Divergence. Reset SYSTEM Clock? (Y/n)="), (uint16_t)p1);
					get_line(ptr, (sizeof(uint8_t)* LINE_SIZE) );
					if (*ptr == 'Y')
					{
						if (getDateTimeDS1307( (ptm)&CurrTimeDate ) == pdTRUE)
							set_system_time( mktime( (ptm)&CurrTimeDate));

					}
				}
#endif

			}

			break;

		}
// 		xSerialPrintf_P(PSTR("\r\nSD Monitor HighWater @ %u\r\n\n"), uxTaskGetStackHighWaterMark(NULL));
	}
}

/*-----------------------------------------------------------*/
/* Standard Tasks                                            */
/*-----------------------------------------------------------*/

static void TaskBlinkRedLED(void *pvParameters) // Main Red LED Flash
{
    (void) pvParameters;

    TickType_t xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

#if defined (portRTC_DEFINED) && defined (portHD44780_LCD)
	tm CurrTimeDate; 			// set up an array for the RTC info.
#endif

	DDRB |= _BV(DDB7); // Set LED to output

    for(;;)
    {

    	PORTB |=  _BV(PORTB7);       // main (red IO_B7) LED on. EtherMega LED on
		vTaskDelayUntil( &xLastWakeTime, ( 64 / portTICK_PERIOD_MS ) );

#if defined (portHD44780_LCD)

		lcd_Locate (1, 0);
		lcd_Printf_P(PSTR("Free Heap:%6u"),xPortGetFreeHeapSize() ); // needs heap_1 or heap_2 for this function to succeed.

#if defined (portRTC_DEFINED)

		if (getDateTimeDS1307( (ptm)&CurrTimeDate ) == pdTRUE){
			lcd_Locate (0, 8);
			lcd_Printf_P(PSTR("%02u:%02u:%02u"), CurrTimeDate.tm_hour, CurrTimeDate.tm_min, CurrTimeDate.tm_sec);
		}

#endif

#endif

		PORTB &= ~_BV(PORTB7);       // main (red IO_B7) LED off. EtherMega LED off
		vTaskDelayUntil( &xLastWakeTime, ( 448 / portTICK_PERIOD_MS ) );

//		xSerialPrintf_P(PSTR("RedLED HighWater @ %u\r\r\n"), uxTaskGetStackHighWaterMark(NULL));
    }

}

/*-----------------------------------------------------------*/
/* Additional helper functions */
/*-----------------------------------------------------------*/

/* Monitor                                                   */

static
void put_dump (const uint8_t *buff, uint32_t ofs, uint8_t cnt)
{
	uint8_t i;


	xSerialPrintf_P(PSTR("%08lX "), ofs);

	for(i = 0; i < cnt; i++)
		xSerialPrintf_P(PSTR(" %02X"), buff[i]);

	xSerialPutChar( &xSerialPort, ' ' );
	for(i = 0; i < cnt; i++)
	{
		xSerialPutChar( &xSerialPort, (buff[i] >= ' ' && buff[i] <= '~') ? buff[i] : '.' );
	}
	xSerialPrint((uint8_t *)"\r\n");
	vTaskDelay( 16 / portTICK_PERIOD_MS ); // Whoa... too fast.
}

static
void get_line (uint8_t *buff, uint8_t len)
{
	uint8_t c;
	uint8_t i = 0;

	for (;;) {
		while ( ! xSerialGetChar( &xSerialPort, &c ))
			vTaskDelay( 1 );

		if (c == '\r') break;
		if ((c == '\b') && i) {
			--i;
			xSerialPutChar( &xSerialPort, c );
			continue;
		}
		if (c >= ' ' && i < len - 1) {	/* Visible chars */
			buff[i++] = c;
			xSerialPutChar( &xSerialPort, c );
		}
	}
	buff[i] = 0;
	xSerialPrint((uint8_t *)"\r\n");
}

#if _FS_MINIMIZE < 1
static
FRESULT scan_files (
	uint8_t* path		/* Pointer to the working buffer with start path */
)
{
	DIR dirs;
	FRESULT res;
	int16_t i;
	uint8_t *fn;

	res = f_opendir(&dirs, path);
	if (res == FR_OK) {
		i = strlen((char *) path);
		while (((res = f_readdir(&dirs, &Finfo)) == FR_OK) && Finfo.fname[0]) {
			if (_FS_RPATH && Finfo.fname[0] == '.') continue;
#if _USE_LFN
			fn = *Finfo.lfname ? Finfo.lfname : Finfo.fname;
#else
			fn = Finfo.fname;
#endif
			if (Finfo.fattrib & AM_DIR) {
				AccDirs++;
				*(path+i) = '/'; strcpy((char *)path+i+1,(char *)fn);
				res = scan_files(path);
				*(path+i) = '\0';
				if (res != FR_OK) break;
			} else {
				xSerialPrintf_P(PSTR("%s/%s\r\n"), path, fn);
				vTaskDelay( 16 / portTICK_PERIOD_MS ); // Whoa... too fast.
				AccFiles++;
				AccSize += Finfo.fsize;
			}
		}
	}

	return res;
}
#endif


static
void put_rc (FRESULT rc)
{
	static const uint8_t *p;
	static const  uint8_t str[] PROGMEM =
		"OK\0" "DISK_ERR\0" "INT_ERR\0" "NOT_READY\0" "NO_FILE\0" "NO_PATH\0"
		"INVALID_NAME\0" "DENIED\0" "EXIST\0" "INVALID_OBJECT\0" "WRITE_PROTECTED\0"
		"INVALID_DRIVE\0" "NOT_ENABLED\0" "NO_FILE_SYSTEM\0" "MKFS_ABORTED\0" "TIMEOUT\0"
		"LOCKED\0" "NOT_ENOUGH_CORE\0" "TOO_MANY_OPEN_FILES\0";
	FRESULT i;

	for (p = str, i = 0; i != rc && pgm_read_byte(p); i++) {
		while(pgm_read_byte(p++));
	}
	xSerialPrintf_P(PSTR("\r\nF:%2u: %S\r\n"), rc, p);
}

/* DESTRUCTIVE DISKIO TEST FUNCTIONS                              */

static
uint32_t pn (
    uint32_t pns
)
{
    static uint32_t lfsr;
    uint16_t n;

    if (pns) {
        lfsr = pns;
        for (n = 0; n < 32; n++) pn(0);
    }
    if (lfsr & 1) {
        lfsr >>= 1;
        lfsr ^= 0x80200003;
    } else {
        lfsr >>= 1;
    }
    return lfsr;
}


static uint8_t
test_diskio (
    uint8_t pdrv,      /* Physical drive number to be checked (all data on the drive will be lost) */
    uint16_t ncyc,     /* Number of test cycles */
    uint8_t* pbuff,    /* Pointer to the working buffer */
    uint16_t sz_buff   /* Size of the working buffer in unit of byte */
)
{
    uint16_t n, cc, ns;
    uint32_t sz_drv, sz_eblk, lba, lba2, pns = 1;
    uint16_t sz_sect;
    uint8_t errors = 0;
    DSTATUS ds;
    DRESULT dr;



    xSerialPrintf_P(PSTR("test_diskio(%u, %u, 0x%08X, 0x%08X)\n"), pdrv, ncyc, (uint16_t)&pbuff, sz_buff);

    if (sz_buff < _MAX_SS + 4) {
    	xSerialPrint_P(PSTR("Insufficient work area to test.\n"));
        return ++errors;
    }

 	if(pbuff == NULL) // if there is no pbuff buffer allocated (pointer is NULL), then allocate buffer.
		if( !(pbuff = (uint8_t *) pvPortMalloc( sizeof(uint8_t) * sz_buff )))
		{
			xSerialPrint_P(PSTR("pvPortMalloc for *pbuff fail..!\r\n"));
        	return ++errors;
		}

    for (cc = 1; cc <= ncyc; cc++) {
    	xSerialPrintf_P(PSTR("**** Test cycle %u of %u start ****\n"), cc, ncyc);

        /* Initialization */
    	xSerialPrintf_P(PSTR(" disk_initalize(%u)"), pdrv);
        ds = disk_initialize(pdrv);

        if (ds & STA_NODISK)
        {
        	xSerialPrint_P(PSTR(" - failed, no disk .\n"));
            return ++errors;
        }
        else if (ds & STA_NOINIT)
        {
        	xSerialPrint_P(PSTR(" - failed initialisation.\n"));
           return ++errors;
        }
        else
        {
        	xSerialPrint_P(PSTR(" - ok.\n"));
        }

        /* Get drive size */
        xSerialPrint_P(PSTR("**** Get drive size ****\n"));
        xSerialPrintf_P(PSTR(" disk_ioctl(%u, GET_SECTOR_COUNT, 0x%X)"), pdrv, (uint16_t)&sz_drv);
        sz_drv = 0;
        dr = disk_ioctl(pdrv, GET_SECTOR_COUNT, &sz_drv);
        if (dr == RES_OK) {
        	xSerialPrint_P(PSTR(" - ok.\n"));
        } else {
        	xSerialPrint_P(PSTR(" - failed.\n"));
            ++errors;
        }
        if (sz_drv < 128) {
        	xSerialPrint_P(PSTR("Failed: Insufficient drive size to test.\n"));
            return ++errors;
        }
        xSerialPrintf_P(PSTR(" Number of sectors on the drive %u is %lu.\n"), pdrv, sz_drv);

#if _MAX_SS != _MIN_SS
        /* Get sector size */
        xSerialPrint_P(PSTR("**** Get sector size ****\n"));
        xSerialPrintf_P(PSTR(" disk_ioctl(%u, GET_SECTOR_SIZE, 0x%X)"), pdrv, (uint16_t)&sz_sect);
        sz_sect = 0;
        dr = disk_ioctl(pdrv, GET_SECTOR_SIZE, &sz_sect);
        if (dr == RES_OK) {
        	xSerialPrint_P(PSTR(" - ok.\n"));
        } else {
        	xSerialPrint_P(PSTR(" - failed.\n"));
        	++errors;
        }
        xSerialPrintf_P(PSTR(" Size of sector is %u bytes.\n"), sz_sect);
#else
        sz_sect = _MAX_SS;
#endif

        /* Get erase block size */
        xSerialPrint_P(PSTR("**** Get block size ****\n"));
        xSerialPrintf_P(PSTR(" disk_ioctl(%u, GET_BLOCK_SIZE, 0x%X)"), pdrv, (uint16_t)&sz_eblk);
        sz_eblk = 0;
        dr = disk_ioctl(pdrv, GET_BLOCK_SIZE, &sz_eblk);
        if (dr == RES_OK) {
        	xSerialPrint_P(PSTR(" - ok.\n"));
        } else {
        	xSerialPrint_P(PSTR(" - failed.\n"));
        	++errors;
        }
        if (dr == RES_OK || sz_eblk >= 2) {
        	xSerialPrintf_P(PSTR(" Size of the erase block is %lu sectors.\n"), sz_eblk);
        } else {
        	xSerialPrint_P(PSTR(" Size of the erase block is unknown.\n"));
        }

        /* Single sector write test */
        xSerialPrint_P(PSTR("**** Single sector write test 1 ****\n"));
        lba = 0;
        for (n = 0, pn(pns); n < sz_sect; n++) pbuff[n] = (uint8_t)pn(0);
        xSerialPrintf_P(PSTR(" disk_write(%u, 0x%X, %lu, 1)"), pdrv, (uint16_t)&pbuff, lba);
        dr = disk_write(pdrv, pbuff, lba, 1);
        if (dr == RES_OK) {
        	xSerialPrint_P(PSTR(" - ok.\n"));
        } else {
        	xSerialPrint_P(PSTR(" - failed.\n"));
        	++errors;
        }

    	vTaskDelay( 128 / portTICK_PERIOD_MS ); // Whoa... too fast.

        xSerialPrintf_P(PSTR(" disk_ioctl(%u, CTRL_SYNC, NULL)"), pdrv);
        dr = disk_ioctl(pdrv, CTRL_SYNC, NULL);
        if (dr == RES_OK) {
        	xSerialPrint_P(PSTR(" - ok.\n"));
        } else {
        	xSerialPrint_P(PSTR(" - failed.\n"));
        	++errors;
        }
        memset(pbuff, 0, sz_sect);
        xSerialPrintf_P(PSTR(" disk_read(%u, 0x%X, %lu, 1)"), pdrv, (uint16_t)&pbuff, lba);
        dr = disk_read(pdrv, pbuff, lba, 1);
        if (dr == RES_OK) {
        	xSerialPrint_P(PSTR(" - ok.\n"));
        } else {
        	xSerialPrint_P(PSTR(" - failed.\n"));
        	++errors;
        }
        for (n = 0, pn(pns); n < sz_sect && pbuff[n] == (uint8_t)pn(0); n++) ;
        if (n == sz_sect) {
        	xSerialPrint_P(PSTR(" Data matched.\n"));
        } else {
        	xSerialPrint_P(PSTR("Failed: Read data differs from the data written.\n"));
        	++errors;
        }
        pns++;

    	vTaskDelay( 128 / portTICK_PERIOD_MS ); // Whoa... too fast.

        /* Multiple sector write test */
        xSerialPrint_P(PSTR("**** Multiple sector write test ****\n"));
        lba = 1; ns = sz_buff / sz_sect;
        if (ns > 4) ns = 4;
        for (n = 0, pn(pns); n < (uint16_t)(sz_sect * ns); n++) pbuff[n] = (uint8_t)pn(0);
        xSerialPrintf_P(PSTR(" disk_write(%u, 0x%X, %lu, %u)"), pdrv, (uint16_t)&pbuff, lba, ns);
        dr = disk_write(pdrv, pbuff, lba, ns);
        if (dr == RES_OK) {
        	xSerialPrint_P(PSTR(" - ok.\n"));
        } else {
        	xSerialPrint_P(PSTR(" - failed.\n"));
        	++errors;
        }

    	vTaskDelay( 128 / portTICK_PERIOD_MS ); // Whoa... too fast.

        xSerialPrintf_P(PSTR(" disk_ioctl(%u, CTRL_SYNC, NULL)"), pdrv);
        dr = disk_ioctl(pdrv, CTRL_SYNC, NULL);
        if (dr == RES_OK) {
        	xSerialPrint_P(PSTR(" - ok.\n"));
        } else {
        	xSerialPrint_P(PSTR(" - failed.\n"));
        	++errors;
        }
        memset(pbuff, 0, sz_sect * ns);
        xSerialPrintf_P(PSTR(" disk_read(%u, 0x%X, %lu, %u)"), pdrv, (uint16_t)&pbuff, lba, ns);
        dr = disk_read(pdrv, pbuff, lba, ns);
        if (dr == RES_OK) {
        	xSerialPrint_P(PSTR(" - ok.\n"));
        } else {
        	xSerialPrint_P(PSTR(" - failed.\n"));
        	++errors;
        }
        for (n = 0, pn(pns); n < (uint16_t)(sz_sect * ns) && pbuff[n] == (uint8_t)pn(0); n++) ;
        if (n == (uint16_t)(sz_sect * ns)) {
        	xSerialPrint_P(PSTR(" Data matched.\n"));
        } else {
        	xSerialPrint_P(PSTR("Failed: Read data differs from the data written.\n"));
        	++errors;
        }
        pns++;

    	vTaskDelay( 128 / portTICK_PERIOD_MS ); // Whoa... too fast.

        /* Single sector write test (misaligned memory address) */
        xSerialPrint_P(PSTR("**** Single sector write test 2 ****\n"));
        lba = 5;
        for (n = 0, pn(pns); n < sz_sect; n++) pbuff[n+3] = (uint8_t)pn(0);
        xSerialPrintf_P(PSTR(" disk_write(%u, 0x%X, %lu, 1)"), pdrv, (uint16_t)(&pbuff+3), lba);
        dr = disk_write(pdrv, pbuff+3, lba, 1);
        if (dr == RES_OK) {
        	xSerialPrint_P(PSTR(" - ok.\n"));
        } else {
        	xSerialPrint_P(PSTR(" - failed.\n"));
        	++errors;
        }

    	vTaskDelay( 128 / portTICK_PERIOD_MS ); // Whoa... too fast.

        xSerialPrintf_P(PSTR(" disk_ioctl(%u, CTRL_SYNC, NULL)"), pdrv);
        dr = disk_ioctl(pdrv, CTRL_SYNC, NULL);
        if (dr == RES_OK) {
        	xSerialPrint_P(PSTR(" - ok.\n"));
        } else {
        	xSerialPrint_P(PSTR(" - failed.\n"));
        	++errors;
        }
        memset(pbuff+5, 0, sz_sect);
        xSerialPrintf_P(PSTR(" disk_read(%u, 0x%X, %lu, 1)"), pdrv, (uint16_t)(&pbuff+5), lba);
        dr = disk_read(pdrv, pbuff+5, lba, 1);
        if (dr == RES_OK) {
        	xSerialPrint_P(PSTR(" - ok.\n"));
        } else {
        	xSerialPrint_P(PSTR(" - failed.\n"));
        	++errors;
        }
        for (n = 0, pn(pns); n < sz_sect && pbuff[n+5] == (uint8_t)pn(0); n++) ;
        if (n == sz_sect) {
        	xSerialPrint_P(PSTR(" Data matched.\n"));
        } else {
        	xSerialPrint_P(PSTR("Failed: Read data differs from the data written.\n"));
        	++errors;
        }
        pns++;

    	vTaskDelay( 128 / portTICK_PERIOD_MS ); // Whoa... too fast.

        /* 4GB barrier test */
        xSerialPrint_P(PSTR("**** 4GB barrier test ****\n"));
        if (sz_drv >= 128 + 0x80000000 / (sz_sect / 2)) {
            lba = 6; lba2 = lba + 0x80000000 / (sz_sect / 2);
            for (n = 0, pn(pns); n < (uint16_t)(sz_sect * 2); n++) pbuff[n] = (uint8_t)pn(0);
            xSerialPrintf_P(PSTR(" disk_write(%u, 0x%X, %lu, 1)"), pdrv, (uint16_t)&pbuff, lba);
            dr = disk_write(pdrv, pbuff, lba, 1);
            if (dr == RES_OK) {
            	xSerialPrint_P(PSTR(" - ok.\n"));
            } else {
            	xSerialPrint_P(PSTR(" - failed.\n"));
            	++errors;
            }

        	vTaskDelay( 128 / portTICK_PERIOD_MS ); // Whoa... too fast.

            xSerialPrintf_P(PSTR(" disk_write(%u, 0x%X, %lu, 1)"), pdrv, (uint16_t)(&pbuff+sz_sect), lba2);
            dr = disk_write(pdrv, pbuff+sz_sect, lba2, 1);
            if (dr == RES_OK) {
            	xSerialPrint_P(PSTR(" - ok.\n"));
            } else {
            	xSerialPrint_P(PSTR(" - failed.\n"));
            	++errors;
            }

        	vTaskDelay( 128 / portTICK_PERIOD_MS ); // Whoa... too fast.

            xSerialPrintf_P(PSTR(" disk_ioctl(%u, CTRL_SYNC, NULL)"), pdrv);
            dr = disk_ioctl(pdrv, CTRL_SYNC, NULL);
            if (dr == RES_OK) {
            	xSerialPrint_P(PSTR(" - ok.\n"));
            } else {
            	xSerialPrint_P(PSTR(" - failed.\n"));
            	++errors;
            }
            memset(pbuff, 0, sz_sect * 2);
            xSerialPrintf_P(PSTR(" disk_read(%u, 0x%X, %lu, 1)"), pdrv, (uint16_t)&pbuff, lba);
            dr = disk_read(pdrv, pbuff, lba, 1);
            if (dr == RES_OK) {
            	xSerialPrint_P(PSTR(" - ok.\n"));
            } else {
            	xSerialPrint_P(PSTR(" - failed.\n"));
            	++errors;
            }

        	vTaskDelay( 128 / portTICK_PERIOD_MS ); // Whoa... too fast.

            xSerialPrintf_P(PSTR(" disk_read(%u, 0x%X, %lu, 1)"), pdrv, (uint16_t)(&pbuff+sz_sect), lba2);
            dr = disk_read(pdrv, pbuff+sz_sect, lba2, 1);
            if (dr == RES_OK) {
            	xSerialPrint_P(PSTR(" - ok.\n"));
            } else {
            	xSerialPrint_P(PSTR(" - failed.\n"));
            	++errors;
            }
            for (n = 0, pn(pns); pbuff[n] == (uint8_t)pn(0) && n < (uint16_t)(sz_sect * 2); n++) ;
            if (n == (uint16_t)(sz_sect * 2)) {
            	xSerialPrint_P(PSTR(" Data matched.\n"));
            } else {
            	xSerialPrint_P(PSTR("Failed: Read data differs from the data written.\n"));
            	++errors;
            }
        } else {
        	xSerialPrint_P(PSTR(" Test skipped.\n"));
        }
        pns++;

        xSerialPrintf_P(PSTR("**** Test cycle %u of %u completed ****\n\n"), cc, ncyc);
    }

    return errors;
}


/*-----------------------------------------------------------*/


void vApplicationStackOverflowHook( TaskHandle_t xTask,
									char *pcTaskName )
{
	/*---------------------------------------------------------------------------*\
	Usage:
	   called by task system when a stack overflow is noticed
	Description:
	   Stack overflow handler -- Shut down all interrupts, send serious complaint
	    to command port.
	Arguments:
	   pxTask - pointer to task handle
	   pcTaskName - pointer to task name
	Results:
	   <none>
	Notes:
	   This routine will never return.
	   This routine is referenced in the task.c file of FreeRTOS as an extern.
	\*---------------------------------------------------------------------------*/

	uint8_t* pC;
	uint16_t baud;

	/* shut down all interrupts */
	portDISABLE_INTERRUPTS();

	/* take over the command line buffer to generate our error message */
	pC = (uint8_t*) LineBuffer;

	strcat_P( (char*) pC, PSTR("\r\n"));
	strcat( (char*) pC, (char*) pcTaskName );
	strcat_P( (char*) pC, PSTR("\r\n"));

	pC = (uint8_t*) LineBuffer;

	/* Force the UART control register to be the way we want, just in case */

	UCSR0C = ( _BV( UCSZ01 ) | _BV( UCSZ00 ) );		// 8 data bits
	UCSR0B = _BV( TXEN0 );							// only enable transmit
	UCSR0A = 0;

	/* Calculate the baud rate register value from the equation in the
	* data sheet.  This calculation rounds to the nearest factor, which
	* means the resulting rate may be either faster or slower than the
	* desired rate (the old calculation was always faster).
	*
	* If the system clock is one of the Magic Frequencies, this
	* computation will result in the exact baud rate
	*/
	baud = ( ( ( configCPU_CLOCK_HZ / ( ( 16UL * 38400 ) / 2UL ) ) + 1UL ) / 2UL ) - 1UL;
	UBRR0 = baud;

	/* Send out the message, without interrupts.  Hard wired to USART 0 */
	while ( *pC )
	{
		while (!(UCSR0A & (1 << UDRE0)));
		UDR0 = *pC;
		pC++;
	}

	while(1){ PINB |= _BV(PINB7); _delay_ms(100); } // main (red PB7) LED flash and die.
}
/*-----------------------------------------------------------*/
