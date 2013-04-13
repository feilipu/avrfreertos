////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////    main.c
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include <avr/io.h>

/* Scheduler include files. */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include <lib_crc.h>

/* serial interface include file. */
#include <lib_serial.h>

/* FatF interface include file. */
#include <ff.h>

/* RAMFS RAM file system interface include file. */
#include <ramfs.h>

#include "xitoa.h"

/*--------------------  Definitions  ---------------------------*/

#define CMD_BUFFER_SIZE		64			// size of working buffer (on heap) for FatFS


#define LINE_SIZE 			32			// size of Client command line (on heap)

/*-------------------- Global Variables ------------------------*/

xComPortHandle xSerialPort;				// Create a handle for the serial port.

uint8_t * LineBuffer;					// put line buffer on heap (with pvPortMalloc).

/*-----------------------------------------------------------*/
/* Create files for the SD card and FatFS. */

static uint32_t AccSize;				/* Work register for fs command */
static uint16_t AccFiles, AccDirs;
static FILINFO Finfo;

#if _USE_LFN
static uint8_t Lfname[_MAX_LFN+1];
#endif

static FATFS Fatfs[_VOLUMES];		/* File system object for each logical drive */

static FIL File[_FS_LOCK];			/* File object. there are _FS_LOCK file objects available */

/* Working buffer */
static uint8_t *Buff;				// Put working buffer on heap (with pvPortMalloc).


/*-----------------  Private Functions  ----------------------*/

static void put_dump (const uint8_t *buff, uint32_t ofs, uint8_t cnt);
static void get_line (uint8_t *buff, uint8_t len);
static FRESULT scan_files (	uint8_t* path );
static void put_rc (FRESULT rc);


/*--------------------   Tasks   ------------------------------*/

// DO NOT FLASH THE LED on PB5. It hangs off the SPI bus and kills things for other clients. etc.

static void TaskMonitor(void *pvParameters);		// Serial monitor for XRAMFS Testing

/*-----------------------------------------------------------*/

/* Main program loop */
int16_t main(void) __attribute__((OS_main));

int16_t main(void)
{

    // turn on the serial port for debugging or for other USART reasons.
	xSerialPort = xSerialPortInitMinimal( 115200, portSERIAL_BUFFER, 16); //  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)

	avrSerialPrint_P(PSTR("\r\n\n\nHello World!\r\n")); // Ok, so we're alive...

    xTaskCreate(
 		TaskMonitor
 		,  (const signed portCHAR *)"Monitor" // Serial Monitor
 		,  208
 		,  NULL
 		,  3
 		,  NULL ); // */

	avrSerialPrintf_P(PSTR("\r\n\nFree Heap Size: %u\r\n"),xPortGetFreeHeapSize() ); // needs heap_1 or heap_2 or heap_4 for this function to succeed.

	vTaskStartScheduler();

	avrSerialPrint_P(PSTR("\r\n\n\nGoodbye... no space for idle task!\r\n")); // Doh, so we're dead...

}



/*---------------------------------------------------------------------------*/

static void TaskMonitor(void *pvParameters) // Monitor for Serial Interface
{
    (void) pvParameters;

    portTickType Timer;

	uint8_t *ptr, *ptr2;
	uint32_t p1, p2, p3;
	uint8_t res, b1, *bp;
	uint16_t s1, s2, cnt;
	uint32_t ofs, sect = 0;
//	FATFS *fs;
	DIR dir;

	// create the working buffers on the heap (so they can be moved later).

	if(LineBuffer == NULL) // if there is no Line buffer allocated (pointer is NULL), then allocate buffer.
		if( !(LineBuffer = (uint8_t *) pvPortMalloc( sizeof(uint8_t) * LINE_SIZE )))
			xSerialPrint_P(PSTR("pvPortMalloc for *LineBuffer fail..!\r\n"));

	if(Buff == NULL) // if there is no Buff buffer allocated (pointer is NULL), then allocate buffer.
		if( !(Buff = (uint8_t *) pvPortMalloc( sizeof(uint8_t) * CMD_BUFFER_SIZE )))
			xSerialPrint_P(PSTR("pvPortMalloc for *Buff fail..!\r\n"));

	xRAMFSarray testRAMFS;			// this is just a  test array for XRAMFS info.

	uint8_t RAMbyte;				// initial fill value

	uint8_t * pLocalRAM = NULL;		// pointer to local ram.

#if _USE_LFN
	Finfo.lfname = Lfname;
	Finfo.lfsize = sizeof (Lfname);
#endif

	xSerialPrint_P(PSTR("\r\nXRAMFS FatFs test monitor"));
	xSerialPrint_P(_USE_LFN ? PSTR("\r\nLFN Enabled") : PSTR("\r\nLFN Disabled"));
	xSerialPrintf_P(PSTR(", Code page: %u\r\n"), _CODE_PAGE);

	while(1)
    {
    	xSerialPutChar(xSerialPort, '>', 100 / portTICK_RATE_MS);

		ptr = LineBuffer;
		get_line(ptr, (uint8_t)(sizeof(uint8_t)* LINE_SIZE));  //sizeof (Line)

		switch (*ptr++) {

		case 'c' :	// Create XRAMFS structure. Allocate and fill local RAM with
					// > c ram_size [fill_bytes]   allocate, fill and calculate CRC8
					// > c                         just calculate a CRC8

			if (xatoi(&ptr, &p1)) {
				testRAMFS.ram_size = (uint16_t)p1;

				// "create" the XRAMFS information.
				if( !(testRAMFS.ram_addr = (uint16_t ) vRAMFSMalloc( sizeof(uint8_t) * testRAMFS.ram_size )))
					xSerialPrint_P(PSTR("vRAMFSMalloc for testRAMFS fail..!\r\n"));

				// create the local RAM on the heap.
				if( !(pLocalRAM = (uint8_t *) pvPortMalloc( sizeof(uint8_t) * testRAMFS.ram_size )))
					xSerialPrint_P(PSTR("pvPortMalloc for *pLocalRam fail..!\r\n"));

				if (xatoi(&ptr, &p1))
				{
					RAMbyte = (uint8_t)p1;
					for (uint16_t i = 0; i < testRAMFS.ram_size; i++)
						pLocalRAM[i] = RAMbyte + i;
				} else {
					for (uint16_t i = 0; i < testRAMFS.ram_size; i++)
						pLocalRAM[i] = (uint8_t)rand();
				}

				// calculate a CRC on the RAM.
				testRAMFS.ram_crc8 = crc8( pLocalRAM, ( sizeof(uint8_t) * testRAMFS.ram_size ) );

				xSerialPrintf_P(PSTR("Addr: %4x Size: %u crc: %2x\r\n"), testRAMFS.ram_addr, testRAMFS.ram_size, testRAMFS.ram_crc8 );
				xSerialPrintf_P(PSTR("xRAMFSGetFreeSize: %u\r\n"),xRAMFSGetFreeSize());
				xSerialPrintf_P(PSTR("Free Heap Size:    %u\r\n"),xPortGetFreeHeapSize() ); // needs heap_1 or heap_2 or heap_4 for this function to succeed.

			} else {

				// calculate a CRC on the RAM (to test).
				testRAMFS.ram_crc8 = crc8( pLocalRAM, ( sizeof(uint8_t) * testRAMFS.ram_size ) );

				xSerialPrintf_P(PSTR("Addr: %4x Size: %u crc: %2x\r\n"), testRAMFS.ram_addr, testRAMFS.ram_size, testRAMFS.ram_crc8 );
				xSerialPrintf_P(PSTR("xRAMFSGetFreeSize: %u\r\n"),xRAMFSGetFreeSize());
				xSerialPrintf_P(PSTR("Free Heap Size:    %u\r\n"),xPortGetFreeHeapSize() ); // needs heap_1 or heap_2 or heap_4 for this function to succeed.
			}
			break;


		case 'p' : // Print the RAM contents.

			if(pLocalRAM != NULL)
				for (uint16_t i = 0; i < testRAMFS.ram_size; i+=8)
				{
					xSerialPrintf_P(PSTR("%4x: %2x %2x %2x %2x %2x %2x %2x %2x\r\n"), testRAMFS.ram_addr + i, pLocalRAM[i],pLocalRAM[i+1],pLocalRAM[i+2],pLocalRAM[i+3],pLocalRAM[i+4],pLocalRAM[i+5],pLocalRAM[i+6],pLocalRAM[i+7]);
					vTaskDelay(  10 / portTICK_RATE_MS );
				}
			xSerialPrint_P(PSTR("\r\n"));
			break;


		case 'r' : // Read the XRAMFS contents into RAM.

			// Set the Command for Read (from XRAMFS)
			testRAMFS.ram_cmd = Read;

			if(ramfs_transfer_block( &testRAMFS, pLocalRAM ))
				xSerialPrint_P(PSTR("Read XRAMFS: Fail\r\n"));
			else
				xSerialPrint_P(PSTR("Read XRAMFS: Success\r\n"));
			break;


		case 'w' : // Write the RAM contents into XRAMFS.

			// Set the Command for Write (to XRAMFS)
			testRAMFS.ram_cmd = Write;

			if(ramfs_transfer_block( &testRAMFS, pLocalRAM ))
				xSerialPrint_P(PSTR("Write XRAMFS: Fail\r\n"));
			else
				xSerialPrint_P(PSTR("Write XRAMFS: Success\r\n"));
			break;


		case 's' : // Swap the RAM contents with the XRAMFS contents.

			// Set the Command for Write (to XRAMFS)
			testRAMFS.ram_cmd = Swap;

			if(ramfs_transfer_block( &testRAMFS, pLocalRAM ))
				xSerialPrint_P(PSTR("Swap XRAMFS: Fail\r\n"));
			else
				xSerialPrint_P(PSTR("Swap XRAMFS: Success\r\n"));
			break;


		case 't' : // Write, Read, randomise, Swap, randomise, and repeat checking for errors.

			// Set the Command for Write (to XRAMFS)
			testRAMFS.ram_cmd = Write;

			if(ramfs_transfer_block( &testRAMFS, pLocalRAM ))
				xSerialPrint_P(PSTR("Write XRAMFS: Fail\r\n"));
			else
				xSerialPrint_P(PSTR("Write XRAMFS: Success\r\n"));

			// Set the Command for Read (from XRAMFS)
			testRAMFS.ram_cmd = Read;

			if(ramfs_transfer_block( &testRAMFS, pLocalRAM ))
				xSerialPrint_P(PSTR("Read XRAMFS: Fail\r\n"));
			else
				xSerialPrint_P(PSTR("Read XRAMFS: Success\r\n"));

			// Randomise the RAM contents
			for (uint16_t i = 0; i < testRAMFS.ram_size; i++)
				pLocalRAM[i] = (uint8_t)rand();

			// Set the Command for Swap (with XRAMFS)
			testRAMFS.ram_cmd = Swap;

			if(ramfs_transfer_block( &testRAMFS, pLocalRAM ))
				xSerialPrint_P(PSTR("Swap XRAMFS: Fail\r\n"));
			else
				xSerialPrint_P(PSTR("Swap XRAMFS: Success\r\n"));

			// Set the Command for Swap (with XRAMFS)
			testRAMFS.ram_cmd = Swap;

			if(ramfs_transfer_block( &testRAMFS, pLocalRAM ))
				xSerialPrint_P(PSTR("Swap XRAMFS: Fail\r\n"));
			else
				xSerialPrint_P(PSTR("Swap XRAMFS: Success\r\n"));
			break;


		case 'z' : // Randomise the RAM contents.

			srand((uint16_t)xTaskGetTickCount()); // seed a random number

			if( (pLocalRAM != NULL) && (testRAMFS.ram_size != 0) )
				for (uint16_t i = 0; i < testRAMFS.ram_size; i++)
					pLocalRAM[i] = (uint8_t)rand();

			// calculate a CRC on the RAM.
			testRAMFS.ram_crc8 = crc8( pLocalRAM, ( sizeof(uint8_t) * testRAMFS.ram_size ) );

			xSerialPrintf_P(PSTR("Addr: %4x Size: %u crc: %2x\r\n"), testRAMFS.ram_addr, testRAMFS.ram_size, testRAMFS.ram_crc8 );
			break;


		case 'l' : // Generate load.

			// Create RAMFS structure. Allocate and fill local RAM with
			// > l [ram_size]    allocate, fill, generate load
			// > l               just generate load (allocate, fill, default 1024 Bytes transfer)

			if (xatoi(&ptr, &p1))
				testRAMFS.ram_size = (uint16_t)p1;
			else
				testRAMFS.ram_size = (uint16_t)256;


				// "create" the RAMFS information.
				if( !(testRAMFS.ram_addr = (uint16_t ) vRAMFSMalloc( sizeof(uint8_t) * testRAMFS.ram_size )))
					xSerialPrint_P(PSTR("vRAMFSMalloc for testRAMFS fail..!\r\n"));

				// create the local RAM on the heap.
				if( !(pLocalRAM = (uint8_t *) pvPortMalloc( sizeof(uint8_t) * testRAMFS.ram_size )))
					xSerialPrint_P(PSTR("pvPortMalloc for *pLocalRam fail..!\r\n"));

			srand((uint16_t)xTaskGetTickCount()); // seed a random number

			while(1)
			{

				xSerialPrint_P(PSTR("XRAMFS Test: "));

				for (uint16_t i = 0; i < testRAMFS.ram_size; i++)
					pLocalRAM[i] = (uint8_t)rand();

				// calculate a CRC on the local RAM.
				testRAMFS.ram_crc8 = crc8( pLocalRAM, ( sizeof(uint8_t) * testRAMFS.ram_size ) );

				xSerialPrintf_P(PSTR("Addr: %4x Size: %u crc: %2x -> "), testRAMFS.ram_addr, testRAMFS.ram_size, testRAMFS.ram_crc8 );

				// Set the Command for Write (to XRAMFS)
				testRAMFS.ram_cmd = Write;

				if(ramfs_transfer_block( &testRAMFS, pLocalRAM ))
					xSerialPrint_P(PSTR("Write XRAMFS: Fail\r\n"));

				// Randomise the RAM contents
				for (uint16_t i = 0; i < testRAMFS.ram_size; i++)
					pLocalRAM[i] = (uint8_t)rand();

				// Set the Command for Swap (with XRAMFS)
				testRAMFS.ram_cmd = Swap;

				if(ramfs_transfer_block( &testRAMFS, pLocalRAM ))
					xSerialPrint_P(PSTR("Swap XRAMFS: Fail\r\n"));

				// Set the Command for Swap (with XRAMFS)
				testRAMFS.ram_cmd = Swap;

				if(ramfs_transfer_block( &testRAMFS, pLocalRAM ))
					xSerialPrint_P(PSTR("Swap XRAMFS: Fail\r\n"));

				// Set the Command for Read (from XRAMFS)
				testRAMFS.ram_cmd = Read;

				if(ramfs_transfer_block( &testRAMFS, pLocalRAM ))
					xSerialPrint_P(PSTR("Read XRAMFS: Fail\r\n"));

				// calculate a CRC on the local RAM.
				if(testRAMFS.ram_crc8 != crc8( pLocalRAM, ( sizeof(uint8_t) * testRAMFS.ram_size ) ))
					xSerialPrint_P(PSTR("CRC8: VERIFICATION FAIL\r\n"));
				else
					xSerialPrint_P(PSTR("Pass\r\n"));

				testRAMFS.ram_addr += testRAMFS.ram_size;

				if( (testRAMFS.ram_addr < (size_t)XRAMSTART) || (testRAMFS.ram_size > (size_t)XRAMEND - testRAMFS.ram_addr) )
					testRAMFS.ram_addr = (size_t)XRAMSTART;

				vTaskDelay( (portTickType) testRAMFS.ram_crc8 / portTICK_RATE_MS );	// delay for a random amount of time (being the CRC8 of the random numbers)
			}
			break;

		case 'd' :
			switch (*ptr++) {
			case 'd' :	/* dd <phy_drv#> [<sector>] - Dump sector */
				if (!xatoi(&ptr, &p1)) break;
				if (!xatoi(&ptr, &p2)) p2 = sect;
				res = disk_read((uint8_t)p1, (uint8_t *)Buff, p2, 1);
				if (res) { xSerialPrintf_P(PSTR("D:%2d\r\n"), res); break; }
				sect = p2 + 1;
				xSerialPrintf_P(PSTR("Sector:%lu\r\n"), p2);
				for (bp=Buff, ofs = 0; ofs < 0x200; bp+=16, ofs+=16)
					put_dump(bp, ofs, 16);
				break;

			case 'i' :	/* di <phy_drv#> - Initialise disk */
				if (!xatoi(&ptr, &p1)) break;
				xSerialPrintf_P(PSTR("D:%2d\r\n"), disk_initialize((uint8_t)p1));
				break;

			case 's' :	/* ds <phy_drv#> - Show disk status */
				if (!xatoi(&ptr, &p1)) break;
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
				put_rc(f_mount((uint8_t)p1, &Fatfs[p1]));
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
				vTaskDelay( 8 / portTICK_RATE_MS ); // Whoa... too fast.
				xSerialPrintf_P(PSTR("FAT start (lba) = %lu\r\nDIR start (lba,cluster) = %lu\r\nData start (lba) = %lu\r\n...\r\n"),
										fs->fatbase, fs->dirbase, fs->database	);
				vTaskDelay( 8 / portTICK_RATE_MS ); // Whoa... too fast.
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
						xSerialPutChar(xSerialPort, ' ', 400 / portTICK_RATE_MS);
					xSerialPrintf_P(PSTR("%s\r\n"), Lfname);
#else
					xSerialPrint("\r\n");
#endif
					vTaskDelay( 8 / portTICK_RATE_MS ); // Whoa... too fast.
				}
				xSerialPrintf_P(PSTR("%4u File(s),%10lu bytes total\r\n%4u Dir(s)"), s1, p1, s2);
#if _FS_MINIMIZE < 1
				if (f_getfree(ptr, (uint32_t*)&p1, &fs) == FR_OK)
					xSerialPrintf_P(PSTR(", %10luK bytes free\r\n"), p1 * fs->csize / 2);
#else
				xSerialPrint("\r\n");
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
				xSerialPrintf_P(PSTR("%lu Bytes read at %lu Bytes/sec.\r\n"), p2, s2 ? (p2 * 1000 / s2 / portTICK_RATE_MS) : 0 );
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
				xSerialPrintf_P(PSTR("%lu Bytes written at %lu Bytes/Sec.\r\n"), p2, s2 ? (p2 * 1000 / s2 / portTICK_RATE_MS) : 0 );
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
				}
				s2 = xTaskGetTickCount() - Timer;
				if (res) put_rc(res);
				xSerialPrintf_P(PSTR(" %lu Bytes at %lu Bytes/Sec\r\n"),	p1, s2 ? (p1 * 1000 / s2 / portTICK_RATE_MS) : 0 );
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
					put_rc(f_chdrive((uint8_t)p1));
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
				if (*ptr == 'Y') put_rc(f_mkfs((uint8_t)p1, (uint8_t)p2, (uint16_t)p3));
				break;
#endif

			default :
				break;

			}
			break;

#ifdef portRTC_DEFINED
		case 't' :	/* t [<year yy> <month mm> <date dd> <day: Sun=1> <hour hh> <minute mm> <second ss>] */

			if (xatoi(&ptr, &p1)) {
				SetTimeDate->Year = (uint8_t)p1;
				xatoi(&ptr, &p1); SetTimeDate->Month = (uint8_t)p1;
				xatoi(&ptr, &p1); SetTimeDate->Date = (uint8_t)p1;
				xatoi(&ptr, &p1); SetTimeDate->Day = (uint8_t)p1;
				xatoi(&ptr, &p1); SetTimeDate->Hour = (uint8_t)p1;
				xatoi(&ptr, &p1); SetTimeDate->Minute = (uint8_t)p1;
				if (!xatoi(&ptr, &p1))
					break;
				SetTimeDate->Second = (uint8_t)p1;

				xSerialPrintf_P(PSTR("Set: %u/%u/%u %2u:%02u:%02u\r\n"), SetTimeDate->Year, SetTimeDate->Month, SetTimeDate->Date, SetTimeDate->Hour, SetTimeDate->Minute, SetTimeDate->Second);
				if (setDateTimeDS1307( SetTimeDate ) == pdTRUE)
					xSerialPrint_P( PSTR("Setting successful\r\n") );

			} else {

				if (getDateTimeDS1307( CurrTimeDate ) == pdTRUE)
					xSerialPrintf_P(PSTR("Current: %u/%u/%u %2u:%02u:%02u\r\n"), CurrTimeDate->Year + 2000, CurrTimeDate->Month, CurrTimeDate->Date, CurrTimeDate->Hour, CurrTimeDate->Minute, CurrTimeDate->Second);
				}

				break;
	#endif


		default :
			break;

		}
		vTaskDelay(  50 / portTICK_RATE_MS );
		xSerialPrintf_P(PSTR("Monitor - Stack HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
		vTaskDelay(  10 / portTICK_RATE_MS );
    }

}


/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/
/* Additional helper functions */
/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
/* Monitor                                                   */
/*-----------------------------------------------------------*/

static
void put_dump (const uint8_t *buff, uint32_t ofs, uint8_t cnt)
{
	uint8_t i;


	xSerialPrintf_P(PSTR("%08lX "), ofs);

	for(i = 0; i < cnt; i++)
		xSerialPrintf_P(PSTR(" %02X"), buff[i]);

	xSerialPutChar(xSerialPort, ' ', 100 / portTICK_RATE_MS);
	for(i = 0; i < cnt; i++)
	{
		xSerialPutChar(xSerialPort, (buff[i] >= ' ' && buff[i] <= '~') ? buff[i] : '.', 200 / portTICK_RATE_MS);
	}
	xSerialPrint((uint8_t *)"\r\n");
	vTaskDelay( 8 / portTICK_RATE_MS ); // Whoa... too fast.
}

static
void get_line (uint8_t *buff, uint8_t len)
{
	uint8_t c;
	uint8_t i = 0;

	for (;;) {
		xSerialGetChar(xSerialPort, &c, portMAX_DELAY);

		if (c == '\r') break;
		if ((c == '\b') && i) {
			--i;
			xSerialPutChar(xSerialPort, c, 100 / portTICK_RATE_MS);
			continue;
		}
		if (c >= ' ' && i < len - 1) {	/* Visible chars */
			buff[i++] = c;
			xSerialPutChar(xSerialPort, c, 100 / portTICK_RATE_MS);
		}
	}
	buff[i] = 0;
	xSerialPrint((uint8_t *)"\r\n");
}

#if _FS_MINIMIZE < 2
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
				vTaskDelay( 8 / portTICK_RATE_MS ); // Whoa... too fast.
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

	for (p = str, i = 0; i != rc && pgm_read_byte_near(p); i++) {
		while(pgm_read_byte_near(p++));
	}
	xSerialPrintf_P(PSTR("\r\nF:%2u: %S\r\n"), rc, p);
	//xSerialPrintf_P(PSTR("\r\nresult code = %u: FR_%S\r\n"), rc, p);
}


/*-----------------------------------------------------------


void vApplicationStackOverflowHook( xTaskHandle xTask,
                                    signed portCHAR *pcTaskName )
{
	DDRB  |= _BV(DDB5);
	PORTB |= _BV(PORTB5);       // main (red PB5) LED on. Arduino LED on and die.
	while(1);
}

-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle xTask,
									signed portCHAR *pcTaskName )
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

	strcat( (char*) pC, "StackOverflow\r\n" );
	strcat( (char*) pC, (char*) pcTaskName );
	strcat( (char*) pC, "\r\n" );

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
	baud = ( ( ( configCPU_CLOCK_HZ / ( ( 16UL * 115200 ) / 2UL ) ) + 1UL ) / 2UL ) - 1UL;
	UBRR0 = baud;

	/* Send out the message, without interrupts.  Hard wired to USART 0 */
	while ( *pC )
	{
		while (!(UCSR0A & (1 << UDRE0)));
		UDR0 = *pC;
		pC++;
	}

	DDRB  |= _BV(DDB5);
	PORTB |= _BV(PORTB5);       // main (red PB5) LED on. Arduino LED on and die.
	while(1);
}

