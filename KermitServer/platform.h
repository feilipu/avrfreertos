/* AVR platform.h for EK */


#ifndef platform_h
#define platform_h

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "serial.h"			// used for the tty send and receive.
#include "spi.h"			// used to read and write to uSD cards.
#include "time.h"			// system time, used to convert to and from FAT file system time.
#include "ff.h"				// FatF interface include file. ffconf.h holds configuration options, and is auto included.

#include "cdefs.h"
#include "debug.h"
#include "kermit.h"			/* Kermit symbols and data structures */

#ifdef _GOLDILOCKS_
//#define MINSIZE			/* Minimum build size */
#define DEBUG				/* Debugging included... */
#define NO_LP               /* No Long packets */
//#define NO_AT             /* No Attribute packets */
//#define NO_CTRLC          /* No 3 consecutive Ctrl-C's to quit */
//#define NO_SSW			/* No Simulated sliding windows */
//#define NO_CRC			/* No Type 2 and 3 block checks */
#define NO_SCAN				// F_SCAN is not completed in avrio.c So don't use this.

//#define FN_MAX  16
#define IBUFLEN 180
#define OBUFLEN 180

#define P_PKTLEN 128
#define P_WSLOTS  4			/* Max is 4 Sliding Window Slots*/
#endif

#ifndef NODEBUG				// to build without debugging code.
#define NODEBUG
#endif /* NODEBUG */

#ifndef IBUFLEN				// be the desired size for the file input buffer.
#define IBUFLEN  4096		// File input buffer size
#endif /* IBUFLEN */

#ifndef OBUFLEN				// to be the desired size for the file output buffer.
#define OBUFLEN  4096       // File output buffer size
#endif /* OBUFLEN */

#ifndef FN_MAX 				// to be the maximum length for a filename.
#define FN_MAX   _MAX_LFN
#endif /* FN_MAX */

#ifndef	P_PKTLEN 			// to override the default maximum packet length.
#define P_PKTLEN 256
#endif /* P_PKTLEN */

#ifndef P_WSLOTS 			//to override the default maximum window slots.
#define P_WSLOTS 1
#endif /* P_WSLOTS */


#ifdef __cplusplus
extern "C" {
#endif

/*
  Sample prototypes for i/o functions.
  The functions are defined in a platform-specific i/o module.
  The function names are unknown to the Kermit module.
  The names can be changed but not their calling conventions.
  The following prototypes are keyed to avrio.c.
*/
/* Communications device/path */
int devopen(char *);
int pktmode(int);
int devsettings(char *);
int devrestore(void);
int devclose(void);

/* Communications i/o functions */
int readpkt(struct k_data *, UCHAR *, int);
int tx_data(struct k_data *, UCHAR *, int);
int inchk(struct k_data *);

/* File i/o functions */
int openfile(struct k_data *, UCHAR *, int);
int writefile(struct k_data *, UCHAR *, int);
int readfile(struct k_data *);
int closefile(struct k_data *, UCHAR, int);
ULONG fileinfo(struct k_data *, UCHAR *, UCHAR *, int, short *, short);

#ifdef __cplusplus
}
#endif

#endif // platform_h

