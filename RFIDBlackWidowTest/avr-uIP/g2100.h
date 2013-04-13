
/*****************************************************************************

  Filename:		g2100.h
  Description:	Driver for the ZeroG Wireless G2100 series devices

 *****************************************************************************

  Driver for the WiShield 1.0 wireless devices

  Copyright(c) 2009 Async Labs Inc. All rights reserved.

  This program is free software; you can redistribute it and/or modify it
  under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  this program; if not, write to the Free Software Foundation, Inc., 59
  Temple Place - Suite 330, Boston, MA  02111-1307, USA.

  Contact Information:
  <asynclabs@asynclabs.com>

   Author               Date        Comment
  ----------------------------------------------------------------------------
   AsyncLabs			02/25/2009	Initial port
   AsyncLabs			05/29/2009	Adding support for new library

 *****************************************************************************/

#ifndef G2100_H_
#define G2100_H_

// Uncomment one line below to
// specify which Arduino pin
// to use as WiShield interrupt
#define USE_DIG2_INTR  // Interrupt INT0   (PD2) on Arduino (also Pin DIG2)
//#define USE_DIG8_INTR  // Interrupt Pin D8 (PB0) on Arduino)

// Max characters in network SSID
#define ZG_MAX_SSID_LENGTH			32

// Security keys
#define ZG_MAX_ENCRYPTION_KEYS 		2
#define ZG_MAX_ENCRYPTION_KEY_SIZE	13
#define ZG_MAX_WPA_PASSPHRASE_LEN	64
#define ZG_MAX_PMK_LEN				32



#define DRV_STATE_INIT					0
#define DRV_STATE_GET_MAC				2
#define DRV_STATE_START_CONN			3
#define DRV_STATE_PROCESS_RX			4
#define DRV_STATE_IDLE					5
#define DRV_STATE_SETUP_SECURITY		6
#define DRV_STATE_INSTALL_PSK			7
#define DRV_STATE_ENABLE_CONN_MANAGE	8


// Command values which appear in ZG_PREAMBLE_CMD_IDX for each SPI message
#define ZG_CMD_FIFO_ACCESS			(0x80)
#define ZG_CMD_WT_FIFO_DATA			(ZG_CMD_FIFO_ACCESS | 0x20)
#define ZG_CMD_WT_FIFO_MGMT			(ZG_CMD_FIFO_ACCESS | 0x30)
#define ZG_CMD_RD_FIFO				(ZG_CMD_FIFO_ACCESS | 0x00)
#define ZG_CMD_WT_FIFO_DONE			(ZG_CMD_FIFO_ACCESS | 0x40)
#define ZG_CMD_RD_FIFO_DONE			(ZG_CMD_FIFO_ACCESS | 0x50)
#define ZG_CMD_WT_REG				(0x00)
#define ZG_CMD_RD_REG				(0x40)

// Type values which appear in ZG_PREAMBLE_TYPE_IDX for each SPI message
#define ZG_MAC_TYPE_TXDATA_REQ		((uint8_t)1)
#define ZG_MAC_TYPE_MGMT_REQ		((uint8_t)2)

#define ZG_MAC_TYPE_TXDATA_CONFIRM	((uint8_t)1)
#define ZG_MAC_TYPE_MGMT_CONFIRM	((uint8_t)2)
#define ZG_MAC_TYPE_RXDATA_INDICATE	((uint8_t)3)
#define ZG_MAC_TYPE_MGMT_INDICATE	((uint8_t)4)

// Subtype values which appear in ZG_PREAMBLE_SUBTYPE_IDX for each SPI message
// Subtype for ZG_MAC_TYPE_TXDATA_REQ and ZG_MAC_TYPE_TXDATA_CONFIRM
#define ZG_MAC_SUBTYPE_TXDATA_REQ_STD			((uint8_t)1)

// Subtype for ZG_MAC_TYPE_MGMT_REQ and ZG_MAC_TYPE_MGMT_CONFIRM
#define ZG_MAC_SUBTYPE_MGMT_SCAN                ((uint8_t)1)
#define ZG_MAC_SUBTYPE_MGMT_REQ_PMK_KEY			((uint8_t)8)
#define ZG_MAC_SUBTYPE_MGMT_REQ_WEP_KEY			((uint8_t)10)
#define ZG_MAC_SUBTYPE_MGMT_REQ_CALC_PSK		((uint8_t)12)
#define ZG_MAC_SUBTYPE_MGMT_REQ_SET_PARAM		((uint8_t)15)
#define ZG_MAC_SUBTYPE_MGMT_REQ_GET_PARAM		((uint8_t)16)
#define ZG_MAC_SUBTYPE_MGMT_REQ_ADHOC_START		((uint8_t)18)
#define ZG_MAC_SUBTYPE_MGMT_REQ_CONNECT			((uint8_t)19)
#define ZG_MAC_SUBTYPE_MGMT_REQ_CONNECT_MANAGE	((uint8_t)20)

// Subtype for ZG_MAC_TYPE_RXDATA_INDICATE
#define ZG_MAC_SUBTYPE_RXDATA_IND_STD			((uint8_t)1)

// Subtype for ZG_MAC_TYPE_MGMT_INDICATE
#define ZG_MAC_SUBTYPE_MGMT_IND_DISASSOC		((uint8_t)1)
#define ZG_MAC_SUBTYPE_MGMT_IND_DEAUTH			((uint8_t)2)
#define ZG_MAC_SUBTYPE_MGMT_IND_CONN_STATUS		((uint8_t)4)

// Parameter IDs for ZG_MAC_SUBTYPE_MGMT_REQ_SET_PARAM
#define ZG_PARAM_MAC_ADDRESS			(1)



// MAC result code
enum {
    ZG_RESULT_SUCCESS = 1,
    ZG_RESULT_INVALID_SUBTYPE,
    ZG_RESULT_CANCELLED,
    ZG_RESULT_FRAME_EOL,
    ZG_RESULT_FRAME_RETRY_LIMIT,
    ZG_RESULT_FRAME_NO_BSS,
    ZG_RESULT_FRAME_TOO_BIG,
    ZG_RESULT_FRAME_ENCRYPT_FAILURE,
    ZG_RESULT_INVALID_PARAMS,
    ZG_RESULT_ALREADY_AUTH,
    ZG_RESULT_ALREADY_ASSOC,
    ZG_RESULT_INSUFFICIENT_RSRCS,
    ZG_RESULT_TIMEOUT,
    ZG_RESULT_BAD_EXCHANGE,	// frame exchange problem with peer (AP or STA)
    ZG_RESULT_AUTH_REFUSED,		// authenticating node refused our request
    ZG_RESULT_ASSOC_REFUSED,	// associating node refused our request
    ZG_RESULT_REQ_IN_PROGRESS,	// only one mlme request at a time allowed
    ZG_RESULT_NOT_JOINED,			// operation requires that device be joined
    								// with target
    ZG_RESULT_NOT_ASSOC,			// operation requires that device be
    								// associated with target
    ZG_RESULT_NOT_AUTH,				// operation requires that device be
    								// authenticated with target
    ZG_RESULT_SUPPLICANT_FAILED,
    ZG_RESULT_UNSUPPORTED_FEATURE,
    ZG_RESULT_REQUEST_OUT_OF_SYNC	// Returned when a request is recognized
    								// but invalid given the current state
    								// of the MAC
};

/*
 * G2100 command registers
 */
#define ZG_INTR_REG					(0x01)	// 8-bit register containing interrupt bits
#define ZG_INTR_MASK_REG			(0x02)	// 8-bit register containing interrupt mask
#define ZG_SYS_INFO_DATA_REG		(0x21)	// 8-bit register to read system info data window
#define ZG_SYS_INFO_IDX_REG			(0x2b)
#define ZG_INTR2_REG				(0x2d)	// 16-bit register containing interrupt bits
#define ZG_INTR2_MASK_REG			(0x2e)	// 16-bit register containing interrupt mask
#define ZG_BYTE_COUNT_REG			(0x2f)	// 16-bit register containing available write size for fifo0
#define ZG_BYTE_COUNT_FIFO0_REG		(0x33)	// 16-bit register containing bytes ready to read on fifo0
#define ZG_BYTE_COUNT_FIFO1_REG		(0x35)	// 16-bit register containing bytes ready to read on fifo1
#define ZG_PWR_CTRL_REG				(0x3d)	// 16-bit register used to control low power mode
#define ZG_INDEX_ADDR_REG			(0x3e)	// 16-bit register to move the data window
#define ZG_INDEX_DATA_REG			(0x3f)	// 16-bit register to read the address in the ZG_INDEX_ADDR_REG

#define ZG_INTR_REG_LEN				(1)
#define ZG_INTR_MASK_REG_LEN		(1)
#define ZG_SYS_INFO_DATA_REG_LEN	(1)
#define ZG_SYS_INFO_IDX_REG_LEN		(2)
#define ZG_INTR2_REG_LEN			(2)
#define ZG_INTR2_MASK_REG_LEN		(2)
#define ZG_BYTE_COUNT_REG_LEN		(2)
#define ZG_BYTE_COUNT_FIFO0_REG_LEN	(2)
#define ZG_BYTE_COUNT_FIFO1_REG_LEN	(2)
#define ZG_PWR_CTRL_REG_LEN			(2)
#define ZG_INDEX_ADDR_REG_LEN		(2)
#define ZG_INDEX_DATA_REG_LEN		(2)

// Registers accessed through ZG_INDEX_ADDR_REG
#define ZG_RESET_STATUS_REG			(0x2a)	// 16-bit read only register providing HW status bits
#define ZG_RESET_REG				(0x2e)	// 16-bit register used to initiate hard reset
#define ZG_PWR_STATUS_REG			(0x3e)	// 16-bit register read to determine when device
											// out of sleep state

#define ZG_RESET_MASK				(0x10)	// the first byte of the ZG_RESET_STATUS_REG
											// used to determine when the G2100 is in reset

#define ZG_ENABLE_LOW_PWR_MASK		(0x01)	// used by the Host to enable/disable sleep state
											// indicates to G2100 that the Host has completed
											// transactions and the device can go into sleep
											// state if possible



#ifdef USE_DIG2_INTR			// This is PD2 or the INT0 pin on Arduino
#define ZG2100_ISR_DISABLE()	(EIMSK &= ~_BV(INT0));
#define ZG2100_ISR_ENABLE()		(EIMSK |=  _BV(INT0));
#endif

#ifdef USE_DIG8_INTR			// This is PB0 or the PCINT0 pin on the Arduino
#define ZG2100_ISR_DISABLE()	PCICR &= ~_BV(PCIE0);	PCMSK0 &= ~_BV(PCINT0);
#define ZG2100_ISR_ENABLE()		PCICR |=  _BV(PCIE0);	PCMSK0 |=  _BV(PCINT0);
#endif

// states for interrupt state machine
#define ZG_INTR_ST_RD_INTR_REG	(1)
#define ZG_INTR_ST_WT_INTR_REG	(2)
#define ZG_INTR_ST_RD_CTRL_REG	(3)

// mask values for ZG_INTR_REG and ZG_INTR2_REG
#define	ZG_INTR_MASK_FIFO1		(0x80)
#define ZG_INTR_MASK_FIFO0		(0x40)
#define ZG_INTR_MASK_ALL		(0xff)
#define ZG_INTR2_MASK_ALL		(0xffff)


// Types of networks
#define ZG_BSS_INFRA			(1)    // infrastructure only
#define ZG_BSS_ADHOC			(2)    // Ad-hoc only (ibss)

#define ZG_SECURITY_TYPE_NONE	0
#define ZG_SECURITY_TYPE_WEP	1
#define ZG_SECURITY_TYPE_WPA	2
#define ZG_SECURITY_TYPE_WPA2	3

#define ZG_CONNECT_REQ_SIZE		(38)

// Blue Signal Connected LED
#define LEDConn_on()	(DDRB |=  _BV(DDB1)); (PORTB |=  _BV(PORTB1));
#define LEDConn_off()	                      (PORTB &= ~_BV(PORTB1));

//Host to Zero G long
#define HTOZGL(a) (	 ((a & 0x000000ff)<<24) \
				|((a & 0x0000ff00)<<8)  \
				|((a & 0x00ff0000)>>8)  \
				|((a & 0xff000000)>>24)	)
#define ZGTOHL(a) HTOZGL(a)

// Host to Zero G short
#define HSTOZGS(a) (uint16_t)(((a)<<8) | ((a)>>8))
#define ZGSTOHS(a) HSTOZGS(a)


typedef struct
{
    uint8_t slot;	/* slot index */
    uint8_t keyLen;
    uint8_t defID;	/* the default wep key id */
    uint8_t ssidLen;	/* num valid bytes in ssid */
    uint8_t ssid[ZG_MAX_SSID_LENGTH];	/* ssid of network */
    uint8_t key[ZG_MAX_ENCRYPTION_KEYS][ZG_MAX_ENCRYPTION_KEY_SIZE];	/* wep key data for 4 default keys */
} zg_wep_key_req_t;

#define ZG_WEP_KEY_REQ_SIZE		(4 + ZG_MAX_SSID_LENGTH + ZG_MAX_ENCRYPTION_KEYS*ZG_MAX_ENCRYPTION_KEY_SIZE)

typedef struct
{
    uint8_t configBits;
    uint8_t phraseLen;	/* number of valid bytes in passphrase */
    uint8_t ssidLen;		/* number of valid bytes in ssid */
    uint8_t reserved;	/* alignment byte */
    uint8_t ssid[ZG_MAX_SSID_LENGTH];	/* the string of characters representing the ssid */
    uint8_t passPhrase[ZG_MAX_WPA_PASSPHRASE_LEN]; /* the string of characters representing the passphrase */
} zg_psk_calc_req_t;

#define ZG_PSK_CALC_REQ_SIZE	(4 + ZG_MAX_SSID_LENGTH + ZG_MAX_WPA_PASSPHRASE_LEN) /* 100 bytes */

typedef struct
{
    uint8_t result;		/* indicating success or other */
    uint8_t macState;	/* current State of the on-chip MAC */
    uint8_t keyReturned;	/* 1 if psk contains key data, 0 otherwise */
    uint8_t reserved;	/* pad byte */
    uint8_t psk[ZG_MAX_PMK_LEN];	/* the psk bytes */
} zg_psk_calc_cnf_t;

typedef struct
{
    uint8_t slot;
    uint8_t ssidLen;
    uint8_t ssid[ZG_MAX_SSID_LENGTH];
    uint8_t keyData[ZG_MAX_PMK_LEN];
} zg_pmk_key_req_t;

#define ZG_PMK_KEY_REQ_SIZE		(2 + ZG_MAX_SSID_LENGTH + ZG_MAX_PMK_LEN)

typedef struct
{
    uint16_t        rssi;                      /* the value of the G1000 RSSI when the data frame was received */
    uint8_t         dstAddr[6];    /* MAC Address to which the data frame was directed. */
    uint8_t         srcAddr[6];    /* MAC Address of the Station that sent the Data frame. */
    uint16_t        arrivalTime_th;               /* the value of the 32-bit G1000 system clock when the frame arrived */
    uint16_t        arrivalTime_bh;
    uint16_t        dataLen;                   /* the length in bytes of the payload which immediately follows this data structure */
} zg_rx_data_ind_t;

typedef struct
{
	uint8_t secType;		/* security type : 0 - none; 1 - wep; 2 - wpa; 3 - wpa2; 0xff - best available */
    uint8_t ssidLen;		/* num valid bytes in ssid */
    uint8_t ssid[ZG_MAX_SSID_LENGTH];	/* the ssid of the target */
    uint16_t sleepDuration;	/* power save sleep duration in units of 100 milliseconds */
    uint8_t modeBss;			/* 1 - infra; 2 - adhoc */
    uint8_t reserved;
} zg_connect_req_t;

// =================================================================================================

#ifdef UIP_SCAN

/* completely describes the details of a BSS (access point) */
/* This structure is used as a single entry in a Scan result */
typedef struct
{
    uint8_t        bssid[6]; /* network BSSID value */
    uint8_t        ssid[32];   /* network SSID value */
    uint8_t        capInfo[2];            /* capability info bits */
    uint16_t       beaconPeriod;          /* network Beacon interval */
    uint16_t       atimWindow; /* only valid if .bssType == Ibss */
    uint8_t        basicRateSet[8]; /* list of network basic rates */
    uint8_t        rssi;            /* signal strength of rx'd
                                  * frame beacon, probe resp */
    uint8_t        numRates;        /* num entries in basicRateSet */
    uint8_t        DtimPeriod;      /* (part of TIM element) */
    uint8_t        bssType;           /* ad-hoc or infrastructure. */
    uint8_t        channel;        /* phy param set */
    uint8_t        ssidLen;       /* number of bytes in ssid */
} tZGBssDesc; /* 58 bytes */

typedef tZGBssDesc *tZGBssDescPtr;

/* use #define size to represent the number of bytes in the structure
 * because not all compilers have an appropriate calculation when using sizeof() */
#define kZGBssDescSZ (58)

/* Represents the input parameters required to
 * conduct a Scan operation */
typedef struct
{
    uint16_t         probeDelay;           /* the number of usec to delay before transmitting a probe
                                          * request following the channel change event */
    uint16_t         minChannelTime;       /* the minimum time to spend on each channel in units
                                          * of TU (1024 usec) */
    uint16_t         maxChannelTime;       /* the maximum time to spend on each channel in units
                                          * of TU (1024 usec) */
    uint8_t          bssid[6];             /* limits the scan to a specific Bss or Broadcast
                                          * (FF:FF:FF:FF:FF:FF) */
    uint8_t          bss;                  /* limits the type of networks to include in scan results.
                                          * Can be one of; kBssInfra kBssAdHoc kBssAny*/
    uint8_t          snType;               /* one of kZGMACSnTypeActive or kZGMACSnTypePassive where
                                          * active indicates the use of Probe Request frames */
    uint8_t          ssidLen;              /* num chars in ssid */
    uint8_t          chnlLen;              /* num channels to scan */
    uint8_t          ssid[32];  /* limits the scan to a specific Service Set (SSID) or
                                         // * broadcast ("\0") */
    uint8_t          channelList[14]; /* zero terminated list of channels
                                                   * to scan */
} tZGScanReq; /* 62 bytes */

typedef tZGScanReq *tZGScanReqPtr;

/* use #define size to represent the number of bytes in the structure
 * because not all compilers have an appropriate calculation when using sizeof() */
#define kZGScanReqSZ    (62)

/* The structure used to return the scan results back to the host system */
typedef struct
{
    uint8_t    result;        /* the result of the scan */
    uint8_t    macState;      /* current state of the on-chip MAC */
    uint8_t    lastChnl;      /* the last channel scanned */
    uint8_t    numBssDesc;    /* The number of tZGMACBssDesc objects that
                             * immediately follows this structure in memory */
} tZGScanResult; /* 4 bytes */

typedef tZGScanResult *tZGScanResultPtr;

#define kZGScanResultSZ    (4)

#endif // UIP_SCAN

// =================================================================================================


void zg_init();
void zg_reset();
void spi_transfer(uint8_t* buf, uint16_t len, uint8_t toggle_cs);
void zg_chip_reset();
void zg_interrupt2_reg();
void zg_interrupt_reg(uint8_t mask, uint8_t state);
void zg_isr();
void zg_process_isr();
void zg_send(uint8_t* buf, uint16_t len);
void zg_recv(uint8_t* buf, uint16_t* len);
uint16_t zg_get_rx_status();
void zg_clear_rx_status();
void zg_set_tx_status(uint8_t status);
uint8_t zg_get_conn_state();
void zg_set_buf(uint8_t* buf, uint16_t buf_len);
uint8_t* zg_get_mac();
void zg_set_ssid(uint8_t* ssid, uint8_t ssid_len);
void zg_set_sec(uint8_t sec_type, uint8_t* sec_key, uint8_t sec_key_len);
void zg_drv_process();

// User Contributed ================================================================================
uint16_t zg_get_rssi();

#ifdef UIP_SCAN
void zg_scan_start();
tZGScanResult* zg_scan_results();
tZGBssDesc* zg_scan_desc(uint8_t item);
uint16_t get_scan_cnt();
#endif // UIP_SCAN

#endif /* G2100_H_ */
