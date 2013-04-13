
/*****************************************************************************

  Filename:		g2100.c
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

#include <string.h>
#include <avr/pgmspace.h>

/* Scheduler includes. */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include <spi.h>

#include "uip.h"
#include "config.h"
#include "g2100.h"

static uint8_t mac[6];
static uint8_t zg_conn_status;

static uint8_t hdr[5];
static uint8_t intr_occured;
static uint8_t intr_valid;
static uint8_t zg_drv_state;
static uint8_t tx_ready;
static uint8_t rx_ready;
static uint8_t cnf_pending;
static uint8_t* zg_buf;
static uint16_t zg_buf_len;
static uint16_t lastRssi;
static uint8_t scan_cnt;
static uint8_t wpa_psk_key[32];

extern xSemaphoreHandle xZGIntrSemaphore;

void zg_init()
{
	uint8_t clr;

	spiBegin(SS_PB2);
	spiSetClockDivider(SPI_CLOCK_DIV2);
	clr = SPSR;
	clr = SPDR;

	intr_occured = 0;
	intr_valid = 0;
	lastRssi = 0;
	zg_drv_state = DRV_STATE_INIT;
	zg_conn_status = 0;
	tx_ready = 0;
	rx_ready = 0;
	cnf_pending = 0;
	zg_buf = uip_buf;
	zg_buf_len = UIP_BUFSIZE;

	zg_chip_reset();
	zg_interrupt2_reg();
	zg_interrupt_reg(0xff, 0);
	zg_interrupt_reg(0x80|0x40, 1);

	ssid_len = (uint8_t)strlen((char *)ssid);
	security_passphrase_len = (uint8_t)strlen_P((PGM_P)security_passphrase);

	/* Create the semaphore used by the ISR to indicate that a Rx frame is ready
	for processing. */
    vSemaphoreCreateBinary( xZGIntrSemaphore );

}


void spi_transfer(uint8_t *buf, uint16_t len, uint8_t toggle_cs)
{

	spiSelect (SS_PB2);

	spiMultiByteTransfer(buf, len);

	if (toggle_cs)
		spiDeselect (SS_PB2);

	return;
}

void zg_chip_reset()
{
	uint8_t loop_cnt = 0;


	do {
		// write reset register addr
		hdr[0] = ZG_INDEX_ADDR_REG;
		hdr[1] = 0x00;
		hdr[2] = ZG_RESET_REG;
		spi_transfer(hdr, 3, 1);

		hdr[0] = ZG_INDEX_DATA_REG;
		hdr[1] = (loop_cnt == 0)?(0x80):(0x0f);
		hdr[2] = 0xff;
		spi_transfer(hdr, 3, 1);
	} while(loop_cnt++ < 1);

	// write reset register data
	hdr[0] = ZG_INDEX_ADDR_REG;
	hdr[1] = 0x00;
	hdr[2] = ZG_RESET_STATUS_REG;
	spi_transfer(hdr, 3, 1);

	do {
		hdr[0] = 0x40 | ZG_INDEX_DATA_REG;
		hdr[1] = 0x00;
		hdr[2] = 0x00;
		spi_transfer(hdr, 3, 1);
	} while((hdr[1] & ZG_RESET_MASK) == 0);

	do {
		hdr[0] = 0x40 | ZG_BYTE_COUNT_REG;
		hdr[1] = 0x00;
		hdr[2] = 0x00;
		spi_transfer(hdr, 3, 1);
	} while((hdr[1] == 0) && (hdr[2] == 0));

}

void zg_interrupt2_reg()
{
	// read the interrupt2 mask register
	hdr[0] = 0x40 | ZG_INTR2_MASK_REG;
	hdr[1] = 0x00;
	hdr[2] = 0x00;
	spi_transfer(hdr, 3, 1);

	// modify the interrupt mask value and re-write the value to the interrupt
	// mask register clearing the interrupt register first
	hdr[0] = ZG_INTR2_REG;
	hdr[1] = 0xff;
	hdr[2] = 0xff;
	hdr[3] = 0;
	hdr[4] = 0;
	spi_transfer(hdr, 5, 1);

	return;
}

void zg_interrupt_reg(uint8_t mask, uint8_t state)
{
	// read the interrupt register
	hdr[0] = 0x40 | ZG_INTR_MASK_REG;
	hdr[1] = 0x00;
	spi_transfer(hdr, 2, 1);

	// now regBuf[0] contains the current setting for the
	// interrupt mask register
	// this is to clear any currently set interrupts of interest
	hdr[0] = ZG_INTR_REG;
	hdr[2] = (hdr[1] & ~mask) | ( (state == 0)? 0 : mask );
	hdr[1] = mask;
	spi_transfer(hdr, 3, 1);

	return;
}

void zg_isr()
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	ZG2100_ISR_DISABLE();
	intr_occured = 1;

	xSemaphoreGiveFromISR( xZGIntrSemaphore, &xHigherPriorityTaskWoken );
}

void zg_process_isr()
{
	uint8_t intr_state = 0;
	uint8_t next_cmd = 0;

	hdr[0] = 0x40 | ZG_INTR_REG;
	hdr[1] = 0x00;
	hdr[2] = 0x00;
	spi_transfer(hdr, 3, 1);

	intr_state = ZG_INTR_ST_RD_INTR_REG;

	do {
		switch(intr_state) {
			case ZG_INTR_ST_RD_INTR_REG:
			{
				uint8_t intr_val = hdr[1] & hdr[2];

				if ( (intr_val & ZG_INTR_MASK_FIFO1) == ZG_INTR_MASK_FIFO1) {
					hdr[0] = ZG_INTR_REG;
					hdr[1] = ZG_INTR_MASK_FIFO1;
					spi_transfer(hdr, 2, 1);

					intr_state = ZG_INTR_ST_WT_INTR_REG;
					next_cmd = ZG_BYTE_COUNT_FIFO1_REG;
				}
				else if ( (intr_val & ZG_INTR_MASK_FIFO0) == ZG_INTR_MASK_FIFO0) {
					hdr[0] = ZG_INTR_REG;
					hdr[1] = ZG_INTR_MASK_FIFO0;
					spi_transfer(hdr, 2, 1);

					intr_state = ZG_INTR_ST_WT_INTR_REG;
					next_cmd = ZG_BYTE_COUNT_FIFO0_REG;
				}
				else if (intr_val) {
					intr_state = 0;
				}
				else {
					intr_state = 0;
				}

				break;
			}
			case ZG_INTR_ST_WT_INTR_REG:
			{
				hdr[0] = 0x40 | next_cmd;
				hdr[1] = 0x00;
				hdr[2] = 0x00;
				spi_transfer(hdr, 3, 1);

				intr_state = ZG_INTR_ST_RD_CTRL_REG;
				break;
			}
			// *************************************************************************************
			// Released version of WiShield library code (ZG_INTR_ST_RD_CTRL_REG)
			// susceptible to "packet too large issue" but allows
			// WiServer/WebServer to work.  Needs to be replaced with
			// good fix.


			case ZG_INTR_ST_RD_CTRL_REG:
			{
				taskENTER_CRITICAL();

				// Get the size of the incoming packet
				uint16_t rx_byte_cnt = (0x0000 | (hdr[1] << 8) | hdr[2]) & 0x0fff;

				// xxx Cap the size of the packet at the maximum buffer, to prevent overflow.
				if( (rx_byte_cnt + 2) > (uint16_t)UIP_BUFSIZE )
					rx_byte_cnt = (uint16_t) UIP_BUFSIZE - 1;

				zg_buf[0] = ZG_CMD_RD_FIFO;
				spi_transfer(zg_buf, rx_byte_cnt + 1, 1);

				// Tell ZG2100 we're done reading from its buffer
				hdr[0] = ZG_CMD_RD_FIFO_DONE;
				spi_transfer(hdr, 1, 1);

				taskEXIT_CRITICAL();

				intr_valid = 1;

				// Done reading interrupt from ZG2100
				intr_state = 0;

				break;
			}

			// *************************************************************************************
		}
	} while (intr_state);
#if defined ( USE_DIG8_INTR )
	// PCINT0 supports only edge triggered INT
	if (PORTB & _BV(PORTB0)) {
		intr_occured = 0;
		ZG2100_ISR_ENABLE();
	}
	else {
		intr_occured = 1;
	}
#elif defined (USE_DIG2_INTR)
	intr_occured = 0;
	ZG2100_ISR_ENABLE();
#endif
}

void zg_send(uint8_t* buf, uint16_t len)
{
//	taskENTER_CRITICAL();
	hdr[0] = ZG_CMD_WT_FIFO_DATA;
	hdr[1] = ZG_MAC_TYPE_TXDATA_REQ;
	hdr[2] = ZG_MAC_SUBTYPE_TXDATA_REQ_STD;
	hdr[3] = 0x00;
	hdr[4] = 0x00;
	spi_transfer(hdr, 5, 0);

	buf[6] = 0xaa;
	buf[7] = 0xaa;
	buf[8] = 0x03;
	buf[9] = buf[10] = buf[11] = 0x00;
	spi_transfer(buf, len, 1);

	hdr[0] = ZG_CMD_WT_FIFO_DONE;
	spi_transfer(hdr, 1, 1);
//	taskEXIT_CRITICAL();
}

void zg_recv(uint8_t* buf, uint16_t* len)
{
	zg_rx_data_ind_t* ptr = (zg_rx_data_ind_t*)&(zg_buf[3]);
	*len = ZGSTOHS( ptr->dataLen );
	lastRssi = ZGSTOHS( ptr->rssi );

	memcpy(&zg_buf[0], &zg_buf[5], 6);
	memcpy(&zg_buf[6], &zg_buf[11], 6);
	memcpy(&zg_buf[12], &zg_buf[29], *len);

	*len += 12;
}

uint16_t zg_get_rx_status()
{
	if (rx_ready) {
		rx_ready = 0;
		return zg_buf_len;
	}
	else {
		return 0;
	}
}

void zg_clear_rx_status()
{
	rx_ready = 0;
}

void zg_set_tx_status(uint8_t status)
{
	tx_ready = status;
}

uint8_t zg_get_conn_state()
{
	return zg_conn_status;
}

void zg_set_buf(uint8_t* buf, uint16_t buf_len)
{
	zg_buf = buf;
	zg_buf_len = buf_len;
}

uint8_t* zg_get_mac()
{
	return mac;
}

void zg_write_wep_key(uint8_t* cmd_buf)
{
	zg_wep_key_req_t* cmd = (zg_wep_key_req_t*)cmd_buf;

	cmd->slot = 3;                    // WEP key slot
	cmd->keyLen = UIP_WEP_KEY_LEN;    // Key length: 5 bytes (64-bit WEP); 13 bytes (128-bit WEP)
	cmd->defID = UIP_WEP_KEY_DEFAULT; // Default key ID: Key 0, 1, 2, 3
	cmd->ssidLen = ssid_len;
	memset(cmd->ssid, 0x00, 32);
	memcpy(cmd->ssid, ssid, ssid_len);
	memcpy_P(cmd->key, wep_keys, ZG_MAX_ENCRYPTION_KEYS * ZG_MAX_ENCRYPTION_KEY_SIZE);

	return;
}

static void zg_calc_psk_key(uint8_t* cmd_buf)
{
	zg_psk_calc_req_t* cmd = (zg_psk_calc_req_t*)cmd_buf;

	cmd->configBits = 0;
	cmd->phraseLen = security_passphrase_len;
	cmd->ssidLen = ssid_len;
	cmd->reserved = 0;
	memset(cmd->ssid, 0x00, 32);
	memcpy(cmd->ssid, ssid, ssid_len);
	memset(cmd->passPhrase, 0x00, 64);
	memcpy_P(cmd->passPhrase, security_passphrase, security_passphrase_len);

	return;
}

static void zg_write_psk_key(uint8_t* cmd_buf)
{
	zg_pmk_key_req_t* cmd = (zg_pmk_key_req_t*)cmd_buf;

	cmd->slot = 0;	// WPA/WPA2 PSK slot
	cmd->ssidLen = ssid_len;
	memset(cmd->ssid, 0x00, 32);
	memcpy(cmd->ssid, ssid, cmd->ssidLen);
	memcpy(cmd->keyData, wpa_psk_key, ZG_MAX_PMK_LEN);

	return;
}

uint16_t zg_get_rssi(){
     return lastRssi;
}

// =================================================================================================
#ifdef UIP_SCAN

tZGScanResult* zg_scan_results(void)
{
    return &uip_buf[3];
}

tZGBssDesc* zg_scan_desc(uint8_t item)
{
	return &uip_buf[7 + (item * sizeof(tZGBssDesc))];
}

uint16_t get_scan_cnt(){
     return scan_cnt;
}

void zg_scan_start()
{
	scan_cnt = 0;
    tZGScanReq* ptr = (tZGScanReq*)&zg_buf[3];

    zg_buf[0] = ZG_CMD_WT_FIFO_MGMT;
    zg_buf[1] = ZG_MAC_TYPE_MGMT_REQ;
    zg_buf[2] = ZG_MAC_SUBTYPE_MGMT_SCAN;

    // Number of usec to delay before transmitting a probe
    // request following the channel change event
    ptr->probeDelay = ( uint16_t ) HSTOZGS( ( uint16_t ) 20);
    // the minimum time to spend on each channel in units
    // of TU (1024 usec)
    ptr->minChannelTime = ( uint16_t ) HSTOZGS(( uint16_t ) 400);
    // Maximum time to spend on each channel in units
    // of TU (1024 usec)
    ptr->maxChannelTime = ( uint16_t ) HSTOZGS( ( uint16_t ) 800);
    // Bssid to restrict the scan too. Or ff:ff:ff:ff:ff:ff
    // to not restrict the scan by bssid
    ptr->bssid[0] = 0xFF;
    ptr->bssid[1] = 0xFF;
    ptr->bssid[2] = 0xFF;
    ptr->bssid[3] = 0xFF;
    ptr->bssid[4] = 0xFF;
    ptr->bssid[5] = 0xFF;
    // Type of networks to be scanned. 1==Infrastructure, 2==Ad-hoc, 3==Any
    ptr->bss = 3;
    // Scan Type. 1==Active (probe requests), 2==Passive (just listens)
    ptr->snType = 1;
    // set ssid length to 0 to do discovery scan
    ptr->ssidLen = 0;
    // State based scan only scans a single channel at a time
    ptr->chnlLen = 11;
    // Zero-terminated list of channels to scan
    //ptr->channelList[0] = 3;
    //ptr->channelList[1] = 6;
    //ptr->channelList[2] = 11;
    //ptr->channelList[3] = 0;
    uint8_t b;
    for ( b=0; b<11; b++ ){
        ptr->channelList[b] = b+1;
    }
    ptr->channelList[13] = 0;
    // Actually send command to zg2100 over SPI
    spi_transfer( zg_buf, 65, 1 );

    zg_buf[0] = ZG_CMD_WT_FIFO_DONE;
    spi_transfer(zg_buf, 1, 1);

    //zg_drv_state = DRV_STATE_IDLE;
}

#endif // UIP_SCAN
// =================================================================================================

void zg_drv_process()
{
//	taskENTER_CRITICAL();

	// TX frame
	if (tx_ready && !cnf_pending) {
		zg_send(zg_buf, zg_buf_len);
		tx_ready = 0;
		cnf_pending = 1;
	}

	// process interrupt
	if (intr_occured)
	{
		zg_process_isr();
	}

	if (intr_valid) {
		switch (zg_buf[1]) {
		case ZG_MAC_TYPE_TXDATA_CONFIRM:
			cnf_pending = 0;
			break;
		case ZG_MAC_TYPE_MGMT_CONFIRM:
			if (zg_buf[3] == ZG_RESULT_SUCCESS) {
				switch (zg_buf[2]) {
                case ZG_MAC_SUBTYPE_MGMT_SCAN:
                    scan_cnt = 1;
                    break;
				case ZG_MAC_SUBTYPE_MGMT_REQ_GET_PARAM:
					mac[0] = zg_buf[7];
					mac[1] = zg_buf[8];
					mac[2] = zg_buf[9];
					mac[3] = zg_buf[10];
					mac[4] = zg_buf[11];
					mac[5] = zg_buf[12];
					zg_drv_state = DRV_STATE_SETUP_SECURITY;
					break;
				case ZG_MAC_SUBTYPE_MGMT_REQ_WEP_KEY:
					zg_drv_state = DRV_STATE_ENABLE_CONN_MANAGE;
					break;
				case ZG_MAC_SUBTYPE_MGMT_REQ_CALC_PSK:
					memcpy(wpa_psk_key, ((zg_psk_calc_cnf_t*)&zg_buf[3])->psk, 32);
					zg_drv_state = DRV_STATE_INSTALL_PSK;
					break;
				case ZG_MAC_SUBTYPE_MGMT_REQ_PMK_KEY:
					zg_drv_state = DRV_STATE_ENABLE_CONN_MANAGE;
					break;
				case ZG_MAC_SUBTYPE_MGMT_REQ_CONNECT_MANAGE:
					zg_drv_state = DRV_STATE_START_CONN;
					break;
				case ZG_MAC_SUBTYPE_MGMT_REQ_CONNECT:
					LEDConn_on();
					zg_conn_status = 1;	// connected
					break;
				default:
					break;
				}
			}
			break;
		case ZG_MAC_TYPE_RXDATA_INDICATE:
			zg_drv_state = DRV_STATE_PROCESS_RX;
			break;
		case ZG_MAC_TYPE_MGMT_INDICATE:
			switch (zg_buf[2]) {
			case ZG_MAC_SUBTYPE_MGMT_IND_DISASSOC:
			case ZG_MAC_SUBTYPE_MGMT_IND_DEAUTH:
				LEDConn_off();
				zg_conn_status = 0;	// lost connection

				//try to reconnect
				zg_drv_state = DRV_STATE_START_CONN;
				break;
			case ZG_MAC_SUBTYPE_MGMT_IND_CONN_STATUS:
				{
					uint16_t status = (((uint16_t)(zg_buf[3]))<<8)|zg_buf[4];

					if (status == 1 || status == 5) {
						LEDConn_off();
						zg_conn_status = 0;	// not connected
					}
					else if (status == 2 || status == 6) {
						LEDConn_on();
						zg_conn_status = 1;	// connected
					}
				}
				break;
			}
			break;
		}

		intr_valid = 0;
	}

	switch (zg_drv_state) {
	case DRV_STATE_INIT:
		zg_drv_state = DRV_STATE_GET_MAC;
		break;
	case DRV_STATE_GET_MAC:
		// get MAC address
		zg_buf[0] = ZG_CMD_WT_FIFO_MGMT;
		zg_buf[1] = ZG_MAC_TYPE_MGMT_REQ;
		zg_buf[2] = ZG_MAC_SUBTYPE_MGMT_REQ_GET_PARAM;
		zg_buf[3] = 0;
		zg_buf[4] = ZG_PARAM_MAC_ADDRESS;
		spi_transfer(zg_buf, 5, 1);

		zg_buf[0] = ZG_CMD_WT_FIFO_DONE;
		spi_transfer(zg_buf, 1, 1);

		zg_drv_state = DRV_STATE_IDLE;
		break;
	case DRV_STATE_SETUP_SECURITY:
		switch (security_type) {
		case ZG_SECURITY_TYPE_NONE:
			zg_drv_state = DRV_STATE_ENABLE_CONN_MANAGE;
			break;
		case ZG_SECURITY_TYPE_WEP:
			// Install all four WEP keys on G2100
			zg_buf[0] = ZG_CMD_WT_FIFO_MGMT;
			zg_buf[1] = ZG_MAC_TYPE_MGMT_REQ;
			zg_buf[2] = ZG_MAC_SUBTYPE_MGMT_REQ_WEP_KEY;
			zg_write_wep_key(&zg_buf[3]);
			spi_transfer(zg_buf, ZG_WEP_KEY_REQ_SIZE+3, 1);

			zg_buf[0] = ZG_CMD_WT_FIFO_DONE;
			spi_transfer(zg_buf, 1, 1);

			zg_drv_state = DRV_STATE_IDLE;
			break;
		case ZG_SECURITY_TYPE_WPA:
		case ZG_SECURITY_TYPE_WPA2:
			// Initiate PSK calculation on G2100
			zg_buf[0] = ZG_CMD_WT_FIFO_MGMT;
			zg_buf[1] = ZG_MAC_TYPE_MGMT_REQ;
			zg_buf[2] = ZG_MAC_SUBTYPE_MGMT_REQ_CALC_PSK;
			zg_calc_psk_key(&zg_buf[3]);
			spi_transfer(zg_buf, ZG_PSK_CALC_REQ_SIZE+3, 1);

			zg_buf[0] = ZG_CMD_WT_FIFO_DONE;
			spi_transfer(zg_buf, 1, 1);

			zg_drv_state = DRV_STATE_IDLE;
			break;
		default:
			break;
		}
		break;
	case DRV_STATE_INSTALL_PSK:
		// Install the PSK key on G2100
		zg_buf[0] = ZG_CMD_WT_FIFO_MGMT;
		zg_buf[1] = ZG_MAC_TYPE_MGMT_REQ;
		zg_buf[2] = ZG_MAC_SUBTYPE_MGMT_REQ_PMK_KEY;
		zg_write_psk_key(&zg_buf[3]);
		spi_transfer(zg_buf, ZG_PMK_KEY_REQ_SIZE+3, 1);

		zg_buf[0] = ZG_CMD_WT_FIFO_DONE;
		spi_transfer(zg_buf, 1, 1);

		zg_drv_state = DRV_STATE_IDLE;
		break;
	case DRV_STATE_ENABLE_CONN_MANAGE:
		// enable connection manager
		zg_buf[0] = ZG_CMD_WT_FIFO_MGMT;
		zg_buf[1] = ZG_MAC_TYPE_MGMT_REQ;
		zg_buf[2] = ZG_MAC_SUBTYPE_MGMT_REQ_CONNECT_MANAGE;
		zg_buf[3] = 0x01;	// 0x01 - enable; 0x00 - disable
		zg_buf[4] = 10;		// num retries to reconnect
		zg_buf[5] = 0x10 | 0x02 | 0x01;	// 0x10 -	enable start and stop indication messages
										// 		 	from G2100 during reconnection
										// 0x02 -	start reconnection on receiving a deauthentication
										// 			message from the AP
										// 0x01 -	start reconnection when the missed beacon count
										// 			exceeds the threshold. uses default value of
										//			100 missed beacons if not set during initialization
		zg_buf[6] = 0;
		spi_transfer(zg_buf, 7, 1);

		zg_buf[0] = ZG_CMD_WT_FIFO_DONE;
		spi_transfer(zg_buf, 1, 1);

		zg_drv_state = DRV_STATE_IDLE;
		break;
	case DRV_STATE_START_CONN:
	{
		zg_connect_req_t* cmd = (zg_connect_req_t*)&zg_buf[3];

		// start connection to AP
		zg_buf[0] = ZG_CMD_WT_FIFO_MGMT;
		zg_buf[1] = ZG_MAC_TYPE_MGMT_REQ;
		zg_buf[2] = ZG_MAC_SUBTYPE_MGMT_REQ_CONNECT;

		cmd->secType = security_type;

		cmd->ssidLen = ssid_len;
		memset(cmd->ssid, 0, 32);
		memcpy(cmd->ssid, ssid, ssid_len);

		// units of 100 milliseconds
		cmd->sleepDuration = 0;

		if (wireless_mode == WIRELESS_MODE_INFRA)
			cmd->modeBss = 1;
		else if (wireless_mode == WIRELESS_MODE_ADHOC)
			cmd->modeBss = 2;

		spi_transfer(zg_buf, ZG_CONNECT_REQ_SIZE+3, 1);

		zg_buf[0] = ZG_CMD_WT_FIFO_DONE;
		spi_transfer(zg_buf, 1, 1);

		zg_drv_state = DRV_STATE_IDLE;
		break;
	}
	case DRV_STATE_PROCESS_RX:
		zg_recv(zg_buf, &zg_buf_len);
		rx_ready = 1;

		zg_drv_state = DRV_STATE_IDLE;
		break;
	case DRV_STATE_IDLE:
		break;
	}
//	taskEXIT_CRITICAL();
}
