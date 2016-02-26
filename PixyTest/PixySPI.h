//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//

/*
  PixySPI.h - Library for interfacing with Pixy over SPI
  Created by Scott Robinson, October 22, 2013.
  Released into the public domain.

  Modified to work with AVRfreeRTOS, by Phillip Stevens @fei_li_pu
*/

#ifndef _PIXYSPI_H
#define _PIXYSPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "spi.h"

#define PIXY_SYNC_BYTE              0x5a
#define PIXY_SYNC_BYTE_DATA         0x5b
#define PIXY_OUTBUF_SIZE            6


static uint8_t outBuf[PIXY_OUTBUF_SIZE];
static uint8_t outLen;
static uint8_t outIndex;



  void pixy_spiInit(uint8_t addr) __attribute__ ((flatten))
  {
    spiSetClockDivider(SPI_CLOCK_DIV16);
    spiBegin(Default);
  }

  void pixy_spiEnd(void) __attribute__ ((flatten))
  {
	  spiEnd();
  }

  uint16_t pixi_getWord(void)
  {
    // ordering is different because Pixy is sending 16 bits through SPI
	// instead of 2 bytes in a 16-bit word as with I2C
    uint16_t w;
	uint8_t c, cout = 0;

	if (outLen)
	{
		w = spiTransfer(PIXY_SYNC_BYTE_DATA);
		cout = outBuf[outIndex++];
		if (outIndex==outLen)
			outLen = 0;
	}
	else
      w = spiTransfer(PIXY_SYNC_BYTE);
    w <<= 8;
	c = spiTransfer(cout);
	w |= c;

    return w;
  }

  uint8_t pixi_getByte(void) __attribute__ ((flatten))
  {
	return spiTransfer(0x00);
  }

  int8_t pixi_putByte(uint8_t *data, uint8_t len)
  {
	if (len>PIXY_OUTBUF_SIZE || outLen!=0)
		return -1;
	memcpy(outBuf, data, len);
	outLen = len;
	outIndex = 0;
	return len;
  }

#ifdef __cplusplus
}
#endif

#endif // _PIXYSPI_H
