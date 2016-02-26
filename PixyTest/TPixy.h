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

// Modified to work with AVRfreeRTOS, by Phillip Stevens @fei_li_pu

#ifndef _TPIXY_H
#define _TPIXY_H


#ifdef __cplusplus
extern "C" {
#endif

#include "spi.h"
#include "PixySPI.h"

#define PIXY_MAXIMUM_BLOCK_QUEUE    130
#define PIXY_MAXIMUM_SERVO_QUEUE	2

#define PIXY_START_WORD             0xaa55
#define PIXY_START_WORDX            0x55aa

#define PIXY_SERVO_PREAMBLE0		0x00
#define PIXY_SERVO_PREAMBLE1		0xff

typedef struct Block
{
  uint16_t itemCheck;
  uint16_t signature;
  uint16_t x;
  uint16_t y;
  uint16_t width;
  uint16_t height;
} Block, * BlockPtr;

typedef struct Servo
{
	uint8_t preamble0;
	uint8_t preamble1;
	uint16_t s0;
	uint16_t s1;
} Servo, * ServoPtr;

extern QueueHandle_t blockQueue;

extern QueueHandle_t servoQueue;

uint16_t getBlock(BlockPtr Block);
int8_t setServos(uint16_t s0, uint16_t s1);

static uint8_t TPixy_getStart();
static void TPixy_resize();



void TPixy_init(void)
{
    if( (blockQueue = xQueueCreate( PIXY_MAXIMUM_BLOCK_QUEUE,  sizeof(Block)) == 0) )
	  return;

    if( (servoQueue = xQueueCreate( PIXY_MAXIMUM_SERVO_QUEUE,  sizeof(Servo)) == 0) )
    {
	  vQueueDelete( blockQueue );
	  return;
    }

    pixy_spiInit(Default);
}

void TPixy_close(void)
{
	pixy_spiEnd();
	vQueueDelete( servoQueue );
	vQueueDelete( blockQueue );
}

static uint8_t TPixy_getStart(void)
{
  uint16_t w, lastw;

  lastw = 0xffff;

  while(1)
  {
    w = pixy_getWord();
    if (w==0 && lastw==0)
	{
      _delay_us(10);
	  return 0;
	}
    else if (w==PIXY_START_WORD && lastw==PIXY_START_WORD)
      return 1;
	else if (w==PIXY_START_WORDX)
	{
	  pixy_getByte(); // resync because we're one byte offset.
	}
	lastw = w;
  }
  return 0;
}


uint16_t TPixy_getBlocks(QueueHandle_t blockQueue)
{
  uint8_t i;
  uint16_t w, checksum, sum;
  Block block;
  static uint8_t skipStart;

  if (!skipStart)
  {
    if (getStart()== 0)
      return 0;
  }
  else
  {
	xQueueReset( blockQueue );
	skipStart = 0;
  }

  for(i=0; uxQueueSpacesAvailable( blockQueue ); ++i)
  {
	  block.itemCheck = pixy_getWord();
    if (block.itemCheck == PIXY_START_WORD) // we've reached the beginning of the next frame
    {
      skipStart = 1;
	  //Serial.println("skip");
      return block.itemCheck;
    }
    else if (checksum==0)
      return block.itemCheck;

    for (i=0, sum=0; i<sizeof(Block)/sizeof(uint16_t); i++)
    {
      w = pixy_getWord();
      sum += w;
      *((uint16_t *)block + i) = w;
    }

    if (checksum==sum)
    	block.itemCheck++;
    else
      return 0;
      //Serial.println("cs error");

	w = pixy_getWord();
    if (w!=PIXY_START_WORD)
      return block.itemCheck;
  }
  return 0;
}

int8_t pixy_setServos(uint16_t s0, uint16_t s1)
{
  Servo setServo;

  setServo.preamble0 = 0x00;
  setServo.preamble1 = 0xff;
  setServo.s0 = s0;
  setServo.s1 = s1;

  return pixi_putByte( (uint8_t *)&setServo, sizeof(Servo));
}

#ifdef __cplusplus
}
#endif

#endif // _TPIXY_H
