#ifndef CHEADER_DANGER
#define CHEADER_DANGER

/*
File:   danger.h
*/

#ifdef __cplusplus
extern "C" {
#endif



// Pin definitions - for Danger Shield
#define SLIDER1  0
#define SLIDER2  1
#define SLIDER3  2

#define LED1 	 IO_D5 // 5
#define LED2  	 IO_D6 // 6

#define BUTTON1  IO_B2 // 10
#define BUTTON2  IO_B3 // 11
#define BUTTON3  IO_B4 // 12

#define DATA 	 IO_D4 // 4   SN74HC595 shift register
#define LATCH 	 IO_D7 // 7
#define CLOCK 	 IO_B0 // 8

// for shiftOut();
#define LSBFIRST 0
#define MSBFIRST 1


/* structure to pass the analogue sample parameters */
typedef struct
{
	uint8_t   adc0;        // ADC value from sensor 0 Slider
	uint8_t   adc1;        // ADC value from sensor 1 Slider
	uint8_t   adc2;        // ADC value from sensor 2 Slider
	uint8_t   adc3;        // ADC value from sensor 3 Photocell
} xADCArray, * pADCArray;



/*-----------------------------------------------------------*/


static void TaskBlinkYellowLED(void *pvParameters); 		// Main Yellow LED Blink

static void TaskADB(void *pvParameters);				// ADB Setup and Poll

static void TaskWrite7SEG(void *pvParameters);			// Write to the 7 Segment (via shift register)

/*-----------------------------------------------------------*/

static void ReadADCSensors(void);		// Read ADC Sensor Values

// shift bits out on the Danger Shield SN74HC595 shift register
//	character = pgm_read_byte(&ledCharSet[(values.adc1 >> 3)]); // retrieve the character from PROGMEM; only do this once.
// reduce the 8 bit value to 5 bits (32 values, 7 segment) code

static void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t latchPin, uint8_t bitOrder, uint8_t bitVal);
//	shiftOut(DATA,CLOCK,LATCH,MSBFIRST,~(character | 0b10000000)); // turn on decimal point
//	shiftOut(DATA,CLOCK,LATCH,MSBFIRST,~(character & 0b01111111)); // turn off decimal point

/*-----------------------------------------------------------*/
prog_uchar ledCharSet[]  = // put these characters in PROGMEM, to save RAM.
	   {
			0b00111111, //  0	0  use pgm_read_byte(&ledCharSet[x])
			0b00000110, //  1	1  to get them out of PROGMEM.
			0b01011011, //  2	2
			0b01001111, //  3	3
			0b01100110, //  4	4
			0b01101101, //  5	5
			0b01111101, //  6	6
			0b00000111, //  7	7
			0b01111111, //  8	8
			0b01101111, //  9	9
			0b01110111, //  A	10
			0b01111100, //  b	11
			0b00111001, //  C	12
			0b01011110, //  d	13
			0b01111001, //  E	14
			0b01110001, //  F	15

			0b00111101, //  G	16
			0b01110110, //  H	17
			0b00110000, //  I	18
			0b00001110, //  J	19
			0b00111000, //  L	20
			0b01010100, //  n	21
			0b01011100, //  o	22
			0b01110011, //  P	23
			0b01100111, //  q	24
			0b01010000, //  r	25
			0b01101101, //  S	26
			0b01111000, //  t	27
			0b00111110, //	U	28
			0b01101110, //  Y	29

			0b01010111, //  λ	30
			0b01110010, //  μ	31

			0b00000001, //  ^	32
			0b00001000, //  _	33
			0b01000000, //  -	34
			0b01110000, //  +	35
			0b01001000, //  =	36
			0b00111001, //  [	37
			0b00001111, //  ]	38
			0b01010011, //  ?	39

	   };   // Shift register bit values to display on the Danger Shield seven-segment display.

#ifdef __cplusplus
}
#endif

#endif // CHEADER_DANGER
