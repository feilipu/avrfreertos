#ifndef PEGGYVIDEOPONG_H
# define PEGGYVIDEOPONG_H
/*
File:   PeggyVideo.h
*/

#ifdef __cplusplus
extern "C" {
#endif

// Define the addresses of the devices that we want to manage on i2c bus.

#define PEGGYVIDEO		0x34      // device address of Peggy Video
#define PONGVIDEO		0x32	  // device address of Pong Video game


// 25 rows * 13 bytes per row == 325
#define DISP_BUFFER_SIZE 325
#define DISP_COLUMN_LENGTH 25
#define DISP_ROW_LENGTH 25
#define DISP_BYTES_LENGTH 13

#define HOTROWLIMIT		1						// maximum consecutive times to enter hotRow handling

#define PLAYER1_X		1						// column for player1 paddle
#define PLAYER2_X		23						// column for player2 paddle


#define g 			0.05						// gravitational acceleration (should be positive.)
#define TIMESTEP 	0.05 / portTICK_PERIOD_MS	// TimeStep
#define BOUNCE		0.98 						// Bounce (only very slow decay)
#define ACCEL		0.25						// Paddle acceleration

#define MAX_HITS		128						// maximum number of paddle hits.


//*******************************************************
//            Circle Definitions drawCircle
//*******************************************************
#define FULLCIRCLE    1
#define OPENSOUTH     2
#define OPENNORTH     3
#define OPENEAST      4
#define OPENWEST      5
#define OPENNORTHEAST 6
#define OPENNORTHWEST 7
#define OPENSOUTHEAST 8
#define OPENSOUTHWEST 9

#define MAX_BRIGHTNESS 15

// Pin definitions - for Danger Shield
#define SLIDER1  	0
#define SLIDER2  	1
#define SLIDER3  	2

#define BUZZER_IO	IO_D3 // 3
#define LED1 	 	IO_D5 // 5
#define LED2  	 	IO_D6 // 6

#define RESTART  	IO_B2 // 10
#define SERVE    	IO_B3 // 11
#define PAUSE    	IO_B4 // 12


#define DATA 	 	IO_D4 // 4   SN74HC595 shift register
#define LATCH 	 	IO_D7 // 7
#define CLOCK 	 	IO_B0 // 8

#define LSBFIRST 	0
#define MSBFIRST 	1


////////////////////////////////////////////////////////////////////////////////////////////

/*
 * Pixels are stored in the frameBuffer[] array two per byte, allowing
 * a maximum of 16 levels of brightness per pixel, including off.
 * The Pixel union allows the code to access either pairs of pixels
 * using the both name, or as individual 4 bit pixels using the
 * even and odd names.
 */
typedef union {
	uint8_t both;
	struct {
		uint8_t even:4;
		uint8_t odd:4;
	};
} Pixel, * pPixel;


// structure to pass the video row parameters.
typedef struct
{
	uint8_t		I2CAddress; // Only needed on the the MASTER side of the transmission. Exclude from CRC8.
    uint8_t		RowNumber;  //
	uint8_t     DoublePixels[DISP_BYTES_LENGTH];     // two 4 bit pixels per byte, upper & lower nibble.
    uint8_t     CRC8;        //
} xVideoRowArray, * pVideoRowArray;


/* structure to pass the analogue sample parameters */
typedef struct
{
	uint8_t   adc0;        // ADC value from sensor 0 Slider
	uint8_t   adc1;        // ADC value from sensor 1 Slider
	uint8_t   adc2;        // ADC value from sensor 2 Slider
	uint8_t   adc3;		   // ADC value from the photo sensor
} xADCArray, * pADCArray;


// Create a Semaphore mutex flag for the ADC. To ensure only single access.
SemaphoreHandle_t xADCSemaphore;

// Declare a variable of type QueueHandle_t.
// This queue will have the I2C LCD values written to it.
QueueHandle_t xI2CQueue;

/*-----------------------------------------------------------*/

static void TaskPong(void *pvParameters); // Play the pong ball game, fast loop, 100Hz

static void TaskBall(void *pvParameters); // Play the ball bouncing game

static void TaskFace(void *pvParameters); // Show a flashing face

static void TaskSlowLoop(void *pvParameters); // Do the pong background tasks, 10Hz

static void TaskWriteI2CVideo(void *pvParameters);   // Write I2C Bus for Video

/*-----------------------------------------------------------*/

static void ReadADCSensors(void);		// Read ADC Sensor Values

// shift bits out on the Danger Shield SN74HC595 shift register
//	character = pgm_read_byte(&ledCharSet[(values.adc1 >> 3)]); // retrieve the character from PROGMEM; only do this once.
// reduce the 8 bit value to 5 bits (32 values, 7 segment) code

static void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t latchPin, uint8_t bitOrder, uint8_t bitVal);
//	shiftOut(DATA,CLOCK,LATCH,MSBFIRST,~(character | 0b10000000)); // turn on decimal point
//	shiftOut(DATA,CLOCK,LATCH,MSBFIRST,~(character & 0b01111111)); // turn off decimal point


/*-----------------------------------------------------------*/

const uint8_t bach[] PROGMEM = "!T110 L8 a gafaeada c+adaeafa >aa>bac#ada c#adaeaf4";

const uint8_t guminam[] PROGMEM = "!T160 L2 a1aa1age+ e1rd>cb-a4b-4 b-1rd>cb-4b-.a4b-4 b-rd.e.f a1b-g1f1 g1. rffe4d.ff1.";

const uint8_t cmajor[] PROGMEM = "!T110 L16 cdefgab>cbagfedc";

const uint8_t ledCharSet[] PROGMEM = // put these characters in PROGMEM, to save RAM.
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



const uint8_t Picture[] PROGMEM = // put these characters in PROGMEM, to save RAM.
{
//   use pgm_read_byte(&Picture[x]) to get them out of PROGMEM.

15, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8, 0, 9, 0,10, 0,11, 0,12,
14, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8, 0, 9, 0,10, 0,11, 0,12,
13, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8, 0, 9, 0,10, 0,11, 0,12,
12, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8, 0, 9, 0,10, 0,11, 0,12,
11, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8, 0, 9, 0,10, 0,11, 0,12,
10, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8, 0, 9, 0,10, 0,11, 0,12,
 9, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8, 0, 9, 0,10, 0,11, 0,12,
 8, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8, 0, 9, 0,10, 0,11, 0,12,
 7, 0, 1, 0, 2, 0, 3, 0,15,15,15,15,15, 0, 7, 0, 8, 0, 9, 0,10, 0,11, 0,12,
 6, 0, 1, 0, 2, 0, 3, 0,15,15,15,15,15,10, 7, 0, 8, 0, 9, 0,10, 0,11, 0,12,
 5, 0, 1, 0, 2, 0, 3, 0,15,15,15,15,15,10,10, 0, 8, 0, 9, 0,10, 0,11, 0,12,
 4, 0, 1, 0, 2, 0, 3, 0,15,15,15,15,15,10,10,10, 8, 0, 9, 0,10, 0,11, 0,12,
 3, 0, 1, 0, 2, 0, 3, 0,15,15,15,15,15,10,10,10, 8, 0, 9, 0,10, 0,11, 0,12,
 2, 0, 1, 0, 2, 0, 3, 0,15,15,15,15,15,10,10,10, 8, 0, 9, 0,10, 0,11, 0,12,
 1, 0, 1, 0, 2, 0, 3, 0,15,15,15,15,15,10,10,10, 8, 0, 9, 0,10, 0,11, 0,12,
 0, 0, 1, 0, 2, 0, 3, 0, 4,10,00,00,00,10,10,10, 8, 0, 9, 0,10, 0,11, 0,12,
 0, 0, 1, 0, 2, 0, 3, 0, 4, 0,10,00,00,00,10,10, 8, 0, 9, 0,10, 0,11, 0,12,
 0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5,10,10,10,10,10, 8, 0, 9, 0,10, 0,11, 0,12,
 0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8, 0, 9, 0,10, 0,11, 0,12,
 0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8, 0, 9, 0,10, 0,11, 0,12,
 0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8, 0, 9, 0,10, 0,11, 0,12,
 0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8, 0, 9, 0,10, 0,11, 0,12,
 0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8, 0, 9, 0,10, 0,11, 0,12,
 0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8, 0, 9, 0,10, 0,11, 0,12,
 0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8, 0, 9, 0,10, 0,11, 0,12,

// OR
/*
11,11,10, 3, 2, 1, 1, 1, 1, 1, 2, 2, 2, 1, 2, 2, 2, 2, 2, 7,13,13,13,14,13,
11,11, 8, 3, 1, 1, 1, 1, 1, 1, 1, 1, 4, 5, 4, 1, 2, 2, 1, 3, 6,12,14,13,13,
10,10, 6, 2, 1, 1, 2, 2, 1, 3, 5, 8,13,15,14, 8, 3, 1, 1, 3, 3, 8,14,13,13,
11, 8, 3, 2, 3, 5, 7, 7, 8,13,15,15,15,15,15,15,11, 5, 3, 4, 3, 7,13,13,13,
11, 8, 3, 3, 7,10,13,14,15,15,15,15,15,15,15,15,15,13, 5, 3, 5, 6,12,14,13,
11, 9, 4, 6, 9,11,13,14,14,15,15,15,14,14,15,15,15,13, 7, 4, 6, 5,10,14,13,
12,11, 6, 7, 9,11,11,13,13,14,14,14,13,14,14,14,15,14, 9, 6, 5, 6, 8,14,13,
11,11, 5, 7, 9,11,11,12,12,14,14,14,14,14,14,15,15,13,11, 7, 4, 4, 5, 9,14,
11, 8, 4, 6, 9,13,14,14,13,14,14,15,13,13,14,14,14,10, 8, 6, 3, 5, 3, 7,11,
11, 7, 2, 5, 6, 7, 8,11,11, 9, 8, 7, 4, 7,11,12,11, 9, 5, 5, 4, 6, 4, 6,11,
10, 8, 3, 2, 1, 1, 2, 4, 8, 5, 2, 1, 1, 2, 5, 9,11,10, 4, 3, 4, 6, 5, 6,13,
10, 9, 4, 1, 1, 0, 1, 1, 8, 6, 2, 0, 2, 3, 2, 6,12,11, 4, 3, 3, 6, 9, 9,13,
 9,10, 6, 2, 1, 2, 1, 3,11,11, 5, 3, 3, 6, 7, 7,12,13, 7, 7, 3, 6,12,12,13,
11,11, 8, 6, 5, 4, 4, 7,14,13,11, 7, 6, 9,11,13,14,14, 9,10, 5, 8,11, 9,13,
 9,11, 9, 8, 8, 7, 8,11,14,13,13,11, 9,11,13,14,13, 8, 6,10, 9,11, 7, 8,12,
 9,10,10, 7, 9,11, 8,12,14,13,11,11,14,14,12,11, 9, 5, 5,10,13,13, 9,11,13,
10,11,10, 7, 6, 5, 5, 8,11,11,10, 6, 9,13,11, 7, 6, 6, 6, 8,10,10,12,13,13,
10, 9,11, 7, 2, 2, 4, 2, 2, 4, 7, 8, 4, 5, 5, 6, 7, 8, 5, 5, 8,11,13,13,13,
10,11,10, 9, 3, 2, 4, 2, 2, 5,11,13, 7, 5, 4, 6, 8, 9, 7, 9,12,14,14,13,13,
10,11,11,10, 5, 2, 3, 3, 3, 5, 7, 7, 6, 7, 5, 7, 8, 8, 9,14,13,14,13,13,13,
10,11,11,11, 5, 3, 3, 4, 6, 7, 7, 8, 8, 7, 4, 6, 8, 6,11,14,13,13,13,13,13,
11,11,11,11, 4, 2, 3, 2, 3, 5, 7, 9, 7, 5, 4, 6, 5, 6,13,11,13,13,13,13,13,
11,11,11, 9, 3, 4, 3, 6, 6, 8, 7, 6, 4, 2, 3, 4, 4, 8,15, 8, 9,13,13,13,13,
11,11,10,11, 5, 2, 2, 3, 4, 3, 3, 3, 2, 2, 2, 1, 4,12,15, 7, 4, 9,13,13,13,
11,11,11,11,11, 4, 1, 1, 2, 2, 2, 2, 1, 1, 1, 3, 7,15,15, 7, 3, 5, 8,11,13,
*/
};



#ifdef __cplusplus
}
#endif

#endif // PEGGYVIDEOONG_H
