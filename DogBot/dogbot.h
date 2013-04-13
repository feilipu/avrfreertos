#ifndef CHEADER_DOGBOT
# define CHEADER_DOGBOT

/*
File:   dogbot.h
*/

/* Define the addresses of the devices that we want to manage on i2c bus.*/
#define SRF10           0xE0      // device address of SRF10 Sonar is 0xE0, see datasheet
#define TPA81           0xD0      // device address of TPA81 Thermopile is 0xD0, see datasheet
#define DS1307          0xD0      // device address of DS1307 Real Time Clock is 0xD0, see datasheet

// SRF10 Sonar Ranging Modes on i2c Bus

#define RANGE_INCHES    0x50
#define RANGE_CM        0x51
#define RANGE_microS    0x52


#define MAX_GAIN_40     0x00
#define MAX_GAIN_50     0x02
#define MAX_GAIN_60     0x03
#define MAX_GAIN_70     0x04
#define MAX_GAIN_80     0x05
#define MAX_GAIN_100    0x06
#define MAX_GAIN_120    0x07
#define MAX_GAIN_140    0x08
#define MAX_GAIN_200    0x09
#define MAX_GAIN_250    0x0A
#define MAX_GAIN_300    0x0B
#define MAX_GAIN_350    0x0C
#define MAX_GAIN_400    0x0D
#define MAX_GAIN_500    0x0E
#define MAX_GAIN_600    0x0F
#define MAX_GAIN_700    0x10
#define STILL_RANGING   0xFF



/* structure to pass the TPA81 Thermopile parameters */
typedef struct
{
    unsigned char    Samples;    // number of samples taken to average result     
    unsigned char    Ambient;    // ambient temperature as measured by Thermopile
    unsigned char    Pixel0;     //
    unsigned char    Pixel1;     // 
    unsigned char    Pixel2;     // 
    unsigned char    Pixel3;     //
    unsigned char    Pixel4;     //
    unsigned char    Pixel5;     //
    unsigned char    Pixel6;     //
    unsigned char    Pixel7;     //
} xThermalArray, * pThermalArray;

/* structure to pass the Sharp IR Sensor & Giro & Accelerometer parameters */
typedef struct
{
    unsigned int   adc0;        // ADC value from sensor 0
    unsigned int   adc1;        // ADC value from sensor 1
//    unsigned int   adc2;        // ADC value from sensor 2
//    unsigned int   adc3;        // ADC value from sensor 3
//    unsigned int   adc4;        // ADC value from sensor 4
//    unsigned int   adc5;        // ADC value from sensor 5
//    unsigned int   adc6;        // ADC value from sensor 6
//    unsigned char   IRdistCM;      // Calculated distance in CM
} xIRArray, * pIRArray;


/*
 * Declare a variable of type xQueueHandle.
 * this queue will have the ADC value written to it.
 */
xQueueHandle xADCQueue;

/*
 * Declare a variable of type xQueueHandle.
 * This queue will have the i2c Sonar value written to it.
 */
xQueueHandle xI2CSonarQueue;

/*
 * Declare a variable of type xQueueHandle.
 * One queue will have the i2c Thermopile values written to it.
 */
xQueueHandle xI2CThermopileQueue;

/*
 * Declare a variable of type xQueueHandle.
 * This queue will have the Transport values written to it.
 */
xQueueHandle xTransportQueue;


/* Create a Semaphore mutex flag for the ADC. To ensure only single access. */
xSemaphoreHandle xADCSemaphore;

/* Create a Semaphore mutex flag for the i2c Bus. To ensure only single access. */
xSemaphoreHandle xI2CSemaphore;

/* Create a Semaphore mutex flag for the LCD. To ensure only single access. */
xSemaphoreHandle xLCDSemaphore;

/* Create a Semaphore mutex flag for EMERGENCY STOP. */
xSemaphoreHandle xEmergencyStopSemaphore;

/* Create a Semaphore mutex flag for ARRIVAL. */
xSemaphoreHandle xAtDestinationSemaphore;


/*-----------------------------------------------------------*/

static void TaskNavigation(void *pvParameters); // Navigation Control

static void TaskBlinkGreenLED(void *pvParameters); // Main Arduino (Green) LED Blink

static void TaskReadADCSensors(void *pvParameters);       // Read ADC Sensor Values (Sharp IR)

static void TaskReadI2CThermopile(void *pvParameters);      // Read I2C Bus for Thermopile
static void TaskReadI2CSonar(void *pvParameters);   // Read I2C Bus for Sonar

static void TaskWriteAnalogLCD(void *pvParameters); // Write ADC Sensors to LCD
static void TaskWriteSonarLCD(void *pvParameters); // Write Sonar to LCD
static void TaskWriteThermopileLCD(void *pvParameters);     // Write Thermopile to LCD

extern void TaskTransport(void *pvParameters); // Transport (Motor & Odometry) Control in transport.c

/*-----------------------------------------------------------*/


#endif // CHEADER_DOGBOT

