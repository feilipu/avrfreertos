////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////    main.c
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////


#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include <avr/io.h>
#include <avr/pgmspace.h>

/* Pololu include files - for LCD access */
#include <pololu/orangutan.h>

/* Scheduler include files. */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

/* i2c Interface include file. */
#include <i2cMultiMaster.h>

/* Dogbot include file. */
#include "dogbot.h"

/* Transport (Motor control) include file */
#include "transport.h"

/* Main program loop */
int16_t main(void) __attribute__((OS_main));

int16_t main(void)
{

    xADCQueue = xQueueCreate( 2, sizeof( xIRArray ) );  // queue for ADC Sensor values
    xI2CSonarQueue = xQueueCreate( 2, sizeof( unsigned int ) );  // queue for i2c Sonar values
    xI2CThermopileQueue = xQueueCreate( 2, sizeof( xThermalArray) );  // queue for i2c Thermopile values
    xTransportQueue = xQueueCreate( 1, sizeof( xTransportVector ) );  // queue for Transport values

    xADCSemaphore = xSemaphoreCreateMutex(); // mutex semaphore for ADC
    xI2CSemaphore = xSemaphoreCreateMutex(); // mutex semaphore for i2c bus
    xLCDSemaphore = xSemaphoreCreateMutex(); // mutex semaphore for LCD

    xEmergencyStopSemaphore = xSemaphoreCreateMutex(); // mutex semaphore for EMERGENCY STOP
    xAtDestinationSemaphore = xSemaphoreCreateMutex(); // mutex semaphore for ARRIVING AT DESTINATION

    set_digital_output(IO_C4, LOW); // initialise Green IO_C4 LED


    set_analog_mode(MODE_8_BIT);    // 8-bit analog-to-digital conversions
    start_analog_conversion(0);     // start initial ADC conversion

    set_motors(0,0);                // initialise the motor speed to zero

    svp_set_mode(SVP_MODE_ENCODERS); // initialise the auxillary processor to provide encoders

    i2c_init();      // init I2C interface, need to do this once only.

    clear();			// clear the LCD, move cursor to start of top line


    xTaskCreate(
		TaskBlinkGreenLED
		,  (const signed portCHAR *)"GreenLED"
		,  128
		,  NULL
		,  3
		,  NULL );


    xTaskCreate(
		TaskNavigation
		,  (const signed portCHAR *)"Navigation"
		,  128
		,  NULL
		,  2
		,  NULL );


    xTaskCreate(
        TaskReadADCSensors
        ,  (const signed portCHAR *)"ReadADCSensors"
        ,  128
        ,  NULL
        ,  1
        ,  NULL );

/*    xTaskCreate(
        TaskReadI2CThermopile
        ,  (const signed portCHAR *)"ReadI2CThermopile"
        ,  128
        ,  NULL
        ,  2
        ,  NULL ); */

    xTaskCreate(
        TaskReadI2CSonar
        ,  (const signed portCHAR *)"ReadI2CSonar"
        ,  128
        ,  NULL
        ,  1
        ,  NULL );


    xTaskCreate(
		TaskWriteAnalogLCD
		,  (const signed portCHAR *)"WriteAnalogLCD"
		,  128
		,  NULL
		,  2
		,  NULL );

    xTaskCreate(
		TaskWriteSonarLCD
		,  (const signed portCHAR *)"WriteSonarLCD"
		,  128
		,  NULL
		,  2
		,  NULL );

/*   xTaskCreate(
		TaskWriteThermopileLCD
		,  (const signed portCHAR *)"WriteThermopileLCD"
		,  128
		,  NULL
		,  3
		,  NULL ); */

    xTaskCreate(
		TaskTransport
		,  (const signed portCHAR *)"TransportManagement"
		,  128
		,  NULL
		,  2
		,  NULL );

    vTaskStartScheduler();
}



/*------------------------ FLASH A LED TO SHOW OS RUNNING --------------------------*/

static void TaskBlinkGreenLED(void *pvParameters) // Main Green LED Flash
{
    (void) pvParameters;;
    portTickType xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialized with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
        set_digital_output( IO_C4, 0);               // main (green IO_C4) LED off
		vTaskDelayUntil( &xLastWakeTime, ( 200 / portTICK_RATE_MS ) );

        set_digital_output( IO_C4, 1);               // main (green IO_C4) LED on
		vTaskDelayUntil( &xLastWakeTime, ( 200 / portTICK_RATE_MS ) );
    }
}

/*------------------ SET THE NAVIGATION TASK RUNNING --------------------------------*/

static void TaskNavigation(void *pvParameters) // Main Navigation Task
{
    (void) pvParameters;;
    portTickType xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialized with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

	xTransportVector xSetVector;

    while(1)
    {
        if( xAtDestinationSemaphore != NULL )
        {
            // See if we can obtain the semaphore.  If the semaphore is not available
            // wait 3 ticks to see if it becomes free.
            if( xSemaphoreTake( xAtDestinationSemaphore, ( portTickType ) 2 ) == pdTRUE )
            {
                xSetVector.X = 50;  // Distance to travel in cm.
                xSetVector.Y = 0;     // Distance to travel in cm.
                xSetVector.Speed = 80;     // speed as a percentage of maximum speed.
                xSetVector.initialPoise = 0;       // Poise given to initiate, establishes angle relative to coordinates.
                xSetVector.finalPoise = 0;         // final Poise to assume.

                if( xTransportQueue != 0 )
                {
                    // Send the Transport structure.  Wait for 10 ticks for space to become
                    // available if necessary.
                    xQueueSendToBack( xTransportQueue, &xSetVector, ( portTickType ) 10 );
                }
            }
        }
        vTaskDelayUntil( &xLastWakeTime, ( 30000 / portTICK_RATE_MS ) );
    }
}

/*---------------------- READ ANALOGUE SENSORS -------------------------------------*/


static void TaskReadADCSensors(void *pvParameters) // Read ADC Sensors
{
    (void) pvParameters;;
    portTickType xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialized with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

	/*

     * Variables for the analogue conversion on ADC Sensors
     */

    const char samples = 20;        // determines the number of samples taken
    unsigned int sum;               // holds the summated samples
    xIRArray values;                // holds the return values to put on queue
    unsigned char i;

    while(1)
    {
        if( xADCSemaphore != NULL )
        {
            // See if we can obtain the semaphore.  If the semaphore is not available
            // wait 10 ticks to see if it becomes free.
            if( xSemaphoreTake( xADCSemaphore, ( portTickType ) 10 ) == pdTRUE )
            {
            // We were able to obtain the semaphore and can now access the
            // shared resource.
            // We want to have the ADC for us alone, as it takes some time to sample,
            // so we don't want it getting stolen during the middle of a conversion.

                i = samples;                     // clean up for next channel
                sum = 0;

                do
                {
                    start_analog_conversion(0);   // start next conversion
                    while( analog_is_converting() )
                    {
                        taskYIELD();     // yield until conversion ready
                    }
	                sum += analog_conversion_result();	// sum the results

                } while (--i);

                values.adc0 = (unsigned int) (sum / (int) samples);    // sample average of ADC result

                i = samples;                     // clean up for next channel
                sum = 0;

                do
                {
                    start_analog_conversion(1);   // start next conversion
                    while( analog_is_converting() )
                    {
                        taskYIELD();     // yield until conversion ready
                    }
	                sum += analog_conversion_result();	// sum the results

                } while (--i);

                values.adc1 = (unsigned int) (sum / (int) samples);    //  sample average of ADC result

                i = samples;                     // clean up for next channel
                sum = 0;

                xSemaphoreGive( xADCSemaphore );

                if( xADCQueue != 0 )
                {
                    // Send the structure.  Wait for 10 ticks for space to become
                    // available if necessary.
                    xQueueSendToBack( xADCQueue, &values, ( portTickType ) 10 );
                }

                vTaskDelayUntil( &xLastWakeTime, ( 200 / portTICK_RATE_MS ) );
            }
        }
    }
}


 /*---------------------- READ I2C BUS -------------------------------------*/

static void TaskReadI2CThermopile(void *pvParameters) // Read i2c Bus for Thermopile
{
    (void) pvParameters;;

    portTickType xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialized with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

	xThermalArray xReadings;     // Holds return values from the Thermopile TPA81.

    // Variable for the averaging process
    unsigned char samples = 0;
    unsigned char ret = 0;      // temporary variable

    while(1)
    {
        if( xI2CSemaphore != NULL )
        {
            // See if we can obtain the semaphore.  If the semaphore is not available
            // wait 10 ticks to see if it becomes free.
            if( xSemaphoreTake( xI2CSemaphore, ( portTickType ) 10 ) == pdTRUE )
            {
                // We were able to obtain the semaphore and can now access the
                // shared resource.

                /*  Reading from the Slave
                1. Send a start sequence
                2. Send 0xD0 ( I2C address of the TPA81 with the R/W bit low (even address)
                3. Send 0x00 (Internal address of the bearing register)
                4. Send a start sequence again (repeated start)
                5. Send 0xD1 ( I2C address of the TPA81 with the R/W bit high (odd address)
                6. Read data byte from TPA81
                7. Repeat, reading the next data byte from TPA81
                8. Send the stop sequence.
                */

                do
                {
                    ret = i2c_start(TPA81+I2C_WRITE);       // set device address and write mode

                    if ( ret )
                    {
                        /* failed to issue start condition, possibly no device found */
                        i2c_stop();
                        set_digital_output( IO_D1, 1);               // red LED on to show error */

                    } else {
                        /* issuing start condition ok, device accessible */
                        i2c_write(0x01);                       // write address = 1 (Ambient)

                        i2c_rep_start(TPA81+I2C_READ);              // set device address and read mode

                        xReadings.Ambient += i2c_readAck();         // read one byte from address 1
                        xReadings.Pixel0 +=  i2c_readAck();         // read one byte from address 2
                        xReadings.Pixel1 +=  i2c_readAck();         // read one byte from address 3
                        xReadings.Pixel2 +=  i2c_readAck();         // read one byte from address 4
                        xReadings.Pixel3 +=  i2c_readAck();         // read one byte from address 5
                        xReadings.Pixel4 +=  i2c_readAck();         // read one byte from address 6
                        xReadings.Pixel5 +=  i2c_readAck();         // read one byte from address 7
                        xReadings.Pixel6 +=  i2c_readAck();         // read one byte from address 8
                        xReadings.Pixel7 +=  i2c_readNak();         // read one last byte from address 9
                        i2c_stop();                                 // set stop condition = release bus

                    }

                } while (++samples < 5);       // Averaging the result over 5 samples.


                // Averaging calculations.

                xReadings.Ambient /= samples;
                xReadings.Pixel0  /= samples;
                xReadings.Pixel1  /= samples;
                xReadings.Pixel2  /= samples;
                xReadings.Pixel3  /= samples;
                xReadings.Pixel4  /= samples;
                xReadings.Pixel5  /= samples;
                xReadings.Pixel6  /= samples;
                xReadings.Pixel7  /= samples;


            }
            // We have finished accessing the shared resource.
            // Release the i2c semaphore.
            xSemaphoreGive( xI2CSemaphore );
        }

        xReadings.Samples  = samples;

        if( xI2CThermopileQueue != 0 )
        {
            // Send the Tranport structure.  Wait for 10 ticks for space to become
            // available if necessary.
            xQueueSendToBack( xI2CThermopileQueue, &xReadings, ( portTickType ) 10 );
        }


        // Reset our structure for renewed sampling and averaging.
        xReadings.Ambient = 0;
        xReadings.Pixel0  = 0;
        xReadings.Pixel1  = 0;
        xReadings.Pixel2  = 0;
        xReadings.Pixel3  = 0;
        xReadings.Pixel4  = 0;
        xReadings.Pixel5  = 0;
        xReadings.Pixel6  = 0;
        xReadings.Pixel7  = 0;
        samples = 0;


        vTaskDelayUntil( &xLastWakeTime, ( 400 / portTICK_RATE_MS ) );
    }
}



static void TaskReadI2CSonar(void *pvParameters) // Read i2c Bus for Sonar
    {
    (void) pvParameters;;

    portTickType xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialized with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

    unsigned char highbyte = 0;
    unsigned char lowbyte = 0;
    unsigned int  range = 0 ; // Holds return from the Sonar SRF 10


    while(1)
    {
       if( xI2CSemaphore != NULL )
       {
            // See if we can obtain the semaphore.  If the semaphore is not available
            // wait 10 ticks to see if it becomes free.
            if( xSemaphoreTake( xI2CSemaphore, ( portTickType ) 10 ) == pdTRUE )
            {
                // We were able to obtain the semaphore and can now access the
                // shared resource.

                /*  Reading from the Slave
                1. Send a start sequence
                2. Send 0xE0 ( I2C address of the SRF10 with the R/W bit low (even address)
                3. Send 0x00 (Internal address of the bearing register)
                4. Send a start sequence again (repeated start)
                5. Send 0xD1 ( I2C address of the SRF10 with the R/W bit high (odd address)
                6. Read data byte from SRF10
                7. Repeat, reading the next data byte from SRF10
                8. Send the stop sequence.
                */

                lowbyte = i2c_start(SRF10+I2C_WRITE);  // set device address and write mode

                if ( lowbyte )
                {
                    /* failed to issue start condition, possibly no device found */
                    i2c_stop();
                    set_digital_output( IO_D1, 1);               // red LED on to show error */

                }else {
                    /* issuing start condition ok, device accessible */
                    i2c_write(0x00);                       // Write Command Register = 0
                    i2c_write(RANGE_CM);                   // Start Ranging in cm.
                    i2c_stop();                            // set stop condition = release bus
                }
            }
        }
        // We have finished accessing the shared resource.
        // Release the i2c semaphore.
        xSemaphoreGive( xI2CSemaphore );

       /*  echo echo echo time */
       vTaskDelayUntil( &xLastWakeTime, ( 80 / portTICK_RATE_MS ) );

       if( xI2CSemaphore != NULL )
       {
            // See if we can obtain the semaphore.  If the semaphore is not available
            // wait 10 ticks to see if it becomes free.
            if( xSemaphoreTake( xI2CSemaphore, ( portTickType ) 10 ) == pdTRUE )
            {
                // We were able to obtain the semaphore and can now access the
                // shared resource.

                i2c_start(SRF10+I2C_READ);                  // set device address and write mode
                i2c_write(0x02);                            // write Range High Byte
                i2c_rep_start(SRF10+I2C_READ);              // set device address and read mode
                highbyte = i2c_readAck();                   // read one byte from high byte  - 255
                lowbyte = i2c_readNak();                    // read one byte from low byte - 255
                i2c_stop();                                 // set stop condition = release bus

                range = (highbyte << 8) + lowbyte;          // Put them together

            }
        }
        // We have finished accessing the shared resource.
        // Release the i2c semaphore.
        xSemaphoreGive( xI2CSemaphore );

        if( xI2CSonarQueue != 0 )
        {
            // Send an unsigned int.  Wait for 10 ticks for space to become
            // available if necessary.
            xQueueSendToBack( xI2CSonarQueue, ( void * ) &range, ( portTickType ) 10 );
        }

        // Reset our variable for new averaging.
        range = 0;
        vTaskDelayUntil( &xLastWakeTime, ( 200 / portTICK_RATE_MS ) );

    }
}
/*--------------------- WRITE TO LCD --------------------------------------*/

static void TaskWriteAnalogLCD(void *pvParameters) // Write to LCD
{
    (void) pvParameters;;

    xIRArray result;  // variable to hold the average ADC value off the queue
    int dist;

    while(1)
    {
        if( xADCQueue != 0 )
        {
        /* Block on the queue to wait for data to arrive. */
            if( xLCDSemaphore != NULL )
            {
                // See if we can obtain the semaphore.  If the semaphore is not available
                // wait 10 ticks to see if it becomes free.
                if( xSemaphoreTake( xLCDSemaphore, ( portTickType ) 10 ) == pdTRUE )
                {
                    // We were able to obtain the semaphore and can now access the
                    // shared resource.

                    if( xQueueReceive( xADCQueue, &result, portMAX_DELAY) )
                    {
                        // print the short range ADC sensor in adc1
//                        lcd_goto_xy(12, 1);          // go to the 13th character of the second LCD line
//                        print_long(result.adc1);
//                        print("R ");			// added spaces are to overwrite left over chars
                        lcd_goto_xy(12, 1);          // go to the 13th character of the second LCD line
                        dist = (int)(3020.67 * pow((portFLOAT)result.adc1,-0.822758) * 0.3);

                        if( dist < 10 || dist > 200)
                        {
                            print("--");
                        }else{
                            print_long(dist);
                        }
                        print("cm ");			// added spaces are to overwrite left over chars

                        // print the long range ADC sensor in adc0
//                        lcd_goto_xy(0, 1);          // go to the first character of the second LCD line
//                        print_long(result.adc0);
//                        print("L ");			// added spaces are to overwrite left over chars
                        lcd_goto_xy(0, 1);			// LCD cursor to 1st character of the second line
                        dist = (int)(2287.25 * pow((portFLOAT)result.adc0,-0.928987));

                        if( dist < 20 || dist > 240)
                        {
                            print("--");
                        }else{
                            print_long(dist);
                        }
                        print("cm ");			// added spaces are to overwrite left over chars
                    }
                }
                xSemaphoreGive( xLCDSemaphore );
                taskYIELD();
            }
        }
    }
}

static void TaskWriteSonarLCD(void *pvParameters) // Write to LCD
{
    (void) pvParameters;;

    unsigned int sonar;    // variable to hold the Sonar value off the queue

    while(1)

    {
        if( xI2CSonarQueue != 0 )
        {
        /* Block on the queue to wait for data to arrive. */
            if( xLCDSemaphore != NULL )
            {
                // See if we can obtain the semaphore.  If the semaphore is not available
                // wait 10 ticks to see if it becomes free.
                if( xSemaphoreTake( xLCDSemaphore, ( portTickType ) 10 ) == pdTRUE )
                {
                    // We were able to obtain the semaphore and can now access the
                    // shared resource.

                    if( xQueueReceive( xI2CSonarQueue, &sonar, portMAX_DELAY) )
                    {
                        lcd_goto_xy(6, 1);          // go to the 7th character of the second LCD line
                        print_long( sonar );        // print the sonar value in centimetres
                        print("cm ");			// added spaces are to overwrite left over chars
                    }
                }
                xSemaphoreGive( xLCDSemaphore );
                taskYIELD();
            }
        }
    }
}

static void TaskWriteThermopileLCD(void *pvParameters) // Write thermopileto LCD
{
    (void) pvParameters;;

    xThermalArray xValues;     // Holds result values from the Thermopile TPA81.

    const char heat_diff = 3;
    while(1)
    {
        /* Block on the queue to wait for data to arrive. */
        if( xI2CThermopileQueue != 0 )
        {

            if( xLCDSemaphore != NULL )
            {
                // See if we can obtain the semaphore.  If the semaphore is not available
                // wait 10 ticks to see if it becomes free.
                if( xSemaphoreTake( xLCDSemaphore, ( portTickType ) 10 ) == pdTRUE )
                {
                    // We were able to obtain the semaphore and can now access the
                    // shared resource.

                    if( xQueueReceive( xI2CThermopileQueue, &xValues, portMAX_DELAY) == pdPASS )
                    {

                        lcd_goto_xy(0, 0);              // go to the first character of the first LCD line
                        if((xValues.Pixel7 - xValues.Ambient) > heat_diff) {print_long(xValues.Pixel7 );}else{print("--");}
                        lcd_goto_xy(2, 0);              // go to the third character of the first LCD line
                        if((xValues.Pixel6 - xValues.Ambient) > heat_diff) {print_long(xValues.Pixel6 );}else{print("--");}
                        lcd_goto_xy(4, 0);              // go to the fifth character of the first LCD line
                        if((xValues.Pixel5 - xValues.Ambient) > heat_diff) {print_long(xValues.Pixel5 );}else{print("--");}
                        lcd_goto_xy(6, 0);              // go to the seventh character of the first LCD line
                        if((xValues.Pixel4 - xValues.Ambient) > heat_diff) {print_long(xValues.Pixel4 );}else{print("--");}
                        lcd_goto_xy(8, 0);              // go to the ninth character of the first LCD line
                        if((xValues.Pixel3 - xValues.Ambient) > heat_diff) {print_long(xValues.Pixel3 );}else{print("--");}
                        lcd_goto_xy(10, 0);              // go to the eleventh character of the first LCD line
                        if((xValues.Pixel2 - xValues.Ambient) > heat_diff) {print_long(xValues.Pixel2 );}else{print("--");}
                        lcd_goto_xy(12, 0);              // go to the thirteenth character of the first LCD line
                        if((xValues.Pixel1 - xValues.Ambient) > heat_diff) {print_long(xValues.Pixel1 );}else{print("--");}
                        lcd_goto_xy(14, 0);              // go to the fifteenth character of the first LCD line
                        if((xValues.Pixel0 - xValues.Ambient) > heat_diff) {print_long(xValues.Pixel0 );}else{print("--");}

                    }
                }
                xSemaphoreGive( xLCDSemaphore );
                taskYIELD();
            }
        }
    }
}

/*------------This is testing code cruft:----------------------------



                set_digital_output( IO_D1, 0);               // red LED on to show process
                vTaskDelay(  500 / portTICK_RATE_MS );
                set_digital_output( IO_D1, 1);               // red LED on to show process
                vTaskDelay( 1000 / portTICK_RATE_MS );

                set_digital_output( IO_D1, 0);               // red LED on to show process
                vTaskDelay( 500 / portTICK_RATE_MS );
                set_digital_output( IO_D1, 1);               // red LED on to show process




-----------------------------------------------------------*/

