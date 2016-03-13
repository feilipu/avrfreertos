/*****************************************************************************
*
* File              : i2cMultiMaster.c
* Compiler          : avr gcc
* Description       : This is a sample driver for the TWI hardware modules.
*                     It is interrupt driven. All functionality is controlled through
*                     passing information to and from functions.
*
****************************************************************************/

#include <stdio.h>
#include <stdbool.h>

#include <avr/interrupt.h>

/* Scheduler include files. */
#include "FreeRTOS.h"

#include "i2cMultiMaster.h"

/* Create a Semaphore binary flag for the i2c Bus. To ensure only single access. */
SemaphoreHandle_t xI2CSemaphore = NULL;

static uint8_t I2C_buf[ I2C_BUFFER_SIZE ];    // Transceiver buffer
static uint8_t I2C_msgSize;                   // Number of bytes to be transmitted.
static uint8_t I2C_state = I2C_NO_STATE;      // State byte. Default set to I2C_NO_STATE.

static uint8_t I2C_checkBusyAfterStop = 0; 	  // Number of busy check times following a Stop_Restart xA0

union I2C_statusReg I2C_statusReg = {0};      // I2C_statusReg is defined in i2cMultiMaster.h

/* Private Functions */

static uint8_t I2C_Transceiver_Busy( void ) __attribute__ ((flatten));

/****************************************************************************
 * Call this function to set up the TWI slave to its initial standby state.
 * Remember to enable interrupts from the main application after initialising the TWI.
 * Pass both the slave address and the requirements for triggering on a general call in the
 * same byte. Use e.g. this notation when calling this function:
 * I2C_Slave_Initialise( (I2C_slaveAddress<<I2C_ADR_BITS) | (TRUE<<I2C_GEN_BIT) );
 * The TWI module is configured to NACK on any requests. Use a I2C_Slave_Start_Transceiver
 * function to start the TWI.
 *****************************************************************************/
void I2C_Slave_Initialise( uint8_t I2C_ownAddress )
{
	// The Semaphore has to be created to allow the I2C bus to be shared.
	//Assuming the I2C bus will be shared.
	// Use this semaphore  (take, give) when calling I2C functions, and it can ensure single access.
    if( xI2CSemaphore == NULL ) 					// Check to see if the semaphore has not been created.
    {
		xI2CSemaphore = xSemaphoreCreateMutex();	// mutex semaphore for I2C bus
		if( ( xI2CSemaphore ) != NULL )
			xSemaphoreGive( ( xI2CSemaphore ) );	// make the I2C available
    }

	I2C_PORT_DIR &= ~(I2C_BIT_SCL | I2C_BIT_SDA);	// set the I2C SDA & SCL inputs.

	TWAR = I2C_ownAddress;                       	// Set own TWI slave address.  Accept TWI General Calls if ODD address.
	TWDR = 0xff;                                  	// Default content = SDA released.
	TWCR = (1<<TWEN)|                            	// Enable TWI-interface and release TWI pins.
		   (0<<TWIE)|(0<<TWINT)|                   	// Disable TWI Interrupt.
		   (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|        	// Do not ACK on any requests, yet.
		   (0<<TWWC);
}


/****************************************************************************
Call this function to set up the TWI master to its initial standby state.
Remember to enable interrupts from the main application after initialising the TWI.
****************************************************************************/
void I2C_Master_Initialise( uint8_t I2C_ownAddress )
{
	// The Semaphore has to be created to allow the I2C bus to be shared.
	// Assuming the I2C bus will be shared.
	// Use this semaphore  (take, give) when calling I2C functions, and it can ensure single access.
    if( xI2CSemaphore == NULL ) 					// Check to see if the semaphore has not been created.
    {
		xI2CSemaphore = xSemaphoreCreateMutex();	// mutex semaphore for I2C bus
		if( ( xI2CSemaphore ) != NULL )
			xSemaphoreGive( ( xI2CSemaphore ) );	// make the I2C bus available
    }


	I2C_PORT_DIR &= ~(I2C_BIT_SCL | I2C_BIT_SDA);	// set the I2C SDA & SCL inputs.
													// Pull up resistors
	I2C_PORT |= (I2C_BIT_SCL | I2C_BIT_SDA);		// only need these set at one place, usually Master.

													// Initialise TWI clock
    TWSR = 0;                         				// no prescaler
    TWBR = ((F_CPU/SCL_CLOCK)-16)/2;  				// must be > 10 for stable operation

	TWAR = I2C_ownAddress;                     		// Set own TWI slave address, in case it is called.
													// Accept TWI General Calls if ODD address.

	TWDR = 0xff;                               		// Default content = SDA released.
	TWCR = (1<<TWEN)|                           	// Enable TWI-interface and release TWI pins.
		   (0<<TWIE)|(0<<TWINT)|                	// Disable Interrupt.
		   (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|     	// No Signal requests.
		   (0<<TWWC);
}

/****************************************************************************
 * Call this function to start the Transceiver without specifying new transmission data.
 * Useful for restarting a transmission, or just starting the transceiver for reception.
 * The driver will reuse the data previously put in the transceiver buffers. The function will
 * hold execution (loop) until the I2C_ISR has completed with the  previous operation, then
 * Initialise the next operation and return.
****************************************************************************/
void I2C_Slave_Start_Transceiver(void)
{
  while ( I2C_Transceiver_Busy() );             // Wait until TWI is ready for next transmission.
  I2C_statusReg.all = 0;
  I2C_state         = I2C_NO_STATE ;
  TWCR = (1<<TWEN)|                             // TWI Interface enabled.
         (1<<TWIE)|(1<<TWINT)|                  // Enable TWI Interrupt and clear the flag.
         (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|       // Prepare to ACK next time the Slave is addressed.
         (0<<TWWC);                             //
}

/****************************************************************************
Call this function to re-send the last message. The driver will reuse the data previously
put in the transceiver buffers. The function will hold execution (loop) until the I2C_ISR
has completed with the previous operation, then initialise the next operation and return.
****************************************************************************/
void I2C_Master_Start_Transceiver(void)
{
  while ( I2C_Transceiver_Busy() );             // Wait until TWI is ready for next transmission.
  I2C_statusReg.all = 0;
  I2C_state         = I2C_NO_STATE ;
  TWCR = (1<<TWEN)|                             // TWI Interface enabled.
         (1<<TWIE)|(1<<TWINT)|                  // Enable TWI Interrupt and clear the flag.
         (1<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|       // Initiate a START condition.
         (0<<TWWC);                             //
}


/*****************************************************************************
 * Call this function to send a prepared message, or start the Transceiver for reception. Include
 * a pointer to the data to be sent if a SLA+W is received. The data will be copied to the TWI
 * buffer.  Also include how many bytes that should be sent. Note that unlike the similar Master
 * function, the Address byte is not included in the message buffers.
 * The function will hold execution (loop) until the I2C_ISR has completed with the previous operation,
 * then initialize the next operation and return.
 ******************************************************************************/
void I2C_Slave_Start_Transceiver_With_Data( uint8_t *msg, uint8_t msgSize )
{

    while ( I2C_Transceiver_Busy() );	// Wait until TWI is ready for next transmission.

    I2C_msgSize = msgSize;				// Number of bytes to transmit.

    // Copy data that may be transmitted if the TWI Master requests data.
    for ( uint8_t i = 0; i < msgSize; i++ )
        I2C_buf[ i ] = msg[ i ];

    I2C_statusReg.all = 0;
    I2C_state         = I2C_NO_STATE ;
    TWCR = (1<<TWEN)|                             // TWI Interface enabled.
           (1<<TWIE)|(1<<TWINT)|                  // Enable TWI Interrupt and clear the flag.
           (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|       // Prepare to ACK next time the Slave is addressed.
           (0<<TWWC);                             //
}

/****************************************************************************
Call this function to send a prepared message. The first byte must contain the slave address and the
read/write bit. Consecutive bytes contain the data to be sent, or empty locations for data to be read
from the slave. Also include how many bytes that should be sent/read including the address byte.
The function will hold execution (loop) until the I2C_ISR has completed with the previous operation,
then initialise the next operation and return.
****************************************************************************/
void I2C_Master_Start_Transceiver_With_Data( uint8_t *msg, uint8_t msgSize )
{
	while ( I2C_Transceiver_Busy() );             // Wait until TWI is ready for next transmission.

	I2C_msgSize = msgSize;                        // Number of data to transmit.
	I2C_buf[0]  = msg[0];                         // Store slave address with R/W setting.

	if (( msg[0] & (true<<I2C_READ_BIT) ) == false)       // If it is a write operation, then also copy data.
		for ( uint8_t i = 1; i < msgSize; i++ )
			I2C_buf[ i ] = msg[ i ];


	I2C_statusReg.all = 0;
	I2C_state         = I2C_NO_STATE ;
	TWCR = (1<<TWEN)|                           // TWI Interface enabled.
		   (1<<TWIE)|(1<<TWINT)|                // Enable TWI Interrupt and clear the flag.
		   (0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|     // Initiate a START condition.
		   (0<<TWWC);                           //
}




/****************************************************************************
 * Call this function to read out the received data from the TWI transceiver buffer. I.e. first
 * call I2C_Start_Transceiver to get the TWI Transceiver to fetch data. Then Run this function to
 * collect the data when they have arrived. Include a pointer to where to place the data and
 * the number of bytes to fetch in the function call. The function will hold execution (loop)
 * until the I2C_ISR has completed with the previous operation, before reading out the data
 * and returning. If there was an error in the previous transmission the function will return
 * the TWI State code.
 *****************************************************************************/
uint8_t I2C_Slave_Get_Data_From_Transceiver( uint8_t *msg, uint8_t msgSize )
{

    while ( I2C_Transceiver_Busy() );    // Wait until TWI finished with the transmission.

    if( I2C_statusReg.lastTransOK )    	// Last transmission completed successfully.
    {
        for ( uint8_t i=0; i<msgSize; i++ )        // Copy data from Transceiver buffer.
            msg[i] = I2C_buf[i];

        I2C_statusReg.RxDataInBuf = false;        // Slave Receive data has been read from buffer.
    }
    return I2C_statusReg.lastTransOK;
}


/****************************************************************************
Call this function to read out the requested data from the TWI transceiver buffer. I.e. first call
I2C_Start_Transceiver to send a request for data to the slave. Then Run this function to collect the
data when they have arrived. Include a pointer to where to place the data and the number of bytes
requested (including the address field) in the function call. The function will hold execution (loop)
until the I2C_ISR has completed with the previous operation, before reading out the data and returning.
If there was an error in the previous transmission the function will return the TWI error code.
****************************************************************************/
uint8_t I2C_Master_Get_Data_From_Transceiver( uint8_t *msg, uint8_t msgSize )
{

  while ( I2C_Transceiver_Busy() );             // Wait until TWI is ready for next transmission.

  if( I2C_statusReg.lastTransOK )               // Last transmission completed successfully.
    for ( uint8_t i=0; i<msgSize; i++ )         // Copy data from Transceiver buffer.
			msg[ i ] = I2C_buf[ i ];

  return( I2C_statusReg.lastTransOK );
}


/****************************************************************************
Call this function to test if the I2C_ISR is busy transmitting.
****************************************************************************/
uint8_t I2C_Transceiver_Busy(void)
{
  return ( TWCR & (1<<TWIE) );                  // IF TWI Interrupt is enabled then the Transceiver is busy
}


/****************************************************************************
 * Manual Bus Check: Add this in your idle code, before issuing a start transceiver command.
 * Used only for multi-master operation.
******************************************************************************/
uint8_t I2C_Check_Free_After_Stop(void) //
{
//	I2C_checkBusyAfterStop = I2C_HOW_MANY_BUSY_CHECKS_AFTER_STOP;

	while ( I2C_checkBusyAfterStop > 0 )	// Call repeatedly
	{
		if((~(I2C_PORT_STATUS) && (I2C_BIT_SCL | I2C_BIT_SDA)) == 0) // both SCL and SDA should be high on idle bus.
			 // Good. The bus is quiet. Count down!
			 --I2C_checkBusyAfterStop;
		 else
			 // Bus is busy. Start the count down all over again.
			 I2C_checkBusyAfterStop = I2C_HOW_MANY_BUSY_CHECKS_AFTER_STOP;
	}
	return true;
}

/****************************************************************************
Call this function to fetch the state information of the previous operation. The function will hold execution (loop)
until the TWI_ISR has completed with the previous operation. If there was an error, then the function
will return the TWI State code.
****************************************************************************/
uint8_t I2C_Get_State_Info(void)
{
  while ( I2C_Transceiver_Busy() );             // Wait until I2C has completed the transmission.
  return ( I2C_state );                         // Return error state.
}



// ********** Interrupt Handler ********** //
/****************************************************************************
This function is the Interrupt Service Routine (ISR), and called when the TWI interrupt is triggered;
that is whenever a TWI event has occurred. This function should not be called directly from the main
application.
****************************************************************************/
ISR(TWI_vect) __attribute__((hot, flatten));
ISR(TWI_vect)
{
  static uint8_t I2C_bufPtr;

  switch (TWSR)
  {

    case I2C_START:             // START has been transmitted
    case I2C_REP_START:         // Repeated START has been transmitted
		I2C_bufPtr = 0;         // Set buffer pointer to the TWI Address location


// Master Transmitter

    case I2C_MTX_ADR_ACK:       // SLA+W has been transmitted and ACK received
    case I2C_MTX_DATA_ACK:      // Data byte has been transmitted and ACK received
		if (I2C_bufPtr < I2C_msgSize)
		{
			TWDR = I2C_buf[I2C_bufPtr++];
			TWCR = (1<<TWEN)|                                 // TWI Interface enabled
				   (1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interrupt and clear the flag to send byte
				   (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           //
				   (0<<TWWC);                                 //
		}else                    // Send STOP after last byte
		{
		I2C_statusReg.lastTransOK = true;                 // Set status bits to completed successfully.
			TWCR = (1<<TWEN)|                                 // TWI Interface enabled
				   (0<<TWIE)|(1<<TWINT)|                      // Disable TWI Interrupt and clear the flag
				   (0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|           // Initiate a STOP condition.
				   (0<<TWWC);                                 //
		}
		break;

    case I2C_MTX_ADR_NACK:      // SLA+W has been transmitted and NACK received
    case I2C_MTX_DATA_NACK:     // Data byte has been transmitted and NACK received
	    I2C_state = TWSR;                                 // Store TWSR and automatically sets clears noErrors bit.
	    // Reset TWI Interface and send START.
		TWCR = 	(1<<TWEN)|                          		// Enable TWI-interface and release TWI pins
				(1<<TWIE)|(1<<TWINT)|               		// Enable TWI Interrupt and clear the flag
				(1<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|    		// Send Start.
				(0<<TWWC);
		break;


// Master Receiver

    case I2C_MRX_DATA_ACK:      // Data byte has been received and ACK transmitted
		I2C_buf[I2C_bufPtr++] = TWDR;

    case I2C_MRX_ADR_ACK:       // SLA+R has been transmitted and ACK received
		if (I2C_bufPtr < (I2C_msgSize-1) )                  // Detect the last byte to NACK it.
		{
			TWCR = (1<<TWEN)|                                 // TWI Interface enabled
				   (1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interrupt and clear the flag to read next byte
				   (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // Send ACK after reception
				   (0<<TWWC);                                 //
		}else                    // Send NACK after next reception
		{
			TWCR = (1<<TWEN)|                                 // TWI Interface enabled
				   (1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interrupt and clear the flag to read next byte
				   (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // Send NACK after reception
				   (0<<TWWC);                                 //
		}
		break;

    case I2C_MRX_DATA_NACK:     // Data byte has been received and NACK transmitted
		I2C_buf[I2C_bufPtr] = TWDR;
		I2C_statusReg.lastTransOK = true;                 // Set status bits to completed successfully.
		TWCR = (1<<TWEN)|                                 // TWI Interface enabled
			   (0<<TWIE)|(1<<TWINT)|                      // Disable TWI Interrupt and clear the flag
			   (1<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|           // Initiate a STOP condition.
			   (0<<TWWC);                                 //
		break;



    case I2C_MRX_ADR_NACK:      // SLA+R has been transmitted and NACK received
 	    I2C_state = TWSR;                                 // Store TWSR and automatically sets clears noErrors bit.
		// Reset TWI Interface and send STOP.
		TWCR = 	(1<<TWEN)|                          		// Enable TWI-interface and release TWI pins
				(1<<TWIE)|(1<<TWINT)|               		// Enable TWI Interrupt and clear the flag
				(1<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|    		// Send stop.
				(0<<TWWC);
		break;

// Slave Transmitter

	case I2C_STX_ADR_ACK:			// Own SLA+R has been received; ACK has been returned
	case I2C_STX_ADR_ACK_M_ARB_LOST:// Arbitration lost in SLA+R/W as Master; own SLA+R has
									// been received; ACK has been returned
	    I2C_bufPtr   = 0;	    	// Set buffer pointer to first data location


	case I2C_STX_DATA_ACK:		// Data byte in TWDR has been transmitted; ACK has been received
	    TWDR = I2C_buf[I2C_bufPtr++];
	    						// Enable TWI Interrupt and clear the flag to send byte
	    TWCR = 	(1<<TWEN) |
	    		(1<<TWIE)|(1<<TWINT)|
	    		(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|
	    		(0<<TWWC);
	    break;


	case I2C_STX_DATA_NACK:  	// Data byte in TWDR has been transmitted; NACK has been received.
	    						// I.e. this could be the end of the transmission.
	    if (I2C_bufPtr == I2C_msgSize) 	    // Have we transmitted all expected data?
	    {
	        I2C_statusReg.lastTransOK = true; // Set status bits to completed successfully.
	    }
	    else
	    {
	        I2C_state = TWSR;     // Master has sent a NACK before all data where sent, Store I2C State as error message.
	    }

	    // Put I2C Transceiver in passive mode.
	    // Enable I2C-interface and release I2C pins
	    TWCR = (1<<TWEN)|
	           (0<<TWIE)|(0<<TWINT)|                // Disable Interrupt
	           (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|     // Do not acknowledge on any new requests.
	           (0<<TWWC);
	   break;


    case I2C_STX_DATA_ACK_LAST_BYTE: // Last data byte in TWDR has been transmitted TWEA = ; ACK has been received

    	// Enable TWI Interrupt and clear the flag to send byte
        TWCR = (1<<TWEN)|                          // Enable TWI-interface and release TWI pins
               (1<<TWIE)|(1<<TWINT)|               // Enable TWI Interrupt and clear the flag to send byte
               (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|    // Acknowledge on any new requests.
               (0<<TWWC);

	    break;

// Slave Receiver

	case I2C_SRX_GEN_ACK:				// General call address has been received; ACK has been returned
	case I2C_SRX_GEN_ACK_M_ARB_LOST:	// Arbitration lost in SLA+R/W as Master; General call
										// address has been received; ACK has been returned
	    I2C_statusReg.genAddressCall = true;

	case I2C_SRX_ADR_ACK:		// Own SLA+W has been received ACK has been returned
	case I2C_SRX_ADR_ACK_M_ARB_LOST:	// Arbitration lost in SLA+R/W as Master; own SLA+W
										// has been received; ACK has been returned

	    I2C_statusReg.RxDataInBuf = true; // Don't need to clear I2C_statusRegister.generalAddressCall due to that it is the default state.

	    I2C_bufPtr   = 0;	    // Set buffer pointer to first data location

	    // Reset the TWI Interrupt to wait for a new event.

	    // TWI Interface enabled
	    // Enable TWI Interrupt and clear the flag to send byte
	    // Expect ACK on this transmission
	    TWCR =  (1<<TWEN)|
	    		(1<<TWIE)|(1<<TWINT)|
	    		(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|
	    		(0<<TWWC);
		break;

	case I2C_SRX_ADR_DATA_ACK:	// Previously addressed with own SLA+W; data has been received; ACK has been returned
	case I2C_SRX_GEN_DATA_ACK:	// Previously addressed with general call; data has been received; ACK has been returned

		I2C_buf[I2C_bufPtr++] = TWDR;

		I2C_statusReg.lastTransOK = true;		// Set flag transmission successful.

		// Reset the TWI Interrupt to wait for a new event.
		TWCR = (1<<TWEN)|                          // TWI Interface enabled
		       (1<<TWIE)|(1<<TWINT)|               // Enable TWI Interrupt and clear the flag to send byte
		       (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|    // Send ACK after next reception
		       (0<<TWWC);
		break;


    case I2C_SRX_ADR_DATA_NACK: // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
    case I2C_SRX_GEN_DATA_NACK: // Previously addressed with general call; data has been received; NOT ACK has been returned

    	// NOT ACK back at the Master
        TWCR = (1<<TWEN)|                          // Enable TWI-interface and release TWI pins
               (1<<TWIE)|(1<<TWINT)|               // Enable TWI Interrupt and clear the flag to send byte
               (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|    // Do not acknowledge on any new requests.
               (0<<TWWC);
		break;

    case I2C_SRX_STOP_RESTART:  // A STOP condition or repeated START condition has been received while still addressed as Slave

    	I2C_checkBusyAfterStop = I2C_HOW_MANY_BUSY_CHECKS_AFTER_STOP; // do some busy checks before hitting the bus again.

        // Put TWI Transceiver in passive mode.
        TWCR = (1<<TWEN)|                          // Enable TWI-interface and release TWI pins
               (0<<TWIE)|(0<<TWINT)|               // Disable Interrupt
               (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|    // Do not acknowledge on any new requests.
               (0<<TWWC);
        break;


// ERRORS AND FAULT CONDITIONS

    case I2C_ARB_LOST:          						// Arbitration lost
		TWCR = (1<<TWEN)|                               // TWI Interface enabled
			   (1<<TWIE)|(1<<TWINT)|                    // Enable TWI Interrupt and clear the flag
			   (1<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|         // Initiate a (RE)START condition.
			   (0<<TWWC);                               //
		break;


    case I2C_BUS_ERROR:         // Bus error due to an illegal START or STOP condition
    case I2C_NO_STATE:          // No relevant state information available TWINT = 0

    default:
		I2C_state = TWSR;                               // Store TWSR and automatically sets clears noErrors bit.

														// Reset TWI Interface
		TWCR = (1<<TWEN)|                          		// Enable TWI-interface and release TWI pins
			 (1<<TWIE)|(1<<TWINT)|               		// Enable TWI Interrupt and clear the flag
			 (1<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|    		// Acknowledge on any new requests. Send stop.
			 (0<<TWWC);                               	//
      	break;
  }
}
