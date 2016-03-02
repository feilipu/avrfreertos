/*****************************************************************************
*
* File              : i2c_multiMaster.h
* Compiler          : avr gcc
* Description       : Header file for i2cMultiMaster.c
*                     Include this file in the application.
*
****************************************************************************/
#ifndef I2CMULTIMASTER_H
#define I2CMULTIMASTER_H

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
  I2C Status/Control register definitions
****************************************************************************/

#define ARDUINO			0x32	  	// device address of Freetronics / Arduino (just making this up, not special)

#define I2C_BUFFER_SIZE 24			// xxx NORMALLY Set this to the largest message size that will be sent including address byte.

//#define I2C_TWPS		0x00        // Not used. This driver presumes prescaler = 00

/* I2C clock in Hz */
//#define SCL_CLOCK		400000L		// HIGH speed. Not all devices will do HIGH speed.
//#define SCL_CLOCK		100000L		// LOW speed. This is the default speed.
#define SCL_CLOCK		153600L		// maximum reliable speed for DS1307 at 22.1184MHz F_CPU


/* A value of 0 turns off this feature. Greater values are slower but more reliable. Try 4 */
#define I2C_HOW_MANY_BUSY_CHECKS_AFTER_STOP     4


/****************************************************************************
  Global definitions
****************************************************************************/

union I2C_statusReg                       // Status byte holding flags.
{
    uint8_t all;
    struct
    {
        uint8_t lastTransOK:1;
        uint8_t RxDataInBuf:1;
        uint8_t genAddressCall:1;   // TRUE = General call, FALSE = TWI Address;
        uint8_t unusedBits:5;
    };
};

extern union I2C_statusReg I2C_statusReg; // DEFINED THIS IN THE LIBRARY.

/* Create a Semaphore binary flag for the i2c Bus. To ensure only single access. */
extern SemaphoreHandle_t xI2CSemaphore;

/****************************************************************************
  Function definitions
****************************************************************************/
void I2C_Slave_Initialise( uint8_t );
void I2C_Slave_Start_Transceiver( void );
void I2C_Slave_Start_Transceiver_With_Data( uint8_t *, uint8_t );
uint8_t I2C_Slave_Get_Data_From_Transceiver( uint8_t *, uint8_t );

void I2C_Master_Initialise( uint8_t );
void I2C_Master_Start_Transceiver( void );
void I2C_Master_Start_Transceiver_With_Data( uint8_t *, uint8_t );
uint8_t I2C_Master_Get_Data_From_Transceiver( uint8_t *, uint8_t );

uint8_t I2C_Check_Free_After_Stop (void);
uint8_t I2C_Get_State_Info( void );

/****************************************************************************
  Bit and byte definitions
****************************************************************************/
#define I2C_READ_BIT  0   // Bit position for R/W bit in "address byte".
#define I2C_ADR_BITS  1   // Bit position for LSB of the slave address bits in the init byte.
#define I2C_GEN_BIT   0   // Bit position for LSB of the general call bit in the init byte.

#define I2C_READ    1	  //defines the data direction (reading from I2C device)
#define I2C_WRITE   0	 // defines the data direction (writing to I2C device)


/****************************************************************************
  TWI State codes
****************************************************************************/
// General TWI Master status codes
#define I2C_START                  0x08  // START has been transmitted
#define I2C_REP_START              0x10  // Repeated START has been transmitted
#define I2C_ARB_LOST               0x38  // Arbitration lost

// TWI Master Transmitter status codes
#define I2C_MTX_ADR_ACK            0x18  // SLA+W has been transmitted and ACK received
#define I2C_MTX_ADR_NACK           0x20  // SLA+W has been transmitted and NACK received
#define I2C_MTX_DATA_ACK           0x28  // Data byte has been transmitted and ACK received
#define I2C_MTX_DATA_NACK          0x30  // Data byte has been transmitted and NACK received

// TWI Master Receiver status codes
#define I2C_MRX_ADR_ACK            0x40  // SLA+R has been transmitted and ACK received
#define I2C_MRX_ADR_NACK           0x48  // SLA+R has been transmitted and NACK received
#define I2C_MRX_DATA_ACK           0x50  // Data byte has been received and ACK transmitted
#define I2C_MRX_DATA_NACK          0x58  // Data byte has been received and NACK transmitted

// TWI Slave Transmitter status codes
#define I2C_STX_ADR_ACK            0xA8  // Own SLA+R has been received; ACK has been returned
#define I2C_STX_ADR_ACK_M_ARB_LOST 0xB0  // Arbitration lost in SLA+R/W as Master; own SLA+R has
                                         // been received; ACK has been returned
#define I2C_STX_DATA_ACK           0xB8  // Data byte in TWDR has been transmitted; ACK has been
                                         // received
#define I2C_STX_DATA_NACK          0xC0  // Data byte in TWDR has been transmitted; NOT ACK has
                                         // been received
#define I2C_STX_DATA_ACK_LAST_BYTE 0xC8  // Last data byte in TWDR has been transmitted
                                         // (TWEA = 0); ACK has been received

// TWI Slave Receiver status codes
#define I2C_SRX_ADR_ACK            0x60  // Own SLA+W has been received ACK has been returned
#define I2C_SRX_ADR_ACK_M_ARB_LOST 0x68  // Arbitration lost in SLA+R/W as Master; own SLA+W
                                         // has been received; ACK has been returned
#define I2C_SRX_GEN_ACK            0x70  // General call address has been received; ACK has been
                                         // returned
#define I2C_SRX_GEN_ACK_M_ARB_LOST 0x78  // Arbitration lost in SLA+R/W as Master; General call
                                         // address has been received; ACK has been returned
#define I2C_SRX_ADR_DATA_ACK       0x80  // Previously addressed with own SLA+W; data has been
                                         // received; ACK has been returned
#define I2C_SRX_ADR_DATA_NACK      0x88  // Previously addressed with own SLA+W; data has been
                                         // received; NOT ACK has been returned
#define I2C_SRX_GEN_DATA_ACK       0x90  // Previously addressed with general call; data has been
                                         // received; ACK has been returned
#define I2C_SRX_GEN_DATA_NACK      0x98  // Previously addressed with general call; data has been
                                         // received; NOT ACK has been returned
#define I2C_SRX_STOP_RESTART       0xA0  // A STOP condition or repeated START condition has been
                                         // received while still addressed as Slave

// TWI Miscellaneous status codes
#define I2C_NO_STATE               0xF8  // No relevant state information available; TWINT = 0
#define I2C_BUS_ERROR              0x00  // Bus error due to an illegal START or STOP condition

#ifdef __cplusplus
}
#endif

#endif /* I2CMULTIMASTER_H */
