
#ifndef sm130_h
#define sm130_h

#ifdef __cplusplus
extern "C" {
#endif


/**	Drivers for a SM130 RFID Module
 *
 *  <a href="http://www.sonmicro.com/en/index.php?option=com_content&view=article&id=57&Itemid=70">SonMicro SM130 RFID module</a>.
 *
 *	Nearly complete implementation of the <a href="http://www.sonmicro.com/en/downloads/Mifare/ds_SM130.pdf">SM130 datasheet</a>.<br>
 *
 */

#include <i2cMultiMaster.h>

// Controls a SonMicro SM130 RFID reader by I2C

// Define the addresses of the devices that we want to manage on i2c bus.

#define ARDUINO			0x32	  // device address of Freetronics / Arduino (just making this up, no magic)
#define SM130_RFID		0x42      // default device address of SM130 Unit

// Pin Definitions
// These pin definitions have been changed from the Sparkfun Defaults.
// Wanted to put DREADY on PD3 to use the External Interrupt INT1
// Also needed to clear conflicts the SPI bus, by moving Pin 11 to Pin 13.

// SCL - PC4 (A4)
// SDA - PC5 (A5)
// DREADY - PD3 (INT1)
// RESET - PD4
// CREAD (SEARCH) LED - PD5
// TAGF (FOUND) LED - PD6

// DREADY/O1: Data Ready Pin. Functional if the Seek For Tag command was executed previously.
// A Logic 1 pulse will be sent as soon as valid Mifare Tag enters into the RF Field.
// Useful to generate interrupt on Master MCU instead of polling I2C continuously.

// RESET. In case a communication problem occurs that Master cannot handle, a hardware reset pin might be useful.

// CREAD: This pin indicates status of Seek for Tag command.
// If it is logic high, then it means the Continuous Read (Seek For Tag)
// is active and module is searching for Mifare Tag continuously.

// TAGF: Same as Data Ready Pin but with longer pulse.
// This pin can trigger external device, drive buzzer circuit or
// simply can be connected to LED indicating there is a valid tag in the field.


#define DREADY             IO_D3 // 3
#define RESET	           IO_D4 // 4
#define CREAD              IO_D5 // 5
#define TAGF               IO_D6 // 6

// This is PD3 or the INT1 pin on Arduino, which is also known as DREADY (active high)
#define SM130_DREADY_ISR_DISABLE()	(EICRA &= ~(_BV(ISC11) | _BV(ISC10))); (EIMSK &= ~_BV(INT1))
#define SM130_DREADY_ISR_ENABLE()	(EICRA |=   _BV(ISC11) | _BV(ISC10));  (EIMSK |=  _BV(INT1))

// DREADY Interrupt (INT1)
#define DREADY_high()		(DDRD |=  _BV(DDD3)); (PORTD |=  _BV(PORTD3))
#define DREADY_low()	    (DDRD |=  _BV(DDD3)); (PORTD &= ~_BV(PORTD3))
#define DREADY_input()		(DDRD &= ~_BV(DDD3)); (PORTD &= ~_BV(PORTD3))

// RESET sm130
#define RESET_on()			(DDRD |=  _BV(DDD4)); (PORTD |=  _BV(PORTD4))
#define RESET_off()			(DDRD |=  _BV(DDD4)); (PORTD &= ~_BV(PORTD4))
#define RESET_input()		(DDRD &= ~_BV(DDD4)); (PORTD &= ~_BV(PORTD4))

// Green Searching LED
#define LEDSearch_on()		(DDRD |=  _BV(DDD5)); (PORTD |=  _BV(PORTD5))
#define LEDSearch_off()	    (DDRD |=  _BV(DDD5)); (PORTD &= ~_BV(PORTD5))
#define LEDSearch_input()	(DDRD &= ~_BV(DDD5)); (PORTD &= ~_BV(PORTD5))

// Yellow Found LED
#define LEDFound_on()		(DDRD |=  _BV(DDD6)); (PORTD |=  _BV(PORTD6))
#define LEDFound_off()	    (DDRD |=  _BV(DDD6)); (PORTD &= ~_BV(PORTD6))
#define LEDFound_input()	(DDRD &= ~_BV(DDD6)); (PORTD &= ~_BV(PORTD6))


// COMMAND CODES

#define CMD_RESET           0x80	// Resets the Module
#define CMD_FIRMWARE_REV    0x81	// Reads the Firmware Revision of the Module
#define CMD_SEEK_TAG        0x82	// Continuously seeks for presence of, and selects a Tag
#define CMD_SELECT_TAG      0x83	// Selects a Tag
// Not Implemented 			0x84
#define CMD_AUTHENTICATE    0x85	// Authenticates the selected Block
#define CMD_READ16          0x86	// Reads from the specified Block
#define CMD_READ_VALUE      0x87	// Reads from a Value Block
// Not Implemented 			0x85
#define CMD_WRITE16         0x89 	// Writes the data to the specified block
#define CMD_WRITE_VALUE     0x8a	// Formats and Writes a Value block
#define CMD_WRITE4          0x8b	// Writes 4 byte data to Mifare Ultralight block
#define CMD_WRITE_KEY       0x8c	// Writes the Key to the EEPROM of the MFRC530
#define CMD_INC_VALUE       0x8d	// Increments a value block
#define CMD_DEC_VALUE       0x8e	// Decrements a value block
// Not Implemented 			0x8f
#define CMD_ANTENNA_POWER   0x90	// Switches ON or OFF the RF field
#define CMD_READ_PORT       0x91	// Reads from the Input port
#define CMD_WRITE_PORT      0x92	// Writes to the Output port
#define CMD_HALT_TAG        0x93	// Halts the PICC (tag)
#define CMD_SET_BAUD        0x94	// Sets the new baud rate
// Not Implemented 			0x95
#define CMD_SLEEP           0x96	// This command puts SM130 in sleep mode
// Not Implemented			0x97-0x99
#define CMD_CHANGE_I2C		0x9a	// Changes the I2C Slave Address
#define CMD_READ_I2C		0x9c	// Reads the I2C Slave Address
// Not Implemented			0x9d-0xff

// RESPONSE CODES

#define RESP_RF_OFF			0x00	// RF Field OFF
#define RESP_RF_ON			0x01	// RF Field ON

#define RESP_INVALID_KEY	0x45 	// Invalid key format in E2PROM
#define RESP_READ_FAIL		0x46 	// Read Failed or Write Failed
#define RESP_INVALID_VALUE	0x49	// Invalid Value Block

#define RESP_SUCCESS		0x4c	// Login Success or Write success or Key Write success.
#define RESP_CMD_EXCECUTING	0x4c	// Command in progress.

#define RESP_FAIL			0x4e	// No Tag present or Login fail or Key Write fail.

#define RESP_FAIL_RF_OFF	0x55	// Access failed due to RF Field is OFF
#define RESP_W_R_FAIL		0x55	// Read after write failed (*)

#define RESP_READ_PROTECTED	0x58	// Unable to Read after write (*)

// KEY TYPE CODES

#define KEY_A				0xaa	// Master Key type A
#define KEY_B				0xbb	// Master Key type B
#define KEY_TRANSPORT		0xff	// Transport Key

// TAG TYPES (returned from sm130_seekTag)

#define MIFARE_ULTRALIGHT   0x01
#define MIFARE_1K           0x02
#define MIFARE_4K           0x03
#define UNKNOWN_TAG			0xff


// signal the vuIP_TASK to resume execution.
xSemaphoreHandle xSM130IntrSemaphore;

// structure to pass the I2C Command Frames, for send and receive.
typedef struct
{
	uint8_t		I2CAddress; // This is the slave address of the sm130 device (doesn't change).

	uint8_t		CMD_Length; // This byte is used to indicate the length of the payload data.
							// This includes the command and the data bytes.
							// When receiving, this byte is used to indicate the length of the payload data.
						    // The master should first analyse this byte and then consequently read the number of bytes
							// indicated by this byte.

	uint8_t		CMD_Code;	// This byte is used to instruct the module on what operation to perform

	uint8_t     CMD_Data[I2C_BUFFER_SIZE - 4];
							// These are parameters necessary for the module to execute the command.
							// For example, for a Read command, the data will be the block number to be read.
							// For a Write command, this will be the block number and 16 bytes of data.
							// For example, for a Read command, the data will be the block number to be read.
							// Last byte following the command data will be the checksum.
} xCMDArray, *pCMDArray;


// this is the definition of the library of keys
//   use pgm_read_byte(&keySpace[y][x]) to get them out of PROGMEM.
extern const uint8_t PROGMEM keySpace[][6]; // put these library keys in PROGMEM, to save RAM.


/*------------ Function Definitions ---------------*/

//! Hardware or software reset of the SM130 module, 0 will do soft reset.
void sm130_reset(uint8_t hardRESET);

//! Software reset of the SM130 module.
uint8_t sm130_softReset(void);

//! Returns a null-terminated string with the firmware version of the SM130 module
uint8_t sm130_getFirmwareVersion(uint8_t *version);

//! Sends a SEEK_TAG command
uint8_t sm130_seekTag(uint8_t *tag);

//! Sends a SELECT_TAG command
uint8_t sm130_selectTag(uint8_t *tag);

//! Sends an AUTHENTICATE command using the specified key
uint8_t sm130_authenticate(uint8_t block, uint8_t keyType, uint8_t *key);

//! Sends an AUTHENTICATE command using the predefined Transport key key
uint8_t sm130_authenticateTransport(uint8_t block);

//! Reads a 16-uint8_t block
uint8_t sm130_readBlock(uint8_t block, uint8_t *dataBlock);

//! Reads a 16-uint8_t block
//uint8_t sm130_readValue(uint8_t block, uint8_t *dataValue);

//! Writes a 16-uint8_t block
uint8_t sm130_writeBlock(uint8_t block, const uint8_t *dataBlock);

//! Writes a 4-uint8_t block
//uint8_t sm130_writeValue(uint8_t block, const uint8_t *dataValue);

//! Sends an AUTHENTICATE command using the specified key
uint8_t sm130_writeKey(uint8_t block, uint8_t keyType, uint8_t *key);

//! Set antenna power (on/off) returns 1 for success, 0 for failure.
uint8_t sm130_setAntennaPower(uint8_t level);

//! Sends a HALT_TAG command
uint8_t sm130_haltTag(void);

//! Sends a SLEEP command (can only wake-up with hardware reset!)
uint8_t sm130_sleep();


//! Returns true if a response packet is available
//uint8_t sm130_available(void);

//! Returns a pointer to the response packet
//uint8_t* sm130_getRawData(pCMDArray xTransmitData);
//! Returns the last executed command
//uint8_t sm130_getCommand(pCMDArray xTransmitData);
//! Returns the packet length, excluding checksum
//uint8_t sm130_getPacketLength(pCMDArray xTransmitData);
//! Returns a pointer to the packet payload
//uint8_t* sm130_getPayload(pCMDArray xTransmitData);
//! Returns the block number for read/write commands
//uint8_t sm130_getBlockNumber(pCMDArray xTransmitData);
//! Returns a pointer to the read block (with a length of 16 bytes)
//uint8_t* sm130_getBlock(pCMDArray xTransmitData);
//! Returns the tag's serial number as a uint8_t array
//uint8_t* sm130_getTagNumber(pCMDArray xTransmitData);
//! Returns the length of the tag's serial number obtained by getTagNumer()
//uint8_t sm130_getTagLength(pCMDArray xTransmitData);
//! Returns the tag's serial number as a hexadecimal
//const char* sm130_getTagString(void);
//! Returns the tag type (SM130::MIFARE_XX)
//uint8_t sm130_getTagType(void);

// utility functions

/**	Convert uint8_t array to null-terminated hexadecimal string.
 *
 *	@param	s	pointer to destination string
 *	@param	array	uint8_t array to convert
 *	@param	len		length of uint8_t array to convert
 */
void arrayToHex(uint8_t *s, uint8_t *array, uint8_t len);

/**	Print byte array as ASCII string.
 *
 *	Non-printable characters (<0x20 or >0x7E) are printed as dot.
 *
 *	@param	array byte array
 *	@param	len length of byte array
 */
void xSerialPrintArrayASCII(uint8_t *array, uint8_t len);


uint8_t getNextLibraryKey(uint8_t *hashKey, uint16_t keyNumber);
void getNextRandomKey(uint8_t *hashKey);
void getNextBruteKey(uint8_t *hashKey);

uint8_t compliantNextKey(uint8_t *hashKey);



#ifdef __cplusplus
}
#endif


#endif // sm130_h
