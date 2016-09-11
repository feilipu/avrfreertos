/*
 * lib_SIM900.c
 *
 *  Created on: 04/10/2013
 *      Author: Phillip Stevens
 */


#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <util/delay.h>
#include <avr/io.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "uIP/sys/ttimer.h"
#include "uIP/sys/stimer.h"

#include "socket.h" // for inet_addr();

/* serial interface include file. */
#include "serial.h"

#include "lib_SIM900Const.h"
#include "lib_SIM900.h"

/* Optionally, create a reference to the handle for the serial port, USART0. */
extern xComPortHandle xSerialPort;
/* Optionally, create a reference to the handle for the other serial port, USART1. */
extern xComPortHandle xSerial1Port;

//////////////// Private Function Declaration //////////

//! It gets if GPRS module is ready or not
/*!
\return nothing. It changes the value of 'not_ready'
 */
static void getIfReady(SIM900StateHandlePtr SIM900_State_Ptr);

//! It checks if GPRS connection is OK
/*!
\param void
\return '1' on GPRS on, '2' on GPRS off, '0' if error.
 */
static uint8_t checkGPRS(SIM900StateHandlePtr SIM900_State_Ptr);

//! Send an AT command to the module
/*!
\param char* theText : string to send to the module
\param char* endOfCommand : string to send to the module
\param char* expectedAnswer : string expected to be answered by the module
\return '1' if expectedAnswer has been detected, '0' if not detected.
 */
static int8_t sendCommand1(SIM900StateHandlePtr SIM900_State_Ptr, const char* theText, const char* expectedAnswer);

//! Wait for data from the module
/*!
\param char* expectedAnswer : string expected to be answered by the module
\param int timeout : specifies the timeout in system_ticks
\return '1' if expectedAnswer has been detected, '0' if not detected.
 */
static int8_t waitForAnswer1(SIM900StateHandlePtr SIM900_State_Ptr, const char* expectedAnswer, const TickType_t timeout);

//! Sends an AT command to the module, functions with two answers
/*!
\param char* theText : string to send to the module
\param char* expectedAnswer1 : string expected to be answered by the module
\param char* expectedAnswer2 : string expected to be answered by the module
\return '1' if expectedAnswer1 has been detected, '2' if expectedAnswer2 has been detected, '0' if not detected.
 */
static int8_t sendCommand2(SIM900StateHandlePtr SIM900_State_Ptr, const char* theText, const char* expectedAnswer1, const char* expectedAnswer2);

//! Wait for data from the module
/*!
\param char* expectedAnswer1 : string 1 expected to be answered by the module
\param char* expectedAnswer2 : string 2 expected to be answered by the module
\param int timeout : specifies the timeout in system_ticks
\return '1' if expectedAnswer1 has been detected, '2' if expectedAnswer2 has been detected, '0' if not detected.
 */
static int8_t waitForAnswer2(SIM900StateHandlePtr SIM900_State_Ptr, const char* expectedAnswer1, const char* expectedAnswer2, const TickType_t timeout);


//////////////////// Private Functions /////////////////////////


/* getIfReady() - gets if GPRS module is ready or not
 *
 * This function gets if GPRS module is ready or not
 *
 * Returns nothing. It changes the value of 'not_ready'
*/
void getIfReady(SIM900StateHandlePtr SIM900_State_Ptr){

	xSerialRxFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// always flush Rx first, otherwise Tx interrupt misbehaves
	xSerialxPrint_P( SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT\r\n") );

	if( waitForAnswer1( SIM900_State_Ptr, ECHO_RESPONSE, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS) == 1 )
		SIM900_State_Ptr->ready = 0; // I am ready
	else
		SIM900_State_Ptr->ready = 1; // I am NOT ready  */
}

/* checkGPRS() - checks if GPRS connection is OK
 *
 * This function checks if GPRS connection is OK
 *
 * Returns '1' on success and '0' if error
*/
uint8_t checkGPRS(SIM900StateHandlePtr SIM900_State_Ptr){

	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// always flush Rx first, otherwise Tx interrupt misbehaves
	xSerialxPrintf_P( SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s\r\n" ), AT_GPRS_CHECK);

	return waitForAnswer2( SIM900_State_Ptr, AT_GPRS_CHECK_ON, AT_GPRS_CHECK_OFF, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS) == 1 ? 1 : 0;
}



//////////////  Private AT Command Functions /////////////////////


int8_t sendCommand1(SIM900StateHandlePtr SIM900_State_Ptr, const char* theText, const char* expectedAnswer){

#if GPRS_debug_mode>0
	xSerialxPrintf_P(&xSerialPort, PSTR("Send command with 1 answer: AT%s\r\n"), theText);
#endif

	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// always flush Rx first, otherwise Tx interrupt misbehaves
	xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR("AT%s\r\n"), theText);

#if GPRS_debug_mode>0
	uint8_t answer = waitForAnswer1( SIM900_State_Ptr, expectedAnswer, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS);
	xSerialxPrintf_P(&xSerialPort, PSTR("Answer: %d\r\n"), answer);
    return answer;
#else
	return waitForAnswer1( SIM900_State_Ptr, expectedAnswer, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS);
#endif


}

int8_t waitForAnswer1(SIM900StateHandlePtr SIM900_State_Ptr, const char* expectedAnswer, const TickType_t timeout){

	uint8_t theLength;
	uint8_t character;
	uint8_t escape_chr = 0;
	uint8_t i = 0;
    tick_timer tick_timer;

	theLength = strlen(expectedAnswer);		// how long is the expected answer?

	if(theLength >= SIM900_State_Ptr->buffer_command_size) theLength = SIM900_State_Ptr->buffer_command_size - 1;

	memset(SIM900_State_Ptr->buffer_command, '\0', theLength);	// so set the minimum number of bytes to null.

	timer_set( &tick_timer, timeout );
    do{
    	if( xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &character) )
    	{
    		if(character == '\r' || character == '\n')
    			++escape_chr;
    		else
    			SIM900_State_Ptr->buffer_command[i++] = character;
      	}
    }while ( (i < theLength) && (escape_chr < 4) && (timer_expired( &tick_timer ) == 0) );

    SIM900_State_Ptr->buffer_command[i] = '\0'; // make sure there's a null terminating byte on received bytes

#if GPRS_debug_mode>0
	xSerialxPrintf_P(&xSerialPort, PSTR("Expected response: %s\r\n"), expectedAnswer);
	xSerialxPrintf_P(&xSerialPort, PSTR("  Actual response: %s\r\n"), SIM900_State_Ptr->buffer_command);
#endif

	if( strncmp((char *)SIM900_State_Ptr->buffer_command, (char *) expectedAnswer, theLength) )
		return 0;
	else
		return 1;
}


//functions with two answers

int8_t sendCommand2(SIM900StateHandlePtr SIM900_State_Ptr, const char* theText, const char* expectedAnswer1, const char* expectedAnswer2){

#if GPRS_debug_mode>0
	xSerialxPrintf_P(&xSerialPort, PSTR("Send command with 2 answers: AT%s\r\n"), theText);
#endif
	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// always flush Rx first, otherwise Tx interrupt misbehaves
	xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR("AT%s\r\n"), theText);

#if GPRS_debug_mode>0
	uint8_t answer = waitForAnswer2( SIM900_State_Ptr, expectedAnswer1, expectedAnswer2, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS);
	xSerialxPrintf_P(&xSerialPort, PSTR("Answer: %d\r\n"), answer);
    return answer;
#else
	return waitForAnswer2( expectedAnswer1, expectedAnswer2, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS);

#endif

return answer;
}

int8_t waitForAnswer2(SIM900StateHandlePtr SIM900_State_Ptr, const char* expectedAnswer1, const char* expectedAnswer2, const TickType_t timeout){

	uint8_t theLength1;
	uint8_t theLength2;
	uint8_t maxLength;

	uint8_t character;
	uint8_t escape_chr = 0;
	uint8_t i = 0;

    tick_timer tick_timer;

	// Gets the maximum length and the minimum length of the 2 strings
	theLength1 = strlen(expectedAnswer1); // how long is the expected answer 1?
	theLength2 = strlen(expectedAnswer2); // how long is the expected answer 2?

	if( theLength1 < theLength2 )
		maxLength = theLength2;
	else
		maxLength = theLength1;

	if(maxLength >= SIM900_State_Ptr->buffer_command_size) maxLength = SIM900_State_Ptr->buffer_command_size - 1;

	memset(SIM900_State_Ptr->buffer_command, '\0', maxLength );	// so set the minimum number of bytes to null.

    timer_set( &tick_timer, timeout );
    do{
    	if( xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &character) )
    	{
    		if(character == '\r' || character == '\n')
    			++escape_chr;
    		else
    			SIM900_State_Ptr->buffer_command[i++] = character;
    	}
    }while ( (i < maxLength) && (escape_chr < 4) && timer_expired( &tick_timer ) == 0);

    SIM900_State_Ptr->buffer_command[i] = '\0'; // make sure there's a null terminating byte on received bytes

#if GPRS_debug_mode>0
	xSerialxPrintf_P(&xSerialPort, PSTR("Expected response: %s -OR- %s\r\n"), expectedAnswer1, expectedAnswer2 );
	xSerialxPrintf_P(&xSerialPort, PSTR("  Actual response: %s\r\n"), SIM900_State_Ptr->buffer_command);
#endif

	if( strncmp((char *)SIM900_State_Ptr->buffer_command, expectedAnswer1, theLength1) )
	{
		if( strncmp((char *)SIM900_State_Ptr->buffer_command, expectedAnswer2, theLength2))
		{
			if (strstr((char *)SIM900_State_Ptr->buffer_command, expectedAnswer1) != NULL)
			{

				if ( ! strcmp(expectedAnswer1, ERROR_CME) || ! strcmp(expectedAnswer1, ERROR_CMS))
				{
					#if GPRS_debug_mode>0
						xSerialxPrint_P(&xSerialPort, PSTR("Special error response 1\r\n"));
					#endif

					return 1;
				}
			}

			if (strstr((char *)SIM900_State_Ptr->buffer_command, expectedAnswer2) != NULL)
			{

				if ( ! strcmp(expectedAnswer2, ERROR_CME) || ! strcmp(expectedAnswer2, ERROR_CMS))
				{
					#if GPRS_debug_mode>0
						xSerialxPrint_P(&xSerialPort, PSTR("Special error response 2\r\n"));
					#endif

					return 2;
				}
			}

			return 0;

		}
		else
			return 2;
	}
	else
		return 1;
}



//////////////////// Public Functions /////////////////////////////////////////

// Power functions

/* SIM900PowerOn(int8_t) - opens USART1 and powers the SIM900 module
 *
 * MUST DO SIM900PowerOn() FIRST, but after creating the SIM900StateHandle in the main task.
 * Opens USART1, allocates command and packet buffers, and powers the SIM900 module
 *
 * Returns SIM900SetMode()
*/
int8_t SIM900PowerOn(SIM900StateHandlePtr SIM900_State_Ptr, eCOMPort ePort, uint8_t command_buffer_size, uint16_t  packet_buffer_size){

	#if GPRS_debug_mode==1
	xSerialxPrint_P(&xSerialPort, PSTR("Debug mode 1\r\n"));
	#endif
	#if GPRS_debug_mode==2
	xSerialxPrint_P(&xSerialPort, PSTR("Debug mode 2\r\n"));
	#endif

	switch(ePort)
	{
	case USART0:
	    // turn on the serial port for communicating with the Arduino GSM Shield SIM900.
		xSerialPort = xSerialPortInitMinimal( USART1, SIM900_RATE, command_buffer_size, packet_buffer_size); //  serial port: USART, WantedBaud, TxQueueLength, RxQueueLength (8n1)
		SIM900_State_Ptr->SIM900SerialPortPtr = &xSerialPort; // must point to xSerialPort, because the USART0 interrupt points there.
		break;
	case USART1:
	default:
    // turn on the other serial port for communicating with the Arduino GSM Shield SIM900.
	xSerial1Port = xSerialPortInitMinimal( USART1, SIM900_RATE, command_buffer_size, packet_buffer_size); //  serial port: USART, WantedBaud, TxQueueLength, RxQueueLength (8n1)
	SIM900_State_Ptr->SIM900SerialPortPtr = &xSerial1Port; // must point to xSerial1Port, because the USART1 interrupt points there.
		break;
	}
	/* Create the buffers used by the serial command functions */
	if( !(SIM900_State_Ptr->buffer_command = (uint8_t *)pvPortMalloc( sizeof(uint8_t) * command_buffer_size )))
		return 0;
	SIM900_State_Ptr->buffer_command_size = command_buffer_size;

	/* Create the buffers used by the serial packet buffer */
	if( !(SIM900_State_Ptr->buffer_packet = (uint8_t *)pvPortMalloc( sizeof(uint8_t) * packet_buffer_size )))
		return 0;
	SIM900_State_Ptr->buffer_packet_size = packet_buffer_size;

	SIM900_State_Ptr->IP_state_actual = PDP_DEACT;

	return (SIM900SetPowerMode(SIM900_State_Ptr, POWER_ON));
}


/* OFF(void) - closes UART1 and powers off the SIM900 module
 *
 * This function closes UART1 and powers off the SIM900 module
 *
 * Returns nothing
*/
void SIM900Off(SIM900StateHandlePtr SIM900_State_Ptr){

	SIM900SetPowerMode(SIM900_State_Ptr, POWER_OFF);

	vPortFree( SIM900_State_Ptr->buffer_packet );
	vPortFree( SIM900_State_Ptr->buffer_command );

	vSerialClose( SIM900_State_Ptr->SIM900SerialPortPtr );
}

/* setMode(uint8_t) - Sets GPRS Power Mode
 *
 * This function selects the active power mode among: ON, SLEEP/HIBERNATE and OFF
 * It does not close the serial port, only sends the proper command to GPRS module
 *
 * Returns '1' on success, '0' if error and '-2' if error with CME error code available
*/
int8_t SIM900SetPowerMode(SIM900StateHandlePtr SIM900_State_Ptr, SIM900_POWER_state pwrMode){

	uint8_t answer = 0;
	seconds_timer secs_timer;

#if GPRS_debug_mode>0
	xSerialxPrintf_P(&xSerialPort, PSTR("SIM900 Setting Power MODE to: %d\r\n"), pwrMode);
#endif

	switch (pwrMode)
	{
	case POWER_ON:

		stimer_set( &secs_timer, 300 ); // try to power on, cycle for five minutes
		do
		{
			DDRB  |= _BV(DDB2) | _BV(DDB3);  // keep the SIM900 NRESET & PERKEY outputs.
			PORTB &= ~(_BV(PORTB2) | _BV(PORTB3)); // keep the SIM900 NRESET & PERKEY low.
			vTaskDelay( 1000 / portTICK_PERIOD_MS );

			PORTB |= _BV(PORTB2);			// set the SIM900 PERKEY high for reset for one second.
			vTaskDelay( 1000 / portTICK_PERIOD_MS );

			PORTB &= ~_BV(PORTB2);			// return the SIM900 PERKEY low and wait for being enabled.
			vTaskDelay(  2250 / portTICK_PERIOD_MS );

			getIfReady(SIM900_State_Ptr);

#if GPRS_debug_mode>0
			xSerialxPrintf_P(&xSerialPort, PSTR("SIM900 not_ready: %d\r\n"), SIM900_State_Ptr->ready);
#endif
		} while ( (SIM900_State_Ptr->ready == 1) && (stimer_expired(&secs_timer) == 0) );

		if (SIM900_State_Ptr->ready == 0) // I am ready so set me up.
		{
			// Disables command echoes: Do this soon as device is ready, because all other responses depend on receiving no echo
			sendCommand1(SIM900_State_Ptr, AT_ECHO_OFF, ECHO_OFF_RESPONSE);
			// Enables numeric error codes:
			sendCommand1(SIM900_State_Ptr, AT_NUMERIC_ERROR, OK_RESPONSE);

			if( sendCommand1(SIM900_State_Ptr, AT_POWER_NO_SLEEP, OK_RESPONSE) != 1 ) return 0;
			answer = sendCommand2(SIM900_State_Ptr, AT_POWER_FULL, OK_RESPONSE, ERROR_CME);
		}

		break;

	case POWER_FULL:
		if( sendCommand1(SIM900_State_Ptr, AT_POWER_NO_SLEEP, OK_RESPONSE) != 1 ) return 0;
		answer = sendCommand2(SIM900_State_Ptr, AT_POWER_FULL, OK_RESPONSE, ERROR_CME);
		break;

	case POWER_RF_OFF:
		if( sendCommand1(SIM900_State_Ptr, AT_POWER_NO_SLEEP, OK_RESPONSE) != 1 ) return 0;
		answer = sendCommand2(SIM900_State_Ptr, AT_POWER_RF_OFF, OK_RESPONSE, ERROR_CME);
		break;

	case POWER_MIN:
		if( sendCommand1(SIM900_State_Ptr, AT_POWER_NO_SLEEP, OK_RESPONSE) != 1 ) return 0;
		answer = sendCommand2(SIM900_State_Ptr, AT_POWER_MIN, OK_RESPONSE, ERROR_CME);
		break;

	case POWER_SLEEP:
		answer = sendCommand1(SIM900_State_Ptr, AT_POWER_SLEEP, OK_RESPONSE);
		break;

	case POWER_OFF:
		SIM900_State_Ptr->state_PWR = pwrMode;
		break;
	}
	if (answer == 1)
	{
		SIM900_State_Ptr->state_PWR = pwrMode;

#if GPRS_debug_mode>0
		xSerialxPrintf_P(&xSerialPort, PSTR("SIM900 Set Power MODE to: %d\r\n"), SIM900_State_Ptr->state_PWR);
#endif
		return 1;
	}
	else if (answer == 2)
	{
		return -(answer);
	}

	return 0;
}

/* getMode(void) - Gets SIM900 Power Mode
 *
 * This function gets the actual SIM900 Power Mode. Possible values are ON, FULL, RF_OFF, MIN, SLEEP and POWER_OFF
 *
 * Returns the power mode
*/
SIM900_POWER_state SIM900GetPowerMode(SIM900StateHandlePtr SIM900_State_Ptr){
	return	SIM900_State_Ptr->state_PWR;
}


/* check(uint8_t) - Checks if GPRS is connected to the network
 *
 * This function checks if GPRS module is connected to the network. If not, there is no sense working with GPRS.
 *
 * It sends a command to GPRS module DEFAULT_TIMEOUT times. If GPRS module does not connect within these tries, function
 * exits. The time is provided in SECONDS, as it make take half a minuted to connect.
 *
 * Returns '1' when connected and '0' if not
*/
uint8_t SIM900CheckNetwork(SIM900StateHandlePtr SIM900_State_Ptr, uint16_t time){

	int8_t answer;
	seconds_timer secs_timer;

    stimer_set( &secs_timer, time );
	do{
		// Sends the command and waits for the answer (0,1 for home network and 0,5 for roaming)
		xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// always flush Rx first, otherwise Tx interrupt misbehaves
		xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR("AT%s\r\n"), AT_REGISTERED);

		answer = waitForAnswer2( SIM900_State_Ptr, "+CREG: 0,1", "+CREG: 0,5", (5 * SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS ) );
	}while ((answer == 0) && ! stimer_expired(&secs_timer));

	if ((answer == 0) || stimer_expired(&secs_timer) ) return 0;

	return 1;
}


//SIM functions
/* setPIN(const char*) - sets PIN to the SIM
 *
 * This function sets the specified PIN to the SIM
 *
 * Returns '1' on success, '0' if error and '-2' if CME error code available
*/
int8_t	SIM900SetPIN(SIM900StateHandlePtr SIM900_State_Ptr, const char* pin){

	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// always flush Rx first, otherwise Tx interrupt misbehaves
	xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR("AT%s\"%s\"\r\n"), AT_PIN, pin);

	int8_t answer = waitForAnswer2(SIM900_State_Ptr, OK_RESPONSE, ERROR_CME, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS);
	return (answer == 2) ? -answer : answer;
}

/* changePIN(const char*, const char*) - changes PIN number to the SIM
 *
 * This function changes the PIN number to the SIM
 *
 * Returns '1' on success, '0' if error and '-2' if CME error code available
*/
int8_t SIM900ChangePIN(SIM900StateHandlePtr SIM900_State_Ptr, const char* old_pin, const char* new_pin){

	// "SC" SIM (lock SIM/UICC card) (SIM/UICC asks password in MT power-up and when this lock command issued)
	// Corresponds to PIN1 code.

	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// always flush Rx first, otherwise Tx interrupt misbehaves
	xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR("AT%s\"SC\",\"%s\",\"%s\"\r\n"), AT_CHANGE_PASSWORD, old_pin, new_pin);

	int8_t answer = waitForAnswer2(SIM900_State_Ptr, OK_RESPONSE, ERROR_CME, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS);
	return answer == 2 ? -answer : answer;
}

/* getIMEI() - gets the IMEI from the SIM card
 *
 * This function gets the IMEI from the SIM card. It stores the IMEI into 'responseIMEI' variable.
 *
 * Returns '1' on success and '0' if error
*/
uint8_t SIM900GetIMEI(SIM900StateHandlePtr SIM900_State_Ptr, char * responseIMEI){

	uint8_t character;
	uint8_t escape_chr = 0;
	uint8_t i = 0;
	tick_timer tick_timer;

	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// always flush Rx first, otherwise Tx interrupt misbehaves
	xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR("AT%s\r\n"), AT_GPRS_IMEI);

    timer_set( &tick_timer, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS );
    do{
    	if( xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &character) )
    	{
    		if(character == '\r' || character == '\n')
    			++escape_chr;
    		else
    			responseIMEI[i++] = character;
      	}
    }while ( (escape_chr < 4) && (i < SIM900_State_Ptr->buffer_command_size - 1) && (timer_expired( &tick_timer ) == 0) );

    responseIMEI[i] = '\0'; // make sure there's a null terminating byte on received bytes

	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// now flush unnecessary "OK"

	return i > 0 ? 1 : 0;
}

/* getIMSI() - gets the IMSI from the SIM card
 *
 * This function gets the IMSI from the SIM card. It stores the IMSI into 'buffer_GPRS' variable.
 *
 * Returns '1' on success, '0' if error and '-2' if CME error code available
*/
int8_t SIM900GetIMSI(SIM900StateHandlePtr SIM900_State_Ptr, char * responseIMSI){

	uint8_t character;
	uint8_t escape_chr = 0;
	uint8_t i = 0;
    tick_timer tick_timer;

	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// always flush Rx first, otherwise Tx interrupt misbehaves
	xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR("AT%s\r\n"), AT_GPRS_IMSI);

    timer_set( &tick_timer, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS );
    do{

    	if( xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &character) )
    	{
    		if(character == '\r' || character == '\n')
    			++escape_chr;
    		else
    			responseIMSI[i++] = character;
    	}

    }while ( (escape_chr < 4) && (i < SIM900_State_Ptr->buffer_command_size - 1) && (timer_expired( &tick_timer ) == 0) );

	if (strstr(responseIMSI, ERROR_CME) != NULL)
	{

	#if GPRS_debug_mode>0
		xSerialxPrint_P(&xSerialPort, PSTR("Special error response 2\r\n"));
	#endif

		return -2;

	}
	return i > 0 ? 1 : 0;
}

#if 0 // fixme

/* manageIncomingData() - manage incoming data from serial port, executing proper functions to store received data
 *
 * This function manages incoming data from serial port, executing proper functions to store received data
 *
 * Returns '1' for call, '2' for SMS, '3' for IP data and '0' for error or not data
*/
int8_t	SIM900ManageIncomingData(SIM900StateHandlePtr SIM900_State_Ptr){

	if ( byteIN == NULL )
	{
		if( ! (byteIN = (uint8_t *)pvPortMalloc( sizeof(uint8_t) * SIM900_PACKET_BUFFER )))
			return 0;
		memset(byteIN, '\0', (size_t)(sizeof(uint8_t) * SIM900_PACKET_BUFFER) );
	}
	uint8_t a=0; //counter and auxiliary variable
	long previous=0;

	serialFlush(_socket);
	previous=millis();
	while (!serialAvailable(_socket) && ((millis()-previous) < 20000));
	previous=millis();
	while ( (millis()-previous) < 2000 )
	{
		while( serialAvailable(_socket) && ((millis()-previous) < 2000) )
		{
			byteIN[a]=serialRead(_socket);
			a++;
		}
	}

	if (a > 0) // this if is conditioned by the fuses selected in WaspSIM900.h
	{
		if ((strstr(byteIN, "+CLIP")) != NULL)
		{
			readCall(byteIN);
			a=1;
		}
		else if ((strstr(byteIN, "+CMTI")) != NULL)
		{
			readSMS(byteIN);
			a=2;
		}
		else
		{
			readIPData(byteIN);
			a=3;
		}

		if ((strstr(byteIN, "+CLIP")) != NULL)
		{
			readCall(byteIN);
			a=1;
		}
		else if ((strstr(byteIN, "+CMTI")) != NULL)
		{
			readSMS(byteIN);
			a=2;
		}

		SIM900ReadIPData(byteIN);
		a=3;
	}
	else
	{
		a=0;
	}
	vPortFree(byteIN);
	if (a == 0)
	{
		return 0;
	}
	return a;
}
#endif

/*switchtoDataMode() - switches from command mode to data mode
 *
 * This function switches from command mode to data modes
 *
 * Returns '1' on success, '0' if error and '-2' if connection is not successfully resumed
*/
int8_t SIM900witchToDataMode(SIM900StateHandlePtr SIM900_State_Ptr){

	xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR("AT%s\r\n"), AT_DATA_MODE);
	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );

	int8_t answer = waitForAnswer2(SIM900_State_Ptr, AT_DATA_MODE_R, AT_DATA_MODE_FAIL, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS);

	return answer == 2 ? -answer : answer;
}

/*switchtoCommandMode() - switches from data mode to command mode
 *
 * This function switches from data mode to command mode
 *
 * Returns '1' on success and '0' if error
*/
int8_t SIM900SwitchToCommandMode(SIM900StateHandlePtr SIM900_State_Ptr){

	vTaskDelay( 1000 / portTICK_PERIOD_MS );	// waits one second after the last data sent

	xSerialxPrint_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR("+++"));
	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );

	vTaskDelay( 500 / portTICK_PERIOD_MS );	// waits one half second after the command

	return waitForAnswer1(SIM900_State_Ptr, OK_RESPONSE, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS) == 1 ? 1 : 0;
}

/* sendCommand(ATcommand) - sends any command to GPRS module
 *
 * This function sends any command to GPRS module
 *
 * It stores in 'buffer_GPRS' variable the answer returned by the GPRS module
 *
 * Returns '1' on success, '0' if error
*/
int8_t SIM900SendCommand(SIM900StateHandlePtr SIM900_State_Ptr, const char* ATcommand){

	uint8_t character;
	uint8_t i = 0;
	uint8_t escape_chr = 0;
	tick_timer tick_timer;

	xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR("AT%s\r\n"), ATcommand);
	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );

    timer_set( &tick_timer, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS );
    do{
    	 if( xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &character) )
    	 {
    		 if(character == '\r' || character == '\n')
    			 ++escape_chr;
    		 else
    			 SIM900_State_Ptr->buffer_command[i++] = character;
    	 }
    } while ( (escape_chr < 4) && (i < SIM900_State_Ptr->buffer_command_size - 1) && (timer_expired( &tick_timer ) == 0) );

    SIM900_State_Ptr->buffer_command[i] = '\0';

	return i > 0 ? 1 : 0; // expecting an OK, or something substantial.
}

/* getcurrentOperator() - Gets the currently selected operator from network
 *
 * This function gets the currently selected operator from network and stores it in 'operator_name'
 *
 * Returns '1' on success, '0' if error and '-2' if CME error code available
*/
int8_t SIM900GetCurrentOperator(SIM900StateHandlePtr SIM900_State_Ptr){

	int8_t answer;
	uint8_t character;
	uint8_t i = 0;
	uint8_t escape_chr = 0;
    tick_timer tick_timer;

	if( (answer = sendCommand2(SIM900_State_Ptr, AT_GET_OPERATOR, AT_GET_OPERATOR_R, ERROR_CME)) != 0)
	{
		timer_set( &tick_timer, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS );
		do{
			if( xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &character) )
			{
				if(character == '\r' || character == '\n')
	    			++escape_chr;
				else
					SIM900_State_Ptr->buffer_command[i++]=character;
			}
		}while ((escape_chr < 2) && (i < SIM900_State_Ptr->buffer_command_size - 1) && (timer_expired( &tick_timer ) == 0));

		SIM900_State_Ptr->buffer_command[i]='\0';
	}
	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// now flush unnecessary "OK"

	return answer == 2 ? -answer : answer;
}

#if 0
/* getAvailableOperators() - Gets the currently available operators from network
 *
 * This function gets the currently available operators from network and stores it in 'operators_list'
 *
 * Returns '1' on success, '0' if error and '-2' if CME error code available
*/
int8_t SIM900GetAvailableOperators(SIM900StateHandlePtr SIM900_State_Ptr){

	uint8_t answer=0;
	int i,j,aux;
	long previous;

	serialFlush(_socket);

	answer=sendCommand2(AT_OPERATOR_LIST, AT_OPERATOR_LIST_R, ERROR_CME);
	if (answer == 2)
	{
		return -2;
	}
	else if (answer == 0)
	{
		return 0;
	}


	previous=millis();
	j=0;
	do{
		while ((serialRead(_socket) !=  ',') && ((millis()-previous)< 10000)); // Gets format used
		i=0;
		aux=serialRead(_socket);
		do{
			while ((serialAvailable(_socket) == 0) && (millis()-previous < 10000));
			operators_list[j].format=aux-0x30;
			aux=serialRead(_socket);
		}while ((aux != ',') && (millis()-previous < 10000));
		while ((serialRead(_socket)!='"') && (millis()-previous < 10000)); // Gets operator name
		i=0;
		do{
			while ((serialAvailable(_socket) == 0) && (millis()-previous < 10000));
			operators_list[j].operator_name[i]=serialRead(_socket);
			i++;
		}while ((operators_list[j].operator_name[i-1] != '"') && (millis()-previous < 10000));
		operators_list[j].operator_name[i-1]='\0';
		j++;
	}while ((waitForData(SIM900_State_Ptr, "OK", AT_OPERATOR_LIST_R, 20, 0, 0) == 2)  && (millis()-previous < 10000) && (j < 5));

	return 1;
}
#endif

/* setPreferredOperator(int, uint8_t, const char*) - Sets the preferred operator in the operators list into GPRS module
 *
 * This function sets the preferred operator in the operators list into GPRS module
 *
 * Returns '1' on success, '0' if error and '-2' if CME error code available
*/
int8_t SIM900SetPreferredOperator(SIM900StateHandlePtr SIM900_State_Ptr, int index, uint8_t format, const char* preferred_operator){

	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// always flush Rx first, otherwise Tx interrupt misbehaves
	xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s%u,%u,\"%s\"\r\n" ), AT_SET_PREF_OPERATOR, index, format, preferred_operator);

	int8_t answer = waitForAnswer2( SIM900_State_Ptr, OK_RESPONSE, ERROR_CME, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS);

	return answer == 2 ? -answer : answer;
}
#if 0
/* getCellInfo() - gets the information from the cell where the module is connected
 *
 * This function gets the information from the cell where the module is connected
 *
 * It stores in 'RSSI' and 'cellID' variables the information from the cell
 *
 * Returns '1' on success, '0' if error and '-2' if CME error code available
*/
int8_t SIM900GetCellInfo(SIM900StateHandlePtr SIM900_State_Ptr){

	unsigned long previous=millis();
	uint8_t counter=0;
	uint8_t a,b=0;
	int answer=0;


	sprintf(buffer_GPRS, "%s=1", AT_GPRS_CELLID);
	answer=sendCommand2(buffer_GPRS, OK_RESPONSE, ERROR);
	if (answer != 1)
	{
		return 0;
	}

	serialFlush(_socket);
	sprintf(buffer_GPRS, "AT%s?\r\n", AT_GPRS_CELLID);
	printString(buffer_GPRS, _socket);

	while ( (!serialAvailable(_socket)) && ((millis()-previous) < 3000) );
	previous=millis();
	a=0;
	while ( (millis()-previous) < 2000 )
	{
		while ((serialAvailable(_socket) && (millis()-previous) < 2000) && (a < 200))
		{
			buffer_GPRS[a]=serialRead(_socket);
			a++;
		}
	}

	a=0;

	counter=0;
	while ( counter < 8 )
	{
		while ((buffer_GPRS[a] != ',') && (a < 200))
		{
			a++;
		}
		a++;
		counter++;
	}
	// Gets cellID
	b=0;
	while ((buffer_GPRS[a] != ',') && (a < 200))
	{
		cellID[b]=buffer_GPRS[a];
		a++;
		b++;
	}
	a++;
	cellID[b]='\0';

	sprintf(buffer_GPRS, "%s=0", AT_GPRS_CELLID);
	answer=sendCommand2(buffer_GPRS, OK_RESPONSE, ERROR);
	if (answer != 1)
	{
		return 0;
	}


	answer=sendCommand2(AT_GPRS_RSSI, AT_GPRS_RSSI, ERROR_CME);
	if (answer == 2)
	{
		return -2;
	}
	else if (answer == 0)
	{
		return 0;
	}

	RSSI=0; // Gets RSSI value, converts it from ASCII to -dBm

	previous=millis();
	while ((serialRead(_socket) != ' ') && ((millis()-previous) < 3000));

	b=serialRead(_socket);
	previous=millis();

	do{
		RSSI*=10;
		RSSI+=(b-0x30);
		b=serialRead(_socket);
	}while ((b != ',') && ((millis()-previous) < 3000));

	#if GPRS_debug_mode>0
		xSerialxPrint_P(&xSerialPort, PSTR("Raw RSSI: "));
		USB.println(RSSI, DEC);
	#endif

	switch(RSSI)
	{
		case 0:
			RSSI= -115;
			break;

		case 1:
			RSSI= -111;
			break;

		case 31:
			RSSI= -52;
			break;

		case 99:
			RSSI= 0;
			break;

		default:
			RSSI= ((RSSI - 2) * 2) - 110;
	}

	#if GPRS_debug_mode>0
		xSerialxPrint_P(&xSerialPort, PSTR("Processed RSSI: "));
		USB.println(RSSI, DEC);
	#endif

	return 1;
}
#endif

/* SIM900GetIP() - Gets IP address when configuring TCP/UDP profiles
 *
 * This function gets IP address as a string when configuring TCP/UDP profiles and stores it in 'SIM900_State_Ptr->buffer_command'
 *
 * Returns '1' on success and '0' if error
*/
int8_t SIM900GetIP(SIM900StateHandlePtr SIM900_State_Ptr){

	uint8_t character;
	uint8_t i = 0;
	uint8_t escape_chr = 0;
	tick_timer tick_timer;

	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );
	xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR("AT%s\r\n"), AT_IP_GET_IP);

    timer_set( &tick_timer, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS );
    do{
    	 if( xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &character) )
    	 {
    		 if(character == '\r' || character == '\n')
    			 ++escape_chr;
    		 else
    			 if( ((0x2F < character) && (character < 0x3A)) || (character == '.') )// Only captures character when it's a ascii number or a dot
    				 SIM900_State_Ptr->buffer_command[i++] = character;
    	 }
    } while ( (escape_chr < 4) && (i < SIM900_State_Ptr->buffer_command_size - 1) && (timer_expired( &tick_timer ) == 0) );

    SIM900_State_Ptr->buffer_command[i] = '\0';

	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );

	return i > 0 ? 1 : 0;
}

/* SIM900CheckIPstatus() - get the status of the IP connection
 *
 * This function gets the status of the IP connection
 *
 * Returns a number with the state, and updates the "IP_state_actual"
*/
SIM900_IP_state SIM900CheckIPstatus(SIM900StateHandlePtr SIM900_State_Ptr){

	uint8_t character;
	uint8_t i = 0;
	uint8_t escape_chr = 0;
    tick_timer tick_timer;

	if (sendCommand1(SIM900_State_Ptr, AT_IP_STATUS, OK_RESPONSE))	// Sends the command:
	{
		timer_set( &tick_timer, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS );
		do{
			if( xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &character) )
			{
				if(character == '\r' || character == '\n')
	    			++escape_chr;
				else
					SIM900_State_Ptr->buffer_command[i++]=character;
			}
		}while ((escape_chr < 6) && (i < SIM900_State_Ptr->buffer_command_size - 1) && (timer_expired( &tick_timer ) == 0));

		SIM900_State_Ptr->buffer_command[i]='\0';
	}

	if (strstr((char *)SIM900_State_Ptr->buffer_command, "IP INITIAL") != NULL)
		SIM900_State_Ptr-> IP_state_actual = IP_INITIAL;

	else if (strstr((char *)SIM900_State_Ptr->buffer_command, "IP START") != NULL)
		SIM900_State_Ptr-> IP_state_actual = IP_START;

	else if (strstr((char *)SIM900_State_Ptr->buffer_command, "IP CONFIG") != NULL)
		SIM900_State_Ptr-> IP_state_actual = IP_CONFIG;

	else if (strstr((char *)SIM900_State_Ptr->buffer_command, "IP GPRSACT") != NULL)
		SIM900_State_Ptr-> IP_state_actual = IP_GPRSACT;

	else if ((strstr((char *)SIM900_State_Ptr->buffer_command, "IP STATUS") != NULL) || (strstr((char *)SIM900_State_Ptr->buffer_command, "IP PROCESSING") != NULL))
		SIM900_State_Ptr-> IP_state_actual = IP_PROCESSING;

	else if ((strstr((char *)SIM900_State_Ptr->buffer_command, "TCP CONNECTING") != NULL) || (strstr((char *)SIM900_State_Ptr->buffer_command, "UDP CONNECTING") != NULL) || (strstr((char *)SIM900_State_Ptr->buffer_command, "SERVER LISTENING") != NULL))
		SIM900_State_Ptr-> IP_state_actual = CONNECTING_LISTENING;

	else if (strstr((char *)SIM900_State_Ptr->buffer_command, "CONNECT OK") != NULL)
		SIM900_State_Ptr-> IP_state_actual = CONNECT_OK;

	else if ((strstr((char *)SIM900_State_Ptr->buffer_command, "TCP CLOSING") != NULL) || (strstr((char *)SIM900_State_Ptr->buffer_command, "UDP CLOSING") != NULL))
		SIM900_State_Ptr-> IP_state_actual = TCP_UDP_CLOSING;

	else if ((strstr((char *)SIM900_State_Ptr->buffer_command, "TCP CLOSED") != NULL) || (strstr((char *)SIM900_State_Ptr->buffer_command, "UDP CLOSED") != NULL))
		SIM900_State_Ptr-> IP_state_actual = TCP_UDP_CLOSED;

	else if (strstr((char *)SIM900_State_Ptr->buffer_command, "PDP DEACT") != NULL)
		SIM900_State_Ptr-> IP_state_actual = PDP_DEACT;

	else SIM900_State_Ptr-> IP_state_actual =  ERRORED;

	return SIM900_State_Ptr-> IP_state_actual;
}

/* configureGPRS_TCP_UDP(uint8_t, uint8_t) - configures GPRS connection with login, password and some other parameters to use TCP or UDP connections
 *
 * This function creates a GPRS connection with the carrier server to get access to the Internet
 *
 * Returns '1' on success, '0' if error, '-2' if error detaching the GPRS connection,
 * '-3' if error attaching the GPRS connection, '-4' if error setting the application mode,
 * '-5' if error setting the connection mode, '-6' if error establishing the connection with the GPRS provider,
 * '-15' if error detaching the GPRS connection with CME error code available,
 * '-16' if error attaching the GPRS connection with CME error code available
*/
int8_t SIM900ConfigureGPRS_TCP_UDP(SIM900StateHandlePtr SIM900_State_Ptr, uint8_t connection_mode, uint8_t app_mode, SIM900_IP_state desired_state){

	uint8_t answer = 0;
	tick_timer tick_timer;

	SIM900CheckIPstatus(SIM900_State_Ptr);

#if GPRS_debug_mode>0
		xSerialxPrintf_P(&xSerialPort, PSTR("IP_state_desired: %u\r\n"), desired_state);
#endif

    timer_set( &tick_timer, 5 * SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS ); // leave for 10x the maximum time out, because we have a few states to traverse.
    while (SIM900_State_Ptr->IP_state_actual != desired_state)
    {

#if GPRS_debug_mode>0
		xSerialxPrintf_P(&xSerialPort, PSTR("IP_state_actual: %u\r\n"), SIM900_State_Ptr->IP_state_actual);
#endif
		switch( SIM900_State_Ptr->IP_state_actual ) // Checks the GPRS state to make sure we know where we are.
		{
		case ERRORED:
			switch( desired_state ) // Checks the GPRS state to make sure we know what we have to do.
			{
			case PDP_DEACT:
				do answer = sendCommand2(SIM900_State_Ptr, AT_GPRS_ATT_OFF, OK_RESPONSE, ERROR_CME);
				while( (answer != 1) && (timer_expired( &tick_timer ) == 0) );
				break;

			default:
				do answer = sendCommand2(SIM900_State_Ptr, AT_IP_SHUT, AT_IP_SHUT_R, ERROR);
				while( (answer != 1) && (timer_expired( &tick_timer ) == 0) );
				break;
			}
			break;

		case IP_INITIAL:
			SIM900_State_Ptr->mux_mode_IP = connection_mode;
			do{
				xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// always flush Rx first, otherwise Tx interrupt misbehaves
				if(connection_mode == SIM900_SINGLE_CONNECTION) // Set the connection mode: single or multi-connection
					xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR("AT%s0\r\n"), AT_IP_CONN_MODE);
				else
					xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR("AT%s1\r\n"), AT_IP_CONN_MODE);
#if GPRS_debug_mode>0
					xSerialxPrintf_P(&xSerialPort, PSTR("Send command with 1 answer: AT%sx\r\n"), AT_IP_CONN_MODE);
#endif
				answer = waitForAnswer1( SIM900_State_Ptr, OK_RESPONSE, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS);
			}while( (answer != 1) && (timer_expired( &tick_timer ) == 0) );
			if(answer != 1)
			{
				SIM900_State_Ptr->IP_state_actual = ERRORED;
				break;
			}

			do{
				xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// always flush Rx first, otherwise Tx interrupt misbehaves
				xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR("AT%s\r\n"), AT_IP_SET_START);
#if GPRS_debug_mode>0
				xSerialxPrintf_P(&xSerialPort, PSTR("Send command with 1 answer: AT%s\r\n"), AT_IP_SET_START);
#endif
				answer = waitForAnswer2( SIM900_State_Ptr, OK_RESPONSE, ERROR, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS);
				if (answer !=1) vTaskDelay( ( SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS  / 2 ) );
			}while( (answer != 1) && (timer_expired( &tick_timer ) == 0) );
			if(answer != 1)
				SIM900_State_Ptr->IP_state_actual = ERRORED;
			break;

		case IP_START:
			switch( desired_state ) // Checks the GPRS state to make sure we know what we have to do.
			{
			case IP_INITIAL:
				do answer = sendCommand2(SIM900_State_Ptr, AT_IP_SHUT, AT_IP_SHUT_R, ERROR);
				while( (answer != 1) && (timer_expired( &tick_timer ) == 0) );
				break;

			default:
				do answer = sendCommand2(SIM900_State_Ptr, AT_IP_BRING, OK_RESPONSE, ERROR); // Brings up GPRS connection
				while( (answer != 1) && (timer_expired( &tick_timer ) == 0) );
				if(answer != 1)
				{
					SIM900_State_Ptr->IP_state_actual = ERRORED;
					break;
				}
			}// float through this, as IP_CONFIG is a transient state and otherwise we may miss entering it.

		case IP_CONFIG:
			switch( desired_state ) // Checks the GPRS state to make sure we know what we have to do.
			{
			case IP_INITIAL:
				do answer = sendCommand2(SIM900_State_Ptr, AT_IP_SHUT, AT_IP_SHUT_R, ERROR);
				while( (answer != 1) && (timer_expired( &tick_timer ) == 0) );
				break;

			default:
				SIM900_State_Ptr->app_mode_IP = app_mode;
				do{
					xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// always flush Rx first, otherwise Tx interrupt misbehaves
					if(app_mode == SIM900_NON_TRANSPARENT) // Set the application mode: transparent or not transparent
						xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR("AT%s0\r\n"), AT_IP_APP_MODE);
					else
						xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR("AT%s1\r\n"), AT_IP_APP_MODE);
#if GPRS_debug_mode>0
					xSerialxPrintf_P(&xSerialPort, PSTR("Send command with 1 answer: AT%sX\r\n"), AT_IP_APP_MODE);
#endif
					answer = waitForAnswer1( SIM900_State_Ptr, OK_RESPONSE, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS);
					if (answer !=1) vTaskDelay( ( SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS  / 4 ) );

				}while( (answer != 1) && (timer_expired( &tick_timer ) == 0) );

				if(answer != 1)
					SIM900_State_Ptr->IP_state_actual = ERRORED;
				break;
			}
			break;

		case IP_GPRSACT:
			switch( desired_state ) // Checks the GPRS state to make sure we know what we have to do.
			{
			case IP_INITIAL:
				do answer = sendCommand2(SIM900_State_Ptr, AT_IP_SHUT, AT_IP_SHUT_R, ERROR);
				while( (answer != 1) && (timer_expired( &tick_timer ) == 0) );
				break;

			case PDP_DEACT:
				do answer = sendCommand2(SIM900_State_Ptr, AT_GPRS_ATT_OFF, OK_RESPONSE, ERROR_CME);
				while( (answer != 1) && (timer_expired( &tick_timer ) == 0) );
				break;

			default:
				do{
#if GPRS_debug_mode>0
					xSerialxPrintf_P(&xSerialPort, PSTR("Send command with 1 answer: AT%s\r\n"), AT_IP_GET_IP);
#endif
					answer = SIM900GetIP( SIM900_State_Ptr);

				}while( (answer != 1) && (timer_expired( &tick_timer ) == 0) );

				if(answer == 1)
				{
					SIM900_State_Ptr->my_IP_addr = htonl( inet_addr( SIM900_State_Ptr->buffer_command )); /**< dotted notation address string.  */
				}
#if GPRS_debug_mode>0
				xSerialxPrintf_P(&xSerialPort, PSTR("IP Address: %s  0x%X\r\n"), SIM900_State_Ptr->buffer_command, SIM900_State_Ptr->my_IP_addr);
#endif
				break;
			}
			break;

		case IP_PROCESSING:
			switch( desired_state ) // Checks the GPRS state to make sure we know what we have to do.
			{
			case IP_INITIAL:
				do answer = sendCommand2(SIM900_State_Ptr, AT_IP_SHUT, AT_IP_SHUT_R, ERROR);
				while( (answer != 1) && (timer_expired( &tick_timer ) == 0) );
				break;

			case PDP_DEACT:
				do answer = sendCommand2(SIM900_State_Ptr, AT_GPRS_ATT_OFF, OK_RESPONSE, ERROR_CME);
				while( (answer != 1) && (timer_expired( &tick_timer ) == 0) );
				break;

			default:
				break;
			}
			break;

		case CONNECTING_LISTENING:
			switch( desired_state ) // Checks the GPRS state to make sure we know what we have to do.
			{
			case IP_INITIAL:
				do answer = sendCommand2(SIM900_State_Ptr, AT_IP_SHUT, AT_IP_SHUT_R, ERROR);
				while( (answer != 1) && (timer_expired( &tick_timer ) == 0) );
				break;

			default:
				break;
			}
			break;

		case CONNECT_OK:
			switch( desired_state ) // Checks the GPRS state to make sure we know what we have to do.
			{
			case IP_INITIAL:
				do answer = sendCommand2(SIM900_State_Ptr, AT_IP_SHUT, AT_IP_SHUT_R, ERROR);
				while( (answer != 1) && (timer_expired( &tick_timer ) == 0) );
				break;

			case PDP_DEACT:
				do answer = sendCommand2(SIM900_State_Ptr, AT_GPRS_ATT_OFF, OK_RESPONSE, ERROR_CME);
				while( (answer != 1) && (timer_expired( &tick_timer ) == 0) );
				break;

			default:
				break;
			}
			break;

		case TCP_UDP_CLOSING:

			break;

		case TCP_UDP_CLOSED:
			switch( desired_state ) // Checks the GPRS state to make sure we know what we have to do.
			{
			case PDP_DEACT:
				do answer = sendCommand2(SIM900_State_Ptr, AT_GPRS_ATT_OFF, OK_RESPONSE, ERROR_CME);
				while( (answer != 1) && (timer_expired( &tick_timer ) == 0) );
				break;

			case IP_INITIAL:
			default:
				do answer = sendCommand2(SIM900_State_Ptr, AT_IP_SHUT, AT_IP_SHUT_R, ERROR);
				while( (answer != 1) && (timer_expired( &tick_timer ) == 0) );
				break;
			}
			break;

		case PDP_DEACT:
			do answer = sendCommand2(SIM900_State_Ptr, AT_GPRS_ATT_ON, OK_RESPONSE, ERROR_CME);
			while( (answer != 1) && (timer_expired( &tick_timer ) == 0) );
			break;
		}

		SIM900CheckIPstatus(SIM900_State_Ptr);
	}
	return answer;
}

/* setLocalPort(const char*, uint16_t) - Sets the number of the internal port for TCP or UDP connections
 *
 * This function sets the number of the internal port for TCP or UDP connections
 *
 * Returns '1' on success, '0' if error
*/
uint8_t SIM900SetLocalPort(SIM900StateHandlePtr SIM900_State_Ptr, const uint8_t* mode, const uint16_t port){

	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// always flush Rx first, otherwise Tx interrupt misbehaves
	xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s\"%s\",%u\r\n" ), AT_IP_LOCAL_PORT, mode, port);

	return (waitForAnswer2( SIM900_State_Ptr, OK_RESPONSE, ERROR, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS) == 1) ? 1 : 0;
}

/* saveGPRS_TCP_UDPconfiguration() - Saves the configuration into the internal NVRAM of the GPRS module
 *
 * This function saves the configuration into the internal NVRAM of the GPRS module
 *
 * Returns '1' on success, '0' if error
*/
uint8_t SIM900SaveGPRS_TCP_UDPconfiguration(SIM900StateHandlePtr SIM900_State_Ptr){
	return( sendCommand1(SIM900_State_Ptr, AT_IP_SAVE_CONF, OK_RESPONSE));
}

/* createSocket(uint8_t, uint8_t, const char*, const char*) - creates a TCP/IP connection to the specified IP and PORT
 *
 * This function creates a TCP/IP connection to the specified IP and PORT
 *
 * In multi connection mode you must specify a number of connection (0-7).
 *
 * Returns '1' on success, '0' if error setting the connection, '-2' if error setting the connection wit CME error code available
 * and '-3' if time out waiting the connection
*/
int8_t SIM900CreateSocket(SIM900StateHandlePtr SIM900_State_Ptr, const SIM900_working_mode working_mode, const uint8_t n_connection, const uint8_t* ip, const uint8_t* port){

	uint8_t character;
    seconds_timer seconds_timer;

	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// always flush Rx first, otherwise Tx interrupt misbehaves

	switch (SIM900_State_Ptr->mux_mode_IP)
	{
		case SIM900_SINGLE_CONNECTION: // Single mode
			switch (working_mode)
			{
				case UDP_CLIENT:
					xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s0\r\n" ), AT_IP_UDP_EXTENDED);
					if( waitForAnswer1( SIM900_State_Ptr, OK_RESPONSE, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS) != 1)
						return 0;

					xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s\"%s\",\"%s\",\"%s\"\r\n" ), AT_IP_CLIENT, AT_UDP, ip, port);
					if( waitForAnswer2( SIM900_State_Ptr, OK_RESPONSE, ERROR_CME, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS) != 1)
						return 0;
					break;

				case TCP_CLIENT:
					xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s\"%s\",\"%s\",\"%s\"\r\n" ), AT_IP_CLIENT, AT_TCP, ip, port);
					if( waitForAnswer2( SIM900_State_Ptr, OK_RESPONSE, ERROR_CME, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS) != 1)
						return 0;
					break;

				case TCP_SERVER:
					xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s1,\"%s\"\r\n" ), AT_IP_SERVER, port);
					if( waitForAnswer1( SIM900_State_Ptr, OK_RESPONSE, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS) != 1)
						return 0;
					break;

				case UDP_EXTENDED:
					xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s1\r\n" ), AT_IP_UDP_EXTENDED);
					if( waitForAnswer1( SIM900_State_Ptr, OK_RESPONSE, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS) != 1)
						return 0;

					xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s\"%s\",\"%s\",\"%s\"\r\n" ), AT_IP_CLIENT, AT_UDP, ip, port);
					if( waitForAnswer2( SIM900_State_Ptr, OK_RESPONSE, ERROR_CME, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS) != 1)
						return 0;

					xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s2,\"%s\",\"%s\"\r\n" ), AT_IP_UDP_EXTENDED, ip, port);
					if( waitForAnswer1( SIM900_State_Ptr, OK_RESPONSE, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS) != 1)
						return 0;
					break;
			}
			break;

		case SIM900_MULTI_CONNECTION: // Multi mode
			switch (working_mode)
			{
				case UDP_CLIENT:
					xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s%u,\"%s\",\"%s\",\"%s\"\r\n" ), AT_IP_CLIENT, n_connection, AT_UDP, ip, port);
					if( waitForAnswer2( SIM900_State_Ptr, OK_RESPONSE, ERROR_CME, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS) != 1)
						return 0;
					break;

				case TCP_CLIENT:
					xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s%u,\"%s\",\"%s\",\"%s\"\r\n" ), AT_IP_CLIENT, n_connection, AT_TCP, ip, port);
					if( waitForAnswer2( SIM900_State_Ptr, OK_RESPONSE, ERROR_CME, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS) != 1)
						return 0;
					break;

				case TCP_SERVER:
					xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s1,\"%s\"\r\n" ), AT_IP_SERVER, port);
					if( waitForAnswer1( SIM900_State_Ptr, OK_RESPONSE, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS) != 1)
						return 0;
					break;

				case UDP_EXTENDED: // not appropriate for this option
					return 0;
					break;
			}
			break;
	}

	stimer_set( &seconds_timer, 20 ); // twenty seconds to connect

	if (working_mode == UDP_CLIENT || working_mode == TCP_CLIENT)
	{
		// Waits 20 seconds to connect
		if (SIM900_State_Ptr->mux_mode_IP == SIM900_SINGLE_CONNECTION)
		{
			while ((xSerialAvailableChar( SIM900_State_Ptr->SIM900SerialPortPtr ) == 0) && (stimer_expired(&seconds_timer) == 0))
				; // wait for the connection to be established

			if( waitForAnswer2( SIM900_State_Ptr, AT_CONNECTED_OK, AT_CONNECTED_FAIL, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS) != 1)
				return 0;
		}
		else
		{
			do xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &character );
			while ((character != ',') && (stimer_expired(&seconds_timer) == 0)); // wait for a ',' which starts the size, unless app_mode_IP == 1

			if( waitForAnswer2( SIM900_State_Ptr, AT_CONNECTED_OK, AT_CONNECTED_FAIL, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS) != 1)
				return 0;
		}

	}
	return 1;
}

/* sendData(SIM900StateHandlePtr, uint8_t*, uint16_t, uint8_t) - sends 'data' to the specified to 'n_connection'
 *
 * This function sends 'data' to the specified to 'n_connection'. In single connection not specifies 'n_connection'.
 *
 * Returns '1' on success, '0' if error waiting the response of the module, '-2' if error with CME error code available
 * '-3' if no feedback detected and '-4' if the send fails
*/
int8_t SIM900SendData(SIM900StateHandlePtr SIM900_State_Ptr, uint8_t* data, uint16_t length, uint8_t n_connection){

	uint8_t answer = 0;
	uint8_t count;

	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// always flush Rx first, otherwise Tx interrupt misbehaves

	switch (SIM900_State_Ptr->app_mode_IP)
	{
	case SIM900_NON_TRANSPARENT: // Non transparent mode

		count=5;
		// prepares the connection to send data
		do{
			switch (SIM900_State_Ptr->mux_mode_IP)
			{
				case SIM900_SINGLE_CONNECTION: // Single mode
					xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s=%d\r\n" ), AT_IP_SEND, length);
					break;

				case SIM900_MULTI_CONNECTION: // Multi mode
					xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s=%c,%d\r\n" ), AT_IP_SEND, n_connection+0x30, length);
					break;
			}

			answer = waitForAnswer2( SIM900_State_Ptr, ">", ERROR_CME, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS);

		}while ((answer != 1) && (--count != 0));

		if (answer == 0)
		{
			return 0;
		}
		else if (answer == 2)
		{
			return -2;
		}

		// Sends data
		count = 5;
		do{
			// Sends data and waits 30 seconds for the feedback
			for( uint16_t x = 0; x<length; ++x)
			{
				while ( xSerialPutChar( SIM900_State_Ptr->SIM900SerialPortPtr, data[x] ) == pdFAIL) // will only fail if buffer is full
					taskYIELD(); // yield to delay a little, only if sending failed, and let something else run.

			}
			answer = waitForAnswer2( SIM900_State_Ptr, AT_IP_SEND_R, AT_IP_SEND_FAIL, 10 * SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS);

		}while ((answer != 1) && (--count != 0));

		if (answer == 0)
		{
			return -3;
		}
		else if (answer == 2)
		{
			return -4;
		}
		break;

	case SIM900_TRANSPARENT: // Transparent mode

		// Transparent mode
		for( uint16_t x = 0; x < length; ++x )
		{
			while ( xSerialPutChar( SIM900_State_Ptr->SIM900SerialPortPtr, data[x] ) == pdFAIL) // will only fail if buffer is full
				taskYIELD(); // yield to delay a little, only if sending failed, and let something else run.

		}
		break;
	}

	return 1;
}

/* readIPData(char*) - Gets data receive from a TCP or UDP connection and stores it in 'buffer_GPRS'
 *
 * This function gets data receive from a TCP or UDP connection and stores it in 'buffer_GPRS'.
 *
 * In multi connection mode also stores the connection number in 'IP_data_from.
 *
 * This function should be executed only inside 'SIM900ManageIncomingData' function.
 *
 * Returns '1' on success and '0' if error
*/
int8_t SIM900ReadIPData(SIM900StateHandlePtr SIM900_State_Ptr, uint8_t* dataIN, uint16_t length){

	uint8_t character;
	uint16_t i = 0;
	uint8_t IP_data_from;
	uint16_t IP_data_length;
	seconds_timer secs_timer;

	memset(SIM900_State_Ptr->buffer_packet, '\0', SIM900_State_Ptr->buffer_packet_size );

	switch (SIM900_State_Ptr->mux_mode_IP)
	{
	case SIM900_SINGLE_CONNECTION: // Single mode

		switch (SIM900_State_Ptr->app_mode_IP)
		{
		case SIM900_NON_TRANSPARENT: // Non transparent mode
			// Non transparent mode
			switch (SIM900_State_Ptr->data_mode_IP)
			{
			case SIM900_DATA_MANUAL: // manual data retrieval

				i = SIM900GetDataManually( SIM900_State_Ptr, SIM900_State_Ptr->buffer_packet_size, 0);
				break;

			case SIM900_DATA_AUTOMATIC: // automatic data retrieval

				do xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &character );
				while ((character != ':') && (stimer_expired(&secs_timer) == 0)); // wait for a ':'

				// Gets the length of the data string that is waiting to be transferred.
				IP_data_length = 0;
				xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &character );
				do{
					IP_data_length *= 10;
					IP_data_length += (character - 0x30);
					xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &character );
				}while ((character != ',') && (stimer_expired(&secs_timer) == 0)); // a routine to get numbers from strings

				do xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &character );
				while ((character != ':') && (stimer_expired(&secs_timer) == 0)); // wait for a ':'

				do{
					while ((xSerialAvailableChar( SIM900_State_Ptr->SIM900SerialPortPtr ) == 0) && (stimer_expired(&secs_timer) == 0))
						;
					xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &(SIM900_State_Ptr->buffer_packet[i++]) );

				}while ((i < (SIM900_State_Ptr->buffer_packet_size - 1)) && (stimer_expired(&secs_timer) == 0));

				SIM900_State_Ptr->buffer_packet[i] = '\0';

				xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// flush out any remaining characters in the ring buffer

				if(i > 0) i = 1; else i = 0;
				break;

			default:
				i = 0;
				break;
			}

		case SIM900_TRANSPARENT: // Transparent mode
				// Transparent mode
			do{
				while ((xSerialAvailableChar( SIM900_State_Ptr->SIM900SerialPortPtr ) == 0) && (stimer_expired(&secs_timer) == 0))
					;
				xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &(SIM900_State_Ptr->buffer_packet[i++]) );

			}while ((i < (SIM900_State_Ptr->buffer_packet_size - 1)) && (stimer_expired(&secs_timer) == 0));

			SIM900_State_Ptr->buffer_packet[i] = '\0';

			xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// flush out any remaining characters in the ring buffer

			if(i > 0) i = 1; else i = 0;
			break;

		default:
			i = 0;
			break;
		}


	case SIM900_MULTI_CONNECTION: // Multi-connection mode

		do xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &character );
		while ((character != ',') && (stimer_expired(&secs_timer) == 0)); // wait for a ','


		xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &character );
		IP_data_from = character - 0x30;	// Gets the connection number

		switch (SIM900_State_Ptr->data_mode_IP)
		{
		case SIM900_DATA_MANUAL: // manual data retrieval

			i = SIM900GetDataManually( SIM900_State_Ptr, SIM900_State_Ptr->buffer_packet_size, IP_data_from);
			break;

		case SIM900_DATA_AUTOMATIC: // automatic data retrieval

			// Gets the length of the data string that is waiting to be transferred.
			IP_data_length = 0;
			xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &character );
			do{
				IP_data_length *= 10;
				IP_data_length += (character - 0x30);
				xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &character );
			}while ((character != ',') && (stimer_expired(&secs_timer) == 0)); // a routine to get numbers from strings

			do xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &character );
			while ((character != '\n') && (stimer_expired(&secs_timer) == 0)); // wait for a '\n'

			do{
				while ((xSerialAvailableChar( SIM900_State_Ptr->SIM900SerialPortPtr ) == 0) && (stimer_expired(&secs_timer) == 0))
					;
				xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &(SIM900_State_Ptr->buffer_packet[i++]) );

			}while ((i < (SIM900_State_Ptr->buffer_packet_size - 1)) && (stimer_expired(&secs_timer) == 0));

			SIM900_State_Ptr->buffer_packet[i] = '\0';

			xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// flush out any remaining characters in the ring buffer

			if(i > 0) i = 1; else i = 0;
			break;

		default:
			i = 0;
			break;
		}

	default:
		break;
	}
	return i;
}

/* closeSocket(uint8_t) - closes the socket specified by 'socket'
 *
 * This function closes the connection specified by 'n_connection'.In single not specifies number of connection. For server use 8
 *
 * Returns '1' on success, '0' if error
*/
uint8_t SIM900CloseSocket(SIM900StateHandlePtr SIM900_State_Ptr, uint8_t n_connection){

	uint8_t character;
    tick_timer tick_timer;

	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// always flush Rx first, otherwise Tx interrupt misbehaves

	timer_set( &tick_timer, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS );

	if (n_connection == 8)
	{
		// Closes TCP server
		xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s0\r\n" ), AT_IP_SERVER);
		if( waitForAnswer2( SIM900_State_Ptr, OK_RESPONSE, ERROR, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS) != 1)
			return 0;
	}
	else
	{
		// Closes TCP or UDP client
		switch (SIM900_State_Ptr->mux_mode_IP)
		{
			case 0: // Single connection mode
				xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s0\r\n" ), AT_IP_CLOSE);
				break;

			case 1: // Multi-connection mode
				xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s%c,0\r\n" ), AT_IP_CLOSE, n_connection+0x30);
				do{
					 xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &character);
				} while ( (character != ',') && (timer_expired( &tick_timer ) == 0) ); // throw away the n_connection number returned
				break;
		}

		if( waitForAnswer2( SIM900_State_Ptr, AT_IP_CLOSE_R, ERROR, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS) != 1)
			return 0;
	}
	return 1;
}

/* QuickcloseSocket(socket) - Enables/disables to close the connection quickly
 *
 * This function enables/disables to close the connection quickly.
 *
 * Returns '1' on success, '0' if error
*/
uint8_t SIM900QuickCloseSocket(SIM900StateHandlePtr SIM900_State_Ptr, uint8_t mode){

	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// always flush Rx first, otherwise Tx interrupt misbehaves

	switch (mode)
	{
		case 0:
			xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s0\r\n" ), AT_IP_QCLOSE);
			break;
		case 1:
			xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s1\r\n" ), AT_IP_QCLOSE);
			break;
	}
	return( waitForAnswer2( SIM900_State_Ptr, OK_RESPONSE, ERROR, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS) != 1 ? 0 : 1);
}

/* getIPfromDNS(const char*) - Gets the IP address from a URL using DNS servers
 *
 * This function gets the IP direction from a URL using DNS servers
 *
 * Returns '1' on success, '0' if error
*/
uint8_t SIM900GetIPfromDNS(SIM900StateHandlePtr SIM900_State_Ptr, const char* IP_query){

	uint8_t character;
	uint8_t i = 0;
	uint8_t escape_chr = 0;
    tick_timer tick_timer;

	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// always flush Rx first, otherwise Tx interrupt misbehaves
	xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s\"%s\"\r\n" ), AT_IP_QUERY_DNS, IP_query);

	if( waitForAnswer2( SIM900_State_Ptr, OK_RESPONSE, ERROR, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS) != 1)
		return 0;

	if( waitForAnswer2( SIM900_State_Ptr, AT_IP_QUERY_DNS_R_1, AT_IP_QUERY_DNS_R_0, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS) != 1)
		return 0;

	timer_set( &tick_timer, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS );

	do{
		 if( xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &character) )
		 {
			 if(character == '"')	// capture 3 '"' to get to the start of the IP address in quotes.
				 ++escape_chr;
		 }
	} while ( (escape_chr < 3) && (timer_expired( &tick_timer ) == 0) );

	escape_chr = 0; // reinitialise our escape character counter

	do{
		 if( xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &character) )
		 {
			 if(character == '\r' || character == '\n')
				 ++escape_chr;
			 else
				 if( ((0x2F < character) && (character < 0x3A)) || (character == '.') )	// Only captures character when it's a ascii number or a dot
					 SIM900_State_Ptr->buffer_command[i++] = character;
		 }
	} while ( (escape_chr < 2) && (i < SIM900_State_Ptr->buffer_command_size - 1) && (timer_expired( &tick_timer ) == 0) );

	SIM900_State_Ptr->buffer_command[i] = '\0';

	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// flush any remaining characters.

	return i > 0 ? 1 : 0;
}


/* IPHeader(uint8_t) - Adds an IP head at the beginning of a package received
 *
 * This function adds an IP head at the beginning of a package received
 *
 * Returns '1' on success, '0' if error
*/
int8_t SIM900AddIPHeader(SIM900StateHandlePtr SIM900_State_Ptr, uint8_t on_off){

	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// always flush Rx first, otherwise Tx interrupt misbehaves
	xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s\"%s\",%u\r\n" ), AT_IP_HEADER, on_off);

	return (waitForAnswer2( SIM900_State_Ptr, OK_RESPONSE, ERROR, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS) == 1) ? 1 : 0;
}

/* SetAutoSendingTimer(uint8_t, uint8_t) - Sets a timer when module is sending data
 *
 * This function sets a timer when module is sending data
 *
 * Returns '1' on success, '0' if error
*/
int8_t SIM900SetAutoSendingTimer(SIM900StateHandlePtr SIM900_State_Ptr, uint8_t mode, uint8_t time){

	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// always flush Rx first, otherwise Tx interrupt misbehaves
	switch (mode)
	{
		case 0:
			xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s0\r\n" ), AT_IP_AUTOSENDING);
			break;
		case 1:
			xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s1,%u\r\n" ), AT_IP_AUTOSENDING, time);
			break;
	}
	return (waitForAnswer2( SIM900_State_Ptr, OK_RESPONSE, ERROR, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS) == 1) ? 1 : 0;
}

/* ShowRemoteIP(uint8_t) - Enables or disables to show remote IP address and port when received data
 *
 * This function enables or disables to show remote IP address and port when received data
 *
 * Returns '1' on success, '0' if error
*/
int8_t SIM900ShowRemoteIP(SIM900StateHandlePtr SIM900_State_Ptr, uint8_t on_off){

	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// always flush Rx first, otherwise Tx interrupt misbehaves
	xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s%u\r\n" ), AT_IP_SHOW_REMOTE_IP, on_off);

	return (waitForAnswer2( SIM900_State_Ptr, OK_RESPONSE, ERROR, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS) == 1) ? 1 : 0;
}

/* ShowProtocolHeader(uint8_t) - Enables or disables to show transfer protocol in IP head when received data
 *
 * This function enables or disables to show transfer protocol in IP head when received data
 *
 * Returns '1' on success, '0' if error
*/
int8_t SIM900ShowProtocolHeader(SIM900StateHandlePtr SIM900_State_Ptr, uint8_t on_off){

	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// always flush Rx first, otherwise Tx interrupt misbehaves
	xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s%u\r\n" ), AT_IP_PROTOCOL_HEADER, on_off);

	return (waitForAnswer2( SIM900_State_Ptr, OK_RESPONSE, ERROR, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS) == 1) ? 1 : 0;
}

/* DiscardInputATData(uint8_t) - Enables or disables to discard input AT data in TCP data send
 *
 * This function enables or disables show to discard input AT data in TCP data send
 *
 * Returns '1' on success, '0' if error
*/
int8_t SIM900DiscardInputATData(SIM900StateHandlePtr SIM900_State_Ptr, uint8_t on_off){

	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// always flush Rx first, otherwise Tx interrupt misbehaves
	xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s%u\r\n" ), AT_IP_DISCARD_AT_DATA, on_off);

	return (waitForAnswer2( SIM900_State_Ptr, OK_RESPONSE, ERROR, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS) == 1) ? 1 : 0;
}

/* SetDataManually(uint8_t, uint8_t) - Enables or disables to get data manually from a TCP or UDP connection
 *
 * This function enables or disables show to get data manually from a TCP or UDP connection
 *
 * Returns '1' on success, '0' if error
*/
int8_t SIM900SetDataManually(SIM900StateHandlePtr SIM900_State_Ptr, uint8_t on_off, uint8_t id){

	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// always flush Rx first, otherwise Tx interrupt misbehaves

	if (SIM900_State_Ptr->app_mode_IP == 0)
		xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s=%u\r\n" ), AT_IP_GET_MANUALLY, on_off);
	else
		xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s=%u,%u\r\n" ), AT_IP_GET_MANUALLY, on_off, id);

	uint8_t answer = waitForAnswer2( SIM900_State_Ptr, OK_RESPONSE, ERROR, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS);

	if( answer == 1 )
		SIM900_State_Ptr->data_mode_IP = on_off;

	return (answer == 1) ? 1 : 0;
}


/* GetDataManually(uint16_t, uint8_t) - Gets data manually from a TCP or UDP connection
 *
 * This function gets data manually from a TCP or UDP connection
 *
 * Returns '1' on success, '0' if error and '2' if buffer_GPRS is full. The answer from the server is
 * limited by the length of buffer_packet_size. To increase the maximum length of the answer, to a maximum of 1460 bytes.
*/
int8_t SIM900GetDataManually(SIM900StateHandlePtr SIM900_State_Ptr, uint16_t data_length, uint8_t id){

	uint8_t character;
	uint16_t i = 0;
	uint16_t IP_data_length;
	seconds_timer secs_timer;

	if (SIM900_State_Ptr->app_mode_IP == SIM900_NON_TRANSPARENT)
		xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s=2,%u\r\n" ), AT_IP_GET_MANUALLY, data_length);
	else
		xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s=2,%u,%u\r\n" ), AT_IP_GET_MANUALLY, id, data_length);

	character = waitForAnswer2( SIM900_State_Ptr, "+CIPRXGET=2,", ERROR, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS);

	switch (character)
	{
	case 0:
		return 0;
		break;

	case 1:
		stimer_set( &secs_timer, 10 ); // try get data stored on the SIM900, we're waiting no more than 10 seconds

		do xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &character );
		while ((character != ':') && (stimer_expired(&secs_timer) == 0)); // wait for a ':' which starts the mode (which we know is 2, to get data)

		do xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &character );
		while ((character != ',') && (stimer_expired(&secs_timer) == 0)); // wait for a ',' which starts the size, unless app_mode_IP == 1


		if (SIM900_State_Ptr->app_mode_IP == SIM900_TRANSPARENT)
		{
			do xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &character );
			while ((character != ',') && (stimer_expired(&secs_timer) == 0)); // wait for a ',' which starts the size, when app_mode_IP == 1
		}

		// Gets the length of the data string that is waiting to be transferred.
		IP_data_length = 0;
		xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &character );
		do{
			IP_data_length *= 10;
			IP_data_length += (character - 0x30);
			xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &character );
		}while ((character != ',') && (stimer_expired(&secs_timer) == 0)); // a routine to get numbers from strings

		do xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &character );
		while ((character != '\n') && (stimer_expired(&secs_timer) == 0));

		do{
			while ((xSerialAvailableChar( SIM900_State_Ptr->SIM900SerialPortPtr ) == 0) && (stimer_expired(&secs_timer) == 0))
				;
			xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &(SIM900_State_Ptr->buffer_packet[i++]) );

		}while ((i < (SIM900_State_Ptr->buffer_packet_size - 1)) && (stimer_expired(&secs_timer) == 0));

		SIM900_State_Ptr->buffer_packet[i] = '\0';

		xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// flush out any remaining characters in the ring buffer

		break;

	case 2:
		return 0;
		break;
	}

	return 1;
}


/* setDNS() - Sets the directions of DNS servers from SIM900constants.h
 *
 * This function sets the directions of DNS servers from SIM900constants.h
 *
 * Returns '1' on success, '0' if error and '-2' if CME error code available
*/
int8_t SIM900SetDNS(SIM900StateHandlePtr SIM900_State_Ptr){

	int8_t answer;

	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// always flush Rx first, otherwise Tx interrupt misbehaves
	xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s\"%s\",\"%s\"\r\n" ), AT_IP_SET_DNS, AT_GPRS_DNS1, AT_GPRS_DNS2);

	answer = waitForAnswer2( SIM900_State_Ptr, OK_RESPONSE, ERROR_CME, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS);

	return answer == 2 ? -answer : answer;
}


/* setDNS(const char*, const char*) - Sets the directions of DNS servers
 *
 * This function sets the directions of DNS servers
 *
 * Returns '1' on success, '0' if error and '-2' if CME error code available
*/
int8_t SIM900ChangeDNS(SIM900StateHandlePtr SIM900_State_Ptr, const char* DNS_dir1, const char* DNS_dir2){

	int8_t answer;

	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// always flush Rx first, otherwise Tx interrupt misbehaves
	xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR( "AT%s\"%s\",\"%s\"\r\n" ), AT_IP_SET_DNS, DNS_dir1, DNS_dir2);

	answer = waitForAnswer2( SIM900_State_Ptr, OK_RESPONSE, ERROR_CME, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS);

	return answer == 2 ? -answer : answer;
}


/* whoamI() - Gets the model of the module
 *
 * This function gets the model of the module and saves it in 'buffer_GPRS'
 *
 * Returns '1' on success, '0' if error
*/
uint8_t SIM900WhoAmI(SIM900StateHandlePtr SIM900_State_Ptr){

	uint8_t character;
	uint8_t i = 0;
	uint8_t escape_chr = 0;
    tick_timer tick_timer;

	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// always flush Rx first, otherwise Tx interrupt misbehaves
	xSerialxPrintf_P(SIM900_State_Ptr->SIM900SerialPortPtr, PSTR("AT%s\r\n"), AT_WHO_AM_I);

	timer_set( &tick_timer, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS );
	do{
		if( xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &character) )
		{
			if(character == '\r' || character == '\n')
				++escape_chr;
			else
				SIM900_State_Ptr->buffer_command[i++] = character;
		}
	}while ((escape_chr < 4) && (i < (SIM900_State_Ptr->buffer_packet_size - 1)) && (timer_expired( &tick_timer ) == 0));

	SIM900_State_Ptr->buffer_command[i] = '\0';

	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// now flush unnecessary "OK"

	return i > 0 ? 1 : 0;
}

/* firmware_version() - Gets the firmware version of the module
 *
 * This function gets the firmware version of the module and saves it in 'buffer_GPRS'
 *
 * Returns '1' on success, '0' if error
*/
uint8_t SIM900FirmwareVersion(SIM900StateHandlePtr SIM900_State_Ptr){
	uint8_t character;
	uint8_t i = 0;
	uint8_t escape_chr = 0;
    tick_timer tick_timer;


	if (sendCommand1( SIM900_State_Ptr, AT_FIRMWARE, AT_FIRMWARE_R))	// Sends the command:
	{
		timer_set( &tick_timer, SIM900_MAX_TIME_OUT / portTICK_PERIOD_MS );
		do{
			if( xSerialGetChar( SIM900_State_Ptr->SIM900SerialPortPtr, &character) )
			{
				if(character == '\r' || character == '\n')
	    			++escape_chr;
				else
					SIM900_State_Ptr->buffer_command[i++] = character;
			}
		}while ((escape_chr < 2) && (i < (SIM900_State_Ptr->buffer_packet_size - 1)) && (timer_expired( &tick_timer ) == 0));

		SIM900_State_Ptr->buffer_command[i] = '\0';
	}
	xSerialFlush( SIM900_State_Ptr->SIM900SerialPortPtr );	// now flush unnecessary "OK"

	return i > 0 ? 1 : 0;
}

/* show_APN() - Shows the apn, login and password constants
 *
 * This function shows the apn, login and password constants
 *
 * Returns nothing
*/
void SIM900ShowAPN(SIM900StateHandlePtr SIM900_State_Ptr){
	// APN parameters depends on SIM
	xSerialxPrintf_P(&xSerialPort, PSTR("APN: %s\r\n"), AT_GPRS_APN);
	xSerialxPrintf_P(&xSerialPort, PSTR("LOGIN: %s\r\n"), AT_GPRS_LOGIN);
	xSerialxPrintf_P(&xSerialPort, PSTR("PASSWORD: %s\r\n"), AT_GPRS_PASSW);
}


