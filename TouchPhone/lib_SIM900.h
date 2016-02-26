/*
 * lib_SIM900.h
 *
 *  Created on: 04/10/2013
 *      Author: Phillip Stevens
 */

#ifndef LIB_SIM900_H_
#define LIB_SIM900_H_

/******************************************************************************
 * Definitions & Declarations
 ******************************************************************************/

#include "lib_SIM900Const.h"

/*! \def SIM900_POWER_ON
    \brief
 */
#define	ENABLE	1
#define	DISABLE	0

/*! \def Alert Sound Mode. Normal mode
 */
#define	SIM900_NORMAL_MODE	0
#define	SIM900_SILENT_MODE	1		// Silent mode (all sounds from SIM900 are prevented)

/*! \def Ringer mode.
 */
#define SIM900_LEVEL_OFF 0			// OFF in this case
#define SIM900_LEVEL_LOW 1
#define SIM900_LEVEL_MEDIUM 2
#define SIM900_LEVEL_HIGH 3
#define SIM900_LEVEL_CRESCENDO 4

/*! \def CLIR mode. Presentation indicator is used according to the subscription of the CLIR service
 */
#define	SIM900_DEFAULT_CLIR	0		// CLIR default
#define	SIM900_SUPPRESS_CLIR	1	// CLIR suppression
#define	SIM900_INVOKE_CLIR		2	// CLIR invocation

/*! \brief Configures the module to use single connection mode
 */
#define	SIM900_SINGLE_CONNECTION 0
#define	SIM900_MULTI_CONNECTION 1

/*! \brief When opening a socket, it can be opened in non transparent mode or transparent mode.
 */
#define	SIM900_NON_TRANSPARENT 0
#define	SIM900_TRANSPARENT 1

/*! \brief When reading data from a socket, it can be read in manual mode.
 */
#define	SIM900_DATA_AUTOMATIC 0
#define	SIM900_DATA_MANUAL 1

/*! \brief When opening a socket, it can be opened as client or server.
 */
#define	SIM900_UDP_CLIENT	0
#define	SIM900_TCP_CLIENT	1
#define	SIM900_TCP_SERVER	2
#define	SIM900_UDP_EXTENDED	3	//  Client for extended UDP

typedef enum
{
	POWER_ON = 0,	// ON in this case
	POWER_FULL,		// Full functionality
	POWER_RF_OFF,	// The RF part of the module will not work
	POWER_MIN,		// Minimum functionality
	POWER_SLEEP,	// Sleep mode
	POWER_OFF		// Powers off the module
} SIM900_POWER_state;

typedef enum
{
	UDP_CLIENT = 0,
	TCP_CLIENT,
	TCP_SERVER,
	UDP_EXTENDED
} SIM900_working_mode;

typedef enum
{
	ERRORED = 0,
	IP_INITIAL,
	IP_START,
	IP_CONFIG,
	IP_GPRSACT,
	IP_PROCESSING,
	CONNECTING_LISTENING,
	CONNECT_OK,
	TCP_UDP_CLOSING,
	TCP_UDP_CLOSED,
	PDP_DEACT
} SIM900_IP_state;

typedef struct
{
	xComPortHandlePtr SIM900SerialPortPtr;  // USART used by the GPRS module, GPRS module is connected to the UART1

	uint8_t ready; // Variable : stores if the module is ready or not (1:not ready, 0:ready)


	SIM900_POWER_state state_PWR;	// Power state.
	SIM900_IP_state IP_state_actual; // IP state currently.
	uint8_t mux_mode_IP;	// IP mux mode '0'= single connection, '1'= multi connection
	uint8_t app_mode_IP;	// IP application mode '0'= non transparent mode, '1'= transparent mode
	uint8_t data_mode_IP;	// IP data mode '0' = get data automatically, '1'= get data manually
	uint32_t my_IP_addr;	// the IP address in network 32bit format.

	uint8_t * buffer_command; // Buffer for command responses
	uint8_t buffer_command_size; // Size of buffer for command responses

	uint16_t socket_ID[4];	// Socket ID It stores the ID of the last socket opened
	uint16_t data_read;	// Length of data read from an URL or from a socket

	uint8_t * buffer_packet;  // Buffer for packets to send or receive
	uint8_t buffer_packet_size; // Size of buffer for packets to send or receive
} SIM900StateHandle, * SIM900StateHandlePtr;


//! Gets IP address when configuring TCP/UDP profiles
/*!
\param void
\return '1' on success, '0' if error. It stores IP direction in 'IP_direction'
*/
int8_t SIM900GetIP(SIM900StateHandlePtr SIM900_State_Ptr);

//! Checks if GPRS connection is OK
/*!
\param void
\return '1' IP INITIAL,'2' IP START,'3' IP CONFIG, '4' IP GPRSACT, '5' IP STATUS, '6' TCP CONNECTING/UDP CONNECTING/SERVER LISTENING,
'7' CONNECT OK, '8' TCP CLOSING/UDP CLOSING, '9' TCP CLOSED/UDP CLOSED, '10' PDP DEACT, '0' if ERRORED
 */
SIM900_IP_state SIM900CheckIPstatus(SIM900StateHandlePtr SIM900_State_Ptr);


//! Configure GPRS connection with login, password and some other parameters for use with TCP or UDP connections
/*!
The configuration parameters from 'lib_SIM900Constants.h' file
\param uint8_t connection_mode : MULTI_CONNECTION or SINGLE_CONNECTION
\param uint8_t app_mode : TRANSPARENT mode or NON_TRANSPARENT mode
\param SIM900_IP_state desired_state : desired connection state
\return '1' on success, '0' if error, '-2' if error detaching the GPRS connection,
	'-3' if error attaching the GPRS connection, '-4' if error setting the application mode,
	'-5' if error setting the connection mode, '-6' if error establishing the connection with the GPRS provider,
	'-15' if error detaching the GPRS connection with CME error code available,
	'-16' if error attaching the GPRS connection with CME error code available.
 */
int8_t SIM900ConfigureGPRS_TCP_UDP(SIM900StateHandlePtr SIM900_State_Ptr, uint8_t connection_mode, uint8_t app_mode, SIM900_IP_state desired_state);

//! Sets the number of the internal port for TCP or UDP connections
/*!
\param const char* mode : TCP or UDP
\param uint16_t port : PORT number
\return '1' on success, '0' if error
 */
uint8_t SIM900SetLocalPort(SIM900StateHandlePtr SIM900_State_Ptr, const uint8_t* mode, const uint16_t port);

//! Saves the configuration into the internal NVRAM of the GPRS module
/*!
\param void
\return '1' on success, '0' if error
 */
uint8_t SIM900SaveGPRS_TCP_UDPconfiguration(SIM900StateHandlePtr SIM900_State_Ptr);


//! Creates a TCP/IP connection to the specified IP and PORT
/*!
\param uint8_t working_mode : TCP_SERVER, TCP_CLIENT, UDP_CLIENT or UDP_EXTENDED
\param uint8_t n_connection : number of the connection id to use (0-7)
\param const char* ip : the IP to open a socket to
\param const char* port : the PORT to open a socket to
\return '1' on success, '0' if error setting the connection,
	'-2' if error setting the connection with CME error code available
	and '-3' if time out waiting the connection
 */
int8_t SIM900CreateSocket(SIM900StateHandlePtr SIM900_State_Ptr, const SIM900_working_mode working_mode, const uint8_t n_connection, const uint8_t* ip, const uint8_t* port);

//! Sends 'data' to the specified 'socket'
/*!
\param uint8_t* data : the data to send to the socket, or 0 for single socket
\param int length : the length of the data for send to the socket
\param uint8_t n_connection: the connection's number
\return '1' on success, '0' if error waiting the response of the module,
	'-2' if error with CME error code available
	'-3' if no feedback detected and '-4' if the send fails
*/
int8_t SIM900SendData(SIM900StateHandlePtr SIM900_State_Ptr, uint8_t* data, uint16_t length, uint8_t n_connection);

//! Gets data receive from a TCP or UDP connection and stores it in 'buffer_packet'
/*!
\param char* dataIN : string of data with TCP/UDP info
\param int length : the length of the data for receive
\return '1' on success, '0' if error
 */
int8_t SIM900ReadIPData(SIM900StateHandlePtr SIM900_State_Ptr, uint8_t* dataIN, uint16_t length);


//! Closes TCP/IP connection
/*!
\param uint8_t n_connection: the connection's number
\return '1' on success, '0' if error
 */
uint8_t SIM900CloseSocket(SIM900StateHandlePtr SIM900_State_Ptr, uint8_t n_connection);

//! Enables/disables to close the connection quickly
/*!
\param uint8_t mode: ENABLE or DISABLE
\return '1' on success, '0' if error
 */
uint8_t SIM900QuickCloseSocket(SIM900StateHandlePtr SIM900_State_Ptr, uint8_t mode);

//! Gets the IP address from a URL using DNS servers
/*! It stores the address in 'buffer_GPRS'
\param const char* IP_query : URL to obtain the IP address
\return '1' on success, '0' if error
 */
uint8_t SIM900GetIPfromDNS(SIM900StateHandlePtr SIM900_State_Ptr, const char* IP_query);

//! Adds an IP header at the beginning of a package for transmission
/*!
\param uint8_t on_off : ENABLE or DISABLE
\return '1' on success, '0' if error
 */
int8_t SIM900AddIPHeader(SIM900StateHandlePtr SIM900_State_Ptr, uint8_t on_off);

//! Sets a timer when module is sending data
/*!
\param uint8_t mode : ENABLE or DISABLE
\param uint8_t time : time in seconds from 0 to 100
\return '1' on success, '0' if error
 */
int8_t SIM900SetAutoSendingTimer(SIM900StateHandlePtr SIM900_State_Ptr, uint8_t mode, uint8_t time);

//! Enables or disables to show remote IP address and port when received data
/*!
\param uint8_t on_off : ENABLE or DISABLE
\return '1' on success, '0' if error
 */
int8_t SIM900ShowRemoteIP(SIM900StateHandlePtr SIM900_State_Ptr, uint8_t on_off);

//! Enables or disables to show transfer protocol in IP head when received data
/*!
\param uint8_t on_off : ENABLE or DISABLE
\return '1' on success, '0' if error
 */
int8_t SIM900ShowProtocolHeader(SIM900StateHandlePtr SIM900_State_Ptr, uint8_t on_off);

//! Enables or disables to discard input AT data in TCP data send
/*!
\param uint8_t on_off : ENABLE or DISABLE
\return '1' on success, '0' if error
 */
int8_t SIM900DiscardInputATData(SIM900StateHandlePtr SIM900_State_Ptr, uint8_t on_off);

//! Enables or disables to get data manually from a TCP or UDP connection
/*!
\param uint8_t on_off : ENABLE or DISABLE
\param uint8_t id : id connection number
\return '1' on success, '0' if error
*/
int8_t SIM900SetDataManually(SIM900StateHandlePtr SIM900_State_Ptr, uint8_t on_off, uint8_t id);

//! Gets data manually from a TCP or UDP connection
/*!
\param uint16_t data_length : the length of the data to get
\param uint8_t id : id connection number
\return '1' on success, '0' if error and '2' if buffer_GPRS is full. The answer from the server
	is limited by the length of buffer_GPRS. To increase the length	of the answer, increase the SIM900_BUFFER_SIZE constant.
 */
int8_t SIM900GetDataManually(SIM900StateHandlePtr SIM900_State_Ptr, uint16_t data_length, uint8_t id);


//! Sets the address of DNS servers from lib_SIM900Constants.h
/*!
\return '1' on success, '0' if error and '-2' if CME error code available
 */
int8_t SIM900SetDNS(SIM900StateHandlePtr SIM900_State_Ptr);

//! Sets the addresses of DNS servers
/*!
\param const char* DNS_dir1 : DNS server direction
\param const char* DNS_dir2 : DNS server direction
\return '1' on success, '0' if error and '-2' if CME error code available
 */
int8_t SIM900ChangeDNS(SIM900StateHandlePtr SIM900_State_Ptr, const char* DNS_dir1, const char* DNS_dir2);

//! Gets the model of the module and saves it in 'buffer_GPRS'
/*!
\return '1' on success, '0' if error
 */
uint8_t SIM900WhoAmI(SIM900StateHandlePtr SIM900_State_Ptr);

//! Gets the firmware version of the module and saves it in 'buffer_GPRS'
/*!
\return '1' on success, '0' if error
 */
uint8_t SIM900FirmwareVersion(SIM900StateHandlePtr SIM900_State_Ptr);

//! Shows the apn, login and password constants
/*!
\return '1' on success, '0' if error
 */
void SIM900ShowAPN(SIM900StateHandlePtr SIM900_State_Ptr);


//////////////////////////////////////////////////////

// Power functions

/* SIM900PowerOn(void) - opens USART1 and powers the SIM900 module
 *
 * Opens USART0 or USART1, allocates command and receive buffers and enables the SIM900 module
 *
 * Returns SIM900SetMode()
*/
int8_t SIM900PowerOn(SIM900StateHandlePtr SIM900_State_Ptr, eCOMPort ePort, uint8_t command_buffer_size, uint16_t  packet_buffer_size);

/* SIM900Off(void) - closes USART1 and powers off the SIM900 module
 *
 * This function powers off the SIM900 module and closes USART1
 *
 * Returns nothing
*/
void SIM900Off(SIM900StateHandlePtr SIM900_State_Ptr);

/* SIM900SetMode(uint8_t) - Sets GPRS Power Mode
 *
 * This function selects the active power mode among: ON, SLEEP/HIBERNATE and OFF
 * It does not close the serial port, only sends the proper command to GPRS module
 *
 * Returns '1' on success, '0' if error and '-2' if error with CME error code available
*/
int8_t SIM900SetPowerMode(SIM900StateHandlePtr SIM900_State_Ptr, SIM900_POWER_state pwrMode);

/* SIM900GetMode(void) - Gets SIM900 Power Mode
 *
 * This function gets the actual SIM900 Power Mode. Possible values are ON, FULL, RF_OFF, MIN, SLEEP and POWER_OFF
 *
 * Returns the power mode
*/
SIM900_POWER_state SIM900GetPowerMode(SIM900StateHandlePtr SIM900_State_Ptr);

/* SIM900CheckNetwork(uint16_t) - Checks if GPRS is connected to the network
 *
 * This function checks if GPRS module is connected to the network. If not, it has no sense working with GPRS.
 *
 * It sends a command to GPRS module DEFAULT_TIMEOUT times. If GPRS module does not connect within these tries, function
 * exits.
 *
 * Returns '1' when connected and '0' if not
*/
uint8_t SIM900CheckNetwork(SIM900StateHandlePtr SIM900_State_Ptr, uint16_t time);

//SIM functions
/* SIM900SetPIN(const char*) - sets PIN to the SIM
 *
 * This function sets the specified PIN to the SIM
 *
 * Returns '1' on success, '0' if error and '-2' if CME error code available
*/
int8_t	SIM900SetPIN(SIM900StateHandlePtr SIM900_State_Ptr, const char* pin);

/* SIM900ChangePIN(const char*, const char*) - changes PIN number to the SIM
 *
 * This function changes the PIN number to the SIM
 *
 * Returns '1' on success, '0' if error and '-2' if CME error code available
*/
int8_t SIM900ChangePIN(SIM900StateHandlePtr SIM900_State_Ptr, const char* old_pin, const char* new_pin);

/* SIM900GetIMEI() - gets the IMEI from the SIM card
 *
 * This function gets the IMEI from the SIM card. It stores the IMEI into responseIMEI variable.
 *
 * Returns '1' on success and '0' if error
*/
uint8_t SIM900GetIMEI(SIM900StateHandlePtr SIM900_State_Ptr, char * responseIMEI);

/* SIM900GetIMSI() - gets the IMSI from the SIM card
 *
 * This function gets the IMSI from the SIM card. It stores the IMSI into 'buffer_GPRS' variable.
 *
 * Returns '1' on success, '0' if error and '-2' if CME error code available
*/
int8_t SIM900GetIMSI(SIM900StateHandlePtr SIM900_State_Ptr, char * responseIMSI);

//Call functions
/* makeCall(SIM900StateHandlePtr, const char*) - makes a call to the selected telephone number
 *
 * This function makes a call to the given telephone number.
 *
 * Returns '1' on success, '0' if error
*/
uint8_t SIM900MakeCall(SIM900StateHandlePtr SIM900_State_Ptr, const char* tlfNumber);

/* SIM900Redial(SIM900StateHandlePtr) - hangs the call up
 *
 * This function redials the last active call.
 *
 * Returns '1' on success and '0' if error
*/
uint8_t SIM900Redial(SIM900StateHandlePtr SIM900_State_Ptr);

/* SIM900Answer(SIM900StateHandlePtr) - hangs the call up
 *
 * This function hangs all the active calls up.
 *
 * Returns '1' on success and '0' if error
*/
uint8_t SIM900Answer(SIM900StateHandlePtr SIM900_State_Ptr);

/* SIM900hangUp(SIM900StateHandlePtr) - hangs the call up
 *
 * This function hangs all the active calls up.
 *
 * Returns '1' on success and '0' if error
*/
uint8_t SIM900HangUp(SIM900StateHandlePtr SIM900_State_Ptr);


/* SIM900ManageIncomingData() - manage incoming data from serial port, executing proper functions to store received data
 *
 * This function manages incoming data from serial port, executing proper functions to store received data
 *
 * Returns '1' for call, '2' for SMS, '3' for IP data and '0' for error or not data
*/
int8_t	SIM900ManageIncomingData(SIM900StateHandlePtr SIM900_State_Ptr);

/*SIM900witchToDataMode() - switches from command mode to data mode
 *
 * This function switches from command mode to data modes
 *
 * Returns '1' on success, '0' if error and '-2' if connection is not successfully resumed
*/
int8_t SIM900witchToDataMode(SIM900StateHandlePtr SIM900_State_Ptr);

/*SIM900SwitchToCommandMode() - switches from data mode to command mode
 *
 * This function switches from data mode to command mode
 *
 * Returns '1' on success and '0' if error
*/
int8_t SIM900SwitchToCommandMode(SIM900StateHandlePtr SIM900_State_Ptr);

/* SIM900SendCommand(const char* ATcommand) - sends any command to GPRS module
 *
 * This function sends any command to GPRS module
 *
 * It stores in 'buffer_GPRS' variable the answer returned by the GPRS module
 *
 * Returns '1' on success, '0' if error
*/
int8_t SIM900SendCommand(SIM900StateHandlePtr SIM900_State_Ptr, const char* ATcommand);

/* SIM900GetCurrentOperator() - Gets the currently selected operator from network
 *
 * This function gets the currently selected operator from network and stores it in 'operator_name'
 *
 * Returns '1' on success, '0' if error and '-2' if CME error code available
*/
int8_t SIM900GetCurrentOperator(SIM900StateHandlePtr SIM900_State_Ptr);

/* SIM900GetAvailableOperators() - Gets the currently available operators from network
 *
 * This function gets the currently available operators from network and stores it in 'operators_list'
 *
 * Returns '1' on success, '0' if error and '-2' if CME error code available
*/
int8_t SIM900GetAvailableOperators(SIM900StateHandlePtr SIM900_State_Ptr);

/* SIM900SetPreferredOperator(int, uint8_t, const char*) - Sets the preferred operator in the operators list into GPRS module
 *
 * This function sets the preferred operator in the operators list into GPRS module
 *
 * Returns '1' on success, '0' if error and '-2' if CME error code available
*/
int8_t SIM900SetPreferredOperator(SIM900StateHandlePtr SIM900_State_Ptr, int index, uint8_t format, const char* preferred_operator);

/* SIM900GetCellInfo() - gets the information from the cell where the module is connected
 *
 * This function gets the information from the cell where the module is connected
 *
 * It stores in 'RSSI' and 'cellID' variables the information from the cell
 *
 * Returns '1' on success, '0' if error and '-2' if CME error code available
*/
int8_t SIM900GetCellInfo(SIM900StateHandlePtr SIM900_State_Ptr);


#endif /* LIB_SIM900_H_ */
