/*
Copyright 2011 Niels Brouwers

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.#include <string.h>
*/
#ifndef __adb_h__
#define __adb_h__

#ifdef __cplusplus
extern "C" {
#endif



// ADB
#define MAX_PAYLOAD 4096;

#define A_SYNC 0x434e5953
#define A_CNXN 0x4e584e43
#define A_OPEN 0x4e45504f
#define A_OKAY 0x59414b4f
#define A_CLSE 0x45534c43
#define A_WRTE 0x45545257

#define ADB_CLASS 0xff
#define ADB_SUBCLASS 0x42
#define ADB_PROTOCOL 0x1

#define ADB_USB_PACKETSIZE 0x40
#define ADB_CONNECTION_RETRY_TIME 1000 / portTICK_RATE_MS // should be 1000mS

typedef struct
{
	uint8_t address;
	uint8_t configuration;
	uint8_t interface;
	uint8_t inputEndPointAddress;
	uint8_t outputEndPointAddress;
} adb_usbConfiguration;

typedef struct
{
	// Command identifier constant
	uint32_t command;

	// First argument
	uint32_t arg0;

	// Second argument
	uint32_t arg1;

	// Payload length (0 is allowed)
	uint32_t data_length;

	// Checksum of data payload
	uint32_t data_check;

	// Command ^ 0xffffffff
	uint32_t magic;

} adb_message;

typedef enum
{
	ADB_UNUSED = 0,
	ADB_CLOSED,
	ADB_OPEN,
	ADB_OPENING,
	ADB_RECEIVING,
	ADB_WRITING
} adb_connectionStatus;

typedef enum
{
	ADB_CONNECT = 0,
	ADB_DISCONNECT,
	ADB_CONNECTION_OPEN,
	ADB_CONNECTION_CLOSE,
	ADB_CONNECTION_FAILED,
	ADB_CONNECTION_RECEIVE
} adb_eventType;

typedef struct _adb_connection adb_connection;

// Event handler
typedef void(adb_eventHandler)(adb_connection * connection, adb_eventType event, uint16_t length, uint8_t * data);

struct _adb_connection
{
	char * connectionString;
	uint32_t localID, remoteID;
//	uint32_t lastConnectionAttempt;
	uint16_t lastConnectionAttempt;   // from freeRTOS in ticks (not millis), so can count less.
	uint16_t dataSize, dataRead;
	adb_connectionStatus status;
	boolean reconnect;
	adb_eventHandler * eventHandler;
	adb_connection * next;
};

void adb_init();
void adb_poll();

void adb_setEventHandler(adb_eventHandler * handler);
adb_connection * adb_addConnection(const char * connectionString, boolean reconnect, adb_eventHandler * eventHandler);
int adb_write(adb_connection * connection, uint16_t length, uint8_t * data);
int adb_writeString(adb_connection * connection, char * str);

#ifdef __cplusplus
}
#endif

#endif
