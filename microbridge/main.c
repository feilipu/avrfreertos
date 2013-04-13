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
#include "avr.h"
#include "adb.h"

adb_connection * logcat, * connection;

void adbEventHandler(adb_connection * connection, adb_eventType event, uint16_t length, uint8_t * data)
{
	int i;

	switch (event)
	{
	case ADB_CONNECT:
		xSerialPrintf("ADB EVENT CONNECT\n");
		break;
	case ADB_DISCONNECT:
		xSerialPrintf("ADB EVENT DISCONNECT\n");
		break;
	case ADB_CONNECTION_OPEN:
		xSerialPrintf("ADB EVENT OPEN connection=[%s]\n", connection->connectionString);
		break;
	case ADB_CONNECTION_CLOSE:
		xSerialPrintf("ADB EVENT CLOSE connection=[%s]\n", connection->connectionString);
		break;
	case ADB_CONNECTION_FAILED:
		xSerialPrintf("ADB EVENT FAILED connection=[%s]\n", connection->connectionString);
		break;
	case ADB_CONNECTION_RECEIVE:

		for (i=0; i<length; i++)
			xSerialPrintf("%c", data[i]);

		break;
	}

}

int main()
{
	// Initialise avr timers
	avr_timerInit();

	// Initialise serial port
	avr_serialInit(57600);

 	// Initialise USB host shield.
	adb_init();
	logcat = adb_addConnection("shell:exec logcat -s microbridge:*", true, adbEventHandler);
	connection = adb_addConnection("tcp:4567", true, adbEventHandler);

	// Init ADC
	uint32_t lastTime = avr_millis();

	while (1)
 	{
		adb_poll();

		DDRC = 0xff;
		PORTC |= 1;
		PORTC |= 0x2;
		PORTC &= ~0x4;

		if ((avr_millis()-lastTime) > 100)
		{
//			// Set VCC as reference, enable left alignment of results, select ADC channel 1
//			ADMUX = (1<<REFS0) | (1<<ADLAR) | 1;
//
//			// Enable ADC
//			ADCSRA |= (1<<ADEN);
//
//			// Start ADC sample
//			ADCSRA |= (1<<ADSC);
//			while (ADCSRA&(1<<ADSC));
//
//			uint8_t value = ADCH;
//
//			xSerialPrintf("ADC[0]: %d\n", value);
//
//			if (connection->status==ADB_OPEN)
//				adb_write(connection, 1, &value);

			lastTime = avr_millis();
		}

 	}

	return 0;
}
