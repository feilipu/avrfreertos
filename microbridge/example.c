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

// Event handler to process incoming data from ADB.
void adbEventHandler(adb_connection * connection, adb_eventType event, uint16_t length, uint8_t * data)
{
	int i;

	switch (event)
	{
	case ADB_CONNECTION_RECEIVE:

		// Write the results to UART
		for (i=0; i<length; i++)
			xSerialPrintf("%c", data[i]);

		break;
	default:
		break;
	}

}

int main()
{
	// Initialise avr timers
	avr_timerInit();

	// Initialise serial port
	avr_serialInit(57600);

 	// Initialise ADB connection
	adb_init();

	// Create a new ADB connection, run logcat
	adb_addConnection("shell:exec logcat -s MYAPP:*", false, adbEventHandler);

	// ADB polling.
	while (1)
		adb_poll();

	return 0;
}
