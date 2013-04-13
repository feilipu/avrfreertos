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
limitations under the License.
*/

#include "avr.h"

/*
void avr_serialInit(uint32_t baud)
{
	uint16_t baud_setting;

	UCSR0A = 0;

	baud_setting = (F_CPU / 8 / baud - 1) / 2;
    // baud_setting = (CLOCKSPEED + (baud * 8L)) / (baud * 16L) - 1;

    // assign the baud_setting, a.k.a. UBBR (USART Baud Rate Register)
    UBRR0H = baud_setting >> 8;
    UBRR0L = baud_setting;

    sbi(UCSR0B, RXEN0);		// Enable Receive
    sbi(UCSR0B, TXEN0);		// Enable Transmit
	sbi(UCSR0C, UCSZ01);	// Frame format: 8data, No parity, 1stop bit
	sbi(UCSR0C, UCSZ00);	// Frame format: 8data, No parity, 1stop bit
}*/


/*
void avr_serialWrite(unsigned char DataOut)
{
	while (!avr_serialCheckTxReady())		// while NOT ready to transmit
        taskYIELD();     					// keep yielding until serial port ready
	UDR0 = DataOut;
}

unsigned char avr_serialRead(void)
{
	while (!avr_serialCheckRxComplete())	// While data is NOT available to read
        taskYIELD();     					// keep yielding until serial port ready
	return UDR0;
}

unsigned char avr_serialCheckRxComplete(void)
{
	return( UCSR0A & (1 << RXC0) );			// nonzero if serial data is available to read.
}

unsigned char avr_serialCheckTxReady(void)
{
	return( UCSR0A & (1 << UDRE0) );		// nonzero if transmit register is ready to receive new data.
}
*/
