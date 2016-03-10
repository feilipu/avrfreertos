/*
@file	webserver.c
@brief	Webserver example program
*/

#include <stdio.h>
#include <string.h>

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

#include <util/delay.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


#include "ff.h"			// SD file handling

#include "socket.h"
#include "inet.h"

#include "time.h"

#if defined (portRTC_DEFINED)
#include "rtc.h"		/* RTC interface include file. */
#endif

#ifdef WEB_DEBUG
#include "serial.h" // serial port for diagnostics
#endif

#define EVB_PAGES_SERVED	"$PAGES_SERVED$"// 14 characters (to erase) including \0
#define EVB_RTC				"$REAL_TIME$"	// 11 characters (to erase) including \0

#define WEBSERVER_SOCKET_TIMEOUT 1500		// Timeout in milliseconds. Wait for up to 1.5 seconds.


extern HTTP_REQUEST *pHTTPRequest;			// < Pointer to HTTP request  - declared in http.c
extern uint8_t *pHTTPResponse;				// < Pointer to HTTP response - declared in http.c

extern uint32_t pagesServed EEMEM;			// non-volatile storage for total pages served.

static uint16_t replace_sys_env_value(uint8_t* base, uint16_t len);	// Replace HTML's variables to system configuration value

/**
 @brief	Analyse HTTP request and then services WEB.
*/
void process_HTTP(
	SOCKET s, 			/**< http server socket */
	uint8_t * buffer, 	/**< buffer pointer included http request */
	uint16_t length		/**< length of http request */
	)
{
	uint8_t * name;
	uint16_t bytes_read;
	TickType_t wait_send;
	FIL source_file;	/* File object for the source file */

	parse_HTTP_request(pHTTPRequest, buffer);			// After analysing request, convert into pHTTPRequest

	/* method Analyse */
	switch (pHTTPRequest->METHOD)
	{
		case METHOD_HEAD:
		case METHOD_GET :
		case METHOD_POST :

			name = get_HTTP_URI_name(pHTTPRequest->URI);

			if (!strcmp((const char *)name, "/")) strcpy((char *)name,"index.htm");	// If URI is "/", respond by index.htm

#ifdef WEB_DEBUG
			if(strlen( (const char *)name) < MAX_INT_STR ) xSerialPrintf_P(PSTR("\r\nPAGE : %s "), name);
			else xSerialPrint_P(PSTR("\r\nFILENAME TOO LONG"));
#endif

			find_HTTP_URI_type(&pHTTPRequest->TYPE, name);	//Check file type (HTML, TEXT, ICO, GIF, JPEG, ZIP are included)

			// OK now we start to respond to the info we've decoded.

			/* Open the specified file stored in the SD card FAT32 */
			if (f_open(&source_file, (const TCHAR *)name, FA_OPEN_EXISTING | FA_READ))
			{	// if file open failure

				memcpy_P( (char *)pHTTPResponse, ERROR_HTML_PAGE, strnlen_P(ERROR_HTML_PAGE, FILE_BUFFER_SIZE) );

#ifdef WEB_DEBUG
				xSerialPrint_P(PSTR("HTTP Unknown file or page.\r\n"));
				xSerialPrintf_P(PSTR("HTTP Response...\r\n%s\r\nResponse Size: %u \r\n"), pHTTPResponse, strlen_P(ERROR_HTML_PAGE));
#endif

				send( s, (const uint8_t*)pHTTPResponse, strlen_P(ERROR_HTML_PAGE));

			}
			else
			{	// if file open success

				make_HTTP_response_header( pHTTPResponse, pHTTPRequest->TYPE, source_file.fsize);

#ifdef WEB_DEBUG
				xSerialPrintf_P(PSTR("HTTP Opened file: %s  Source Size: %u \r\n"), name, source_file.fsize);
				xSerialPrintf_P(PSTR("HTTP Response Header...\r\n%s\r\nResponse Header Size: %u \r\n"), pHTTPResponse, strlen((char*)pHTTPResponse ));
#endif

				send(s, (const uint8_t*)pHTTPResponse, strlen((char*)pHTTPResponse ));

				wait_send = xTaskGetTickCount();

				while(getSn_TX_FSR(s)!= WIZCHIP_getTxMAX(s))

				{
					if( (xTaskGetTickCount() - wait_send) > (WEBSERVER_SOCKET_TIMEOUT / portTICK_PERIOD_MS) ) // wait up to 1.5 Sec
					{
#ifdef WEB_DEBUG
						xSerialPrint_P(PSTR("HTTP Response head send fail\r\n"));
#endif
						break;
					}
					vTaskDelay( 0 ); // yield until next tick.
				}

				for (;;)
				{
					if ( f_read(&source_file, pHTTPResponse, (sizeof(uint8_t)*(FILE_BUFFER_SIZE) ), &bytes_read) || bytes_read == 0 )
						break;   // read error or reached end of file

					if(pHTTPRequest->TYPE == PTYPE_HTML) // if we've got a html document, there might be some system variables to set
					{
						*(pHTTPResponse + bytes_read + 1) = 0; // make the buffer a string, null terminated
						bytes_read = replace_sys_env_value(pHTTPResponse, bytes_read); // Replace html system environment value to real value
					}

					if (send(s, (const uint8_t*)pHTTPResponse, bytes_read) != bytes_read)
						break;  // TCP/IP send error

					wait_send = xTaskGetTickCount();

					while(getSn_TX_FSR(s)!= WIZCHIP_getTxMAX(s))

					{
						if( (xTaskGetTickCount() - wait_send) > (WEBSERVER_SOCKET_TIMEOUT / portTICK_PERIOD_MS) ) // wait up to 1.5 Sec
						{
#ifdef WEB_DEBUG
							xSerialPrint_P(PSTR("HTTP Response body send fail\r\n"));
#endif
							break;
						}
						vTaskDelay( 0 ); // yield until next tick.
					}
				}
				f_close(&source_file);

				eeprom_busy_wait();
				eeprom_update_dword( &pagesServed, eeprom_read_dword(&pagesServed) +1 );
			}
			break;

		case METHOD_ERR :

			memcpy_P( (char *)pHTTPResponse, ERROR_REQUEST_PAGE, strnlen_P(ERROR_REQUEST_PAGE, FILE_BUFFER_SIZE) );

#ifdef WEB_DEBUG
			xSerialPrint_P(PSTR("HTTP Method Error.\r\n"));
			xSerialPrintf_P(PSTR("HTTP Response...\r\n%s\r\nResponse Size: %u \r\n"), pHTTPResponse, strlen_P(ERROR_REQUEST_PAGE));
#endif

			send( s, (const uint8_t*)pHTTPResponse, strlen_P(ERROR_REQUEST_PAGE));

			break;

		default :
			break;
	}
}

/**
 @brief	Replace HTML's variables to system configuration value
*/
static uint16_t replace_sys_env_value(
	uint8_t* base, 	/**< pointer to html buffer */
	uint16_t len
	)
{
	uint8_t sys_string[16]; // if there are more characters to substituted in response file, then make this bigger
	uint8_t *tptr, *ptr;
	time_t time_stamp;
	tm local_time;

	tptr = ptr = base;

	while((ptr = (uint8_t*)strchr((char*)tptr, '$')))
	{
		if((tptr = (uint8_t*)strstr((char*)ptr, EVB_PAGES_SERVED)))
		{
			memset(tptr,0x20,14); // erase system variable trigger string
			if( eeprom_is_ready() )
				sprintf_P((char *)sys_string, PSTR("%08u"), eeprom_read_dword(&pagesServed) ); // number of pages served by the http server.
			memcpy(tptr,sys_string,8); // copy the characters, but don't copy the /0
			tptr+=8;
		}

#if defined (portRTC_DEFINED)
		else if((tptr = (uint8_t*)strstr((char*)ptr, EVB_RTC)))
		{
			memset(tptr,0x20,11); // erase system variable trigger string
			if ( getDateTimeDS1307( (tm*)&local_time ) == pdTRUE)
				sprintf_P((char *)sys_string, PSTR("%02u:%02u:%02u"), local_time.tm_hour, local_time.tm_min, local_time.tm_sec);
			memcpy(tptr,sys_string,8);  // copy the characters, but don't copy the /0
			tptr+=8;
		}
#else
		else if((tptr = (uint8_t*)strstr((char*)ptr, EVB_RTC)))
		{
			memset(tptr,0x20,11); // erase system variable trigger string
	    	time(&time_stamp);
	    	localtime_r( &time_stamp, &local_time);
			sprintf_P((char *)sys_string, PSTR("%02d:%02d:%02d"), local_time.tm_hour, local_time.tm_min, local_time.tm_sec);
			memcpy(tptr,sys_string,8);  // copy the characters, but don't copy the /0
			tptr+=8;
		}
#endif
		else	// tptr == 0 && ptr!=0;
		{
			if(ptr==base)
			{
//				xSerialPrint_P(PSTR("$ Character"));
				return len;
			}
//			xSerialPrint_P(PSTR("REPLACE CONTINUE"));
			tptr = ptr;
			break;
		}
	}
	if(!ptr) return len;
	return (uint16_t)(tptr-base);
}
