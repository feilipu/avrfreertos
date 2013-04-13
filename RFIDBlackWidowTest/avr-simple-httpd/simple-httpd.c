#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <avr/io.h>
#include <avr/pgmspace.h>

#include "../avr-uIP/uip.h"
#include "simple-httpd.h"


/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

/* serial interface include file. */
#include <lib_serial.h> // temporary

/*------------------------------------------------------*/

// just any variable to write into the webpage, until we build the proper model
unsigned char mfg_id[4] = {0xDE, 0xAD, 0xBE, 0xEF };

// This is the html page that is served up by the httpd
// The variables values are entered later by the sprintf memcpy to overwrite the correct characters.
const uint8_t PROGMEM webpage[] PROGMEM = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n"
									"<html><body style=\"margin:100px\">"
									"<center>"
									"<h1>Flash Mfg ID: 000 000 000</h1>"
									"<form method=post action=\"/upload\" enctype=\"multipart/form-data\">"
									"<b>FS Image Upload</b>"
									"<p>"
									"<input type=file name=i size=40> &nbsp; <input type=submit value=\"Upload\">"
									"</form>"
									"</center>"
									"</body></html>";

/*------------------------------------------------------*/

static int handle_connection(struct simple_httpd_state *s);
static unsigned short fill_buf(void* blk);

/*------------------------------------------------------*/

void simple_httpd_init(void)
{
	uip_listen(HTONS(80));
}

void simple_httpd_appcall(void)
{
	struct simple_httpd_state *s = &(uip_conn->appstate);

	if(uip_connected()) {
		PSOCK_INIT(&s->p, s->inputbuf, sizeof(s->inputbuf));
	}

  handle_connection(s);

}

/*------------------------------------------------------*/

static int handle_connection(struct simple_httpd_state *s)
{
	PSOCK_BEGIN(&s->p);

	const char http_get[5] = {0x47, 0x45, 0x54, 0x20, };	/* "GET " */

	// the incoming GET request will have the following format:
	// GET / HTTP/1.1 ....
	// we have to parse this string to determine the resource being requested
	// if the requested resource is not the root webpage ('/') then,
	// GET /<resource name> HTTP/1.1 ....
	// we should parse the specific resource and react appropriately

	// read incoming data until we read a space character
	PSOCK_READTO(&s->p, ISO_space);

	// parse the data to determine if it was a GET request
	if(strncmp(s->inputbuf, http_get, 4) != 0){
		PSOCK_CLOSE_EXIT(&s->p);
	}

	// continue reading until the next space character
	PSOCK_READTO(&s->p, ISO_space);

	// determine the requested resource
	// in this case, we check if the request was for the '/' root page
	// AKA index.html
	if(s->inputbuf[0] != ISO_slash){
		PSOCK_CLOSE_EXIT(&s->p);				// request for unknown webpage, close and exit
	}

	// not supported, modify to add support for additional resources
	if(s->inputbuf[1] != ISO_space){
		PSOCK_CLOSE_EXIT(&s->p);			// request for unavailable resource
	}

	PSOCK_GENERATOR_SEND(&s->p, fill_buf, 0); 	// generate the web page response with fill_buf from PSTR variable

	PSOCK_CLOSE(&s->p);
	PSOCK_END(&s->p);
}

unsigned short fill_buf(void* blk)
{
	uint16_t webpage_len;
	char int_string[4];

	webpage_len = (strlen_P(webpage)>uip_mss())?uip_mss():strlen_P(webpage);

	xSerialPrintf_P(PSTR("fill_buf : %s\r\n"), webpage);

	memcpy_P(uip_appdata, webpage, webpage_len);

	sprintf(int_string, "%X", mfg_id[0]);
	memcpy(&uip_appdata[104], int_string, 2);
	sprintf(int_string, "%X", mfg_id[1]);
	memcpy(&uip_appdata[108], int_string, 2);
	return webpage_len;
}
