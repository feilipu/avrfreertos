/**
 @file		http.c
 @brief 	functions associated with http processing
 */

#include <stdio.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "serial.h"

#include "inet.h"

#if defined(_WIZCHIP_)		// Definition in freeRTOSBoardDefs.h

HTTP_REQUEST *pHTTPRequest;	// < Pointer to HTTP request buffer
uint8_t *pHTTPResponse;		// < Pointer to HTTP response buffer

static SOCKET HTTPD_SOCK;			// < Socket for the HTTP daemon.

/**
 * @brief		Initialise the socket & buffer for HTTP server
 */
uint8_t init_HTTP(SOCKET s)
{

	if( getSn_SR(s) != SOCK_CLOSED )	// Check the preferred socket is available,
	{
		s = getSocket(SOCK_CLOSED, 0);	// otherwise find free socket,
		if(s == _WIZCHIP_MAX_SOC_NUM_ )        	// If there is no free socket?
			return 0;
	}

	if(!socket(s, Sn_MR_TCP, IP_PORT_HTTP, 0x00)) // initialise the socket for DHCP service
	{
		xSerialPrintf_P(PSTR("HTTPD socket: %d, initialise fail..!\r\n"),s);
		return 0;
	}
#ifdef HTTP_DEBUG
	else
		xSerialPrintf_P(PSTR("HTTPD socket: %d, initialise success..!\r\n"),s);
#endif

	if(pHTTPRequest == NULL) // if there is no buffer allocated (pointer is NULL), then allocate request buffer for all HTTP functions.
	{
		if( !(pHTTPRequest = (HTTP_REQUEST *) pvPortMalloc( sizeof(HTTP_REQUEST) )))
		{
			xSerialPrint_P(PSTR("HTTP Request Buffer: malloc fail..!\r\n"));
			return 0;
		}
#ifdef HTTP_DEBUG
		else
			xSerialPrint_P(PSTR("HTTP Request Buffer: malloc success..!\r\n"));
#endif
	}

	if(pHTTPResponse == NULL) // if there is no buffer allocated (pointer is NULL), then allocate response buffer for all HTTP functions.
	{
		if( !(pHTTPResponse = (uint8_t *) pvPortMalloc( sizeof(uint8_t) * (FILE_BUFFER_SIZE + 1) )))
		{
			xSerialPrint_P(PSTR("HTTP Response Buffer: malloc fail..!\r\n"));
			vPortFree(pHTTPRequest);
			return 0;
		}
#ifdef HTTP_DEBUG
		else
			xSerialPrint_P(PSTR("HTTP Response Buffer: malloc success..!\r\n"));
#endif
	}

	HTTPD_SOCK = s;

	return 1;
}



SOCKET get_HTTP_socket(void)				// Get the socket assigned for HTTPD
{
	return HTTPD_SOCK;
}


/**
 @brief	convert escape characters(%XX) to ascii character
 */
void unescape_HTTP_URL(
	uint8_t * url	/**< pointer to be converted ( escape characters )*/
	)
{
	uint16_t x, y;

	for (x = 0, y = 0; url[y]; ++x, ++y) {
		if ((url[x] = url[y]) == '%') {
			url[x] = C2D(url[y+1])*0x10+C2D(url[y+2]);
			y+=2;
		}
	}
	url[x] = '\0';
}


/**
 @brief	make response header such as html, gif, jpeg, zip, etc.
 */
void make_HTTP_response_header(
	uint8_t * buf, 	/**< pointer to response header to be made */
	uint8_t type, 	/**< response type */
	uint32_t len	/**< size of response file / document */
	)
{
	PGM_P head = 0;
	uint8_t tmp[16];

	switch (type) {

		case PTYPE_HTML :
			head = RES_HTMLHEAD_OK;
			break;

		case PTYPE_GIF :
			head = RES_GIFHEAD_OK;
			break;

		case PTYPE_ICO :
			head = RES_ICOHEAD_OK;
			break;

		case PTYPE_TEXT :
			head = RES_TEXTHEAD_OK;
			break;

		case PTYPE_JPEG :
			head = RES_JPEGHEAD_OK;
			break;

		case PTYPE_FLASH :
			head = RES_FLASHHEAD_OK;
			break;

		case PTYPE_MPEG :
			head = RES_MPEGHEAD_OK;
			break;

		case PTYPE_PDF :
			head = RES_PDFHEAD_OK;
			break;

		case PTYPE_ZIP :
			head = RES_ZIPHEAD_OK;
			break;

		default:
#ifdef HTTP_DEBUG
			xSerialPrint_P(PSTR("\r\n\r\nHTTP RESPONSE HEADER UNKNOWN-\r\n"));
#endif
			break;
	}

	sprintf((char *)tmp,"%ld", len);

	strcpy_P((char *)buf, head);
	strlcat((char *)buf, (const char *)tmp, 16);
	strcat_P((char *)buf, PSTR("\r\n\r\n\0"));
}


/**
 @brief	find MIME type of a file
 */
void find_HTTP_URI_type(
	uint8_t * type, 	/**< type to be returned */
	uint8_t * buf		/**< file name */
	)
{
	/* Decide type according to extension*/
	if 		(strstr_P((const char *)buf, PSTR(".html")) || strstr_P((const char *)buf, PSTR(".HTML")))	*type = PTYPE_HTML;
	else if (strstr_P((const char *)buf, PSTR(".htm" )) || strstr_P((const char *)buf, PSTR(".HTM")))	*type = PTYPE_HTML;
	else if (strstr_P((const char *)buf, PSTR(".text")) || strstr_P((const char *)buf, PSTR(".TEXT")))	*type = PTYPE_TEXT;
	else if (strstr_P((const char *)buf, PSTR(".txt" )) || strstr_P((const char *)buf, PSTR(".TXT")))	*type = PTYPE_TEXT;
	else if (strstr_P((const char *)buf, PSTR(".jpeg")) || strstr_P((const char *)buf, PSTR(".JPEG")))	*type = PTYPE_JPEG;
	else if (strstr_P((const char *)buf, PSTR(".jpg" )) || strstr_P((const char *)buf, PSTR(".JPG")))	*type = PTYPE_JPEG;
	else if (strstr_P((const char *)buf, PSTR(".ico" )) || strstr_P((const char *)buf, PSTR(".ICO")))	*type = PTYPE_ICO;
	else if (strstr_P((const char *)buf, PSTR(".gif" )) || strstr_P((const char *)buf, PSTR(".GIF")))	*type = PTYPE_GIF;
	else if (strstr_P((const char *)buf, PSTR(".mpeg")) || strstr_P((const char *)buf, PSTR(".MPEG")))	*type = PTYPE_MPEG;
	else if (strstr_P((const char *)buf, PSTR(".mpg" )) || strstr_P((const char *)buf, PSTR(".MPG")))	*type = PTYPE_MPEG;
	else if (strstr_P((const char *)buf, PSTR(".pdf" )) || strstr_P((const char *)buf, PSTR(".PDF")))	*type = PTYPE_PDF;
	else if (strstr_P((const char *)buf, PSTR(".zip" )) || strstr_P((const char *)buf, PSTR(".ZIP")))	*type = PTYPE_ZIP;
	else if (strstr_P((const char *)buf, PSTR(".swf" )) || strstr_P((const char *)buf, PSTR(".SWF")))	*type = PTYPE_FLASH;
	else if (strstr_P((const char *)buf, PSTR(".cgi" )) || strstr_P((const char *)buf, PSTR(".CGI")))	*type = PTYPE_CGI;
	else 																								*type = PTYPE_ERR;
}



/**
 @brief	parse http request from a peer
 */
void parse_HTTP_request(
	HTTP_REQUEST *request, 	/**< request to be returned */
	uint8_t *buffer			/**< pointer to be parsed */
	)
{
	uint8_t *nexttok;
	nexttok = (uint8_t *)strtok((char *)buffer," ");
	if(!nexttok)
	{
		request->METHOD = METHOD_ERR;
		return;
	}
	if 	(!strcmp_P((const char *)nexttok, PSTR("GET")) || !strcmp_P((const char *)nexttok, PSTR("get")))
	{
		request->METHOD = METHOD_GET;
		nexttok = (uint8_t *)strtok(NULL," ");
#ifdef HTTP_DEBUG
		xSerialPrint_P(PSTR("METHOD_GET "));
#endif
	}
	else if (!strcmp_P((const char *)nexttok, PSTR("HEAD")) || !strcmp_P((const char *)nexttok, PSTR("head")))
	{
		request->METHOD = METHOD_HEAD;
		nexttok = (uint8_t *)strtok(NULL," ");
#ifdef HTTP_DEBUG
		xSerialPrint_P(PSTR("METHOD_HEAD "));
#endif

	}
	else if (!strcmp_P((const char *)nexttok, PSTR("POST")) || !strcmp_P((const char *)nexttok, PSTR("post")))
	{
		nexttok = (uint8_t *)strtok((char *)NULL,"\0");
		request->METHOD = METHOD_POST;
#ifdef HTTP_DEBUG
		xSerialPrint_P(PSTR("METHOD_POST "));
#endif
	}
	else
	{
		request->METHOD = METHOD_ERR;
#ifdef HTTP_DEBUG
		xSerialPrint_P(PSTR("METHOD_ERR "));
#endif
	}

	if(!nexttok)
	{
		request->METHOD = METHOD_ERR;
#ifdef HTTP_DEBUG
		xSerialPrint_P(PSTR("METHOD_ERR "));
#endif
		return;
	}

	strcpy((char *)request->URI,(const char *)nexttok);

#ifdef HTTP_DEBUG
	{
		uint16_t i;
		xSerialPrint_P(PSTR("\nhttp_request->URI: "));
		for(i=0; i < strlen((const char *)request->URI);i++)
			xSerialPrintf_P(PSTR("%c"),request->URI[i]);
		xSerialPrint_P(PSTR(""));
	}
#endif

}


/**
 @brief	get next parameter value in the request
 */
uint8_t* get_HTTP_param_value(
	uint8_t* uri,
	uint8_t* param_name
	)
{
	uint8_t tempURI[MAX_URI_SIZE];
	uint8_t* name=0;
	if(!uri || !param_name) return 0;

	strlcpy((char *)tempURI, (const char *)uri, MAX_URI_SIZE);
	if((name=(uint8_t*)strstr((const char *)tempURI, (const char *)param_name)))
	{
		name += strlen((const char *)param_name) + 1; // strlen(para_name) + strlen("=")
		if((name = (uint8_t*)strtok((char *)name,"& \r\n\t\0")))
		{
			unescape_HTTP_URL((uint8_t*)name);
			replacetochar((uint8_t*)name,'+',' ');
		}
	}
#ifdef HTTP_DEBUG
	xSerialPrintf_P(PSTR("\n%s=%s "),param_name,name);
#endif
	return (uint8_t*)name;
}


uint8_t* get_HTTP_URI_name(uint8_t* uri)
{
	uint8_t tempURI[MAX_URI_SIZE];
	uint8_t* uri_name;
	if(!uri) return 0;
	strlcpy((char *)tempURI, (const char *)uri, MAX_URI_SIZE);
	uri_name = (uint8_t*)strtok((char *)tempURI, " ?");
	if(strcmp( (const char *)uri_name, "/" )) uri_name++;
#ifdef HTTP_DEBUG
	xSerialPrintf_P(PSTR("\nuri_name=%s "),uri_name);
#endif
	return (uint8_t*)uri_name;
}

#endif // #if   defined(_WIZCHIP_)		// Definition in freeRTOSBoardDefs.h
