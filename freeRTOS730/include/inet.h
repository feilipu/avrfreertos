/**
 * @file		inet.h
 * @brief 		functions for Internet, dhcp, ping, ntp, http, etc
 */

#ifndef _INET_H_
#define _INET_H_

#ifdef __cplusplus
extern "C" {
#endif

/* DEBUG DEFINES */

//#define DHCP_DEBUG
//#define HTTP_DEBUG
//#define WEB_DEBUG


/* SYSTEM DEFINES */

/* Ping Definitions. */
#define PING_OPT_LEN			1400	// length of a ping packet optional data (+8 bytes for full packet construct).
										// probably 32 bytes is enough, but if RAM is available... use it.
#define PING_ID					0xBEEF  // ID of pinging device

/* dhcp Definitions. */
#define DHCP_INITIAL_XID		0xDEADF00D // Initial XID value, incremented for each request.
#define DEFAULT_LEASETIME		0xffffffff // Infinite lease time.

/* http Definitions. */
#define HTTP_FS					1		// Drive number for HTTP - see _FS_LOCK in ffconfig.h
#define HTTP_PATH				"/http" // directory for web server. Files placed here will be seen in http root.
#define MAX_URI_SIZE			256 	// Length of the requested file name.
#define HTTP_PREFERRED_SOCKET	2		// This is just set to any socket. 2 is not special.

#if ( defined(portEXT_RAM) && !defined(portEXT_RAMFS) )
#define FILE_BUFFER_SIZE 		1300	// size of file working buffer (on heap) with extended RAM (set to under MTP, best efficiency).
										// On the wire 54 bytes added to this size.
#else
#define FILE_BUFFER_SIZE 		512	// size of file working buffer (on heap) for standard EtherMega
#endif


/* DHCP state machine. */
#define DHCP_PREFERRED_SOCKET	3		// This is just set to any socket. 3 is not special.

#define	DHCP_MAX_RETRY			3
#define	DHCP_WAIT_TIME			5

#define	DHCP_STATE_DISCOVER		1
#define	DHCP_STATE_REQUEST		2
#define	DHCP_STATE_LEASED		3
#define	DHCP_STATE_REREQUEST	4
#define	DHCP_STATE_RELEASE		5

#define DHCP_FLAGSBROADCAST		0x8000

/* DHCP message OP code */
#define DHCP_BOOTREQUEST		1
#define DHCP_BOOTREPLY			2

/* DHCP message type */
#define	DHCP_DISCOVER			1
#define DHCP_OFFER				2
#define	DHCP_REQUEST			3
#define	DHCP_DECLINE			4
#define	DHCP_ACK				5
#define DHCP_NAK				6
#define	DHCP_RELEASE			7
#define DHCP_INFORM				8

#define DHCP_HTYPE10MB			1
#define DHCP_HTYPE100MB			2

#define DHCP_HLENETHERNET		6
#define DHCP_HOPS				0
#define DHCP_SECS				0

#define DHCP_MAX_OPT			16
#define MAGIC_COOKIE			0x63825363



/* HTTP Method */
#define		METHOD_ERR		0		/**< Error Method. */
#define		METHOD_GET		1		/**< GET Method.   */
#define		METHOD_HEAD		2		/**< HEAD Method.  */
#define		METHOD_POST		3		/**< POST Method.  */

/* HTTP GET Method */
#define		PTYPE_ERR		0		/**< Error file. */
#define		PTYPE_HTML		1		/**< HTML file.  */
#define		PTYPE_GIF		2		/**< GIF file.   */
#define		PTYPE_TEXT		3		/**< TEXT file.  */
#define		PTYPE_JPEG		4		/**< JPEG file.  */
#define		PTYPE_FLASH		5		/**< FLASH file. */
#define		PTYPE_MPEG		6		/**< MPEG file.  */
#define		PTYPE_PDF		7		/**< PDF file.   */
#define		PTYPE_ZIP		8		/**< ZIP file.   */
#define 	PTYPE_CGI		9		/**< CGI         */

/* HTTP response */
#define		STATUS_OK			200
#define		STATUS_CREATED		201
#define		STATUS_ACCEPTED		202
#define		STATUS_NO_CONTENT	204
#define		STATUS_MV_PERM		301
#define		STATUS_MV_TEMP		302
#define		STATUS_NOT_MODIF	304
#define		STATUS_BAD_REQ		400
#define		STATUS_UNAUTH		401
#define		STATUS_FORBIDDEN	403
#define		STATUS_NOT_FOUND	404
#define		STATUS_INT_SERR		500
#define		STATUS_NOT_IMPL		501
#define		STATUS_BAD_GATEWAY	502
#define		STATUS_SERV_UNAVAIL	503

#define		MAX_INT_STR			80

/* HTML Doc. for ERROR */
#define ERROR_HTML_PAGE PSTR("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: 76\r\n\r\n<HTML>\r\n<BODY>\r\nDoh! The page you requested was not found.\r\n</BODY>\r\n</HTML>\r\n\0")
#define ERROR_REQUEST_PAGE PSTR("HTTP/1.1 400 OK\r\nContent-Type: text/html\r\nContent-Length: 50\r\n\r\n<HTML>\r\n<BODY>\r\nInvalid request.\r\n</BODY>\r\n</HTML>\r\n\0")

/* Response header for HTML*/
#define RES_HTMLHEAD_OK	PSTR("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: ")

/* Response head for TEXT */
#define RES_TEXTHEAD_OK	PSTR("HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nContent-Length: ")

/* Response head for GIF */
#define RES_GIFHEAD_OK PSTR("HTTP/1.1 200 OK\r\nContent-Type: image/gif\r\nContent-Length: ")

/* Response head for JPEG */
#define RES_JPEGHEAD_OK	PSTR("HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nContent-Length: ")

/* Response head for FLASH */
#define RES_FLASHHEAD_OK PSTR("HTTP/1.1 200 OK\r\nContent-Type: application/x-shockwave-flash\r\nContent-Length: ")

/* Response head for MPEG */
#define RES_MPEGHEAD_OK PSTR("HTTP/1.1 200 OK\r\nContent-Type: video/mpeg\r\nContent-Length: ")

/* Response head for PDF */
#define RES_PDFHEAD_OK PSTR("HTTP/1.1 200 OK\r\nContent-Type: application/pdf\r\nContent-Length: ")

/* Response head for ZIP */
#define RES_ZIPHEAD_OK PSTR("HTTP/1.1 200 OK\r\nContent-Type: application/zip\r\nContent-Length: ")

/////////////////////////// PING TYPEDEF ///////////////////////////

/**
 * @brief		for the PING message
 */
typedef struct _PING_MSG
{
	uint8_t		type;		/* Type = 8 Ping 8 */
	uint8_t		code;		/* Code = 0 */
	uint16_t	checksum;	/** Internet Checksum */
	uint16_t	id;			/**< Identification  */
	uint16_t	seqNum;		/**< Sequence Number */
	uint8_t		OPT[PING_OPT_LEN];
}PING_MSG;

typedef struct _PING_LOG
{
	uint16_t CheckSumErr;	/**< Check sum Error Count */
	uint16_t UnreachableMSG;	/**< Count of receiving unreachable message from a peer */
	uint16_t TimeExceedMSG;	/**< Count of receiving time exceeded message from a peer */
	uint16_t UnknownMSG;	/**< Count of receiving unknown message from a peer */
	uint16_t ARPErr;		/**< count of fail to send ARP to the specified peer */
	uint16_t PingRequest;	/**< Count of sending ping-request message to the specified peer */
	uint16_t PingReply;	/**< Count of receiving ping reply message from the specifed peer */
	uint16_t Loss;		/**< Count of timeout  */
}PING_LOG;


/////////////////////////// DHCP TYPEDEF ///////////////////////////

/**
 * @brief	DHCP option and value (cf. RFC-1533)
 */
enum
{
	padOption			=	0,
	subnetMask			=	1,
	timerOffset			=	2,
	routersOnSubnet		=	3,
	timeServer			=	4,
	nameServer			=	5,
	dns					=	6,
	logServer			=	7,
	cookieServer		=	8,
	lprServer			=	9,
	impressServer		=	10,
	resourceLocationServer	=	11,
	hostName			=	12,
	bootFileSize		=	13,
	meritDumpFile		=	14,
	domainName			=	15,
	swapServer			=	16,
	rootPath			=	17,
	extentionsPath		=	18,
	IPforwarding		=	19,
	nonLocalSourceRouting	=	20,
	policyFilter		=	21,
	maxDgramReasmSize	=	22,
	defaultIPTTL		=	23,
	pathMTUagingTimeout	=	24,
	pathMTUplateauTable	=	25,
	ifMTU				=	26,
	allSubnetsLocal		=	27,
	broadcastAddr		=	28,
	performMaskDiscovery	=	29,
	maskSupplier		=	30,
	performRouterDiscovery	=	31,
	routerSolicitationAddr	=	32,
	staticRoute			=	33,
	trailerEncapsulation	=	34,
	arpCacheTimeout		=	35,
	ethernetEncapsulation	=	36,
	tcpDefaultTTL		=	37,
	tcpKeepaliveInterval	=	38,
	tcpKeepaliveGarbage	=	39,
	nisDomainName		=	40,
	nisServers			=	41,
	ntpServers			=	42,
	vendorSpecificInfo	=	43,
	netBIOSnameServer	=	44,
	netBIOSdgramDistServer	=	45,
	netBIOSnodeType		=	46,
	netBIOSscope		=	47,
	xFontServer			=	48,
	xDisplayManager		=	49,
	dhcpRequestedIPaddr	=	50,
	dhcpIPaddrLeaseTime	=	51,
	dhcpOptionOverload	=	52,
	dhcpMessageType		=	53,
	dhcpServerIdentifier	=	54,
	dhcpParamRequest	=	55,
	dhcpMsg				=	56,
	dhcpMaxMsgSize		=	57,
	dhcpT1value			=	58,
	dhcpT2value			=	59,
	dhcpClassIdentifier	=	60,
	dhcpClientIdentifier	=	61,
	endOption			=	255
};

/**
 * @brief		for the DHCP message
 */
typedef struct _RIP_MSG
{
	uint8_t	op;
	uint8_t	htype;
	uint8_t	hlen;
	uint8_t	hops;
	uint32_t	xid;
	uint16_t	secs;
	uint16_t	flags;
	uint8_t	ciaddr[4];
	uint8_t	yiaddr[4];
	uint8_t	siaddr[4];
	uint8_t	giaddr[4];
	uint8_t	chaddr[16];
	uint8_t	sname[64];
	uint8_t	file[128];
	uint8_t	OPT[312];
}RIP_MSG;


/////////////////////////// HTTP TYPEDEF ///////////////////////////

/**
 @brief 	Structure of HTTP REQUEST
 */
typedef struct _HTTP_REQUEST
{
	uint8_t	METHOD;						/**< request method(METHOD_GET...). */
	uint8_t	TYPE;						/**< request type(PTYPE_HTML...).   */
	uint8_t	URI[MAX_URI_SIZE];			/**< request file name.             */
} HTTP_REQUEST;


/////////////////////////// PING FUNCTIONS ///////////////////////////

uint8_t	ping(uint8_t count, uint16_t time, uint8_t* addr, PING_LOG* log); /* Send ping-request to the specified peer and receive ping-reply from the specified peer. */
void 	DisplayPingStatistics(PING_LOG log);/* Display result of ping */


/////////////////////////// DHCP FUNCTIONS ///////////////////////////

void  	init_dhcp_client(SOCKET s, void(*ip_update)(void), void(*ip_conflict)(void));	// Initialise the DHCP client
uint8_t	getIP_DHCPS(void);								// Get the network configuration from the DHCP server
SOCKET	get_DHCP_socket(void);							// Get the socket assigned for DHCP
void  	check_DHCP_state(SOCKET s);						// Check the DHCP state


/////////////////////////// HTTP FUNCTIONS ///////////////////////////

uint8_t init_httpd_ch(SOCKET s);			// Initialise the socket & buffers for HTTP server
SOCKET	get_HTTP_socket(void);				// Get the socket assigned for HTTPD

/* http service */
void proc_http(SOCKET s, uint8_t *buf, uint16_t len);		// processing HTTP

void unescape_http_url(uint8_t *url);								/* convert escape character to ascii */
void parse_http_request(HTTP_REQUEST *request, uint8_t *buf);		/* parse request from peer */
void find_http_uri_type(uint8_t *type, uint8_t *buf);				/* find MIME type of a file */
void make_http_response_head(uint8_t *buf, uint8_t type, uint32_t len);	/* make response header */
uint8_t* get_http_param_value(uint8_t *uri, uint8_t *param_name);	/* get the user-specific parameter value */
uint8_t* get_http_uri_name(uint8_t *uri);


#ifdef __cplusplus
}
#endif


#endif	/* _INET_H_ */
