#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "w5500.h"
#include "socket.h"
#include "md5.h"

#ifdef __cplusplus
extern "C" {
#endif


//#define __DEF_PPP_DBG__ // debug message for [Phase] and [Network Init]
//#define __DEF_PPP_DBG1__ // debug message for checking 'Txbuf overflow' and etc.


#define		PPP_OPTION_BUF_LEN 64

//PPPoE retry count value
#define 	PPP_MAX_RETRYSEND_COUNT		5
#define 	PPP_MAX_RETRYRECV_COUNT		32
#define 	PPP_MAX_RETRY_COUNT			5
// PPPoE EtherType definition
#define 	PPPoE_DISCOVERY		0x8863
#define 	PPPoE_SESSION		0x8864

// PPPoE Frame field definition
#define		PPPoE_VER_TYPE		0x11

// PPPoE Code definition
#define 	PPPoE_PADI			0x09
#define 	PPPoE_PADO			0x07
#define 	PPPoE_PADR			0x19
#define 	PPPoE_PADS			0x65
#define 	PPPoE_PADT			0xa7
#define		PPPoE_SESSIONDATA	0x00

// PPPoE Discovery Tag type definition
#define 	PPPoED_END_OF_LIST			0x0000
#define 	PPPoED_SERVICE_NAME			0x0101
#define 	PPPoED_AC_NAME				0x0102
#define 	PPPoED_HOST_UNIQ			0x0103
#define 	PPPoED_AC_COOKIE			0x0104
#define 	PPPoED_VENDER_SPECIFIC		0x0105

// PPPoE Protocol definition
#define		PPPoE_LCP			0xC021
#define		PPPoE_PAP			0xC023
#define		PPPoE_CHAP			0xC223
#define		PPPoE_IPCP			0x8021

// PPPoE Protocol Code definition
	// LCP using 0x01 ~ 0x0b
	// PAP using 0x01 ~ 0x03
	// IPCP using 0x01 ~ 0x07
#define		PPP_CONFIG_REQ		0x01
#define		PPP_CONFIG_ACK		0x02
#define		PPP_CONFIG_NAK		0x03
#define		PPP_CONFIG_REJ		0x04
#define		PPP_TERM_REQ		0x05
#define		PPP_TERM_ACK		0x06
#define		PPP_CODE_REJ		0x07
#define		PPP_PROT_REJ		0x08
#define		PPP_ECHO_REQ		0x09
#define		PPP_ECHO_REP		0x0a
#define		PPP_DIS_REQ			0x0b

// PPPoE LCP Type definition
#define		LCP_MRU				0x01
#define		LCP_AUTH			0x03
#define		LCP_MAGICNUM		0x05
#define		LCP_PROTOCOMP		0x07
#define		LCP_ADDRCOMP		0x08

// PPPoE CHAP Algorithm
#define		MD5					0x05
#define		MS_CHAP				0x80
#define		MS_CHAP_V2			0x81

// PPPoE stage control flags
#define   FLAG_DISCOVERY_RCV_PADO 	0X0001
#define   FLAG_DISCOVERY_RCV_PADS	0X0002
#define   FLAG_LCP_CR_RCV			0X0004
#define   FLAG_LCP_CR_SNT			0X0008
#define   FLAG_PAP_ACK_RCV			0X0010
#define   FLAG_CHAP_SUC_RCV			0X0020
#define   FLAG_IPCP_CR_SNT			0X0040
#define   FLAG_IPCP_CR_RCV			0X0080
#define   FLAG_IPCP_NAK_RCV			0X0100
#define   FLAG_TERMINATION_ACK_RCV	0X0200
#define   FLAG_PADO_SERVICENAME		0X0400

/*
// PPPoE Field value definition
// -> not used.
#define		PPPoE_SESSION_ID	0x0000
#define		LCP_MAGICNUM_VAL	0x00112299
*/

// Logical variable definition
#define		PPP_SUCCESS			1
#define		PPP_FAIL			0
#define 	PPP_RETRY			2

#define     OPTMSG_LEN			80
#define		CV_HV_LEN			16


#define		PPP_FAILED			        0

#define PPP_FRAME_SIZE 		128
#define PPP_RXFRAME_SIZE 	1514
// PPPoE message
typedef struct _PPPMSG
{
	uint8_t dst_mac[6];
	uint8_t src_mac[6];
	uint16_t ether_type; 			// 0x8863 : PPPoE Discovery, 0x8864 : PPPoE Session
	uint8_t version_type; 		// 4-bit 'version' = 0001, 4-bit 'type' = 0001 default
	uint8_t frame_code;
	uint16_t session_id;
	uint16_t len;
}PPPMSG;

// PPPoE Protocol
typedef struct _PROTOCOL
{
	uint16_t protocol;
	uint8_t pcode;
	uint8_t id;
	uint16_t len;
	uint8_t opt[OPTMSG_LEN];
}PROTOCOL;

// PPPoE Start function
uint8_t ppp_start(uint8_t * pppoe_buf);
//PPPoE termination function
uint8_t do_lcp_terminate(void);

#ifdef __cplusplus
}
#endif
