/*
@file		w5200.h

 * ----------	-------		------------------------------------------------
 * 03/13/2012	1.6.1		Added clearSUBR(), applySUBR() and modified setSUBR() functions
 *							      because of the ARP errata.
 *							Keep SUBR 0.0.0.0 unless using TCP connect() or UDP sendto()
 *							Use the SUBN_VAR variable to read the real subnet.
 * ----------	-------		------------------------------------------------
*/
#ifndef	_W5200_H_
#define	_W5200_H_

#include "wizchip_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

#if   (_WIZCHIP_ == 5200)		// Definition in freeRTOSBoardDefs.h

/**
 @brief chip base register   define memory map for W5200
*/
#define COMMON_BASE					0x0000

/**
 @brief socket base register
*/
#define CH_BASE						(COMMON_BASE + 0x4000)
/**
 @brief	size of each channel register map
 */
#define CH_SIZE						0x0100


#define __DEF_WIZCHIP_MAP_TXBUF__ (COMMON_BASE + 0x8000) /* Internal Tx buffer address of the W5200 */
#define __DEF_WIZCHIP_MAP_RXBUF__ (COMMON_BASE + 0xC000) /* Internal Rx buffer address of the W5200 */

#ifdef __DEF_WIZCHIP_INT__
  // W5100 use external interrupt 4
  #define WIZCHIP_ISR_DISABLE()	(EIMSK &= ~(0x10))
  #define WIZCHIP_ISR_ENABLE()	(EIMSK |= 0x10)
  #define WIZCHIP_ISR_GET(X)		(X = EIMSK)
  #define WIZCHIP_ISR_SET(X)		(EIMSK = X)
#else // leave the interrupts empty.
  #define WIZCHIP_ISR_DISABLE()
  #define WIZCHIP_ISR_ENABLE()
#endif

//////////////////////////////////////////////////////////////


#define MR				(COMMON_BASE + 0x0000)
/**
 @brief Gateway IP Register address
 */
#define GAR0			(COMMON_BASE + 0x0001)
#define GAR1			(COMMON_BASE + 0x0002)
#define GAR2			(COMMON_BASE + 0x0003)
#define GAR3			(COMMON_BASE + 0x0004)
/**
 @brief Subnet mask Register address
 */
#define SUBR0			(COMMON_BASE + 0x0005)
#define SUBR1			(COMMON_BASE + 0x0006)
#define SUBR2			(COMMON_BASE + 0x0007)
#define SUBR3			(COMMON_BASE + 0x0008)
/**
 @brief Source MAC Register address
 */
#define SHAR0			(COMMON_BASE + 0x0009)
#define SHAR1			(COMMON_BASE + 0x000A)
#define SHAR2			(COMMON_BASE + 0x000B)
#define SHAR3			(COMMON_BASE + 0x000C)
#define SHAR4			(COMMON_BASE + 0x000D)
#define SHAR5			(COMMON_BASE + 0x000E)
/**
 @brief Source IP Register address
 */
#define SIPR0			(COMMON_BASE + 0x000F)
#define SIPR1			(COMMON_BASE + 0x0010)
#define SIPR2			(COMMON_BASE + 0x0011)
#define SIPR3			(COMMON_BASE + 0x0012)
/**
 @brief Interrupt Register
 */
#define IR				(COMMON_BASE + 0x0015)
/**
 @brief Socket Interrupt Mask Register
 */
#define IMR				(COMMON_BASE + 0x0016)
/**
 @brief Timeout register address( 1 is 100us )
 */
#define RTR0			(COMMON_BASE + 0x0017)
#define RTR1			(COMMON_BASE + 0x0018)
/**
 @brief Retry count register
 */
#define RCR				(COMMON_BASE + 0x0019)
/**
 @brief Authentication type register address in PPPoE mode
 */
#define PATR0			(COMMON_BASE + 0x001C)
#define PATR1			(COMMON_BASE + 0x001D)
#define PPPALGO			(COMMON_BASE + 0x001E)

/**
 @brief chip version register address
 */
#define VERSIONR		(COMMON_BASE + 0x001F)

/**
 @briefPPP LCP Request Timer register  in PPPoE mode
 */
#define PTIMER			(COMMON_BASE + 0x0028)
/**
 @brief PPP LCP Magic number register  in PPPoE mode
 */
#define PMAGIC			(COMMON_BASE + 0x0029)

/**
 @brief Unreachable IP register address in UDP mode (reserved)
 */
//#define UIPR0			(COMMON_BASE + 0x002A)
/**
 @brief Unreachable Port register address in UDP mode (reserved)
 */
//#define UPORT0			(COMMON_BASE + 0x002E)
//#define UPORT1			(COMMON_BASE + 0x002F)
/**
 @brief set Interrupt low level timer register address
 */
#define INTLEVEL0		(COMMON_BASE + 0x0030)
#define INTLEVEL1		(COMMON_BASE + 0x0031)
/**
 @brief Socket Interrupt Register 2
 */
#define IR2				(COMMON_BASE + 0x0034)
/**
 @brief PHY Status Register
 */
#define PSTATUS			(COMMON_BASE + 0x0035)
/**
 @brief Interrupt mask register 2
 */
#define IMR2			(COMMON_BASE + 0x0036)


/**
 @brief socket Mode register
 */
#define Sn_MR(ch)			(CH_BASE + ch * CH_SIZE + 0x0000)
/**
 @brief channel Sn_CR register
 */
#define Sn_CR(ch)			(CH_BASE + ch * CH_SIZE + 0x0001)
/**
 @brief channel interrupt register
 */
#define Sn_IR(ch)			(CH_BASE + ch * CH_SIZE + 0x0002)
/**
 @brief channel status register
 */
#define Sn_SR(ch)			(CH_BASE + ch * CH_SIZE + 0x0003)
/**
 @brief source port register
 */
#define Sn_PORT0(ch)		(CH_BASE + ch * CH_SIZE + 0x0004)
#define Sn_PORT1(ch)		(CH_BASE + ch * CH_SIZE + 0x0005)
/**
 @brief Peer MAC register address
 */
#define Sn_DHAR0(ch)		(CH_BASE + ch * CH_SIZE + 0x0006)
#define Sn_DHAR1(ch)		(CH_BASE + ch * CH_SIZE + 0x0007)
#define Sn_DHAR2(ch)		(CH_BASE + ch * CH_SIZE + 0x0008)
#define Sn_DHAR3(ch)		(CH_BASE + ch * CH_SIZE + 0x0009)
#define Sn_DHAR4(ch)		(CH_BASE + ch * CH_SIZE + 0x000A)
#define Sn_DHAR5(ch)		(CH_BASE + ch * CH_SIZE + 0x000B)


/**
 @brief Peer IP register address
 */
#define Sn_DIPR0(ch)		(CH_BASE + ch * CH_SIZE + 0x000C)
#define Sn_DIPR1(ch)		(CH_BASE + ch * CH_SIZE + 0x000D)
#define Sn_DIPR2(ch)		(CH_BASE + ch * CH_SIZE + 0x000E)
#define Sn_DIPR3(ch)		(CH_BASE + ch * CH_SIZE + 0x000F)

/**
 @brief Peer port register address
 */
#define Sn_DPORT0(ch)		(CH_BASE + ch * CH_SIZE + 0x0010)
#define Sn_DPORT1(ch)		(CH_BASE + ch * CH_SIZE + 0x0011)
/**
 @brief Maximum Segment Size(Sn_MSSR0) register address
 */
#define Sn_MSSR0(ch)		(CH_BASE + ch * CH_SIZE + 0x0012)
#define Sn_MSSR1(ch)		(CH_BASE + ch * CH_SIZE + 0x0013)
/**
 @brief Protocol of IP Header field register in IP raw mode
 */
#define Sn_PROTO(ch)		(CH_BASE + ch * CH_SIZE + 0x0014)

/**
 @brief IP Type of Service(TOS) Register
 */
#define Sn_TOS(ch)			(CH_BASE + ch * CH_SIZE + 0x0015)
/**
 @brief IP Time to live(TTL) Register
 */
#define Sn_TTL(ch)			(CH_BASE + ch * CH_SIZE + 0x0016)
/**
 @brief Receive memory size register
 */
#define Sn_RXMEM_SIZE(ch)	(CH_BASE + ch * CH_SIZE + 0x001E)
/**
 @brief Transmit memory size register
 */
#define Sn_TXMEM_SIZE(ch)	(CH_BASE + ch * CH_SIZE + 0x001F)
/**
 @brief Transmit free memory size register
 */
#define Sn_TX_FSR0(ch)		(CH_BASE + ch * CH_SIZE + 0x0020)
#define Sn_TX_FSR1(ch)		(CH_BASE + ch * CH_SIZE + 0x0021)
/**
 @brief Transmit memory read pointer register address
 */
#define Sn_TX_RD0(ch)		(CH_BASE + ch * CH_SIZE + 0x0022)
#define Sn_TX_RD1(ch)		(CH_BASE + ch * CH_SIZE + 0x0023)
/**
 @brief Transmit memory write pointer register address
 */
#define Sn_TX_WR0(ch)		(CH_BASE + ch * CH_SIZE + 0x0024)
#define Sn_TX_WR1(ch)		(CH_BASE + ch * CH_SIZE + 0x0025)
/**
 @brief Received data size register
 */
#define Sn_RX_RSR0(ch)		(CH_BASE + ch * CH_SIZE + 0x0026)
#define Sn_RX_RSR1(ch)		(CH_BASE + ch * CH_SIZE + 0x0027)
/**
 @brief Read point of Receive memory
 */
#define Sn_RX_RD0(ch)		(CH_BASE + ch * CH_SIZE + 0x0028)
#define Sn_RX_RD1(ch)		(CH_BASE + ch * CH_SIZE + 0x0029)
/**
 @brief Write point of Receive memory
 */
#define Sn_RX_WR0(ch)		(CH_BASE + ch * CH_SIZE + 0x002A)
#define Sn_RX_WR1(ch)		(CH_BASE + ch * CH_SIZE + 0x002B)
/**
 @brief socket interrupt mask register
 */
#define Sn_IMR(ch)			(CH_BASE + ch * CH_SIZE + 0x002C)
/**
 @brief frag field value in IP header register
 */
#define Sn_FRAG0(ch)		(CH_BASE + ch * CH_SIZE + 0x002D)
#define Sn_FRAG1(ch)		(CH_BASE + ch * CH_SIZE + 0x002E)
/**
 @brief Keep Timer register (reserved)
 */
#define Sn_KEEP_TIMER(ch)	(CH_BASE + ch * CH_SIZE + 0x002F)

/* MODE register values */
#define MR_RST			0x80		/**< reset */
#define MR_WOL			0x20		/**< Wake on Lan */
#define MR_PB			0x10		/**< ping block */
#define MR_PPPOE		0x08		/**< enable pppoe */
#define MR_LB  			0x04		/**< little or big endian selector in indirect mode */
#define MR_AI			0x02		/**< auto-increment in indirect mode */
#define MR_IND			0x01		/**< enable indirect mode */

/* IR register values */
#define IR_CONFLICT	    0x80		/**< check ip conflict */
//#define IR_UNREACH	0x40		/**< get the destination unreachable message in UDP sending (reserved) */
#define IR_PPPoE		0x20		/**< get the PPPoE close message */
//#define IR_MAGIC		0x10		/**< get the magic packet interrupt (reserved) */
//#define IR_SOCK(ch)	(0x01 << ch) 	/**< check socket interrupt (reserved) */


/* PSTATUS register values */
#define PHYSTATUS_LINK			0x20
#define PHYSTATUS_POWERSAVE		0x10
#define PHYSTATUS_POWERDOWN		0x08

/* Sn_MR values */
#define Sn_MR_CLOSE		0x00		/**< unused socket */
#define Sn_MR_TCP		0x01		/**< TCP */
#define Sn_MR_UDP		0x02		/**< UDP */
#define Sn_MR_IPRAW	    0x03		/**< IP LAYER RAW SOCK */
#define Sn_MR_MACRAW	0x04		/**< MAC LAYER RAW SOCK */
#define Sn_MR_PPPOE		0x05		/**< PPPoE */
#define Sn_MR_ND		0x20		/**< No Delayed Ack(TCP) flag */
#define Sn_MR_MULTI		0x80		/**< support multi-casting flag */

/* Sn_CR values */
#define Sn_CR_OPEN		0x01		/**< Initialise or open socket */
#define Sn_CR_LISTEN	0x02		/**< wait connection request in tcp mode(Server mode) */
#define Sn_CR_CONNECT	0x04		/**< send connection request in tcp mode(Client mode) */
#define Sn_CR_DISCON	0x08		/**< send closing request in tcp mode */
#define Sn_CR_CLOSE		0x10		/**< close socket */
#define Sn_CR_SEND		0x20		/**< update txbuf pointer, send data */
#define Sn_CR_SEND_MAC	0x21		/**< send data with MAC address, so without ARP process */
#define Sn_CR_SEND_KEEP	0x22		/**< send keep alive message */
#define Sn_CR_RECV		0x40		/**< update rxbuf pointer, recv data */

#ifdef __DEF_WIZCHIP_PPP__
	#define Sn_CR_PCON		0x23
	#define Sn_CR_PDISCON	0x24
	#define Sn_CR_PCR		0x25
	#define Sn_CR_PCN		0x26
	#define Sn_CR_PCJ		0x27
#endif

/* Sn_IR values */
#ifdef __DEF_WIZCHIP_PPP__
	#define Sn_IR_PRECV		0x80
	#define Sn_IR_PFAIL		0x40
	#define Sn_IR_PNEXT		0x20
#endif

#define Sn_IR_SEND_OK		0x10		/**< complete sending */
#define Sn_IR_TIMEOUT		0x08		/**< assert timeout */
#define Sn_IR_RECV			0x04		/**< receiving data */
#define Sn_IR_DISCON		0x02		/**< closed socket */
#define Sn_IR_CON			0x01		/**< established connection */

/* Sn_SR values */
#define SOCK_CLOSED			0x00		/**< closed */
#define SOCK_ARP			0x01		/**< arp protocol initiated */
#define SOCK_INIT 			0x13		/**< init state */
#define SOCK_LISTEN			0x14		/**< listen state */
#define SOCK_SYNSENT	   	0x15		/**< connection state */
#define SOCK_SYNRECV		0x16		/**< connection state */
#define SOCK_ESTABLISHED	0x17		/**< success to connect */
#define SOCK_FIN_WAIT		0x18		/**< closing state */
#define SOCK_CLOSING		0x1A		/**< closing state */
#define SOCK_TIME_WAIT		0x1B		/**< closing state */
#define SOCK_CLOSE_WAIT		0x1C		/**< closing state */
#define SOCK_LAST_ACK		0x1D		/**< closing state */
#define SOCK_UDP			0x22		/**< udp socket */
#define SOCK_IPRAW			0x32		/**< ip raw mode socket */
#define SOCK_MACRAW			0x42		/**< mac raw mode socket */
#define SOCK_PPPOE			0x5F		/**< pppoe socket */


/*********************************************************
* W5200 access function
*********************************************************/

void	WIZCHIP_init(void); // reset W5200 - First call to make
void	WIZCHIP_sysinit(uint8_t * tx_size, uint8_t * rx_size); // setting tx/rx buf size
uint8_t	WIZCHIP_getISR(uint8_t s);
void	WIZCHIP_putISR(uint8_t s, uint8_t val);

uint8_t	 WIZCHIP_read(uint16_t addr) __attribute__ ((hot));
uint8_t  WIZCHIP_write(uint16_t addr, uint8_t data) __attribute__ ((hot));

uint16_t WIZCHIP_getRxMAX(uint8_t s);
uint16_t WIZCHIP_getTxMAX(uint8_t s);
uint16_t WIZCHIP_getRxMASK(uint8_t s);
uint16_t WIZCHIP_getTxMASK(uint8_t s);
uint16_t WIZCHIP_getRxBASE(uint8_t s);
uint16_t WIZCHIP_getTxBASE(uint8_t s);


void	setRTR(uint16_t timeout); // set retry duration for data transmission, connection, closing ...
void	setRCR(uint8_t retry); // set retry count (above the value, assert timeout interrupt)
void	setIMR(uint8_t mask); // set interrupt mask.
uint8_t	getIR( void );
void	setSn_MSS(SOCKET s, uint16_t Sn_MSSR); // set maximum segment size
void	setSn_PROTO(SOCKET s, uint8_t proto); // set IP Protocol value using IP-Raw mode
uint8_t  getSn_IR(SOCKET s); // get socket interrupt status
uint8_t  getSn_SR(SOCKET s); // get socket status
uint16_t getSn_TX_FSR(SOCKET s); // get socket TX free buf size
uint16_t getSn_RX_RSR(SOCKET s); // get socket RX recv buf size

void	setSn_TTL(SOCKET s, uint8_t ttl);
void	setMR(uint8_t val);
uint8_t getMR(void);

void	setGAR(uint8_t * addr); // set gateway address
void	setSHAR(uint8_t * addr); // set local MAC address
void	setSIPR(uint8_t * addr); // set local IP address


// the ARP errata fix
void	saveSUBR(un_l2cval * addr);	// save subnet mask address off chip
void	setSUBR(void);				// set subnet mask address on chip
void	clearSUBR(void);			// clear subnet mask address on chip
void	getSUBR(un_l2cval * addr);	// get subnet mask address off chip

void	getGAR(uint8_t * addr);
void	getSHAR(uint8_t * addr);
void	getSIPR(uint8_t * addr);

#ifdef __DEF_WIZCHIP_PPP__
uint8_t pppinit(uint8_t *id, uint8_t idlen, uint8_t *passwd, uint8_t passwdlen);
uint8_t pppterm(uint8_t *mac,uint8_t *sessionid);
#endif

uint16_t WIZCHIP_write_buf(uint16_t addr, uint8_t *buf, uint16_t len) __attribute__ ((hot));
uint16_t WIZCHIP_read_buf( uint16_t addr, uint8_t *buf, uint16_t len) __attribute__ ((hot));

void	WIZCHIP_send_data_processing(SOCKET s, uint8_t *wizdata, uint16_t len);
void	WIZCHIP_recv_data_processing(SOCKET s, uint8_t *wizdata, uint16_t len);

void	WIZCHIP_write_data(SOCKET s, uint8_t * src, uint8_t * dst, uint16_t len);
void	WIZCHIP_read_data(SOCKET s, uint8_t * src, uint8_t * dst, uint16_t len);

#endif // #if (_WIZCHIP_ == 5200)

#ifdef __cplusplus
}
#endif

#endif
