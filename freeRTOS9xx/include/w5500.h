/*
@file    w5500.h
*/
#ifndef  _W5500_H_
#define  _W5500_H_

#include "wizchip_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

#if   (_WIZCHIP_ == 5500)		// Definition in freeRTOSBoardDefs.h

/**
 @brief Mode Register address
 * W5500 SPI Frame consists of 16bits Offset Address in Address Phase,
 * 8bits Control Phase and N bytes Data Phase.
 * 0                8                16               24                   ~
 * |----------------|----------------|----------------|----------------------
 * |        16bit offset Address     | Control Bits   |  Data Phase
 *
 * The 8bits Control Phase is reconfigured with Block Select bits (BSB[4:0]),
 * Read/Write Access Mode bit (RWB) and SPI Operation Mode (OM[1:0]).
 * Block Select bits select a block as like common register, socket register, tx buffer and tx buffer.
 * Address value is defined as 16bit offset Address, BSB[4:0] and the three bits of zero-padding.(The RWB and OM [1:0] are '0 'padding)
 * Please, refer to W5500 data sheet for more detail about Memory Map.
 *
 */


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

/**
 @brief Mode Register address
 */
#define MR                          (0x000000)

/**
 @brief Gateway IP Register address
 */
#define GAR0                        (0x000100)
#define GAR1                        (0x000200)
#define GAR2                        (0x000300)
#define GAR3                        (0x000400)
/**
 @brief Subnet mask Register address
 */
#define SUBR0                       (0x000500)
#define SUBR1                       (0x000600)
#define SUBR2                       (0x000700)
#define SUBR3                       (0x000800)

/**
 @brief Source MAC Register address
 */
#define SHAR0                       (0x000900)
#define SHAR1                       (0x000A00)
#define SHAR2                       (0x000B00)
#define SHAR3                       (0x000C00)
#define SHAR4                       (0x000D00)
#define SHAR5                       (0x000E00)
/**
 @brief Source IP Register address
 */
#define SIPR0                       (0x000F00)
#define SIPR1                       (0x001000)
#define SIPR2                       (0x001100)
#define SIPR3                       (0x001200)
/**
 @brief set Interrupt low level timer register address
 */
#define INTLEVEL0                   (0x001300)
#define INTLEVEL1                   (0x001400)
/**
 @brief Interrupt Register
 */
#define IR                          (0x001500)
/**
 @brief Interrupt mask register
 */
#define IMR                         (0x001600)
/**
 @brief Socket Interrupt Register
 */
#define SIR                         (0x001700)
/**
 @brief Socket Interrupt Mask Register
 */
#define SIMR                        (0x001800)
/**
 @brief Timeout register address( 1 is 100us )
 */
#define RTR0                        (0x001900)
#define RTR1                        (0x001A00)
/**
 @brief Retry count register
 */
#define RCR                         (0x001B00)
/**
 @briefPPP LCP Request Timer register  in PPPoE mode
 */
#define PTIMER                      (0x001C00)
/**
 @brief PPP LCP Magic number register  in PPPoE mode
 */
#define PMAGIC                      (0x001D00)
/**
 @brief PPP Destination MAC Register address
 */
#define PDHAR0                      (0x001E00)
#define PDHAR1                      (0x001F00)
#define PDHAR2                      (0x002000)
#define PDHAR3                      (0x002100)
#define PDHAR4                      (0x002200)
#define PDHAR5                      (0x002300)
/**
 @brief PPP Session Identification Register
 */
#define PSID0                       (0x002400)
#define PSID1                       (0x002500)
/**
 @brief PPP Maximum Segment Size(MSS) register
 */
#define PMR0                        (0x002600)
#define PMR1                        (0x002700)
/**
 @brief Unreachable IP register address in UDP mode
 */
#define UIPR0                       (0x002800)
#define UIPR1                       (0x002900)
#define UIPR2                       (0x002A00)
#define UIPR3                       (0x002B00)
/**
 @brief Unreachable Port register address in UDP mode
 */
#define UPORT0                      (0x002C00)
#define UPORT1                      (0x002D00)
/**
 @brief PHY Configuration Register
 */
#define PHYCFGR                     (0x002E00)
/**
 @brief chip version register address
 */
#define VERSIONR                    (0x003900)



/**
 @brief socket Mode register
 */
#define Sn_MR(ch)                       (0x000008 + (ch<<5))

/**
 @brief channel Sn_CR register
 */
#define Sn_CR(ch)                       (0x000108 + (ch<<5))
/**
 @brief channel interrupt register
 */
#define Sn_IR(ch)                       (0x000208 + (ch<<5))
/**
 @brief channel status register
 */
#define Sn_SR(ch)                       (0x000308 + (ch<<5))
/**
 @brief source port register
 */
#define Sn_PORT0(ch)                    (0x000408 + (ch<<5))
#define Sn_PORT1(ch)                    (0x000508 + (ch<<5))
/**
 @brief Peer MAC register address
 */
#define Sn_DHAR0(ch)                    (0x000608 + (ch<<5))
#define Sn_DHAR1(ch)                    (0x000708 + (ch<<5))
#define Sn_DHAR2(ch)                    (0x000808 + (ch<<5))
#define Sn_DHAR3(ch)                    (0x000908 + (ch<<5))
#define Sn_DHAR4(ch)                    (0x000A08 + (ch<<5))
#define Sn_DHAR5(ch)                    (0x000B08 + (ch<<5))
/**
 @brief Peer IP register address
 */
#define Sn_DIPR0(ch)                    (0x000C08 + (ch<<5))
#define Sn_DIPR1(ch)                    (0x000D08 + (ch<<5))
#define Sn_DIPR2(ch)                    (0x000E08 + (ch<<5))
#define Sn_DIPR3(ch)                    (0x000F08 + (ch<<5))
/**
 @brief Peer port register address
 */
#define Sn_DPORT0(ch)                   (0x001008 + (ch<<5))
#define Sn_DPORT1(ch)                   (0x001108 + (ch<<5))
/**
 @brief Maximum Segment Size(Sn_MSSR0) register address
 */
#define Sn_MSSR0(ch)                    (0x001208 + (ch<<5))
#define Sn_MSSR1(ch)                    (0x001308 + (ch<<5))
/**
 @brief Protocol of IP Header field register in IP raw mode
 */
#define Sn_PROTO(ch)                    (0x001408 + (ch<<5))
/**
 @brief IP Type of Service(TOS) register
 */
#define Sn_TOS(ch)                      (0x001508 + (ch<<5))
/**
 @brief IP Time to live(TTL) register
 */
#define Sn_TTL(ch)                      (0x001608 + (ch<<5))
/**
 @brief Receive memory size register
 */
#define Sn_RXMEM_SIZE(ch)               (0x001E08 + (ch<<5))
/**
 @brief Transmit memory size register
 */
#define Sn_TXMEM_SIZE(ch)               (0x001F08 + (ch<<5))
/**
 @brief Transmit free memory size register
 */
#define Sn_TX_FSR0(ch)                  (0x002008 + (ch<<5))
#define Sn_TX_FSR1(ch)                  (0x002108 + (ch<<5))
/**
 @brief Transmit memory read pointer register address
 */
#define Sn_TX_RD0(ch)                   (0x002208 + (ch<<5))
#define Sn_TX_RD1(ch)                   (0x002308 + (ch<<5))
/**
 @brief Transmit memory write pointer register address
 */
#define Sn_TX_WR0(ch)                   (0x002408 + (ch<<5))
#define Sn_TX_WR1(ch)                   (0x002508 + (ch<<5))
/**
 @brief Received data size register
 */
#define Sn_RX_RSR0(ch)                  (0x002608 + (ch<<5))
#define Sn_RX_RSR1(ch)                  (0x002708 + (ch<<5))
/**
 @brief Read point of Receive memory
 */
#define Sn_RX_RD0(ch)                   (0x002808 + (ch<<5))
#define Sn_RX_RD1(ch)                   (0x002908 + (ch<<5))
/**
 @brief Write point of Receive memory
 */
#define Sn_RX_WR0(ch)                   (0x002A08 + (ch<<5))
#define Sn_RX_WR1(ch)                   (0x002B08 + (ch<<5))
/**
 @brief socket interrupt mask register
 */
#define Sn_IMR(ch)                      (0x002C08 + (ch<<5))
/**
 @brief frag field value in IP header register
 */
#define Sn_FRAG0(ch)                    (0x002D08 + (ch<<5))
#define Sn_FRAG1(ch)                    (0x002E08 + (ch<<5))
/**
 @brief Keep Timer register
 */
#define Sn_KPALVTR(ch)                  (0x002F08 + (ch<<5))

/* MODE register values */
#define MR_RST                       0x80 /**< reset */
#define MR_WOL                       0x20 /**< Wake on Lan */
#define MR_PB                        0x10 /**< ping block */
#define MR_PPPOE                     0x08 /**< enable pppoe */
#define MR_UDP_FARP                  0x02 /**< enable FORCE ARP */


/* IR register values */
#define IR_CONFLICT                  0x80 /**< check ip conflict */
#define IR_UNREACH                   0x40 /**< get the destination unreachable message in UDP sending */
#define IR_PPPoE                     0x20 /**< get the PPPoE close message */
#define IR_MAGIC                     0x10 /**< get the magic packet interrupt */


/* PHYCFGR register value */
#define PHYCFGR_RST                 ~(1<<7)  //< For PHY reset, must operate AND mask.
#define PHYCFGR_OPMD                 (1<<6)   // Configure PHY with OPMDC value
#define PHYCFGR_OPMDC_ALLA           (7<<3)
#define PHYCFGR_OPMDC_PDOWN          (6<<3)
#define PHYCFGR_OPMDC_NA             (5<<3)
#define PHYCFGR_OPMDC_100FA          (4<<3)
#define PHYCFGR_OPMDC_100F           (3<<3)
#define PHYCFGR_OPMDC_100H           (2<<3)
#define PHYCFGR_OPMDC_10F            (1<<3)
#define PHYCFGR_OPMDC_10H            (0<<3)
#define PHYCFGR_DPX_FULL             (1<<2)
#define PHYCFGR_DPX_HALF             (0<<2)
#define PHYCFGR_SPD_100              (1<<1)
#define PHYCFGR_SPD_10               (0<<1)
#define PHYCFGR_LNK_ON               (1<<0)
#define PHYCFGR_LNK_OFF              (0<<0)

/* IMR register values */
/**
 * @brief IP Conflict Interrupt Mask.
 * @details 0: Disable IP Conflict Interrupt\n
 * 1: Enable IP Conflict Interrupt
 */
#define IM_IR7                  	 0x80

/**
 * @brief Destination unreachable Interrupt Mask.
 * @details 0: Disable Destination unreachable Interrupt\n
 * 1: Enable Destination unreachable Interrupt
 */
#define IM_IR6                  	 0x40

/**
 * @brief PPPoE Close Interrupt Mask.
 * @details 0: Disable PPPoE Close Interrupt\n
 * 1: Enable PPPoE Close Interrupt
 */
#define IM_IR5                  	 0x20

/**
 * @brief Magic Packet Interrupt Mask.
 * @details 0: Disable Magic Packet Interrupt\n
 * 1: Enable Magic Packet Interrupt
 */
#define IM_IR4                  	 0x10

/* Sn_MR values */
#define Sn_MR_CLOSE                  0x00     /**< unused socket */
#define Sn_MR_TCP                    0x01     /**< TCP */
#define Sn_MR_UDP                    0x02     /**< UDP */
#define Sn_MR_IPRAW                  0x03     /**< IP LAYER RAW SOCK */
#define Sn_MR_MACRAW                 0x04     /**< MAC LAYER RAW SOCK */
#define Sn_MR_PPPOE                  0x05     /**< PPPoE */
#define Sn_MR_UCASTB                 0x10     /**< Unicast Block in UDP Multicasting*/
#define Sn_MR_ND                     0x20     /**< No Delayed Ack(TCP) flag */
#define Sn_MR_MC                     0x20     /**< Multicast IGMP (UDP) flag */
#define Sn_MR_BCASTB                 0x40     /**< Broadcast block in UDP Multicasting */
#define Sn_MR_MULTI                  0x80     /**< support UDP Multicasting */

 /* Sn_MR values on MACRAW MODE */
#define Sn_MR_MIP6N                  0x10     /**< IPv6 packet Block */
#define Sn_MR_MMB                    0x20     /**< IPv4 Multicasting Block */
#define Sn_MR_MFEN                   0x80     /**< support MAC filter enable */
#define Sn_MR_MC                     0x20     /**< enable Multicast Blocking. This bit is applied only in MACRAW mode. It blocks to receive the packet with multicast MAC address. */

/* Sn_CR values */
#define Sn_CR_OPEN                   0x01     /**< initialize or open socket */
#define Sn_CR_LISTEN                 0x02     /**< wait connection request in tcp mode(Server mode) */
#define Sn_CR_CONNECT                0x04     /**< send connection request in tcp mode(Client mode) */
#define Sn_CR_DISCON                 0x08     /**< send closing request in tcp mode */
#define Sn_CR_CLOSE                  0x10     /**< close socket */
#define Sn_CR_SEND                   0x20     /**< update txbuf pointer, send data */
#define Sn_CR_SEND_MAC               0x21     /**< send data with MAC address, so without ARP process */
#define Sn_CR_SEND_KEEP              0x22     /**<  send keep alive message */
#define Sn_CR_RECV                   0x40     /**< update rxbuf pointer, recv data */

#ifdef __DEF_WIZCHIP_PPP__
   #define Sn_CR_PCON                0x23
   #define Sn_CR_PDISCON             0x24
   #define Sn_CR_PCR                 0x25
   #define Sn_CR_PCN                 0x26
   #define Sn_CR_PCJ                 0x27
#endif

/* Sn_IR values */
#ifdef __DEF_WIZCHIP_PPP__
   #define Sn_IR_PRECV               0x80
   #define Sn_IR_PFAIL               0x40
   #define Sn_IR_PNEXT               0x20
#endif

#define Sn_IR_SEND_OK                0x10     /**< complete sending */
#define Sn_IR_TIMEOUT                0x08     /**< assert timeout */
#define Sn_IR_RECV                   0x04     /**< receiving data */
#define Sn_IR_DISCON                 0x02     /**< closed socket */
#define Sn_IR_CON                    0x01     /**< established connection */

/* Sn_SR values */
#define SOCK_CLOSED                  0x00     /**< closed */
#define SOCK_ARP		     0x01     /**< arp protocol initiated */
#define SOCK_INIT                    0x13     /**< init state */
#define SOCK_LISTEN                  0x14     /**< listen state */
#define SOCK_SYNSENT                 0x15     /**< connection state */
#define SOCK_SYNRECV                 0x16     /**< connection state */
#define SOCK_ESTABLISHED             0x17     /**< success to connect */
#define SOCK_FIN_WAIT                0x18     /**< closing state */
#define SOCK_CLOSING                 0x1A     /**< closing state */
#define SOCK_TIME_WAIT               0x1B     /**< closing state */
#define SOCK_CLOSE_WAIT              0x1C     /**< closing state */
#define SOCK_LAST_ACK                0x1D     /**< closing state */
#define SOCK_UDP                     0x22     /**< udp socket */
#define SOCK_IPRAW                   0x32     /**< ip raw mode socket */
#define SOCK_MACRAW                  0x42     /**< mac raw mode socket */
#define SOCK_PPPOE                   0x5F     /**< pppoe socket */


/*********************************************************
* W5500 access function
*********************************************************/

void	WIZCHIP_init(void); // reset iinchip
void	WIZCHIP_sw_reset(void); // Reset Wiz550io by softly. Reset the W5500 chip, and preserve only the MAC address.
void	WIZCHIP_sysinit(uint8_t * tx_size, uint8_t * rx_size); // setting tx/rx buf size
uint8_t	WIZCHIP_getISR(uint8_t s);
void	WIZCHIP_putISR(uint8_t s, uint8_t val);

uint8_t	WIZCHIP_read(uint32_t addrbsb) __attribute__ ((hot, flatten));
void	WIZCHIP_write( uint32_t addrbsb,  uint8_t data) __attribute__ ((hot, flatten));

uint16_t	WIZCHIP_getRxMAX(uint8_t s);
uint16_t	WIZCHIP_getTxMAX(uint8_t s);

void	setRTR(uint16_t timeout); // set retry duration for data transmission, connection, closing ...
void	setRCR(uint8_t retry); // set retry count (above the value, assert timeout interrupt)
void 	clearIR(uint8_t mask); // clear interrupt
uint8_t	getIR( void );
void	setSn_MSS(SOCKET s, uint16_t Sn_MSSR); // set maximum segment size
void	setSn_PROTO(SOCKET s, uint8_t proto); // set IP Protocol value using IP-Raw mode
uint8_t	getSn_IR(SOCKET s); // get socket interrupt status
uint8_t	getSn_SR(SOCKET s); // get socket status
uint16_t getSn_TX_FSR(SOCKET s); // get socket TX free buf size
uint16_t getSn_RX_RSR(SOCKET s); // get socket RX recv buf size


void setSn_TTL(SOCKET s, uint8_t ttl);
void setMR(uint8_t val);
uint8_t getMR(void);

void setGAR(uint8_t * addr); // set gateway address
void setSHAR(uint8_t * addr); // set local MAC address
void setSIPR(uint8_t * addr); // set local IP address

void setSUBR(uint8_t * addr); // set subnet mask
void getSUBR(uint8_t * addr);

void getGAR(uint8_t * addr);
void getSHAR(uint8_t * addr);
void getSIPR(uint8_t * addr);

uint16_t WIZCHIP_write_buf(uint32_t addrbsb, uint8_t * buf,uint16_t len) __attribute__ ((hot));
uint16_t WIZCHIP_read_buf(uint32_t addrbsb, uint8_t* buf,uint16_t len) __attribute__ ((hot));

void WIZCHIP_send_data_processing(SOCKET s, uint8_t *wizdata, uint16_t len);
void WIZCHIP_recv_data_processing(SOCKET s, uint8_t *wizdata, uint16_t len);

void WIZCHIP_write_data(SOCKET s, uint8_t * src, uint8_t * dst, uint16_t len);
void WIZCHIP_read_data(SOCKET s, uint8_t * src, uint8_t * dst, uint16_t len);

#endif // #if (_WIZCHIP_ == 5500)

#ifdef __cplusplus
}
#endif

#endif
