//Project specific configurations
#ifndef __UIP_GLOBAL_CONF_H__
#define __UIP_GLOBAL_CONF_H__

//Mac address definition - (get your own - don't use mine)
#define ETHADDR0		0x00
#define ETHADDR1		0x08
#define ETHADDR2		0xDC
#define ETHADDR3		0x1D
#define ETHADDR4		0x62
#define ETHADDR5		0x6C

//Mac address definition for uIP
#define UIP_ETHADDR0    ETHADDR0
#define UIP_ETHADDR1    ETHADDR1
#define UIP_ETHADDR2    ETHADDR2
#define UIP_ETHADDR3    ETHADDR3
#define UIP_ETHADDR4    ETHADDR4
#define UIP_ETHADDR5    ETHADDR5

#define UIP_IPADDR0 192
#define UIP_IPADDR1 168
#define UIP_IPADDR2 1
#define UIP_IPADDR3 32

#define UIP_NETMASK0 255
#define UIP_NETMASK1 255
#define UIP_NETMASK2 255
#define UIP_NETMASK3 0

#define UIP_DRIPADDR0 192
#define UIP_DRIPADDR1 168
#define UIP_DRIPADDR2 1
#define UIP_DRIPADDR3 254

/** @} */
/*------------------------------------------------------------------------------*/

/**
 * \name Application specific configurations
 * @{
 *
 * An uIP application is implemented using a single application
 * function that is called by uIP whenever a TCP/IP event occurs. The
 * name of this function must be registered with uIP at compile time
 * using the UIP_TCP_APPCALL definition.
 *
 * uIP applications can store the application state within the
 * uip_conn structure by specifying the type of the application
 * structure by typedef:ing the type uip_tcp_appstate_t and uip_udp_appstate_t.
 *
 *
 * The following example illustrates how this can look.
 \code


void httpd_appcall(void);

#define UIP_TCP_APPCALL     httpd_appcall

struct httpd_state {
  uint8_t state;
  uint16_t count;
  char *dataptr;
  char *script;
};
typedef struct httpd_state uip_tcp_appstate_t
 \endcode
 */

/**
 * \var #define UIP_TCP_APPCALL
 *
 * The name of the application function that uIP should call in
 * response to TCP/IP events.
 *
 */

/**
 * \var typedef uip_tcp_appstate_t
 *
 * The type of the application state that is to be stored in the
 * uip_conn structure. This usually is typedef:ed to a struct holding
 * application state information.
 */

/**
 * \var typedef uip_udp_appstate_t
 *
 * The type of the application state that is to be stored in the
 * uip_conn structure. This usually is typedef:ed to a struct holding
 * application state information.
 */
/** @} */

// void tcp_appcall(void); // xxx a_tcp_appcall() is a callback for when no TCP app is being run (debugging).

//#define UIP_TCP_APPCALL()   // tcp_appcall() // xxx a_tcp_appcall() is a callback for when no TCP app is being run (debugging).

/* UIP_TCP_APPCALL: the name of the application function. This function
   must return void and take no arguments (i.e., C type "void appfunc(void)"). */
#define UIP_TCP_APPCALL tcp_apps_appcall

/*
typedef struct tcp_appstate_t
 {
   uint8_t state;
   uint8_t p;
   uint16_t count;
   char *dataptr;
   char *script;
 } tcp_appstate_t; */

 /**
  * The name of the function that should be called when UDP datagrams arrive.
  *
  * \hideinitializer
  */
void udp_appcall(void);

//#define UIP_UDP_APPCALL()   // udp_appcall() // xxx a_udp_appcall() is a callback for when no UDP app that is being run (debugging).

/* UIP_UDP_APPCALL: the name of the application function. This function
   must return void and take no arguments (i.e., C type "void appfunc(void)"). */
#define UIP_UDP_APPCALL udp_apps_appcall

/*
typedef struct udp_appstate_t
{
  uint8_t state;
  uint8_t p;
  uint16_t count;
  char *dataptr;
  char *script;
} uip_udp_appstate_t; // */


// include the tcp application headers
#include <uIP/apps/tcp-apps.h>

// include the udp application headers
#include <uIP/apps/udp-apps.h>

//Include uip.h gives all the uip configurations in uip-conf.h
#include <uIP/uip.h>

// Include the ARP protocol
#include <uIP/uip-arp.h>

#endif /*__UIP_GLOBAL_CONF_H__*/
