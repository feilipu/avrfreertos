//Project specific configurations
#ifndef __GLOBAL_CONF_H__
#define __GLOBAL_CONF_H__

//Mac address definition - printed on top of module (get your own - don't use mine)
#define ETHADDR0		0x00
#define ETHADDR1		0x1E
#define ETHADDR2		0xC0
#define ETHADDR3		0x00
#define ETHADDR4		0x12
#define ETHADDR5		0xE9

//Mac address definition for uIP
#define UIP_ETHADDR0    ETHADDR0
#define UIP_ETHADDR1    ETHADDR1
#define UIP_ETHADDR2    ETHADDR2
#define UIP_ETHADDR3    ETHADDR3
#define UIP_ETHADDR4    ETHADDR4
#define UIP_ETHADDR5    ETHADDR5

#define USE_DHCP 0
#define UIP_IPADDR0 10
#define UIP_IPADDR1 42
#define UIP_IPADDR2 0
#define UIP_IPADDR3 88
#define UIP_NETMASK0 255
#define UIP_NETMASK1 255
#define UIP_NETMASK2 255
#define UIP_NETMASK3 0
#define UIP_DRIPADDR0 10
#define UIP_DRIPADDR1 42
#define UIP_DRIPADDR2 0
#define UIP_DRIPADDR3 254

//Include uip.h gives all the uip configurations in uip-conf.h
#include "uip.h"
#include "config.h" // ZG2100 configuration information

#endif /*__GLOBAL_CONF_H__*/
