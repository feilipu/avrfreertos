#ifndef __UDP_APPS_H__
#define __UDP_APPS_H__

#include "time.h"

#include "uIP/uip-ip-addr.h"
#include "uIP/psock.h"
#include "uIP/sys/stimer.h"

struct dhcpc_state {
	uint8_t state;
	struct uip_udp_conn *conn;
	struct stimer stimer;
	const void *mac_addr;
	uint8_t mac_len;

	uint8_t serverid[4];
	uint16_t lease_time[2];

	uip_ipaddr_t ipaddr;
	uip_ipaddr_t netmask;
	uip_ipaddr_t dnsaddr;
	uip_ipaddr_t default_router;
};


#define UIP_DHCP

typedef struct dhcpc_state uip_udp_appstate_t; // make sure this doesn't conflict with other uip_udp_appstate_t definitions.

void udp_apps_init(void);

void udp_apps_appcall(void);

#include <uIP/apps/dhcpc.h>

#endif /* __UDP_APP_H__ */
