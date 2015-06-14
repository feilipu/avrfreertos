#include <stdlib.h>
#include <stdio.h>

#include <inet.h>
#include <uIP/uip.h>

#include <uIP/sys/stimer.h>
#include <uIP/apps/udp-apps.h>
#include <uIP/apps/dhcpc.h>


extern uip_eth_addr  my_eth_addr; // definition of the Ethernet address.

extern struct dhcpc_state dhcpc_state; // this is the DHCP state variable.

/*------------------------------------------------------*/

void udp_apps_init(void)
{
#ifdef UIP_DHCP
	dhcpc_init( my_eth_addr.addr, UIP_LLADDR_LEN);
#endif // UIP_DHCP
}

void udp_apps_appcall(void)
{
#ifdef UIP_DHCP
	switch(uip_udp_conn->lport)
	{
		case UIP_HTONS(IP_PORT_DHCP_SERVER):
		case UIP_HTONS(IP_PORT_DHCP_CLIENT):
			dhcpc_appcall();
			break;
//		case UIP_HTONS(IP_PORT_NTP):
//			ntpd_appcall();
			break;
		default:
			break;
	}


	if( stimer_expired(&dhcpc_state.stimer) )
	{
		dhcpc_state.state = DHCP_STATE_INITIAL;
		dhcpc_unconfigured(&dhcpc_state);
		dhcpc_appcall();
	}
#endif // UIP_DHCP
}
