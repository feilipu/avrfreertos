#include <stdlib.h>
#include <stdio.h>

#include "inet.h"
#include "uIP/uip.h"

#include "uIP/apps/tcp-apps.h"
#include "uIP/apps/simple-httpd.h"


/*------------------------------------------------------*/

void tcp_apps_init(void)
{
	simple_httpd_init();
	//httpd_init();
}

void tcp_apps_appcall(void)
{
	switch(uip_conn->lport)
	{
		case UIP_HTONS(IP_PORT_HTTP):
			simple_httpd_appcall();
			break;
//		case UIP_HTONS(IP_PORT_HTTP):
//			httpd_appcall();
			break;
		default:
			break;
	}
}
