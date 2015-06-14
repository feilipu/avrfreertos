#ifndef __TCP_APPS_H__
#define __TCP_APPS_H__

#include "time.h"

#include "uIP/uip-ip-addr.h"
#include "uIP/psock.h"
#include "uIP/sys/ttimer.h"

struct httpd_state {
  uint8_t state;
  uint16_t count;
  char *dataptr;
  char *script;
};

struct simple_httpd_state {
	struct psock p;
	uint8_t inputbuf[32];
};

//typedef struct httpd_state uip_tcp_appstate_t; // make sure this doesn't conflict with other uip_tcp_appstate_t definitions.
typedef struct simple_httpd_state uip_tcp_appstate_t; // make sure this doesn't conflict with other uip_tcp_appstate_t definitions.

void tcp_apps_init(void);

void tcp_apps_appcall(void);

#include <uIP/apps/simple-httpd.h>

#endif /* __UDP_APP_H__ */
