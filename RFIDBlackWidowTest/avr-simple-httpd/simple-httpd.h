#ifndef __SIMPLE_HTTPD_H__
#define __SIMPLE_HTTPD_H__

#include "../avr-uIP/psock.h"

#define ISO_nl      0x0a
#define ISO_space   0x20
#define ISO_slash   0x2f

/* UIP_APPCALL: the name of the application function. This function
   must return void and take no arguments (i.e., C type "void
   appfunc(void)"). */
#define UIP_APPCALL simple_httpd_appcall


typedef struct simple_httpd_state {
	struct psock p;
	char inputbuf[10];
} uip_tcp_appstate_t;

void simple_httpd_init(void);
void simple_httpd_appcall(void);


#endif /* __SIMPLE_HTTPD_H__ */
