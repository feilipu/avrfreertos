#ifndef __SIMPLE_HTTPD_H__
#define __SIMPLE_HTTPD_H__

#include "tcp-apps.h"

#define ISO_nl      '\n'
#define ISO_space   ' '
#define ISO_slash   '/'

void simple_httpd_init(void);
void simple_httpd_appcall(void);


#endif /* __SIMPLE_HTTPD_H__ */
