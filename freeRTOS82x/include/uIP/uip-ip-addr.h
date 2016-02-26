//IP address configurations
#ifndef __UIP_IP_ADDR_H__
#define __UIP_IP_ADDR_H__

#include <stdint.h>

#include "uIP/uip-conf.h"
/**
 * Representation of an IP address.
 * MUST COME FIRST
 * It is separated out, for udp and tcp applications to use when defining their appstate
 *
 */
typedef union uip_ip4addr_t {
  uint8_t  u8[4];			/* Initializer, must come first. */
  uint16_t u16[2];
  uint32_t u32;
} uip_ip4addr_t;

typedef union uip_ip6addr_t {
  uint8_t  u8[16];			/* Initializer, must come first. */
  uint16_t u16[8];
  uint32_t u32[4];
} uip_ip6addr_t;


#if UIP_CONF_IPV6
typedef union uip_ip6addr_t uip_ipaddr_t;
#else /* UIP_CONF_IPV6 */
typedef union uip_ip4addr_t uip_ipaddr_t;
#endif /* UIP_CONF_IPV6 */

#endif /*__UIP_GLOBAL_CONF_H__*/
