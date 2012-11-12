#ifndef __UIP_CONF_H__
#define __UIP_CONF_H__

#include <inttypes.h>
#include <avr32/io.h>
#include <stdio.h>
#include <stdbool.h>

/**
 * 8 bit datatype
 *
 * This typedef defines the 8-bit type used throughout uIP.
 *
 * \hideinitializer
 */
typedef uint8_t u8_t;

/**
 * 16 bit datatype
 *
 * This typedef defines the 16-bit type used throughout uIP.
 *
 * \hideinitializer
 */
typedef uint16_t u16_t;

/**
 * Statistics datatype
 *
 * This typedef defines the dataype used for keeping statistics in
 * uIP.
 *
 * \hideinitializer
 */
typedef unsigned short uip_stats_t;

/**
 * Maximum number of TCP connections.
 *
 * \hideinitializer
 */
#define UIP_CONF_MAX_CONNECTIONS 2

/**
 * Maximum number of listening TCP ports.
 *
 * \hideinitializer
 */
#define UIP_CONF_MAX_LISTENPORTS 1

/**
 * uIP buffer size.
 *
 * \hideinitializer
 */
#define UIP_CONF_BUFFER_SIZE     1600	//600	//2048

#define UIP_CONF_EXTERNAL_BUFFER
extern u8_t* uip_buf;

/**
 * CPU byte order.
 *
 * \hideinitializer
 */
#define BIG_ENDIAN				 1234
#define UIP_CONF_BYTE_ORDER      BIG_ENDIAN		//LITTLE_ENDIAN

/**
 * Logging on or off
 *
 * \hideinitializer
 */
#define UIP_CONF_LOGGING         0

/**
 * UDP support on or off requires DHCP if ON
 *
 * \hideinitializer
 */
#define UIP_CONF_UDP             1

/**
 * UDP checksums on or off
 *
 * \hideinitializer
 */
#define UIP_CONF_UDP_CHECKSUMS   0		// no time to calculate ...

/**
 * uIP statistics on or off
 *
 * \hideinitializer
 */
#define UIP_CONF_STATISTICS      0

/**
 * Broadcast support. Needed for dhcp and SDR discovery protocol
 *
 * \hideinitializer
 */
#define UIP_CONF_BROADCAST		1

/**
 * The maximum amount of concurrent UDP connections.
 *
 * \hideinitializer
 */
#define UIP_CONF_UDP_CONNS		3		// need at least three: 1) DHCP 2)SDR discovery protocol 3)transmitting data to host

// enable IP_V6 (uip-neighbor.c has to be added to project)
#define UIP_CONF_IPV6			0

// do not try re-assemble fragmented incoming packets
#define UIP_REASSEMBLY			0

//Include app configuration
#include "apps-conf.h"

#endif /* __UIP_CONF_H__ */

/** @} */
/** @} */
