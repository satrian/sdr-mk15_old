#ifndef __CLOCK_ARCH_H__
#define __CLOCK_ARCH_H__

#include <stdint.h>

typedef uint32_t clock_time_t;

#define CLOCK_CONF_SECOND		(clock_time_t)1000L		// we are using the 1ms timer resolution for clock_time() as 1ms for AT32UC3B

void clock_init(void);
uint32_t millis(void);

#include "clock.h"

#endif /* __CLOCK_ARCH_H__ */
