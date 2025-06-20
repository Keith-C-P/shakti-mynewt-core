#include <stddef.h>
#include <mcu/sys_clock.h>
#include "bsp/bsp.h"

uint32_t
get_cpu_freq(void)
{
   return CLOCK_FREQUENCY;
}

void
select_clock(const clock_config_t *cfg)
{
}
