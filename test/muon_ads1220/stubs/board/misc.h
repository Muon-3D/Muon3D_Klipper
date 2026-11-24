// Host-test stub of the mcu timer header (see ../../fw_harness.c)
#ifndef __STUB_MISC_H
#define __STUB_MISC_H
#include <stdint.h>
#define CONFIG_CLOCK_FREQ 12000000
uint32_t timer_read_time(void);
static inline uint32_t timer_from_us(uint32_t us)
{
    return us * (CONFIG_CLOCK_FREQ / 1000000);
}
#endif
