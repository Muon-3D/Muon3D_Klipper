#include <stdint.h>
uint32_t timer_read_time(void);
#define timer_from_us(us) ((uint32_t)(us) * 12u)
#define timer_is_before(a,b) ((int32_t)((a)-(b)) < 0)
