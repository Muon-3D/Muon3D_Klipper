// Host-test stub of the mcu GPIO header (see ../../fw_harness.c)
#ifndef __STUB_GPIO_H
#define __STUB_GPIO_H
#include <stdint.h>
struct gpio_out { int pin; };
struct gpio_out gpio_out_setup(uint32_t pin, uint8_t val);
void gpio_out_write(struct gpio_out g, uint8_t val);
#endif
