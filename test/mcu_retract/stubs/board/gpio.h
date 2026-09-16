#ifndef TEST_GPIO_H
#define TEST_GPIO_H
struct gpio_out { int pin; };
struct gpio_out gpio_out_setup(unsigned int pin, unsigned int value);
void gpio_out_write(struct gpio_out pin, unsigned int value);
void gpio_out_toggle_noirq(struct gpio_out pin);
#endif
