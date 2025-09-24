#ifndef RPI_H
#define RPI_H

#include <stddef.h>
#include "types.h"

void gpio_init();
void set_gpio_pin(u32 pin, u32 value);

#define GPIO_INDICATOR_BITS 8
void update_gpio_indicator(u32 value);
#endif /* rpi.h */
