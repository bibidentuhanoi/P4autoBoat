#pragma once

typedef int gpio_num_t;

#define GPIO_MODE_OUTPUT 1

int gpio_set_direction(gpio_num_t gpio, int mode);
int gpio_set_level(gpio_num_t gpio, int level);
