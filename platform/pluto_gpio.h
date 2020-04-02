//
// Created by lukas on 30.03.20.
//

#ifndef TRANSCEIVER_PLUTO_GPIO_H
#define TRANSCEIVER_PLUTO_GPIO_H

#include <pthread.h>

enum pin_level {LOW=0, HIGH=1};
enum pin_direction {IN=0, OUT=1};

#define PLUTO_GPIO_BASE 906

#define PIN_MIO0 0
#define PIN_MIO10 10

struct gpio_pin_s;
typedef struct gpio_pin_s* gpio_pin;

// initializer
gpio_pin pluto_gpio_init(int pin_id, int direction);
void pluto_gpio_destroy(gpio_pin gpio);

// pin write functions
void pluto_gpio_pin_write(gpio_pin gpio, int level);
void pluto_gpio_pin_write_delayed(gpio_pin gpio, int level, int delay_us);


#endif //TRANSCEIVER_PLUTO_GPIO_H
