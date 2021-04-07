#pragma once

#include <stdint.h>
#include "gpio.h"

void leds_init(void);
void leds_update_1ms(void);
void led_activate(PinDef pin, size_t millis_enable, size_t millis_disable);
