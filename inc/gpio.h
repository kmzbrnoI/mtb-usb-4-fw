#pragma once

#include <stdbool.h>
#include "stm32f1xx_hal.h"

typedef struct {
	GPIO_TypeDef* port;
	uint32_t pin;
} PinDef;

extern const PinDef pin_led_red;
extern const PinDef pin_led_green;
extern const PinDef pin_led_blue;
extern const PinDef pin_led_yellow;

extern const PinDef pin_usb_dn;
extern const PinDef pin_usb_dp;
extern const PinDef pin_usb_dp_pullup;

void gpio_init(void);

inline void gpio_pin_init(PinDef pin, uint32_t mode, uint32_t pull, uint32_t speed);
bool gpio_pin_read(PinDef pin);
void gpio_pin_write(PinDef pin, bool value);
void gpio_pin_toggle(PinDef pin);
