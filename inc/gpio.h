#pragma once

#include <stdbool.h>
#include "stm32f1xx_hal.h"

typedef struct {
	GPIO_TypeDef* port;
	uint32_t pin;
} PinDef;

extern const PinDef pinLedRed;
extern const PinDef pinLedGreen;
extern const PinDef pinLedBlue;
extern const PinDef pinLedYellow;

void gpio_init(void);

inline void gpio_pin_init(PinDef pin, uint32_t mode, uint32_t pull, uint32_t speed);
bool gpio_pin_read(PinDef pin);
void gpio_pin_write(PinDef pin, bool value);
void gpio_pin_toggle(PinDef pin);
