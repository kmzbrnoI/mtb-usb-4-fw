#pragma once

extern "C" {

#include "stm32f1xx_hal.h"

void Error_Handler(void);

#define LED_BLUE_Pin GPIO_PIN_12
#define LED_BLUE_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_13
#define LED_RED_GPIO_Port GPIOB
#define RED_YEL_Pin GPIO_PIN_14
#define RED_YEL_GPIO_Port GPIOB
#define LED_GR_Pin GPIO_PIN_15
#define LED_GR_GPIO_Port GPIOB

}
