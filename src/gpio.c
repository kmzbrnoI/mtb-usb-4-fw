#include "gpio.h"
#include "stm32f1xx_hal.h"

const PinDef pinLedRed = {GPIOB, GPIO_PIN_13};
const PinDef pinLedGreen = {GPIOB, GPIO_PIN_15};
const PinDef pinLedBlue = {GPIOB, GPIO_PIN_12};
const PinDef pinLedYellow = {GPIOB, GPIO_PIN_14};

const PinDef usbDnPin = {GPIOA, GPIO_PIN_11};
const PinDef usbDpPin = {GPIOA, GPIO_PIN_12};
const PinDef usbDpPullUpPin = {USBD_DP_PORT, 1 << USBD_DP_PIN};

void gpio_init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2 | pinLedBlue.pin | pinLedRed.pin | pinLedYellow.pin | pinLedGreen.pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : PB2 LED_BLUE_Pin LED_RED_Pin RED_YEL_Pin LED_GR_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_2 | pinLedBlue.pin | pinLedRed.pin | pinLedYellow.pin | pinLedGreen.pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PA8 PA9 */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	gpio_pin_init(usbDnPin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
	gpio_pin_init(usbDpPin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
	gpio_pin_init(usbDpPullUpPin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
}

void gpio_pins_init(GPIO_TypeDef* port, uint32_t pinMask, uint32_t mode, uint32_t pull, uint32_t speed) {
	GPIO_InitTypeDef init;
	init.Pin = pinMask;
	init.Mode = mode;
	init.Pull = pull;
	init.Speed = speed;
	HAL_GPIO_Init(port, &init);
}

inline void gpio_pin_init(PinDef pin, uint32_t mode, uint32_t pull, uint32_t speed) {
	gpio_pins_init(pin.port, pin.pin, mode, pull, speed);
}

bool gpio_pin_read(PinDef pin) {
	return HAL_GPIO_ReadPin(pin.port, pin.pin) == GPIO_PIN_SET;
}

void gpio_pin_write(PinDef pin, bool value) {
	HAL_GPIO_WritePin(pin.port, pin.pin, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void gpio_pin_toggle(PinDef pin) {
	HAL_GPIO_TogglePin(pin.port, pin.pin);
}
