#include "mtbbus.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"

UART_HandleTypeDef h_uart_mtbbus;

bool mtbbus_init(void) {
	h_uart_mtbbus.Instance = USART3;
	h_uart_mtbbus.Init.BaudRate = 115200;
	h_uart_mtbbus.Init.WordLength = UART_WORDLENGTH_9B;
	h_uart_mtbbus.Init.StopBits = UART_STOPBITS_1;
	h_uart_mtbbus.Init.Parity = UART_PARITY_NONE;
	h_uart_mtbbus.Init.Mode = UART_MODE_TX_RX;
	h_uart_mtbbus.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	h_uart_mtbbus.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&h_uart_mtbbus) != HAL_OK)
		return false;

	__HAL_RCC_USART3_CLK_ENABLE();

	gpio_pin_init(pin_usart_mtb_tx, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, false);
	gpio_pin_init(pin_usart_mtb_rx, GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH, false);

	return true;
}
