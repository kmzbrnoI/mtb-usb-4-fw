#include "mtbbus.h"
#include "stm32f1xx_hal.h"

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
	return (HAL_UART_Init(&h_uart_mtbbus) == HAL_OK);
}
