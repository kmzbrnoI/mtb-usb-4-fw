#include <stdint.h>
#include "mtbbus.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_dma.h"
#include "gpio.h"

UART_HandleTypeDef _h_uart_mtbbus;
static DMA_HandleTypeDef _dma_tx_handle;

USART_TypeDef* const _uart = USART3;

const IRQn_Type _uart_tx_dma_irqn = DMA1_Channel2_IRQn;
const unsigned _uart_tx_dma_irq_prio = 8;

const IRQn_Type _uart_irqn = USART3_IRQn;
const unsigned _uart_irq_prio = 10;


DMA_Channel_TypeDef* const _uart_tx_dma_channel = DMA1_Channel2;


bool mtbbus_init(void) {
	__HAL_RCC_USART3_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	_h_uart_mtbbus.Instance = _uart;
	_h_uart_mtbbus.Init.BaudRate = 38400;
	_h_uart_mtbbus.Init.WordLength = UART_WORDLENGTH_8B;
	_h_uart_mtbbus.Init.StopBits = UART_STOPBITS_1;
	_h_uart_mtbbus.Init.Parity = UART_PARITY_NONE;
	_h_uart_mtbbus.Init.Mode = UART_MODE_TX_RX;
	_h_uart_mtbbus.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	_h_uart_mtbbus.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&_h_uart_mtbbus) != HAL_OK)
		return false;

	// UART TX burst is started ad hoc each time
	_dma_tx_handle.Instance = _uart_tx_dma_channel;
	_dma_tx_handle.Init.Direction = DMA_MEMORY_TO_PERIPH;
	_dma_tx_handle.Init.Mode = DMA_NORMAL;
	_dma_tx_handle.Init.MemInc = DMA_MINC_ENABLE;
	_dma_tx_handle.Init.PeriphInc = DMA_PINC_DISABLE;
	_dma_tx_handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	_dma_tx_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	_dma_tx_handle.Init.Priority = DMA_PRIORITY_MEDIUM;
	if (HAL_DMA_Init(&_dma_tx_handle) != HAL_OK)
		return false;
	HAL_NVIC_SetPriority(_uart_tx_dma_irqn, _uart_tx_dma_irq_prio, 0);
	HAL_NVIC_EnableIRQ(_uart_tx_dma_irqn);
	__HAL_LINKDMA(&_h_uart_mtbbus, hdmatx, _dma_tx_handle);

	HAL_NVIC_SetPriority(_uart_irqn, _uart_irq_prio, 0);
	HAL_NVIC_EnableIRQ(_uart_irqn);

	gpio_pin_init(pin_usart_mtb_tx, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, false);
	gpio_pin_init(pin_usart_mtb_rx, GPIO_MODE_AF_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH, false);
	gpio_pin_init(pin_usart_mtb_dir, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, false);

	return true;
}

void mtbbus_send(uint8_t *data, size_t length) {
	gpio_pin_write(pin_usart_mtb_dir, true);
	HAL_UART_Transmit_DMA(&_h_uart_mtbbus, data, length);
}

bool mtbbus_can_send(void) {
	HAL_DMA_PollForTransfer(&_dma_tx_handle, HAL_DMA_FULL_TRANSFER, 0);
	return _dma_tx_handle.State == HAL_DMA_STATE_READY;
}

void DMA1_Channel2_IRQHandler() {
	HAL_DMA_IRQHandler(&_dma_tx_handle);
}

void USART3_IRQHandler(void) {
	HAL_UART_IRQHandler(&_h_uart_mtbbus);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3)
		gpio_pin_write(pin_usart_mtb_dir, false);
}
