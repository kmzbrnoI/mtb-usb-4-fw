#include <stdint.h>
#include "mtbbus.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_dma.h"
#include "stm32f1xx_ll_usart.h"
#include "gpio.h"

UART_HandleTypeDef _h_uart_mtbbus;
uint8_t *_data_to_send;
size_t _size_to_send;

static DMA_HandleTypeDef _dma_tx_handle;

USART_TypeDef* const _uart = USART3;
const IRQn_Type _uart_tx_dma_irqn = DMA1_Channel2_IRQn;
//const IRQn_Type _uart_irqn = USART3_IRQn;
const unsigned _uart_tx_dma_irq_prio = 8;
//const unsigned _uart_irq_prio = 10;
DMA_Channel_TypeDef* const _uart_tx_dma_channel = DMA1_Channel2;


bool mtbbus_init(void) {
	__HAL_RCC_USART3_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	LL_USART_InitTypeDef init;
	LL_USART_StructInit(&init);
	init.BaudRate = 38400;
	init.DataWidth = LL_USART_DATAWIDTH_8B;
	init.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	init.Parity = LL_USART_PARITY_NONE;
	init.StopBits = LL_USART_STOPBITS_1;
	init.TransferDirection = LL_USART_DIRECTION_TX_RX;
	LL_USART_Init(_uart, &init);
	LL_USART_Enable(_uart);

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
	LL_USART_EnableDMAReq_TX(_uart);

	gpio_pin_init(pin_usart_mtb_tx, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, false);
	gpio_pin_init(pin_usart_mtb_rx, GPIO_MODE_AF_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH, false);
	gpio_pin_init(pin_usart_mtb_dir, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, false);

	return true;
}

void mtbbus_send(uint8_t *data, size_t length) {
	gpio_pin_write(pin_usart_mtb_dir, true);
	HAL_DMA_Start(&_dma_tx_handle, (uintptr_t)data, (uintptr_t)&_uart->DR, length);
}

bool mtbbus_can_send(void) {
	HAL_DMA_PollForTransfer(&_dma_tx_handle, HAL_DMA_FULL_TRANSFER, 0);
	return _dma_tx_handle.State == HAL_DMA_STATE_READY;
}
