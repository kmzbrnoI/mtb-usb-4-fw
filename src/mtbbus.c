#include <stdint.h>
#include "mtbbus.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_dma.h"
#include "gpio.h"
#include "crc16modbus.h"

/* Low-level UART handler, interrupts etc. -----------------------------------*/

UART_HandleTypeDef _h_uart_mtbbus;
static DMA_HandleTypeDef _dma_tx_handle;
static DMA_HandleTypeDef _dma_rx_handle;

USART_TypeDef* const _uart = USART3;
const IRQn_Type _uart_tx_dma_irqn = DMA1_Channel2_IRQn;
const unsigned _uart_tx_dma_irq_prio = 8;
const IRQn_Type _uart_rx_dma_irqn = DMA1_Channel3_IRQn;
const unsigned _uart_rx_dma_irq_prio = 8;
const IRQn_Type _uart_irqn = USART3_IRQn;
const unsigned _uart_irq_prio = 10;
DMA_Channel_TypeDef* const _uart_tx_dma_channel = DMA1_Channel2;
DMA_Channel_TypeDef* const _uart_rx_dma_channel = DMA1_Channel3;

/* Buffers -------------------------------------------------------------------*/

uint16_t _out_buf[MTBBUS_OUT_BUF_SIZE];
uint16_t mtbbus_received_data[MTBBUS_IN_BUF_SIZE];
bool _receiving_first = false;
bool mtbbus_received_read = true;
volatile size_t _response_counter = 0;

#define RESPONSE_COUNTER_FULL 4 // 200 us

/* Higher-level data structures ----------------------------------------------*/

size_t mtbbus_received_addr = 0;
size_t _inquiry_module = 0;
size_t _sent_command_code = 0;

/* Private function prototypes -----------------------------------------------*/

void _inquiry_response_ok(size_t addr);
void _message_received();
void _message_timeout();
static inline void _rx_interrupt_enable();
static inline void _rx_interrupt_disable();

/* Private code --------------------------------------------------------------*/

bool mtbbus_init(void) {
	__HAL_RCC_USART3_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	_h_uart_mtbbus.Instance = _uart;
	_h_uart_mtbbus.Init.BaudRate = 38400;
	_h_uart_mtbbus.Init.WordLength = UART_WORDLENGTH_9B;
	_h_uart_mtbbus.Init.StopBits = UART_STOPBITS_1;
	_h_uart_mtbbus.Init.Parity = UART_PARITY_NONE;
	_h_uart_mtbbus.Init.Mode = UART_MODE_TX_RX;
	_h_uart_mtbbus.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	_h_uart_mtbbus.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_NVIC_SetPriority(_uart_irqn, _uart_irq_prio, 0);
	HAL_NVIC_EnableIRQ(_uart_irqn);
	if (HAL_UART_Init(&_h_uart_mtbbus) != HAL_OK)
		return false;

	// UART RX runs indefinitely in circular mode
	_dma_rx_handle.Instance = _uart_rx_dma_channel;
	_dma_rx_handle.Init.Direction = DMA_PERIPH_TO_MEMORY;
	_dma_rx_handle.Init.Mode = DMA_CIRCULAR;
	_dma_rx_handle.Init.MemInc = DMA_MINC_ENABLE;
	_dma_rx_handle.Init.PeriphInc = DMA_PINC_DISABLE;
	_dma_rx_handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	_dma_rx_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	_dma_rx_handle.Init.Priority = DMA_PRIORITY_MEDIUM;
	HAL_DMA_Init(&_dma_rx_handle);
	//HAL_DMA_Start(&_dma_rx_handle, uintptr_t(&(controlUart->DR)), uintptr_t(rxFifo.data()), rxFifo.size());
	//LL_USART_EnableDMAReq_RX(controlUart);
	HAL_NVIC_SetPriority(_uart_rx_dma_irqn, _uart_rx_dma_irq_prio, 0);
	HAL_NVIC_EnableIRQ(_uart_rx_dma_irqn);
	__HAL_LINKDMA(&_h_uart_mtbbus, hdmarx, _dma_rx_handle);

	// UART TX burst is started ad hoc each time
	_dma_tx_handle.Instance = _uart_tx_dma_channel;
	_dma_tx_handle.Init.Direction = DMA_MEMORY_TO_PERIPH;
	_dma_tx_handle.Init.Mode = DMA_NORMAL;
	_dma_tx_handle.Init.MemInc = DMA_MINC_ENABLE;
	_dma_tx_handle.Init.PeriphInc = DMA_PINC_DISABLE;
	_dma_tx_handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	_dma_tx_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	_dma_tx_handle.Init.Priority = DMA_PRIORITY_MEDIUM;
	if (HAL_DMA_Init(&_dma_tx_handle) != HAL_OK)
		return false;
	HAL_NVIC_SetPriority(_uart_tx_dma_irqn, _uart_tx_dma_irq_prio, 0);
	HAL_NVIC_EnableIRQ(_uart_tx_dma_irqn);
	__HAL_LINKDMA(&_h_uart_mtbbus, hdmatx, _dma_tx_handle);

	gpio_pin_init(pin_usart_mtb_tx, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, false);
	gpio_pin_init(pin_usart_mtb_rx, GPIO_MODE_AF_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH, false);
	gpio_pin_init(pin_usart_mtb_dir, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, false);

	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	return true;
}

bool mtbbus_can_send(void) {
	HAL_DMA_PollForTransfer(&_dma_tx_handle, HAL_DMA_FULL_TRANSFER, 0);
	return (_dma_tx_handle.State == HAL_DMA_STATE_READY) && (mtbbus_received_read);
}

void DMA1_Channel2_IRQHandler() {
	HAL_DMA_IRQHandler(&_dma_tx_handle);
}

void DMA1_Channel3_IRQHandler() {
	HAL_DMA_IRQHandler(&_dma_rx_handle);
}

void USART3_IRQHandler(void) {
	HAL_UART_IRQHandler(&_h_uart_mtbbus);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		gpio_pin_write(pin_usart_mtb_dir, false);

		// read just a single byte in DMA mode (first byte contains number of bytes following)
		_response_counter = RESPONSE_COUNTER_FULL;
		_receiving_first = true;
		HAL_UART_DMAStop(&_h_uart_mtbbus);
		HAL_UART_Receive_DMA(&_h_uart_mtbbus, (uint8_t*)mtbbus_received_data, 1);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		if (_receiving_first) {
			// first byte received → read rest of bytes in DMA mode
			_receiving_first = false;
			HAL_UART_DMAStop(&_h_uart_mtbbus);
			HAL_UART_Receive_DMA(&_h_uart_mtbbus, (uint8_t*)&mtbbus_received_data[1], mtbbus_received_data[0]+2);
		} else {
			// all data received
			_message_received();
		}
	}
}

void _message_received() {
	// TODO: maybe it is better/faster to calc xor as data arrives (receive data per 1 byte)
	// call to crc16modbus_bytes can take time; this function is called from interrupt → should be fast
	uint16_t crc = crc16modbus_bytes(0, mtbbus_received_data, mtbbus_received_data[0]+1);
	size_t xorpos = mtbbus_received_data[0]+1;

	if ((mtbbus_received_data[xorpos] != (crc & 0xFF)) || (mtbbus_received_data[xorpos+1] != ((crc>>8) & 0xFF)))
		return; // bad checksum

	if ((_inquiry_module == 0) || (mtbbus_received_data[1] != MTBBUS_CMD_MISO_ACK)) {
		_inquiry_module = 0;
		mtbbus_received_read = false;
		mtbbus_received();
	} else {
		_inquiry_response_ok(mtbbus_received_addr);
	}
}

void _message_timeout() {
	gpio_pin_toggle(pin_led_red);
}

void EXTI15_10_IRQHandler(void) {
	if (__HAL_GPIO_EXTI_GET_FLAG(pin_usart_mtb_rx.pin)) {
		gpio_pin_toggle(pin_debug_a);
		_response_counter = 0;
		_rx_interrupt_disable();
		HAL_GPIO_EXTI_IRQHandler(pin_usart_mtb_rx.pin);
	}
}

static inline void _rx_interrupt_enable() {
	gpio_pin_init(pin_usart_mtb_rx, GPIO_MODE_IT_FALLING, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH, true);
}

static inline void _rx_interrupt_disable() {
	gpio_pin_init(pin_usart_mtb_rx, GPIO_MODE_AF_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH, true);
}

void mtbbus_update_50us(void) {
	if (_response_counter > 0) {
		_response_counter--;
		if (_response_counter == 0) {
			_message_timeout();
			gpio_pin_toggle(pin_debug_b);
		}
	}
}

/* Higher-level MTBbus functions ---------------------------------------------*/

bool mtbbus_send(uint8_t addr, uint8_t command_code, uint8_t *data, size_t datalen) {
	if (!mtbbus_can_send())
		return false;

	if (command_code != MTBBUS_CMD_MOSI_MODULE_INQUIRY)
		_inquiry_module = 0;

	mtbbus_received_addr = addr;
	_sent_command_code = command_code;

	_out_buf[0] = 0x0100 + addr;
	_out_buf[1] = datalen+1;
	_out_buf[2] = command_code;
	for (size_t i = 0; i < datalen; i++)
		_out_buf[3+i] = data[i];
	size_t total_len = datalen+3;

	uint16_t crc = crc16modbus_bytes(0, _out_buf, total_len);
	_out_buf[total_len] = crc & 0xFF;
	_out_buf[total_len+1] = (crc >> 8) & 0xFF;
	total_len += 2;

	gpio_pin_write(pin_usart_mtb_dir, true);
	_rx_interrupt_enable();
	HAL_UART_Transmit_DMA(&_h_uart_mtbbus, (uint8_t*)_out_buf, total_len);

	return true;
}

void mtbbus_module_inquiry(uint8_t module_addr) {
	_inquiry_module = module_addr;
	mtbbus_send(module_addr, MTBBUS_CMD_MOSI_MODULE_INQUIRY, NULL, 0);
}

void _inquiry_response_ok(size_t addr) {
	_inquiry_module = 0;
}
