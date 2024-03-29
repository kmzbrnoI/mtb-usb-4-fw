#include <stdint.h>
#include "mtbbus.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_dma.h"
#include "gpio.h"
#include "crc16modbus.h"
#include "modules.h"
#include "leds.h"

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
const unsigned _uart_irq_prio = 8;
DMA_Channel_TypeDef* const _uart_tx_dma_channel = DMA1_Channel2;
DMA_Channel_TypeDef* const _uart_rx_dma_channel = DMA1_Channel3;

/* Buffers -------------------------------------------------------------------*/

volatile uint16_t _out_buf[MTBBUS_OUT_BUF_SIZE];
volatile uint16_t mtbbus_received_data[MTBBUS_IN_BUF_SIZE];
volatile bool _receiving_first = false;
volatile size_t _receiving = 0; // 0 = not receiving, 1..RECEIVING_UPDATE_TIMEOUT = receiving
volatile bool _sending = false;
volatile size_t _response_counter = 0;
volatile bool received = false;

#define RESPONSE_COUNTER_FULL 5 // 250 us
#define RECEIVING_UPDATE_TIMEOUT 1000 // 50 ms

/* Higher-level data structures ----------------------------------------------*/

volatile MtbBusRxFlags mtbbus_rx_flags = {.all=0};

volatile size_t mtbbus_addr = 0;
volatile size_t mtbbus_command_code = 0;
volatile size_t _inquiry_module = 0;

#define INQUIRY_NONEXIST_PER_CYCLE 10

/* Private function prototypes -----------------------------------------------*/

static inline void _inquiry_response_ok(size_t addr);
static inline void _inquiry_response_timeout(size_t addr);
static inline void _message_received();
static inline void _message_timeout();
static inline void _rx_interrupt_enable();
static inline void _rx_interrupt_disable();
static inline void _mtbbus_send_buf(size_t total_len);

/* Private code --------------------------------------------------------------*/

bool mtbbus_init(uint32_t speed) {
	__HAL_RCC_USART3_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	_h_uart_mtbbus.Instance = _uart;
	_h_uart_mtbbus.Init.BaudRate = speed;
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
	gpio_pin_init(pin_usart_mtb_rx, GPIO_MODE_IT_FALLING, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH, true);
	gpio_pin_init(pin_usart_mtb_dir, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, false);

	// Lower ext interrupt priority to same priority as others to avoid deadlocks
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, _uart_irq_prio, 0);

	return true;
}

bool mtbbus_change_speed(uint32_t speed) {
	HAL_UART_DeInit(&_h_uart_mtbbus);
	_h_uart_mtbbus.Init.BaudRate = speed;
	return (HAL_UART_Init(&_h_uart_mtbbus) == HAL_OK);
}

bool mtbbus_can_send(void) {
	return (!_sending) && (_receiving == 0) && (mtbbus_rx_flags.all == 0) && (_response_counter == 0) && (!received);
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

		HAL_UART_DMAStop(&_h_uart_mtbbus);
		_sending = false;

		if (mtbbus_addr != 0) {
			// read just a single byte in DMA mode (first byte contains number of bytes following)
			for (volatile size_t i = 0; i != 0x02; i++); // delay ~1 us
			_response_counter = RESPONSE_COUNTER_FULL;
			_receiving_first = true;
			HAL_UART_Receive_DMA(&_h_uart_mtbbus, (uint8_t*)mtbbus_received_data, 1);
		} else {
			// Broadcast → simulate module 0 sent ACK
			_rx_interrupt_disable();
			mtbbus_received_data[0] = 1;
			mtbbus_received_data[1] = MTBBUS_CMD_MISO_ACK;
			mtbbus_rx_flags.sep.received = true;
		}
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
			_receiving = 0;
			received = true;
		}
	}
}

void mtbbus_poll(void) {
	if (received) {
		_message_received();
		received = false;
	}
}

void _message_received() {
	uint16_t crc = crc16modbus_bytes(0, (uint16_t*)mtbbus_received_data, mtbbus_received_data[0]+1);
	size_t crcpos = mtbbus_received_data[0]+1;

	if ((mtbbus_received_data[crcpos] != (crc & 0xFF)) || (mtbbus_received_data[crcpos+1] != ((crc>>8) & 0xFF))) {
		mtbbus_bad_checksum();
		return;
	}

	if (_inquiry_module == 0) {
		mtbbus_rx_flags.sep.received = true;
	} else {
		_inquiry_response_ok(mtbbus_addr);
		if (mtbbus_received_data[1] != MTBBUS_CMD_MISO_ACK) {
			module_set_changed(mtbbus_addr, true);
			mtbbus_rx_flags.sep.received = true;
		}
	}
}

void _message_timeout() {
	HAL_UART_DMAStop(&_h_uart_mtbbus);
	_rx_interrupt_disable();
	_receiving_first = false;
	if (_inquiry_module == 0)
		mtbbus_rx_flags.sep.timeout_pc = true;
	else
		_inquiry_response_timeout(mtbbus_addr);
}

void EXTI15_10_IRQHandler(void) {
	if (__HAL_GPIO_EXTI_GET_FLAG(pin_usart_mtb_rx.pin)) {
		if (_response_counter > 0) {
			_receiving = RECEIVING_UPDATE_TIMEOUT;
			_response_counter = 0;
			_rx_interrupt_disable();
		}
		HAL_GPIO_EXTI_IRQHandler(pin_usart_mtb_rx.pin);
	}
}

void _rx_interrupt_enable() {
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void _rx_interrupt_disable() {
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
}

void mtbbus_update_50us(void) {
	if (_response_counter > 0) {
		_response_counter--;
		if (_response_counter == 0)
			_message_timeout();
	}
	// In device sends only part of message, do not block bus
	if (_receiving > 0)
		_receiving--;
}

/* Higher-level MTBbus functions ---------------------------------------------*/

bool mtbbus_send(uint8_t addr, uint8_t command_code, uint8_t *data, size_t datalen) {
	if (!mtbbus_can_send())
		return false;
	_sending = true;

	if (command_code != MTBBUS_CMD_MOSI_MODULE_INQUIRY)
		_inquiry_module = 0;

	mtbbus_addr = addr;
	mtbbus_command_code = command_code;

	_out_buf[0] = 0x0100 + addr;
	_out_buf[1] = datalen+1;
	_out_buf[2] = command_code;
	for (size_t i = 0; i < datalen; i++)
		_out_buf[3+i] = data[i];

	_mtbbus_send_buf(datalen+3);
	return true;
}

bool mtbbus_send_from_ring(volatile ring_buffer* buf) {
	if ((!mtbbus_can_send()) || (ring_length(buf) < 3))
		return false;
	_sending = true;

	uint8_t addr = ring_get_byte_begin(buf, 1);
	uint8_t command_code = ring_get_byte_begin(buf, 2);

	if (command_code != MTBBUS_CMD_MOSI_MODULE_INQUIRY)
		_inquiry_module = 0;

	mtbbus_addr = addr;
	mtbbus_command_code = command_code;

	uint8_t datalen = ring_get_byte_begin(buf, 0)-3;
	_out_buf[0] = 0x0100 + addr; // device address
	_out_buf[1] = datalen+1;
	_out_buf[2] = command_code;
	for (size_t i = 0; i < datalen; i++)
		_out_buf[3+i] = ring_get_byte_begin(buf, 3+i);

	_mtbbus_send_buf(datalen+3);
	return true;
}

void _mtbbus_send_buf(size_t total_len) {
	uint16_t crc = crc16modbus_bytes(0, (uint16_t*)_out_buf, total_len);
	_out_buf[total_len] = crc & 0xFF;
	_out_buf[total_len+1] = (crc >> 8) & 0xFF;
	total_len += 2;

	gpio_pin_write(pin_usart_mtb_dir, true);
	for (volatile size_t i = 0; i != 0x02; i++); // delay ~1 us
	_rx_interrupt_enable();
	HAL_UART_Transmit_DMA(&_h_uart_mtbbus, (uint8_t*)_out_buf, total_len);
}

void mtbbus_module_inquiry(uint8_t module_addr) {
	uint8_t data[1] = {0x02 | (module_changed(module_addr) ? 1 : 0)};
	_inquiry_module = module_addr;
	mtbbus_send(module_addr, MTBBUS_CMD_MOSI_MODULE_INQUIRY, data, 1);
}

void _inquiry_response_ok(size_t addr) {
	_inquiry_module = 0;
	module_reset_attempts(addr);
	if (!module_active(addr)) {
		mtbbus_rx_flags.sep.discovered = true;
		module_set_active(addr, true);
	}
}

void _inquiry_response_timeout(size_t addr) {
	_inquiry_module = 0;
	module_set_changed(addr, false);

	if (module_active(addr)) {
		led_activate(pin_led_red, 100, 100);
		mtbbus_rx_flags.sep.timeout_inquiry = true;
		if (module_dec_attempts(addr) == 0)
			module_set_active(addr, false);
	}
}

/* Modules polling -----------------------------------------------------------*/

void mtbbus_modules_inquiry(void) {
	static uint8_t _inquiry_exist_module = 0;
	static uint8_t _inquiry_nonexist_module = 0;
	static size_t _inquiry_nonexist_counter = 0;

	// increment counters
	if (_inquiry_exist_module == 0) {
		_inquiry_nonexist_module++;
		if (_inquiry_nonexist_module == 0)
			_inquiry_nonexist_module = 1;
		_inquiry_nonexist_counter++;

		if (_inquiry_nonexist_counter >= INQUIRY_NONEXIST_PER_CYCLE) {
			// move to next cycle of active modules requesting
			// if no module exists, continue with reqiesting nonexistent
			_inquiry_nonexist_counter = 0;
			_inquiry_exist_module = module_next_active_addr(0);
		}
	} else {
		_inquiry_exist_module = module_next_active_addr(_inquiry_exist_module);
		_inquiry_nonexist_counter = 0;
	}

	// inquiry module
	if (_inquiry_exist_module == 0) {
		mtbbus_module_inquiry(_inquiry_nonexist_module);
		// inquiry up to INQUIRY_NONEXIST_PER_CYCLE nonexisting modules
	} else {
		// inquiry next existing module
		mtbbus_module_inquiry(_inquiry_exist_module);
	}
}
