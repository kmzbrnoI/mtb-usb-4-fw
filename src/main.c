#include "main.h"
#include "usb_cdc_link.h"
#include "i2c.h"
#include "mtbbus.h"
#include "gpio.h"
#include "modules.h"

UART_HandleTypeDef h_uart_debug;
TIM_HandleTypeDef h_tim2;

/* Private function prototypes -----------------------------------------------*/

static void error_handler();
static void init(void);
static bool clock_init(void);
static bool debug_uart_init(void);
void usb_received(uint8_t command_code, uint8_t *data, size_t data_size);
void forward_mtbbus_received_to_usb();

/* Private code --------------------------------------------------------------*/

int main(void) {
	init();

	while (true) {
		if (!mtbbus_received_read) {
			if (cdc_main_can_send())
				forward_mtbbus_received_to_usb();
			else if (!cdc_dtr_ready) // computer does not listen → throw data away
				mtbbus_received_read = true;
		}
		if ((mtbbus_received_no_response) && (cdc_main_can_send())) {
			cdc_send_error(MTBUSB_ERROR_NO_RESPONSE, mtbbus_sent_command_code, mtbbus_sent_addr);
			mtbbus_received_no_response = false;
		}

		mtbbus_module_inquiry(1);

		HAL_Delay(500);
		gpio_pin_toggle(pin_led_yellow);
	}
}

void init(void) {
	if (!clock_init())
		error_handler();
	HAL_Init();

	gpio_init();

	gpio_pin_write(pin_led_red, true);
	gpio_pin_write(pin_led_yellow, true);
	gpio_pin_write(pin_led_green, true);
	gpio_pin_write(pin_led_blue, true);

	if (!mtbbus_init())
		error_handler();
	if (!i2c_init())
		error_handler();

	cdc_init();
	cdc_main_received = usb_received;

	debug_uart_init();
	modules_init();

	__HAL_AFIO_REMAP_SWJ_NOJTAG();

	HAL_Delay(100);

	gpio_pin_write(pin_led_red, false);
	gpio_pin_write(pin_led_yellow, false);
	gpio_pin_write(pin_led_green, false);
	gpio_pin_write(pin_led_blue, false);
}

bool clock_init(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	// HSE 8 MHz
	// SYSCLK 48 MHz
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
		return false;

	/** Initializes the CPU, AHB and APB buses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
		return false;

	// USB 48 MHz
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
		return false;

	__HAL_RCC_AFIO_CLK_ENABLE();
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_RCC_TIM2_CLK_ENABLE();

	// Timer 1 @ 50 us (MTBbus timeout)
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	h_tim2.Instance = TIM2;
	h_tim2.Init.Prescaler = 32;
	h_tim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	h_tim2.Init.Period = 73;
	h_tim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	h_tim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&h_tim2) != HAL_OK)
		return false;
	HAL_TIM_Base_Start_IT(&h_tim2);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&h_tim2, &sClockSourceConfig) != HAL_OK)
		return false;
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&h_tim2, &sMasterConfig) != HAL_OK)
		return false;

	HAL_NVIC_EnableIRQ(TIM2_IRQn);

	return true;
}

static bool debug_uart_init(void) {
	h_uart_debug.Instance = USART2;
	h_uart_debug.Init.BaudRate = 115200;
	h_uart_debug.Init.WordLength = UART_WORDLENGTH_8B;
	h_uart_debug.Init.StopBits = UART_STOPBITS_1;
	h_uart_debug.Init.Parity = UART_PARITY_NONE;
	h_uart_debug.Init.Mode = UART_MODE_TX_RX;
	h_uart_debug.Init.HwFlowCtl = UART_HWCONTROL_CTS;
	h_uart_debug.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&h_uart_debug) != HAL_OK)
		return false;

	__HAL_RCC_USART2_CLK_ENABLE();

	gpio_pin_init(pin_debug_cts, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, false);
	gpio_pin_init(pin_debug_rx, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, false);
	gpio_pin_init(pin_debug_tx, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, false);

	return true;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1)
		HAL_IncTick();
}

void error_handler(void) {
	__disable_irq();
	while (true);
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
}
#endif /* USE_FULL_ASSERT */

/* Interrupt handlers --------------------------------------------------------*/

// This function handles Non maskable interrupt.
void NMI_Handler(void) {
	while (true);
}

void HardFault_Handler(void) {
	while (true);
}

void MemManage_Handler(void) {
	while (true);
}

void BusFault_Handler(void) {
	while (true);
}

void UsageFault_Handler(void) {
	while (true);
}

void SVC_Handler(void) {}

void DebugMon_Handler(void) {}

// This function handles Pendable request for system service.
void PendSV_Handler(void) {}

void SysTick_Handler(void) {
	HAL_IncTick();
}

void TIM2_IRQHandler(void) {
	mtbbus_update_50us();
	HAL_TIM_IRQHandler(&h_tim2);
}

/* USB -----------------------------------------------------------------------*/

void usb_received(uint8_t command_code, uint8_t *data, size_t data_size) {
	cdc_send_ack();
}

/* MTBbus --------------------------------------------------------------------*/

void mtbbus_received() {
}

void forward_mtbbus_received_to_usb() {
	cdc_tx.separate.data[0] = mtbbus_sent_addr;
	for (size_t i = 1; i < mtbbus_received_data[0]+1; i++)
		cdc_tx.separate.data[i] = mtbbus_received_data[i] & 0xFF;
	cdc_main_send_nocopy(MTBUSB_CMD_MP_FORWARD, mtbbus_received_data[0]+1);
	mtbbus_received_read = true;
}
