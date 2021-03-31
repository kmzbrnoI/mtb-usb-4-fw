#include "main.h"
#include "usb_cdc_link.h"
#include "i2c.h"
#include "mtbbus.h"
#include "gpio.h"

UART_HandleTypeDef h_uart_debug;

/* Private function prototypes -----------------------------------------------*/

static void error_handler();
static void init(void);
static bool clock_init(void);
static bool debug_uart_init(void);

/* Private user code ---------------------------------------------------------*/

int main(void) {
	init();

	while (true) {
	}
}

void init(void) {
	if (!clock_init())
		error_handler();
	HAL_Init();

	gpio_init();

	gpio_pin_write(pinLedRed, true);
	gpio_pin_write(pinLedYellow, true);
	gpio_pin_write(pinLedGreen, true);
	gpio_pin_write(pinLedBlue, true);

	if (!mtbbus_init())
		error_handler();
	if (!i2c_init())
		error_handler();
	cdcLinkInit();
	debug_uart_init();

	// HAL_Delay(1);

	gpio_pin_write(pinLedRed, false);
	gpio_pin_write(pinLedYellow, false);
	gpio_pin_write(pinLedGreen, false);
	gpio_pin_write(pinLedBlue, false);
}

bool clock_init(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
		return false;

	/** Initializes the CPU, AHB and APB buses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
		return false;

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
	return (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) == HAL_OK);

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
	return (HAL_UART_Init(&h_uart_debug) != HAL_OK);
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
