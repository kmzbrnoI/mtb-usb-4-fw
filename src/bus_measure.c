#include "stm32f1xx_hal.h"
#include "bus_measure.h"
#include "gpio.h"
#include "adafruit_ina219.h"

I2C_HandleTypeDef hi2c1;

bool bus_measure_init(void) {
	__HAL_AFIO_REMAP_I2C1_ENABLE();
	__HAL_RCC_I2C1_CLK_ENABLE();

	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
		return false;

	gpio_pin_init(pin_i2c_scl, GPIO_MODE_AF_OD, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, false);
	gpio_pin_init(pin_i2c_sda, GPIO_MODE_AF_OD, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, false);

	NVIC_EnableIRQ(I2C1_EV_IRQn);
	ina219_setCalibration_16V_400mA();

	return true;
}
