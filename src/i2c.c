#include "stm32f1xx_hal.h"
#include "i2c.h"
#include "gpio.h"

I2C_HandleTypeDef hi2c1;

bool i2c_init(void) {
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

	gpio_pin_init(pin_i2c_scl, GPIO_MODE_AF_OD, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
	gpio_pin_init(pin_i2c_sda, GPIO_MODE_AF_OD, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);

	__HAL_AFIO_REMAP_I2C1_ENABLE();
	__HAL_RCC_I2C1_CLK_ENABLE();

	return true;
}
