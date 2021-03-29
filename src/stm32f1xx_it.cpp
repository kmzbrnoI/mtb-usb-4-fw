/* Interrupt Service Routines. */
#include "main.h"
#include "stm32f1xx_it.h"

extern TIM_HandleTypeDef htim1;

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

void SysTick_Handler(void) {}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

void TIM1_UP_IRQHandler(void) {
	HAL_TIM_IRQHandler(&htim1);
}
