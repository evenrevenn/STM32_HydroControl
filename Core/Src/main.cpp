/*
 * FreeRTOS V202212.01
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

/* Standard includes. */
#include <string.h>

#ifdef __cplusplus
extern "C"{
#define bool C_INT_BOOL
#endif

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Library includes. */
// #include "stm32f10x_it.h"


#ifdef __cplusplus
#undef bool
}
#endif

#include "stm32f1xx.h"

#include "UART_handler.h"
#include <i2c_slave.h>
#include <i2c_master.h>
#include "motion_control.h"

/*
 * Configure the hardware for the demo.
 */
void prvBlinkingTask(void *pvParameters);
static void prvSetupHardware(void);

static QueueHandle_t UART_Queue;

/*-----------------------------------------------------------*/

int main(void)
{
#ifdef DEBUG
  debug();
#endif

	/* Set up the clocks and memory interface. */
	prvSetupHardware();

	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	GPIOC->CRH &= ~(GPIO_CRH_CNF13);
	GPIOC->CRH |= GPIO_CRH_MODE13;

	xTaskCreate(prvBlinkingTask, "Blinking", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	UART_Queue = xQueueCreate(128, sizeof(char));
	xTaskCreate(UARTHandler::vUARTTask, "UARTTask", 200, &UART_Queue, 3, NULL);

	xTaskCreate(I2CMasterHandler::vI2CMasterTask, "I2CMasterTask", 200, NULL, 3, NULL);
	xTaskCreate(I2CSlaveHandler::vI2CSlaveTask, "I2CSlaveTask", 200, NULL, 4, NULL);
	// xTaskCreate(MotionControl::vMotionControlTask, "MotionControlTask", 200, NULL, 2, NULL);

    /* Start the scheduler. */
	vTaskStartScheduler();

    /* Will only get here if there was insufficient memory to create the idle
    task.  The idle task is created within vTaskStartScheduler(). */
	for( ;; );
}
/*-----------------------------------------------------------*/

void prvBlinkingTask(void *pvParameters)
{
	const TickType_t xDelay = pdMS_TO_TICKS(500);
	for (;;)
	{
		GPIOC->ODR ^= GPIO_ODR_ODR13;
		vTaskDelay(xDelay);
	}
}

static void prvSetupHardware(void)
{
	/* RCC system reset(for debug purpose). */
	/* Set HSION bit */
	RCC->CR |= RCC_CR_HSION;

	/* Reset SW[1:0], HPRE[3:0], PPRE1[2:0], PPRE2[2:0], ADCPRE[1:0] and MCO[2:0] bits */
	RCC->CFGR &= (uint32_t)0xF8FF0000;

	/* Reset HSEON, CSSON and PLLON bits */
	RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON);

	/* Reset HSEBYP bit */
	RCC->CR &= ~(RCC_CR_HSEBYP);

	/* Reset PLLSRC, PLLXTPRE, PLLMUL[3:0] and USBPRE bits */
	RCC->CFGR &= (uint32_t)0xFF80FFFF;

	/* Disable all interrupts */
	RCC->CIR = 0x00000000;

	/* Enable HSE. */
	RCC->CR |= RCC_CR_HSEON;

	/* Wait till HSE is ready. */
	while (!(RCC->CR & RCC_CR_HSERDY));

	/* HCLK = SYSCLK. */
	/* Clear HPRE[3:0] bits */
	RCC->CFGR &= ~(RCC_CFGR_HPRE);

	/* PCLK2  = HCLK. */
	RCC->CFGR &= ~(RCC_CFGR_PPRE2);

	/* PCLK1  = HCLK/2. */
	RCC->CFGR &= ~(RCC_CFGR_PPRE1);

	/* Set PPRE1[2:0] bits according to RCC_HCLK value */
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

	/* ADCCLK = PCLK2/4. */
	/* Clear ADCPRE[1:0] bits */
	RCC->CFGR &= ~(RCC_CFGR_ADCPRE);

	/* Set ADCPRE[1:0] bits according to RCC_PCLK2 value */
	RCC->CFGR |= RCC_CFGR_ADCPRE_DIV4;

	/* Flash 2 wait state. */
	*(volatile unsigned long *)0x40022000 = 0x01;

	/* PLLCLK = 8MHz * 9 = 72 MHz */
	/* Clear PLLSRC, PLLXTPRE and PLLMUL[3:0] bits */
	RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);

	/* Set the PLL configuration bits */
	RCC->CFGR |= RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9;

	/* Set ADC prescaler */
	RCC->CFGR &= ~(RCC_CFGR_ADCPRE);
	RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;

	/* Enable PLL. */
	RCC->CR |= RCC_CR_PLLON;

	/* Wait till PLL is ready. */
	while (!(RCC->CR & RCC_CR_PLLRDY));

	/* Select PLL as system clock source. */
	/* Clear SW[1:0] bits */
	RCC->CFGR &= RCC_CFGR_SW;

	/* Set SW[1:0] bits according to RCC_SYSCLKSource value */
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	/* Wait till PLL is used as system clock source. */
	while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);

	/* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE and AFIO clocks */
	RCC->APB2ENR |= (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPDEN | RCC_APB2ENR_IOPEEN | RCC_APB2ENR_AFIOEN);

	/* Set the Vector Table base address at 0x08000000. */
	SCB->VTOR = FLASH_BASE;

	// SCB->AIRCR = (uint32_t)0x05FA0000 | (uint32_t)0x300;

	/* Configure HCLK clock as SysTick clock source. */
	SysTick->CTRL |= (uint32_t)0x00000004;

	FLASH->ACR &= ~(FLASH_ACR_LATENCY);
	FLASH->ACR |= FLASH_ACR_LATENCY_2;


	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	GPIOB->CRH |= GPIO_CRH_CNF10_0;
	GPIOB->CRH &= ~(GPIO_CRH_CNF10_1);
	GPIOB->CRH |= (GPIO_CRH_MODE10_Msk | GPIO_CRH_MODE11_Msk);

	GPIOB->CRL |= GPIO_CRL_CNF6_0;
	GPIOB->CRL &= ~(GPIO_CRL_CNF6_1);
	GPIOB->CRL |= (GPIO_CRL_MODE6_Msk | GPIO_CRL_MODE7_Msk);

	GPIOB->ODR |= (GPIO_ODR_ODR6 | GPIO_ODR_ODR7 | GPIO_ODR_ODR10 | GPIO_ODR_ODR11);
}

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
	/* This function will get called if a task overflows its stack.   If the
	parameters are corrupt then inspect pxCurrentTCB to find which was the
	offending task. */

	(void)pxTask;
	(void)pcTaskName;

	for (;;);
}
/*-----------------------------------------------------------*/

void assert_failed(unsigned char *pucFile, unsigned long ulLine)
{
	(void)pucFile;
	(void)ulLine;

	for (;;);
}

/*-------------------------------------------------------*/
/* IRQ HANDLERS */

extern "C"{
void USART1_IRQHandler(void)
{
    char uart_buff = 0;

	if (USART1->SR & USART_SR_TXE)
	{

		if (xQueueReceiveFromISR(UART_Queue, &uart_buff, NULL) == pdTRUE){
			USART1->DR = uart_buff;
		}
		else{
			USART1->CR1 &= ~(USART_CR1_TXEIE);
		}
	}
	// if (USART1->SR & USART_SR_RXNE)
	// {
	// 	uart_buff = USART1->DR;
	// 	xQueueSendFromISR(manager.getUARTReceiveQueue(), &uart_buff, NULL);
	// }
}

void DMA1_Channel1_IRQHandler(void)
{
	if (DMA1->ISR & DMA_ISR_TEIF1){
		asm("nop");
	}
	else if (DMA1->ISR & DMA_ISR_TCIF1){
		asm("nop");
	}
}

#pragma GCC push_options
#pragma GCC optimize ("O0")

void I2C2_EV_IRQHandler(void)
{
	if (I2C2->SR1 & I2C_SR1_ADDR){
		if (I2C2->SR2 & I2C_SR2_TRA){
			I2CSlaveHandler::getInstance().processCommand();
		}
	}
	if (I2C2->SR1 & I2C_SR1_BTF){
		if (I2C2->SR2 & I2C_SR2_TRA){
			I2CSlaveHandler::getInstance().writeDataToBus();
			I2CSlaveHandler::getInstance().writeDataToBus();
		}
		else{
			I2CSlaveHandler::getInstance().readDataFromBus();
			I2CSlaveHandler::getInstance().readDataFromBus();
		}
	}
	if (I2C2->SR1 & I2C_SR1_RXNE){
		I2CSlaveHandler::getInstance().readDataFromBus();
	}
	if (I2C2->SR1 & I2C_SR1_TXE){
		I2CSlaveHandler::getInstance().writeDataToBus();
	}
	if (I2C2->SR1 & I2C_SR1_STOPF){
		I2C2->CR1 &= ~(I2C_CR1_STOP);
		
		I2CSlaveHandler::getInstance().giveSemaphoreFromISR();
	}
}

void I2C2_ER_IRQHandler(void)
{
	if (I2C2->SR1 & I2C_SR1_AF){
		I2C2->SR1 &= ~(I2C_SR1_AF);
	}
	if (I2C2->SR1 & I2C_SR1_BERR){
		I2C2->SR1 &= ~(I2C_SR1_BERR);
	}
}

#pragma GCC pop_options

}