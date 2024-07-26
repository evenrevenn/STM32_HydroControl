/*
 * i2c_handler.cpp
 *
 *  Created on: Nov 17, 2023
 *      Author: AleksandrA
 */

#include "i2c_slave.h"
#include "motion_control.h"
#include "stm32f1xx.h"
#include <cstdint>
#include <cstdio>

I2CSlaveHandler &I2CSlaveHandler::getInstance()
{
    static I2CSlaveHandler handler;

	return handler;
}

void I2CSlaveHandler::addCallback(I2C_CommandId command, std::function<void(uint32_t)> callback)
{
	if (callbacks_map_.find(command) != callbacks_map_.end()){
		print("Command %hhx already exist", command);

		return;
	}

	callbacks_map_[command] = callback;
}

void I2CSlaveHandler::transactionProcess()
{
	if (I2C2->SR2 & I2C_SR2_TRA){
		startWriteToBus();

		while((I2C2->SR1 & I2C_SR1_AF) == 0){
			uint8_t byte = 0;
			transmit_buffer_.readData(&byte, 1);

			if (!writeDataToBus()){
				I2C2->CR1 &= ~(I2C_CR1_ACK);
				I2C2->CR1 |= I2C_CR1_STOP;
				
				break;
			}
		}

		stopWritingToBus();
	}
	else{
		startReadFromBus();

		while((I2C2->SR1 & I2C_SR1_STOPF) == 0){
			if (!readDataFromBus()){
				I2C2->CR1 &= ~(I2C_CR1_ACK);
				I2C2->CR1 |= I2C_CR1_STOP;

				break;
			}
		}

		stopReadingFromBus();

		processCommand();
	}
}

void I2CSlaveHandler::processCommand()
{
	uint8_t command = 0;

	if (receive_buffer_.readData(&command, 1) == 0){
		return;
	}

	auto search = callbacks_map_.find(I2C_CommandId(command));
	if (search == callbacks_map_.end()){
		print("Received wrong command 0x%hx", command);

		return;
	}

	uint8_t args_num = (command >> 5) & 0x03;
	if (args_num == 3){
		args_num = 4;
	}
	
	uint32_t args = 0;
	if (receive_buffer_.readData((uint8_t *)&args, args_num) < args_num){
		return;
	}

	search->second(args);
	// if ((command & 0x80) != 0){
		// NVIC_ClearPendingIRQ(I2C2_EV_IRQn);
		// NVIC_EnableIRQ(I2C2_EV_IRQn);

		// search->second(args);
	// }
	// else{
		// search->second(args);

		// NVIC_ClearPendingIRQ(I2C2_EV_IRQn);
		// NVIC_EnableIRQ(I2C2_EV_IRQn);
	// }
}

void I2CSlaveHandler::vI2CSlaveTask(void *pvParameters)
{
	I2CSlaveHandler &i2c_handler = I2CSlaveHandler::getInstance();
	// TaskHandle_t processing_task_handle;

	// xTaskCreate(vCommandProcessingTask, "I2C processing", 200, NULL, 3, &processing_task_handle);

	while(true)
	{
		i2c_handler.takeSemaphore();
		// i2c_handler.transactionProcess();
		i2c_handler.processCommand();

		// I2C2->CR1 |= I2C_CR1_ACK;
	}
}

// void I2CSlaveHandler::vCommandProcessingTask(void *pvParameters)
// {
// 	const QueueHandle_t &functions_queue = I2CSlaveHandler::getInstance().functionsQueue();
// 	I2C_Function function;

// 	while(true)
// 	{
// 		if (xQueueReceive(functions_queue, &function, portMAX_DELAY)){
// 			function.function(function.args);
// 		}
// 	}
// }

void I2CSlaveHandler::_i2c_test_slave_receive4(I2C_Args args)
{
	print("Received: 0x%hX 0x%hX", args.getShort<0>(), args.getShort<2>());
}

void I2CSlaveHandler::_i2c_test_slave_receive2(I2C_Args args)
{
	print("Received: 0x%hX 0x%hX", args.getByte<0>(), args.getByte<1>());
}

void I2CSlaveHandler::_i2c_test_slave_receive1(I2C_Args args)
{
	print("Received: 0x%hX", args.getByte<0>());	
}

void I2CSlaveHandler::_i2c_test_slave_receive0(I2C_Args args)
{
	print("Received: zero args");
}

void I2CSlaveHandler::_i2c_test_slave_transmit4(I2C_Args args)
{
	volatile uint32_t longuint = args.getWord();
	prepareDataToSend((uint8_t *)&longuint, sizeof(longuint));
}

void I2CSlaveHandler::_i2c_test_slave_transmit2(I2C_Args args)
{
	volatile uint16_t shortuint = args.getShort<0>();
	prepareDataToSend((uint8_t *)&shortuint, sizeof(shortuint));
}

void I2CSlaveHandler::_i2c_test_slave_transmit1(I2C_Args args)
{
	volatile uint8_t byte = args.getByte<0>();
	prepareDataToSend((uint8_t *)&byte, sizeof(byte));
}

void I2CSlaveHandler::_i2c_test_slave_transmit0(I2C_Args args)
{
	volatile uint8_t byte = 0xFF;
	prepareDataToSend((uint8_t *)&byte, sizeof(byte));
}

void I2CSlaveHandler::startWriteToBus()
{
    uint32_t read = I2C2->SR1 | I2C2->SR2;
	(void) read;

	I2C2->CR1 |= I2C_CR1_ACK;
}

void I2CSlaveHandler::startReadFromBus()
{
    uint32_t read = I2C2->SR1 | I2C2->SR2;
	(void) read;

	I2C2->CR1 |= I2C_CR1_ACK;
}

#pragma GCC push_options
#pragma GCC optimize ("O0")

void I2CSlaveHandler::stopWritingToBus()
{
	// NVIC_EnableIRQ(I2C2_EV_IRQn);

	if (I2C2->SR1 & I2C_SR1_AF){
		I2C2->SR1 &= ~(I2C_SR1_AF);
	}

	if (I2C2->SR1 & I2C_SR1_STOPF){
		I2C2->CR1 &= ~(I2C_CR1_STOP);
	}
}

void I2CSlaveHandler::stopReadingFromBus()
{
	if (I2C2->SR1 & I2C_SR1_AF){
		I2C2->SR1 &= ~(I2C_SR1_AF);
	}

	if (I2C2->SR1 & I2C_SR1_STOPF){
		I2C2->CR1 &= ~(I2C_CR1_STOP);
	}
	
	// NVIC_ClearPendingIRQ(I2C2_EV_IRQn);
	// NVIC_EnableIRQ(I2C2_EV_IRQn);
}

#pragma GCC pop_options

bool I2CSlaveHandler::writeDataToBus()
{
    // uint32_t count = 0;
	// while((I2C2->SR1 & I2C_SR1_TXE) == 0){
	// 	vTaskDelay(pdMS_TO_TICKS(1));
	// 	count++;
	// 	if (count > DEBUG_DELAY){
	// 		//I2C1->CR1 &= ~(I2C_CR1_SWRST);
	// 		logger.printLog("Failed write data in shift reg");
	// 		return false;
	// 	}
	// }
	
	uint8_t data_byte = 0;
	if (transmit_buffer_.readData(&data_byte, 1)){
		I2C2->DR = data_byte;

		return true;
	}
	else{
		return false;
	}
}

bool I2CSlaveHandler::readDataFromBus()
{
	// uint32_t count = 0;
	// while((I2C2->SR1 & I2C_SR1_RXNE) == 0){

	// 	if (I2C2->SR1 & I2C_SR1_STOPF){
	// 		return true;
	// 	}

	// 	vTaskDelay(pdMS_TO_TICKS(1));
	// 	count++;
	// 	if (count > DEBUG_DELAY){
	// 		//I2C2->CR1 &= ~(I2C_CR1_SWRST);
	// 		logger.printLog("Failed read data");
	// 		return false;
	// 	}
	// }
	const uint8_t data_reg = I2C2->DR;

    return receive_buffer_.writeData(&data_reg, 1);
}

I2CSlaveHandler::I2CSlaveHandler():
logger(UARTHandler::getInstance()),
receive_buffer_(64),
transmit_buffer_(64)
{
	initI2CSlave();

	I2C_CMD_INIT(test_slave_receive4, I2CSlaveHandler, (*this));
	I2C_CMD_INIT(test_slave_receive2, I2CSlaveHandler, (*this));
	I2C_CMD_INIT(test_slave_receive1, I2CSlaveHandler, (*this));
	I2C_CMD_INIT(test_slave_receive0, I2CSlaveHandler, (*this));
	I2C_CMD_INIT(test_slave_transmit4, I2CSlaveHandler, (*this));
	I2C_CMD_INIT(test_slave_transmit2, I2CSlaveHandler, (*this));
	I2C_CMD_INIT(test_slave_transmit1, I2CSlaveHandler, (*this));
	I2C_CMD_INIT(test_slave_transmit0, I2CSlaveHandler, (*this));

	i2c_semaphore_ = xSemaphoreCreateCounting(16, 0);
	// functions_queue_ = xQueueCreate(32, sizeof(I2C_Function));
}

void I2CSlaveHandler::initI2CSlave()
{
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

	/*-----------------------*/
	RCC->APB1RSTR |= RCC_APB1RSTR_I2C2RST;
	RCC->APB1RSTR &= ~(RCC_APB1RSTR_I2C2RST);
	GPIOB->CRH |= (GPIO_CRH_MODE10_Msk | GPIO_CRH_MODE11_Msk);

	I2C2->CR1 &= ~(I2C_CR1_PE);

	GPIOB->CRH |= GPIO_CRH_CNF10_0;
	GPIOB->CRH &= ~(GPIO_CRH_CNF10_1);
	GPIOB->ODR &= ~(GPIO_ODR_ODR10);
	if ((GPIOB->IDR & GPIO_IDR_IDR10) == 0){
		logger.printLog("SCL low");
	}

	GPIOB->CRH |= GPIO_CRH_CNF11_0;
	GPIOB->CRH &= ~(GPIO_CRH_CNF11_1);
	GPIOB->ODR &= ~(GPIO_ODR_ODR11);
	if ((GPIOB->IDR & GPIO_IDR_IDR11) == 0){
		logger.printLog("SDA low");
	}

	GPIOB->ODR |= GPIO_ODR_ODR11;
	if (GPIOB->IDR & GPIO_IDR_IDR11){
		logger.printLog("SDA high");
	}
	GPIOB->ODR |= GPIO_ODR_ODR10;
	if (GPIOB->IDR & GPIO_IDR_IDR10){
		logger.printLog("SCL high");
	}

	GPIOB->CRH |= (GPIO_CRH_CNF10_Msk | GPIO_CRH_CNF11_Msk);


	I2C2->CR1 |= I2C_CR1_SWRST;
	I2C2->CR1 &= ~(I2C_CR1_SWRST);

	/*-----------------------*/

	/*GPIOB->CRL |= (GPIO_CRL_CNF6_Msk | GPIO_CRL_CNF7_Msk);
	GPIOB->CRL |= (GPIO_CRL_MODE6_Msk | GPIO_CRL_MODE7_Msk);
	GPIOB->ODR |= (GPIO_ODR_ODR6 | GPIO_ODR_ODR7);*/

	I2C2->CCR &= ~(I2C_CCR_FS);


	I2C2->CR2 |= 36UL;
	// I2C2->CCR &= ~(I2C_CCR_CCR_Msk);
	// I2C2->CCR |= 180UL;
	// I2C2->TRISE = 37UL;

	I2C2->OAR1 = 0x5B << 1;

	I2C2->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN;

	I2C2->CR1 |= I2C_CR1_PE;

	I2C2->CR1 |= I2C_CR1_ACK;

	NVIC_EnableIRQ(I2C2_EV_IRQn);
	NVIC_EnableIRQ(I2C2_ER_IRQn);

	logger.printLog("I2C Slave initialized");
}
