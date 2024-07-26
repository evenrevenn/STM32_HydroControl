#include "i2c_master.h"
#include "i2c_utils.h"
#include "i2c_slave.h"
#include "motion_control.h"

#include "FreeRTOS.h"

I2CMasterHandler& I2CMasterHandler::getInstance()
{
	static I2CMasterHandler handler;

	// static bool is_first = true;
	// if (is_first){
	// 	handler.initI2CMaster();
	// 	is_first = false;
	// }

	return handler;
}

I2CMasterHandler::I2CMasterHandler():
logger(UARTHandler::getInstance())
{
	initI2CMaster();
}

I2CMasterHandler::~I2CMasterHandler()
{
}

void I2CMasterHandler::vI2CMasterTask(void *pvParameters)
{
	I2CMasterHandler &master = I2CMasterHandler::getInstance();

	vTaskDelay(pdMS_TO_TICKS(100));

	bool test_result = master.testSlaveReceiveI2C();
	if (!test_result){ master.stopWritingToBus(); }

	print(test_result ? "I2C Receive test: SUCCESS" : "Receive test: FAILED");
    vTaskDelay(pdMS_TO_TICKS(1000));

	test_result = master.testSlaveTransmitI2C();
	if (!test_result){ master.stopWritingToBus(); }

	print(test_result ? "I2C Transmit test: SUCCESS" : "Transmit test: FAILED");
	
	(void)test_result;
    vTaskDelay(pdMS_TO_TICKS(1000));

	while(true)
	{
		vTaskDelay(pdMS_TO_TICKS(500));

		bool writing = true;
		while(true){
			writing = true;
			if (!master.startWriteToBus(0x5B)){
				break;
			}
			if (!master.writeDataToBus(_i2c_getCurrentAngle_id::value)){
				break;
			}
			if (!master.writeDataToBus(1)){
				break;
			}
			writing = false;
			if (!master.startReadFromBus(0x5B)){
				break;
			}
			angle_t angle = 0;
			if (!master.readDataFromBus((uint8_t *)&angle, sizeof(angle))){
				break;
			}

			writing = true;
			if (!master.startWriteToBus(0x5B)){
				break;
			}
			if (!master.writeDataToBus(_i2c_setMotorSpeed_id::value)){
				break;
			}
			if (!master.writeDataToBus(1)){
				break;
			}
			angle >>= 3;
			if (!master.writeDataToBus(angle & 0xFF)){
				break;
			}
			if (!master.writeDataToBus(angle >> 8)){
				break;
			}
		}

		if (writing){
			master.stopWritingToBus();
		}
	}
}

bool I2CMasterHandler::testSlaveReceiveI2C()
{
	if (!startWriteToBus(0x5B)){ return false; }
	if (!writeDataToBus(_i2c_test_slave_receive0_id::value)){ return false; }
	stopWritingToBus();

	if (!startWriteToBus(0x5B)){ return false; }
	if (!writeDataToBus(_i2c_test_slave_receive1_id::value)){ return false; }
	if (!writeDataToBus(0xFA)){ return false; }
	stopWritingToBus();

	if (!startWriteToBus(0x5B)){ return false; }
	if (!writeDataToBus(_i2c_test_slave_receive2_id::value)){ return false; }
	if (!writeDataToBus(0xFF)){ return false; }
	if (!writeDataToBus(0xAA)){ return false; }
	stopWritingToBus();

	if (!startWriteToBus(0x5B)){ return false; }
	if (!writeDataToBus(_i2c_test_slave_receive4_id::value)){ return false; }
	if (!writeDataToBus(0xFF)){ return false; }
	if (!writeDataToBus(0xCC)){ return false; }
	if (!writeDataToBus(0xBB)){ return false; }
	if (!writeDataToBus(0xAA)){ return false; }
	stopWritingToBus();

	return true;
}

bool I2CMasterHandler::testSlaveTransmitI2C()
{
    if (!startWriteToBus(0x5B)){ return false; }
	if (!writeDataToBus(_i2c_test_slave_transmit0_id::value)){ return false; }
    if (!startReadFromBus(0x5B)){ return false; }
    {
	uint8_t byte = 0;
	if (!readDataFromBus(&byte, 1)){ return false; } 
	print("Transmitted: 0x%hX", byte);
	}
	
	if (!startWriteToBus(0x5B)){ return false; }
	if (!writeDataToBus(_i2c_test_slave_transmit1_id::value)){ return false; }
	if (!writeDataToBus(0xFA)){ return false; }
	if (!startReadFromBus(0x5B)){ return false; }
    {
	uint8_t byte = 0;
	if (!readDataFromBus(&byte, 1)){ return false; } 
	print("Transmitted: 0x%hX", byte);
	}

	if (!startWriteToBus(0x5B)){ return false; }
	if (!writeDataToBus(_i2c_test_slave_transmit2_id::value)){ return false; }
	if (!writeDataToBus(0xFF)){ return false; }
	if (!writeDataToBus(0xAA)){ return false; }
	if (!startReadFromBus(0x5B)){ return false; }
    {
	uint16_t shortint = 0;
	if (!readDataFromBus((uint8_t *)&shortint, 2)){ return false; } 
	print("Transmitted: 0x%hX", shortint);
	}

	if (!startWriteToBus(0x5B)){ return false; }
	if (!writeDataToBus(_i2c_test_slave_transmit4_id::value)){ return false; }
	if (!writeDataToBus(0xFF)){ return false; }
	if (!writeDataToBus(0xCC)){ return false; }
	if (!writeDataToBus(0xBB)){ return false; }
	if (!writeDataToBus(0xAA)){ return false; }
	{
	uint32_t longint = 0;
	if (!readDataFromBus((uint8_t *)&longint, 4)){ return false; } 
	print("Transmitted: 0x%X", longint);
	}

	return true;
}

void I2CMasterHandler::initI2CMaster()
{
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

	/*-----------------------*/
	RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
	RCC->APB1RSTR &= ~(RCC_APB1RSTR_I2C1RST);
	GPIOB->CRL |= (GPIO_CRL_MODE6_Msk | GPIO_CRL_MODE7_Msk);

	I2C1->CR1 &= ~(I2C_CR1_PE);

	GPIOB->CRL |= GPIO_CRL_CNF6_0;
	GPIOB->CRL &= ~(GPIO_CRL_CNF6_1);
	GPIOB->ODR &= ~(GPIO_ODR_ODR6);
	if ((GPIOB->IDR & GPIO_IDR_IDR6) == 0){
		logger.printLog("SCL low");
	}

	GPIOB->CRL |= GPIO_CRL_CNF7_0;
	GPIOB->CRL &= ~(GPIO_CRL_CNF7_1);
	GPIOB->ODR &= ~(GPIO_ODR_ODR7);
	if ((GPIOB->IDR & GPIO_IDR_IDR7) == 0){
		logger.printLog("SDA low");
	}

	GPIOB->ODR |= GPIO_ODR_ODR7;
	if (GPIOB->IDR & GPIO_IDR_IDR7){
		logger.printLog("SDA high");
	}
	GPIOB->ODR |= GPIO_ODR_ODR6;
	if (GPIOB->IDR & GPIO_IDR_IDR6){
		logger.printLog("SCL high");
	}

	GPIOB->CRL |= (GPIO_CRL_CNF6_Msk | GPIO_CRL_CNF7_Msk);


	I2C1->CR1 |= I2C_CR1_SWRST;
	I2C1->CR1 &= ~(I2C_CR1_SWRST);

	/*-----------------------*/

	/*GPIOB->CRL |= (GPIO_CRL_CNF6_Msk | GPIO_CRL_CNF7_Msk);
	GPIOB->CRL |= (GPIO_CRL_MODE6_Msk | GPIO_CRL_MODE7_Msk);
	GPIOB->ODR |= (GPIO_ODR_ODR6 | GPIO_ODR_ODR7);*/

	I2C1->CCR &= ~(I2C_CCR_FS);

	I2C1->CR2 |= 36UL;
	I2C1->CCR &= ~(I2C_CCR_CCR_Msk);
	I2C1->CCR |= 180UL;
	I2C1->TRISE = 37UL;

	I2C1->CR1 |= I2C_CR1_PE;

	logger.printLog("I2C Master initialized");
}

bool I2CMasterHandler::startWriteToBus(uint8_t slaveAddress)
{
	I2C1->CR1 |= I2C_CR1_ACK;

	I2C1->CR1 |= I2C_CR1_START;
	// if (I2C1->SR2 & I2C_SR2_BUSY){
	// 	logger.printLog("Bus is busy");
	// }
	volatile uint32_t count = 0;
	while((I2C1->SR1 & I2C_SR1_SB) == 0){
		// vTaskDelay(pdMS_TO_TICKS(1));
		count++;
		if (count > DEBUG_DELAY){
			//I2C1->CR1 &= ~(I2C_CR1_SWRST);
			logger.printLog("Failed generate start");
			return false;
		}
	}

	I2C1->DR = slaveAddress << 1;

	while((I2C1->SR1 & I2C_SR1_ADDR) == 0){
		// vTaskDelay(pdMS_TO_TICKS(1));
		count++;
		if (count > DEBUG_DELAY){
			//I2C1->CR1 &= ~(I2C_CR1_SWRST);
			logger.printLog("Failed confirm address");
			return false;
		}
	}

	uint32_t read = I2C1->SR1 | I2C1->SR2;
	(void) read;
	return true;
}

bool I2CMasterHandler::startReadFromBus(uint8_t slaveAddress)
{
	uint32_t count = 0;
	while((I2C1->SR1 & I2C_SR1_BTF) == 0){
		// vTaskDelay(pdMS_TO_TICKS(1));
		count++;
		if (count > DEBUG_DELAY){
			//I2C1->CR1 &= ~(I2C_CR1_SWRST);
			logger.printLog("Failed wait to stop");
			break;
		}
	}

	I2C1->CR1 |= I2C_CR1_ACK;

	I2C1->CR1 |= I2C_CR1_START;
	count = 0;
	while((I2C1->SR1 & I2C_SR1_SB) == 0){
		// vTaskDelay(pdMS_TO_TICKS(1));
		count++;
		if (count > DEBUG_DELAY){
			//I2C1->CR1 &= ~(I2C_CR1_SWRST);
			logger.printLog("Failed generate start");
			return false;
		}
	}
	I2C1->DR = (slaveAddress << 1) | (uint8_t) 1;
	return true;
}

void I2CMasterHandler::stopWritingToBus()
{
	uint32_t count = 0;
	while((I2C1->SR1 & I2C_SR1_BTF) == 0){
		// vTaskDelay(pdMS_TO_TICKS(1));
		count++;
		if (count > DEBUG_DELAY){
			//I2C1->CR1 &= ~(I2C_CR1_SWRST);
			logger.printLog("Failed wait to stop");
			break;
		}
	}
	I2C1->CR1 &= ~(I2C_CR1_ACK);
	I2C1->CR1 |= I2C_CR1_STOP;
}

bool I2CMasterHandler::writeDataToBus(uint8_t data)
{
	volatile uint32_t count = 0;
	while((I2C1->SR1 & I2C_SR1_TXE) == 0){
		// vTaskDelay(pdMS_TO_TICKS(1));
		count++;
		if (count > DEBUG_DELAY){
			//I2C1->CR1 &= ~(I2C_CR1_SWRST);
			logger.printLog("Failed write data in shift reg");
			return false;
		}
	}
	I2C1->DR = data;

	// count = 0;
	// while((I2C1->SR1 & I2C_SR1_BTF) == 0){
	// 	count++;
	// 	if (count > DEBUG_DELAY){
	// 		logger.printLog("Failed wait to stop");
	// 		return false;
	// 	}
	// }

	return true;
}

bool I2CMasterHandler::readDataFromBus(uint8_t *array, uint16_t bytes_num)
{
	uint32_t count = 0;
	while((I2C1->SR1 & I2C_SR1_ADDR) == 0){
		// vTaskDelay(pdMS_TO_TICKS(1));
		count++;
		if (count > DEBUG_DELAY){
			//I2C1->CR1 &= ~(I2C_CR1_SWRST);
			logger.printLog("Failed confirm address");
			return false;
		}
	}
	if (bytes_num == 1){
		I2C1->CR1 &= ~(I2C_CR1_ACK);
	}
	uint32_t read = I2C1->SR1 | I2C1->SR2;
	(void) read;

	uint16_t num = 0;
	for (uint16_t bytes_to_read = bytes_num; bytes_to_read > 0; bytes_to_read--)
	{
		//sprintf(bytesBuf, "Bytes to read %d", bytes_to_read);
		//logger.printLog(bytesBuf);

		if (bytes_to_read > 3){
			//logger.printLog("Reading multiple bytes");
			uint32_t count = 0;
			while((I2C1->SR1 & I2C_SR1_RXNE) == 0){
				// vTaskDelay(pdMS_TO_TICKS(1));
				count++;
				if (count > DEBUG_DELAY){
					//I2C1->CR1 &= ~(I2C_CR1_SWRST);
					logger.printLog("Failed to receive byte");
					return false;
				}
			}
			array[num] = I2C1->DR;
			num++;
			continue;
		}
		if (bytes_to_read == 3){
			//logger.printLog("Reading 3 bytes");
			while((I2C1->SR1 & I2C_SR1_BTF) == 0);
			I2C1->CR1 &= ~(I2C_CR1_ACK);
			array[num] = I2C1->DR;
			I2C1->CR1 |= I2C_CR1_STOP;
			array[num + 1] = I2C1->DR;
			while ((I2C1->SR1 & I2C_SR1_RXNE) == 0);
			array[num + 2] = I2C1->DR;
			return true;
		}
		if (bytes_to_read == 2){
			//logger.printLog("Reading 2 bytes");
			count = 0;
			while((I2C1->SR1 & I2C_SR1_RXNE) == 0){
				// vTaskDelay(pdMS_TO_TICKS(1));
				count++;
				if (count > DEBUG_DELAY){
					//I2C1->CR1 &= ~(I2C_CR1_SWRST);
					return false;
				}
			}
			I2C1->CR1 &= ~(I2C_CR1_ACK);
			uint32_t count = 0;
			while((I2C1->SR1 & I2C_SR1_BTF) == 0){
				// vTaskDelay(pdMS_TO_TICKS(1));
				count++;
				if (count > DEBUG_DELAY){
					//I2C1->CR1 &= ~(I2C_CR1_SWRST);
					return false;
				}
			}
			I2C1->CR1 |= I2C_CR1_STOP;
			array[num] = I2C1->DR;
			count = 0;
			while((I2C1->SR1 & I2C_SR1_RXNE) == 0){
				// vTaskDelay(pdMS_TO_TICKS(1));
				count++;
				if (count > DEBUG_DELAY){
					//I2C1->CR1 &= ~(I2C_CR1_SWRST);
					logger.printLog("Failed to read second byte");
					return false;
				}
			}
			array[num + 1] = I2C1->DR;
			return true;
		}
		if (bytes_to_read == 1){
			I2C1->CR1 |= I2C_CR1_STOP;
			//logger.printLog("Reading 1 byte");
			count = 0;
			while((I2C1->SR1 & I2C_SR1_RXNE) == 0){
				// vTaskDelay(pdMS_TO_TICKS(1));
				count++;
				if (count > DEBUG_DELAY){
					//I2C1->CR1 &= ~(I2C_CR1_SWRST);
					logger.printLog("Failed to read one byte");
					return false;
				}
			}
			array[num] = I2C1->DR;
			return true;
		}
	}
	logger.printLog("Failed to select bytes num");
	return false;
}