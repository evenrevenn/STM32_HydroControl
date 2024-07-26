/*
 * i2c_handler.h
 *
 *  Created on: Nov 17, 2023
 *      Author: AleksandrA
 */

#ifndef INC_I2C_MASTER_H_
#define INC_I2C_MASTER_H_

#include "UART_handler.h"

#include <cstdint>

class I2CMasterHandler
{
public:
	static I2CMasterHandler &getInstance();
	~I2CMasterHandler();

	static void vI2CMasterTask(void *pvParameters);
	bool testSlaveReceiveI2C();
	bool testSlaveTransmitI2C();

	bool startWriteToBus(uint8_t slaveAddress);
	bool startReadFromBus(uint8_t slaveAddress);
	void stopWritingToBus();
	bool writeDataToBus(uint8_t data);
	bool readDataFromBus(uint8_t *array, uint16_t bytes_num);

private:
	I2CMasterHandler();
	void initI2CMaster();
	
	UARTHandler &logger;
	static const uint32_t DEBUG_DELAY = 10000;
};

#endif //INC_I2C_MASTER_H
