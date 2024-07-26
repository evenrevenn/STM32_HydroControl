/*
 * i2c_handler.h
 *
 *  Created on: Nov 17, 2023
 *      Author: AleksandrA
 */

#ifndef INC_I2C_SLAVE_H_
#define INC_I2C_SLAVE_H_

#include "i2c_utils.h"
#include "UART_handler.h"
#include "allocator.h"

#include <cstdint>
#include <functional>
#include <map>

#include "FreeRTOS.h"
#include "semphr.h"

RESERVE_I2C_ID(test_slave_receive4, 0, 3, false);
RESERVE_I2C_ID(test_slave_receive2, 0, 2, false);
RESERVE_I2C_ID(test_slave_receive1, 0, 1, false);
RESERVE_I2C_ID(test_slave_receive0, 0, 0, false);

RESERVE_I2C_ID(test_slave_transmit4, 0, 3, true);
RESERVE_I2C_ID(test_slave_transmit2, 0, 2, true);
RESERVE_I2C_ID(test_slave_transmit1, 0, 1, true);
RESERVE_I2C_ID(test_slave_transmit0, 0, 0, true);

class I2CSlaveHandler
{
public:
	static I2CSlaveHandler &getInstance();
	void addCallback(I2C_CommandId command, std::function<void(uint32_t)> callback);

	void transactionProcess();


	bool writeDataToBus();
	bool readDataFromBus();
	void processCommand();

	void prepareDataToSend(const uint8_t *data, size_t length) {transmit_buffer_.writeData(data, length);}

	static void vI2CSlaveTask(void *pvParameters);
	static void vCommandProcessingTask(void *pvParameters);
	void giveSemaphoreFromISR() {xSemaphoreGiveFromISR(i2c_semaphore_, NULL);}
	void takeSemaphore() {xSemaphoreTake(i2c_semaphore_, portMAX_DELAY);}

	// const QueueHandle_t & functionsQueue() const {return functions_queue_;}

	I2C_CMD_DECLARE(test_slave_receive4);
	I2C_CMD_DECLARE(test_slave_receive2);
	I2C_CMD_DECLARE(test_slave_receive1);
	I2C_CMD_DECLARE(test_slave_receive0);
	I2C_CMD_DECLARE(test_slave_transmit4);
	I2C_CMD_DECLARE(test_slave_transmit2);
	I2C_CMD_DECLARE(test_slave_transmit1);
	I2C_CMD_DECLARE(test_slave_transmit0);

private:
	void startWriteToBus();
	void startReadFromBus();
	void stopWritingToBus();
	void stopReadingFromBus();

	I2CSlaveHandler();
	void initI2CSlave();
	
	UARTHandler &logger;
	static const uint32_t DEBUG_DELAY = 5;

	CircularBuffer receive_buffer_;
	CircularBuffer transmit_buffer_;

	SemaphoreHandle_t i2c_semaphore_;

	I2C_CallbackMap callbacks_map_;
};

#endif //INC_I2C_SLAVE_H
