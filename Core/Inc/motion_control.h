#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "i2c_slave.h"
#include "angle_detectors.h"
#include "motors_handler.h"
#include "allocator.h"
#include "FreeRTOS.h"

RESERVE_I2C_ID(getCurrentAngle, 1, 1, true)
RESERVE_I2C_ID(setMotorSpeed, 2, 2, false)
RESERVE_I2C_ID(setTargetAngle, 3, 2, false)

typedef uint16_t angle_t;

class MutexGuard
{
public:
    MutexGuard(SemaphoreHandle_t mutex) : mutex_(mutex) {xSemaphoreTake(mutex_, portMAX_DELAY);}
    ~MutexGuard(){xSemaphoreGive(mutex_);}

private:
    const SemaphoreHandle_t &mutex_;
};

class MotionControl
{
public:
    MotionControl(int joints_num);

    static void vMotionControlTask(void *pvParameters);

    I2C_CMD_DECLARE(getCurrentAngle);
    I2C_CMD_DECLARE(setMotorSpeed);

    I2C_CMD_DECLARE(setTargetAngle);

private:
    PotentiometersHandler angles_handler_;
    MotorsHandler motors_handler_;

    std::vector<angle_t, Heap4Alocator<angle_t>> target_angles_;
    SemaphoreHandle_t mutex_;
};

#endif //MOTION_CONTROL_H