#include "motion_control.h"

MotionControl::MotionControl(int joints_num):
angles_handler_(joints_num),
motors_handler_(joints_num),
mutex_(xSemaphoreCreateMutex())
{
    angles_handler_.startConversion();

    I2CSlaveHandler &i2c_handler = I2CSlaveHandler::getInstance();

    I2C_CMD_INIT(setTargetAngle, MotionControl, i2c_handler);
    I2C_CMD_INIT(setMotorSpeed, MotionControl, i2c_handler);
    I2C_CMD_INIT(getCurrentAngle, MotionControl, i2c_handler);
}

void MotionControl::vMotionControlTask(void *pvParameters)
{
    MotionControl controller(1);

    while(true)
    {

    }
}

void MotionControl::_i2c_getCurrentAngle(I2C_Args args)
{
    angle_t angle = 0;

    {
        MutexGuard guard(mutex_);

        angle = angles_handler_.getConversionResult(args.getByte<0>());
    }

    I2CSlaveHandler::getInstance().prepareDataToSend((const uint8_t *)&angle, sizeof(angle));
}

void MotionControl::_i2c_setTargetAngle(I2C_Args args)
{

}

void MotionControl::_i2c_setMotorSpeed(I2C_Args args)
{
    uint8_t motor_num = args.getByte<0>();
    uint16_t speed = args.getShort<1>();

    {
        MutexGuard guard(mutex_);

        motors_handler_.setMotorSpeed(speed, motor_num);
    }
}
