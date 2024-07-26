#ifndef MOTOR_H
#define MOTOR_H

#include <cstdint>
#include <vector>
#include "allocator.h"

class Motor
{
public:
    friend class MotorsHandler;

private:
    Motor(int id);
    void setDuty(uint16_t duty_count);

    uint8_t id_;
};

class MotorsHandler
{
public:
    MotorsHandler(int motors_num);
    void setMotorSpeed(uint16_t speed, uint8_t id);

private:
    std::vector<Motor, Heap4Alocator<Motor>> motors_;
};

#endif //MOTOR_H