#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_
extern "C"
{
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
}

class MotorDriver
{
public:
    MotorDriver(PWMDriver *device, uint8_t forward_channel, uint8_t reverse_channel);
    ~MotorDriver();
    void Set(double duty_cycle);
private:
    PWMDriver *m_driver;
    uint8_t m_forward_channel;
    uint8_t m_reverse_channel;
};

#endif /* MOTOR_DRIVER_H_ */
