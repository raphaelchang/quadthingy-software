#include "motor_driver.h"

#define ESC_UPDATE_RATE		25000
#define TIM_CLOCK			10000000

static PWMConfig pwmcfg = {
    TIM_CLOCK,
    (uint16_t)((uint32_t)TIM_CLOCK / (uint32_t)ESC_UPDATE_RATE),
    NULL,
    {
        {PWM_OUTPUT_ACTIVE_HIGH, NULL},
        {PWM_OUTPUT_ACTIVE_HIGH, NULL},
        {PWM_OUTPUT_ACTIVE_HIGH, NULL},
        {PWM_OUTPUT_ACTIVE_HIGH, NULL}
    },
    0,
    0
};

MotorDriver(PWMDriver *device, uint8_t forward_channel, uint8_t reverse_channel)
{
    m_device = device;
    m_forward_channel = forward_channel;
    m_reverse_channel = reverse_channel;
    pwmStart(m_device, &pwmcfg);
    pwmEnableChannel(m_device, m_forward_channel, 0);
    pwmEnableChannel(m_device, m_reverse_channel, 0);
}

MotorDriver::~MotorDriver()
{
}

void MotorDriver::Set(double duty_cycle)
{
    uint32_t cnt_val;
    uint8_t sign = 1;
    if (duty_cycle < 0)
    {
        duty_cycle = -duty_cycle;
        sign = -1;
    }
    if (duty_cycle > 1)
        duty_cycle = 1;

    cnt_val = (uint32_t)(duty_cycle * (uint32_t)TIM_CLOCK / (uint32_t)ESC_UPDATE_RATE);
    if (sign == 1)
    {
        pwmEnableChannel(m_device, m_forward_channel, cnt_val);
        pwmEnableChannel(m_device, m_reverse_channel, 0);
    }
    else
    {
        pwmEnableChannel(m_device, m_forward_channel, 0);
        pwmEnableChannel(m_device, m_reverse_channel, cnt_val);
    }
}

