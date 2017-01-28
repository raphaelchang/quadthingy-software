#include "motor_driver.h"
extern "C"
{
#include "stm32f4xx_conf.h"
}

#define ESC_UPDATE_RATE		25000
#define TIM_CLOCK			10000000
#define SYSTEM_CORE_CLOCK       168000000

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

MotorDriver::MotorDriver(PWMDriver *driver, uint8_t forward_channel, uint8_t reverse_channel)
{
    m_driver = driver;
    m_forward_channel = forward_channel;
    m_reverse_channel = reverse_channel;
    pwmStart(m_driver, &pwmcfg);
    pwmEnableChannel(m_driver, m_forward_channel, 0);
    pwmEnableChannel(m_driver, m_reverse_channel, 0);
    m_raw_driver = false;
}

MotorDriver::MotorDriver(uint8_t timerNum, uint8_t forward_channel, uint8_t reverse_channel)
{
    m_timer_num = timerNum;
    m_forward_channel = forward_channel;
    m_reverse_channel = reverse_channel;
    m_raw_driver = true;
    if (timerNum == 12)
    {
        TIM_DeInit(TIM12);
        TIM12->CNT = 0;
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);

        TIM_TimeBaseInitTypeDef timerInitStructure;
        timerInitStructure.TIM_Prescaler = 0;
        timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
        timerInitStructure.TIM_Period = SYSTEM_CORE_CLOCK / 2 / ESC_UPDATE_RATE;
        timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
        timerInitStructure.TIM_RepetitionCounter = 0;
        TIM_TimeBaseInit(TIM12, &timerInitStructure);
        TIM_Cmd(TIM12, ENABLE);

        TIM_OCInitTypeDef outputChannelInit;
        outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
        outputChannelInit.TIM_Pulse = 0;
        outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
        outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
        TIM_OC1Init(TIM12, &outputChannelInit);
        TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);
        TIM12->CCR1 = 0;
        TIM_OC2Init(TIM12, &outputChannelInit);
        TIM_OC2PreloadConfig(TIM12, TIM_OCPreload_Enable);
        TIM12->CCR2 = 0;
        TIM_CtrlPWMOutputs(TIM12, ENABLE);
    }
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
        if (!m_raw_driver)
        {
            pwmEnableChannel(m_driver, m_forward_channel, cnt_val);
            pwmEnableChannel(m_driver, m_reverse_channel, 0);
        }
        else
        {
            if (m_timer_num == 12)
            {
                if (m_forward_channel == 0)
                {
                    TIM12->CCR1 = (int)(duty_cycle * TIM12->ARR);
                    TIM12->CCR2 = 0;
                }
                else
                {
                    TIM12->CCR2 = (int)(duty_cycle * TIM12->ARR);
                    TIM12->CCR1 = 0;
                }
            }
        }
    }
    else
    {
        if (!m_raw_driver)
        {
            pwmEnableChannel(m_driver, m_forward_channel, 0);
            pwmEnableChannel(m_driver, m_reverse_channel, cnt_val);
        }
        else
        {
            if (m_timer_num == 12)
            {
                if (m_forward_channel == 0)
                {
                    TIM12->CCR2 = (int)(duty_cycle * TIM12->ARR);
                    TIM12->CCR1 = 0;
                }
                else
                {
                    TIM12->CCR1 = (int)(duty_cycle * TIM12->ARR);
                    TIM12->CCR2 = 0;
                }
            }
        }
    }
}

