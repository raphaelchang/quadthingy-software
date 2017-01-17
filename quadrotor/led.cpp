#include "led.h"

LED::LED() {
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    // Default LED values
    int i, bit;

    for (i = 0; i < LED_BUFFER_LEN; i++) {
        m_rgb_data[i] = 0;
    }
    for (i = 0; i < BITBUFFER_LEN; i++)
    {
        m_bitbuffer[i] = 0;
    }

    for (i = 0; i < LED_BUFFER_LEN; i++) {
        uint32_t writeCmd = 0x3AA;
        for (bit = 0; bit < 12; bit++)
        {
            if (writeCmd & (1 << (12 - bit - 1)))
            {
                SET_ONE_AT_INDEX(m_bitbuffer, bit * 3 + i * BITBUFFER_LED_LEN);
            }
            else
            {
                SET_ZERO_AT_INDEX(m_bitbuffer, bit * 3 + i * BITBUFFER_LED_LEN);
            }
        }
        uint32_t tmp_color = rgbToLocal(m_rgb_data[i]);
        uint8_t r = (tmp_color >> 16) & 0xFF;
        uint8_t g = (tmp_color >> 8) & 0xFF;
        uint8_t b = tmp_color & 0xFF;

        for (bit = 0; bit < 8; bit++) {
            if (g & (1 << (8 - bit - 1))) {
                SET_ONE_AT_INDEX(m_bitbuffer, BITBUFFER_CMD_LEN + 12 + bit * 3 + i * BITBUFFER_LED_LEN);
            } else {
                SET_ZERO_AT_INDEX(m_bitbuffer, BITBUFFER_CMD_LEN + 12 + bit * 3 + i * BITBUFFER_LED_LEN);
            }
        }
        for (bit = 0; bit < 8; bit++) {
            if (r & (1 << (8 - bit - 1))) {
                SET_ONE_AT_INDEX(m_bitbuffer, BITBUFFER_CMD_LEN + 48 + bit * 3 + i * BITBUFFER_LED_LEN);
            } else {
                SET_ZERO_AT_INDEX(m_bitbuffer, BITBUFFER_CMD_LEN + 48 + bit * 3 + i * BITBUFFER_LED_LEN);
            }
        }
        for (bit = 0; bit < 8; bit++) {
            if (b & (1 << (8 - bit - 1))) {
                SET_ONE_AT_INDEX(m_bitbuffer, BITBUFFER_CMD_LEN + 84 + bit * 3 + i * BITBUFFER_LED_LEN);
            } else {
                SET_ZERO_AT_INDEX(m_bitbuffer, BITBUFFER_CMD_LEN + 84 + bit * 3 + i * BITBUFFER_LED_LEN);
            }
        }
    }

    // Generate gamma correction table
    for (i = 0; i < 256; i++) {
        m_gamma_table[i] = (int)roundf(powf((float)i / 255.0, 1.0 / 0.45) * 255.0);
    }

    palSetPadMode(GPIOB, 8,
            PAL_MODE_ALTERNATE(GPIO_AF_TIM4) |
            PAL_STM32_OTYPE_PUSHPULL |
            PAL_STM32_OSPEED_MID1);

    // DMA clock enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    DMA_DeInit(DMA1_Stream7);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&TIM4->CCR3;

    DMA_InitStructure.DMA_Channel = DMA_Channel_2;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)m_bitbuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = BITBUFFER_LEN;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    DMA_Init(DMA1_Stream7, &DMA_InitStructure);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    // Time Base configuration
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = TIM_PERIOD;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    // Channel 3 Configuration in PWM mode
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = m_bitbuffer[0];
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC3Init(TIM4, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

    // TIM4 counter enable
    TIM_Cmd(TIM4, ENABLE);

    DMA_Cmd(DMA1_Stream7, ENABLE);

    // TIM4 Update DMA Request enable
    TIM_DMACmd(TIM4, TIM_DMA_CC3, ENABLE);

    // Main Output Enable
    TIM_CtrlPWMOutputs(TIM4, ENABLE);
}

void LED::SetColor(int led, uint32_t color) {
    if (led < TLC5973_LED_NUM) {
        m_rgb_data[led] = color;

        color = rgbToLocal(color);

        int bit;
        uint8_t r = (color >> 16) & 0xFF;
        uint8_t g = (color >> 8) & 0xFF;
        uint8_t b = color & 0xFF;

        for (bit = 0; bit < 8; bit++) {
            if (g & (1 << (8 - bit - 1))) {
                SET_ONE_AT_INDEX(m_bitbuffer, BITBUFFER_CMD_LEN + 12 + bit * 3 + led * BITBUFFER_LED_LEN);
            } else {
                SET_ZERO_AT_INDEX(m_bitbuffer, BITBUFFER_CMD_LEN + 12 + bit * 3 + led * BITBUFFER_LED_LEN);
            }
        }
        for (bit = 0; bit < 8; bit++) {
            if (r & (1 << (8 - bit - 1))) {
                SET_ONE_AT_INDEX(m_bitbuffer, BITBUFFER_CMD_LEN + 48 + bit * 3 + led * BITBUFFER_LED_LEN);
            } else {
                SET_ZERO_AT_INDEX(m_bitbuffer, BITBUFFER_CMD_LEN + 48 + bit * 3 + led * BITBUFFER_LED_LEN);
            }
        }
        for (bit = 0; bit < 8; bit++) {
            if (b & (1 << (8 - bit - 1))) {
                SET_ONE_AT_INDEX(m_bitbuffer, BITBUFFER_CMD_LEN + 84 + bit * 3 + led * BITBUFFER_LED_LEN);
            } else {
                SET_ZERO_AT_INDEX(m_bitbuffer, BITBUFFER_CMD_LEN + 84 + bit * 3 + led * BITBUFFER_LED_LEN);
            }
        }
    }
}

uint32_t LED::GetColor(int led) {
    if (led < TLC5973_LED_NUM) {
        return m_rgb_data[led];
    }

    return 0;
}

void LED::SetAllOff() {
    int i, bit;

    for (i = 0; i < TLC5973_LED_NUM; i++) {
        m_rgb_data[i] = 0;
    }

    for (i = 0; i < LED_BUFFER_LEN; i++) {
        for (bit = 0; bit < 8; bit++) {
            SET_ZERO_AT_INDEX(m_bitbuffer, BITBUFFER_CMD_LEN + 12 + bit * 3 + i * BITBUFFER_LED_LEN);
        }
        for (bit = 0; bit < 8; bit++) {
            SET_ZERO_AT_INDEX(m_bitbuffer, BITBUFFER_CMD_LEN + 48 + bit * 3 + i * BITBUFFER_LED_LEN);
        }
        for (bit = 0; bit < 8; bit++) {
            SET_ZERO_AT_INDEX(m_bitbuffer, BITBUFFER_CMD_LEN + 84 + bit * 3 + i * BITBUFFER_LED_LEN);
        }
    }
}

void LED::SetAll(uint32_t color) {
    int i, bit;

    for (i = 0; i < TLC5973_LED_NUM; i++) {
        m_rgb_data[i] = color;
    }
    for (i = 0; i < LED_BUFFER_LEN; i++) {
        uint32_t tmp_color = rgbToLocal(m_rgb_data[i]);
        uint8_t r = (tmp_color >> 16) & 0xFF;
        uint8_t g = (tmp_color >> 8) & 0xFF;
        uint8_t b = tmp_color & 0xFF;

        for (bit = 0; bit < 8; bit++) {
            if (g & (1 << (8 - bit - 1))) {
                SET_ONE_AT_INDEX(m_bitbuffer, BITBUFFER_CMD_LEN + 12 + bit * 3 + i * BITBUFFER_LED_LEN);
            } else {
                SET_ZERO_AT_INDEX(m_bitbuffer, BITBUFFER_CMD_LEN + 12 + bit * 3 + i * BITBUFFER_LED_LEN);
            }
        }
        for (bit = 0; bit < 8; bit++) {
            if (r & (1 << (8 - bit - 1))) {
                SET_ONE_AT_INDEX(m_bitbuffer, BITBUFFER_CMD_LEN + 48 + bit * 3 + i * BITBUFFER_LED_LEN);
            } else {
                SET_ZERO_AT_INDEX(m_bitbuffer, BITBUFFER_CMD_LEN + 48 + bit * 3 + i * BITBUFFER_LED_LEN);
            }
        }
        for (bit = 0; bit < 8; bit++) {
            if (b & (1 << (8 - bit - 1))) {
                SET_ONE_AT_INDEX(m_bitbuffer, BITBUFFER_CMD_LEN + 84 + bit * 3 + i * BITBUFFER_LED_LEN);
            } else {
                SET_ZERO_AT_INDEX(m_bitbuffer, BITBUFFER_CMD_LEN + 84 + bit * 3 + i * BITBUFFER_LED_LEN);
            }
        }
    }
}

uint32_t LED::rgbToLocal(uint32_t color) {
    uint32_t r = (color >> 16) & 0xFF;
    uint32_t g = (color >> 8) & 0xFF;
    uint32_t b = color & 0xFF;

    r = m_gamma_table[r];
    g = m_gamma_table[g];
    b = m_gamma_table[b];

    return (g << 16) | (r << 8) | b;
}
