#ifndef LED_H_
#define LED_H_

extern "C"
{
#include <stdint.h>
#include "ch.h"
#include "hal.h"
#include <math.h>
#include "stm32f4xx_conf.h"
}

// Hex color definitions
#define COLOR_BLACK          0x000000
#define COLOR_WHITE          0xFFFFFF

#define COLOR_BLUE           0x0000FF
#define COLOR_GREEN          0x00FF00
#define COLOR_RED            0xFF0000

#define COLOR_NAVY           0x000080
#define COLOR_DARKBLUE       0x00008B
#define COLOR_DARKGREEN      0x006400
#define COLOR_DARKCYAN       0x008B8B
#define COLOR_CYAN           0x00FFFF
#define COLOR_TURQUOISE      0x40E0D0
#define COLOR_INDIGO         0x4B0082
#define COLOR_DARKRED        0x800000
#define COLOR_OLIVE          0x808000
#define COLOR_GRAY           0x808080
#define COLOR_SKYBLUE        0x87CEEB
#define COLOR_BLUEVIOLET     0x8A2BE2
#define COLOR_LIGHTGREEN     0x90EE90
#define COLOR_DARKVIOLET     0x9400D3
#define COLOR_YELLOWGREEN    0x9ACD32
#define COLOR_BROWN          0xA52A2A
#define COLOR_DARKGRAY       0xA9A9A9
#define COLOR_SIENNA         0xA0522D
#define COLOR_LIGHTBLUE      0xADD8E6
#define COLOR_GREENYELLOW    0xADFF2F
#define COLOR_SILVER         0xC0C0C0
#define COLOR_LIGHTGREY      0xD3D3D3
#define COLOR_LIGHTCYAN      0xE0FFFF
#define COLOR_VIOLET         0xEE82EE
#define COLOR_AZUR           0xF0FFFF
#define COLOR_BEIGE          0xF5F5DC
#define COLOR_MAGENTA        0xFF00FF
#define COLOR_TOMATO         0xFF6347
#define COLOR_GOLD           0xFFD700
#define COLOR_ORANGE         0xFFA500
#define COLOR_SNOW           0xFFFAFA
#define COLOR_YELLOW         0xFFFF00

#define TLC5973_CLK_HZ 1200000
#define TLC5973_LED_NUM 4
#define TIM_PERIOD			(((168000000 / 2 / TLC5973_CLK_HZ) - 1))
#define LED_BUFFER_LEN		(TLC5973_LED_NUM)
#define BITBUFFER_LED_LEN       153
#define BITBUFFER_PAD		24
#define BITBUFFER_CMD_LEN       36
#define BITBUFFER_LEN		(BITBUFFER_LED_LEN * TLC5973_LED_NUM + BITBUFFER_PAD)
#define DUTY_CYCLE		(TIM_PERIOD * 0.4)
#define SET_ONE_AT_INDEX(buffer, index) \
    buffer[index] = DUTY_CYCLE; \
    buffer[index + 1] = DUTY_CYCLE; \
    buffer[index + 2] = 0;
#define SET_ZERO_AT_INDEX(buffer, index) \
    buffer[index] = DUTY_CYCLE; \
    buffer[index + 1] = 0; \
    buffer[index + 2] = 0;

class LED
{
public:
    LED();
    void SetColor(int led, uint32_t color);
    uint32_t GetColor(int led);
    void SetAllOff();
    void SetAll(uint32_t color);
    uint16_t* GetBuffer();
private:
    uint32_t rgbToLocal(uint32_t color);
    uint16_t m_bitbuffer[BITBUFFER_LEN];
    uint32_t m_rgb_data[LED_BUFFER_LEN];
    uint8_t m_gamma_table[256];
};

#endif /* LED_H_ */
