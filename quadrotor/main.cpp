#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "comm_usb.h"
#include "bno055.h"
#include "controller.h"
#include "chprintf.h"
#include <stdio.h>
#include <string.h>
#include "math.h"

int main(void) {
    halInit();
    chSysInit();

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    palSetPadMode(GPIOC, 2, PAL_MODE_INPUT |
            PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(GPIOC, 1, PAL_MODE_INPUT |
            PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(GPIOB, 9, PAL_MODE_INPUT |
            PAL_STM32_OSPEED_HIGHEST);
    chThdSleepMilliseconds(1000);

    comm_usb_serial_init();
    uint32_t r = 0;
    uint32_t g = 85;
    uint32_t b = 170;
    bno055_init();
    for(;;)
    {
        chThdSleepMilliseconds(10);
    }
}
