#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "comm_usb.h"
#include "bno055.h"
#include "led.h"
#include "controller.h"
#include "vl53l0x.h"
#include "dw1000.h"
#include "isl29501.h"
#include "ms5611.h"
#include "chprintf.h"
#include <stdio.h>
#include <string.h>
#include "math.h"
#include "hw_conf.h"
#include <inttypes.h>

//#define RUN

const I2CConfig i2ccfg = {
    OPMODE_I2C,
    100000,
    STD_DUTY_CYCLE,
};

static THD_WORKING_AREA(controller_update_wa, 1024);
static THD_FUNCTION(controller_update, arg) {
    (void)arg;

    chRegSetThreadName("Controller update");

    Controller *controller = new Controller();
    for (;;)
    {
        controller->SetThrottle(0.5);
        controller->Update();
        chThdSleepMilliseconds(1);
    }
}

int main(void) {
    halInit();
    chSysInit();

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    palSetPadMode(GPIOC, 0, PAL_MODE_INPUT);
    palSetPadMode(GPIOC, 8, PAL_MODE_INPUT);
    palSetPadMode(GPIOC, 1, PAL_MODE_OUTPUT_PUSHPULL |
                                    PAL_STM32_OSPEED_HIGHEST);
    palSetPad(GPIOC, 1);
    palSetPadMode(MOTOR_DRIVER_1_FWD_GPIO, MOTOR_DRIVER_1_FWD_PIN, PAL_MODE_ALTERNATE(MOTOR_DRIVER_1_AF) |
            PAL_STM32_OTYPE_PUSHPULL |
            PAL_STM32_OSPEED_MID1);
    palSetPadMode(MOTOR_DRIVER_1_REV_GPIO, MOTOR_DRIVER_1_REV_PIN, PAL_MODE_ALTERNATE(MOTOR_DRIVER_1_AF) |
            PAL_STM32_OTYPE_PUSHPULL |
            PAL_STM32_OSPEED_MID1);
    palSetPadMode(MOTOR_DRIVER_2_FWD_GPIO, MOTOR_DRIVER_2_FWD_PIN, PAL_MODE_ALTERNATE(MOTOR_DRIVER_2_AF) |
            PAL_STM32_OTYPE_PUSHPULL |
            PAL_STM32_OSPEED_MID1);
    palSetPadMode(MOTOR_DRIVER_2_REV_GPIO, MOTOR_DRIVER_2_REV_PIN, PAL_MODE_ALTERNATE(MOTOR_DRIVER_2_AF) |
            PAL_STM32_OTYPE_PUSHPULL |
            PAL_STM32_OSPEED_MID1);
    palSetPadMode(MOTOR_DRIVER_3_FWD_GPIO, MOTOR_DRIVER_3_FWD_PIN, PAL_MODE_ALTERNATE(MOTOR_DRIVER_3_AF) |
            PAL_STM32_OTYPE_PUSHPULL |
            PAL_STM32_OSPEED_MID1);
    palSetPadMode(MOTOR_DRIVER_3_REV_GPIO, MOTOR_DRIVER_3_REV_PIN, PAL_MODE_ALTERNATE(MOTOR_DRIVER_3_AF) |
            PAL_STM32_OTYPE_PUSHPULL |
            PAL_STM32_OSPEED_MID1);
    palSetPadMode(MOTOR_DRIVER_4_FWD_GPIO, MOTOR_DRIVER_4_FWD_PIN, PAL_MODE_ALTERNATE(MOTOR_DRIVER_4_AF) |
            PAL_STM32_OTYPE_PUSHPULL |
            PAL_STM32_OSPEED_MID1);
    palSetPadMode(MOTOR_DRIVER_4_REV_GPIO, MOTOR_DRIVER_4_REV_PIN, PAL_MODE_ALTERNATE(MOTOR_DRIVER_4_AF) |
            PAL_STM32_OTYPE_PUSHPULL |
            PAL_STM32_OSPEED_MID1);
    palClearPad(MOTOR_DRIVER_1_FWD_GPIO, MOTOR_DRIVER_1_FWD_PIN);
    palClearPad(MOTOR_DRIVER_1_REV_GPIO, MOTOR_DRIVER_1_REV_PIN);
    palClearPad(MOTOR_DRIVER_2_FWD_GPIO, MOTOR_DRIVER_2_FWD_PIN);
    palClearPad(MOTOR_DRIVER_2_REV_GPIO, MOTOR_DRIVER_2_REV_PIN);
    palClearPad(MOTOR_DRIVER_3_FWD_GPIO, MOTOR_DRIVER_3_FWD_PIN);
    palClearPad(MOTOR_DRIVER_3_REV_GPIO, MOTOR_DRIVER_3_REV_PIN);
    palClearPad(MOTOR_DRIVER_4_FWD_GPIO, MOTOR_DRIVER_4_FWD_PIN);
    palClearPad(MOTOR_DRIVER_4_REV_GPIO, MOTOR_DRIVER_4_REV_PIN);

    i2cStart(&I2C_DEV, &i2ccfg);
    palSetPadMode(SCL_GPIO, SCL_PIN, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
    palSetPadMode(SDA_GPIO, SDA_PIN, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
    LED *led = new LED();
#ifndef RUN
    BNO055 *imu = new BNO055();
#else
    chThdCreateStatic(controller_update_wa, sizeof(controller_update_wa), NORMALPRIO, controller_update, NULL);
#endif
    DW1000 *dw1000 = new DW1000();
    ISL29501 *tof = new ISL29501();
    VL53L0X *sensor = new VL53L0X(false);
    MS5611 *baro = new MS5611();
    comm_usb_serial_init();
    sensor->setTimeout(MS2ST(500));

    // lower the return signal rate limit (default is 0.25 MCPS)
    sensor->setSignalRateLimit(0.1);
    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    sensor->setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor->setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

    // increase timing budget to 200 ms
    sensor->setMeasurementTimingBudget(20000);

    int i = 0;
    bool buttonReleased = false;
    bool buttonPressed = false;
    for(;;)
    {
        if (!palReadPad(GPIOC, 0))
        {
            buttonReleased = true;
        }
        if (buttonReleased && palReadPad(GPIOC, 0))
        {
            buttonPressed = true;
        }
        if (buttonPressed && !palReadPad(GPIOC, 0))
        {
            palClearPad(GPIOC, 1);
            chThdSleepMilliseconds(1000);
            break;
        }
        led->SetAll(0xFFFFFF);
        uint16_t *bbuffer = led->GetBuffer();
        Eigen::Vector3d vector(0, 0, 0);
        Eigen::Vector3d vector_grav(0, 0, 0);
#ifndef RUN
        vector = imu->GetVector(VECTOR_EULER);
        vector_grav = imu->GetVector(VECTOR_GYROSCOPE);
        chprintf((BaseSequentialStream*)&SDU1, "power button: %d \n", palReadPad(GPIOC, 0));
        chprintf((BaseSequentialStream*)&SDU1, "imu: %f %f %f, grav: %f %f %f\n", vector(0), vector(1), vector(2), vector_grav(0), vector_grav(1), vector_grav(2));
#endif
        chprintf((BaseSequentialStream*)&SDU1, "range: %d\n", sensor->readRangeSingleMillimeters());
        chprintf((BaseSequentialStream*)&SDU1, "isl: %d\n", tof->GetAmbientLight());
        double realTemperature = baro->readTemperature();
        long realPressure = baro->readPressure();
        double realAltitude = baro->getAltitude(realPressure);
        double realTemperature2 = baro->readTemperature(true);
        long realPressure2 = baro->readPressure(true);
        double realAltitude2 = baro->getAltitude(realPressure2);
        chprintf((BaseSequentialStream*)&SDU1, "barometer: %f %ld %f %f %ld %f\n", realTemperature, realPressure, realAltitude, realTemperature2, realPressure2, realAltitude2);
        char buffer [100];
        uint32_t id = dw1000->GetDeviceID();
        snprintf(buffer, 100, "dw: %08x\n", id);
        //chprintf((BaseSequentialStream*)&SDU1, "%s", buffer);
        for (int j = 0; j < 648; j++)
        {
            //if (j % 3 == 0)
                //chprintf((BaseSequentialStream*)&SDU1, " | ", bbuffer[j]);
            //chprintf((BaseSequentialStream*)&SDU1, "%d ", bbuffer[j]);
        }
        //chprintf((BaseSequentialStream*)&SDU1, "\n");
        chThdSleepMilliseconds(1000);
    }
}
