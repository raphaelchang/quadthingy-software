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
#include <Eigen/Dense>
#include "utils.h"

#define RUN

const I2CConfig i2ccfg = {
    OPMODE_I2C,
    100000,
    STD_DUTY_CYCLE,
};
static Controller *controller;
static systime_t period;
static uint8_t *rxbuf;
static uint32_t rxlen;

static THD_WORKING_AREA(led_update_wa, 256);
static THD_FUNCTION(led_update, arg) {
    (void)arg;

    chRegSetThreadName("LED update");

    LED *led = new LED();
    uint8_t rgb[3] = {0, 0, 255};
    uint8_t colorCycle = 2;
    uint8_t colorIndex = 0;
    for (;;)
    {
        if (colorIndex == 255)
        {
            colorIndex = 0;
            colorCycle = (colorCycle + 1) % 3;
        }
        rgb[colorCycle]--;
        rgb[(colorCycle + 1) % 3]++;
        colorIndex++;
        led->SetAll((uint32_t)(rgb[0] << 16) | (uint16_t)(rgb[1] << 8) | rgb[2]);
        chThdSleepMilliseconds(2);
    }
}

static THD_WORKING_AREA(controller_update_wa, 4096);
static THD_FUNCTION(controller_update, arg) {
    (void)arg;

    chRegSetThreadName("Controller update");

    double throttle = 0;
    static systime_t lastTime = chVTGetSystemTime();

    for (;;)
    {
        //if (throttle < 0.5)
        //{
            //throttle += 0.005;
        //}
        //controller->SetThrottle(throttle);
        period = chVTTimeElapsedSinceX(lastTime);
        lastTime = chVTGetSystemTime();
        controller->Update();
        chThdSleepMilliseconds(1);
    }
}

static THD_WORKING_AREA(dw_update_wa, 4096);
static THD_FUNCTION(dw_update, arg) {
    (void)arg;

    chRegSetThreadName("DW1000 update");
    DW1000 *dw1000 = new DW1000();

    for (;;)
    {
        // Configure reception
        dw1000_rx_conf_t rx_conf;
        rx_conf.is_delayed = 0;
        rx_conf.timeout = 0;//0xFFFF; // ~65 ms

        // Receive
        dw1000->ConfigureRX( &rx_conf );
        dw1000->Receive( DW_TRANCEIVE_SYNC );
        rxbuf = dw1000->GetRXBuffer(&rxlen);

        // Configure transmission
        dw1000_tx_conf_t tx_conf;
        tx_conf.data_len = 10;
        tx_conf.is_delayed = 0;
        dw1000->ConfigureTX( &tx_conf );

        // Transmit
        static uint8_t counter = 0;
        uint8_t  p_data[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
        p_data[0] = counter++;
        dw1000->Transmit( p_data, tx_conf.data_len, DW_TRANCEIVE_SYNC );
        chThdSleepMilliseconds(5);
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
    chThdCreateStatic(led_update_wa, sizeof(led_update_wa), NORMALPRIO, led_update, NULL);
#ifndef RUN
    BNO055 *imu = new BNO055();
#else
    controller = new Controller();
    chThdCreateStatic(controller_update_wa, sizeof(controller_update_wa), HIGHPRIO, controller_update, NULL);
    //MotorDriver* m_md1 = new MotorDriver(&MOTOR_DRIVER_1_PWM_DEV, MOTOR_DRIVER_1_FWD_CH, MOTOR_DRIVER_1_REV_CH);
    //MotorDriver* m_md2 = new MotorDriver(&MOTOR_DRIVER_2_PWM_DEV, MOTOR_DRIVER_2_FWD_CH, MOTOR_DRIVER_2_REV_CH);
    //MotorDriver* m_md3 = new MotorDriver(&MOTOR_DRIVER_3_PWM_DEV, MOTOR_DRIVER_3_FWD_CH, MOTOR_DRIVER_3_REV_CH);
    //MotorDriver* m_md4 = new MotorDriver(MOTOR_DRIVER_4_PWM_DEV, MOTOR_DRIVER_4_FWD_CH, MOTOR_DRIVER_4_REV_CH);
        //m_md1->Set(0.5);
        //m_md2->Set(-0.5);
        //m_md3->Set(0.5);
        //m_md4->Set(-0.5);
#endif
    //chThdCreateStatic(dw_update_wa, sizeof(dw_update_wa), HIGHPRIO, dw_update, NULL);
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

    uint8_t numAltSamples = 20;
    double altitudeAvgs[numAltSamples];
    double altitudeAvg = 0;
    uint8_t currAltInx = 0;
    double altZero;
    for (uint8_t i = 0; i < numAltSamples; i++)
    {
        long realPressure = baro->readPressure(true);
        double realAltitude = baro->getAltitude(realPressure);
        altitudeAvgs[i] = realAltitude;
    }
    for (uint8_t i = 0; i < numAltSamples; i++)
    {
        altitudeAvg += altitudeAvgs[i] / numAltSamples;
    }
    altZero = altitudeAvg;
    double throttle = 0;


    controller->Enable();
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
            //palClearPad(GPIOC, 1);
            //chThdSleepMilliseconds(1000);
            //break;
        }
        uint16_t range = sensor->readRangeSingleMillimeters();
        chprintf((BaseSequentialStream*)&SDU1, "range: %d\n", range);
        //chprintf((BaseSequentialStream*)&SDU1, "isl: %d\n", tof->GetAmbientLight());
        double realTemperature = baro->readTemperature(true);
        long realPressure = baro->readPressure(true);
        double realAltitude = baro->getAltitude(realPressure);
        altitudeAvg -= altitudeAvgs[currAltInx] / numAltSamples;
        altitudeAvgs[currAltInx] = realAltitude;
        currAltInx = (currAltInx + 1) % numAltSamples;
        altitudeAvg += realAltitude / numAltSamples;
        //double altError = 0.25 - (altitudeAvg - altZero);
        int16_t altError = 500 - (int16_t)range;
        throttle += altError * 0.001;
        if (throttle > 0.9)
            throttle = 0.9;
        if (throttle < 0.5)
            throttle = 0.5;
#ifdef RUN
        controller->SetThrottle(0.5);
#endif
        chprintf((BaseSequentialStream*)&SDU1, "throttle: %f\n", throttle);
        //chprintf((BaseSequentialStream*)&SDU1, "barometer: %f %f %f\n", realTemperature, altitudeAvg - altZero, throttle);
        Eigen::Vector3f vector(0, 0, 0);
        Eigen::Vector3f vector_grav(0, 0, 0);
        Eigen::Vector3f vector_acc(0, 0, 0);
#ifndef RUN
        vector = imu->GetVector(VECTOR_EULER);
        vector_grav = imu->GetVector(VECTOR_GYROSCOPE);
        vector_acc = imu->GetVector(VECTOR_ACCELEROMETER);
        uint8_t calib = imu->ReadAddress(BNO055_CALIB_STAT_ADDR);
        uint8_t calibdata[22];
        imu->GetSensorOffsets(calibdata);
        chprintf((BaseSequentialStream*)&SDU1, "imu: %d %f %f %f %f %f %f %f %f %f\n", calib, vector(0), vector(1), vector(2), vector_grav(0), vector_grav(1), vector_grav(2), vector_acc(0), vector_acc(1), vector_acc(2));
        for (uint8_t i = 0; i < 22; i++)
        {
            chprintf((BaseSequentialStream*)&SDU1, "0x%x, ", calibdata[i]);
        }
        chprintf((BaseSequentialStream*)&SDU1, "\n");
#else
        vector = controller->GetOrientation();
        vector_grav = controller->GetRotationRate();
        vector_acc = controller->m_imu->m_imu->GetVector(VECTOR_EULER);
        //uint8_t calib = controller->m_imu->m_imu->ReadAddress(BNO055_CALIB_STAT_ADDR);
        chprintf((BaseSequentialStream*)&SDU1, "imu: %f %f %f %f %f %f %f %f %f\n", vector(0), vector(1), vector(2), vector_grav(0), vector_grav(1), vector_grav(2), vector_acc(0), vector_acc(1), vector_acc(2));
        chprintf((BaseSequentialStream*)&SDU1, "%d\n", ST2US(period));
#endif
        //chprintf((BaseSequentialStream*)&SDU1, "dw: ");
        //for (uint8_t i = 0; i < rxlen; i++)
        //{
            //chprintf((BaseSequentialStream*)&SDU1, "%d ", rxbuf[i]);
        //}
        //chprintf((BaseSequentialStream*)&SDU1, "\n");
        chThdSleepMilliseconds(10);
    }
}
