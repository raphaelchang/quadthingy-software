#include "isl29501.h"
#include "hw_conf.h"

ISL29501::ISL29501()
{
    systime_t tmo = MS2ST(4);
    uint8_t txbuf[2];
    uint8_t rxbuf[1];
    txbuf[0] = CMD_ADDR;
    txbuf[1] = 0xD7;
    i2cAcquireBus(&I2C_DEV);
    i2cMasterTransmitTimeout(&I2C_DEV, ISL_I2C_ADDR, txbuf, 2, rxbuf, 0, tmo);
    chThdSleepMilliseconds(250);
    txbuf[1] = 0x49;
    i2cMasterTransmitTimeout(&I2C_DEV, ISL_I2C_ADDR, txbuf, 2, rxbuf, 0, tmo);
    i2cReleaseBus(&I2C_DEV);
}

ISL29501::~ISL29501()
{
}

uint8_t ISL29501::GetDeviceID()
{
    systime_t tmo = MS2ST(4);
    uint8_t txbuf[1];
    uint8_t rxbuf[1];
    txbuf[0] = DEV_ID_ADDR;
    i2cAcquireBus(&I2C_DEV);
    uint8_t stat = i2cMasterTransmitTimeout(&I2C_DEV, ISL_I2C_ADDR, txbuf, 1, rxbuf, 1, tmo);
    i2cReleaseBus(&I2C_DEV);
    return rxbuf[0];
}

uint8_t ISL29501::GetStatus()
{
    systime_t tmo = MS2ST(4);
    uint8_t txbuf[1];
    uint8_t rxbuf[1];
    txbuf[0] = STATUS_ADDR;
    i2cAcquireBus(&I2C_DEV);
    uint8_t stat = i2cMasterTransmitTimeout(&I2C_DEV, ISL_I2C_ADDR, txbuf, 1, rxbuf, 1, tmo);
    i2cReleaseBus(&I2C_DEV);
    return rxbuf[0];
}

uint16_t ISL29501::GetDistance()
{
    systime_t tmo = MS2ST(4);
    uint8_t txbuf[1];
    uint8_t rxbuf[2];
    txbuf[0] = DIST_MSB_ADDR;
    i2cAcquireBus(&I2C_DEV);
    uint8_t stat = i2cMasterTransmitTimeout(&I2C_DEV, ISL_I2C_ADDR, txbuf, 1, rxbuf, 1, tmo);
    txbuf[0] = DIST_LSB_ADDR;
    stat = i2cMasterTransmitTimeout(&I2C_DEV, ISL_I2C_ADDR, txbuf, 1, rxbuf + 1, 1, tmo);
    i2cReleaseBus(&I2C_DEV);
    return (uint16_t)rxbuf[0] << 8 | rxbuf[1];
}

uint8_t ISL29501::GetAmbientLight()
{
    systime_t tmo = MS2ST(4);
    uint8_t txbuf[1];
    uint8_t rxbuf[1];
    txbuf[0] = AMBIENT_LIGHT_ADDR;
    i2cAcquireBus(&I2C_DEV);
    uint8_t stat = i2cMasterTransmitTimeout(&I2C_DEV, ISL_I2C_ADDR, txbuf, 1, rxbuf, 1, tmo);
    i2cReleaseBus(&I2C_DEV);
    return rxbuf[0];
}
