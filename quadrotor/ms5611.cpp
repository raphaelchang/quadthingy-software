#include "ms5611.h"
#include <math.h>

MS5611::MS5611(ms5611_osr_t osr)
{
    reset();

    setOversampling(osr);

    chThdSleepMilliseconds(100);

    readPROM();
}

MS5611::~MS5611()
{
}

// Set oversampling value
void MS5611::setOversampling(ms5611_osr_t osr)
{
    switch (osr)
    {
	case MS5611_ULTRA_LOW_POWER:
	    ct = 1;
	    break;
	case MS5611_LOW_POWER:
	    ct = 2;
	    break;
	case MS5611_STANDARD:
	    ct = 3;
	    break;
	case MS5611_HIGH_RES:
	    ct = 5;
	    break;
	case MS5611_ULTRA_HIGH_RES:
	    ct = 10;
	    break;
    }

    uosr = osr;
}

// Get oversampling value
ms5611_osr_t MS5611::getOversampling(void)
{
    return (ms5611_osr_t)uosr;
}

void MS5611::reset(void)
{
    systime_t tmo = MS2ST(4);
    uint8_t txbuf[1];
    uint8_t rxbuf[1];
    txbuf[0] = MS5611_CMD_RESET;
    i2cAcquireBus(&I2C_DEV);
    uint8_t stat = i2cMasterTransmitTimeout(&I2C_DEV, MS5611_ADDRESS, txbuf, 1, rxbuf, 0, tmo);
    i2cReleaseBus(&I2C_DEV);
}

void MS5611::readPROM(void)
{
    for (uint8_t offset = 0; offset < 6; offset++)
    {
	fc[offset] = readRegister16(MS5611_CMD_READ_PROM + (offset * 2));
    }
}

uint32_t MS5611::readRawTemperature(void)
{
    systime_t tmo = MS2ST(4);
    uint8_t txbuf[1];
    uint8_t rxbuf[1];
    txbuf[0] = MS5611_CMD_CONV_D2 + uosr;
    i2cAcquireBus(&I2C_DEV);
    uint8_t stat = i2cMasterTransmitTimeout(&I2C_DEV, MS5611_ADDRESS, txbuf, 1, rxbuf, 0, tmo);
    i2cReleaseBus(&I2C_DEV);

    chThdSleepMilliseconds(ct);

    return readRegister24(MS5611_CMD_ADC_READ);
}

uint32_t MS5611::readRawPressure(void)
{
    systime_t tmo = MS2ST(4);
    uint8_t txbuf[1];
    uint8_t rxbuf[1];
    txbuf[0] = MS5611_CMD_CONV_D1 + uosr;
    i2cAcquireBus(&I2C_DEV);
    uint8_t stat = i2cMasterTransmitTimeout(&I2C_DEV, MS5611_ADDRESS, txbuf, 1, rxbuf, 0, tmo);
    i2cReleaseBus(&I2C_DEV);

    chThdSleepMilliseconds(ct);

    return readRegister24(MS5611_CMD_ADC_READ);
}

int32_t MS5611::readPressure(bool compensation)
{
    uint32_t D1 = readRawPressure();

    uint32_t D2 = readRawTemperature();
    int32_t dT = D2 - (uint32_t)fc[4] * 256;

    int64_t OFF = (int64_t)fc[1] * 65536 + (int64_t)fc[3] * dT / 128;
    int64_t SENS = (int64_t)fc[0] * 32768 + (int64_t)fc[2] * dT / 256;

    if (compensation)
    {
	int32_t TEMP = 2000 + ((int64_t) dT * fc[5]) / 8388608;

	OFF2 = 0;
	SENS2 = 0;

	if (TEMP < 2000)
	{
	    OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
	    SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
	}

	if (TEMP < -1500)
	{
	    OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
	    SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
	}

	OFF = OFF - OFF2;
	SENS = SENS - SENS2;
    }

    uint32_t P = (D1 * SENS / 2097152 - OFF) / 32768;

    return P;
}

double MS5611::readTemperature(bool compensation)
{
    uint32_t D2 = readRawTemperature();
    int32_t dT = D2 - (uint32_t)fc[4] * 256;

    int32_t TEMP = 2000 + ((int64_t) dT * fc[5]) / 8388608;

    TEMP2 = 0;

    if (compensation)
    {
	if (TEMP < 2000)
	{
	    TEMP2 = (dT * dT) / (2 << 30);
	}
    }

    TEMP = TEMP - TEMP2;

    return ((double)TEMP/100);
}

// Calculate altitude from Pressure & Sea level pressure
double MS5611::getAltitude(double pressure, double seaLevelPressure)
{
    return (44330.0f * (1.0f - pow((double)pressure / (double)seaLevelPressure, 0.1902949f)));
}

// Calculate sea level from Pressure given on specific altitude
double MS5611::getSeaLevel(double pressure, double altitude)
{
    return ((double)pressure / pow(1.0f - ((double)altitude / 44330.0f), 5.255f));
}

// Read 16-bit from register (oops MSB, LSB)
uint16_t MS5611::readRegister16(uint8_t reg)
{
    systime_t tmo = MS2ST(4);
    uint8_t txbuf[1];
    uint8_t rxbuf[2];
    txbuf[0] = reg;
    i2cAcquireBus(&I2C_DEV);
    uint8_t stat = i2cMasterTransmitTimeout(&I2C_DEV, MS5611_ADDRESS, txbuf, 1, rxbuf, 2, tmo);
    i2cReleaseBus(&I2C_DEV);

    return (uint16_t)rxbuf[0] << 8 | rxbuf[1];
}

// Read 24-bit from register (oops XSB, MSB, LSB)
uint32_t MS5611::readRegister24(uint8_t reg)
{
    systime_t tmo = MS2ST(4);
    uint8_t txbuf[1];
    uint8_t rxbuf[3];
    txbuf[0] = reg;
    i2cAcquireBus(&I2C_DEV);
    uint8_t stat = i2cMasterTransmitTimeout(&I2C_DEV, MS5611_ADDRESS, txbuf, 1, rxbuf, 3, tmo);
    i2cReleaseBus(&I2C_DEV);

    return ((int32_t)rxbuf[0] << 16) | ((int32_t)rxbuf[1] << 8) | rxbuf[2];
}
