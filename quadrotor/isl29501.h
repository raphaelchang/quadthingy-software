#ifndef _ISL29501_H_
#define _ISL29501_H_

extern "C"
{
#include <stdint.h>
#include "ch.h"
#include "hal.h"
#include <string.h>
#include <math.h>
}

#define ISL_I2C_ADDR 0x54
#define DEV_ID_ADDR 0x00
#define STATUS_ADDR 0x02
#define DIST_MSB_ADDR 0xD1
#define DIST_LSB_ADDR 0xD2
#define AMBIENT_LIGHT_ADDR 0xE3
#define CMD_ADDR 0xB0

class ISL29501
{
public:
    ISL29501();
    ~ISL29501();
    uint8_t GetDeviceID();
    uint8_t GetStatus();
    uint16_t GetDistance();
    uint8_t GetAmbientLight();
};

#endif /* _ISL29501_H_ */
