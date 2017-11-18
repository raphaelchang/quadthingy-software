#ifndef _HW_CONF_H_
#define _HW_CONF_H_

#include "hal.h"

#define I2C_DEV I2CD2
#define SCL_GPIO GPIOB
#define SCL_PIN 10
#define SDA_GPIO GPIOB
#define SDA_PIN 11

#define SCK_GPIO GPIOA
#define SCK_PIN 5
#define MISO_GPIO GPIOA
#define MISO_PIN 6
#define MOSI_GPIO GPIOA
#define MOSI_PIN 7
#define CS_GPIO GPIOC
#define CS_PIN 5
#define SPI_DEV SPID1
#define IRQ_GPIO GPIOC
#define IRQ_PIN 4

#endif /* _HW_CONF_H_ */
