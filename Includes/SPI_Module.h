#ifndef SPI_MODULE_H_
#define SPI_MODULE_H_

#include <stdint.h>
#include <zephyr/device.h>
#include <stdbool.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>

#define STM32_SPI_CS             10

extern struct device *spi_dev;
extern struct spi_config spi_cfg;

void spi_init(void);
uint8_t spi_transfer_w_bytes(int8_t *data, uint16_t len);
uint8_t spi_transfer_w(int8_t data);
uint8_t spi_transfer_r_bytes(int8_t *data, uint16_t len);
uint8_t spi_transfer_r(int8_t data);
int8_t stm_spi_send(uint8_t *out, uint16_t length);
int8_t stm_spi_read(uint8_t *in, uint16_t length);

#endif
