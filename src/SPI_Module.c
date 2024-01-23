#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <errno.h>
#include <hal/nrf_gpio.h>
#include <inttypes.h>
#include <zephyr/kernel.h>
#include <stdio.h>
#include <stdlib.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "Includes/SPI_Module.h"

#define STM32_READ_CMD 0x40
#define STM32_WRITE_CMD 0x00
#define MAX_DATA_LENGTH 128

#define STM32_SPI_CS 10

struct device *spi_dev;

struct spi_config spi_cfg = {
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
    .frequency = 1000000,
};

//// SPI speed
// uint32_t spi_speed_general = 8000000;

//// Use slower SPI speed when writing audio samples @ 1MHz
//// Actual speed depends on frequency domain setup
//// not used in this example
// uint32_t spi_speed_spi_sample_reg_write = 1000000;

void spi_init(void) {
  const char *const spiName = "SPI_3";
  spi_dev = device_get_binding(spiName);

  if (spi_dev == NULL) {
    printk("Could not get %s device\n", spiName);
    return;
  }
}

uint8_t spi_transfer_w_bytes(int8_t *data, uint16_t len) {
  struct spi_buf tx_buf = {
      .buf = data,
      .len = len};

  struct spi_buf_set tx = {
      .buffers = &tx_buf,
      .count = 1};

  spi_transceive(spi_dev, &spi_cfg, &tx, NULL);
}

uint8_t spi_transfer_w(int8_t data) {
  uint8_t tx_buffer = data;

  struct spi_buf tx_buf = {
      .buf = &tx_buffer,
      .len = sizeof(tx_buffer)};

  struct spi_buf_set tx = {
      .buffers = &tx_buf,
      .count = 1};

  spi_transceive(spi_dev, &spi_cfg, &tx, NULL);
}

uint8_t spi_transfer_r_bytes(int8_t *data, uint16_t len) {
  struct spi_buf rx_buf = {
      .buf = data,
      .len = len};

  struct spi_buf_set rx = {
      .buffers = &rx_buf,
      .count = 1};

  spi_transceive(spi_dev, &spi_cfg, NULL, &rx);
}

uint8_t spi_transfer_r(int8_t data) {
  uint8_t rx_buffer;
  struct spi_buf rx_buf = {
      .buf = &rx_buffer,
      .len = sizeof(rx_buffer),
  };

  struct spi_buf_set rx = {
      .buffers = &rx_buf,
      .count = 1};

  spi_transceive(spi_dev, &spi_cfg, NULL, &rx);
  return rx_buffer;
}

int8_t stm_spi_read(uint8_t *in, uint16_t length) {
  int err;
  struct spi_buf rx_buf = {
      .buf = in,
      .len = length};

  struct spi_buf_set rx = {
      .buffers = &rx_buf,
      .count = 1};

  nrf_gpio_pin_clear(STM32_SPI_CS);
  err = spi_transceive(spi_dev, &spi_cfg, NULL, &rx);
  nrf_gpio_pin_set(STM32_SPI_CS);
  return err;
}

int8_t stm_spi_send(uint8_t *out, uint16_t length) {
  int err;
  struct spi_buf tx_buf1 = {
      .buf = out,
      .len = length};

  struct spi_buf_set tx1 = {
      .buffers = &tx_buf1,
      .count = 1};

  nrf_gpio_pin_clear(STM32_SPI_CS);
  err = spi_transceive(spi_dev, &spi_cfg, &tx1, NULL);
  nrf_gpio_pin_set(STM32_SPI_CS);
  return err;
}