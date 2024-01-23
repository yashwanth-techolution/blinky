/* This file is for USS (Ultasonic Sensor) and its use I2c1 */

#include <inttypes.h>
#include <zephyr/device.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <zephyr/sys/printk.h>

#include "Includes/GPIO_Expander.h"
#include "Includes/I2C_Module.h"


int gpio_expander_init(void) 
{
  uint8_t buffer[2];          // Initialize a buffer for two bytes to write to the device

  // First, we must set the I/O direction to output.
  // Write 0x00 to all IODIRA and IODIRB registers.

  buffer[0] = IODIRA;
  buffer[1] = OUTPUT;
  i2c1SendRegister(buffer, 2, MCP230_I2C_ADDR);

  buffer[0] = IODIRB;
  buffer[1] = OUTPUT;
  i2c1SendRegister(buffer, 2, MCP230_I2C_ADDR);

  gpio_expander_all_gpios_off();
  return 0;
}

void gpio_expander_all_gpios_on(void) 
{
  uint8_t buffer[2];         // Initialize a buffer for two bytes to write to the device

  // This function sets all outputs on both chips to 0 (off, low state).

  buffer[0] = GPIOA;
  buffer[1] = ALL_ON;
  i2c1SendRegister(buffer, 2, MCP230_I2C_ADDR);

  buffer[0] = GPIOB;
  buffer[1] = ALL_ON;
  i2c1SendRegister(buffer, 2, MCP230_I2C_ADDR);
}

void gpio_expander_all_gpios_off(void) 
{
  uint8_t buffer[2];         // Initialize a buffer for two bytes to write to the device

  // This function sets all outputs on both chips to 0 (off, low state).

  buffer[0] = GPIOA;
  buffer[1] = ALL_OFF;
  i2c1SendRegister(buffer, 2, MCP230_I2C_ADDR);

  buffer[0] = GPIOB;
  buffer[1] = ALL_OFF;
  i2c1SendRegister(buffer, 2, MCP230_I2C_ADDR);
}

void gpio_expander_port_A_pin_set(int byte)
{
  int8_t ret;
  uint8_t read_buf;         // Initialize a buffer for two bytes to write to the device
  uint8_t buff = GPIOA;         // Initialize a buffer for two bytes to write to the device

  ret = i2c1ReadRegister(MCP230_I2C_ADDR, &buff, 1, &read_buf, 1);
  
  read_buf = read_buf | (1 << byte);

  uint8_t buffer[2];         // Initialize a buffer for two bytes to write to the device

  buffer[0] = GPIOA;
  buffer[1] = read_buf;
  i2c1SendRegister(buffer, 2, MCP230_I2C_ADDR);
}

void gpio_expander_port_B_pin_set(int byte)
{
  int8_t ret;
  uint8_t read_buf;         // Initialize a buffer for two bytes to write to the device
  uint8_t buff = GPIOB;         // Initialize a buffer for two bytes to write to the device

  ret = i2c1ReadRegister(MCP230_I2C_ADDR, &buff, 1, &read_buf, 1);
  
  read_buf = read_buf | (1 << byte);

  uint8_t buffer[2];         // Initialize a buffer for two bytes to write to the device

  buffer[0] = GPIOB;
  buffer[1] = read_buf;
  i2c1SendRegister(buffer, 2, MCP230_I2C_ADDR);
}

void gpio_expander_port_A_pin_clear(int byte)
{
  int8_t ret;
  uint8_t read_buf;         // Initialize a buffer for two bytes to write to the device
  uint8_t buff = GPIOA;         // Initialize a buffer for two bytes to write to the device

  ret = i2c1ReadRegister(MCP230_I2C_ADDR, &buff, 1, &read_buf, 1);

  read_buf = read_buf & ~(1 << byte);

  uint8_t buffer[2];         // Initialize a buffer for two bytes to write to the device

  buffer[0] = GPIOA;
  buffer[1] = read_buf;
  i2c1SendRegister(buffer, 2, MCP230_I2C_ADDR);
}

void gpio_expander_port_B_pin_clear(int byte)
{
  int8_t ret;
  uint8_t read_buf;         // Initialize a buffer for two bytes to write to the device
  uint8_t buff = GPIOB;         // Initialize a buffer for two bytes to write to the device

  ret = i2c1ReadRegister(MCP230_I2C_ADDR, &buff, 1, &read_buf, 1);

  read_buf = read_buf & ~(1 << byte);

  uint8_t buffer[2];         // Initialize a buffer for two bytes to write to the device

  buffer[0] = GPIOB;
  buffer[1] = read_buf;
  i2c1SendRegister(buffer, 2, MCP230_I2C_ADDR);
}

void gpio_expander_set_output(int byte1, int byte2) 
{
  uint8_t buffer[2];         // Initialize a buffer for two bytes to write to the device

  // This function sends bytes (received as arguments) 
  // to the chips' GPIOA, GPIOB registers.

  buffer[0] = GPIOA;
  buffer[1] = byte1;
  i2c1SendRegister(buffer, 2, MCP230_I2C_ADDR);

  buffer[0] = GPIOB;
  buffer[1] = byte2;
  i2c1SendRegister(buffer, 2, MCP230_I2C_ADDR);
}
