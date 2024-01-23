#include <zephyr/device.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <zephyr/sys/printk.h>

#include "Includes/EEPROM_Module.h"
#include "Includes/I2C_Module.h"

/*
 * brief: Wrapper function to write EEPROM Memory IC 
 */
int Write_EEPROM(uint16_t mem_location, uint8_t *data, uint32_t num) {
  int ret;
  uint8_t i2c_buff[2 + num];
  i2c_buff[0] = (mem_location >> 8) & 0xFF;
  i2c_buff[1] = mem_location & 0xFF;

  for(int i = 0; i < num; i++) {
    i2c_buff[2 + i] = *(data++);
  }

  ret = i2cSendRegister(&i2c_buff[0], (2 + num), EEPROM_I2C_ADDR);

  if (ret) {
    printk("Error sending to EEPROM, error code (%d)\n", ret);
  }
  return ret;
}


/*
 * brief: Wrapper function to read EEPROM Memory IC 
 */
int Read_EEPROM(uint16_t mem_location, void *read_buf, uint32_t num) {
  int ret;
  uint8_t magreg[2] = {(mem_location >> 8) & 0xFF, mem_location & 0xFF};
  ret = i2cReadRegister(EEPROM_I2C_ADDR, &magreg[0], 2, read_buf, num);

  if (ret) {
    printk("Error reading from EEPROM, error code (%d)\n", ret);
  }
  return ret;
}