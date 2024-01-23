#include <inttypes.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <zephyr/sys/printk.h>

#include "Includes/I2C_Module.h"

/*
 * brief: Initialise the I2C Peripheral
 */
void i2c_init()
{  
  int ret;
  
  i2c_dev = device_get_binding(I2C_DEV);
  if (!i2c_dev) {
    printk("I2C: Device driver not found.\n");
    return;
  }

  ret = i2c_configure(i2c_dev, I2C_SPEED_FAST);
  //ret = i2c_configure(i2c_dev, I2C_SPEED_STANDARD);
  if (ret) {
    printk("I2C: not configured..............\n");
    return;
  }
}

/*
 * brief: Wrapper function to write to provided register address over I2C 
 */
uint8_t i2cSendRegister(const uint8_t *buf, uint32_t num_bytes, uint16_t i2c_addr)
{
    uint8_t stts;
    
  printk("********************************************8 %d %d\n",num_bytes,i2c_addr);
    stts = i2c_write(i2c_dev, buf, num_bytes, i2c_addr);
    
  printk("********************************************Send result   %d\n",stts);
    
    return stts;
}

/*
 * brief: Wrapper function to read from provided register address over I2C 
 */
uint8_t i2cReadRegister(uint16_t i2c_addr, const void *write_buf, size_t num_write, void *read_buf, size_t num_read)
{
    uint8_t stts;

    stts = i2c_write_read(i2c_dev, i2c_addr, write_buf, num_write, read_buf, num_read);
    
    return stts;
}


/*
 * brief: Wrapper function to read from provided register address over I2C 
 */
uint8_t i2cRead(uint16_t i2c_addr, void *read_buf, uint32_t num_bytes)
{

    uint8_t stts;
    uint8_t * read_buf_ptr = (uint8_t *) read_buf;

    printk("********************************************10 %d  %d\n",i2c_addr,num_bytes);
    stts = i2c_read(i2c_dev, read_buf, num_bytes,i2c_addr);
    
    printk("********************************************read result  %d\n",stts);
    for (uint8_t i = 0; i < num_bytes; i++)
    {
       printk("******************************************** %d    %d\n",i, read_buf_ptr[i]);
    }
    return stts;
}
