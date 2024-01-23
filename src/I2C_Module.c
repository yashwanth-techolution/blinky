#include <inttypes.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/posix/semaphore.h>

#include "Includes/I2C_Module.h"


// Create i2c device structure
const struct device *i2c_dev;   //for IMU and EEPROM
const struct device *i2c_1_dev; //for USS (Ultrasonic Sensor)

struct k_sem i2c_sem, i2c1_sem; //for both I2C

//k_sem_init(&my_sem, 0, 1);
K_SEM_DEFINE(i2c_sem, 0, 1);    //for IMU and EEPROM
K_SEM_DEFINE(i2c1_sem, 0, 1);   //for USS (Ultrasonic Sensor)

/*
 * brief: Initialise the I2C Peripheral
 */
void i2c_init() {   //for IMU and EEPROM
  int ret;
  
  k_sem_give(&i2c_sem);

  i2c_dev = device_get_binding(I2C_DEV);
  if (!i2c_dev) {
    printk("I2C: Device driver not found.\n");
    return;
  }

  ret = i2c_configure(i2c_dev, I2C_SPEED_FAST);
  if (ret) {
    printk("I2C: not configured\n");
    return;
  }
}


/*
 * brief: Initialise the I2C Peripheral
 */
void i2c_1_init() {   //for USS (Ultrasonic Sensor)
  int ret;
  
  k_sem_give(&i2c1_sem);

  i2c_1_dev = device_get_binding(I2C_1_DEV);
  if (!i2c_1_dev) {
    printk("I2C 1: Device driver not found.\n");
    return;
  }

  ret = i2c_configure(i2c_1_dev, I2C_SPEED_FAST);
  if (ret) {
    printk("I2C 1: not configured\n");
    return;
  }
}


int recover_i2c_bus() {   //for both I2C0 and I2C1
  int stts;

  stts = i2c_recover_bus(i2c_dev);
  
  if(stts) {
    printk("I2C Bus Recovery Error (%d)\n", stts);
  } else {
    printk("I2C Bus Recovered\n");
  }
  return stts;
}


/*
 * brief: Wrapper function to write to provided register address over I2C 
 */
int i2cSendRegister(const uint8_t *buf, uint32_t num_bytes, uint16_t i2c_addr) {    //for IMU and EEPROM
  int stts = 0; //Uttam made it 0 by default.

  if (k_sem_take(&i2c_sem, K_MSEC(50)) != 0) {
    printk("Semaphore not released Input data not available!\n");
  } else {
    // Debug print
    //printk("\nSent %d bytes to %X\n", num_bytes,i2c_addr);
    stts = i2c_write(i2c_dev, buf, num_bytes, i2c_addr);
  }

  if(stts) {
    printk("I2C Sending Error (%d)\n", stts);
  }

  k_sem_give(&i2c_sem);
  return stts;
}


/*
 * brief: Wrapper function to write to provided register address over I2C 
 */
int i2c1SendRegister(const uint8_t *buf, uint32_t num_bytes, uint16_t i2c_addr) {   //for USS (Ultrasonic Sensor)
  int stts = 0;

  if (k_sem_take(&i2c1_sem, K_MSEC(50)) != 0) {
    printk("Semaphore not released Input data not available!\n");
  } else {
    // Debug print
    //printk("\nSent %d bytes to %X\n", num_bytes,i2c_addr);
    stts = i2c_write(i2c_1_dev, buf, num_bytes, i2c_addr);
  }

  if(stts) {
    printk("I2C1 Sending Error (%d)\n", stts);
  }

  k_sem_give(&i2c1_sem);
  return stts;
}


int i2cSendBurst(uint16_t i2c_addr, uint8_t write_buf, void *read_buf, size_t num_bytes) {    //for IMU and EEPROM
  int stts = 0;

  if (k_sem_take(&i2c_sem, K_MSEC(50)) != 0) {
    printk("Semaphore not released Input data not available!\n");
  } else {
    stts = i2c_burst_write(i2c_dev, i2c_addr, write_buf, read_buf, num_bytes);
  }

  if(stts) {
    printk("I2C Sending Error (%d)\n", stts);
  }
  
  k_sem_give(&i2c_sem);
  return stts;
}


int i2c1SendBurst(uint16_t i2c_addr, uint8_t write_buf, void *read_buf, size_t num_bytes) {   //for USS (Ultrasonic Sensor)
  int stts = 0;

  if (k_sem_take(&i2c1_sem, K_MSEC(50)) != 0) {
    printk("Semaphore not released Input data not available!\n");
  } else {
    stts = i2c_burst_write(i2c_1_dev, i2c_addr, write_buf, read_buf, num_bytes);
  }

  if(stts) {
    printk("I2C1 Sending Error (%d)\n", stts);
  }

  k_sem_give(&i2c1_sem);
  return stts;
}


/*
 * brief: Wrapper function to read from provided register address over I2C 
 */
int i2cReadRegister(uint16_t i2c_addr, const void *write_buf, size_t num_write,     //for IMU and EEPROM
    void *read_buf, size_t num_read) {
  int stts = 0;

  if (k_sem_take(&i2c_sem, K_MSEC(50)) != 0) {
    printk("Semaphore not released Input data not available!\n");
  } else {
    stts = i2c_write_read(i2c_dev, i2c_addr, write_buf, num_write, read_buf, num_read);
  }

  if(stts) {
    printk("I2C Read Error (%d)\n", stts);
  }
  
  k_sem_give(&i2c_sem);
  return stts;
}

/*
 * brief: Wrapper function to read from provided register address over I2C 
 */
int i2c1ReadRegister(uint16_t i2c_addr, const void *write_buf, size_t num_write,    //for USS (Ultrasonic Sensor)
    void *read_buf, size_t num_read) {
  int stts = 0;

  if (k_sem_take(&i2c1_sem, K_MSEC(50)) != 0) {
    printk("Semaphore not released Input data not available!\n");
  } else {
    stts = i2c_write_read(i2c_1_dev, i2c_addr, write_buf, num_write, read_buf, num_read);
  }

  if(stts) {
    printk("I2C1 Read Error (%d)\n", stts);
  }
  
  k_sem_give(&i2c1_sem);
  return stts;
}


/*
 * brief: Wrapper function to read from provided register address over I2C 
 */
int i2cRead(uint16_t i2c_addr, void *read_buf, uint32_t num_bytes) {    //for IMU and EEPROM
  int stts = 0;
  //uint8_t * read_buf_ptr = (uint8_t *) read_buf;

  if (k_sem_take(&i2c_sem, K_MSEC(50)) != 0) {
    printk("Semaphore not released Input data not available!\n");
  } else {
    // Debug print
    //printk("\nReceived %d bytes from %X\n",num_bytes, i2c_addr);
    stts = i2c_read(i2c_dev, read_buf, num_bytes, i2c_addr);
  }

  if(stts) {
    printk("I2C Reading Error (%d)\n", stts);
  }
  
  k_sem_give(&i2c_sem);
  return stts;
}


int i2c1Read(uint16_t i2c_addr, void *read_buf, uint32_t num_bytes) {   //for USS (Ultrasonic Sensor)
  int stts = 0;
  //uint8_t * read_buf_ptr = (uint8_t *) read_buf;

  if (k_sem_take(&i2c1_sem, K_MSEC(50)) != 0) {
    printk("Semaphore not released Input data not available!\n");
  } else {
    // Debug print
    //printk("\nReceived %d bytes from %X\n",num_bytes, i2c_addr);
    stts = i2c_read(i2c_1_dev, read_buf, num_bytes, i2c_addr);
  }

  if(stts) {
    printk("I2C1 Reading Error (%d)\n", stts);
  }

  k_sem_give(&i2c1_sem);
  return stts;
}
