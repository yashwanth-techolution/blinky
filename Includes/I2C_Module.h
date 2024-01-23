#ifndef I2C_MOD_H_
#define I2C_MOD_H_

#include <inttypes.h>
#include<stdint.h>
#include<stdbool.h>

// Get I2C Device lable 
#define I2C_DEV DT_LABEL(DT_ALIAS(i2c_0))
#define I2C_1_DEV DT_LABEL(DT_ALIAS(i2c_1))


/**********************************************************************************************************
 * Function name  :   i2c_init()
 *
 * Description    :   1. This function gets the device binding of I2C peripheral from the device tree and 
 *                       initializes the same.
 *
 * Params         :   None
 *
 * Returns        :   Nothing
 *
 ***********************************************************************************************************/
void i2c_init();

void i2c_1_init();

int recover_i2c_bus();

/**********************************************************************************************************
 * Function name  :   i2cSendRegister()
 *
 * Description    :   This function will write to a provided register address over I2C                     
 *
 * Params         :   1. <in> const uint8_t *buf    : Starting address of buffer containing data to write
 *
 *                    2. <in> uint32_t num_bytes    : Number of bytes to be written.
 *
 *                    3. <in> uint16_t i2c_addr     : Address of the I2C slave 
 *
 * Returns        :   0 on success
 *
 ***********************************************************************************************************/
int i2cSendRegister(const uint8_t *buf, uint32_t num_bytes, uint16_t i2c_addr);     //for IMU & EEPROM
int i2c1SendRegister(const uint8_t *buf, uint32_t num_bytes, uint16_t i2c_addr);    //for USS

int i2cSendBurst(uint16_t i2c_addr, uint8_t write_buf, void *read_buf, size_t num_bytes);   //for IMU & EEPROM
int i2c1SendBurst(uint16_t i2c_addr, uint8_t write_buf, void *read_buf, size_t num_bytes);  //for USS
/**********************************************************************************************************
 * Function name  :   i2cReadRegister()
 *
 * Description    :   This function will read from a provided register address over I2C                     
 *
 * Params         :   1. <in> uint16_t i2c_addr    : Address of the I2C slave.
 *
 *                    2. <in> const void *write_buf: Register address of the I2C slave from where to read
 *
 *                    3. <in> size_t num_write     : Number of bytes in the I2C write command,
 *
 *                    4. <out> void *read_buf      : Buffer to write the data so read over I2C
 *
 *                    5. <out> size_t num_read     : Number of bytes read from I2C slave.
 *
 * Returns        :   0 on success
 *
 ***********************************************************************************************************/
int i2cReadRegister(uint16_t i2c_addr, const void *write_buf, size_t num_write, void *read_buf, size_t num_read);   //for IMU & EEPROM
int i2c1ReadRegister(uint16_t i2c_addr, const void *write_buf, size_t num_write, void *read_buf, size_t num_read);  //for USS

int i2cRead(uint16_t i2c_addr, void *read_buf, size_t num_read);      //for IMU & EEPROM
int i2c1Read(uint16_t i2c_addr, void *read_buf, uint32_t num_bytes);  //for USS

#endif
