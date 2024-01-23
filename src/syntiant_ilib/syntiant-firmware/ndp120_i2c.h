/*
 * SYNTIANT CONFIDENTIAL
 * _____________________
 *
 *   Copyright (c) 2020 Syntiant Corporation
 *   All Rights Reserved.
 *
 *  NOTICE:  All information contained herein is, and remains the property of
 *  Syntiant Corporation and its suppliers, if any.  The intellectual and
 *  technical concepts contained herein are proprietary to Syntiant Corporation
 *  and its suppliers and may be covered by U.S. and Foreign Patents, patents in
 *  process, and are protected by trade secret or copyright law.  Dissemination
 *  of this information or reproduction of this material is strictly forbidden
 *  unless prior written permission is obtained from Syntiant Corporation.
 */

#ifndef _NDP120_I2C_H_
#define _NDP120_I2C_H_

/**
 * @brief I2C init
 */
void i2c_init(void);


/**
 * @brief read I2C
 *  
 * @param dev_addr i2c device address
 * @param buf
 * @param max_bytes
 */
int read_i2c(int dev_addr, uint8_t *buf, int max_bytes);

/**
 * @brief read I2C register
 *
 *  +--+--------+--+--+--------+--+* +--+---------+--+--+===============+--+--+ 
 *  |S |dev addr|RW|A |reg addr|A |* |S | dev addr|RW|A |data from slave|NA|P |
 *  |1b| 7 bits |1b|1b|8 bits  |1b|* |1b| 7 bits  |1b|1b|8 bits         |1b|1b|
 *  +--+--------+--+--+--------+--+* +--+---------+--+--+===============+--+--+ 
 *               0 (write)                         1 (read)            NACK STOP
 *  
 * @param dev_addr i2c device address
 * @param buf
 * @param max_bytes
 */
int read_i2c_register(
    int dev_addr,   /* 7 bits I2C device address */
    int reg_offset,
    int reg_byte_cnt,
    uint8_t *buf,   /* buffer */
    int max_bytes);


/**
 * @brief write I2C
 *
 * @param dev_addr i2c device address
 * @param buf
 * @param max_bytes 
 *  
 */
int write_i2c(int dev_addr, uint8_t *buf,  int max_bytes);

/**
 * I2C Write device register, value
 *
 *+--+--------+--+--+---------+--+----------------+--+--+ 
 *|S |dev addr|RW|A | reg addr|A | data to slave  |A |P |
 *|1b| 7 bits |1b|1b| 8 bits  |1b| 8 bits         |1b|1b|
 *+--+--------+--+--+---------+--+----------------+--+--+ 
 *             0 (write)                           ACK STOP
 * 
 * @param dev_addr i2c device address
 * @param reg_offset
 * @param reg_off_bytes_cnt
 * @param buf
 * @param write_bytes 
 */
int write_i2c_register(
    int dev_addr,   /* 7 bits I2C device address */
    int reg_offset,
    int reg_off_bytes_cnt,
    uint8_t *buf,   /* buffer */
    int write_bytes);

void enable_i2c(int pullup_mode );

void config_i2c(uint32_t freq, uint32_t clk_freq, int mode);
#endif
