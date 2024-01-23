#ifndef IMU_MODULE_H_
#define IMU_MODULE_H_

#include <stdint.h>
#include <stdbool.h>


// BMX160 i2c device addres.
#define BMX160_I2C_ADDR 0x68

// IMU Median array size
#define MEDIAN_ARRY_SIZE  5U

// Pi constant for YAW caluclation
#define PI 3.14159265



/**********************************************************************************************************
 * Thread name    :   IMU_READRaw()
 *
 * Description    :   This routines reads the raw imu value from I2C Sensor which is handled by I2C_Module.c
 *
 * Params         :   None
 *
 * Return         :   IMU Value in int8_t.
 ***********************************************************************************************************/
int8_t initIMU();


/**********************************************************************************************************
 * Thread name    :   IMU_READRaw()
 *
 * Description    :   This routines reads the raw imu value from I2C Sensor which is handled by I2C_Module.c
 *
 * Params         :   None
 *
 * Return         :   IMU Value in int.
 ***********************************************************************************************************/

int16_t IMU_READRaw(void);


/**********************************************************************************************************
 * Thread name    :   IMU_READMedian()
 *
 * Description    :   This routines returns the 'IMU Median value' out of imu values measured by IMU_READRaw()
 *
 * Params         :   None
 *
 * Return         :   IMU Value in int.
 ***********************************************************************************************************/
int16_t IMU_READMedian(void);

int16_t IMU_READMedian_2(void);
int calibrate_IMU();


#endif
