//#include <device.h>
//#include <errno.h>
//#include <math.h>
//#include <stdio.h>
//#include <sys/printk.h>

//#include "Includes/IMU_Module.h"
////#include "Includes/BNO080.h"


//int16_t compass_cal_val[6] = {-191, 1545, -1703, 49, -337, -211}; // set-2 at sample door 12_08_2021

//float compass_scal_x = 1;
//float compass_scal_y = 1;
//float compass_scal_z = 1;
//int compass_offset_x = 0;
//int compass_offset_y = 0;
//int compass_offset_z = 0;


///*
// * brief: Initialise the IMU Sensor
// */


//int8_t initIMU()
//{
//  uint8_t reg1[2];
//  int ret;
//  //data and address of the register to be transmitted for initilalization
//  //BEGIN....
//  reg1[0] = 0x7E;
//  reg1[1] = 0x11;
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  k_msleep(50U);
//  reg1[0] = 0x7E;
//  reg1[1] = 0x15;
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  k_msleep(100U);
//  reg1[0] = 0x7E;
//  reg1[1] = 0x19;
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  k_msleep(10U);
//  // set magne config....
//  reg1[0] = 0x4C;
//  reg1[1] = 0x80;  // changed from 80 to 3c
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  k_msleep(10U);
//  // sleep mode
//  reg1[0] = 0x4F;
//  reg1[1] = 0x01;
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  reg1[0] = 0x4E;
//  reg1[1] = 0x4B;
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  // REPXY regular preset
//  reg1[0] = 0x4F;
//  reg1[1] = 0x17;
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  reg1[0] = 0x4E;
//  reg1[1] = 0x51;
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  // REPZ regular preset....
//  reg1[0] = 0x4F;
//  reg1[1] = 0x52;
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  reg1[0] = 0x4E;
//  reg1[1] = 0x52;
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  reg1[0] = 0x4F;
//  reg1[1] = 0x02;
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  reg1[0] = 0x4E;
//  reg1[1] = 0x4C;
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  reg1[0] = 0x4D;
//  reg1[1] = 0x42;
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  reg1[0] = 0x44;
//  reg1[1] = 0x05;
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  ////added for accel range
//  //reg1[0] = 0x41;
//  //reg1[1] = 0x0C;
//  //ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  reg1[0] = 0x4C;
//  reg1[1] = 0x00;
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  k_msleep(50U);
//  // set magne config....
//  reg1[0] = 0x03;
//  reg1[1] = 0x01;  // changed from 80 to 3c
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  k_msleep(10U);
//  //BEGIN....
//  reg1[0] = 0x7E;
//  reg1[1] = 0x11;
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  k_msleep(50U);
//  reg1[0] = 0x7E;
//  reg1[1] = 0x15;
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  k_msleep(100U);
//  reg1[0] = 0x7E;
//  reg1[1] = 0x19;
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  k_msleep(10U);
//  // set magne config....
//  reg1[0] = 0x4C;
//  reg1[1] = 0x80;  // changed from 80 to 3c
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  k_msleep(10U);
//  // sleep mode
//  reg1[0] = 0x4F;
//  reg1[1] = 0x01;
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  reg1[0] = 0x4E;
//  reg1[1] = 0x4B;
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  // REPXY regular preset
//  reg1[0] = 0x4F;
//  reg1[1] = 0x17;
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  reg1[0] = 0x4E;
//  reg1[1] = 0x51;
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  // REPZ regular preset....
//  reg1[0] = 0x4F;
//  reg1[1] = 0x52;
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  reg1[0] = 0x4E;
//  reg1[1] = 0x52;
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  reg1[0] = 0x4F;
//  reg1[1] = 0x02;
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  reg1[0] = 0x4E;
//  reg1[1] = 0x4C;
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  reg1[0] = 0x4D;
//  reg1[1] = 0x42;
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  reg1[0] = 0x44;
//  reg1[1] = 0x05;
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  ////added for accel range
//  //reg1[0] = 0x41;
//  //reg1[1] = 0x0C;
//  //ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  reg1[0] = 0x4C;
//  reg1[1] = 0x00;
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  k_msleep(50U);
//  // set magne config....
//  reg1[0] = 0x03;
//  reg1[1] = 0x01;  // changed from 80 to 3c
//  ret = i2cSendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
//  k_msleep(10U);

//  return ret;
//}



///*
//*  Fetching RAW IMU data from BMX160 Sensor.
//*  Returns RAW IMU data.
//*/
//int16_t IMU_READRaw(void) 
//{
//  uint8_t data[20] = {0};
//  // address bytes are used to read the data(accel,roll,yaw,pitch) from the particular location.
//   //these address locations are defined in the Datasheet

// uint8_t imureg[20] = {0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
//  int16_t mag_x, mag_y, mag_z;
//  int16_t acc_x, acc_y, acc_z;
//  int16_t gyr_x, gyr_y, gyr_z;
//  int rhall;
//  int orgyaw;
//  int ret, yaw;

//  double roll;
//  double pitch;
//  double azimuth;
//  double X_h, Y_h;
  
//  mag_x = 0;
//  mag_y = 0;
//  mag_z = 0;

//  for(int k = 0; k <= 10; k++)
//  {
//    // read BMX160 registers data
//    ret = i2cReadRegister(BMX160_I2C_ADDR, &imureg[0], 20, &data[0], 20);
//    if (ret) 
//    {
//      printk("Error reading from FRAM! error code (%d)\n", ret);
//    } 
//    else 
//    {
//      mag_x += (int16_t)((data[1] << 8) | data[0]);
//      mag_y += (int16_t)((data[3] << 8) | data[2]);
//      mag_z += (int16_t)((data[5] << 8) | data[4]);
//      rhall = (int16_t)((data[7] << 8) | data[6]);
//      gyr_x = (int16_t)((data[9] << 8) | data[8]);
//      gyr_y = (int16_t)((data[11] << 8) | data[10]);
//      gyr_z = (int16_t)((data[13] << 8) | data[12]);
//      acc_x = (int16_t)((data[15] << 8) | data[14]);
//      acc_y = (int16_t)((data[17] << 8) | data[16]);
//      acc_z = (int16_t)((data[19] << 8) | data[18]);
//    }
//    k_msleep(2U);
//  }
   
//  mag_x = mag_x/10;
//  mag_y = mag_y/10;
//  mag_z = mag_z/10;

//  //printk("mag x = %d , ", mag_x);
//  //printk("mag y = %d , ", mag_y);
//  //printk("mag z = %d , ", mag_z);

//  //printk("rhall = %d \n ", rhall);

//  //printk("gyr x = %d , ", gyr_x);
//  //printk("gyr y = %d , ", gyr_y);
//  //printk("gyr z = %d \n ", gyr_z);
  
//  //printk("acc x = %d , ", acc_x);
//  //printk("acc y = %d , ", acc_y);
//  //printk("acc z = %d \n ", acc_z);

//  /* Calculate pitch and roll, in the range (-pi,pi) */
//  pitch = atan2((double)-acc_x, sqrt((long)acc_z*(long)acc_z + (long)acc_y*(long)acc_y));
//  roll = atan2((double)acc_y, sqrt((long)acc_z*(long)acc_z  + (long)acc_x*(long)acc_x));


//  X_h = (double)mag_x*cos(pitch) + (double)mag_y*sin(roll)*sin(pitch) + (double)mag_z*cos(roll)*sin(pitch);
//  Y_h = (double)mag_y*cos(roll) - (double)mag_z*sin(roll);

//  // offset compansation
//  X_h = (compass_scal_x * X_h) + compass_offset_x;
//  Y_h = (compass_scal_y * Y_h) + compass_offset_y;

//  azimuth = (180 * atan2(-Y_h, X_h) / PI);
//  if(azimuth < 0)
//  {	/* Convert Azimuth in the range (0, 2pi) */
//    azimuth = 360 + azimuth;
//  }
//  else if(azimuth > 360)
//  {	/* Convert Azimuth in the range (0, 2pi) */
//    azimuth = azimuth - 360;
//  }
  
//  //printk(" IMU_round = %d \t", (int)roundf(azimuth));
//  //printf(" IMU_Float = %.1f \n", azimuth);
//  return (int)roundf(azimuth);
//}


//int calibrate_IMU()
//{
//  uint8_t data[20] = {0};
//  uint8_t imureg[20] = {0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
//  int16_t mag_x, mag_y, mag_z;
//  int16_t acc_x, acc_y, acc_z;
//  int16_t gyr_x, gyr_y, gyr_z;
//  int rhall;
//  int orgyaw;
//  int ret, yaw;
  
//  printk("Calibration sarted please rotate IMU sensor all directions.\n");

  
//#if 1
//  for(int k = 0; k <= 1; k++)
//  {
//    // read BMX160 registers data
//    ret = i2cReadRegister(BMX160_I2C_ADDR, &imureg[0], 20, &data[0], 20);
//    if (ret) 
//    {
//      printk("Error reading from FRAM! error code (%d)\n", ret);
//    }
//    else 
//    {
//      mag_x = (int16_t)((data[1] << 8) | data[0]);
//      mag_y = (int16_t)((data[3] << 8) | data[2]);
//      mag_z = (int16_t)((data[5] << 8) | data[4]);
//      rhall = (int16_t)((data[7] << 8) | data[6]);
//      gyr_x = (int16_t)((data[9] << 8) | data[8]);
//      gyr_y = (int16_t)((data[11] << 8) | data[10]);
//      gyr_z = (int16_t)((data[13] << 8) | data[12]);
//      acc_x = (int16_t)((data[15] << 8) | data[14]);
//      acc_y = (int16_t)((data[17] << 8) | data[16]);
//      acc_z = (int16_t)((data[19] << 8) | data[18]);
//    }

//    if(mag_x < compass_cal_val[0])
//    {
//      compass_cal_val[0] = mag_x;
//      //compass_cal_val[0] = mag_x;
//    }
//    if(mag_x > compass_cal_val[1])
//    {
//      compass_cal_val[1] = mag_x;
//      //compass_cal_val[1] = 1625;
//    }

//    if(mag_y < compass_cal_val[2])
//    {
//      compass_cal_val[2] = mag_y;
//      //compass_cal_val[2] = -1687;
//    }
//    if(mag_y > compass_cal_val[3])
//    {
//      compass_cal_val[3] = mag_y;
//      //compass_cal_val[3] = 233;
//    }

//    if(mag_z < compass_cal_val[4])
//    {
//      compass_cal_val[4] = mag_z;
//      //compass_cal_val[4] = -359;
//    }
//    if(mag_z > compass_cal_val[5])
//    {
//      compass_cal_val[5] = mag_z;
//      //compass_cal_val[5] = -237;
//    }
    
//    k_msleep(50U);
//    printk(".");
//  }

//#endif

//  printk("\n");
  
//  printk("Xmin = %d \n", compass_cal_val[0]);
//  printk("Xmax = %d \n", compass_cal_val[1]);
//  printk("Ymin = %d \n", compass_cal_val[2]);
//  printk("Ymax = %d \n", compass_cal_val[3]);
//  printk("Zmin = %d \n", compass_cal_val[4]);
//  printk("Zmax = %d \n", compass_cal_val[5]);

//  compass_scal_x = ((float)compass_cal_val[3] - compass_cal_val[2])/((float)compass_cal_val[1] - compass_cal_val[0]);
//  compass_scal_y = ((float)compass_cal_val[1] - compass_cal_val[0])/((float)compass_cal_val[3] - compass_cal_val[2]);
//  compass_scal_z = ((float)compass_cal_val[1] - compass_cal_val[0])/((float)compass_cal_val[5] - compass_cal_val[4]);
  
//  printk("before \n");
//  printf("Xsf = %f\n", compass_scal_x);
//  printf("Ysf = %f\n", compass_scal_y);
//  printf("Zsf = %f\n", compass_scal_z);

//  if(compass_scal_x < 1)
//  {
//    compass_scal_x = 1;
//  }
//  if(compass_scal_y < 1)
//  {
//    compass_scal_y = 1;
//  }
//  if(compass_scal_z < 1)
//  {
//    compass_scal_z = 1;
//  }
  
//  printk("after\n");
//  printf("Xsf = %f\n", compass_scal_x);
//  printf("Ysf = %f\n", compass_scal_y);
//  printf("Zsf = %f\n", compass_scal_z);

//  compass_offset_x = (((compass_cal_val[1] - compass_cal_val[0])/2) - compass_cal_val[1]) * compass_scal_x;
//  compass_offset_y = (((compass_cal_val[3] - compass_cal_val[2])/2) - compass_cal_val[3]) * compass_scal_y;
//  compass_offset_z = (((compass_cal_val[5] - compass_cal_val[4])/2) - compass_cal_val[5]) * compass_scal_z;
  
//  printk("Xoffset = %d\n", compass_offset_x);
//  printk("Yoffset = %d\n", compass_offset_y);
//  printk("Zoffset = %d\n", compass_offset_z);

//  k_msleep(2000U);

//  //printk("mag x = %d , ", mag_x);
//  //printk("mag y = %d , ", mag_y);
//  //printk("mag z = %d , ", mag_z);
//}


///*
// * brief: Wrapper function to calculate and return the MEDIAN value from set of IMU Values measured by IMU_READRaw() routine.
// */
//int16_t IMU_READMedian(void) 
//{
//  uint16_t yaw = IMU_READRaw();
//  //printk(" IMU = %d \n", yaw);
//  return yaw;
//}

///*
// * brief: Wrapper function to calculate and return the MEDIAN value from set of IMU Values measured by IMU_READRaw() routine.
// */
//int16_t IMU_READMedian_2(void)
//{
//  int16_t i, key, j;
//  int16_t a[MEDIAN_ARRY_SIZE];
//  for (i = 0; i < MEDIAN_ARRY_SIZE; i++) {
//    a[i] = IMU_READRaw();
//    k_msleep(10U);
//  }
////Finding median of an array: sorted array middle element will be the median.

//  for (i = 1; i < MEDIAN_ARRY_SIZE; i++) //sorting an array
//  {
//    key = a[i];
//    j = i - 1;
//    while (j >= 0 && a[j] > key) {
//      a[j + 1] = a[j];
//      j = j - 1;
//    }
//    a[j + 1] = key;
//  }
//  printk("IMU_Median: %d\n", a[(MEDIAN_ARRY_SIZE - 1) / 2]);
//  return a[(MEDIAN_ARRY_SIZE - 1) / 2];
//}










#include <zephyr/device.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <zephyr/sys/printk.h>

#include "Includes/IMU_Module.h"
#include "Includes/I2C_Module.h"


int16_t compass_cal_val[6] = {-191, 1545, -1703, 49, -337, -211}; // set-2 at sample door 12_08_2021

float compass_scal_x = 1;
float compass_scal_y = 1;
float compass_scal_z = 1;
int compass_offset_x = 0;
int compass_offset_y = 0;
int compass_offset_z = 0;


/*
 * brief: Initialise the IMU Sensor
 */


int8_t initIMU()
{
  uint8_t reg1[2];
  int ret;
  //data and address of the register to be transmitted for initilalization
  //BEGIN....
  reg1[0] = 0x7E;
  reg1[1] = 0x11;
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  k_msleep(50U);
  reg1[0] = 0x7E;
  reg1[1] = 0x15;
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  k_msleep(100U);
  reg1[0] = 0x7E;
  reg1[1] = 0x19;
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  k_msleep(10U);
  // set magne config....
  reg1[0] = 0x4C;
  reg1[1] = 0x80;  // changed from 80 to 3c
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  k_msleep(10U);
  // sleep mode
  reg1[0] = 0x4F;
  reg1[1] = 0x01;
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  reg1[0] = 0x4E;
  reg1[1] = 0x4B;
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  // REPXY regular preset
  reg1[0] = 0x4F;
  reg1[1] = 0x17;
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  reg1[0] = 0x4E;
  reg1[1] = 0x51;
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  // REPZ regular preset....
  reg1[0] = 0x4F;
  reg1[1] = 0x52;
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  reg1[0] = 0x4E;
  reg1[1] = 0x52;
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  reg1[0] = 0x4F;
  reg1[1] = 0x02;
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  reg1[0] = 0x4E;
  reg1[1] = 0x4C;
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  reg1[0] = 0x4D;
  reg1[1] = 0x42;
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  reg1[0] = 0x44;
  reg1[1] = 0x05;
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  ////added for accel range
  //reg1[0] = 0x41;
  //reg1[1] = 0x0C;
  //ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  reg1[0] = 0x4C;
  reg1[1] = 0x00;
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  k_msleep(50U);
  // set magne config....
  reg1[0] = 0x03;
  reg1[1] = 0x01;  // changed from 80 to 3c
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  k_msleep(10U);
  //BEGIN....
  reg1[0] = 0x7E;
  reg1[1] = 0x11;
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  k_msleep(50U);
  reg1[0] = 0x7E;
  reg1[1] = 0x15;
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  k_msleep(100U);
  reg1[0] = 0x7E;
  reg1[1] = 0x19;
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  k_msleep(10U);
  // set magne config....
  reg1[0] = 0x4C;
  reg1[1] = 0x80;  // changed from 80 to 3c
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  k_msleep(10U);
  // sleep mode
  reg1[0] = 0x4F;
  reg1[1] = 0x01;
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  reg1[0] = 0x4E;
  reg1[1] = 0x4B;
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  // REPXY regular preset
  reg1[0] = 0x4F;
  reg1[1] = 0x17;
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  reg1[0] = 0x4E;
  reg1[1] = 0x51;
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  // REPZ regular preset....
  reg1[0] = 0x4F;
  reg1[1] = 0x52;
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  reg1[0] = 0x4E;
  reg1[1] = 0x52;
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  reg1[0] = 0x4F;
  reg1[1] = 0x02;
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  reg1[0] = 0x4E;
  reg1[1] = 0x4C;
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  reg1[0] = 0x4D;
  reg1[1] = 0x42;
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  reg1[0] = 0x44;
  reg1[1] = 0x05;
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  ////added for accel range
  //reg1[0] = 0x41;
  //reg1[1] = 0x0C;
  //ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  reg1[0] = 0x4C;
  reg1[1] = 0x00;
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  k_msleep(50U);
  // set magne config....
  reg1[0] = 0x03;
  reg1[1] = 0x01;  // changed from 80 to 3c
  ret = i2c1SendRegister(&reg1[0], 2, BMX160_I2C_ADDR);
  k_msleep(10U);

  return ret;
}



/*
*  Fetching RAW IMU data from BMX160 Sensor.
*  Returns RAW IMU data.
*/
int16_t IMU_READRaw(void) 
{
  uint8_t data[20] = {0};
  // address bytes are used to read the data(accel,roll,yaw,pitch) from the particular location.
  //these address locations are defined in the Datasheet

  uint8_t imureg[20] = {0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
  int16_t mag_x, mag_y, mag_z;
  int16_t acc_x, acc_y, acc_z;
  int16_t gyr_x, gyr_y, gyr_z;
  int rhall;
  int orgyaw;
  int ret, yaw;

  double roll;
  double pitch;
  double azimuth;
  double X_h, Y_h;
  
  mag_x = 0;
  mag_y = 0;
  mag_z = 0;

  for(int k = 0; k <= 10; k++)
  {
    // read BMX160 registers data
    ret = i2c1ReadRegister(BMX160_I2C_ADDR, &imureg[0], 20, &data[0], 20);
    if (ret) 
    {
      printk("Error reading from FRAM! error code (%d)\n", ret);
    } 
    else 
    {
      mag_x += (int16_t)((data[1] << 8) | data[0]);
      mag_y += (int16_t)((data[3] << 8) | data[2]);
      mag_z += (int16_t)((data[5] << 8) | data[4]);
      rhall = (int16_t)((data[7] << 8) | data[6]);
      gyr_x = (int16_t)((data[9] << 8) | data[8]);
      gyr_y = (int16_t)((data[11] << 8) | data[10]);
      gyr_z = (int16_t)((data[13] << 8) | data[12]);
      acc_x = (int16_t)((data[15] << 8) | data[14]);
      acc_y = (int16_t)((data[17] << 8) | data[16]);
      acc_z = (int16_t)((data[19] << 8) | data[18]);
    }
    k_msleep(2U);
  }
   
  mag_x = mag_x/10;
  mag_y = mag_y/10;
  mag_z = mag_z/10;

  //printk("mag x = %d , ", mag_x);
  //printk("mag y = %d , ", mag_y);
  //printk("mag z = %d , ", mag_z);

  //printk("rhall = %d \n ", rhall);

  //printk("gyr x = %d , ", gyr_x);
  //printk("gyr y = %d , ", gyr_y);
  //printk("gyr z = %d \n ", gyr_z);
  
  //printk("acc x = %d , ", acc_x);
  //printk("acc y = %d , ", acc_y);
  //printk("acc z = %d \n ", acc_z);

  /* Calculate pitch and roll, in the range (-pi,pi) */
  pitch = atan2((double)-acc_x, sqrt((long)acc_z*(long)acc_z + (long)acc_y*(long)acc_y));
  roll = atan2((double)acc_y, sqrt((long)acc_z*(long)acc_z  + (long)acc_x*(long)acc_x));


  X_h = (double)mag_x*cos(pitch) + (double)mag_y*sin(roll)*sin(pitch) + (double)mag_z*cos(roll)*sin(pitch);
  Y_h = (double)mag_y*cos(roll) - (double)mag_z*sin(roll);

  // offset compansation
  X_h = (compass_scal_x * X_h) + compass_offset_x;
  Y_h = (compass_scal_y * Y_h) + compass_offset_y;

  azimuth = (180 * atan2(-Y_h, X_h) / PI);
  if(azimuth < 0)
  {	/* Convert Azimuth in the range (0, 2pi) */
    azimuth = 360 + azimuth;
  }
  else if(azimuth > 360)
  {	/* Convert Azimuth in the range (0, 2pi) */
    azimuth = azimuth - 360;
  }
  
  //printk(" IMU_round = %d \t", (int)roundf(azimuth));
  //printf(" IMU_Float = %.1f \n", azimuth);
  return (int)roundf(azimuth);
}


int calibrate_IMU()
{
  uint8_t data[20] = {0};
  uint8_t imureg[20] = {0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
  int16_t mag_x, mag_y, mag_z;
  int16_t acc_x, acc_y, acc_z;
  int16_t gyr_x, gyr_y, gyr_z;
  int rhall;
  int orgyaw;
  int ret, yaw;
  
  printk("Calibration sarted please rotate IMU sensor all directions.\n");

  
#if 1
  for(int k = 0; k <= 1; k++)
  {
    // read BMX160 registers data
    ret = i2c1ReadRegister(BMX160_I2C_ADDR, &imureg[0], 20, &data[0], 20);
    if (ret) 
    {
      printk("Error reading from FRAM! error code (%d)\n", ret);
    }
    else 
    {
      mag_x = (int16_t)((data[1] << 8) | data[0]);
      mag_y = (int16_t)((data[3] << 8) | data[2]);
      mag_z = (int16_t)((data[5] << 8) | data[4]);
      rhall = (int16_t)((data[7] << 8) | data[6]);
      gyr_x = (int16_t)((data[9] << 8) | data[8]);
      gyr_y = (int16_t)((data[11] << 8) | data[10]);
      gyr_z = (int16_t)((data[13] << 8) | data[12]);
      acc_x = (int16_t)((data[15] << 8) | data[14]);
      acc_y = (int16_t)((data[17] << 8) | data[16]);
      acc_z = (int16_t)((data[19] << 8) | data[18]);
    }

    if(mag_x < compass_cal_val[0])
    {
      compass_cal_val[0] = mag_x;
      //compass_cal_val[0] = mag_x;
    }
    if(mag_x > compass_cal_val[1])
    {
      compass_cal_val[1] = mag_x;
      //compass_cal_val[1] = 1625;
    }

    if(mag_y < compass_cal_val[2])
    {
      compass_cal_val[2] = mag_y;
      //compass_cal_val[2] = -1687;
    }
    if(mag_y > compass_cal_val[3])
    {
      compass_cal_val[3] = mag_y;
      //compass_cal_val[3] = 233;
    }

    if(mag_z < compass_cal_val[4])
    {
      compass_cal_val[4] = mag_z;
      //compass_cal_val[4] = -359;
    }
    if(mag_z > compass_cal_val[5])
    {
      compass_cal_val[5] = mag_z;
      //compass_cal_val[5] = -237;
    }
    
    k_msleep(50U);
    printk(".");
  }

#endif

  printk("\n");
  
  printk("Xmin = %d \n", compass_cal_val[0]);
  printk("Xmax = %d \n", compass_cal_val[1]);
  printk("Ymin = %d \n", compass_cal_val[2]);
  printk("Ymax = %d \n", compass_cal_val[3]);
  printk("Zmin = %d \n", compass_cal_val[4]);
  printk("Zmax = %d \n", compass_cal_val[5]);

  compass_scal_x = ((float)compass_cal_val[3] - compass_cal_val[2])/((float)compass_cal_val[1] - compass_cal_val[0]);
  compass_scal_y = ((float)compass_cal_val[1] - compass_cal_val[0])/((float)compass_cal_val[3] - compass_cal_val[2]);
  compass_scal_z = ((float)compass_cal_val[1] - compass_cal_val[0])/((float)compass_cal_val[5] - compass_cal_val[4]);
  
  printk("before \n");
  printf("Xsf = %f\n", compass_scal_x);
  printf("Ysf = %f\n", compass_scal_y);
  printf("Zsf = %f\n", compass_scal_z);

  if(compass_scal_x < 1)
  {
    compass_scal_x = 1;
  }
  if(compass_scal_y < 1)
  {
    compass_scal_y = 1;
  }
  if(compass_scal_z < 1)
  {
    compass_scal_z = 1;
  }
  
  printk("after\n");
  printf("Xsf = %f\n", compass_scal_x);
  printf("Ysf = %f\n", compass_scal_y);
  printf("Zsf = %f\n", compass_scal_z);

  compass_offset_x = (((compass_cal_val[1] - compass_cal_val[0])/2) - compass_cal_val[1]) * compass_scal_x;
  compass_offset_y = (((compass_cal_val[3] - compass_cal_val[2])/2) - compass_cal_val[3]) * compass_scal_y;
  compass_offset_z = (((compass_cal_val[5] - compass_cal_val[4])/2) - compass_cal_val[5]) * compass_scal_z;
  
  printk("Xoffset = %d\n", compass_offset_x);
  printk("Yoffset = %d\n", compass_offset_y);
  printk("Zoffset = %d\n", compass_offset_z);

  k_msleep(2000U);

  //printk("mag x = %d , ", mag_x);
  //printk("mag y = %d , ", mag_y);
  //printk("mag z = %d , ", mag_z);

  return 0;
}


/*
 * brief: Wrapper function to calculate and return the MEDIAN value from set of IMU Values measured by IMU_READRaw() routine.
 */
int16_t IMU_READMedian(void) 
{
  uint16_t yaw = IMU_READRaw();
  //printk(" IMU = %d \n", yaw);
  return yaw;
}

/*
 * brief: Wrapper function to calculate and return the MEDIAN value from set of IMU Values measured by IMU_READRaw() routine.
 */
int16_t IMU_READMedian_2(void)
{
  int16_t i, key, j;
  int16_t a[MEDIAN_ARRY_SIZE];
  for (i = 0; i < MEDIAN_ARRY_SIZE; i++) {
    a[i] = IMU_READRaw();
    k_msleep(10U);
  }
//Finding median of an array: sorted array middle element will be the median.

  for (i = 1; i < MEDIAN_ARRY_SIZE; i++) //sorting an array
  {
    key = a[i];
    j = i - 1;
    while (j >= 0 && a[j] > key) {
      a[j + 1] = a[j];
      j = j - 1;
    }
    a[j + 1] = key;
  }
  printk("IMU_Median: %d\n", a[(MEDIAN_ARRY_SIZE - 1) / 2]);
  return a[(MEDIAN_ARRY_SIZE - 1) / 2];
}