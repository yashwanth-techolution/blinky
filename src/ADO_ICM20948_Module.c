#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <math.h>
#include <stdio.h>
#include <zephyr/kernel.h>

#include "IMU/ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "Includes/ADO_ICM20948_Module.h"
#include "Includes/I2C_Module.h"
//#define SERIAL_PORT Serial

//#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
//#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined

//#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL 1 // The value of the last bit of the I2C address.                \
                       // On the SparkFun 9DoF IMU breakout the default is 1, and when \
                       // the ADR jumper is closed the value becomes 0

// ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object

#define PI 3.14159265359

int init_ICM20948_setup() {
#ifndef QUAT_ANIMATION
  printk("ICM-20948 Example\n");
#endif

  k_msleep(100);
  bool initialized = false;
  while (!initialized) {
    // Initialize the ICM-20948
    // If the DMP is enabled, .begin performs a minimal startup.
    // We need to configure the sample mode etc. manually.
#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    ICM_20948_I2C_begin();
#endif

#ifndef QUAT_ANIMATION
    printk("Initialization of the sensor returned: ");
    printk(ICM_20948_statusString());
#endif
    if (status != ICM_20948_Stat_Ok) {
#ifndef QUAT_ANIMATION
      printk("\nTrying again...\n");
#endif
      k_msleep(500);
    } else {
      initialized = true;
    }
  }

#ifndef QUAT_ANIMATION
  printk("Device connected!\n");
#endif

  bool success = true; // Use success to show if the DMP configuration was successful
  // Initialize the DMP. initializeDMP is a weak function.
  // You can overwrite it if you want to e.g. to change the sample rate
  success &= (ICM_20948_initializeDMP() == ICM_20948_Stat_Ok);

  // Enable the DMP Game Rotation Vector sensor
  success &= (ICM_20948_enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR, true) == ICM_20948_Stat_Ok);

  // Configuring DMP to output data at multiple ODRs:
  // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  success &= (ICM_20948_setDMPODRrate(DMP_ODR_Reg_Quat6, 4) == ICM_20948_Stat_Ok); // Set to the maximum

  // Enable the FIFO
  success &= (ICM_20948_enableFIFO(true) == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (ICM_20948_enableDMP(true) == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (ICM_20948_resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (ICM_20948_resetFIFO() == ICM_20948_Stat_Ok);

  // Check success
  if (success) {
#ifndef QUAT_ANIMATION
    printk("DMP enabled!\n\n\n");
    return 1;
#endif
  } else {
    printk("Enable DMP failed!\n");
    printk("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h...\n");
    return 0;
  }
}

double yaw = 0;
double yaw_icm = 0;
double prev_icm_reading = 180;
float tempYaw_icm = 180.0f, prevYaw_icm = 180.0f;
icm_20948_DMP_data_t data;

double check_for_data(void) {
  int count = 0;
  // printf("[");
  while (1) {
    ICM_20948_readDMPdataFromFIFO(&data);
    count++;
    if (status != 10) { // Was valid data available?
      if (status == 0) {
        if ((data.header & DMP_header_bitmap_Quat6) > 0) { // We have asked for GRV data so we should receive Quat6
          double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
          double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
          double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

          double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
          double q2sqr = q2 * q2;

          double t3 = +2.0 * (q0 * q3 + q1 * q2);
          double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
          yaw = atan2(t3, t4) * 180.0 / PI;

          // printf("\nDevice status-->Raw_Yaw=%0.2lf\n",yaw);
          yaw += 180;
        }
      }
    } else {
      // printk("\n Device status -> fifo count =%d\n",count);
      // count=0;
      // printf(", ");

      if (((prevYaw_icm - yaw) > .1) || ((prevYaw_icm - yaw) < -.1)) {
        // printf(" Device_status -> Raw_Yaw +180 = %0.2lf\n", yaw);
        if (tempYaw_icm < (prevYaw_icm - yaw)) {
          tempYaw_icm = (prevYaw_icm - yaw) - tempYaw_icm;
          prevYaw_icm = yaw;
          return (tempYaw_icm);
        } else {
          tempYaw_icm += -(prevYaw_icm - yaw);
          prevYaw_icm = yaw;
          return (tempYaw_icm);
        }
        // tempYaw_icm += -(prevYaw_icm - yaw);
        // prevYaw_icm = yaw;
        // return (tempYaw_icm);
      } else {
        // printf("Device_status -> Raw_IMU_Yaw = %0.2lf,\n", yaw);
        prevYaw_icm = yaw;
        return (tempYaw_icm);
      }
    }
  }
}

double Read_ICM20948_YAW_Data(void) {
  double yaw_value;
  yaw_value = check_for_data();

  // icm_20948_DMP_data_t data;
  // ICM_20948_readDMPdataFromFIFO(&data);

  // if ((status == ICM_20948_Stat_Ok) || (status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  //{

  //  if ((data.header & DMP_header_bitmap_Quat6) > 0) // We have asked for GRV data so we should receive Quat6
  //  {
  //   float q1 = ((float)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
  //   float q2 = ((float)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
  //   float q3 = ((float)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

  //   double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
  //   double q2sqr = q2 * q2;

  //   // roll (x-axis rotation)
  //   double t0 = +2.0 * (q0 * q1 + q2 * q3);
  //   double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
  //   double roll = atan2(t0, t1) * 180.0 / PI;

  //   // pitch (y-axis rotation)
  //   double t2 = +2.0 * (q0 * q2 - q3 * q1);
  //   t2 = t2 > 1.0 ? 1.0 : t2;
  //   t2 = t2 < -1.0 ? -1.0 : t2;
  //   double pitch = asin(t2) * 180.0 / PI;

  //   // yaw (z-axis rotation)
  //   double t3 = +2.0 * (q0 * q3 + q1 * q2);
  //   double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
  //   yaw = atan2(t3, t4) * 180.0 / PI;
  //   yaw += 180;
  //  }
  //}
  return yaw_value;
}