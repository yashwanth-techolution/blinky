/*
  This is a library written for the BNO080
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14686

  Written by Nathan Seidle @ SparkFun Electronics, December 28th, 2017

  The BNO080 IMU is a powerful triple axis gyro/accel/magnetometer coupled with an ARM processor
  to maintain and complete all the complex calculations for various VR, inertial, step counting,
  and movement operations.

  This library handles the initialization of the BNO080 and is able to query the sensor
  for different readings.

  https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.3

  SparkFun code, firmware, and software is released under the MIT License.
	Please see LICENSE.md for further details.
*/


#ifndef BNO080_MODULE_H_
#define BNO080_MODULE_H_

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


#include<stdint.h>
#include<stdbool.h>

#include<zephyr/device.h>
#include<zephyr/devicetree.h>

//#define BNO_RESET_PIN  3U

//The default I2C address for the BNO080 on the SparkX breakout is 0x4B. 0x4A is also possible.
#define BNO080_DEFAULT_ADDRESS 0x4B


//The catch-all default is 32
#define I2C_BUFFER_LENGTH 32

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//All the ways we can configure or talk to the BNO080, figure 34, page 36 reference manual
//These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

//All the different sensors and features we can get reports from
//These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD 0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR 0x2A
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER 0x11
#define SENSOR_REPORTID_STABILITY_CLASSIFIER 0x13
#define SENSOR_REPORTID_RAW_ACCELEROMETER 0x14
#define SENSOR_REPORTID_RAW_GYROSCOPE 0x15
#define SENSOR_REPORTID_RAW_MAGNETOMETER 0x16
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E
#define SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR 0x28
#define SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 0x29

//Record IDs from figure 29, page 29 reference manual
//These are used to read the metadata for each sensor type
#define FRS_RECORDID_ACCELEROMETER 0xE302
#define FRS_RECORDID_GYROSCOPE_CALIBRATED 0xE306
#define FRS_RECORDID_MAGNETIC_FIELD_CALIBRATED 0xE309
#define FRS_RECORDID_ROTATION_VECTOR 0xE30B

// Reset complete packet (BNO08X Datasheet p.24 Figure 1-27)
#define EXECUTABLE_RESET_COMPLETE 0x1

//Command IDs from section 6.4, page 42
//These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

#define CALIBRATE_ACCEL 0
#define CALIBRATE_GYRO 1
#define CALIBRATE_MAG 2
#define CALIBRATE_PLANAR_ACCEL 3
#define CALIBRATE_ACCEL_GYRO_MAG 4
#define CALIBRATE_STOP 5

#define MAX_PACKET_SIZE 128 //Packets can be up to 32k but we don't have that much RAM.
#define MAX_METADATA_SIZE 9 //This is in words. There can be many but we mostly only care about the first 9 (Qs, range, etc)


bool initIMU_BNO080();
float IMU_READRawBNO080();

bool bno080Begin(); //By default use the default I2C addres, and use Wire port, and don't declare an INT pin
//int Hard_Reset_BNO080();  
void softReset();	  //Try to reset the IMU via software
bool hasReset(); //Returns true if the sensor has reported a reset. Reading this will unflag the reset.
uint8_t resetReason(); //Query the IMU for the reason it last reset
void modeOn();	  //Use the executable channel to turn the BNO on
void modeSleep();	  //Use the executable channel to put the BNO to sleep

float qToFloat(int16_t fixedPointValue, uint8_t qPoint); //Given a Q value, converts fixed point floating to regular floating point number

bool waitForI2C(); //Delay based polling for I2C traffic
bool waitForSPI(); //Delay based polling for INT pin to go low
bool receivePacket(void);
bool getData(uint16_t bytesRemaining); //Given a number of bytes, send the requests in I2C_BUFFER_LENGTH chunks
bool sendPacket(uint8_t channelNumber, uint8_t dataLength);
void printPacket(void); //Prints the current shtp header and data packets
void printHeader(void); //Prints the current shtp header (only)

void enableRotationVector(uint16_t timeBetweenReports);
void enableGameRotationVector(uint16_t timeBetweenReports);
void enableARVRStabilizedRotationVector(uint16_t timeBetweenReports);
void enableARVRStabilizedGameRotationVector(uint16_t timeBetweenReports);
void enableAccelerometer(uint16_t timeBetweenReports);
void enableLinearAccelerometer(uint16_t timeBetweenReports);
void enableGyro(uint16_t timeBetweenReports);
void enableMagnetometer(uint16_t timeBetweenReports);
void enableTapDetector(uint16_t timeBetweenReports);
void enableStepCounter(uint16_t timeBetweenReports);
void enableStabilityClassifier(uint16_t timeBetweenReports);
void enableRawAccelerometer(uint16_t timeBetweenReports);
void enableRawGyro(uint16_t timeBetweenReports);
void enableRawMagnetometer(uint16_t timeBetweenReports);
void enableGyroIntegratedRotationVector(uint16_t timeBetweenReports);

bool dataAvailable(void);
uint16_t getReadings(void);
uint16_t parseInputReport(void);   //Parse sensor readings out of report
uint16_t parseCommandReport(void); //Parse command responses out of report

void getQuat(float *i, float *j, float *k, float *real, float *radAccuracy, uint8_t *accuracy);
float getQuatI();
float getQuatJ();
float getQuatK();
float getQuatReal();
float getQuatRadianAccuracy();
uint8_t getQuatAccuracy();

void getAccel(float *x, float *y, float *z, uint8_t *accuracy);
float getAccelX();
float getAccelY();
float getAccelZ();
uint8_t getAccelAccuracy();

void getLinAccel(float *x, float *y, float *z, uint8_t *accuracy);
float getLinAccelX();
float getLinAccelY();
float getLinAccelZ();
uint8_t getLinAccelAccuracy();

void getGyro(float *x, float *y, float *z, uint8_t *accuracy);
float getGyroX();
float getGyroY();
float getGyroZ();
uint8_t getGyroAccuracy();

void getFastGyro(float *x, float *y, float *z);
float getFastGyroX();
float getFastGyroY();
float getFastGyroZ();

void getMag(float *x, float *y, float *z, uint8_t *accuracy);
float getMagX();
float getMagY();
float getMagZ();
uint8_t getMagAccuracy();

void calibrateAccelerometer();
void calibrateGyro();
void calibrateMagnetometer();
void calibratePlanarAccelerometer();
void calibrateAll();
void endCalibration();
void saveCalibration();
void requestCalibrationStatus(); //Sends command to get status
bool calibrationComplete();   //Checks ME Cal response for byte 5, R0 - Status

uint8_t getTapDetector();
uint32_t getTimeStamp();
uint16_t getStepCount();
uint8_t getStabilityClassifier();
uint8_t getActivityClassifier();

int16_t getRawAccelX();
int16_t getRawAccelY();
int16_t getRawAccelZ();

int16_t getRawGyroX();
int16_t getRawGyroY();
int16_t getRawGyroZ();

int16_t getRawMagX();
int16_t getRawMagY();
int16_t getRawMagZ();

float getRoll();
float getPitch();
float getYaw();

void setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports);
void setFeatureCommandExtra(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig);
void sendCommand(uint8_t command);
void sendCalibrateCommand(uint8_t thingToCalibrate);

//Metadata functions
int16_t getQ1(uint16_t recordID);
int16_t getQ2(uint16_t recordID);
int16_t getQ3(uint16_t recordID);
float getResolution(uint16_t recordID);
float getRange(uint16_t recordID);
uint32_t readFRSword(uint16_t recordID, uint8_t wordNumber);
void frsReadRequest(uint16_t recordID, uint16_t readOffset, uint16_t blockSize);
bool readFRSdata(uint16_t recordID, uint8_t startLocation, uint8_t wordsToRead);

#endif