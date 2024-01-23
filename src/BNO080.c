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
  Arduino IDE 1.8.5

	SparkFun code, firmware, and software is released under the MIT License.
	Please see LICENSE.md for further details.
*/
#include <math.h>
#include <stdio.h>
#include <hal/nrf_gpio.h>
//#include <nrfx/hal/nrf_gpio.h>

#include "BNO080.h"
#include "Includes/I2C_Module.h"

int8_t BNO080_error_status;  // added for IMU sensor error report.

//Global Variables
// SHTP buffers for communicating with BNO080 over I2C
// Each packet has a header of 4 bytes
uint8_t shtpHeader[4]; 
uint8_t shtpData[MAX_PACKET_SIZE];

uint8_t sequenceNumber[6] = {0, 0, 0, 0, 0, 0}; //There are 6 com channels.
uint8_t calibrationStatus;	

int16_t rotationVector_Q1 = 14;
uint32_t timeStamp;
uint16_t rawFastGyroX, rawFastGyroY, rawFastGyroZ;
uint16_t rawQuatI, rawQuatJ, rawQuatK, rawQuatReal, rawQuatRadianAccuracy, quatAccuracy;	
bool _hasReset = false;

	
//Registers
const uint8_t CHANNEL_COMMAND = 0;
const uint8_t CHANNEL_EXECUTABLE = 1;
const uint8_t CHANNEL_CONTROL = 2;
const uint8_t CHANNEL_REPORTS = 3;
const uint8_t CHANNEL_WAKE_REPORTS = 4;
const uint8_t CHANNEL_GYRO = 5;


//These are the raw sensor values (without Q applied) pulled from the user requested Input Report
uint16_t rawAccelX, rawAccelY, rawAccelZ, accelAccuracy;
uint16_t rawLinAccelX, rawLinAccelY, rawLinAccelZ, accelLinAccuracy;
uint16_t rawGyroX, rawGyroY, rawGyroZ, gyroAccuracy;
uint16_t rawMagX, rawMagY, rawMagZ, magAccuracy;
uint16_t rawQuatI, rawQuatJ, rawQuatK, rawQuatReal, rawQuatRadianAccuracy, quatAccuracy;
uint16_t rawFastGyroX, rawFastGyroY, rawFastGyroZ;
uint8_t tapDetector;
uint16_t stepCount;
uint32_t timeStamp;
uint8_t stabilityClassifier;
uint8_t activityClassifier;
uint8_t *_activityConfidences;						  //Array that store the confidences of the 9 possible activities
uint8_t calibrationStatus;							  //Byte R0 of ME Calibration Response
uint16_t memsRawAccelX, memsRawAccelY, memsRawAccelZ; //Raw readings from MEMS sensor
uint16_t memsRawGyroX, memsRawGyroY, memsRawGyroZ;	//Raw readings from MEMS sensor
uint16_t memsRawMagX, memsRawMagY, memsRawMagZ;		  //Raw readings from MEMS sensor

//Send command to reset IC
//Read all advertisement packets from sensor
//The sensor has been seen to reset twice if we attempt too much too quickly.
//This seems to work reliably.
void softReset(void)
{
  BNO080_error_status = 0;

  // temporary for testing
  k_msleep(100);
  
  shtpData[0] = 1; //Reset
 
  //debug 
  printk("soft rreset \n");

  //Attempt to start communication with sensor
  sendPacket(CHANNEL_EXECUTABLE, 1); //Transmit packet on channel 1, 1 byte

  //debug 
  printk("soft rreset  2 \n");

  //Read all incoming data and flush it
  k_sleep(K_MSEC(200));
  while (receivePacket() == true)
  {
    k_msleep(50);
  }

  //k_sleep(K_MSEC(200));
  //while (receivePacket() == true)
  //{
  //  k_msleep(50);
  //}
}


//Sends the packet to enable the ar/vr stabilized rotation vector
void enableARVRStabilizedGameRotationVector(uint16_t timeBetweenReports)
{
  setFeatureCommand(SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR, timeBetweenReports);
}


//Given a sensor's report ID, this tells the BNO080 to begin reporting the values
void setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports)
{
  setFeatureCommandExtra(reportID, timeBetweenReports, 0); //No specific config
}

//Given a sensor's report ID, this tells the BNO080 to begin reporting the values
//Also sets the specific config word. Useful for personal activity classifier
void setFeatureCommandExtra(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig)
{
  long microsBetweenReports = (long)timeBetweenReports * 1000L;

  shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND;	 //Set feature command. Reference page 55
  shtpData[1] = reportID;	                         //Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
  shtpData[2] = 0;				         //Feature flags
  shtpData[3] = 0;				         //Change sensitivity (LSB)
  shtpData[4] = 0;				         //Change sensitivity (MSB)
  shtpData[5] = (microsBetweenReports >> 0) & 0xFF;      //Report interval (LSB) in microseconds. 0x7A120 = 500ms
  shtpData[6] = (microsBetweenReports >> 8) & 0xFF;      //Report interval
  shtpData[7] = (microsBetweenReports >> 16) & 0xFF;     //Report interval
  shtpData[8] = (microsBetweenReports >> 24) & 0xFF;     //Report interval (MSB)
  shtpData[9] = 0;				         //Batch Interval (LSB)
  shtpData[10] = 0;				         //Batch Interval
  shtpData[11] = 0;			                 //Batch Interval
  shtpData[12] = 0;				         //Batch Interval (MSB)
  shtpData[13] = (specificConfig >> 0) & 0xFF;	         //Sensor-specific config (LSB)
  shtpData[14] = (specificConfig >> 8) & 0xFF;	         //Sensor-specific config
  shtpData[15] = (specificConfig >> 16) & 0xFF;	         //Sensor-specific config
  shtpData[16] = (specificConfig >> 24) & 0xFF;	         //Sensor-specific config (MSB)

  //Transmit packet on channel 2, 17 bytes
  sendPacket(CHANNEL_CONTROL, 17);
}


//Updates the latest variables if possible
//Returns false if new readings are not available
bool dataAvailable(void)
{
  return (getReadings() != 0);
}

uint16_t getReadings(void)
{
  if (receivePacket() == true)
  {
    //Check to see if this packet is a sensor reporting its data to us
    if (shtpHeader[2] == CHANNEL_REPORTS && shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP)
    {
      return parseInputReport(); //This will update the rawAccelX, etc variables depending on which feature report is found
    }
    else if (shtpHeader[2] == CHANNEL_CONTROL)
    {
      return parseCommandReport(); //This will update responses to commands, calibrationStatus, etc.
    }
    else if(shtpHeader[2] == CHANNEL_GYRO)
    {
      return parseInputReport(); //This will update the rawAccelX, etc variables depending on which feature report is found
    }
  }
  else
  {
    //printk("No data packet received from IMU sensor\n");
  }
  return 0;
}


//This function pulls the data from the input report
//The input reports vary in length so this function stores the various 16-bit values as globals

//Unit responds with packet that contains the following:
//shtpHeader[0:3]: First, a 4 byte header
//shtpData[0:4]: Then a 5 byte timestamp of microsecond clicks since reading was taken
//shtpData[5 + 0]: Then a feature report ID (0x01 for Accel, 0x05 for Rotation Vector)
//shtpData[5 + 1]: Sequence number (See 6.5.18.2)
//shtpData[5 + 2]: Status
//shtpData[3]: Delay
//shtpData[4:5]: i/accel x/gyro x/etc
//shtpData[6:7]: j/accel y/gyro y/etc
//shtpData[8:9]: k/accel z/gyro z/etc
//shtpData[10:11]: real/gyro temp/etc
//shtpData[12:13]: Accuracy estimate
uint16_t parseInputReport(void)
{
  //Calculate the number of data bytes in this packet
  int16_t dataLength = ((uint16_t)shtpHeader[1] << 8 | shtpHeader[0]);
  dataLength &= ~(1 << 15); //Clear the MSbit. This bit indicates if this package is a continuation of the last.
  //Ignore it for now. TODO catch this as an error and exit

  dataLength -= 4; //Remove the header bytes from the data count

  timeStamp = ((uint32_t)shtpData[4] << (8 * 3)) | ((uint32_t)shtpData[3] << (8 * 2)) | ((uint32_t)shtpData[2] << (8 * 1)) | ((uint32_t)shtpData[1] << (8 * 0));
  //printk("timestamp : %d\n", timeStamp);
  // The gyro-integrated input reports are sent via the special gyro channel and do no include the usual ID, sequence, and status fields
  if(shtpHeader[2] == CHANNEL_GYRO) 
  {
    rawQuatI = (uint16_t)shtpData[1] << 8 | shtpData[0];
    rawQuatJ = (uint16_t)shtpData[3] << 8 | shtpData[2];
    rawQuatK = (uint16_t)shtpData[5] << 8 | shtpData[4];
    rawQuatReal = (uint16_t)shtpData[7] << 8 | shtpData[6];
    rawFastGyroX = (uint16_t)shtpData[9] << 8 | shtpData[8];
    rawFastGyroY = (uint16_t)shtpData[11] << 8 | shtpData[10];
    rawFastGyroZ = (uint16_t)shtpData[13] << 8 | shtpData[12];

    return SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR;
  }

  uint8_t status = shtpData[5 + 2] & 0x03; //Get status bits
  uint16_t data1 = (uint16_t)shtpData[5 + 5] << 8 | shtpData[5 + 4];
  uint16_t data2 = (uint16_t)shtpData[5 + 7] << 8 | shtpData[5 + 6];
  uint16_t data3 = (uint16_t)shtpData[5 + 9] << 8 | shtpData[5 + 8];
  uint16_t data4 = 0;
  uint16_t data5 = 0; //We would need to change this to uin32_t to capture time stamp value on Raw Accel/Gyro/Mag reports
  //printk("data1 : %d, data2 : %d, data3 : %d\n", data1, data2, data3);
  if (dataLength - 5 > 9)
  {
    data4 = (uint16_t)shtpData[5 + 11] << 8 | shtpData[5 + 10];
  }
  if (dataLength - 5 > 11)
  {
    data5 = (uint16_t)shtpData[5 + 13] << 8 | shtpData[5 + 12];
  }

  //Store these generic values to their proper global variable
  if (shtpData[5] == SENSOR_REPORTID_ACCELEROMETER)
  {
    accelAccuracy = status;
    rawAccelX = data1;
    rawAccelY = data2;
    rawAccelZ = data3;
  }
  else if (shtpData[5] == SENSOR_REPORTID_LINEAR_ACCELERATION)
  {
    accelLinAccuracy = status;
    rawLinAccelX = data1;
    rawLinAccelY = data2;
    rawLinAccelZ = data3;
  }
  else if (shtpData[5] == SENSOR_REPORTID_GYROSCOPE)
  {
    gyroAccuracy = status;
    rawGyroX = data1;
    rawGyroY = data2;
    rawGyroZ = data3;
  }
  else if (shtpData[5] == SENSOR_REPORTID_MAGNETIC_FIELD)
  {
    magAccuracy = status;
    rawMagX = data1;
    rawMagY = data2;
    rawMagZ = data3;
  }
  else if (shtpData[5] == SENSOR_REPORTID_ROTATION_VECTOR ||
          shtpData[5] == SENSOR_REPORTID_GAME_ROTATION_VECTOR ||
          shtpData[5] == SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR ||
          shtpData[5] == SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR)
  {
    quatAccuracy = status;
    rawQuatI = data1;
    rawQuatJ = data2;
    rawQuatK = data3;
    rawQuatReal = data4;

    //Only available on rotation vector and ar/vr stabilized rotation vector,
    // not game rot vector and not ar/vr stabilized rotation vector
    rawQuatRadianAccuracy = data5;
  }
  else if (shtpData[5] == SENSOR_REPORTID_TAP_DETECTOR)
  {
    tapDetector = shtpData[5 + 4]; //Byte 4 only
  }
  else if (shtpData[5] == SENSOR_REPORTID_STEP_COUNTER)
  {
    stepCount = data3; //Bytes 8/9
  }
  else if (shtpData[5] == SENSOR_REPORTID_STABILITY_CLASSIFIER)
  {
    stabilityClassifier = shtpData[5 + 4]; //Byte 4 only
  }
  else if (shtpData[5] == SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER)
  {
    activityClassifier = shtpData[5 + 5]; //Most likely state

    //Load activity classification confidences into the array
    for (uint8_t x = 0; x < 9; x++)					   //Hardcoded to max of 9. TODO - bring in array size
    {      
      _activityConfidences[x] = shtpData[5 + 6 + x]; //5 bytes of timestamp, byte 6 is first confidence byte
    }
  }
  else if (shtpData[5] == SENSOR_REPORTID_RAW_ACCELEROMETER)
  {
    memsRawAccelX = data1;
    memsRawAccelY = data2;
    memsRawAccelZ = data3;
  }
  else if (shtpData[5] == SENSOR_REPORTID_RAW_GYROSCOPE)
  {
    memsRawGyroX = data1;
    memsRawGyroY = data2;
    memsRawGyroZ = data3;
  }
  else if (shtpData[5] == SENSOR_REPORTID_RAW_MAGNETOMETER)
  {
    memsRawMagX = data1;
    memsRawMagY = data2;
    memsRawMagZ = data3;
  }
  else if (shtpData[5] == SHTP_REPORT_COMMAND_RESPONSE)
  {    
    //The BNO080 responds with this report to command requests. It's up to use to remember which command we issued.
    uint8_t command = shtpData[5 + 2]; //This is the Command byte of the response

    if (command == COMMAND_ME_CALIBRATE)
    {   
      calibrationStatus = shtpData[5 + 5]; //R0 - Status (0 = success, non-zero = fail)
    }
  }
  else
  {
    //This sensor report ID is unhandled.
    //See reference manual to add additional feature reports as needed
    return 0;
  }

  //TODO additional feature reports may be strung together. Parse them all.
  return shtpData[5];
}


//Return the acceleration component
uint8_t getQuatAccuracy()
{
    return (quatAccuracy);
}
float getRoll()
{
    float dqw = getQuatReal();
    float dqx = getQuatI();
    float dqy = getQuatJ();
    float dqz = getQuatK();

    float norm = sqrt(dqw*dqw + dqx*dqx + dqy*dqy + dqz*dqz);
    dqw = dqw/norm;
    dqx = dqx/norm;
    dqy = dqy/norm;
    dqz = dqz/norm;

    float ysqr = dqy * dqy;

    // roll (x-axis rotation)
    float t0 = +2.0 * (dqw * dqx + dqy * dqz);
    float t1 = +1.0 - 2.0 * (dqx * dqx + ysqr);
    float roll = atan2(t0, t1);

    return (roll);
}

// Return the pitch (rotation around the y-axis) in Radians
float getPitch()
{
    float dqw = getQuatReal();

    float dqx = getQuatI();
    float dqy = getQuatJ();
    float dqz = getQuatK();

    float norm = sqrt(dqw*dqw + dqx*dqx + dqy*dqy + dqz*dqz);
    dqw = dqw/norm;
    dqx = dqx/norm;
    dqy = dqy/norm;
    dqz = dqz/norm;

    float ysqr = dqy * dqy;

    // pitch (y-axis rotation)
    float t2 = +2.0 * (dqw * dqy - dqz * dqx);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    float pitch = asin(t2);
    
    return (pitch);
}

// Return the yaw / heading (rotation around the z-axis) in Radians
float getYaw()
{
    float dqw = getQuatReal();
    float dqx = getQuatI();
    float dqy = getQuatJ();
    float dqz = getQuatK();

    float norm = sqrt(dqw*dqw + dqx*dqx + dqy*dqy + dqz*dqz);
    dqw = dqw/norm;
    dqx = dqx/norm;
    dqy = dqy/norm;
    dqz = dqz/norm;

    float ysqr = dqy * dqy;

    // yaw (z-axis rotation)
    float t3 = +2.0 * (dqw * dqz + dqx * dqy);
    float t4 = +1.0 - 2.0 * (ysqr + dqz * dqz);
    float yaw = atan2(t3, t4);

    //debug print
    //printf("YAW : %f\n", norm);
    return (yaw);
}


//Return the rotation vector quaternion I
float getQuatI()
{
    float quat = qToFloat(rawQuatI, rotationVector_Q1);
  
    return (quat);
}

//Return the rotation vector quaternion J
float getQuatJ()
{
    float quat = qToFloat(rawQuatJ, rotationVector_Q1);
    
    return (quat);
}

//Return the rotation vector quaternion K
float getQuatK()
{
    float quat = qToFloat(rawQuatK, rotationVector_Q1);
    return (quat);
}

//Return the rotation vector quaternion Real
float getQuatReal()
{
    float quat = qToFloat(rawQuatReal, rotationVector_Q1);
    return (quat);
}


//Given a register value and a Q point, convert to float
//See https://en.wikipedia.org/wiki/Q_(number_format)
float qToFloat(int16_t fixedPointValue, uint8_t qPoint)
{
    float qFloat = fixedPointValue;
    qFloat *= pow(2, qPoint * -1);
    return (qFloat);
}



//This function pulls the data from the command response report

//Unit responds with packet that contains the following:
//shtpHeader[0:3]: First, a 4 byte header
//shtpData[0]: The Report ID
//shtpData[1]: Sequence number (See 6.5.18.2)
//shtpData[2]: Command
//shtpData[3]: Command Sequence Number
//shtpData[4]: Response Sequence Number
//shtpData[5 + 0]: R0
//shtpData[5 + 1]: R1
//shtpData[5 + 2]: R2
//shtpData[5 + 3]: R3
//shtpData[5 + 4]: R4
//shtpData[5 + 5]: R5
//shtpData[5 + 6]: R6
//shtpData[5 + 7]: R7
//shtpData[5 + 8]: R8
uint16_t parseCommandReport(void)
{
  if (shtpData[0] == SHTP_REPORT_COMMAND_RESPONSE)
  {
    //The BNO080 responds with this report to command requests. It's up to use to remember which command we issued.
    uint8_t command = shtpData[2]; //This is the Command byte of the response

    if (command == COMMAND_ME_CALIBRATE)
    {
      calibrationStatus = shtpData[5 + 0]; //R0 - Status (0 = success, non-zero = fail)
    }
    return shtpData[0];
  }
  else
  {
    //This sensor report ID is unhandled.
    //See reference manual to add additional feature reports as needed
  }

  //TODO additional feature reports may be strung together. Parse them all.
  return 0;
}


//Given the data packet, send the header then the data
//Returns false if sensor does not ACK
//TODO - Arduino has a max 32 byte send. Break sending into multi packets if needed.
bool sendPacket(uint8_t channelNumber, uint8_t dataLength)
{
  uint8_t packetLength = dataLength + 4; //Add four bytes for the header
  uint8_t tempHeader[4 + MAX_PACKET_SIZE] = {0};
  int ret = 0;

  if(packetLength > I2C_BUFFER_LENGTH) return(false); //You are trying to send too much. Break into smaller packets.

  //Send the 4 byte packet header
  tempHeader[0] = (packetLength & 0xFF);			  //Packet length LSB
  tempHeader[1] = (packetLength >> 8);                        //Packet length MSB
  tempHeader[2] = channelNumber;                              //Channel number  
  tempHeader[3] = sequenceNumber[channelNumber]++;            //Send the sequence number, increments with each packet sent, different counter for each channel

  if(dataLength > 0)
  {
    memcpy((void *)&tempHeader[4], (const void *)shtpData, dataLength);
  }

  // Now send the header so prepared over Nordic I2C
  ret = i2cSendRegister((const uint8_t *)tempHeader, (uint32_t)4 + dataLength, BNO080_DEFAULT_ADDRESS);
  if(ret != 0)
  {
    printk("Error (%d) Sending IMU Data\n", ret);
  }
  return (true);
}


//Check to see if there is any new data available
//Read the contents of the incoming packet into the shtpData array
bool receivePacket(void)
{
  int ret = 0;
  uint8_t packetLSB = 0;
  uint8_t packetMSB = 0;

  /*Read the header to determine the length of data to read*/
  //ret = i2cRead((uint16_t)BNO080_DEFAULT_ADDRESS, (void *)shtpHeader, (uint32_t)4);
  ret = i2cRead((uint16_t)BNO080_DEFAULT_ADDRESS, shtpHeader, (uint32_t)4);
  //while( ret != 0)
  if( ret != 0)
  {
    //k_msleep(50);
    printk("Error (%d) Reading IMU Data\n", ret);
  }
      
  // Get the Data length bytes
  packetLSB = shtpHeader[0];
  packetMSB = shtpHeader[1];
 
  //Calculate the number of data bytes in this packet
  uint16_t dataLength = (((uint16_t)packetMSB) << 8) | ((uint16_t)packetLSB);

  //printk("Before left shift dtalength = %d \n", dataLength);
  //Clear the MSbit.
  dataLength &= ~(1 << 15); //This bit indicates if this package is a continuation of the last. Ignore it for now.
  //TODO catch this as an error and exit

  if (dataLength == 0)
  {
    //Packet is empty
    printk("data packet empty \n");
    printk("SHTP Header = %d, %d, %d, %d \n", shtpHeader[0], shtpHeader[1], shtpHeader[2], shtpHeader[3]);

    return (false); //All done
  }

  //Remove the header bytes from the data count
  dataLength -= 4; 

  // Query the data of dataLength bytes ( with 4 Bytes header again)
  getData(dataLength);

  // Quickly check for reset complete packet. No need for a seperate parser.
  // This function is also called after soft reset, so we need to catch this
  // packet here otherwise we need to check for the reset packet in multiple
  // places.
  if (shtpHeader[2] == CHANNEL_EXECUTABLE && shtpData[0] == EXECUTABLE_RESET_COMPLETE) 
  {
    // TODO: use this flag to identify if the sensor has reset
    _hasReset = true;
  } 
  return (true); //We're done!
}


//Sends multiple requests to sensor until all data bytes are received from sensor
//The shtpData buffer has max capacity of MAX_PACKET_SIZE. Any bytes over this amount will be lost.
//Arduino I2C read limit is 32 bytes. Header is 4 bytes, so max data we can read per interation is 28 bytes
bool getData(uint16_t bytesRemaining)
{
  int ret = 0;
  uint8_t tempBuffer[MAX_PACKET_SIZE]; // This buffer will hold the unwanted data
  bool isFirstPacket = true;

  //printk("In getData bytes = %d\n", bytesRemaining);

  //Setup a series of chunked 124 byte reads
  while (bytesRemaining > 0)
  {
      uint16_t numberOfBytesToRead = bytesRemaining;
      if (numberOfBytesToRead > (I2C_BUFFER_LENGTH - 4))
      {
        numberOfBytesToRead = (I2C_BUFFER_LENGTH - 4);  
      }
      
      // Read the actual data after headers are read
      ret = i2cRead((uint16_t)BNO080_DEFAULT_ADDRESS, (void *)tempBuffer, (uint32_t)numberOfBytesToRead + 4);
      if(ret != 0)
      {
        printk("Error (%d) Reading IMU Data 2\n", ret);
      }

      if((isFirstPacket == true) && (ret == 0))
      {
       // Discard first 4 header bytes and copy remaining data to shtpData
        memcpy((void *)shtpData, (const void *)&tempBuffer[4], numberOfBytesToRead);
      }
      else
      {
        // If the packet is not first MAX_BUF_LEN, it could be garbage data which we must flush out of I2C 
        // and hence it will be read but not copied to shtpData buffer
      }
      bytesRemaining -= numberOfBytesToRead;
      isFirstPacket = false;
  }
  
  //printk("Out getData bytes = %d\n", bytesRemaining);

  return (true); //Done!
}
//bool getData(uint16_t bytesRemaining)
//{
//  int ret = 0;
//  uint8_t tempBuffer[MAX_PACKET_SIZE]; // This buffer will hold the unwanted data
//  bool isFirstPacket = true;
//  uint16_t dataSpot = 0; //Start at the beginning of shtpData array

//  //Setup a series of chunked 124 byte reads
//  while (bytesRemaining > 0)
//  {
//      uint16_t numberOfBytesToRead = bytesRemaining;
//      if (numberOfBytesToRead > (I2C_BUFFER_LENGTH - 4))
//      {
//              numberOfBytesToRead = (I2C_BUFFER_LENGTH - 4);   
//      }
//      // Read the actual data after headers are read
//      ret = i2cRead((uint16_t)BNO080_DEFAULT_ADDRESS, (void *)tempBuffer, (uint32_t)numberOfBytesToRead + 4);
      
//      if(ret == 0)
//      {
//        // Discard first 4 header bytes and copy remaining data to shtpData
//        memcpy(&shtpData[dataSpot], &tempBuffer[4], numberOfBytesToRead);
//        dataSpot += numberOfBytesToRead;
//      }

//      bytesRemaining -= numberOfBytesToRead;
//      isFirstPacket = false;
//  }
//  return (true); //Done!
//}


//Sends the packet to enable the rotation vector
void enableGameRotationVector(uint16_t timeBetweenReports)
{
  setFeatureCommand(SENSOR_REPORTID_GAME_ROTATION_VECTOR, timeBetweenReports);
}



/*!
 *  @brief  Enter Normal mode (i.e., wake)
 */
bool initIMU_BNO080() 
{
  //// reset the NDP
  //nrf_gpio_cfg_output(BNO_RESET_PIN);

  //Hard_Reset_BNO080();

  // Begin by resetting the IMU
  softReset();

  //Check communication with device
  shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
  shtpData[1] = 0;							  //Reserved

  //Transmit packet on channel 2, 2 bytes
  sendPacket(CHANNEL_CONTROL, 2);

  //Now we wait for response
  if (receivePacket() == true)
  {
    if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
    {      
      return (true);
    }
  }
  
  return (false); //Something went wrong
}

uint16_t IMU_getReadings(void)
{
  if (receivePacket() == true)
  {
    //Check to see if this packet is a sensor reporting its data to us
    if (shtpHeader[2] == CHANNEL_REPORTS && shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP)
    {
      printk("\n Output channel Report \n");
      return parseInputReport(); //This will update the rawAccelX, etc variables depending on which feature report is found
    }
    else if (shtpHeader[2] == CHANNEL_CONTROL)
    {
      printk("\n Output command Report \n");
      return parseCommandReport(); //This will update responses to commands, calibrationStatus, etc.
    }
  }
  return 0;
}

bool IMU_dataAvailable(void)
{
  return (IMU_getReadings() != 0);
}


float tempYaw = 180.0f,prevYaw = 180.0f;
float IMU_READRawBNO080()
{
  // Initialise to -ve so that if data is not available, this function should not return 0.0 which is a valid IMU val
  float yaw = -1.0;
  float diffYaw = -1.0;
  if (dataAvailable() == true)
  {
    uint8_t sensorAccuracy = getQuatAccuracy();
    yaw = (getYaw()) * 180.0 / 3.14159265359; // Convert yaw / heading to degrees
    yaw += 180.0f;
    if(((prevYaw - yaw) >.1) || ((prevYaw - yaw )< -.1))
    {
      tempYaw += -(prevYaw - yaw);
      prevYaw = yaw;
      return (tempYaw);
    }
    else
    {
      prevYaw = yaw;
      return (tempYaw);
    }
  }
  else
  {
    //printk("IMU sensor data not available\n");
    return (yaw);
  }
}

//int Hard_Reset_BNO080()
//{
//  nrf_gpio_pin_clear(BNO_RESET_PIN);
//  k_msleep(500U);
//  nrf_gpio_pin_set(BNO_RESET_PIN);
//  printk("IMU Reseted\n");

//  return 0;
//}