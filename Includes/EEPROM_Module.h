#ifndef __EEPROM_MODULE_H_
#define __EEPROM_MODULE_H_

#include<stdint.h>
#include<stdbool.h>


// BMX160 i2c device addres.
#define EEPROM_I2C_ADDR 0x50

#define INSTALLATION_MEMORY_LOCATION	        0x0064	// 100 in decimal 
#define CALIBRATION_MEMORY_LOCATION	        0x00AD  // 173 in decimal
#define CONFIGURATION_MEMORY_LOCATION	        0x012C  // 300 in decimal
#define AUDIO_FILE_ADDRESS_LOCATION             0x0032  // 50 in decimal
#define OPERATION_MEMORY_LOCATION               0x0190  // 400 in decimal

//---------------------------added for wifi cred ------------------------------//
#define WIFI_SSID_LENGTH               0x03E8  // 1000 in decimal
#define WIFI_SSID_LOCATION             0x03E9  // 1001 in decimal

#define WIFI_PASSWORD_LENGTH           0x041A  // 1050 in decimal
#define WIFI_PASSWORD_LOCATION         0x041B  // 1051 in decimal

#define AUTO_CLOSE_TIME_LOCATION       0x05DC   //1500 in decimal

//---------------------------------------end--------------------------------------------//

/**********************************************************************************************************
 * Function name  :   Write_EEPROM()
 *
 * Description    :   1. This function will write the given bytes to the respective EEPROM's memory location
 *                       
 *
 * Params         :   1. <in> uint16_t mem_location : Starting address of data to be written
 *
 *                    2. <in> uint8_t * data        : Pointer to data buffer to be written in EEPROM
 *
 *                    3. <in> uint32_t num          : Number of bytes to be written in EEPROM. 
 *
 * Returns        :   0 on success, otherwise error code
 *
 ***********************************************************************************************************/
int Write_EEPROM(uint16_t mem_location, uint8_t *data, uint32_t num);

/**********************************************************************************************************
 * Function name  :   Read_EEPROM()
 *
 * Description    :   1. This function will read the given number of bytes from EEPROM to given buffer                     
 *
 * Params         :   1. <in> uint16_t mem_location : Starting address from where to read
 *
 *                    2. <out> void * data          : Pointer to data buffer to write the read values
 *
 *                    3. <in> uint32_t num          : Number of bytes to be read from EEPROM. 
 *
 * Returns        :   0 on success, otherwise error code
 *
 ***********************************************************************************************************/
int Read_EEPROM(uint16_t mem_location, void *read_buf, uint32_t num);
#endif