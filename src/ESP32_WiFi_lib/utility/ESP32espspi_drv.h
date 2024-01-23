
/*
            ESP8266         |
    GPIO    NodeMCU   Name  |   Uno
   ===================================
     15       D8       SS   |   D10**
     13       D7      MOSI  |   D11
     12       D6      MISO  |   D12
     14       D5      SCK   |   D13

*/

#ifndef _ESPSPI_DRV_H_INCLUDED
#define _ESPSPI_DRV_H_INCLUDED


#include <hal/nrf_gpio.h> // added by samuel


#include "Includes/SPI_Module.h"
#include "ESP32wifispi_drv.h"


// The command codes are fixed by ESP8266 hardware
#define CMD_WRITESTATUS  0x01
#define CMD_WRITEDATA    0x02
#define CMD_READDATA     0x03
#define CMD_READSTATUS   0x04

// Message indicators
#define MESSAGE_FINISHED     0xDF
#define MESSAGE_CONTINUES    0xDC

// SPI Status
enum {
    SPISLAVE_RX_BUSY,
    SPISLAVE_RX_READY,
    SPISLAVE_RX_CRC_PROCESSING,
    SPISLAVE_RX_ERROR
};
enum {
    SPISLAVE_TX_NODATA,
    SPISLAVE_TX_READY,
    SPISLAVE_TX_PREPARING_DATA,
    SPISLAVE_TX_WAITING_FOR_CONFIRM
};

//// How long we will wait for slave to be ready
#define SLAVE_TX_READY_TIMEOUT_WIFI_BEGIN     1000UL
#define SLAVE_TX_READY_TIMEOUT     3000UL



void EspSpiDrv_sendCmd(const uint8_t cmd, const uint8_t numParam);
void EspSpiDrv_endCmd();

void EspSpiDrv_sendParam(const uint8_t* param, const uint8_t param_len);
void EspSpiDrv_sendParam_1byte(const uint8_t param);

void EspSpiDrv_sendBuffer(const uint8_t* param, const uint16_t param_len);

uint8_t EspSpiDrv_waitResponseCmd(const uint8_t cmd, const uint8_t numParam, uint8_t* param, uint8_t* param_len, bool *stop_flag);
uint8_t EspSpiDrv_waitResponseCmd16(const uint8_t cmd, uint8_t numParam, uint8_t* param, uint16_t* param_len, bool *stop_flag);
int8_t EspSpiDrv_waitResponseParams(const uint8_t cmd, const uint8_t numParam, tParam* params, bool *stop_flag);


void espSpiProxy_begin(uint8_t cs_pin, uint8_t hwresetPin); // added by samuel

void readData(uint8_t* buf, uint16_t *len);

void writeData(uint8_t * data, uint16_t len);

void flush(uint8_t indicator);

void writeByte(uint8_t b);

void writeByte_buf(uint8_t *b, uint16_t len);

uint8_t readByte();

uint8_t readByte_buff(uint8_t *b, uint16_t len);

int8_t waitForSlaveTxReady(bool *stop_flag);

void espSpiProxy_hardReset(int8_t hwResetPin);

#endif

