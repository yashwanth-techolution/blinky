#ifndef _SRVSPI_DRV_H_INCLUDED
#define _SRVSPI_DRV_H_INCLUDED

#include <inttypes.h>
#include "wifi_spi.h"

typedef enum eProtMode {
    TCP_MODE, 
    UDP_MODE, 
    TCP_MODE_WITH_TLS }
tProtMode;


// Start server TCP / UDP on port specified
//static bool startServer(const uint16_t port, const uint8_t sock, const uint8_t protMode=TCP_MODE);

// Start server UDP Multicast on port specified listening given ip address
//static bool startServerMulticast(const uint32_t ipAddress, const uint16_t port, const uint8_t sock);

//static void stopServer(const uint8_t sock);

bool ServerSpiDrv_startClient(const uint32_t ipAddress, const uint16_t port, const uint8_t sock, const uint8_t protMode, bool *stop_flag);

void ServerSpiDrv_stopClient(const uint8_t sock, bool *stop_flag);
                                                                                  
//static uint8_t getServerState(const uint8_t sock);

uint8_t ServerSpiDrv_getClientState(const uint8_t sock, bool *stop_flag);

uint16_t ServerSpiDrv_availData(const uint8_t sock, bool *stop_flag);

bool ServerSpiDrv_getData(const uint8_t sock, int16_t *data, uint8_t peek);

bool ServerSpiDrv_getDataBuf(const uint8_t sock, uint8_t *_data, uint16_t *_dataLen, bool *stop_flag);

//static bool insertDataBuf(const uint8_t sock, const uint8_t *_data, const uint16_t _dataLen);

//static bool sendUdpData(const uint8_t sock);

bool ServerSpiDrv_sendData(const uint8_t sock, const uint8_t *data, const uint16_t len, bool *stop_flag);

//static bool beginUdpPacket(uint32_t ip, uint16_t port, uint8_t sock);

//static uint16_t parsePacket(const uint8_t sock);

//static uint8_t verifySSLClient(const uint8_t sock, uint8_t *fingerprint, const char *host);

//static uint8_t checkDataSent(uint8_t sock);

#endif
