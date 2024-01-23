#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <errno.h>
#include <hal/nrf_gpio.h>
#include <inttypes.h>
#include <zephyr/kernel.h>
#include <stdio.h>
#include <stdlib.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "ESP32espspi_drv.h"
#include "ESP32srvspi_drv.h"

#include "wl_types.h"

///*
//    Start server TCP / UDP on port specified
//*/
// bool ServerSpiDrv::startServer(uint16_t port, uint8_t sock, uint8_t protMode)
//{
//    // Send Command
//    EspSpiDrv::sendCmd(START_SERVER_TCP_CMD, PARAM_NUMS_3);
//    EspSpiDrv::sendParam(reinterpret_cast<const uint8_t*>(&port), sizeof(port));
//    EspSpiDrv::sendParam(sock);
//    EspSpiDrv::sendParam(protMode);
//    EspSpiDrv::endCmd();

//    uint8_t _data = 0;
//    uint8_t _dataLen = sizeof(_data);
//    if (!EspSpiDrv::waitResponseCmd(START_SERVER_TCP_CMD, PARAM_NUMS_1, &_data, &_dataLen))
//    {
//        WARN(FPSTR(WiFiSpiDrv::ERROR_WAITRESPONSE));
//        _data = 0;
//    }

//    return (_data == 1);  // return value 1 means ok
//}

///*
//    Start server UDP Multicast on port specified listening given ip address
//*/
// bool ServerSpiDrv::startServerMulticast(const uint32_t ipAddress, const uint16_t port, const uint8_t sock)
//{
//    // Send Command
//    EspSpiDrv::sendCmd(START_SERVER_MULTICAST_CMD, PARAM_NUMS_3);
//    EspSpiDrv::sendParam(reinterpret_cast<const uint8_t*>(&ipAddress), sizeof(ipAddress));
//    EspSpiDrv::sendParam(reinterpret_cast<const uint8_t*>(&port), sizeof(port));
//    EspSpiDrv::sendParam(sock);
//    EspSpiDrv::endCmd();

//    uint8_t _data = 0;
//    uint8_t _dataLen = sizeof(_data);
//    if (!EspSpiDrv::waitResponseCmd(START_SERVER_MULTICAST_CMD, PARAM_NUMS_1, &_data, &_dataLen))
//    {
//        WARN(FPSTR(WiFiSpiDrv::ERROR_WAITRESPONSE));
//        _data = 0;
//    }

//    return (_data == 1);  // return value 1 means ok
//}

///*
// *
// */
// void ServerSpiDrv::stopServer(uint8_t sock)
//{
//    // Send Command
//    EspSpiDrv::sendCmd(STOP_SERVER_TCP_CMD, PARAM_NUMS_1);
//    EspSpiDrv::sendParam(sock);
//    EspSpiDrv::endCmd();

//    if (!EspSpiDrv::waitResponseCmd(STOP_SERVER_TCP_CMD, PARAM_NUMS_0, NULL, NULL))
//    {
//        WARN(FPSTR(WiFiSpiDrv::ERROR_WAITRESPONSE));
//    }
//}

/*
    Start client TCP on port specified
    Note: protMode is ignored
*/
bool ServerSpiDrv_startClient(uint32_t ipAddress, uint16_t port,
    uint8_t sock, uint8_t protMode, bool *stop_flag) {
  // printk("ipaddress %d", ipAddress);
  //  Send Command
  EspSpiDrv_sendCmd(START_CLIENT_TCP_CMD, PARAM_NUMS_4);
  EspSpiDrv_sendParam_1byte(protMode);
  EspSpiDrv_sendParam((const uint8_t *)(&port), sizeof(port));
  EspSpiDrv_sendParam_1byte(sock);
  EspSpiDrv_sendParam((const uint8_t *)(&ipAddress), sizeof(ipAddress));
  EspSpiDrv_endCmd();

  uint8_t _data = 0;
  uint8_t _dataLen = sizeof(_data);
  if (!EspSpiDrv_waitResponseCmd(START_CLIENT_TCP_CMD, PARAM_NUMS_1,
          &_data, &_dataLen, stop_flag)) {
    printk("Error waitResponse\n");
    _data = 0;
  }
  return (_data == 1); // return value 1 means ok
}

bool ServerSpiDrv_startClientURL(const uint8_t *url, uint16_t port,
    uint8_t sock, uint8_t protMode, bool *stop_flag) {

  EspSpiDrv_sendCmd(START_CLIENT_TCP_CMD, PARAM_NUMS_4);
  EspSpiDrv_sendParam_1byte(protMode);
  EspSpiDrv_sendParam((const uint8_t *)(&port), sizeof(port));
  EspSpiDrv_sendParam_1byte(sock);
  EspSpiDrv_sendParam((const uint8_t *)url, strlen(url));
  EspSpiDrv_endCmd();

  uint8_t _data = 0;
  uint8_t _dataLen = sizeof(_data);
  if (!EspSpiDrv_waitResponseCmd(START_CLIENT_TCP_CMD, PARAM_NUMS_1,
          &_data, &_dataLen, stop_flag)) {
    printk("Error waitResponse\n");
    _data = 0;
  }
  return (_data == 1); // return value 1 means ok
}

/*
 *
 */
void ServerSpiDrv_stopClient(uint8_t sock, bool *stop_flag) {
  // Send Command
  EspSpiDrv_sendCmd(STOP_CLIENT_TCP_CMD, PARAM_NUMS_1);
  EspSpiDrv_sendParam_1byte(sock);
  EspSpiDrv_endCmd();

  uint8_t _data = 0;
  uint8_t _dataLen = sizeof(_data);
  if (!EspSpiDrv_waitResponseCmd(STOP_CLIENT_TCP_CMD, PARAM_NUMS_1,
          &_data, &_dataLen, stop_flag)) {
    printk("Error waitResponse\n");
  }
}

/*
 *
 */
uint8_t ServerSpiDrv_getClientState(const uint8_t sock, bool *stop_flag) {
  // Send Command
  EspSpiDrv_sendCmd(GET_CLIENT_STATE_TCP_CMD, PARAM_NUMS_1);
  EspSpiDrv_sendParam_1byte(sock);
  EspSpiDrv_endCmd();

  // Wait for reply
  uint8_t _data = -1;
  uint8_t _dataLen = sizeof(_data);
  if (!EspSpiDrv_waitResponseCmd(GET_CLIENT_STATE_TCP_CMD,
          PARAM_NUMS_1, &_data, &_dataLen, stop_flag)) {
    printk("Error waitResponse\n");
    _data = 0;
  }
  return (_data ? ESTABLISHED : CLOSED);
}

/*
 *
 */
uint16_t ServerSpiDrv_availData(const uint8_t sock, bool *stop_flag) {
  // Send Command
  EspSpiDrv_sendCmd(AVAIL_DATA_TCP_CMD, PARAM_NUMS_1);
  EspSpiDrv_sendParam_1byte(sock);
  EspSpiDrv_endCmd();

  // Wait for reply
  uint16_t _data16 = 0;
  uint8_t _dataLen = sizeof(_data16);
  if (!EspSpiDrv_waitResponseCmd(AVAIL_DATA_TCP_CMD, PARAM_NUMS_1,
          (uint8_t *)(&_data16), &_dataLen, stop_flag)) {
    printk("Error waitResponse\n");
    _data16 = 0;
  }
  return _data16;
}

/*
 *
 */
bool ServerSpiDrv_getData(const uint8_t sock, int16_t *data, const uint8_t peek) {
  // Send Command
  EspSpiDrv_sendCmd(GET_DATA_TCP_CMD, PARAM_NUMS_2);
  EspSpiDrv_sendParam_1byte(sock);
  EspSpiDrv_sendParam_1byte(peek);
  EspSpiDrv_endCmd();

  // Wait for reply
  int16_t _data16 = -1; // -1 is error indicator
  uint8_t _dataLen = sizeof(_data16);
  if (!EspSpiDrv_waitResponseCmd(GET_DATA_TCP_CMD, PARAM_NUMS_1,
          (uint8_t *)(&_data16), &_dataLen, NULL)) {
    printk("Error waitResponse\n");
    _dataLen = 0;
  } else {
    *data = _data16;
  }
  return (_dataLen == sizeof(_data16));
}

/*
    Reads data into a buffer. There must be enough data for the whole buffer or the return code is false (error).
    It is not very clever but it is how the original WiFi library works.
 */
bool ServerSpiDrv_getDataBuf(const uint8_t sock, uint8_t *_data,
    uint16_t *_dataLen, bool *stop_flag) {
  // Send Command
  EspSpiDrv_sendCmd(GET_DATABUF_TCP_CMD, PARAM_NUMS_2);
  EspSpiDrv_sendParam_1byte(sock);
  EspSpiDrv_sendParam((uint8_t *)(_dataLen), sizeof(*_dataLen));
  EspSpiDrv_endCmd();

  // Wait for reply (length is 16 bit integer)
  uint16_t _dataLenRead = *_dataLen;
  if (!EspSpiDrv_waitResponseCmd16(GET_DATABUF_TCP_CMD, PARAM_NUMS_1,
          _data, &_dataLenRead, stop_flag)) {
    printk("Error waitResponse\n");
    _dataLenRead = 0;
  }

  if (_dataLenRead != *_dataLen) {
    return false;
  }
  return true;
}

/*
 *
 */
bool ServerSpiDrv_sendData(const uint8_t sock, const uint8_t *data,
    const uint16_t len, bool *stop_flag) {
  // Send Command
  EspSpiDrv_sendCmd(SEND_DATA_TCP_CMD, PARAM_NUMS_2);
  EspSpiDrv_sendParam_1byte(sock);
  EspSpiDrv_sendBuffer(data, len);
  EspSpiDrv_endCmd();

  // Wait for reply
  uint16_t _data16;
  uint8_t _dataLen = sizeof(_data16);
  if (!EspSpiDrv_waitResponseCmd(SEND_DATA_TCP_CMD, PARAM_NUMS_1,
          (uint8_t *)(&_data16), &_dataLen, stop_flag)) {
    printk("Error waitResponse\n");
    _data16 = 0;
  }
  return (_dataLen == sizeof(_data16) && _data16 == len);
}