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
#include "ESP32wifispi_drv.h"
#include "wl_types.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Array of data to cache the information related to the networks discovered
char _networkSsid[32] = {0};
int32_t _networkRssi = 0;
uint8_t _networkEncr = 0;

// Cached values of retrieved data
char _ssid[32] = {0};
uint8_t _bssid[6] = {0};
uint8_t _mac[6] = {0};
uint8_t _localIp[6] = {0};
uint8_t _subnetMask[6] = {0};
uint8_t _gatewayIp[6] = {0};

// Firmware and protocol version
char fwVersion[6] = {0};
char protocolVersion[6] = {0};

/*
 *
 */
int8_t WiFiSpiDrv_getNetworkData(uint8_t *ip, uint8_t *mask, uint8_t *gwip) {
  tParam params[PARAM_NUMS_3] = {{WL_IPV4_LENGTH, (char *)ip}, {WL_IPV4_LENGTH,
      (char *)mask}, {WL_IPV4_LENGTH, (char *)gwip}};

  // Send Command
  EspSpiDrv_sendCmd(GET_IPADDR_CMD, PARAM_NUMS_0);

  if (!EspSpiDrv_waitResponseParams(GET_IPADDR_CMD, PARAM_NUMS_3, params, NULL)) {
    printk("Error waitResponse\n");
    return WL_FAILURE;
  }
  return WL_SUCCESS;
}

/*
 *
 */
int8_t WiFiSpiDrv_getScannedData(uint8_t networkItem, char *ssid,
    int32_t *rssi, uint8_t *encr) {
  tParam params[PARAM_NUMS_3] = {{WL_SSID_MAX_LENGTH, (char *)ssid},
        {sizeof(*rssi), (char *)rssi}, {sizeof(*encr), (char *)encr}};

  // Send Command
  EspSpiDrv_sendCmd(GET_SCANNED_DATA_CMD, PARAM_NUMS_1);
  EspSpiDrv_sendParam_1byte(networkItem);
  EspSpiDrv_endCmd();
  ssid[0] = 0;

  if (!EspSpiDrv_waitResponseParams(GET_SCANNED_DATA_CMD, PARAM_NUMS_3, params, NULL)) {
    printk("Error waitResponse\n");
    return WL_FAILURE;
  }

  ssid[params[0].paramLen] = 0;
  return WL_SUCCESS;
}

/*
 *
 */
bool WiFiSpiDrv_getRemoteData(uint8_t sock, uint8_t *ip, uint16_t *port) {
  tParam params[PARAM_NUMS_2] = {{4, (char *)ip}, {sizeof(*port), (char *)port}};

  // Send Command
  EspSpiDrv_sendCmd(GET_REMOTE_DATA_CMD, PARAM_NUMS_1);
  EspSpiDrv_sendParam_1byte(sock);
  EspSpiDrv_endCmd();

  if (!EspSpiDrv_waitResponseParams(GET_REMOTE_DATA_CMD, PARAM_NUMS_2, params, NULL)) {
    printk("Error waitResponse\n");
    return false;
  }
  return true;
}

/*
 *
 */
void WiFiSpiDrv_wifiDriverInit(uint8_t cs_pin, uint8_t hwresetPin) {
  espSpiProxy_begin(cs_pin, hwresetPin);
}

/*
 *
 */
int8_t WiFiSpiDrv_wifiSetNetwork(const char *ssid, uint8_t ssid_len, bool *stop_flag) {
  // Test the input
  if (ssid_len > WL_SSID_MAX_LENGTH) {
    return WL_FAILURE;
  }

  // Send Command
  EspSpiDrv_sendCmd(SET_NET_CMD, PARAM_NUMS_1);
  EspSpiDrv_sendParam((const uint8_t *)(ssid), ssid_len);
  EspSpiDrv_endCmd();

  // Wait for reply
  uint8_t _data = -1;
  uint8_t _dataLen = sizeof(_data);
  if (!EspSpiDrv_waitResponseCmd(SET_NET_CMD, PARAM_NUMS_1, &_data, &_dataLen, stop_flag)) {
    printk("Error waitResponse\n");
    _data = WL_FAILURE;
  }
  return _data;
}

/*
 * Connects to AP with given parameters
 * Returns: status - see getConnectionStatus()
 */
uint8_t WiFiSpiDrv_wifiSetPassphrase(const char *ssid, const uint8_t ssid_len,
    const char *passphrase, const uint8_t len, bool *stop_flag) {
  // Test the input
  if (ssid_len > WL_SSID_MAX_LENGTH || len > WL_WPA_KEY_MAX_LENGTH) {
    return WL_FAILURE;
  }

  // Send Command
  EspSpiDrv_sendCmd(SET_PASSPHRASE_CMD, PARAM_NUMS_2);
  EspSpiDrv_sendParam((const uint8_t *)(ssid), ssid_len);
  EspSpiDrv_sendParam((const uint8_t *)(passphrase), len);
  EspSpiDrv_endCmd();

  // Wait for reply
  uint8_t _data = -1;
  uint8_t _dataLen = sizeof(_data);
  if (!EspSpiDrv_waitResponseCmd(SET_PASSPHRASE_CMD, PARAM_NUMS_1,
        &_data, &_dataLen, stop_flag)) {
    printk("Error waitResponse\n");
    _data = WL_FAILURE;
  }
  return _data;
}

/*
 *
 */
uint8_t WiFiSpiDrv_disconnect() {
  // Send Command
  EspSpiDrv_sendCmd(DISCONNECT_CMD, PARAM_NUMS_0);

  // Wait for reply
  uint8_t _data = -1;
  uint8_t _dataLen = sizeof(_data);
  if (!EspSpiDrv_waitResponseCmd(DISCONNECT_CMD, PARAM_NUMS_1,
        &_data, &_dataLen, NULL)) {
    printk("Error waitResponse\n");
    _data = WL_FAILURE;
  }
  return _data;
}

/*
 *
 */
uint8_t WiFiSpiDrv_getConnectionStatus(bool *stop_flag) {
  // Send Command
  EspSpiDrv_sendCmd(GET_CONN_STATUS_CMD, PARAM_NUMS_0);

  // Wait for reply
  uint8_t _data = -1;
  uint8_t _dataLen = sizeof(_data);
  if (!EspSpiDrv_waitResponseCmd(GET_CONN_STATUS_CMD, PARAM_NUMS_1,
        &_data, &_dataLen, stop_flag)) {
    printk("Error waitResponse\n");
  }
  return _data;
}

/*
 *
 */
uint8_t *WiFiSpiDrv_getMacAddress() {
  // Send Command
  EspSpiDrv_sendCmd(GET_MACADDR_CMD, PARAM_NUMS_0);

  // Wait for reply
  uint8_t _dataLen = WL_MAC_ADDR_LENGTH;
  if (!EspSpiDrv_waitResponseCmd(GET_MACADDR_CMD, PARAM_NUMS_1,
        _mac, &_dataLen, NULL)) {
    printk("Error waitResponse\n");
  }

  if (_dataLen != WL_MAC_ADDR_LENGTH) {
    printk("Error badReply\n");
  }
  return _mac;
}

/*
 *
 */
int8_t WiFiSpiDrv_getIpAddress(IPAddress *ip) {
  int8_t status = WiFiSpiDrv_getNetworkData(_localIp, _subnetMask, _gatewayIp);
  if (status == WL_SUCCESS) {
    ip->first_byte = _localIp[0];
    ip->second_byte = _localIp[1];
    ip->third_byte = _localIp[2];
    ip->fourth_byte = _localIp[3];
  }
  return status;
}

/*
 *
 */
int8_t WiFiSpiDrv_getSubnetMask(IPAddress *mask) {
  int8_t status = WiFiSpiDrv_getNetworkData(_localIp, _subnetMask, _gatewayIp);

  if (status == WL_SUCCESS) {
    mask->first_byte = _subnetMask[0];
    mask->second_byte = _subnetMask[1];
    mask->third_byte = _subnetMask[2];
    mask->fourth_byte = _subnetMask[3];
  }
  return status;
}

/*
 *
 */
int8_t WiFiSpiDrv_getGatewayIP(IPAddress *ip) {
  int8_t status = WiFiSpiDrv_getNetworkData(_localIp, _subnetMask, _gatewayIp);

  if (status == WL_SUCCESS) {
    ip->first_byte = _gatewayIp[0];
    ip->second_byte = _gatewayIp[1];
    ip->third_byte = _gatewayIp[2];
    ip->fourth_byte = _gatewayIp[3];
  }
  return status;
}

/*
 *
 */
char *WiFiSpiDrv_getCurrentSSID() {
  // Send Command
  EspSpiDrv_sendCmd(GET_CURR_SSID_CMD, PARAM_NUMS_0);

  // Wait for reply
  uint8_t _dataLen = WL_SSID_MAX_LENGTH;

  if (!EspSpiDrv_waitResponseCmd(GET_CURR_SSID_CMD, PARAM_NUMS_1,
        (uint8_t *)(_ssid), &_dataLen, NULL)) {
    printk("Error waitResponse\n");
  }

  _ssid[_dataLen] = 0;
  return _ssid;
}

/*
 *
 */
uint8_t *WiFiSpiDrv_getCurrentBSSID() {
  // Send Command
  EspSpiDrv_sendCmd(GET_CURR_BSSID_CMD, PARAM_NUMS_0);

  // Wait for reply
  uint8_t _dataLen = WL_MAC_ADDR_LENGTH;

  if (!EspSpiDrv_waitResponseCmd(GET_CURR_BSSID_CMD, PARAM_NUMS_1,
        (uint8_t *)_bssid, &_dataLen, NULL)) {
    printk("Error waitResponse\n");
  }

  if (_dataLen != WL_MAC_ADDR_LENGTH) {
    printk("Error badReply\n");
  }
  return _bssid;
}

/*
 *
 */
int32_t WiFiSpiDrv_getCurrentRSSI() {
  // Send Command
  EspSpiDrv_sendCmd(GET_CURR_RSSI_CMD, PARAM_NUMS_0);

  // Wait for reply
  int32_t _rssi;
  uint8_t _dataLen = sizeof(_rssi);

  if (!EspSpiDrv_waitResponseCmd(GET_CURR_RSSI_CMD, PARAM_NUMS_1,
        (uint8_t *)(&_rssi), &_dataLen, NULL)) {
    printk("Error waitResponse\n");
    _dataLen = 0;
  }

  if (_dataLen != sizeof(_rssi)) {
    printk("Error badReply\n");
  }
  return _rssi;
}

/*
 *
 */
int8_t WiFiSpiDrv_startScanNetworks() {
  // Send Command
  EspSpiDrv_sendCmd(START_SCAN_NETWORKS, PARAM_NUMS_0);
  int8_t _data = -1;
  uint8_t _dataLen = sizeof(_data);

  if (!EspSpiDrv_waitResponseCmd(START_SCAN_NETWORKS, PARAM_NUMS_1,
        (uint8_t *)(&_data), &_dataLen, NULL)) {
    printk("Error waitResponse\n");
  }
  return _data;
}

/*
 *
 */
int8_t WiFiSpiDrv_getScanNetworks() {
  // Send Command
  EspSpiDrv_sendCmd(SCAN_NETWORKS, PARAM_NUMS_0);
  int8_t _data = -1;
  uint8_t _dataLen = sizeof(_data);

  if (!EspSpiDrv_waitResponseCmd(SCAN_NETWORKS, PARAM_NUMS_1,
        (uint8_t *)(&_data), &_dataLen, NULL)) {
    printk("Error waitResponse\n");
  }
  return _data;
}

/*
 *
 */
char *WiFiSpiDrv_getSSIDNetworks(uint8_t networkItem) {
  int8_t status = WiFiSpiDrv_getScannedData(networkItem, _networkSsid,
      &_networkRssi, &_networkEncr);

  if (status != WL_SUCCESS) {
    _networkSsid[0] = 0; // Empty string
  }
  return _networkSsid;
}

/*
 *
 */
uint8_t WiFiSpiDrv_getEncTypeNetworks(uint8_t networkItem) {
  int8_t status = WiFiSpiDrv_getScannedData(networkItem, _networkSsid,
      &_networkRssi, &_networkEncr);
  if (status != WL_SUCCESS) {
    return 0;
  }
  return _networkEncr;
}

/*
 *
 */
int32_t WiFiSpiDrv_getRSSINetworks(uint8_t networkItem) {
  int8_t status = WiFiSpiDrv_getScannedData(networkItem, _networkSsid,
      &_networkRssi, &_networkEncr);
  if (status != WL_SUCCESS) {
    return 0;
  }
  return _networkRssi;
}

/*
 *
 */
int8_t WiFiSpiDrv_getHostByName(const char *aHostname, IPAddress *aResult, bool *stop_flag) {
  // Send Command
  EspSpiDrv_sendCmd(GET_HOST_BY_NAME_CMD, PARAM_NUMS_1);
  EspSpiDrv_sendParam((const uint8_t *)(aHostname), strlen(aHostname));
  EspSpiDrv_endCmd();

  // Wait for reply
  uint8_t _ipAddr[WL_IPV4_LENGTH];
  uint8_t _status;
  tParam params[PARAM_NUMS_2] = {{sizeof(_status), (char *)&_status},
        {sizeof(_ipAddr), (char *)_ipAddr}};

  for (int i = 1; i < 2; ++i) {
    if (!waitForSlaveTxReady(stop_flag)) {
      // TODO: add proper return type.
      break;
    }
  }

  if (!EspSpiDrv_waitResponseParams(GET_HOST_BY_NAME_CMD, PARAM_NUMS_2, params, stop_flag)) {
    printk("Error waitResponse\n");
    return WL_FAILURE;
  }

  if (params[0].paramLen != sizeof(_status) || params[1].paramLen != sizeof(_ipAddr)) {
    printk("Error badReply\n");
    return 0;
  }

  aResult->first_byte = _ipAddr[0];
  aResult->second_byte = _ipAddr[1];
  aResult->third_byte = _ipAddr[2];
  aResult->fourth_byte = _ipAddr[3];
  return _status;
}

/*
 *
 */
const char *WiFiSpiDrv_getFwVersion() {
  // Send Command
  EspSpiDrv_sendCmd(GET_FW_VERSION_CMD, PARAM_NUMS_0);

  // Wait for reply
  uint8_t _dataLen = WL_FW_VER_LENGTH;

  if (!EspSpiDrv_waitResponseCmd(GET_FW_VERSION_CMD, PARAM_NUMS_1,
      (uint8_t *)fwVersion, &_dataLen, NULL)) {
    printk("Error waitResponse\n");
  }
  return fwVersion;
}

/*
 * Perform remote software reset of the ESP8266 module.
 * The reset succeedes only if the SPI communication is not broken.
 * The function does not wait for the ESP8266.
 */
void WiFiSpiDrv_softReset(void) {
  // Send Command
  EspSpiDrv_sendCmd(SOFTWARE_RESET_CMD, PARAM_NUMS_0);

  // Wait for reply
  if (!EspSpiDrv_waitResponseCmd(SOFTWARE_RESET_CMD, PARAM_NUMS_0, NULL, NULL, NULL)) {
    printk("Error waitResponse\n");
  }
}

/*
 *
 */
const char *WiFiSpiDrv_getProtocolVersion() {
  // Send Command
  EspSpiDrv_sendCmd(GET_PROTOCOL_VERSION_CMD, PARAM_NUMS_0);

  // Wait for reply
  uint8_t _dataLen = WL_PROTOCOL_VER_LENGTH;
  if (!EspSpiDrv_waitResponseCmd(GET_PROTOCOL_VERSION_CMD, PARAM_NUMS_1,
      (uint8_t *)protocolVersion, &_dataLen, NULL)) {
    printk("Error waitResponse\n");
  }
  return protocolVersion;
}

// ------------------------------ ESP32 WiFi Module Self OTA APIs ---------------------------------------

/*
 *
 */
int8_t WiFiSpiDrv_startSelfOTAServer() {
  // Send Command
  EspSpiDrv_sendCmd(START_ESP32_SELF_OTA, PARAM_NUMS_0);

  int8_t _data = -1;
  uint8_t _dataLen = sizeof(_data);
  if (!EspSpiDrv_waitResponseCmd(START_ESP32_SELF_OTA, PARAM_NUMS_1,
      (uint8_t *)(&_data), &_dataLen, NULL)) {
    printk("Error waitResponse\n");
  }
  return _data;
}

/*
 *
 */
int8_t WiFiSpiDrv_stopSelfOTAServer() {
  // Send Command
  EspSpiDrv_sendCmd(STOP_ESP32_SELF_OTA, PARAM_NUMS_0);

  int8_t _data = -1;
  uint8_t _dataLen = sizeof(_data);
  if (!EspSpiDrv_waitResponseCmd(STOP_ESP32_SELF_OTA, PARAM_NUMS_1,
      (uint8_t *)(&_data), &_dataLen, NULL)) {
    printk("Error waitResponse\n");
  }
  return _data;
}

/*
 *
 */
int8_t WiFiSpiDrv_getSelfOTAStatus(uint8_t *OTAPercent) {
  // Send Command
  EspSpiDrv_sendCmd(GET_ESP32_SELF_OTA_STATUS, PARAM_NUMS_0);

  // Wait for reply
  int8_t _status = -1;
  tParam params[PARAM_NUMS_2] = {{sizeof(_status), (char *)&_status},
      {1, (char *)OTAPercent}};

  if (!EspSpiDrv_waitResponseParams(GET_ESP32_SELF_OTA_STATUS,
          PARAM_NUMS_2, params, NULL)) {
    printk("Error waitResponse\n");
  }

  if (params[0].paramLen != sizeof(_status) || params[1].paramLen != 1) {
    printk("Error badReply\n");
  }
  return _status;
}