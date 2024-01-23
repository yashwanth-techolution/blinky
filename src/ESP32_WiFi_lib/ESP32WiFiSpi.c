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

#include "ESP32WiFiSpi.h"
#include "utility/ESP32espspi_drv.h"
#include "utility/ESP32wifispi_drv.h"
#include "utility/wl_definitions.h"
#include "utility/wl_types.h"

// Protocol version
const char *protocolVer = "0.3.0";

// Hardware reset pin
int8_t hwResetPin = -1;

// No assumptions about the value of MAX_SOCK_NUM
int16_t _state[MAX_SOCK_NUM] = {0};
uint16_t _server_port[MAX_SOCK_NUM] = {0};

void WiFiSpi_Socket_init() {
  // Initialize the connection arrays
  for (uint8_t i = 0; i < MAX_SOCK_NUM; ++i) {
    _state[i] = NA_STATE;
    _server_port[i] = 0;
  }
}

/* SPI intit */
void WiFiSpi_init(int8_t pin, int8_t HWResetPin) {
  // Initialize the connection arrays
  for (uint8_t i = 0; i < MAX_SOCK_NUM; ++i) {
    _state[i] = NA_STATE;
    _server_port[i] = 0;
  }

  WiFiSpiDrv_wifiDriverInit(pin, HWResetPin);
  hwResetPin = HWResetPin;
  espSpiProxy_hardReset(hwResetPin);
}

uint8_t WiFiSpi_getSocket() {
  for (uint8_t i = 0; i < MAX_SOCK_NUM; ++i) {
    if (_state[i] == NA_STATE) { // _state is for both server and client
      return i;
    }
  }
  return SOCK_NOT_AVAIL;
}

/*
 *
 */
const char *WiFiSpi_firmwareVersion() {
  return WiFiSpiDrv_getFwVersion();
}

/*
 *
 */
uint8_t WiFiSpi_begin_with_ssid(const char *ssid, bool *stop_flag) {
  uint8_t status = WL_IDLE_STATUS;
  uint8_t attempts = WL_MAX_ATTEMPT_CONNECTION;

  if (WiFiSpiDrv_wifiSetNetwork(ssid, strlen(ssid), stop_flag) != WL_FAILURE) {
    do {
      k_msleep(WL_DELAY_START_CONNECTION);
      status = WiFiSpiDrv_getConnectionStatus(stop_flag);
    } while (((status == WL_IDLE_STATUS) || (status == WL_SCAN_COMPLETED) ||
        (status == WL_DISCONNECTED)) && (--attempts > 0));
  } else {
    status = WL_CONNECT_FAILED;
  }
  return status;
}

/*
 *
 */
uint8_t WiFiSpi_begin(const char *ssid, const char *passphrase, bool *stop_flag) {
  uint8_t status = WL_IDLE_STATUS;
  uint8_t attempts = WL_MAX_ATTEMPT_CONNECTION;

  // SSID and passphrase for WPA connection
  if (WiFiSpiDrv_wifiSetPassphrase(ssid, strlen(ssid), passphrase,
        strlen(passphrase), stop_flag) != WL_FAILURE) {
    do {
      k_msleep(WL_DELAY_START_CONNECTION); // added by samuel
      status = WiFiSpiDrv_getConnectionStatus(stop_flag);
    } while (((status == WL_IDLE_STATUS) || (status == WL_SCAN_COMPLETED)
        || (status == WL_DISCONNECTED)) && (--attempts > 0));
  } else {
    status = WL_CONNECT_FAILED;
  }
  return status;
}

/*
 *
 */
int WiFiSpi_disconnect() {
  return WiFiSpiDrv_disconnect();
}

/*
 *
 */
uint8_t *WiFiSpi_macAddress(uint8_t *mac) {
  uint8_t *_mac = WiFiSpiDrv_getMacAddress();
  memcpy(mac, _mac, WL_MAC_ADDR_LENGTH);
  return mac;
}

/*
 *
 */
IPAddress WiFiSpi_localIP() {
  IPAddress ret;
  WiFiSpiDrv_getIpAddress(&ret);
  return ret;
}

/*
 *
 */
IPAddress WiFiSpi_subnetMask() {
  IPAddress ret;
  WiFiSpiDrv_getSubnetMask(&ret);
  return ret;
}

/*
 *
 */
IPAddress WiFiSpi_gatewayIP() {
  IPAddress ret;
  WiFiSpiDrv_getGatewayIP(&ret);
  return ret;
}

/*
 *
 */
char *WiFiSpi_SSID() {
  return WiFiSpiDrv_getCurrentSSID();
}

/*
 *
 */
uint8_t *WiFiSpi_BSSID() {
  return WiFiSpiDrv_getCurrentBSSID();
}

/*
 *
 */
int32_t WiFiSpi_RSSI() {
  return WiFiSpiDrv_getCurrentRSSI();
}

uint8_t WiFiSpi_encryptionType() {
  return 0;//WiFiSpiDrv_getCurrentEncryptionType();
}

/*
 *
 */
int8_t WiFiSpi_scanNetworks() {
#define WIFI_SCAN_RUNNING (-1)
#define WIFI_SCAN_FAILED (-2)

  uint8_t attempts = 10;
  int8_t numOfNetworks = 0;

  if (WiFiSpiDrv_startScanNetworks() == WIFI_SCAN_FAILED) {
    return WL_FAILURE;
  }

  do {
    k_msleep(6000U); // added by samuel
    numOfNetworks = WiFiSpiDrv_getScanNetworks();

    if (numOfNetworks == WIFI_SCAN_FAILED) {
      return WL_FAILURE;
    }
  } 
  while ((numOfNetworks == WIFI_SCAN_RUNNING) && (--attempts > 0));
  return numOfNetworks;
}

/*
 *
 */
char *WiFiSpi_SSID_with_index(uint8_t networkItem) {
  return WiFiSpiDrv_getSSIDNetworks(networkItem);
}

/*
 *
 */
int32_t WiFiSpi_RSSI_with_index(uint8_t networkItem) {
  return WiFiSpiDrv_getRSSINetworks(networkItem);
}

/*
 *
 */
uint8_t WiFiSpi_encryptionType_with_index(uint8_t networkItem) {
  return WiFiSpiDrv_getEncTypeNetworks(networkItem);
}

/*
 *
 */
uint8_t WiFiSpi_status(bool *stop_flag) {
  return WiFiSpiDrv_getConnectionStatus(stop_flag);
}

/*
 *
 */
int8_t WiFiSpi_hostByName(const char *aHostname, IPAddress *aResult, bool *stop_flag) {
  return WiFiSpiDrv_getHostByName(aHostname, aResult, stop_flag);
}

/*
 * Perform remote software reset of the ESP8266 module.
 * The reset succeedes only if the SPI communication is not broken.
 * The function does not wait for the ESP8266.
 */
void WiFiSpi_softReset(void) {
  WiFiSpiDrv_softReset();
}

/*
 *
 */
const char *WiFiSpi_protocolVersion() {
  return WiFiSpiDrv_getProtocolVersion();
}

/*
 *
 */
uint8_t WiFiSpi_checkProtocolVersion() {
  const char *s = WiFiSpiDrv_getProtocolVersion();
  for (const char *p = protocolVer; *p; ++p, ++s) {
    if (*p != *s) {
      return 0;
    }
  }
  return (*s == 0);
}

/*
 *
 */
void WiFiSpi_hardReset(void) {
  espSpiProxy_hardReset(hwResetPin);
}

// --------------------- ESP32 WiFi Module Self OTA APIs --------------------------

int WiFiSpi_startSelfOTAServer(void) {
  return WiFiSpiDrv_startSelfOTAServer();
}

int WiFiSpi_stopSelfOTAServer(void) {
  return WiFiSpiDrv_stopSelfOTAServer();
}

int WiFiSpi_getSelfOTAStatus(uint8_t *OTAPercent) {
  return WiFiSpiDrv_getSelfOTAStatus(OTAPercent);
}