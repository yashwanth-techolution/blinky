#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <zephyr.h>
#include <device.h>
#include <kernel.h>
#include <inttypes.h>
#include <sys/util.h>
#include <devicetree.h>
#include <sys/printk.h>
#include <drivers/spi.h>
#include <hal/nrf_gpio.h>
#include <drivers/gpio.h>

///*
//  WiFiSPI.cpp - Library for Arduino SPI connection to ESP8266
//  Copyright (c) 2017 Jiri Bilek. All rights reserved.

//  -----
  
//  Based on WiFi.cpp - Library for Arduino Wifi shield.
//  Copyright (c) 2011-2014 Arduino LLC.  All right reserved.

//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.

//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.

//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//*/

#include "WiFiSpi.h"
#include "utility/wifispi_drv.h"
#include "utility/espspi_proxy.h"

//extern "C" {
  #include "utility/debug.h"
  #include "utility/wl_types.h"
  #include "utility/wl_definitions.h"
//}

//#define SS 20   // added by samuel

// Protocol version
//const char *WiFiSpiClass::protocolVer = "0.3.0";
const char *protocolVer = "0.3.0";  // added by samuel

//// Hardware reset pin
//int8_t WiFiSpiClass::hwResetPin = -1;
int8_t hwResetPin = -1;  // added by samuel

// No assumptions about the value of MAX_SOCK_NUM
//int16_t 	WiFiSpiClass::_state[MAX_SOCK_NUM];
int16_t _state[MAX_SOCK_NUM] = {0};  // added by samuel

//uint16_t 	WiFiSpiClass::_server_port[MAX_SOCK_NUM];
uint16_t _server_port[MAX_SOCK_NUM] = {0};  // adde by samuel

void WiFiSpi_Socket_init()  // added by samuel
{
  // Initialize the connection arrays
  for (uint8_t i = 0; i < MAX_SOCK_NUM; ++i)
  {
    _state[i] = NA_STATE;
    _server_port[i] = 0;
  }
}

//void WiFiSpiClass::init(int8_t pin, uint32_t max_speed, SPIClass *in_spi, int8_t hwResetPin)
//void WiFiSpi_init(int8_t pin, uint32_t max_speed, int8_t HWResetPin)  // added by samuel
void WiFiSpi_init(int8_t pin, int8_t HWResetPin)  // added by samuel
{
    //// Initialize the connection arrays
    //for (uint8_t i = 0; i < MAX_SOCK_NUM; ++i)
    //{
    //  _state[i] = NA_STATE;
    //  _server_port[i] = 0;
    //}

    /* cmd by samuel */
    //if (pin < 0)
    //    pin = SS;
    /* by samuel */
        
    //WiFiSpiDrv::wifiDriverInit(pin, max_speed, in_spi);
    //WiFiSpiDrv_wifiDriverInit(pin, max_speed);  // added by samuel
    WiFiSpiDrv_wifiDriverInit(pin, HWResetPin);  // added by samuel

    hwResetPin = HWResetPin;
    espSpiProxy_hardReset(hwResetPin);
}


uint8_t WiFiSpi_getSocket()
{
    for (uint8_t i = 0; i < MAX_SOCK_NUM; ++i)
    {
      //if (WiFiSpiClass::_state[i] == NA_STATE)  // _state is for both server and client
      if (_state[i] == NA_STATE)  // added by samuel // _state is for both server and client
      {
        return i;
      }
    }
    return SOCK_NOT_AVAIL;
}

///*
// * 
// */
//const char* WiFiSpiClass::firmwareVersion()
//{
//    return WiFiSpiDrv::getFwVersion();
//}

///*
// * 
// */
//uint8_t WiFiSpiClass::begin(const char* ssid)
//{
//	uint8_t status = WL_IDLE_STATUS;
//	uint8_t attempts = WL_MAX_ATTEMPT_CONNECTION;

//    if (WiFiSpiDrv::wifiSetNetwork(ssid, strlen(ssid)) != WL_FAILURE)
//    {
//	    do
//	    {
//		    //delay(WL_DELAY_START_CONNECTION);
//                    k_msleep(WL_DELAY_START_CONNECTION); // added by samuel
//		    status = WiFiSpiDrv::getConnectionStatus();
//	    }
//	    while (((status == WL_IDLE_STATUS) || (status == WL_SCAN_COMPLETED) || (status == WL_DISCONNECTED)) && (--attempts > 0));
//    } else
//	    status = WL_CONNECT_FAILED;
   
//    return status;
//}

/*
 * 
 */
uint8_t WiFiSpi_begin(const char* ssid, const char *passphrase)
{
    uint8_t status = WL_IDLE_STATUS;
    uint8_t attempts = WL_MAX_ATTEMPT_CONNECTION;

    // SSID and passphrase for WPA connection
    if (WiFiSpiDrv_wifiSetPassphrase(ssid, strlen(ssid), passphrase, strlen(passphrase)) != WL_FAILURE)
    {
      do
      {
        //delay(WL_DELAY_START_CONNECTION);
        k_msleep(WL_DELAY_START_CONNECTION); // added by samuel
        status = WiFiSpiDrv_getConnectionStatus();
      }
      while (((status == WL_IDLE_STATUS) || (status == WL_SCAN_COMPLETED) || (status == WL_DISCONNECTED)) && (--attempts > 0));
    } 
    else
    {
      status = WL_CONNECT_FAILED;
    }
    
    return status;
}

///*
// * 
// */
//bool WiFiSpiClass::config(IPAddress local_ip)
//{
//	return WiFiSpiDrv::config((uint32_t)local_ip, 0, 0, 0, 0);
//}

///*
// * 
// */
//bool WiFiSpiClass::config(IPAddress local_ip, IPAddress dns_server)
//{
//	return WiFiSpiDrv::config((uint32_t)local_ip, 0, 0, (uint32_t)dns_server, 0);
//}

///*
// * 
// */
//bool WiFiSpiClass::config(IPAddress local_ip, IPAddress dns_server, IPAddress gateway)
//{
//	return WiFiSpiDrv::config((uint32_t)local_ip, (uint32_t)gateway, 0, (uint32_t)dns_server, 0);
//}

///*
// * 
// */
//bool WiFiSpiClass::config(IPAddress local_ip, IPAddress dns_server, IPAddress gateway, IPAddress subnet)
//{
//	return WiFiSpiDrv::config((uint32_t)local_ip, (uint32_t)gateway, (uint32_t)subnet, (uint32_t)dns_server, 0);
//}

///*
// * 
// */
//bool WiFiSpiClass::setDNS(IPAddress dns_server1)
//{
//    return WiFiSpiDrv::config(0, 0, 0, (uint32_t)dns_server1, 0);
//}

///*
// * 
// */
//bool WiFiSpiClass::setDNS(IPAddress dns_server1, IPAddress dns_server2)
//{
//    return WiFiSpiDrv::config(0, 0, 0, (uint32_t)dns_server1, (uint32_t)dns_server2);
//}

///*
// * 
// */
//int WiFiSpiClass::disconnect()
//{
//  return WiFiSpiDrv::disconnect();
//}

///*
// * 
// */
//uint8_t* WiFiSpiClass::macAddress(uint8_t* mac)
//{
//	uint8_t* _mac = WiFiSpiDrv::getMacAddress();
//	memcpy(mac, _mac, WL_MAC_ADDR_LENGTH);
//    return mac;
//}

/*
 * 
 */
IPAddress WiFiSpi_localIP()
{
	IPAddress ret;
	//WiFiSpiDrv_getIpAddress(ret);
	WiFiSpiDrv_getIpAddress(&ret);  // added by samuel
	return ret;
}

///*
// * 
// */
//IPAddress WiFiSpiClass::subnetMask()
//{
//	IPAddress ret;
//	WiFiSpiDrv::getSubnetMask(ret);
//	return ret;
//}

///*
// * 
// */
//IPAddress WiFiSpiClass::gatewayIP()
//{
//	IPAddress ret;
//	WiFiSpiDrv::getGatewayIP(ret);
//	return ret;
//}

/*
 * 
 */
char* WiFiSpi_SSID()
{
    return WiFiSpiDrv_getCurrentSSID();
}

///*
// * 
// */
//uint8_t* WiFiSpiClass::BSSID()
//{
//	return WiFiSpiDrv::getCurrentBSSID();
//}

/*
 * 
 */
int32_t WiFiSpi_RSSI()
{
    return WiFiSpiDrv_getCurrentRSSI();
}

///*///uint8_t WiFiSpiClass::encryptionType()
//{
//  return WiFiSpiDrv::getCurrentEncryptionType();
//}*/

///*
// * 
// */
//int8_t WiFiSpiClass::scanNetworks()
//{
//    #define WIFI_SCAN_RUNNING   (-1)
//    #define WIFI_SCAN_FAILED    (-2)

//    uint8_t attempts = 10;
//    int8_t numOfNetworks = 0;

//    if (WiFiSpiDrv::startScanNetworks() == WIFI_SCAN_FAILED)
//        return WL_FAILURE;

//    do
//    {
//        //delay(2000);
//        k_msleep(2000U); // added by samuel
//        numOfNetworks = WiFiSpiDrv::getScanNetworks();

//        if (numOfNetworks == WIFI_SCAN_FAILED)
//            return WL_FAILURE;
//    }
//    while ((numOfNetworks == WIFI_SCAN_RUNNING) && (--attempts > 0));
    
//    return numOfNetworks;
//}

///*
// * 
// */
//char* WiFiSpiClass::SSID(uint8_t networkItem)
//{
//	return WiFiSpiDrv::getSSIDNetworks(networkItem);
//}

///*
// * 
// */
//int32_t WiFiSpiClass::RSSI(uint8_t networkItem)
//{
//	return WiFiSpiDrv::getRSSINetworks(networkItem);
//}

///*
// * 
// */
//uint8_t WiFiSpiClass::encryptionType(uint8_t networkItem)
//{
//  return WiFiSpiDrv::getEncTypeNetworks(networkItem);
//}

/*
 * 
 */
uint8_t WiFiSpi_status()
{
  return WiFiSpiDrv_getConnectionStatus();
}

/*
 * 
 */
//int8_t WiFiSpi_hostByName(const char* aHostname, IPAddress& aResult)
int8_t WiFiSpi_hostByName(const char* aHostname, IPAddress *aResult)  // added by samuel
{
    return WiFiSpiDrv_getHostByName(aHostname, aResult);
}

///*
// * Perform remote software reset of the ESP8266 module. 
// * The reset succeedes only if the SPI communication is not broken.
// * The function does not wait for the ESP8266.
// */
//void WiFiSpiClass::softReset(void) {
//    WiFiSpiDrv::softReset();
//}

///*
// * 
// */
//const char* WiFiSpiClass::protocolVersion()
//{
//    return WiFiSpiDrv::getProtocolVersion();
//}

///*
// *
// */
//const char* WiFiSpiClass::masterProtocolVersion() 
//{
//    return protocolVer;
//}

/*
 *
 */
uint8_t WiFiSpi_checkProtocolVersion()
{
    const char* s = WiFiSpiDrv_getProtocolVersion();
    for (const char* p = protocolVer; *p; ++p, ++s)
    {
      if (*p != *s)
      {
        return 0;
      }
    }
    return (*s == 0);
}

/*
 *
 */
void WiFiSpi_hardReset(void)
{
    //if (hwResetPin <= 0)
    //    return;  // no reset pin

    ////pinMode(hwResetPin, OUTPUT);
    //nrf_gpio_cfg_output(hwResetPin); // added by samuel

    espSpiProxy_hardReset(hwResetPin);
}

///*
// *
// */

//uint8_t WiFiSpiClass::setSSLFingerprint(uint8_t* fingerprint)
//{
//    return WiFiSpiDrv::setSSLFingerprint(fingerprint);
//}


//WiFiSpiClass WiFiSpi;
