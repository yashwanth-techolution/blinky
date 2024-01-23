#include <errno.h>
#include <stdio.h>
#include <device.h>
#include <stdlib.h>
#include <zephyr.h>
#include <kernel.h>
#include <string.h>
#include <inttypes.h>
#include <sys/util.h>
#include <devicetree.h>
#include <sys/printk.h>
#include <drivers/spi.h>
#include <drivers/gpio.h>
#include <hal/nrf_gpio.h>

///*
//  WiFiSpiClient.cpp - Library for Arduino SPI connection to ESP8266
//  Copyright (c) 2017 Jiri Bilek. All rights reserved.

//  -----
  
//  Based on WiFiClient.cpp - Library for Arduino Wifi shield.
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

//extern "C" {
  #include "utility/debug.h"
  #include "utility/wl_types.h"
  #include "utility/wl_definitions.h"
//}

#include "WiFiSpi.h"
#include "WiFiSpiClient.h"
#include "utility/wifi_spi.h"
#include "utility/srvspi_drv.h"
#include "utility/wifispi_drv.h"

//WiFiSpiClient::WiFiSpiClient() : _sock(SOCK_NOT_AVAIL) {
//}

//WiFiSpiClient::WiFiSpiClient(uint8_t sock) : _sock(sock) {
//}

uint8_t _sock = SOCK_NOT_AVAIL;

/*
 * 
 */
int WiFiSpiClient_connect(const char* host, uint16_t port)
{
	IPAddress remote_addr;
	//if (WiFiSpi_hostByName(host, remote_addr))
	if (WiFiSpi_hostByName(host, &remote_addr))  // added by samuel
	{
          printk("WiFiSpi_hostByName success %d.%d.%d.%d\n", remote_addr.first_byte, remote_addr.second_byte, remote_addr.third_byte, remote_addr.fourth_byte);  // added by samuel
	  //return _connect(remote_addr, port, false);
	  return WiFiSpiClient__connect(remote_addr, port, false);        
	}
	return 0;
}

/*
 * 
 */
int WiFiSpiClient_connect_With_Ip(IPAddress ip, uint16_t port)
{
    return WiFiSpiClient__connect(ip, port, false);
}

///*
// * 
// */
//int WiFiSpiClient::connectSSL(const char* host, uint16_t port)
//{
//  IPAddress remote_addr;
//  if (WiFiSpi.hostByName(host, remote_addr))
//  {
//    return _connect(remote_addr, port, true);
//  }
//  return 0;
//}

///*
// * 
// */
//int WiFiSpiClient::connectSSL(IPAddress ip, uint16_t port)
//{
//    return _connect(ip, port, true);
//}

/*
 * 
 */
int WiFiSpiClient__connect(IPAddress ip, uint16_t port, bool isSSL)
{
    _sock = WiFiSpi_getSocket();
     
    /* added by samuel */
    uint32_t ipp = ((ip.first_byte) | (ip.second_byte << 8) | (ip.third_byte << 16) | (ip.fourth_byte << 24));
    //printk("--> %d.%d.%d.%d\n", ip.first_byte, ip.second_byte, ip.third_byte, ip.fourth_byte);  // added by samuel

    if (_sock != SOCK_NOT_AVAIL)
    {
      //if (! ServerSpiDrv_startClient((uint32_t)(ip), port, _sock, (isSSL ? TCP_MODE_WITH_TLS : TCP_MODE)))
      if (! ServerSpiDrv_startClient(ipp, port, _sock, (isSSL ? TCP_MODE_WITH_TLS : TCP_MODE)))  // added by samuel
      {
        return 0;   // unsuccessfull
      }
      //WiFiSpiClass::_state[_sock] = _sock;
      _state[_sock] = _sock;
    } 
    else 
    {
      printk("No Socket available\n");
      return 0;
    }
    return 1;
}

/*
 * 
 */
size_t WiFiSpiClient_write(const uint8_t *buf, size_t size)
{
    if (_sock >= MAX_SOCK_NUM || size == 0)
    {
      //setWriteError();  // cmd by samuel
      return 0;
    }

    if (!ServerSpiDrv_sendData(_sock, buf, size))
    {
      //setWriteError();  // cmd by samuel
      return 0;
    }
    return size;
}

/* extra function added by samuel */
size_t WiFiSpiClient_write_in(const char *str)  
{ 
  return WiFiSpiClient_write((const uint8_t *)(str), strlen(str)); 
}


/*
 * 
 */
int WiFiSpiClient_available() 
{
    if (_sock == SOCK_NOT_AVAIL)
    {
      return 0;
    }
    else
    {
      return ServerSpiDrv_availData(_sock);
    }
}

/*
 * 
 */
int WiFiSpiClient_read() 
{
    int16_t b;
    ServerSpiDrv_getData(_sock, &b, 0);  // returns -1 when error
    return b;
}

/*
    Reads data into a buffer.
    Return: 0 = success, size bytes read
           -1 = error (either no data or communication error)
 */
int WiFiSpiClient_read_buff(uint8_t* buf, size_t size) 
{
    // sizeof(size_t) is architecture dependent
    // but we need a 16 bit data type here
    uint16_t _size = size;
    if (!ServerSpiDrv_getDataBuf(_sock, buf, &_size))
        return -1;
    return 0;
}

///*
// * 
// */
//int WiFiSpiClient::peek() 
//{
//    int16_t b;
//    ServerSpiDrv::getData(_sock, &b, 1);  // returns -1 when error
//    return b;
//}

///*
// * 
// */
//void WiFiSpiClient::flush() {
//  // TODO: a real check to ensure transmission has been completed
//}

/*
 * 
 */
void WiFiSpiClient_stop() 
{
  if (_sock == SOCK_NOT_AVAIL)
  {
    return;
  } 
  ServerSpiDrv_stopClient(_sock);

  int count = 0;
  // wait maximum 5 secs for the connection to close
  //while (status() != CLOSED && ++count < 500)
  while (WiFiSpiClient_status() != CLOSED && ++count < 500)  // added by samuel
  {
    k_msleep(10U);
  }
  //if (WiFiSpiClass::_server_port[_sock] == 0)
  //  WiFiSpiClass::_state[_sock] = NA_STATE;  // Close only if it isn't server connection
  if (_server_port[_sock] == 0)
  {
    _state[_sock] = NA_STATE;  // added by samuel// Close only if it isn't server connection
  }
  _sock = SOCK_NOT_AVAIL;
}

/*
 * 
 */
uint8_t WiFiSpiClient_connected()
{
    if (_sock == SOCK_NOT_AVAIL)
    {
      return 0;
    }
    else
    {    
      //return (status() == ESTABLISHED);
      return (WiFiSpiClient_status() == ESTABLISHED);  // added by samuel
    }
}

/*
 * 
 */
uint8_t WiFiSpiClient_status() 
{
    if (_sock == SOCK_NOT_AVAIL)
    {
      return CLOSED;
    }
    else
    {
      return ServerSpiDrv_getClientState(_sock);
    }
}

///*
// * 
// */
//WiFiSpiClient::operator bool() {
//  return (_sock != SOCK_NOT_AVAIL);
//}

///*
// * 
// */
//uint8_t WiFiSpiClient::verifySSL(uint8_t* fingerprint, const char *host) {
//    if (_sock == SOCK_NOT_AVAIL || host[0] == 0)
//        return 0;

//    return ServerSpiDrv::verifySSLClient(_sock, fingerprint, host);
//}

///*
// * 
// */
//IPAddress WiFiSpiClient::remoteIP()
//{
//    uint8_t _remoteIp[4];
//    uint16_t _remotePort;

//    if (WiFiSpiDrv::getRemoteData(_sock, _remoteIp, &_remotePort))
//        return IPAddress(_remoteIp);
//    else
//        return IPAddress(0UL);
//}

///*
// * 
// */
//uint16_t WiFiSpiClient::remotePort()
//{
//    uint8_t _remoteIp[4];
//    uint16_t _remotePort;

//    if (WiFiSpiDrv::getRemoteData(_sock, _remoteIp, &_remotePort))
//        return _remotePort;
//    else
//        return 0;
//}
