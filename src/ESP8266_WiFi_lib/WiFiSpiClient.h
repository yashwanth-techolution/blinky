/*
  WiFiSpiClient.h - Library for Arduino with ESP8266 as slave.
  Copyright (c) 2017 Jiri Bilek. All right reserved.

  ---
  
  Based on WiFiClient.h - Library for Arduino Wifi shield.
  Copyright (c) 2011-2014 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _WIFISPICLIENT_H_INCLUDED
#define _WIFISPICLIENT_H_INCLUDED

#include <string.h>

//#include "Print.h"
//#include "IPAddress.h"
//#include "utility/Client.h"



////class WiFiSpiClient : public Client {

////private:
//    uint8_t _sock;   //not used

////public:
//    WiFiSpiClient();
//    WiFiSpiClient(uint8_t sock);
//    virtual ~WiFiSpiClient() {};

    uint8_t WiFiSpiClient_status();
//    virtual int connect(IPAddress ip, uint16_t port);
  //  int WiFiSpiClient_connect_With_Ip(IPAddress ip, uint16_t port);
    
    //virtual int WiFiSpiClient_connect(const char *host, uint16_t port);
    int WiFiSpiClient_connect(const char *host, uint16_t port);

//    virtual int connectSSL(IPAddress ip, uint16_t port);
//    virtual int connectSSL(const char *host, uint16_t port);

//    uint8_t verifySSL(uint8_t* fingerprint, const char *host);

    //virtual size_t WiFiSpiClient_write(const uint8_t *buf, size_t size);
//    size_t WiFiSpiClient_write(const uint8_t *buf, size_t size);
  //  size_t WiFiSpiClient_write_in(const char *str); /* extra function added by samuel */

//    virtual size_t write(uint8_t b)
//        { return write(&b, 1); }
//    virtual size_t write(const char *str)  
//        { return write(reinterpret_cast<const uint8_t *>(str), strlen(str)); }
//    virtual size_t write(const void *str, size_t size) 
//        { return write(reinterpret_cast<const uint8_t *>(str), size); }

    //virtual int WiFiSpiClient_available();
    int WiFiSpiClient_available();

    //virtual int WiFiSpiClient_read();
    int WiFiSpiClient_read();

    int WiFiSpiClient_read_buff(uint8_t *buf, size_t size);
//    virtual int peek();
//    virtual void flush();

    //virtual void WiFiSpiClient_stop();
    void WiFiSpiClient_stop();

    //virtual uint8_t WiFiSpiClient_connected();
    uint8_t WiFiSpiClient_connected();

//    virtual operator bool();

//    using Print::write;

//    // Return the IP address of the host who sent the current incoming packet
//    IPAddress remoteIP();
  
//    // Return the port of the host who sent the current incoming packet
//    uint16_t remotePort();

////private:
    int WiFiSpiClient__connect(IPAddress ip, uint16_t port, bool isSSL);

////};

#endif
