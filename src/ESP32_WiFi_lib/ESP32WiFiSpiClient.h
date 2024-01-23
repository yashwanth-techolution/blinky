#ifndef _WIFISPICLIENT_H_INCLUDED
#define _WIFISPICLIENT_H_INCLUDED

#include <string.h>


//WiFiSpiClient();
//WiFiSpiClient(uint8_t sock);
//virtual ~WiFiSpiClient() {};

int WiFiSpiClient_connect(const char *host, uint16_t port, bool *stop_flag);

int WiFiSpiClient_connect_With_Ip(IPAddress ip, uint16_t port, bool *stop_flag);
int WiFiSpiClient_connect_With_URL(const char *url, uint16_t port, bool *stop_flag);  

//virtual int connectSSL(IPAddress ip, uint16_t port);
//virtual int connectSSL(const char *host, uint16_t port);

int WiFiSpiClient__connect(IPAddress ip, uint16_t port, bool isSSL, bool *stop_flag);
int WiFiSpiClient__connect_url(const char *url, uint16_t port, bool isSSL, bool *stop_flag);
size_t WiFiSpiClient_write(const uint8_t *buf, size_t size, bool *stop_flag);
size_t WiFiSpiClient_write_in(const char *str, bool *stop_flag); /* extra function added by samuel */

int WiFiSpiClient_available(bool *stop_flag);

int WiFiSpiClient_read();

int WiFiSpiClient_read_buff(uint8_t *buf, size_t size, bool *stop_flag);

//virtual int peek();
//virtual void flush();

void WiFiSpiClient_stop(bool *stop_flag);

uint8_t WiFiSpiClient_connected(bool *stop_flag);

uint8_t WiFiSpiClient_status(bool *stop_flag);

//virtual operator bool();

//uint8_t verifySSL(uint8_t* fingerprint, const char *host);

//Return the IP address of the host who sent the current incoming packet
//IPAddress remoteIP();
  
//Return the port of the host who sent the current incoming packet
//uint16_t remotePort();

#endif
