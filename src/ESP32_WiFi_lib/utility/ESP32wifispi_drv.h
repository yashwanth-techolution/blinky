#ifndef _WIFISPI_DRV_H_INCLUDED
#define _WIFISPI_DRV_H_INCLUDED

#include <inttypes.h>
#include "wifi_spi.h"

// Key index length
///#define KEY_IDX_LEN     1
// 100 ms secs of delay to test if the connection is established
#define WL_DELAY_START_CONNECTION 500
// Firmware version string length (format a.b.c)
#define WL_FW_VER_LENGTH 6
// Protocol version string length (format a.b.c)
#define WL_PROTOCOL_VER_LENGTH 6

#define DUMMY_DATA  0xFF

/*
 * Get network Data information
 */
int8_t WiFiSpiDrv_getNetworkData(uint8_t *ip, uint8_t *mask, uint8_t *gwip);

/*
 * Get scanned data information
 */
int8_t WiFiSpiDrv_getScannedData(uint8_t networkItem, char *ssid, int32_t *rssi, uint8_t *encr);

/*
 * Get remote Data information on UDP socket
 */
bool WiFiSpiDrv_getRemoteData(uint8_t sock, uint8_t *ip, uint16_t *port);

/*
 * Driver initialization, pin is GPIO port number used as SS
 */
void WiFiSpiDrv_wifiDriverInit(uint8_t cs_pin, uint8_t hwresetPin);  // added by samuel

/*
 * Set the desired network which the connection manager should try to
 * connect to.
 *
 * The ssid of the desired network should be specified.
 *
 * param ssid: The ssid of the desired network.
 * param ssid_len: Lenght of ssid string.
 * return: WL_SUCCESS or WL_FAILURE
 */
int8_t WiFiSpiDrv_wifiSetNetwork(const char* ssid, uint8_t ssid_len, bool *stop_flag);

/* Start Wifi connection with passphrase
 * the most secure supported mode will be automatically selected
 *
 * param ssid: Pointer to the SSID string.
 * param ssid_len: Length of ssid string.
 * param passphrase: Passphrase. Valid characters in a passphrase
 *        must be between ASCII 32-126 (decimal).
 * param len: Length of passphrase string.
 * return: WL_SUCCESS or WL_FAILURE
 */
uint8_t WiFiSpiDrv_wifiSetPassphrase(const char* ssid, const uint8_t ssid_len, const char *passphrase, const uint8_t len, bool *stop_flag);

/* Set ip configuration disabling dhcp client
 *
 * param local_ip: 	Static ip configuration
 * param gateway: 	Static gateway configuration
 * param subnet: 	Static subnet mask configuration
 * param dns_server1: Static DNS server1 configuration
 * param dns_server2: Static DNS server2 configuration
 */
//static bool config(uint32_t local_ip, uint32_t gateway, uint32_t subnet, uint32_t dns_server1, uint32_t dns_server2);

/*
 * Disconnect from the network
 *
 * return: WL_SUCCESS or WL_FAILURE
 */
uint8_t WiFiSpiDrv_disconnect();

/*
 * Disconnect from the network
 *
 * return: one value of wl_status_t enum
 */
uint8_t WiFiSpiDrv_getConnectionStatus(bool *stop_flag);

/*
 * Get the interface MAC address.
 *
 * return: pointer to uint8_t array with length WL_MAC_ADDR_LENGTH
 */
uint8_t* WiFiSpiDrv_getMacAddress();

/*
 * Get the interface IP address.
 *
 * return: copy the ip address value in IPAddress object
 */
int8_t WiFiSpiDrv_getIpAddress(IPAddress *ip);  // added by samuel

/*
 * Get the interface subnet mask address.
 *
 * return: copy the subnet mask address value in IPAddress object
 */
int8_t WiFiSpiDrv_getSubnetMask(IPAddress *mask);

/*
 * Get the gateway ip address.
 *
 * return: copy the gateway ip address value in IPAddress object
 */
int8_t WiFiSpiDrv_getGatewayIP(IPAddress *ip);

/*
 * Return the current SSID associated with the network
 *
 * return: ssid string
 */
char* WiFiSpiDrv_getCurrentSSID();

/*
 * Return the current BSSID associated with the network.
 * It is the MAC address of the Access Point
 *
 * return: pointer to uint8_t array with length WL_MAC_ADDR_LENGTH
 */
uint8_t* WiFiSpiDrv_getCurrentBSSID();

/*
 * Return the current RSSI /Received Signal Strength in dBm)
 * associated with the network
 *
 * return: signed value
 */
int32_t WiFiSpiDrv_getCurrentRSSI();

/*
 * Start scan WiFi networks available
 *
 * return: Number of discovered networks
 */
int8_t WiFiSpiDrv_startScanNetworks();

/*
 * Get the networks available
 *
 * return: Number of discovered networks
 */
int8_t WiFiSpiDrv_getScanNetworks();

/*
 * Return the SSID discovered during the network scan.
 *
 * param networkItem: specify from which network item want to get the information
 *
 * return: ssid string of the specified item on the networks scanned list
 */
char* WiFiSpiDrv_getSSIDNetworks(uint8_t networkItem);

/*
 * Return the encryption type of the networks discovered during the scanNetworks
 *
 * param networkItem: specify from which network item want to get the information
 *
 * return: encryption type (enum wl_enc_type) of the specified item on the networks scanned list
 */
uint8_t WiFiSpiDrv_getEncTypeNetworks(uint8_t networkItem);

/*
 * Return the RSSI of the networks discovered during the scanNetworks
 *
 * param networkItem: specify from which network item want to get the information
 *
 * return: signed value of RSSI of the specified item on the networks scanned list
 */
int32_t WiFiSpiDrv_getRSSINetworks(uint8_t networkItem);

/*
 * Resolve the given hostname to an IP address.
 * param aHostname: Name to be resolved
 * param aResult: IPAddress structure to store the returned IP address
 * result: 1 if aIPAddrString was successfully converted to an IP address,
 *         0 error, hostname not found
 *        -1 error, command was not performed
 */
int8_t WiFiSpiDrv_getHostByName(const char* aHostname, IPAddress *aResult, bool *stop_flag);  // added by samuel

/*
 * Get the firmware version
 * result: version as string with this format a.b.c
 */
const char* WiFiSpiDrv_getFwVersion();

/*
 * Perform software reset of the ESP8266 module. 
 * The reset succeedes only if the SPI communication is not broken.
 * After the reset wait for the ESP8266 to came to life again. Typically, the ESP8266 boots within 100 ms,
 * but with the WifiManager installed on ESP it can be a couple of seconds.
 */
void WiFiSpiDrv_softReset(void);

/*
 * Get the SPI protocol version
 * result: version as string with this format a.b.c
 */
const char* WiFiSpiDrv_getProtocolVersion();

/*
 * Sets or clears the certificate fingerprint for SSL connection
 * fingerprint - SHA1 of server certificate - must be 20 bytes (not character string!)
 */
//static uint8_t setSSLFingerprint(uint8_t *fingerprint);



// ------------------------------ ESP32 WiFi Module Self OTA APIs ---------------------------------------


/*
 * 
 */
int8_t WiFiSpiDrv_startSelfOTAServer();

/*
 * 
 */
int8_t WiFiSpiDrv_stopSelfOTAServer();

/*
 * 
 */
int8_t WiFiSpiDrv_getSelfOTAStatus(uint8_t* OTAPercent);




//-----------------------------------------------------------------------------------------------




//static uint8_t reqHostByName(const char* aHostname);

//static int getHostByName(IPAddress& aResult);

/* Start Wifi connection with WEP encryption.
 * Configure a key into the device. The key type (WEP-40, WEP-104)
 * is determined by the size of the key (5 bytes for WEP-40, 13 bytes for WEP-104).
 *
 * param ssid: Pointer to the SSID string.
 * param ssid_len: Lenght of ssid string.
 * param key_idx: The key index to set. Valid values are 0-3.
 * param key: Key input buffer.
 * param len: Lenght of key string.
 * return: WL_SUCCESS or WL_FAILURE
 */
//static int8_t wifiSetKey(const char* ssid, uint8_t ssid_len, uint8_t key_idx, const void *key, const uint8_t len);

/* Set DNS ip configuration
 *
 * param validParams: set the number of parameters that we want to change
 * 					 i.e. validParams = 1 means that we'll change only dns_server1
 * 					 	  validParams = 2 means that we'll change dns_server1 and dns_server2
 * param dns_server1: Static DNS server1 configuration
 * param dns_server2: Static DNS server2 configuration
 */
//static void setDNS(uint8_t validParams, uint32_t dns_server1, uint32_t dns_server2);

/*
 * Return the Encryption Type associated with the network
 *
 * return: one value of wl_enc_type enum
 */
//static uint8_t getCurrentEncryptionType();

#endif
