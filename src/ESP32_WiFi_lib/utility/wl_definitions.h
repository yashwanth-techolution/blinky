#ifndef _WL_DEFINITIONS_H_INCLUDED
#define _WL_DEFINITIONS_H_INCLUDED

// Maximum size of a SSID
#define WL_SSID_MAX_LENGTH 32
// Length of passphrase. Valid lengths are 8-63.
#define WL_WPA_KEY_MAX_LENGTH 63
// Length of key in bytes. Valid values are 5 and 13.
#define WL_WEP_KEY_MAX_LENGTH 13
// Size of a MAC-address or BSSID
#define WL_MAC_ADDR_LENGTH 6
// Size of a MAC-address or BSSID
#define WL_IPV4_LENGTH 4
// Maximum size of a SSID list
#define WL_NETWORKS_LIST_MAXNUM	10
// Maxmium number of socket
#define	MAX_SOCK_NUM		4
// Socket not available constant
#define SOCK_NOT_AVAIL  255
// Default state value for Wifi state field
#define NA_STATE -1
// Maximum waiting time to establish wifi connection is 10 s (in WL_DELAY_START_CONNECTION = 500 ms)
#define WL_MAX_ATTEMPT_CONNECTION	20


/* added by samuel */
#define SERVER_CONNECTION_FAILED    -2
#define NO_SOCKET_AVAILBLE          -3
#define FAILED_SEND_DATA            -4
#define SERVER_ERROR                -5
#define WIFI_DATA_SEND_PAUSED       -6


typedef enum {
	WL_NO_SHIELD = 255,
        WL_IDLE_STATUS = 0,
        WL_NO_SSID_AVAIL,
        WL_SCAN_COMPLETED,
        WL_CONNECTED,
        WL_CONNECT_FAILED,
        WL_CONNECTION_LOST,
        WL_DISCONNECTED
} wl_status_t;

/* Encryption modes */
/*enum wl_enc_type {  // Values map to 802.11 encryption suites...
        ENC_TYPE_WEP  = 5,
        ENC_TYPE_TKIP = 2,
        ENC_TYPE_CCMP = 4,
        // ... except these two, 7 and 8 are reserved in 802.11-2007
        ENC_TYPE_NONE = 7,
        ENC_TYPE_AUTO = 8
};*/

typedef struct {
  uint8_t first_byte;
  uint8_t second_byte;
  uint8_t third_byte;
  uint8_t fourth_byte;
}IPAddress;

#endif /* WL_DEFINITIONS_H_ */
