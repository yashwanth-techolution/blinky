#ifndef ADO_ESP32WIFI_MODULE_H_
#define ADO_ESP32WIFI_MODULE_H_

#define ESP32
//#define ESP8266

#include<stdint.h>
#include <zephyr/kernel.h>
#include "Includes/ADO_Audio_files.h"

#define WIFI_MODULE_ON       true   // Active low GPIO pin requires true.
#define WIFI_MODULE_OFF      false  // Active low GPIO pin requires false.

extern bool wifi_module_active;
extern char body[32769];
extern int wificonnected_response;

// added by ashok for wifi-cred
//extern char ssid[];
//extern char pass[];


#ifdef ESP32
  #define WIFI_AUDIO_TRANSFER_BYTES     16000
#endif

#ifdef ESP8266
  #define WIFI_AUDIO_TRANSFER_BYTES   1024
#endif


#define WAV_AUDIO_HEADER_LENGTH       44
#define MAX_LEN                       32765



void printWifiStatus();
uint8_t ADO_WiFi_status(bool *stop_flag);
void WiFi_module_hard_reset();
void WiFi_module_on_off(bool on);
int WiFi_Module_int();
int WiFi_Network_Connect(bool *stop_flag);
void read_params();  // ESP32
bool check_ado_wifi_status(void);
bool check_ado_wifi_status_for_ssid(void);
void ADO_Wifi_Thread(void);
int multipart_sendFile(audio_auto_ai_parameters_t audio_auto_ai_params, uint8_t* audio_buff, uint16_t buff_len, bool *stop_flag); 

//int Send_data_to_auto_ai(uint8_t* audio_buf_array, uint16_t len, uint16_t detect_count, float* confidence_score);
int Send_data_to_auto_ai(audio_auto_ai_parameters_t audio_auto_ai_params, uint8_t* audio_buff, uint16_t buff_len, bool *stop_flag);
int8_t Get_data_from_auto_ai(bool *stop_flag);
int8_t Get_pending_data_from_auto_ai(bool *stop_flag);


/* added for downloading STM Hex file */
bool read_and_cmp_stm_fw_version(char *fw_ver, bool *stop_flag);
int Download_stm_hex_file_from_server(bool *stop_flag);


/* added for downloading STM Hex file */
bool request_and_cmp_stm_fw_version(char *fw_ver, bool *stop_flag);
int request_and_download_stm_hex_file_from_server(bool *stop_flag);

extern const k_tid_t wifi_notify_thrd_id;
extern uint8_t wifi_status_response;

extern bool wifi_thread_busy,server_connected;


typedef struct {
uint8_t cmd_type;
uint8_t ssid[32];
uint8_t ssidlen;
uint8_t pass[32];
uint8_t passlen;

}wifi_data;

#ifdef ESP32
  // ------------------------------ ESP32 WiFi Module Self OTA APIs ---------------------------------------

  int ESP32_startSelfOTAServer(void);

  int ESP32_stopSelfOTAServer(void);

  int ESP32_getSelfOTAStatus(uint8_t* OTAPercent);

  void ESP32_SelfOTA(bool* esp32OTAstart_stop_flag);

  void send_ssid_to_ED(void);
  void check_ssid_in_eeprom(void);
  void send_wifi_rssi(void);
#endif

#endif  // ADO_ESP32WIFI_MODULE_H_
