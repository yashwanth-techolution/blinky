#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <errno.h>
#include <hal/nrf_gpio.h>
#include <inttypes.h>
#include <zephyr/kernel.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "Includes/ADO_STM32H7_WW.h"
#include "Includes/ADO_WiFi_Module.h"
#include "Includes/Command_Manager_Module.h"

//--------added by ashok -----------------------//

#include "Includes/EEPROM_Module.h"

//-------------end-------------------------------//
#ifdef ESP8266
#include "ESP8266_WiFi_lib/WiFiSpi.h"
#endif

#ifdef ESP32
#include "ESP32_WiFi_lib/ESP32WiFiSpi.h"
#endif

#ifdef ESP8266
#define WiFi_SEND_DELAY 25U
#endif
#define WIFI_NTFY_THRD_PRIORITY 10
#define WIFI_NTFY_THRD_STACK_SIZE 1024

#define WIFI_STAT_POLL_INTERVAL K_MSEC(50)
K_THREAD_DEFINE(wifi_notify_thrd_id, WIFI_NTFY_THRD_STACK_SIZE,
    ADO_Wifi_Thread, NULL, NULL, NULL, WIFI_NTFY_THRD_PRIORITY, K_ESSENTIAL, 0);

//// USA WiFi credentials
////char ssid[] = "NETGEAR98";     // your network SSID (name)
////char pass[] = "festivenest069";  // your network password

//// WiFi credentials
// char ssid[] = "Techolution";     // your network SSID (name)
// char pass[] = "wearethebest";  // your network password

//// OTA_Status testing WiFi credentials
// char ssid[] = "Vineeth";     // your network SSID (name)
// char pass[] = "iron7613";       // your network password

//-----------------------------added by ashok ---------------------------//
// Wifi credentials
char ssid[32] = {0}; // your network SSID (name)
char pass[32] = {0}; // your network password
// bool wifi_thread_busy = false;

#define WiFi_RESET_PIN 11
#define WiFi_EN_PIN 7
#define WiFi_SPI_CS 39

char body[32769] = {0};

uint8_t wav_format_header_array[] = {0x52, 0x49, 0x46, 0x46, 0x24, 0x7D,
    0x00, 0x00, 0x57, 0x41, 0x56, 0x45, 0x66, 0x6D, 0x74, 0x20, 0x10, 0x00,
    0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x80, 0x3E, 0x00, 0x00, 0x00, 0x7D,
    0x00, 0x00,0x02, 0x00, 0x10, 0x00, 0x64, 0x61, 0x74, 0x61, 0x00, 0x7D,
    0x00, 0x00};

bool wifi_module_active = true, server_connected = false;
bool wifi_connection_begin_flag = false;

struct key_value {
  char key[75];
  char value[75];
};

int8_t process_header(char header[], int start, int end, int *line);
void process_header_second(char header[], int start, int end,
    int num_of_keys, struct key_value *store_key_value);

struct k_mbox wifi_notify_mb;

uint8_t wifi_status_response = 0;

void ADO_Wifi_Thread(void) {
  // Mailbox variables
  struct k_mbox_msg recv_msg;

  wifi_data recvdata = {
      0,
  };
  // Initialize the mailbox
  k_mbox_init(&wifi_notify_mb);

  while (1) {
    // prepare the mailbox receive buffer
    recv_msg.info = 0;
    recv_msg.size = sizeof(recvdata);
    recv_msg.rx_source_thread = cmd_mgr_thrd_id;

    // retrieve and delete the message if recieved
    k_mbox_get(&cmd_mgr_mb, &recv_msg, &recvdata, WIFI_STAT_POLL_INTERVAL);

    switch (recvdata.cmd_type) {
    //case WIFI_STATUS:
    //  check_ado_wifi_status();
    //  break;

    case WIFI_CONNECT:
      printk("\n WIFI_CONNECT");
      // wifi_connect_cmd_call();
      // wifi_connect_cmd("Techolution",11,"wearethebest",12,&stop_wifi_send);
      // wifi_thread_busy = true;
      wifi_connect_cmd(recvdata.ssid, recvdata.ssidlen, recvdata.pass,
          recvdata.passlen, &stop_wifi_send);
      //  wifi_thread_busy = false;
      break;

    case WIFI_RECONNECT:
      printk("\n WIFI_RECONNECT ");
      // wifi_connect_cmd_call();
      //  wifi_thread_busy = true;
      wifi_connect_cmd(recvdata.ssid, recvdata.ssidlen, recvdata.pass,
          recvdata.passlen, &stop_wifi_send);
      // wifi_thread_busy = false;
      // WiFi_password_store(cmd_mgr_msg_cpy.msg_buf_len, (char *)&cmd_mgr_msg_cpy.msg_buf[0]);
      break;

    case WIFI_SIGNAL_STRENGTH:
      send_wifi_rssi();
      break;

    case WIFI_CONNECTED_SSID:
      printk("\n--- requested ssid from edge device -- \n");

      if (check_ado_wifi_status_for_ssid()) {
        printk("\n--- ADO is connected to wifi   ---\n");
        send_ssid_to_ED();
      } else {
        printk("\n-------- ADO is not connected to wifi ----\n");
        // wifi_thread_busy = true;
        check_ssid_in_eeprom(); // check credentials in eeprom

        /*ADO_notify_ble(ble_notify_thrd_id, ADO_WIFI_CONTROL_RESPONSE,
        WIFI_CONNECTED_SSID, WIFI_CONNECTED_SSID_RESP, NULL,NULL); */

        // wifi_thread_busy = false;
      }

      break;
      //  default:
      //    break;
    }
    recvdata.cmd_type = 0U;
  }
}

void send_wifi_rssi(void) {
  int8_t evt_wifi_msg[MAX_WIFI_CON_STR_LEN] = {0};
  uint8_t evt_wifi_msg_len = 0;
  int8_t rssi = 0;
  rssi = WiFiSpi_RSSI(); // long
  printk("signal strength (RSSI): %d dBm\n", rssi);
  evt_wifi_msg[0] = rssi;
  evt_wifi_msg_len = sizeof(evt_wifi_msg);
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_WIFI_CONTROL_RESPONSE, WIFI_SIGNAL_STRENGTH,
      WIFI_SIGNAL_STRENGTH_RESP, evt_wifi_msg, evt_wifi_msg_len);
}

bool check_ado_wifi_status_for_ssid(void) {
  // printk("\n---------- testing wifi status -------------- \n");
  uint8_t evt_wifi_msg[MAX_WIFI_CON_STR_LEN] = {0};
  uint8_t evt_wifi_msg_len = 0;
  // evt_wifi_msg[0] = ADO_get_wifi_state();

  uint8_t res = ADO_WiFi_status(&stop_wifi_send);
  // printk("\n res = %d\n",res);

  if (res == 3) { // 3-> connected
    printk("\n DS -> ---- wifi is connected -----\n");
    evt_wifi_msg[0] = 1;
    evt_wifi_msg_len = sizeof(evt_wifi_msg);
    printk("\n Edge Device called for wifi state, and the value is: %d\n", evt_wifi_msg[0]);
    /* ADO_notify_ble(cmd_mgr_thrd_id, ADO_DEVICE_STATUS_RESPONSE, WIFI_CONNECTION_STATUS,
        WIFI_CONNECTION_STATUS_RESP, evt_wifi_msg, evt_wifi_msg_len); */
    return true;
  } else {
    printk("\n DS -> -- wifi is not connected ----\n");
    evt_wifi_msg[0] = 0;
    evt_wifi_msg_len = sizeof(evt_wifi_msg);
    printk("\n Edge Device called for wifi state, and the value is: %d\n", evt_wifi_msg[0]);
    /*ADO_notify_ble(cmd_mgr_thrd_id, ADO_DEVICE_STATUS_RESPONSE, WIFI_CONNECTION_STATUS,
    WIFI_CONNECTION_STATUS_RESP, evt_wifi_msg, evt_wifi_msg_len); */
    return false;
  }
}

bool check_ado_wifi_status(void) {
  // printk("\n---------- testing wifi status -------------- \n");
  uint8_t evt_wifi_msg[MAX_WIFI_CON_STR_LEN] = {0};
  uint8_t evt_wifi_msg_len = 0;
  // evt_wifi_msg[0] = ADO_get_wifi_state();

  if (server_connected) {
    printk("\n DS -> ---- wifi is connected_server_connected-----\n");
    evt_wifi_msg[0] = 1;
    evt_wifi_msg_len = sizeof(evt_wifi_msg);
    printk("\n Edge Device called for wifi state, and the value is: %d\n", evt_wifi_msg[0]);
    ADO_notify_ble(cmd_mgr_thrd_id, ADO_DEVICE_STATUS_RESPONSE, WIFI_CONNECTION_STATUS,
        WIFI_CONNECTION_STATUS_RESP, evt_wifi_msg, evt_wifi_msg_len);
  } else {
    uint8_t res = ADO_WiFi_status(&stop_wifi_send);
    // printk("\n res = %d\n",res);

    if (res == 3) { // 3-> connected
      printk("\n DS -> ---- wifi is connected -----\n");
      evt_wifi_msg[0] = 1;
      evt_wifi_msg_len = sizeof(evt_wifi_msg);
      printk("\n Edge Device called for wifi state, and the value is: %d\n", evt_wifi_msg[0]);
      ADO_notify_ble(cmd_mgr_thrd_id, ADO_DEVICE_STATUS_RESPONSE, WIFI_CONNECTION_STATUS,
          WIFI_CONNECTION_STATUS_RESP, evt_wifi_msg, evt_wifi_msg_len);
      return true;
    } else {
      printk("\n DS -> -- wifi is not connected ----\n");
      evt_wifi_msg[0] = 0;
      evt_wifi_msg_len = sizeof(evt_wifi_msg);
      printk("\n Edge Device called for wifi state, and the value is: %d\n", evt_wifi_msg[0]);
      ADO_notify_ble(cmd_mgr_thrd_id, ADO_DEVICE_STATUS_RESPONSE, WIFI_CONNECTION_STATUS,
          WIFI_CONNECTION_STATUS_RESP, evt_wifi_msg, evt_wifi_msg_len);
      return false;
    }
  }
  //-------------------------------------------------//

  // evt_wifi_msg_len = sizeof(evt_wifi_msg);
  // printk("\n Edge Device called for wifi state, and the value is: %d\n", evt_wifi_msg[0]);
  /* ADO_notify_ble(cmd_mgr_thrd_id, ADO_DEVICE_STATUS_RESPONSE, WIFI_CONNECTION_STATUS,
      WIFI_CONNECTION_STATUS_RESP, evt_wifi_msg, evt_wifi_msg_len); */
}

void send_ssid_to_ED(void) {
  uint8_t ssid_length = strlen(ssid);
  printk("\n--- ssid = %s, ssid_length =%d --\n", ssid, ssid_length);
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_WIFI_CONTROL_RESPONSE, WIFI_CONNECTED_SSID,
      WIFI_CONNECTED_SSID_RESP, ssid, ssid_length);
}

void check_ssid_in_eeprom(void) {
  uint8_t evt_wifi_msg[1] = {0};
  uint8_t evt_wifi_msg_len = 0;
  int eeprom_ssid_length_r[1] = {0}, eeprom_pass_length_r[1] = {0};

  // Read from eeprom to save ssid and password
  Read_EEPROM(WIFI_SSID_LENGTH, eeprom_ssid_length_r, 1);
  Read_EEPROM(WIFI_SSID_LOCATION, ssid, eeprom_ssid_length_r[0]);
  int count = 0, i = 0;

  for (i = 0; i < 10; i++) {
    if (ssid[i] == 255) {
      count++;
    }
  }

  if (count == 10) {
    count = 0;
    uint8_t evt_wifi_msg[] = "-"; // No wifi cred in eeprom
    uint8_t evt_wifi_msg_len = strlen(evt_wifi_msg);

    printk("\n----- There is no credentials in eeprom send to edge"
        "device that wifi need to be configured ----\n");
    ADO_notify_ble(cmd_mgr_thrd_id, ADO_WIFI_CONTROL_RESPONSE, WIFI_CONNECTED_SSID,
        WIFI_CONNECTED_SSID_RESP, evt_wifi_msg, evt_wifi_msg_len);

    evt_wifi_msg[0] = 0;
    evt_wifi_msg_len = sizeof(evt_wifi_msg);
    printk("\n sending wifi state to ED, and the value is: %d\n", evt_wifi_msg[0]);
    /* ADO_notify_ble(cmd_mgr_thrd_id, ADO_DEVICE_STATUS_RESPONSE, WIFI_CONNECTION_STATUS,
        WIFI_CONNECTION_STATUS_RESP, evt_wifi_msg, evt_wifi_msg_len); */
  } else {
    printk("--------- EEPROM have wifi credentials and trying to connect to wifi------\n");
    Read_EEPROM(WIFI_PASSWORD_LENGTH, eeprom_pass_length_r, 1);
    Read_EEPROM(WIFI_PASSWORD_LOCATION, pass, eeprom_pass_length_r[0]);

    uint8_t ssid_length = strlen(ssid);
    uint8_t pass_length = strlen(pass);
    ADO_notify_ble(cmd_mgr_thrd_id, ADO_WIFI_CONTROL_RESPONSE, WIFI_CONNECTED_SSID,
        WIFI_CONNECTED_SSID_RESP, ssid, ssid_length);
  }
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  printk("SSID: %s\n", WiFiSpi_SSID());
  // print your WiFi shield's IP address:
  IPAddress ip = WiFiSpi_localIP();
  printk("IP Address: %d.%d.%d.%d \n", ip.first_byte, ip.second_byte,
      ip.third_byte, ip.fourth_byte);
  // print the received signal strength:
  long rssi = WiFiSpi_RSSI();
  printk("signal strength (RSSI): %d dBm\n", rssi);
}

/*
 * brief: This function sends hardware reset pulse to wifi module.
 */
void WiFi_module_hard_reset() {
  WiFiSpi_hardReset();
  printk("Wi-Fi Hardware Restarted\n");
}

/*
 * brief: This function enables(on) and disables(off) wifi module.
 */
void WiFi_module_on_off(bool on) {
  if ((WIFI_MODULE_ON) == on) {
    nrf_gpio_pin_set(WiFi_EN_PIN);
    wifi_module_active = true;
    printk("Wi-Fi Module Enabled\n");
  } else {
    nrf_gpio_pin_clear(WiFi_EN_PIN);
    wifi_module_active = false;
    printk("Wi-Fi Module Disabled\n");
  }
}

// ------------- wifi int and connect and status function --------------------

#ifdef ESP32
uint8_t ADO_WiFi_status(bool *stop_flag) {
  return WiFiSpi_status(stop_flag);
}

int WiFi_Module_int() {
  nrf_gpio_cfg_output(WiFi_EN_PIN);
  nrf_gpio_pin_set(WiFi_EN_PIN);

  // Initialize the WifiSpi library
  WiFiSpi_init(WiFi_SPI_CS, WiFi_RESET_PIN);
  k_msleep(10U);

  WiFi_module_on_off(WIFI_MODULE_ON);
  k_msleep(10U);
  int k = 0;

  for (; k < 5; k++) {
    // check for the presence of the shield:
    if (WiFiSpi_status(NULL) != WL_NO_SHIELD) {
      break;
    }
    k_msleep(100);
    printk("k = %d\n", k);
  }

  if (5 == k) {
    printk("WiFi shield not present\n");
    return WL_NO_SHIELD;
  }
  k_msleep(100);

  /* cmd for giving kernel error */
  if (!WiFiSpi_checkProtocolVersion()) {
    printk("Protocol version mismatch. Please upgrade the firmware\n");
    return -1;
  }
  return 0;
}

int WiFi_Network_Connect(bool *stop_flag) {
  int status; // the Wifi radio's status
  int i = 0;
  uint8_t evt_wifi_msg[1] = {0};
  uint8_t evt_wifi_msg_len = 0;
  char ssid_name[33] = {0};

  WiFiSpi_Socket_init();
  check_ssid();

  do {
    printk("Attempting to connect to WPA SSID: %s\n", ssid);
    // Connect to WPA/WPA2 network:
    status = WiFiSpi_begin(ssid, pass, stop_flag);

    for (int t = 0; t < 100; t++) {
      if ((stop_flag != NULL) && (*stop_flag)) {
        return WIFI_DATA_SEND_PAUSED;
      }
      k_msleep(50U);
    }

    status = WiFiSpi_status(stop_flag);
    i++;
    printk("\nno. of attempts to connect wifi is %d maximum attempts is 4\n", i);
  } while ((status != WL_CONNECTED) && (i < 5)); // 5 ->4

  if (status != WL_CONNECTED) {
    printk("\nfaild to connect to WiFi. No.of attempts = %d\n", i);

    if (new_wifi == true) {
      evt_wifi_msg[0] = 5; // STATUS = NOT_CONNECTED
      evt_wifi_msg_len = sizeof(evt_wifi_msg);
      ADO_notify_ble(cmd_mgr_thrd_id,
          ADO_WIFI_CONTROL_RESPONSE,
          WIFI_CONTROL_CONNECTION_STATUS_CONN,
          WIFI_CONTROL_CONNECTION_STATUS_RESP_CONN,
          evt_wifi_msg, evt_wifi_msg_len);
    }
    return WL_CONNECT_FAILED;
  } else {
    printk("\nConnected to wifi\n");
    evt_wifi_msg[0] = 1; // STATUS = CONNECTED
    evt_wifi_msg_len = sizeof(evt_wifi_msg);
    ADO_notify_ble(cmd_mgr_thrd_id,
        ADO_WIFI_CONTROL_RESPONSE,
        WIFI_CONTROL_CONNECTION_STATUS_CONN,
        WIFI_CONTROL_CONNECTION_STATUS_RESP_CONN,
        evt_wifi_msg, evt_wifi_msg_len);
  }
  printWifiStatus();
  return WL_CONNECTED;
}

int WiFi_Network_Connect_cmd(char *temp, int ssid_length,
    char *temp_pass, int pass_length, bool *stop_flag) {
  int status; // the Wifi radio's status
  int i = 0;
  uint8_t evt_wifi_msg[1] = {0};
  uint8_t evt_wifi_msg_len = 0;

  printk("ssid_length=%d\n", ssid_length);
  int eeprom_ssid_length_r[2] = {0}, eeprom_ssid_length_w[2] = {0},
      eeprom_password_length_r[2] = {0}, eeprom_password_length_w[2] = {0};
  char eeprom_ssid_r[32] = {0}, eeprom_pass_r[32] = {0};
  int count = 0;

  for (count = 0; count < ssid_length; count++) {
    ssid[count] = temp[count];
  }

  ssid[count] = '\0';
  count = 0;

  for (count = 0; count < pass_length; count++) {
    pass[count] = temp_pass[count];
  }

  pass[count] = '\0';
  WiFiSpi_Socket_init();
  // new code modified by yashwanth
  // current wifi connecting time 5 to 10 sec with below code previously nearly 20 sec
  do {
    printk("CMD --> Attempting to connect to WPA SSID: %s and password is %s\n", ssid, pass);
    // Connect to WPA/WPA2 network:
    wifi_connection_begin_flag = true;
    // to achive 1second timing with SLAVE_TX_READY_TIMEOUT_WIFI_BEGIN macro used in waitForSlaveTxReady function
    status = WiFiSpi_begin(ssid, pass, stop_flag);

    if (i == 0) {
      status = WiFiSpi_begin(ssid, pass, stop_flag);
    }

    wifi_connection_begin_flag = false;

    for (int t = 0; t < (i + 1) * 20; t++) {
      if ((stop_flag != NULL) && (*stop_flag)) {
        printk("-----------------wifi stop flag is true------------------------\n");
        return WIFI_DATA_SEND_PAUSED;
      }
      k_msleep(50U);
    }
    status = WiFiSpi_status(stop_flag);
    i++;
    printk("\n CMD --> no. of attempts to connect wifi is %d maximum attempts is 3\n", i);
  } while ((status != WL_CONNECTED) && (i < 3)); // 5 ->4

  if (status != WL_CONNECTED) {
    printk("\nCMD --> failed to connect to WiFi. No.of attempts = %d\n", i);

    if (new_wifi == true) {
      evt_wifi_msg[0] = 5; // STATUS = NOT_CONNECTED
      evt_wifi_msg_len = sizeof(evt_wifi_msg);
      wifi_status_response = 5;
      ADO_notify_ble(cmd_mgr_thrd_id,
      ADO_WIFI_CONTROL_RESPONSE,
          WIFI_CONTROL_CONNECTION_STATUS_CONN,
          WIFI_CONTROL_CONNECTION_STATUS_RESP_CONN,
          evt_wifi_msg, evt_wifi_msg_len);
    }
    return WL_CONNECT_FAILED;
  } else {
    printk("\nCMD --> Connected to wifi\n");
    evt_wifi_msg[0] = 1; // STATUS = CONNECTED
    evt_wifi_msg_len = sizeof(evt_wifi_msg);
    wifi_status_response = 1;
    ADO_notify_ble(cmd_mgr_thrd_id,
        ADO_WIFI_CONTROL_RESPONSE,
        WIFI_CONTROL_CONNECTION_STATUS_CONN,
        WIFI_CONTROL_CONNECTION_STATUS_RESP_CONN,
        evt_wifi_msg, evt_wifi_msg_len);
    printk("-------------------Start saving wifi cred in eeprom-------------------------------\n");
    eeprom_ssid_length_w[0] = ssid_length;

    // eeprom_ssid_length_w[1]='\0';

    Write_EEPROM(WIFI_SSID_LENGTH, eeprom_ssid_length_w, 1); 
    // write cycle takes 5 ms according to datasheet
    k_msleep(10);

    Read_EEPROM(WIFI_SSID_LENGTH, eeprom_ssid_length_r, 1);
    k_msleep(10);

    printk("read from eeprom ssid length is %d and %d\n",
        eeprom_ssid_length_r[0], eeprom_ssid_length_r[0] + 1);
    Write_EEPROM(WIFI_SSID_LOCATION, ssid, ssid_length); // ssid_length+1
    k_msleep(10);

    Read_EEPROM(WIFI_SSID_LOCATION, eeprom_ssid_r, eeprom_ssid_length_r[0]);
    k_msleep(10);
    printk("read from eeprom ssid is %s\n", eeprom_ssid_r);
    k_msleep(10);

    //---------------------password storage --------------------------------------//

    eeprom_password_length_w[0] = pass_length;
    // eeprom_password_length_w[1]='\0';

    Write_EEPROM(WIFI_PASSWORD_LENGTH, eeprom_password_length_w, 1); 
    // write cycle takes 5 ms according to datasheet
    k_msleep(10);

    Read_EEPROM(WIFI_PASSWORD_LENGTH, eeprom_password_length_r, 1);
    k_msleep(10);

    printk("read from eeprom password length is %d and %d\n",
        eeprom_password_length_r[0], eeprom_password_length_r[0] + 1);
    Write_EEPROM(WIFI_PASSWORD_LOCATION, pass, pass_length); // ssid_length+1
    k_msleep(10);

    Read_EEPROM(WIFI_PASSWORD_LOCATION, eeprom_pass_r, eeprom_password_length_r[0]);
    k_msleep(10);

    printk("read from eeprom password is %s\n", eeprom_pass_r);
    k_msleep(10);
  }
  printWifiStatus();
  return WL_CONNECTED;
}

void check_ssid(void) {
  int eeprom_ssid_length_r[2] = {0}, eeprom_password_length_r[2] = {0};
  int count = 0;

  for (int i = 0; i < 10; i++) {
    if (ssid[i] == 0) {
      count++;
    }
  }

  if (count == 10) {
    count = 0;
    printk("There is no wifi_cred. So reading from from EEPROM\n");

    // Read from eeprom to save ssid and password
    Read_EEPROM(WIFI_SSID_LENGTH, eeprom_ssid_length_r, 1);
    Read_EEPROM(WIFI_SSID_LOCATION, ssid, eeprom_ssid_length_r[0]);
    Read_EEPROM(WIFI_PASSWORD_LENGTH, eeprom_password_length_r, 1);
    Read_EEPROM(WIFI_PASSWORD_LOCATION, pass, eeprom_password_length_r[0]);

    printk("read from eeprom. ssid = %s and password = %s", ssid, pass);
    printk("\nConnecting to %s WiFi Module.......................", ssid);
    wifi_connect_cmd(ssid, strlen(ssid), pass, strlen(pass), false);
  }
}

#endif // ESP32

#ifdef ESP8266
uint8_t ADO_WiFi_status(bool *stop_flag) {
  return WiFiSpi_status();
}

int WiFi_Module_int() {
  nrf_gpio_cfg_output(WiFi_EN_PIN);
  nrf_gpio_pin_set(WiFi_EN_PIN);

  // Initialize the WifiSpi library
  WiFiSpi_init(WiFi_SPI_CS, WiFi_RESET_PIN);

  k_msleep(10U);
  // WiFi_module_on_off(WIFI_MODULE_ON);
  k_msleep(10U);

  // check for the presence of the shield:
  if (WiFiSpi_status() == WL_NO_SHIELD) {
    printk("Wi-Fi shield not present\n");
    return WL_NO_SHIELD;
  }
  k_msleep(100);

  if (!WiFiSpi_checkProtocolVersion()) {
    printk("Protocol version mismatch. Please upgrade the firmware\n");
    return -1;
  }
  return 0;
}

int WiFi_Network_Connect(bool *stop_flag) {
  int status; // the Wifi radio's status
  int i = 0;

  WiFiSpi_Socket_init();

  do {
    printk("Attempting to connect to WPA SSID: %s\n", ssid);
    // Connect to WPA/WPA2 network:
    status = WiFiSpi_begin(ssid, pass);
    for (int t = 0; t < 100; t++) {
      if ((stop_flag != NULL) && (*stop_flag)) {
        return WIFI_DATA_SEND_PAUSED;
      }
      k_msleep(50U);
    }
    status = WiFiSpi_status();
    i++;
  } while ((status != WL_CONNECTED) && (i < 5));

  if (status != WL_CONNECTED) {
    printk("\nfaild to connect to WiFi. No.of attempts = %d\n", i);
    return WL_CONNECT_FAILED;
  } else {
    printk("\nConnected to wifi\n");
  }
  printWifiStatus();
  return WL_CONNECTED;
}
#endif // ESP8266

// --------------------------------------------------------------------------

#ifdef ESP32
void read_params() {
  // Check Firmware Command
  printk("\nFirmware protocol version = %s\n", WiFiSpi_firmwareVersion());

  // Check MAC Address Command
  uint8_t mac_ptr[6];
  WiFiSpi_macAddress(mac_ptr);
  printk("\nMac Address = %02X: %02X: %02X: %02X: %02X: %02X\n",
      mac_ptr[0], mac_ptr[1], mac_ptr[2], mac_ptr[3], mac_ptr[4], mac_ptr[5]);

  // Check WiFi Local IP Command
  IPAddress Lip = WiFiSpi_localIP();
  printk("\nLocal IP Address: %d.%d.%d.%d \n", Lip.first_byte,
      Lip.second_byte, Lip.third_byte, Lip.fourth_byte);

  // Check WiFi Subnet Mask IP Command
  IPAddress Subip = WiFiSpi_subnetMask();
  printk("\nSubnet Mask IP Address: %d.%d.%d.%d \n", Subip.first_byte,
      Subip.second_byte, Subip.third_byte, Subip.fourth_byte);

  // Check WiFi Gateway IP Command
  IPAddress Gatip = WiFiSpi_gatewayIP();
  printk("\nGateway IP Address: %d.%d.%d.%d \n", Gatip.first_byte,
      Gatip.second_byte, Gatip.third_byte, Gatip.fourth_byte);

  // Start WiFi Network Scan Command
  printk("\nStart Scan Networks = %d\n", WiFiSpi_scanNetworks());

  // Get WiFi SSID Name Command
  printk("\nGet Scan Data = %s\n", WiFiSpi_SSID_with_index(5));

  // Check SPI Protocol Command
  printk("\nProtocol version = %s\n", WiFiSpi_protocolVersion());

  // Get BSSID address Command
  uint8_t *ipB = WiFiSpi_BSSID();
  printk("\nBSSID Address: %02X: %02X: %02X: %02X: %02X: %02X\n",
      ipB[0], ipB[1], ipB[2], ipB[3], ipB[4], ipB[5]);

  // WiFi Disconnect Command
  printk("\nDisconnection Status = %d\n", WiFiSpi_disconnect());

  //// Module Soft Reset Command
  // printk("\nWiFi Softreset\n");
  // WiFiSpi_softReset();
}

int prepare_data(const char *str, int *len) {
  memcpy(&body[*len], str, strlen(str));
  *len += strlen(str);
}

int multipart_sendFile(audio_auto_ai_parameters_t audio_auto_ai_params,
    uint8_t *audio_buff, uint16_t buff_len, bool *stop_flag) {
  IPAddress server = {34, 93, 212, 15}; // IP address of Auto-AI Server.
  //const char *url = "https://autoai-backend-exjsxe2nda-uc.a.run.app";
  int k = 0;
  int ret = 0;
  int length = 0;
  char Slength[5];
  int size = 0;

  length = 996 + audio_auto_ai_params.params_str_length + buff_len + WAV_AUDIO_HEADER_LENGTH;
  printf("\n==>params_str_length %d", audio_auto_ai_params.params_str_length);
  itoa(length, Slength, 10);

  // memset(body, 0, 32769);
  body[0] = 0;

  prepare_data("POST /pingtest HTTP/1.1\r\nHost: 34.93.212.15:5001\r\n"
                  "Content-Type: multipart/form-data; boundary=------------------------d74496d66958873e\r\n"  // 88
                  "Content-Length: ", &size);

  //prepare_data("POST /resource HTTP/1.1\r\nHost: autoai-backend-exjsxe2nda-uc.a.run.app\r\n"
  //             "Content-Type: multipart/form-data; boundary=------------------------d74496d66958873e\r\n" // 88
  //             "Content-Length: ",
  //    &size);

  prepare_data(Slength, &size);

  prepare_data("\r\nExpect: 100-continue\r\n"
               "\r\n--------------------------d74496d66958873e\r\n"      // 46
               "Content-Disposition: form-data; name=\"status\"\r\n\r\n" // 49
               "backlog"                                                 // 7
               "\r\n--------------------------d74496d66958873e\r\n"      // 46
               "Content-Disposition: form-data; name=\"csv\"\r\n\r\n",
      &size);

  prepare_data(audio_auto_ai_params.csv, &size);

  prepare_data("\r\n--------------------------d74496d66958873e\r\n"     // 46
               "Content-Disposition: form-data; name=\"model\"\r\n\r\n" // 48
               "61ef9d4324e24310d102c5c7"                               // 24
               "\r\n--------------------------d74496d66958873e\r\n"     // 46
               "Content-Disposition: form-data; name=\"label\"\r\n\r\n",
      &size);

  prepare_data(audio_auto_ai_params.label, &size);

  prepare_data("\r\n--------------------------d74496d66958873e\r\n" // 46
               "Content-Disposition: form-data; name=\"tag\"\r\n\r\n",
      &size); // 46

  prepare_data(audio_auto_ai_params.tag, &size);

  prepare_data("\r\n--------------------------d74496d66958873e\r\n" // 46
               "Content-Disposition: form-data; name=\"confidence_score\"\r\n\r\n",
      &size); // 59

  prepare_data(audio_auto_ai_params.ww_confidence_score, &size);

  prepare_data("\r\n--------------------------d74496d66958873e\r\n"          // 46
               "Content-Disposition: form-data; name=\"prediction\"\r\n\r\n" // 53
               "predicted"                                                   // 9
               "\r\n--------------------------d74496d66958873e\r\n"          // 46
               "Content-Disposition: form-data; name=\"model_type\"\r\n\r\n" // 53
               "audio"                                                       // 5
               "\r\n--------------------------d74496d66958873e\r\n"          // 46
               "Content-Disposition: form-data; name=\"resource\"; filename=\"",
      &size); // 59
  prepare_data(audio_auto_ai_params.file_name, &size);

  prepare_data("\"\r\nContent-Type: audio/wav\r\n\r\n", &size); // 30

  body[size] = 0;

  printk("\n -----size = %d,-----\n", size);

  printk("\nStarting connection to server...\n");
  if (WiFiSpiClient_connect_With_Ip(server, 5001, stop_flag)) {

  //if (WiFiSpiClient_connect_With_URL(url, 443, stop_flag)) {
    printk("\nconnected to server\n");
    ret = WiFiSpiClient_write(body, size, stop_flag);
    printk("size 1 = %d \n", ret); // added by samuel

    if ((stop_flag != NULL) && (*stop_flag)) {
      return WIFI_DATA_SEND_PAUSED;
    }

    ret = WiFiSpiClient_write(wav_format_header_array, WAV_AUDIO_HEADER_LENGTH, stop_flag);
    printk("size 2 = %d \n", ret); // added by samuel

    if ((stop_flag != NULL) && (*stop_flag)) {
      return WIFI_DATA_SEND_PAUSED;
    }

    while (k < buff_len) {
      if ((buff_len - k) < WIFI_AUDIO_TRANSFER_BYTES) {
        ret = WiFiSpiClient_write(audio_buff + k, (buff_len - k), stop_flag);
        printk("size %d = %d \n", (3 + k), ret); // added by samuel

        if ((stop_flag != NULL) && (*stop_flag)) {
          return WIFI_DATA_SEND_PAUSED;
        }
      } else {
        ret = WiFiSpiClient_write(audio_buff + k, WIFI_AUDIO_TRANSFER_BYTES, stop_flag);
        printk("size %d = %d \n", (3 + k), ret); // added by samuel

        if ((stop_flag != NULL) && (*stop_flag)) {
          return WIFI_DATA_SEND_PAUSED;
        }
      }
      k += WIFI_AUDIO_TRANSFER_BYTES;
    }

    ret = WiFiSpiClient_write_in("\r\n--------------------------d74496d66958873e--\r\n\r\n", stop_flag); // 50
    printk("size last = %d \n", ret);

    if ((stop_flag != NULL) && (*stop_flag)) {
      return WIFI_DATA_SEND_PAUSED;
    }
  } else {
    printk("\nfailed to connect server\n");
    return SERVER_CONNECTION_FAILED;
  }
  return 0;
}

int8_t Get_data_from_auto_ai(bool *stop_flag) {
  uint8_t len = 0;
  uint8_t pos = 0;
  char bufff[50] = {0};
  int8_t err = -1;
  unsigned long int time_stamp;

  time_stamp = k_uptime_get();

  while (k_uptime_get() <= (time_stamp + 5000)) {
    len = WiFiSpiClient_available(stop_flag);

    if (len != 0) {
      printf("\n------------length = %d,  pos=%d  len+pos = %d before modification ----\n", len, pos, (len + pos));
    }

    if ((stop_flag != NULL) && (*stop_flag)) {
      return WIFI_DATA_SEND_PAUSED;
    }

    if (len) {
      // len = (len >= 49)?49:len;
      len = ((len + pos) >= 49) ? (49 - pos) : len;
      printk("----len = %d----\n", len);
      WiFiSpiClient_read_buff(&bufff[pos], len, stop_flag);
      if ((stop_flag != NULL) && (*stop_flag)) {
        return WIFI_DATA_SEND_PAUSED;
      }

      printk("%s", &bufff[pos]);

      if (err) {
        if ((strstr(bufff, "HTTP/1.0 200 OK") != NULL) || (strstr(bufff, "HTTP/1.1 200 OK") != NULL)) {
          ww_from_server = 200;
          err = 0;
          WiFiSpiClient_stop(stop_flag);

          if ((stop_flag != NULL) && (*stop_flag)) {
            return WIFI_DATA_SEND_PAUSED;
          }
          break;
        } else if ((strstr(bufff, "HTTP/1.1 201") != NULL) || (strstr(bufff, "HTTP/1.0 201") != NULL)) {
          ww_from_server = 201;
          err = 0;
          WiFiSpiClient_stop(stop_flag);

          if ((stop_flag != NULL) && (*stop_flag)) {
            return WIFI_DATA_SEND_PAUSED;
          }
          break;
        } else if ((strstr(bufff, "HTTP/1.1 202") != NULL) || (strstr(bufff, "HTTP/1.0 202") != NULL)) {
          ww_from_server = 202;
          err = 0;
          WiFiSpiClient_stop(stop_flag);

          if ((stop_flag != NULL) && (*stop_flag)) {
            return WIFI_DATA_SEND_PAUSED;
          }
          break;
        } else if ((strstr(bufff, "HTTP/1.1 203") != NULL) || (strstr(bufff, "HTTP/1.0 203") != NULL)) {
          ww_from_server = 203;
          err = 0;
          WiFiSpiClient_stop(stop_flag);

          if ((stop_flag != NULL) && (*stop_flag)) {
            return WIFI_DATA_SEND_PAUSED;
          }
          break;
        } else if ((strstr(bufff, "HTTP/1.1 204") != NULL) || (strstr(bufff, "HTTP/1.0 204") != NULL)) {
          ww_from_server = 204;
          err = 0;
          WiFiSpiClient_stop(stop_flag);

          if ((stop_flag != NULL) && (*stop_flag)) {
            return WIFI_DATA_SEND_PAUSED;
          }
          break;
        } else if ((strstr(bufff, "HTTP/1.1 205") != NULL) || (strstr(bufff, "HTTP/1.0 205") != NULL)) {
          ww_from_server = 205;
          err = 0;
          WiFiSpiClient_stop(stop_flag);

          if ((stop_flag != NULL) && (*stop_flag)) {
            return WIFI_DATA_SEND_PAUSED;
          }
          break;
        } else {
          err = -1;
        }
      }

      pos += len;
      printk("----------------pos = pos+len = %d -------------------------\n", pos);

      if (50 < pos) {
        WiFiSpiClient_stop(stop_flag);

        if ((stop_flag != NULL) && (*stop_flag)) {
          return WIFI_DATA_SEND_PAUSED;
        }
        break;
      }
    } else {
      // if the server's disconnected, stop the client:
      if (!WiFiSpiClient_connected(stop_flag)) {
        if ((stop_flag != NULL) && (*stop_flag)) {
          return WIFI_DATA_SEND_PAUSED;
        }
        printk("\ndisconnecting from server.\n");
        WiFiSpiClient_stop(stop_flag);

        if ((stop_flag != NULL) && (*stop_flag)) {
          return WIFI_DATA_SEND_PAUSED;
        }
        break;
      }

      if ((stop_flag != NULL) && (*stop_flag)) {
        return WIFI_DATA_SEND_PAUSED;
      }
      k_msleep(100U);
    }
  }

  if (err) {
    printk("\nfailed\n");
  } else {
    printk("\nSuccess\n");
  }
  return err;
}

// TODO
/*
 * added to read pending response from auto ai srever.
 * Use this function later to read response and update audio file address in external QSPI flash.
 */
int8_t Get_pending_data_from_auto_ai(bool *stop_flag) {
  // if the server's disconnected, stop the client:
  if (!WiFiSpiClient_connected(stop_flag)) {
    return -1;
  }

  uint8_t len = 0;
  uint8_t pos = 0;
  char bufff[50] = {0};
  int8_t err = -1;
  unsigned long int time_stamp;

  time_stamp = k_uptime_get();

  while (k_uptime_get() <= (time_stamp + 5000)) {
    len = WiFiSpiClient_available(stop_flag);
    if (len) {
      // len = (len >= 49)?49:len;
      len = ((len + pos) >= 49) ? (49 - pos) : len;
      printk("----len = %d----\n", len);
      WiFiSpiClient_read_buff(&bufff[pos], len, stop_flag);
      printk("%s", &bufff[pos]);

      if (err) {
        if ((strstr(bufff, "HTTP/1.0 200 OK") != NULL) || (strstr(bufff, "HTTP/1.1 200 OK") != NULL)) {
          err = 0;
          WiFiSpiClient_stop(stop_flag);
          break;
        } else {
          err = -1;
        }
      }
      pos += len;

      if (50 < pos) {
        WiFiSpiClient_stop(stop_flag);
        break;
      }
    } else {
      // if the server's disconnected, stop the client:
      if (!WiFiSpiClient_connected(stop_flag)) {
        printk("\ndisconnecting from server.\n");
        WiFiSpiClient_stop(stop_flag);
        break;
      }
      k_msleep(25U);

      if ((stop_flag != NULL) && (*stop_flag)) {
        return WIFI_DATA_SEND_PAUSED;
      }
    }
  }

  if (err) {
    printk("\nfailed\n");
  } else {
    printk("\nSuccess\n");
  }
  return err;
}

void check_and_stop_prev_client(bool *stop_flag) {
  if (!WiFiSpiClient_connected(stop_flag)) {
    printk("\ndisconnecting from server.\n");
    WiFiSpiClient_stop(stop_flag);
  }
}

int Send_data_to_auto_ai(audio_auto_ai_parameters_t audio_auto_ai_params,
    uint8_t *audio_buff, uint16_t buff_len, bool *stop_flag) {
  uint8_t wifi_status = ADO_WiFi_status(stop_flag);

  if ((stop_flag != NULL) && (*stop_flag)) {
    return WIFI_DATA_SEND_PAUSED;
  }

  if (WL_CONNECTED != wifi_status) {
    printk("WiFi Network not connected status = %d\n", wifi_status);
    return WL_CONNECTION_LOST; // return error
  }

  check_and_stop_prev_client(stop_flag);

  if ((stop_flag != NULL) && (*stop_flag)) {
    return WIFI_DATA_SEND_PAUSED;
  }

  wifi_status = multipart_sendFile(audio_auto_ai_params, audio_buff, buff_len, stop_flag);

  // check for wifi send paused or not due to commands.
  if (wifi_status) {
    return wifi_status;
  }

  printk("++++++++++++++++++++++++++++++\n");
  // get server status from wifi module.
  return Get_data_from_auto_ai(stop_flag);
}

/* added for downloading STM Hex file */
bool read_and_cmp_stm_fw_version(char *fw_ver, bool *stop_flag) {
  int len = 0, body_pos = 0, header_end = 0, body_length = 0, num_of_next_line = 0, chunk_no = 0;
  char confirmation_string[] = "HTTP/1.1 200 OK\r\n";
  bool header_completed = false, body_completed = false;
  char header[1000] = {0};
  char body_2[15] = {0};

  while (1) {
    len = WiFiSpiClient_available(stop_flag);
    if (len) {
      char data[len];
      WiFiSpiClient_read_buff(data, len, stop_flag);

      if (!header_completed /*&& !body_completed*/) {
        if (strstr(data, confirmation_string)) {
          // printk("Confirmation string found...\n");
          int i;

          for (i = 17; (!(data[i - 3] == '\n' && data[i - 1] == '\n') && i <= len); i++) {
            header[i - 17] = data[i];
          }

          header_end = i - 1;
          printk("\nHeader is: \n");

          for (int j = 0; j <= header_end - 17; j++) {
            printk("%c", header[j]);
          }

          process_header(header, 0, header_end - 17, &num_of_next_line);
          printk("Number of lines in the header are: %d\n", num_of_next_line);
          struct key_value store_key_value[num_of_next_line - 1];
          process_header_second(header, 0, header_end - 17, num_of_next_line - 1, store_key_value);
          char content_length[] = "Content-Length";

          for (int j = 0; j < num_of_next_line - 1; j++) {
            printk("%s -> %s\n", store_key_value[j].key, store_key_value[j].value);
            if (strcmp(store_key_value[j].key, content_length) == 0) {
              body_length = atoi(store_key_value[j].value);
            }
          }
          printk("\nThe body length is: %d\n", body_length);
          header_completed = true;

          if (len > i) {
            memcpy(body_2, data + i, len - i);
            body_pos = len - i;
            // printk("Body pos is: %d\n", body_pos);

            if (body_pos + (chunk_no * MAX_LEN) >= body_length) {
              printk("Completed in one go\n");
              *(body_2 + body_pos) = '\0';
              printk("%s\n", body_2);
              body_completed = true;
              break;
            }
          }
        } else {
          printk("Confirmation Not string found...\n");
          printk("%s\n", data);
        }
      }
    } else {
      if (!WiFiSpiClient_connected(stop_flag)) {
        printk("\ndisconnecting from server.\n");
        WiFiSpiClient_stop(stop_flag);
        break;
      }
      k_msleep(100);
    }
  }
  if (header_completed && body_completed) {
    // printk("Received complete Body\n");
    //  Compare current version string with received version string.
    if (strcmp(fw_ver, body_2)) {
      strcpy(fw_ver, body_2);
      printk("before storing in qspi = %s\n", fw_ver); /// added by balakrishna
      return true;
    }
  }
  return false;
}

int Download_stm_hex_file_from_server(bool *stop_flag) {
  int len = 0, body_pos = 0, header_end = 0, body_length = 0, num_of_next_line = 0, chunk_no = 0;
  char confirmation_string[] = "HTTP/1.1 200 OK\r\n";
  bool header_completed = false, body_completed = false;
  char header[1000] = {0};
  int err = -1;

  while (1) {
    if ((stop_flag != NULL) && (*stop_flag)) {
      return WIFI_DATA_SEND_PAUSED;
    }

    len = WiFiSpiClient_available(stop_flag);

    if (len) {
      // printk("\nlen is %d\n",len);
      char data[len];
      WiFiSpiClient_read_buff(data, len, stop_flag);

      if (!header_completed /*&& !body_completed*/) {
        if (strstr(data, confirmation_string)) {
          printk("Confirmation string found...\n");
          int i;

          for (i = 17; (!(data[i - 3] == '\n' && data[i - 1] == '\n') && i <= len); i++) {
            header[i - 17] = data[i];
          }
          header_end = i - 1;
          printk("\nHeader is: \n");

          for (int j = 0; j <= header_end - 17; j++) {
            printk("%c", header[j]);
          }

          process_header(header, 0, header_end - 17, &num_of_next_line);
          printk("Number of lines in the header are: %d\n", num_of_next_line);
          struct key_value store_key_value[num_of_next_line]; // NOTE: adjust
          process_header_second(header, 0, header_end - 17, num_of_next_line - 1, store_key_value);
          char content_length[] = "Content-Length";

          for (int j = 0; j < num_of_next_line - 3; j++) {
            printk("%s -> %s\n", store_key_value[j].key, store_key_value[j].value);
            if (strcmp(store_key_value[j].key, content_length) == 0) {
              body_length = atoi(store_key_value[j].value);
            }
          }

          printk("\nThe body length is: %d\n", body_length);
          header_completed = true;

          if (len > i) {
            memcpy(body + 2, data + i, len - i);
            body_pos = len - i;
            printk("Body pos is: %d\n", body_pos);

            if (body_pos + (chunk_no * MAX_LEN) >= body_length) {
              printk("Completed in one go\n");
              *(body + 2 + body_pos) = '\0';
              // printk("%s\n", body + 2);
              //  Validate and Store data into external QSPI flash memory
              body[0] = body_pos & 0xFF;
              body[1] = (body_pos >> 8) & 0xFF;

              if (validate_stm_new_hex_file_chunk(&body[0], body_pos)) {
                // printk("%s\n", body + 2);
                printk("\ndisconnecting from server.\n");
                WiFiSpiClient_stop(stop_flag);
                return -1;
              }
              store_stm32_hex_file(body, body_pos + 2);
              body_completed = true;
              break;
            }
          }
        } else {
          printk("Confirmation Not string found...\n");
          printk("%s\n", data);
          WiFiSpiClient_stop(stop_flag);
          return -1;
        }
      } else if (header_completed && !body_completed) {
        if (body_pos + len >= MAX_LEN) {
          chunk_no++;
          memcpy(body + 2 + body_pos, data, (MAX_LEN - body_pos));
          *(body + 2 + MAX_LEN) = '\0';
          // printf("%s", body + 2);
          //  Validate and Store data into external QSPI flash memory
          body[0] = MAX_LEN & 0xFF;
          body[1] = (MAX_LEN >> 8) & 0xFF;

          if (validate_stm_new_hex_file_chunk(&body[0], MAX_LEN)) {
            // printk("%s\n", body + 2);
            printk("\ndisconnecting from server.\n");
            WiFiSpiClient_stop(stop_flag);
            return -1;
          }

          store_stm32_hex_file(body, MAX_LEN + 2);
          memcpy(body + 2, data + (MAX_LEN - body_pos), (len + body_pos - (MAX_LEN)));
          body_pos = len + body_pos - (MAX_LEN);
        } else {
          memcpy(body + 2 + body_pos, data, len);
          body_pos += len;
        }
        if (body_pos + (chunk_no * MAX_LEN) >= body_length) {
          *(body + 2 + body_pos) = '\0';
          // printf("%s\n", body + 2);
          //  Validate and Store data into external QSPI flash memory
          body[0] = body_pos & 0xFF;
          body[1] = (body_pos >> 8) & 0xFF;

          if (validate_stm_new_hex_file_chunk(&body[0], body_pos)) {
            // printk("%s\n", body + 2);
            printk("\ndisconnecting from server.\n");
            WiFiSpiClient_stop(stop_flag);
            return -1;
          }
          store_stm32_hex_file(body, body_pos + 2);
          body_completed = true;
          break;
        }
      }
    } else {
      if (!WiFiSpiClient_connected(stop_flag)) {
        printk("\ndisconnecting from server.\n");
        WiFiSpiClient_stop(stop_flag);
        break;
      }
      k_msleep(100);
    }
  }
  if (header_completed && body_completed) {
    printk("Received complete Body\n");
    err = 0;
  }
  return err;
}

int8_t process_header(char header[], int start, int end, int *line) {
  for (int i = start; i <= end; i++) {
    if (header[i] == '\n')
      (*line)++;
  }
  return 0;
}

void process_header_second(char header[], int start, int end, int num_of_keys,
    struct key_value *store_key_value) {
  int temp_start = start;
  int temp_end_key = 0;
  int position_of_key = num_of_keys;

  for (int i = start; i <= end; i++) {
    if (header[i] == ':') {
      temp_end_key = i - 1;
      int j, l;

      for (j = temp_start, l = 0; j <= temp_end_key; j++, l++) {
        store_key_value[num_of_keys - position_of_key].key[l] = header[j];
      }
      store_key_value[num_of_keys - position_of_key].key[l] = '\0';
      temp_start = i + 2;

      while (header[i] != '\n') {
        i++;
      }
      temp_end_key = i - 2;

      for (j = temp_start, l = 0; j <= temp_end_key; j++, l++) {
        store_key_value[num_of_keys - position_of_key].value[l] = header[j];
      }
      store_key_value[num_of_keys - position_of_key].value[l] = '\0';
      temp_start = i + 1;
      position_of_key--;
    }
  }
}

/* added for downloading STM Hex file */
bool request_and_cmp_stm_fw_version(char *fw_ver, bool *stop_flag) {
  // IPAddress server_GET = {34, 93, 125, 222};
  // IPAddress server_GET = {35, 200, 239, 95};
  IPAddress server_GET = {34, 93, 142, 131};

  uint8_t sock = SOCK_NOT_AVAIL;
  sock = WiFiSpi_getSocket();

  if (sock == SOCK_NOT_AVAIL) {
    printk("Socket Not Available\n");
    WiFiSpi_Socket_init();
  }

  //Write IP address for update STM Firmware using OTA
  if (WiFiSpiClient_connect_With_Ip(server_GET, 8501, stop_flag)) {
    printk("\nconnected to server\n");
    WiFiSpiClient_write_in("GET /firmware/version HTTP/1.1\r\n"
                           "Host: 34.93.142.131:8501\r\n"
                           "Connection: close\r\n\r\n",
        stop_flag);

    return read_and_cmp_stm_fw_version(fw_ver, stop_flag);
  } else {
    printk("Failed to connect to server......\n");
  }
  return false;
}

int request_and_download_stm_hex_file_from_server(bool *stop_flag) {
  // IPAddress server_GET = {34, 93, 125, 222};
  // IPAddress server_GET = {35, 200, 239, 95};
  IPAddress server_GET = {34, 93, 142, 131};

  int ret = -1;
  int i = 6; // 6 -> 4
  while (--i) {
    if (WiFiSpiClient_connect_With_Ip(server_GET, 8501, stop_flag)) {
      printk("\nConnected to server\n");
      ret = WiFiSpiClient_write_in("GET /firmware/file HTTP/1.1\r\n"
                                   "Host: 34.93.142.131:8503\r\n"
                                   "Connection: close\r\n\r\n",
          stop_flag);
      printk("\nret = %d\n", ret);
      if (ret) {
        ret = Download_stm_hex_file_from_server(stop_flag);
        break;
      }
    } else {
      // printk("connecting to server no.of.attempts is %d max attaempts is 4",i);
      printk("Failed to connect for downloading the file......\n");
    }

    if (i == 3) {
      // printk("WiFi Hardware reseting...\n");
      // WiFi_module_hard_reset();
      WiFi_Network_Connect(stop_flag); // 25sec
    }
    k_msleep(500U);
  }
  return ret;
}

// -------------- ESP32 WiFi Module Self OTA APIs -------------------------

int ESP32_startSelfOTAServer(void) {
  return WiFiSpi_startSelfOTAServer();
}

int ESP32_stopSelfOTAServer(void) {
  return WiFiSpi_stopSelfOTAServer();
}

int ESP32_getSelfOTAStatus(uint8_t *OTAPercent) {
  return WiFiSpi_getSelfOTAStatus(OTAPercent);
}

void ESP32_SelfOTA(bool *esp32OTAstart_stop_flag) {
  int8_t res = 0;
  uint8_t OTA_Percent = 1;

  char wifi_fw_str[15] = {0};
  wifi_fw_str[14] = 0;

  uint8_t evt_wifi_msg[6] = {0};
  uint8_t evt_wifi_msg_len = 0;

  strcpy(wifi_fw_str, WiFiSpi_firmwareVersion());
  printk("Firmware protocol version = %s\n", wifi_fw_str);

  res = ESP32_startSelfOTAServer();

  // Check WiFi Local IP Command
  IPAddress Lip = WiFiSpi_localIP();
  printk("Local IP Address: %d.%d.%d.%d\n", Lip.first_byte,
      Lip.second_byte, Lip.third_byte, Lip.fourth_byte);

  printk("1. OTA resp = %d\t", res);

  evt_wifi_msg[0] = res;
  evt_wifi_msg[1] = Lip.first_byte;
  evt_wifi_msg[2] = Lip.second_byte;
  evt_wifi_msg[3] = Lip.third_byte;
  evt_wifi_msg[4] = Lip.fourth_byte;
  evt_wifi_msg_len = 5;
  ADO_notify_ble(cmd_mgr_thrd_id,
      ADO_OTA_UPDATE_RESPONSE,
      WIFI_MOD_OTA_START,
      WIFI_MOD_OTA_START_RESP,
      evt_wifi_msg, evt_wifi_msg_len);

  if (res == OTA_SERVER_STARTED) {
    while (1) {
      k_msleep(2000U);
      if ((esp32OTAstart_stop_flag != NULL) && !(*esp32OTAstart_stop_flag)) {
        printk("ESP32 Stop OTA command received...\n");
        res = WiFiSpi_stopSelfOTAServer();
        evt_wifi_msg[0] = res;
        evt_wifi_msg_len = 1;
        ADO_notify_ble(cmd_mgr_thrd_id,
            ADO_OTA_UPDATE_RESPONSE,
            WIFI_MOD_OTA_STOP,
            WIFI_MOD_OTA_STOP_RESP,
            evt_wifi_msg, evt_wifi_msg_len);
        k_msleep(1000U);
        return;
      }

      res = WiFiSpi_getSelfOTAStatus(&OTA_Percent);
      printk("2. OTA resp = %d, Percent = %d\t", res, OTA_Percent);

      switch (res) {
      case OTA_NOT_INITIATED:
        printk("OTA not Yet Started..\t");
        break;

      case WIFI_NOT_CONNECTED:
        printk("WiFi Not Connected..\t");
        break;

      case OTA_SERVER_STARTED:
        printk("OTA Server Started..\t");
        break;

      case OTA_SERVER_RUNNING:
        printk("OTA Server Running..\t");
        break;

      case OTA_SERVER_ERROR:
        printk("Error while creating OTA Server..\t");
        break;

      case OTA_SERVER_STOPPED:
        printk("OTA Server Stopped..\t");
        break;

      case OTA_UPDATE_STARTED:
        printk("OTA Update Started..\t");
        break;

      case OTA_UPDATING:
        printk("OTA Update Running..\t");
        break;

      case OTA_UPDATE_DONE:
        printk("OTA Completed..\t");
        break;

      case ERROR_WHILE_OTA:
        printk("Error While OTA..\t");
        break;

      default:
        break;
      }

      evt_wifi_msg[0] = res;
      evt_wifi_msg[1] = Lip.first_byte;
      evt_wifi_msg[2] = Lip.second_byte;
      evt_wifi_msg[3] = Lip.third_byte;
      evt_wifi_msg[4] = Lip.fourth_byte;
      evt_wifi_msg[5] = OTA_Percent;
      evt_wifi_msg_len = 6;
      ADO_notify_ble(cmd_mgr_thrd_id,
          ADO_OTA_UPDATE_RESPONSE,
          WIFI_MOD_OTA_STATUS,
          WIFI_MOD_OTA_STATUS_RESP,
          evt_wifi_msg, evt_wifi_msg_len);

      if ((res == OTA_UPDATE_DONE) || (res == OTA_NOT_INITIATED)) {
        break;
      } else if (res == WIFI_NOT_CONNECTED) {
        WiFi_Network_Connect(NULL);
      }
    }
    k_msleep(3000U);
    printk("3. Completed OTA.\n");

    if (strcmp(wifi_fw_str, WiFiSpi_firmwareVersion())) {
      strcpy(wifi_fw_str, WiFiSpi_firmwareVersion());
      printk("OTA success\nFirmware protocol version = %s\n", wifi_fw_str);

      evt_wifi_msg[0] = OTA_UPDATE_DONE;
      evt_wifi_msg[1] = Lip.first_byte;
      evt_wifi_msg[2] = Lip.second_byte;
      evt_wifi_msg[3] = Lip.third_byte;
      evt_wifi_msg[4] = Lip.fourth_byte;
      evt_wifi_msg[5] = OTA_Percent;
      evt_wifi_msg_len = 6;
      ADO_notify_ble(cmd_mgr_thrd_id,
          ADO_OTA_UPDATE_RESPONSE,
          WIFI_MOD_OTA_STATUS,
          WIFI_MOD_OTA_STATUS_RESP,
          evt_wifi_msg, evt_wifi_msg_len);
    } else {
      strcpy(wifi_fw_str, WiFiSpi_firmwareVersion());
      printk("OTA failed\nFirmware protocol version = %s\n", wifi_fw_str);

      evt_wifi_msg[0] = OTA_UPDATE_FAILED;
      evt_wifi_msg[1] = Lip.first_byte;
      evt_wifi_msg[2] = Lip.second_byte;
      evt_wifi_msg[3] = Lip.third_byte;
      evt_wifi_msg[4] = Lip.fourth_byte;
      evt_wifi_msg[5] = OTA_Percent;
      evt_wifi_msg_len = 6;
      ADO_notify_ble(cmd_mgr_thrd_id,
          ADO_OTA_UPDATE_RESPONSE,
          WIFI_MOD_OTA_STATUS,
          WIFI_MOD_OTA_STATUS_RESP,
          evt_wifi_msg, evt_wifi_msg_len);
    }
  }
}
#endif // ESP32

#ifdef ESP8266
int multipart_sendFile(audio_auto_ai_parameters_t audio_auto_ai_params,
    uint8_t *audio_buff, uint16_t buff_len, bool *stop_flag) {
  IPAddress server = {34, 132, 219, 249}; // IP address of Auto-AI Server.

  int k = 0;
  int ret = 0;
  int length = 0;
  char Slength[5];

  length = 996 + audio_auto_ai_params.params_str_length + buff_len + WAV_AUDIO_HEADER_LENGTH;

  itoa(length, Slength, 10);

  printk("\nStarting connection to server...\n");
  if (WiFiSpiClient_connect_With_Ip(server, 3000)) {
    printk("\nconnected to server\n");
    ret = WiFiSpiClient_write_in("POST /resource HTTP/1.1\r\nHost: 34.132.219.249:3000\r\n"
                                 "Content-Type: multipart/form-data; boundary=------------------------d74496d66958873e\r\n" // 88
                                 "Content-Length: ");
    ret = WiFiSpiClient_write_in(Slength);
    printk("size 1 = %d \n", ret); // added by samuel
    k_msleep(WiFi_SEND_DELAY);     // added by samuel
    if ((stop_flag != NULL) && (*stop_flag)) {
      return WIFI_DATA_SEND_PAUSED;
    }

    ret = WiFiSpiClient_write_in("\r\nExpect: 100-continue\r\n"
                                 "\r\n--------------------------d74496d66958873e\r\n"      // 46
                                 "Content-Disposition: form-data; name=\"status\"\r\n\r\n" // 49
                                 "backlog"                                                 // 7
                                 "\r\n--------------------------d74496d66958873e\r\n"      // 46
                                 "Content-Disposition: form-data; name=\"csv\"\r\n\r\n");  // 46
    ret = WiFiSpiClient_write_in(audio_auto_ai_params.csv);
    printk("size 2 = %d \n", ret); // added by samuel
    k_msleep(WiFi_SEND_DELAY);     // added by samuel
    if ((stop_flag != NULL) && (*stop_flag)) {
      return WIFI_DATA_SEND_PAUSED;
    }

    ret = WiFiSpiClient_write_in("\r\n--------------------------d74496d66958873e\r\n"       // 46
                                 "Content-Disposition: form-data; name=\"model\"\r\n\r\n"   // 48
                                 "61ef9d4324e24310d102c5c7"                                 // 24
                                 "\r\n--------------------------d74496d66958873e\r\n"       // 46
                                 "Content-Disposition: form-data; name=\"label\"\r\n\r\n"); // 48
    ret = WiFiSpiClient_write_in(audio_auto_ai_params.label);
    printk("size 3 = %d \n", ret);
    k_msleep(WiFi_SEND_DELAY);
    if ((stop_flag != NULL) && (*stop_flag)) {
      return WIFI_DATA_SEND_PAUSED;
    }

    ret = WiFiSpiClient_write_in("\r\n--------------------------d74496d66958873e\r\n"     // 46
                                 "Content-Disposition: form-data; name=\"tag\"\r\n\r\n"); // 46
    ret = WiFiSpiClient_write_in(audio_auto_ai_params.tag);
    ret = WiFiSpiClient_write_in("\r\n--------------------------d74496d66958873e\r\n"                  // 46
                                 "Content-Disposition: form-data; name=\"confidence_score\"\r\n\r\n"); // 59
    ret = WiFiSpiClient_write_in(audio_auto_ai_params.ww_confidence_score);
    printk("size 4 = %d \n", ret);
    k_msleep(WiFi_SEND_DELAY);
    if ((stop_flag != NULL) && (*stop_flag)) {
      return WIFI_DATA_SEND_PAUSED;
    }

    ret = WiFiSpiClient_write_in("\r\n--------------------------d74496d66958873e\r\n"               // 46
                                 "Content-Disposition: form-data; name=\"prediction\"\r\n\r\n"      // 53
                                 "predicted"                                                        // 9
                                 "\r\n--------------------------d74496d66958873e\r\n"               // 46
                                 "Content-Disposition: form-data; name=\"model_type\"\r\n\r\n"      // 53
                                 "audio"                                                            // 5
                                 "\r\n--------------------------d74496d66958873e\r\n"               // 46
                                 "Content-Disposition: form-data; name=\"resource\"; filename=\""); // 59
    ret = WiFiSpiClient_write_in(audio_auto_ai_params.file_name);
    printk("size 5 = %d \n", ret);
    k_msleep(WiFi_SEND_DELAY);
    if ((stop_flag != NULL) && (*stop_flag)) {
      return WIFI_DATA_SEND_PAUSED;
    }

    ret = WiFiSpiClient_write_in("\"\r\nContent-Type: audio/wav\r\n\r\n"); // 30
    ret = WiFiSpiClient_write(wav_format_header_array, WAV_AUDIO_HEADER_LENGTH);
    printk("size 6 = %d \n", ret);
    k_msleep(WiFi_SEND_DELAY);
    if ((stop_flag != NULL) && (*stop_flag)) {
      return WIFI_DATA_SEND_PAUSED;
    }

    while (k < buff_len) {
      if ((buff_len - k) < WIFI_AUDIO_TRANSFER_BYTES) {
        ret = WiFiSpiClient_write(audio_buff + k, (buff_len - k));
        printk("size %d = %d \n", (7 + k), ret);
        k_msleep(WiFi_SEND_DELAY);
        if ((stop_flag != NULL) && (*stop_flag)) {
          return WIFI_DATA_SEND_PAUSED;
        }
      } else {
        ret = WiFiSpiClient_write(audio_buff + k, WIFI_AUDIO_TRANSFER_BYTES);
        printk("size %d = %d \n", (7 + k), ret);
        k_msleep(WiFi_SEND_DELAY);
        if ((stop_flag != NULL) && (*stop_flag)) {
          return WIFI_DATA_SEND_PAUSED;
        }
      }
      k += WIFI_AUDIO_TRANSFER_BYTES;
    }

    ret = WiFiSpiClient_write_in("\r\n--------------------------d74496d66958873e--\r\n\r\n"); // 50
    printk("size last = %d \n", ret);
    k_msleep(WiFi_SEND_DELAY);
    if ((stop_flag != NULL) && (*stop_flag)) {
      return WIFI_DATA_SEND_PAUSED;
    }
  } else {
    printk("\nfailed to connect server\n");
    return SERVER_CONNECTION_FAILED;
  }
  return 0;
}

int8_t Get_data_from_auto_ai(bool *stop_flag) {
  uint8_t len = 0;
  uint8_t pos = 0;
  char bufff[50] = {0};
  int8_t err = -1;
  unsigned long int time_stamp;

  time_stamp = k_uptime_get();

  while (k_uptime_get() <= (time_stamp + 5000)) {
    len = WiFiSpiClient_available();
    if (len) {
      // len = (len >= 49)?49:len;
      len = ((len + pos) >= 49) ? (49 - pos) : len;
      printk("----len = %d----\n", len);
      WiFiSpiClient_read_buff(&bufff[pos], len);
      printk("%s", &bufff[pos]);

      if (err) {
        if ((strstr(bufff, "HTTP/1.0 200 OK") != NULL) || (strstr(bufff, "HTTP/1.1 200 OK") != NULL)) {
          err = 0;
          WiFiSpiClient_stop();
          break;
        } else {
          err = -1;
        }
      }

      pos += len;
      if (50 < pos) {
        WiFiSpiClient_stop();
        break;
      }
    } else {
      // if the server's disconnected, stop the client:
      if (!WiFiSpiClient_connected()) {
        printk("\ndisconnecting from server.\n");
        WiFiSpiClient_stop();
        break;
      }
      k_msleep(25U);
      if ((stop_flag != NULL) && (*stop_flag)) {
        return WIFI_DATA_SEND_PAUSED;
      }
    }
  }

  if (err) {
    printk("\nfailed\n");
  } else {
    printk("\nSuccess\n");
  }
  return err;
}

// TODO
/*
 * added to read pending response from auto ai srever.
 * Use this function later to read response and update audio file address in external QSPI flash.
 */
int8_t Get_pending_data_from_auto_ai(bool *stop_flag) {
  // if the server's disconnected, stop the client:
  if (!WiFiSpiClient_connected()) {
    return -1;
  }

  uint8_t len = 0;
  uint8_t pos = 0;
  char bufff[50] = {0};
  int8_t err = -1;
  unsigned long int time_stamp;

  time_stamp = k_uptime_get();

  while (k_uptime_get() <= (time_stamp + 5000)) {
    len = WiFiSpiClient_available();
    if (len) {
      // len = (len >= 49)?49:len;
      len = ((len + pos) >= 49) ? (49 - pos) : len;
      printk("----len = %d----\n", len);
      WiFiSpiClient_read_buff(&bufff[pos], len);
      printk("%s", &bufff[pos]);

      if (err) {
        if ((strstr(bufff, "HTTP/1.0 200 OK") != NULL) || (strstr(bufff, "HTTP/1.1 200 OK") != NULL)) {
          err = 0;
          WiFiSpiClient_stop();
          break;
        } else {
          err = -1;
        }
      }

      pos += len;
      if (50 < pos) {
        WiFiSpiClient_stop();
        break;
      }
    } else {
      // if the server's disconnected, stop the client:
      if (!WiFiSpiClient_connected()) {
        printk("\ndisconnecting from server.\n");
        WiFiSpiClient_stop();
        break;
      }
      k_msleep(25U);
      if ((stop_flag != NULL) && (*stop_flag)) {
        return WIFI_DATA_SEND_PAUSED;
      }
    }
  }

  if (err) {
    printk("\nfailed\n");
  } else {
    printk("\nSuccess\n");
  }
  return err;
}

void check_and_stop_prev_client(bool *stop_flag) {
  if (!WiFiSpiClient_connected()) {
    printk("\ndisconnecting from server.\n");
    WiFiSpiClient_stop();
  }
}

int Send_data_to_auto_ai(audio_auto_ai_parameters_t audio_auto_ai_params,
    uint8_t *audio_buff, uint16_t buff_len, bool *stop_flag) {
  uint8_t wifi_status = ADO_WiFi_status(stop_flag);

  if (WL_CONNECTED != wifi_status) {
    printk("WiFi Network not connected status = %d\n", wifi_status);
    return WL_CONNECTION_LOST; // return error
  }

  wifi_status = multipart_sendFile(audio_auto_ai_params, audio_buff, buff_len, stop_flag);

  // check for wifi send paused or not due to commands.
  if (wifi_status) {
    return wifi_status;
  }

  printk("++++++++++++++++++++++++++++++\n");
  // get server status from wifi module.
  return Get_data_from_auto_ai(stop_flag);
}

/* added for downloading STM Hex file */
bool read_and_cmp_stm_fw_version(char *fw_ver, bool *stop_flag) {
  int len = 0, body_pos = 0, header_end = 0, body_length = 0, num_of_next_line = 0, chunk_no = 0;
  char confirmation_string[] = "HTTP/1.1 200 OK\r\n";
  bool header_completed = false, body_completed = false;
  char header[1000] = {0};
  char body_2[15] = {0};

  while (1) {
    len = WiFiSpiClient_available();
    if (len) {
      char data[len];
      WiFiSpiClient_read_buff(data, len);

      if (!header_completed /*&& !body_completed*/) {
        if (strstr(data, confirmation_string)) {
          // printk("Confirmation string found...\n");
          int i;

          for (i = 17; (!(data[i - 3] == '\n' && data[i - 1] == '\n') && i <= len); i++) {
            header[i - 17] = data[i];
          }

          header_end = i - 1;
          printk("\nHeader is: \n");

          for (int j = 0; j <= header_end - 17; j++) {
            printk("%c", header[j]);
          }

          process_header(header, 0, header_end - 17, &num_of_next_line);
          printk("Number of lines in the header are: %d\n", num_of_next_line);
          struct key_value store_key_value[num_of_next_line - 1];
          process_header_second(header, 0, header_end - 17, num_of_next_line - 1, store_key_value);
          char content_length[] = "Content-Length";

          for (int j = 0; j < num_of_next_line - 1; j++) {
            printk("%s -> %s\n", store_key_value[j].key, store_key_value[j].value);
            if (strcmp(store_key_value[j].key, content_length) == 0) {
              body_length = atoi(store_key_value[j].value);
            }
          }

          printk("\nThe body length is: %d\n", body_length);
          header_completed = true;

          if (len > i) {
            memcpy(body_2, data + i, len - i);
            body_pos = len - i;
            // printk("Body pos is: %d\n", body_pos);

            if (body_pos + (chunk_no * MAX_LEN) >= body_length) {
              printk("Completed in one go\n");
              *(body_2 + body_pos) = '\0';
              printk("%s\n", body_2);
              body_completed = true;
              break;
            }
          }
        } else {
          printk("Confirmation Not string found...\n");
          printk("%s\n", data);
        }
      }
    } else {
      if (!WiFiSpiClient_connected()) {
        printk("\ndisconnecting from server.\n");
        WiFiSpiClient_stop();
        break;
      }
      k_msleep(100);
    }
  }
  if (header_completed && body_completed) {
    // printk("Received complete Body\n");
    //  Compare current version string with received version string.
    if (strcmp(fw_ver, body_2)) {
      strcpy(fw_ver, body_2);
      printk("before storing in qspi = %s\n", fw_ver); /// added by balakrishna
      return true;
    }
  }
  return false;
}

int Download_stm_hex_file_from_server(bool *stop_flag) {
  int len = 0, body_pos = 0, header_end = 0, body_length = 0, num_of_next_line = 0, chunk_no = 0;
  char confirmation_string[] = "HTTP/1.1 200 OK\r\n";
  bool header_completed = false, body_completed = false;
  char header[1000] = {0};
  int err = -1;

  while (1) {
    if ((stop_flag != NULL) && (*stop_flag)) {
      return WIFI_DATA_SEND_PAUSED;
    }

    len = WiFiSpiClient_available();
    if (len) {
      char data[len];
      WiFiSpiClient_read_buff(data, len);

      if (!header_completed /*&& !body_completed*/) {
        if (strstr(data, confirmation_string)) {
          printk("Confirmation string found...\n");
          int i;
          for (i = 17; (!(data[i - 3] == '\n' && data[i - 1] == '\n') && i <= len); i++) {
            header[i - 17] = data[i];
          }
          header_end = i - 1;
          printk("\nHeader is: \n");
          for (int j = 0; j <= header_end - 17; j++) {
            printk("%c", header[j]);
          }

          process_header(header, 0, header_end - 17, &num_of_next_line);
          printk("Number of lines in the header are: %d\n", num_of_next_line);

          struct key_value store_key_value[num_of_next_line]; // NOTE: adjust

          process_header_second(header, 0, header_end - 17, num_of_next_line - 1, store_key_value);
          char content_length[] = "Content-Length";
          for (int j = 0; j < num_of_next_line - 3; j++) {
            printk("%s -> %s\n", store_key_value[j].key, store_key_value[j].value);
            if (strcmp(store_key_value[j].key, content_length) == 0) {
              body_length = atoi(store_key_value[j].value);
            }
          }
          printk("\nThe body length is: %d\n", body_length);

          header_completed = true;

          if (len > i) {
            memcpy(body + 2, data + i, len - i);
            body_pos = len - i;
            printk("Body pos is: %d\n", body_pos);

            if (body_pos + (chunk_no * MAX_LEN) >= body_length) {
              printk("Completed in one go\n");
              *(body + 2 + body_pos) = '\0';
              // printk("%s\n", body + 2);
              //  Validate and Store data into external QSPI flash memory
              body[0] = body_pos & 0xFF;
              body[1] = (body_pos >> 8) & 0xFF;
              if (validate_stm_new_hex_file_chunk(&body[0], body_pos)) {
                // printk("%s\n", body + 2);
                printk("\ndisconnecting from server.\n");
                WiFiSpiClient_stop();
                return -1;
              }
              store_stm32_hex_file(body, body_pos + 2);
              body_completed = true;
              break;
            }
          }
        } else {
          printk("Confirmation Not string found...\n");
          printk("%s\n", data);
          WiFiSpiClient_stop();
          return -1;
        }
      } else if (header_completed && !body_completed) {
        if (body_pos + len >= MAX_LEN) {
          chunk_no++;
          memcpy(body + 2 + body_pos, data, (MAX_LEN - body_pos));
          *(body + 2 + MAX_LEN) = '\0';
          // printf("%s", body + 2);
          //  Validate and Store data into external QSPI flash memory
          body[0] = MAX_LEN & 0xFF;
          body[1] = (MAX_LEN >> 8) & 0xFF;
          if (validate_stm_new_hex_file_chunk(&body[0], MAX_LEN)) {
            // printk("%s\n", body + 2);
            printk("\ndisconnecting from server.\n");
            WiFiSpiClient_stop();
            return -1;
          }
          store_stm32_hex_file(body, MAX_LEN + 2);

          memcpy(body + 2, data + (MAX_LEN - body_pos), (len + body_pos - (MAX_LEN)));
          body_pos = len + body_pos - (MAX_LEN);
        } else {
          memcpy(body + 2 + body_pos, data, len);
          body_pos += len;
        }
        if (body_pos + (chunk_no * MAX_LEN) >= body_length) {
          *(body + 2 + body_pos) = '\0';
          // printf("%s\n", body + 2);
          //  Validate and Store data into external QSPI flash memory
          body[0] = body_pos & 0xFF;
          body[1] = (body_pos >> 8) & 0xFF;
          if (validate_stm_new_hex_file_chunk(&body[0], body_pos)) {
            // printk("%s\n", body + 2);
            printk("\ndisconnecting from server.\n");
            WiFiSpiClient_stop();
            return -1;
          }
          store_stm32_hex_file(body, body_pos + 2);
          body_completed = true;
          break;
        }
      }
    } else {
      if (!WiFiSpiClient_connected()) {
        printk("\ndisconnecting from server.\n");
        WiFiSpiClient_stop();
        break;
      }
      k_msleep(100);
    }
  }
  if (header_completed && body_completed) {
    printk("Received complete Body\n");
    err = 0;
  }
  return err;
}

int8_t process_header(char header[], int start, int end, int *line) {
  for (int i = start; i <= end; i++) {
    if (header[i] == '\n')
      (*line)++;
  }
  return 0;
}

void process_header_second(char header[], int start, int end,
    int num_of_keys, struct key_value *store_key_value) {
  int temp_start = start;
  int temp_end_key = 0;
  int position_of_key = num_of_keys;

  for (int i = start; i <= end; i++) {
    if (header[i] == ':') {
      temp_end_key = i - 1;
      int j, l;
      for (j = temp_start, l = 0; j <= temp_end_key; j++, l++) {
        store_key_value[num_of_keys - position_of_key].key[l] = header[j];
      }
      store_key_value[num_of_keys - position_of_key].key[l] = '\0';
      temp_start = i + 2;

      while (header[i] != '\n') {
        i++;
      }
      temp_end_key = i - 2;

      for (j = temp_start, l = 0; j <= temp_end_key; j++, l++) {
        store_key_value[num_of_keys - position_of_key].value[l] = header[j];
      }
      store_key_value[num_of_keys - position_of_key].value[l] = '\0';
      temp_start = i + 1;
      position_of_key--;
    }
  }
}

/* added for downloading STM Hex file */
bool request_and_cmp_stm_fw_version(char *fw_ver, bool *stop_flag) {
  IPAddress server_GET = {34, 93, 125, 222};

  if (WiFiSpiClient_connect_With_Ip(server_GET, 5003)) {
    printk("\nconnected to server\n");
    WiFiSpiClient_write_in("GET /firmware/version HTTP/1.1\r\n"
                           "Host: 34.93.125.222:5003\r\n"
                           "Connection: close\r\n\r\n");

    return read_and_cmp_stm_fw_version(fw_ver, stop_flag);
  } else {
    printk("Failed to connect to server......\n");
  }
  return false;
}

int request_and_download_stm_hex_file_from_server(bool *stop_flag) {
  // IPAddress server_GET = {34, 93, 125, 222};
  IPAddress server_GET = {35, 200, 239, 95};
  int ret = -1;
  int i = 6;
  while (--i) {
    if (WiFiSpiClient_connect_With_Ip(server_GET, 5003)) {
      printk("\nConnected to server\n");
      ret = WiFiSpiClient_write_in("GET /firmware/file HTTP/1.1\r\n"
                                   "Host: 35.200.239.95:5003\r\n"
                                   "Connection: close\r\n\r\n");
      printk("\nret = %d\n", ret);
      if (ret) {
        ret = Download_stm_hex_file_from_server(stop_flag);
        break;
      }
    } else {
      printk("Failed to connect for downloading the file......\n");
    }

    if (i == 3) {
      // printk("WiFi Hardware reseting...\n");
      // WiFi_module_hard_reset();
      WiFi_Network_Connect(stop_flag);
    }
    k_msleep(500U);
  }
  return ret;
}
#endif // ESP8266