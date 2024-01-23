//#include <dk_buttons_and_leds.h>
#include <zephyr/kernel.h>
#include <stdbool.h>
#include <string.h>
#include <zephyr/sys/printk.h>

#include "Includes/ADO_Audio_files.h"
#include "Includes/ADO_Operations_Module.h"
#include "Includes/ADO_Reset_Calib_Module.h"
#include "Includes/ADO_STM32H7_WW.h"
#include "Includes/ADO_WiFi_Module.h"
#include "Includes/AnalogIn.h"
#include "Includes/BLE_Module.h"
#include "Includes/Calibration_Module.h"
#include "Includes/Command_Manager_Module.h"
#include "Includes/Device_Status_Module.h"
#include "Includes/Display_Module.h"
#include "Includes/EEPROM_Module.h"
#include "Includes/IPC_Manager_Module.h"
#include "Includes/Install_ADO.h"
#include "Includes/Intent_Module.h"
#include "Includes/Motor_Control_Module.h"

/* Initialise the default ADO operation speed percentage in ADO
Operations if not set explicitly from ED. */
uint8_t config_pwm_percent = 0; // MIN_OPR_MOTOR_SPEED_PERCENT;

audio_auto_ai_parameters_t audio_data_for_ai; // store user live feedback

/* size of stack area used by each thread */
#define MY_STACK_SIZE 1024
#define MY_STACK_SIZE_INTENT 2048
#define MY_STACK_SIZE_OPERATION 2048
#define CMD_MGR_THRD_STACK_SIZE 2048
#define RESET_CALIB_THRD_STACK_SIZE 1024

#define OTA_BLOCKING

/* scheduling priority used by each thread */
#define MY_PRIORITY 7

// Command manager thread definitions
#define CMD_MGR_THRD_PRIORITY 6

//  Reset calibration thread definitions
#define RESET_CALIB_THRD_PRIORITY 8

// Mailbox declaration
struct k_mbox cmd_mgr_mb;

bool WIFI_ACTIVE();
bool wifi_activity;

bool Motor_active();
bool motor_activity;

bool STM_active();
bool stm_activity;
bool settings_open = false;
bool Display_active();
bool display_activity;
bool Auto_close_timer_off=true;
uint8_t instal_start_cmd = 0;

// Define the command manager thread at compile time,
// so that it needs not to be invoked separately by any
// function but by scheduler immediately after power ON
K_THREAD_DEFINE(cmd_mgr_thrd_id, CMD_MGR_THRD_STACK_SIZE, ADO_CmdMgr_Thread, NULL,
    NULL, NULL, CMD_MGR_THRD_PRIORITY, K_ESSENTIAL, 0);

// Installation thread related variable definition
struct k_thread my_thread_data;
k_tid_t my_tid_install;
K_THREAD_STACK_DEFINE(my_stack_area, MY_STACK_SIZE);

// Calibration thread related variable definition
struct k_thread cal_thread_data;
k_tid_t ado_calib_thrd_id;
K_THREAD_STACK_DEFINE(cal_thread_stack, MY_STACK_SIZE);

// Operation thread related variable definition
struct k_thread opr_thread_data;
k_tid_t ado_opr_thrd_id;
K_THREAD_STACK_DEFINE(opr_thread_stack, MY_STACK_SIZE_OPERATION);

// Intent detection thread related variable definition
struct k_thread intent_thread_data;
k_tid_t ado_intent_thrd_id;
K_THREAD_STACK_DEFINE(intent_thread_stack, MY_STACK_SIZE_INTENT);

// Reset calibration thread related variable definition
struct k_thread reset_calib_thrd_data;
k_tid_t reset_calib_thrd_id;
K_THREAD_STACK_DEFINE(reset_calib_thrd_stack, RESET_CALIB_THRD_STACK_SIZE);

// timer variable
struct k_timer intent_timer, timer;

bool stop_wifi_send = true;
bool esp32_OTA_start = false;

bool emergency_exit = false;

// added for wifi_cred and different operations
bool new_wifi = false;
bool reconnect_to_wifi = false;
bool new_wifi_cred = false;

bool ssid_flag = false;
bool pass_flag = false;
bool ota_info=false;

uint8_t evt_msg[1] = {0};
uint8_t evt_msg_len = 0;

char temp[32] = {0};
char temp_pass[32] = {0};

int ssid_length = 0;
int pass_length = 0;

int auto_close_timer = 60;

/*
 * brief: This will translate the received command and
    act/delegate it to other thread as per command.
 */
void ADO_CmdMgr_Thread() {
  // Enum instances to identify the command
  ado_wifi_control_cmd_t wifi_control_cmd;
  ado_config_cmd_t config_cmd;
  ado_operation_cmd_t operate_cmd;
  ado_dev_stat_cmd_t dev_stat_cmd;
  ado_installation_cmd_t installation_cmd;
  ado_intent_detection_cmd_t intent_detection_cmd;
  ado_reset_calib_cmd_t reset_calib_cmd;
  ado_ota_update_cmd_t ota_update_cmd;

  bool intent_cmd = false;
  bool display_cmd = false;

  printk("###############");
  k_timer_init(&timer, timer_expire, NULL);

  printk("###############");
  k_timer_init(&intent_timer, timer_expire_intent, NULL);

  // Mailbox variables
  struct k_mbox_msg recv_msg;
  struct k_mbox_msg send_msg;

  ado_cmd_mgr_msg_t cmd_mgr_msg;
  ado_cmd_mgr_msg_t ble_notify_msg;

  // initialise cmd_mgr mail box
  k_mbox_init(&cmd_mgr_mb);

  uint8_t msg_type = 0;
  uint8_t cmd_type = 0;
  uint8_t res_type = 0;

  uint8_t evt_msg[1] = {0};
  uint8_t evt_msg_len = 0, lock_msg = 0;

  char temp[32] = {0};
  char temp_pass[32] = {0};

  int ssid_length = 0;
  int pass_length = 0;

  while (1) {
    // prepare the mailbox receive buffer
    recv_msg.info = 0;
    recv_msg.size = sizeof(cmd_mgr_msg);
    recv_msg.rx_source_thread = K_ANY;

    // retrieve and delete the message if recieved
    k_mbox_get(&cmd_mgr_mb, &recv_msg, &cmd_mgr_msg, K_FOREVER);

    // Get the msg type
    msg_type = cmd_mgr_msg.msg_type;
    if (msg_type == COMMAND) {
      // Get the command type
      // TODO: Move the packetisation logic here in future
      cmd_type = cmd_mgr_msg.msg_cmd;

      // Check for the command type
      switch (cmd_type) {
      case ADO_CALIBRATION:
        // set a default user response for server when calibration.
        strcpy(audio_data_for_ai.live_user_response, "Not Sure");
#ifdef OTA_BLOCKING

        if (!ota_updating) {
          stop_wifi_send = true;
          ado_calibration(cmd_mgr_msg.msg_buf); // Identify the command
        } else {
          // TODO: send response as ota updating.
          printk("OTA Updating..\n");
        }
#else
        stop_wifi_send = true;
        ado_calibration(cmd_mgr_msg.msg_buf); // Identify the command
#endif
        cmd_type = 0;
        break;

      case ADO_OPERATIONS:
#ifdef OTA_BLOCKING
        if ((!ota_updating) && (!battery_low)) {
          printk("\n CMD--->Enter into operations\n");

          if (cmd_mgr_msg.msg_buf_len == 2) {
            printk("\nCMD--->-----------------buffer length=2-------------------\n");

            if (cmd_mgr_msg.msg_buf[1] == 4) {
              printk("\nCMD--->---------------Emergency exit -------------------\n");
              stop_wifi_send = true;
              emergency_exit = true;
              new_wifi_cred = false; // added by ashok for wifi_cred
              operate_cmd = cmd_mgr_msg.msg_buf[0];
              ado_operation(operate_cmd);
              cmd_type = 0;
              break;
            } else {
              if (cmd_mgr_msg.msg_buf[1] == 1) {
                printk("\nCMD--->----------------Normal Open-------------------\n");
              }

              // Door Open when Auto Close timer is OFF
              if (cmd_mgr_msg.msg_buf[1] == 2) {
                // door open + Timer Off.......................................................
                // printk("\n CASE 2");
                printk("\nCMD--->----------------Door Open from settings settings"
                       " = %d-------------------\n",
                    settings_open ? 1 : 0);
                // settings_open = true; //IF settings_open = true, boot will not perform any operation. (Even voice)
                stop_wifi_send = true;
                new_wifi_cred = false;
                operate_cmd = cmd_mgr_msg.msg_buf[0];
                ado_operation(operate_cmd);
                cmd_type = 0;
                break;
              }

              if (cmd_mgr_msg.msg_buf[1] == 3) {
                printk("\nCMD--->----------------Quick Grant-------------------\n");
              }

              // Door Open when Auto Close timer is ON
              if (!emergency_exit && !settings_open) {
                // door open + Timer...........................................................
                // printk("\n CASE 4");
                printk("\nCMD--->------No emergency exit No settings_open. executing command-------\n");
                stop_wifi_send = true;
                new_wifi_cred = false;
                operate_cmd = cmd_mgr_msg.msg_buf[0];
                ado_operation(operate_cmd);
              } else {
                if (emergency_exit) {
                  printk("\nCMD--->----------Bot is in Emergency exit---------------\n");
                }

                if (settings_open) {
                  printk("\nCMD--->----------Bot is in  settings_open---------------\n");
                }
              }
            }
          } else {
            printk("\nCMD--->-----------Buffer length=1----------------------------\n");
            operate_cmd = cmd_mgr_msg.msg_buf[0];

            // Perform door close (Auto close timer On + OFF conditions)
            if (operate_cmd == CLOSE) {
              // printk("\n CASE 6");
              // door Close + Timer ON/OFF..............................................
              settings_open = false;
              emergency_exit = false;
              printk("\nCMD--->-----------Close command emergency = 0--------------\n");
            }

            // Perform door close (Auto close timer On + OFF conditions)
            if (!emergency_exit && !settings_open) {
              // printk("\n CASE 7");
              // door Close + Timer ON / OFF................................................
              printk("\nCMD--->-----No emergency exit serving command--------------\n");
              stop_wifi_send = true;
              new_wifi_cred = false;
              operate_cmd = cmd_mgr_msg.msg_buf[0];
              ado_operation(operate_cmd);
            } else {
              printk("CMD--->Door is in emergency exit or dashbord open");
              operate_cmd = cmd_mgr_msg.msg_buf[0];

              if (operate_cmd == OPEN) {
                ado_operation(operate_cmd);
              }
            }
          }
        } else {
          // TODO: send response as ota updating.
          printk("OTA Updating..\n");
        }
#else
        stop_wifi_send = true;
        operate_cmd = cmd_mgr_msg.msg_buf[0];
        ado_operation(operate_cmd);
#endif
        cmd_type = 0;
        break;

      case ADO_INSTALLATION:
#ifdef OTA_BLOCKING

        if (!ota_updating) {
          if (!is_thread_dead((char *)k_thread_state_str(ado_opr_thrd_id,0,0))) {
            instal_start_cmd = cmd_mgr_msg.msg_buf[0];
            cmd_type = 0;
            // TODO: send response as ota updating.
            printk("====>ado_opr_thrd_id Running\n");
            break;
          }

          stop_wifi_send = true;
          // Record the configurable params as per the received command
          // Init the enum with received value to compare
          installation_cmd = cmd_mgr_msg.msg_buf[0];
          ado_installation(installation_cmd);
          instal_start_cmd = 0;
        } else {
          // TODO: send response as ota updating.
          printk("OTA Updating.. OR ado_opr_thrd_id Running\n");
        }
#else
        stop_wifi_send = true;
        // Record the configurable params as per the received command
        // Init the enum with received value to compare
        installation_cmd = cmd_mgr_msg.msg_buf[0];
        ado_installation(installation_cmd);
#endif
        cmd_type = 0;
        break;

      case ADO_DEVICE_STATUS:
        dev_stat_cmd = cmd_mgr_msg.msg_buf[0];
        printk("ADO_DEVICE_STATUS  %d\r\n", dev_stat_cmd);
        ado_dev_status(dev_stat_cmd);
        cmd_type = 0;
        break;

      case ADO_CONFIG:
        printk("ADO_CONFIG\r\n");
        // Record the configurable params as per the received command
        // Init the enum with received value to compare
        config_cmd = cmd_mgr_msg.msg_buf[0];

        switch (config_cmd) {
        case SET_SPEED:
          // Record the required PWM Percentage to rotate the motor in next steps
          // for now, update a global variable and later can be stored to EEPROM/Flash
          printk("SET_SPEED\r\n");

          config_pwm_percent = cmd_mgr_msg.msg_buf[1];
          printf("\n Recived Speed = %d", config_pwm_percent);
          Gear_motor_params.end_RPM = speedmap(config_pwm_percent, 30, 100, 40,
              Gear_motor_params.fw_speed_cap);
          // Gear_motor_params.bw_end_RPM = speedmap(config_pwm_percent,30,100,40,Gear_motor_params.bw_speed_cap);

          // write to EEPROM
          save_config_data();

          // Notify over BLE that the configurations are modified
          ADO_notify_ble(ble_notify_thrd_id, ADO_CONFIG_RESPONSE, SET_SPEED, SPEED_SET, NULL, 0);
          config_cmd = 0;
          Speed(Gear_motor_params.start_RPM, Gear_motor_params.end_RPM,
              Gear_motor_params.gearbox_ratio, Gear_motor_params.micro_steps,
              Gear_motor_params.step_angle, Gear_motor_params.bw_start_RPM,
              Gear_motor_params.end_RPM);
          break;

        case INTENT_CONTROL:
          stop_wifi_send = true;
          printk("INTENT_CONTROL\r\n");
          // Get the received command to toggle the intent detection
          intent_cmd = (bool)cmd_mgr_msg.msg_buf[1];
          ado_toggle_intent(intent_cmd);
          config_cmd = 0;
          break;

        case REQ_ADO_CONFIG:
          printk("REQ_ADO_CONFIG    %s\r\n", ADO_FW_VER_STR);
          ado_get_devConf();
          config_cmd = 0;
          break;

        case DISPLAY_CONTROL: // Added for testing only via BLE
          printk("DISPLAY_CONTROL\r\n");
          // Get the received command to toggle the ADO DIPLAY
          display_cmd = (bool)cmd_mgr_msg.msg_buf[1];
          ado_toggle_display(display_cmd);
          config_cmd = 0;
          break;

        case AUTO_CLOSE_TIME:
          // Record the required PWM Percentage to rotate the motor in next steps
          // for now, update a global variable and later can be stored to EEPROM/Flash
          printk("STORING AUTO CLOSE TIME\r\n");
          auto_close_timer = cmd_mgr_msg.msg_buf[1];
          auto_close_timer = auto_close_timer * 2; // Uttam has doubt of this line
          printk("\nAuto close timer =%d\n", auto_close_timer / 2);
          Auto_close_timer_off=true;
          // write to EEPROM
          save_auto_close_data();
          k_msleep(100);
          load_auto_close_data();

          // Notify over BLE that the configurations are modified
          ADO_notify_ble(ble_notify_thrd_id, ADO_CONFIG_RESPONSE, AUTO_CLOSE_TIME,
              AUTO_CLOSE_TIME_RESP, auto_close_timer, 3);
          config_cmd = 0;
          break;

        case AUTO_CLOSE_TIMER_OFF:
          printk("AUTO CLOSE TIMER OFF\r\n");
          Auto_close_timer_off=false;
          // Notify over BLE that the configurations are modified
          ADO_notify_ble(ble_notify_thrd_id, ADO_CONFIG_RESPONSE, AUTO_CLOSE_TIMER_OFF,
              AUTO_CLOSE_TIMER_OFF_RESP, NULL, 0);
          config_cmd = 0;
          break;

        default:
          break;
        }
        cmd_type = 0;
        break;

      case ADO_INTENT_REQUEST:
        stop_wifi_send = true;
        intent_detection_cmd = cmd_mgr_msg.msg_buf[0];
        ado_intent(intent_detection_cmd);

        // Sending audio file to server
        if (cmd_mgr_msg.msg_buf[0] == 5) {
          printf("\n Start Audio file dump to RELF.ai..........................................\n");
          current_power_mode = DUMPING;
          dump_audio_file_to_server = true;
        }

        if (cmd_mgr_msg.msg_buf[0] == 6) {
          printf("\n Stop Audio file dump to RELF.ai............................................\n");
          current_power_mode = DUMPING_STOP;
          dump_audio_file_to_server = false;
        }
        // upto this for audio file sending to server.

        printf("\n++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");

        // This cases for Live user response.
        if (cmd_mgr_msg.msg_buf_len == 2) {
          printk("\nCMD--->-----------------buffer length = 2-------------------\n");

          if (cmd_mgr_msg.msg_buf[1] == 1) {
            printk("\nConfirmed");
            strcpy(audio_data_for_ai.live_user_response, "Confirmed");
            printf("\nResult from audio_data_for_ai.live_user_response = %s\n", audio_data_for_ai.live_user_response);
          } else if (cmd_mgr_msg.msg_buf[1] == 2) {
            printk("\nRejected");
            strcpy(audio_data_for_ai.live_user_response, "Rejected");
            printf("\nResult from audio_data_for_ai.live_user_response = %s\n", audio_data_for_ai.live_user_response);
          } else if (cmd_mgr_msg.msg_buf[1] == 3) {
            printk("\nNot Sure");
            strcpy(audio_data_for_ai.live_user_response, "Not Sure");
            printf("\nResult from audio_data_for_ai.live_user_response = %s\n", audio_data_for_ai.live_user_response);
          }
        }
        printf("\n++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");

        cmd_type = 0;
        break;

      case ADO_RESET_REQUEST:
#ifdef OTA_BLOCKING
        if (!ota_updating) {
          stop_wifi_send = true;
          reset_calib_cmd = cmd_mgr_msg.msg_buf[0];
          ado_reset_calib(reset_calib_cmd);
        } else {
          // TODO: send response as ota updating.
          printk("OTA Updating..\n");
        }
#else
        stop_wifi_send = true;
        reset_calib_cmd = cmd_mgr_msg.msg_buf[0];
        ado_reset_calib(reset_calib_cmd);
#endif
        cmd_type = 0;
        break;

      case ADO_WIFI_CONTROL:
        printk("ADO_WiFi_CONTROL\r\n");
        // Record the configurable params as per the received command
        // Init the enum with received value to compare
        wifi_control_cmd = cmd_mgr_msg.msg_buf[0];

        switch (wifi_control_cmd) {
        case WIFI_MODULE_STATE:
          printk("\n--- WiFi module state command send by ED ---\n");
          break;

        case WIFI_SCAN:
          printk("\n--- WIFI_SCAN state command send by ED ---\n");
          break;

        case WIFI_SSID:
          printk("ADO_WiFi_SSID is \r\n");
          printk("msg_buf_len in command manager=%d\n", cmd_mgr_msg.msg_buf_len);

          for (int i = 1; i < cmd_mgr_msg.msg_buf_len + 1; i++) {
            printk("%d = %c\n", i, cmd_mgr_msg.msg_buf[i]);
          }

          memset(temp, 0, sizeof(temp));

          for (int i = 0; ((i < cmd_mgr_msg.msg_buf_len) && (cmd_mgr_msg.msg_buf[i + 1] != 0x03)); i++) {
            if ((cmd_mgr_msg.msg_buf[i + 1] > 32) && (cmd_mgr_msg.msg_buf[i + 1]) < 127) {
              temp[i] = cmd_mgr_msg.msg_buf[i + 1];
            }
          }

          printk("stored ssid =%s\n", temp);

          for (int i = 0; ((i < cmd_mgr_msg.msg_buf_len)); i++) {
            printk("temp[%d]=%c and 0x%x\n", i, temp[i], temp[i]);
          }

          temp[cmd_mgr_msg.msg_buf_len] = '\0';
          ssid_length = cmd_mgr_msg.msg_buf_len;
          // strcpy(temp,cmd_mgr_msg.msg_buf[1]);
          printk("temp_ssid is %s\n", temp);
          evt_msg[0] = 0; // STATUS = ssid received
          evt_msg_len = sizeof(evt_msg);
          ADO_notify_ble(ble_notify_thrd_id, ADO_WIFI_CONTROL_RESPONSE,
              WIFI_SSID, WIFI_SSID_RESP, evt_msg, evt_msg_len);
          ssid_flag = true;
          break;

        case WIFI_PASSWORD:
          printk("ADO_WiFi_PASSWORD is \r\n");
          printk("msg_buf_len in command manager=%d\n", cmd_mgr_msg.msg_buf_len);

          for (int i = 1; i < cmd_mgr_msg.msg_buf_len + 1; i++) {
            printk("%d = %c\n", i, cmd_mgr_msg.msg_buf[i]);
          }

          // int res=-1;

          for (int i = 0; i < cmd_mgr_msg.msg_buf_len; i++) {
            temp_pass[i] = cmd_mgr_msg.msg_buf[i + 1];
          }

          temp_pass[cmd_mgr_msg.msg_buf_len] = '\0';
          pass_length = cmd_mgr_msg.msg_buf_len;
          // strcpy(temp,cmd_mgr_msg.msg_buf[1]);
          printk("temp_pass is %s\n", temp_pass);

          // strcpy(pass,temp_pass);
          // printk("pass in cmd is %s\n",pass);

          evt_msg[0] = 0; // STATUS = password received
          evt_msg_len = sizeof(evt_msg);
          ADO_notify_ble(ble_notify_thrd_id, ADO_WIFI_CONTROL_RESPONSE,
              WIFI_PASSWORD, WIFI_PASS_RESP, evt_msg, evt_msg_len);
          pass_flag = true;
          break;

        case WIFI_CONNECT:
          if ((ssid_flag == true) && (pass_flag == true)) {
            ssid_flag = false;
            pass_flag = false;
            new_wifi = true;
            new_wifi_cred = true;
            stop_wifi_send = true;
            evt_msg[0] = 2; // STATUS = CONNECT RESPONSE ...
            evt_msg_len = sizeof(evt_msg);

            ADO_notify_ble(ble_notify_thrd_id, ADO_WIFI_CONTROL_RESPONSE,
                WIFI_CONNECT, WIFI_CONNECT_RESP, evt_msg, evt_msg_len);
            printk("connecting to wifi\n");
            evt_msg[0] = 3; // STATUS = CONNECTING...
            evt_msg_len = sizeof(evt_msg);
            ADO_notify_ble(ble_notify_thrd_id,
                ADO_WIFI_CONTROL_RESPONSE,
                WIFI_CONTROL_CONNECTION_STATUS_CONN,
                WIFI_CONTROL_CONNECTION_STATUS_RESP_CONN,
                evt_msg, evt_msg_len);

            printk("new wifi cred obtained....\n");
            WiFi_module_on_off(WIFI_MODULE_ON);
            WiFi_module_hard_reset();

            // wifi_connect_cmd(temp,ssid_length,temp_pass,pass_length,&stop_wifi_send);
            ado_wifi_status_to_wifithred(wifi_control_cmd, temp, ssid_length,
                temp_pass, pass_length);
            new_wifi = false;
          } else {
            printf("--------------------------------------------------------------------------\n");
            printf("send ssid and password\n");
            evt_msg[0] = 1; // STATUS = CONNECT RESPONSE but no proper ssid and password...
            evt_msg_len = sizeof(evt_msg);
            ADO_notify_ble(ble_notify_thrd_id, ADO_WIFI_CONTROL_RESPONSE, WIFI_CONNECT,
                WIFI_CONNECT_RESP, evt_msg, evt_msg_len);
            printf("--------------------------------------------------------------------------\n");
          }
          break;

        case WIFI_RECONNECT:
          printk("\n--WIFI Reconnect ssid_flag =%d, pass_flag =%d \n", ssid_flag, pass_flag);

          if ((ssid_flag == true) && (pass_flag == true)) {
            ssid_flag = false;
            pass_flag = false;

            new_wifi = true;
            new_wifi_cred = true;
            stop_wifi_send = true;

            evt_msg[0] = 0; // STATUS = Re_connection...
            evt_msg_len = sizeof(evt_msg);
            ADO_notify_ble(ble_notify_thrd_id, ADO_WIFI_CONTROL_RESPONSE, WIFI_RECONNECT,
                WIFI_RECONNECT_RESP, evt_msg, evt_msg_len);

            printk("Re-connecting to wifi\n");
            evt_msg[0] = 3; // STATUS = CONNECTING...
            evt_msg_len = sizeof(evt_msg);
            ADO_notify_ble(ble_notify_thrd_id, ADO_WIFI_CONTROL_RESPONSE,
                WIFI_CONTROL_CONNECTION_STATUS_CONN, WIFI_CONTROL_CONNECTION_STATUS_RESP_CONN,
                evt_msg, evt_msg_len);

            // printk("new wifi cred obtained....\n");
            WiFi_module_on_off(WIFI_MODULE_ON);
            WiFi_module_hard_reset();
            // wifi_connect_cmd(temp,ssid_length,temp_pass,pass_length,&stop_wifi_send);
            ado_wifi_status_to_wifithred(wifi_control_cmd, temp, ssid_length, temp_pass, pass_length);
            new_wifi = false;
          } else {
            printf("--------------------------------------------------------------------------\n");
            printf("send ssid and password\n");
            evt_msg[0] = 1; // STATUS = CONNECT RESPONSE but no proper ssid and password...
            evt_msg_len = sizeof(evt_msg);
            ADO_notify_ble(ble_notify_thrd_id, ADO_WIFI_CONTROL_RESPONSE, WIFI_CONNECT,
                WIFI_CONNECT_RESP, evt_msg, evt_msg_len);
            printf("--------------------------------------------------------------------------\n");
          }
          break;

        case WIFI_DISCONNECT:
          printk("\n--- WiFi disconnect state command send by ED ---\n");
          break;

        case WIFI_CONTROL_CONNECTION_STATUS_CONN:
          printk("\n--- WIFI_CONTROL_CONNECTION_STATUS_CONN state command send by ED ---\n");
          break;

        case WIFI_SIGNAL_STRENGTH:
          printk("------- WiFi signal strength from edge device ------------\n");
          ado_wifi_status_to_wifithred(wifi_control_cmd, "", 1, "", 1);
          break;

        case WIFI_CONNECTED_SSID:
          printk("\n--- requested ssid from edge devicecmd -- \n");
          ado_wifi_status_to_wifithred(wifi_control_cmd, temp, ssid_length, temp_pass, pass_length);
          break;
        }
        // TODO: added all wifi related controles.
        cmd_type = 0;
        break;

      case ADO_OTA_UPDATE:
        ota_update_cmd = cmd_mgr_msg.msg_buf[0];
        ado_ota_update_t(ota_update_cmd);
        cmd_type = 0;
        break;

      case DOORBOT_EM_LOCK:
        printk("\n==================>>>>>>>>>> DOORBOT_EM_LOCK %d   ", cmd_mgr_msg.msg_buf[1]);

        if (cmd_mgr_msg.msg_buf[1] == 1) {
          lock_msg = EM_LOCK_ENABLE;
          Lock_sendCmdACK(&lock_msg, 1);
          printk("\n==> DOORBOT_EM_LOCK ENABLED");
        } else if (cmd_mgr_msg.msg_buf[1] == 2) {
          lock_msg = EM_LOCK_DISABLE;
          Lock_sendCmdACK(&lock_msg, 1);
          printk("\n==> DOORBOT_EM_LOCK DISABLED");
        }
        break;
#if UWB
      case UWB_PARAMS:
        printk("UWB_PARAMS is \r\n");
        if (cmd_mgr_msg.msg_buf[1] == 0xA5) {
          Uwb_init_AND();
        } else if (cmd_mgr_msg.msg_buf[1] == 0x0A) {
          Uwb_init_IOs();
        } else if (cmd_mgr_msg.msg_buf[1] == 0x0C) {
          Uwb_stop();
        } else if (cmd_mgr_msg.msg_buf[1] == 0x0B) {
          if (cmd_mgr_msg.msg_buf[6] == 0x19) {
            Uwb_iphone_config(&cmd_mgr_msg.msg_buf[1]);
          } else {
            Uwb_phone_config(&cmd_mgr_msg.msg_buf[1]);
          }
          // uint8_t uwbtx_data[1]= {0x02};
          // ADO_notify_ble(ble_notify_thrd_id, UWB_PRAMS_RESPONSE, 1, 1, uwbtx_data, 1);
        }
        break;
#endif
      default:
        cmd_type = 0;
        break;
      }
    } else if (msg_type == RESPONSE) {
      // Get the destination for the response
      res_type = cmd_mgr_msg.msg_cmd;

      switch (res_type) {
      case BLE_NOTIFY:
        // prepare a mailbox message to be sent to ADO_BleNotify_Thread
        send_msg.info = 0;
        send_msg.size = sizeof(cmd_mgr_msg);
        send_msg.tx_data = &cmd_mgr_msg;
        send_msg.tx_target_thread = ble_notify_thrd_id;

        // send the mailbox message and wait till the other thread receives the message
        k_mbox_put(&ble_notify_mb, &send_msg, MAX_MAILBOX_MSG_SEND_TIMEOUT);
        res_type = 0U;
        break;

      default:
        res_type = 0U;
        break;
      }
    } else {
      // Invalid message type
      msg_type = 0;
    }
  }
}

void wifi_connect_cmd(char *temp, int ssid_length, char *temp_pass, int pass_length) {
  int err;
  uint8_t evt_msg[1] = {0};
  uint8_t evt_msg_len = 0;

  // WiFi_Network_Connect(NULL);
  printf("===================   cmd--> connecting from cmd ============================\n");
  stop_wifi_send = false;
  // added for operations working while wifi connecting
  err = WiFi_Network_Connect_cmd(temp, ssid_length, temp_pass, pass_length, &stop_wifi_send);
  printf("=======================   CMD--> err = %d ==============================\n", err);

  if ((err != 3)) { // if err=3 --> connected + err=4 --> connection failure err = -6 wifi stop due to some operations
    if ((new_wifi_cred == true) && (stop_wifi_send == true)) {
      printf("=============new wifi cred + stop wifi are true=====================\n");
      new_wifi_cred = false;
    } else {
      printf("cred stop while due to operations command  or connection failure to ed\n");
      evt_msg[0] = 5; // STATUS = connection failure
      evt_msg_len = sizeof(evt_msg);

      ADO_notify_ble(cmd_mgr_thrd_id,
          ADO_WIFI_CONTROL_RESPONSE,
          WIFI_CONTROL_CONNECTION_STATUS_CONN,
          WIFI_CONTROL_CONNECTION_STATUS_RESP_CONN,
          evt_msg, evt_msg_len);
      ssid_flag = true;
      pass_flag = true;
      printf("ssid_flag = %d and pass_flag = %d", ssid_flag, pass_flag);
    }
  }
}

void abort_all_thread() {
  char *thread_state = NULL;
  thread_state = k_thread_state_str(my_tid_install,0,0);
  printk("\n thread state is %s\n", thread_state);

  if (!is_thread_dead(thread_state)) {
    k_thread_abort(my_tid_install);
  }

  thread_state = k_thread_state_str(ado_calib_thrd_id,0,0);

  if (!is_thread_dead(thread_state)) {
    k_thread_abort(ado_calib_thrd_id);
  }

  thread_state = k_thread_state_str(ado_intent_thrd_id,0,0);

  if (!is_thread_dead(thread_state)) {
    k_thread_abort(ado_intent_thrd_id);
  }

  thread_state = k_thread_state_str(reset_calib_thrd_id,0,0);

  if (!is_thread_dead(thread_state)) {
    k_thread_abort(reset_calib_thrd_id);
  }
}

void timer_expire_intent(struct k_timer *timer) {
  char *thread_state = NULL;
  thread_state = k_thread_state_str(ado_intent_thrd_id,0,0);

  if (!is_thread_dead(thread_state)) {
    k_thread_abort(ado_intent_thrd_id);
  }
}

/*
 *  brief: This function called by edge device (or) intent detection thread
 *          to execute open/close operation of door.
 */
void ado_operation(uint8_t operate_cmd) {
  struct k_mbox_msg opr_mb_msg;
  char *thread_state = NULL;

  // debug print
  printk("ADO OPERATION COMMAND = %d \n", operate_cmd);

  // Check if the operation thread is already running
  if ((is_thread_dead((char *)k_thread_state_str(my_tid_install,0,0))) &&    // check installation thread
      (is_thread_dead((char *)k_thread_state_str(ado_calib_thrd_id,0,0)))) { // check calibration thread
    // Check if the operation thread is already running
    thread_state = k_thread_state_str(ado_opr_thrd_id,0,0);

    if (is_thread_dead(thread_state)) {
      // Create a new operations thread, it will keep running always
      // TODO: Suspend/resume this thread if not running from a long time,
      //        presently we made it exit after MAX_OPR_THREAD_IDLE_CYCLE_CNT * 50ms
      //        it will be created again when there is a new operation command.

      // Start the operation thread, only when it is not already running
      ado_opr_thrd_id = k_thread_create(&opr_thread_data, opr_thread_stack,
          K_THREAD_STACK_SIZEOF(opr_thread_stack),
          ADO_Operations_Thread,
          NULL, NULL, NULL,
          6, 0, K_NO_WAIT);
    }

    // prepare a mailbox message to be sent to ado operation thread
    opr_mb_msg.info = 0;
    opr_mb_msg.size = sizeof(operate_cmd);
    opr_mb_msg.tx_data = &operate_cmd;
    opr_mb_msg.tx_target_thread = ado_opr_thrd_id;

    // send the mailbox message and wait till the other thread receives the message
    k_mbox_put(&cmd_mgr_mb, &opr_mb_msg, MAX_MAILBOX_MSG_SEND_TIMEOUT);
  }
}

/*
 *  brief: This function is to be called when there is any calibration command received
 *         by ADO
 */
void ado_calibration(uint8_t *calib_data) {
  struct k_mbox_msg calib_mb_msg;
  char *thread_state = NULL;
  ado_calib_cmd_t calib_cmd = calib_data[0];

  // debug print
  printk("ADO CALIBRATION COMMAND = %d \n", calib_cmd);

  if (calib_cmd == CAL_START) {
    // Check if the ADO is already installed on the door otherwise return from this function
    if (is_ADO_Installed(&clamp_data) == false) {
      // TODO: Add the reporting to the ED that the ADO needs to be installed first, before calibrating.
      // Notify BLE that ADO_NOT_INSTALLED (To be added in Enum), modify the following line and uncomment it.
      // ADO_notify_ble(ble_notify_thrd_id, ADO_CALIBRATION_RESPONSE, calib_cmd, CAL_NOT_STARTED, NULL, 0);
      printk("ADO not installed\n\r");
      return; // important here, as the calibration thread should not even start if the device is not installed.
    }

    // Check if the calibration thread is NOT already running, change it to thread status query methods later
    thread_state = k_thread_state_str(ado_calib_thrd_id,0,0);

    // Debug print
    printk("Calib thread state: %s\n", thread_state);

    if (is_thread_dead(thread_state)) {
      // Start the calibration thread, only when it is not already running
      /* checking if operation thread is already running, if it is,
         then abort it to solve the sleep mode during calibration problem */
      char *thread_state_op = NULL;
      thread_state_op = k_thread_state_str(ado_opr_thrd_id,0,0);

      if (!is_thread_dead(thread_state_op)) {
        k_thread_abort(ado_opr_thrd_id);
        execution_time_close = 3;
        flag_close = 0;
      }

      ado_calib_thrd_id = k_thread_create(&cal_thread_data, cal_thread_stack,
          K_THREAD_STACK_SIZEOF(cal_thread_stack),
          InitCalibrationProcess,
          NULL, NULL, NULL,
          MY_PRIORITY, 0, K_NO_WAIT);
    }

    // prepare a mailbox message to be sent to ado calibration thread
    calib_mb_msg.info = 0;
    // Send 10 bytes only since it is sufficient for all the calib commands
    calib_mb_msg.size = sizeof(uint8_t) * 10;
    calib_mb_msg.tx_data = calib_data;
    calib_mb_msg.tx_target_thread = ado_calib_thrd_id;

    // send the mailbox message and wait till the other thread receives the message
    k_mbox_put(&cmd_mgr_mb, &calib_mb_msg, MAX_MAILBOX_MSG_SEND_TIMEOUT);
  } else {
    // Check if the calibration thread is NOT already running, change it to thread status query methods later
    thread_state = k_thread_state_str(ado_calib_thrd_id,0,0);

    if (is_thread_dead(thread_state)) {
      // Notify BLE that CAL_NOT_STARTED
      ADO_notify_ble(ble_notify_thrd_id, ADO_CALIBRATION_RESPONSE, calib_cmd, CAL_NOT_STARTED, NULL, 0);

      // debug print
      printk("CAL_NOT_Started\n");
    } else {
      // prepare a mailbox message to be sent to ado calibration thread
      calib_mb_msg.info = 0;
      // Send 10 bytes only since it is sufficient for all the calib commands
      calib_mb_msg.size = sizeof(uint8_t) * 10;
      calib_mb_msg.tx_data = calib_data;
      calib_mb_msg.tx_target_thread = ado_calib_thrd_id;

      // send the mailbox message and wait till the other thread receives the message
      k_mbox_put(&cmd_mgr_mb, &calib_mb_msg, MAX_MAILBOX_MSG_SEND_TIMEOUT);
    }
  }
}

/*
 *  brief: This function is to be called when there is any ADO
    installation command received by ADO.
 */
void ado_installation(uint8_t installation_cmd) {
  struct k_mbox_msg install_mb_msg;
  char *thread_state = NULL;

  // debug print
  printk("ADO INSTALLATION COMMAND = %d \n", installation_cmd);

  // Check if nothing is running currently
  thread_state = k_thread_state_str(my_tid_install,0,0);
  if (is_thread_dead(thread_state)) {
    if (installation_cmd == INS_START) {
      // TODO: Load the settings from the EEPROM and set the required variables

      // Create thread for installation ADO with respect to installation command.
      my_tid_install = k_thread_create(&my_thread_data, my_stack_area,
          K_THREAD_STACK_SIZEOF(my_stack_area),
          ado_install_uninstall,
          NULL, NULL, NULL,
          MY_PRIORITY, 0, K_NO_WAIT);

      // prepare a mailbox message to be sent to ado calibration thread
      install_mb_msg.info = 0;
      install_mb_msg.size = sizeof(installation_cmd);
      install_mb_msg.tx_data = &installation_cmd;
      install_mb_msg.tx_target_thread = my_tid_install;

      // send the mailbox message and wait till the other thread receives the message
      // Changed since the installation logic needs delay more than MAX_MAILBOX_MSG_SEND_TIMEOUT
      k_mbox_put(&cmd_mgr_mb, &install_mb_msg, K_MSEC(100));
    } else {
      // Notify over BLE that installation NOT started
      ADO_notify_ble(ble_notify_thrd_id, ADO_INSTALLATION_RESPONSE,
          installation_cmd, INSTALLATION_NOT_STARTED, NULL, 0);

      // Debug prints
      printk("Installation NOT started, start it first\n");
    }
  } else {
    // prepare a mailbox message to be sent to ado calibration thread
    install_mb_msg.info = 0;
    install_mb_msg.size = sizeof(installation_cmd);
    install_mb_msg.tx_data = &installation_cmd;
    install_mb_msg.tx_target_thread = my_tid_install;

    // send the mailbox message and wait till the other thread receives the message
    // Changed since the installation logic needs delay more than MAX_MAILBOX_MSG_SEND_TIMEOUT
    k_mbox_put(&cmd_mgr_mb, &install_mb_msg, K_MSEC(100));
  }
}

/*
 *  brief: This function is to be called when the ADO recieve any intent control command received
 *         like INT_START or INT_STOP etc.
 */
void ado_intent(uint8_t intent_detection_cmd) {
  struct k_mbox_msg intent_mb_msg;
  char *thread_state = NULL;

  // Check if the intent detection thread is already running
  thread_state = k_thread_state_str(ado_intent_thrd_id,0,0);

  if (is_thread_dead(thread_state)) {
    // Create a new operations thread, it will keep running always
    // TODO: Suspend/resume this thread if not running from a long time,
    //        presently we made it exit after MAX_OPR_THREAD_IDLE_CYCLE_CNT * 50ms
    //        it will be created again when there is a new intent command.

    // Start the intent thread, only when it is not already running
    printk("Intent detection thread started\n");
    ado_intent_thrd_id = k_thread_create(&intent_thread_data, intent_thread_stack,
        K_THREAD_STACK_SIZEOF(intent_thread_stack),
        ADO_intent_detect_thread,
        NULL, NULL, NULL,
        5, 0, K_NO_WAIT);
  } else {
    k_timer_start(&intent_timer, K_MSEC(20000), K_MSEC(0));
  }

  // prepare a mailbox message to be sent to ado calibration thread
  intent_mb_msg.info = 0;
  intent_mb_msg.size = sizeof(intent_detection_cmd);
  intent_mb_msg.tx_data = &intent_detection_cmd;
  intent_mb_msg.tx_target_thread = ado_intent_thrd_id;

  // send the mailbox message and wait till the other thread receives the message
  k_mbox_put(&cmd_mgr_mb, &intent_mb_msg, MAX_MAILBOX_MSG_SEND_TIMEOUT);
}

/*
 *  brief: This function is to be called when the ADO recieve any intent control command received
 *         like INT_START or INT_STOP etc.
 */
void ado_dev_status(uint8_t dev_status_cmd) {
  struct k_mbox_msg devStat_mb_msg;
  char *thread_state = NULL;
  int err = 0;

  /*  TODO: Since the Device status thread is an ESSENTIAL thread
            and is supposed to run always, but even in case it gets
            terminated somehow, check its status and start it again.
  */
  // prepare a mailbox message to be sent to ado device status thread
  devStat_mb_msg.info = 0;
  devStat_mb_msg.size = sizeof(dev_status_cmd);
  devStat_mb_msg.tx_data = &dev_status_cmd;
  devStat_mb_msg.tx_target_thread = dev_stat_thrd_id;

  // send the mailbox message and wait till the other thread receives the message
  err = k_mbox_put(&cmd_mgr_mb, &devStat_mb_msg, MAX_MAILBOX_MSG_SEND_TIMEOUT);

  if (err == 0) {
    printk("\n ado_dev_status msg sent successfully");
  } else if (ENOMSG == err) {
    printk("\n ado_dev_status Returned without waiting");
  } else if (EAGAIN == err) {
    printk("\nado_dev_status Waiting period timed out");
  } else {
    printk("\nun known  error send cmdmngr to ado_dev_status %d", err);
    err = k_mbox_put(&cmd_mgr_mb, &devStat_mb_msg, MAX_MAILBOX_MSG_SEND_TIMEOUT);

    if (err == 0) {
      printk("\n cmdmngr to dev status retry msg sent successfully");
    } else {
      printk("\n cmdmngr to dev status retry msg sent failed");
    }
  }
}

void ado_wifi_status_to_wifithred(uint8_t wifi_status_cmd, uint8_t *ssid,
    uint8_t ssidlen, uint8_t *passw, uint8_t passwlen) {
  wifi_data senddata = {
      0,
  };

  struct k_mbox_msg wifiStat_mb_msg;
  char *thread_state = NULL;
  int err = 0;
  /*  TODO: Since the Device status thread is an ESSENTIAL thread
            and is supposed to run always, but even in case it gets
            terminated somehow, check its status and start it again.
  */
  // prepare a mailbox message to be sent to ado device status thread
  senddata.cmd_type = wifi_status_cmd;
  senddata.ssidlen = ssidlen;
  memcpy(senddata.ssid, ssid, senddata.ssidlen);
  senddata.passlen = passwlen;
  memcpy(senddata.pass, passw, senddata.passlen);

  wifiStat_mb_msg.info = 0;
  wifiStat_mb_msg.size = sizeof(senddata);
  wifiStat_mb_msg.tx_data = &senddata;
  wifiStat_mb_msg.tx_target_thread = wifi_notify_thrd_id;

  // send the mailbox message and wait till the other thread receives the message
  err = k_mbox_put(&cmd_mgr_mb, &wifiStat_mb_msg, K_MSEC(100));

  if (err == 0) {
    printk("\n ado_wifi_status msg sent successfully");
  } else if (ENOMSG == err) {
    printk("\n ado_wifi_status Returned without waiting");
  } else if (EAGAIN == err) {
    printk("\n ado_wifi_status Waiting period timed out");
  } else {
    int numb_msgs;
    numb_msgs = k_msgq_num_used_get(&cmd_mgr_mb);
    printk("\nun known  error ado_wifi_status %d number of msgs in q %d", err, numb_msgs);
  }
}

/*
 *  brief:
 */
void ado_reset_calib(uint8_t reset_calib_cmd) {
  struct k_mbox_msg res_calib_mb_msg;
  char *thread_state = NULL;
  ado_reset_calib_cmd_t res_calib_cmd = reset_calib_cmd;

  // Debug print
  // printk("res_calib_cmd1 : %d\r\n", res_calib_cmd);

  if (res_calib_cmd == RESET_START) {
    // Check the preconditions of doorbot being installed and calibrated or not
    // 1. Check if the doorbot is installed and calibrated
    if (is_doorbot_uninstalled_uncalibrated() == true) {
      // debug print
      printk("doorbot is uninstalled or uncalibrated\n");
      return;
      // Important! as we must not allow starting the RESET_CALIB sequence if doorbot is not installed or calibrated
    }

    // Check if the calibration thread is NOT already running, change it to thread status query methods later
    thread_state = k_thread_state_str(reset_calib_thrd_id,0,0);

    // Debug print
    printk("Reset Calib thread state: %s\n", thread_state);

    if (is_thread_dead(thread_state)) {
      // Start the calibration thread, only when it is not already running
      reset_calib_thrd_id = k_thread_create(&reset_calib_thrd_data, reset_calib_thrd_stack,
          K_THREAD_STACK_SIZEOF(reset_calib_thrd_stack),
          ADO_Reset_Calib_Thread,
          NULL, NULL, NULL,
          RESET_CALIB_THRD_PRIORITY, 0, K_NO_WAIT);
    }
  }

  // prepare a mailbox message to be sent to ado calibration thread
  res_calib_mb_msg.info = 0;
  res_calib_mb_msg.size = sizeof(uint8_t);
  res_calib_mb_msg.tx_data = &reset_calib_cmd;
  res_calib_mb_msg.tx_target_thread = reset_calib_thrd_id;

  // send the mailbox message and wait till the other thread receives the message
  k_mbox_put(&cmd_mgr_mb, &res_calib_mb_msg, MAX_MAILBOX_MSG_SEND_TIMEOUT);
}

void ado_ota_update_t(uint8_t ota_update_cmd) {
  printk("ADO OTA COMMAND = %d \n", ota_update_cmd);

  switch (ota_update_cmd) {
  case CHECK_UPDATE:;
    uint8_t evt_msg_cur[1] = {0};
    ADO_notify_ble(ble_notify_thrd_id, ADO_OTA_UPDATE_RESPONSE, ota_update_cmd,
        CHECK_UPDATE_RESP, evt_msg_cur, 1);
    printk("CHECK_UPDATE\n");
    // TODO: added for fast testing by samuel
    k_msleep(100U);

    if (OTA_update_pending) { // this condition saves time when check update commands comes in middle of OTA update
      printf("New STM firmware update available......\n");
      uint8_t evt_msg[1] = {2};
      uint8_t evt_msg_len = sizeof(evt_msg);

      ADO_notify_ble(ble_notify_thrd_id, ADO_OTA_UPDATE_RESPONSE, CHECK_UPDATE,
          CHECK_UPDATE_RESP, evt_msg, evt_msg_len);
      k_msleep(500U);

      uint8_t stm_old_new_ver_numbers[6] = {0};
      stm_old_new_fw_version_to_edge_device(stm_old_new_ver_numbers);
      /* ADO_notify_ble(cmd_mgr_thrd_id, ADO_OTA_UPDATE_RESPONSE, STM_UPDATE_VERSION,
          STM_UPDATE_VERSION_RESP, stm_old_new_ver_numbers, 6); */
      ADO_notify_ble(ble_notify_thrd_id, ADO_OTA_UPDATE_RESPONSE, UPDATE_INFO,
          UPDATE_INFO_RESP, stm_old_new_ver_numbers, 6);
    } else {
      stop_wifi_send = true;
      check_for_ota_update = true;
    }
    break;

  case UPDATE_TARGET_TYPE:;
    uint8_t evt_msg_utt[1] = {1};
    ADO_notify_ble(ble_notify_thrd_id, ADO_OTA_UPDATE_RESPONSE, ota_update_cmd,
        UPDATE_TARGET_TYPE_RESP, evt_msg_utt, 1);
    printk("UPDATE_TARGET_TYPE\n");
    break;

  case UPDATE_INFO:;
    ota_info=true; 
    printk("UPDATE_INFO\n");
    break;

  case START_UPDATE:
    ADO_notify_ble(ble_notify_thrd_id, ADO_OTA_UPDATE_RESPONSE,
        ota_update_cmd, START_UPDATE_RESP, NULL, 0);
    printk("START_UPDATE\n");
    break;

  case CANCEL_UPDATE:
    ADO_notify_ble(ble_notify_thrd_id, ADO_OTA_UPDATE_RESPONSE,
        ota_update_cmd, CANCEL_UPDATE_RESP, NULL, 0);
    printk("CANCEL_UPDATE\n");
    break;

  case UPDATE_PROGRESS:;
    // uint8_t evt_msg_upr[1] = {5};
    /* ADO_notify_ble(ble_notify_thrd_id, ADO_OTA_UPDATE_RESPONSE,
        ota_update_cmd, UPDATE_PROGRESS_RESP, evt_msg_upr, 1); */
    printk("UPDATE_PROGRESS\n");
    break;

  case STM_UPDATE_VERSION:;
    uint8_t evt_msg[6] = {1, 2, 3, 1, 2, 4};
    uint8_t evt_msg_len = 6;
    /* ADO_notify_ble(ble_notify_thrd_id, ADO_OTA_UPDATE_RESPONSE,
        ota_update_cmd, STM_UPDATE_VERSION_RESP, evt_msg, evt_msg_len); */
    printk("STM_UPDATE_VERSION\n");
    break;

  case NORDIC_UPDATE_VERSION:;
    uint8_t evt_msg_n[6] = {2, 2, 3, 2, 2, 4};
    //uint8_t evt_msg_len_n = 6;
    //ADO_notify_ble(ble_notify_thrd_id, ADO_OTA_UPDATE_RESPONSE,
    //    ota_update_cmd, NORDIC_UPDATE_VERSION_RESP, evt_msg_n, evt_msg_len_n);
    printk("NORDIC_UPDATE_VERSION\n");
    break;

  case WIFI_MOD_OTA_START:;
    // uint8_t evt_msg_a[5] = {0, 192, 168, 3, 22};
    // uint8_t evt_msg_len_a = 5;
    /* ADO_notify_ble(ble_notify_thrd_id, ADO_OTA_UPDATE_RESPONSE
         ota_update_cmd, WIFI_MOD_OTA_START_RESP, evt_msg_a, evt_msg_len_a); */
    printk("WIFI_MOD_OTA_START\n");
    esp32_OTA_start = true;
    break;

  case WIFI_MOD_OTA_STOP:;
    // uint8_t evt_msg_b[1] = {0};
    // uint8_t evt_msg_len_b = 1;
    /* ADO_notify_ble(ble_notify_thrd_id, ADO_OTA_UPDATE_RESPONSE,
        ota_update_cmd, WIFI_MOD_OTA_STOP_RESP, evt_msg_b, evt_msg_len_b); */
    printk("WIFI_MOD_OTA_STOP\n");
    esp32_OTA_start = false;
    break;

  case WIFI_MOD_OTA_STATUS:;
    // uint8_t evt_msg_c[6] = {4, 192, 168, 3, 22, 20};
    // uint8_t evt_msg_len_c = 6;
    /* ADO_notify_ble(ble_notify_thrd_id, ADO_OTA_UPDATE_RESPONSE,
        ota_update_cmd, WIFI_MOD_OTA_STATUS_RESP, evt_msg_c, evt_msg_len_c); */
    printk("WIFI_MOD_OTA_STATUS\n");
    break;

  default:
    break;
  }
}

/*
 *  brief: This function is to be called when we need to notify the command manager
 *         for executing the passed command.
 */
void ADO_notify_cmd_mgr(uint8_t msg_type, uint8_t cmd_type,
    uint8_t cmd, uint8_t *cmd_msg, uint8_t cmd_msg_len) {
  // Mailbox variables
  struct k_mbox_msg send_msg;
  ado_cmd_mgr_msg_t cmd_mgr_msg;

  // prepare the mailbox message
  cmd_mgr_msg.msg_type = (ado_cmd_msg_type_t)msg_type;
  cmd_mgr_msg.msg_cmd = (ado_cmd_type_t)cmd_type;
  cmd_mgr_msg.msg_buf[0] = cmd;
  cmd_mgr_msg.msg_buf_len = 1U;

  // check if we have any extra bytes to be sent, if yes frame them
  if ((cmd_msg_len > 0) && (cmd_msg != NULL)) {
    memcpy(&cmd_mgr_msg.msg_buf[1], cmd_msg, cmd_msg_len);
    cmd_mgr_msg.msg_buf_len = cmd_msg_len + 1U; // 1 byte for cmd
  }

  // Now prepare the message to be sent to cmd_mgr
  send_msg.info = 0;
  send_msg.size = sizeof(cmd_mgr_msg);
  send_msg.tx_data = &cmd_mgr_msg;
  send_msg.tx_target_thread = cmd_mgr_thrd_id;

  // send the mailbox message and wait till the other thread receives the message
  k_mbox_put(&cmd_mgr_mb, &send_msg, MAX_MAILBOX_MSG_SEND_TIMEOUT);
}

// test function
void ADO_notify_short(k_tid_t dest_thrd_id, float *msg, uint8_t len) {
  // Mailbox variables
  struct k_mbox_msg send_msg;

  // Now prepare the message to be sent to cmd_mgr
  send_msg.info = 0;
  send_msg.size = len;
  send_msg.tx_data = msg;
  send_msg.tx_target_thread = dest_thrd_id;

  // send the mailbox message and wait till the other thread receives the message
  k_mbox_put(&cmd_mgr_mb, &send_msg, MAX_MAILBOX_MSG_SEND_TIMEOUT);
}

/*
 *  brief: This function reads the respective global configuration
    variables and writes them to EEPROM
 */
void save_config_data() {
  int error = 0;
  uint8_t data[2];

  data[0] = config_pwm_percent;
  data[1] = intent.isIntentEnabled;

  error = Write_EEPROM(CONFIGURATION_MEMORY_LOCATION, &data[0], 2);

  if (error) {
    printk("error saving config data to EEPROM trying one more time");
    error = Write_EEPROM(CONFIGURATION_MEMORY_LOCATION, &data[0], 2);
  }
}

/*
 *  brief: This function reads the stored configuration variables
    from EEPROM to respective global variables
 */
void load_config_data() {
  int error = 0;
  uint8_t data[2];

  error = Read_EEPROM(CONFIGURATION_MEMORY_LOCATION, &data[0], 2);

  if (error) {
    printk("error loading config data from EEPROM trying one more time");
    error = Read_EEPROM(CONFIGURATION_MEMORY_LOCATION, &data[0], 2);
  }

  config_pwm_percent = data[0];
  Gear_motor_params.end_RPM = speedmap(config_pwm_percent, 30, 100, 40,
      Gear_motor_params.fw_speed_cap);
  /* Gear_motor_params.bw_end_RPM = speedmap(config_pwm_percent,30,100,40
      Gear_motor_params.bw_speed_cap);  */
  intent.isIntentEnabled = data[1];

  // debug print
  printk("Read PWM_percent: %d, intent_state: %d to EEPROM\r\n",
      config_pwm_percent, intent.isIntentEnabled);
}

void save_auto_close_data() {
  int error = 0;
  uint8_t data[2] = {0};
  data[0] = auto_close_timer;
  error = Write_EEPROM(AUTO_CLOSE_TIME_LOCATION, &data[0], 1);

  if (error) {
    printk("\n error saving AUTO CLOSE TIMER data to EEPROM \n");
  } else {
    printk("\n Saved AUTO CLOSE TIMER data to EEPROM \n");
  }
}

/*
 *  brief: This function reads the stored configuration variables
    from EEPROM to respective global variables
 */
void load_auto_close_data() {
  int error = 0;
  uint8_t data[2] = {0};
  uint8_t read_data = 0;

  error = Read_EEPROM(AUTO_CLOSE_TIME_LOCATION, &data[0], 1);

  if (error) {
    printk("error loading config data from EEPROM trying one more time");
  }

  read_data = data[0];
  auto_close_timer = data[0];
  // debug print
  printk("Read read_data: %d, auto_close_timer: %d from EEPROM\r\n",
      read_data / 2, auto_close_timer / 2);
}

/*
 *  brief: This function reads the respective global configuration
    variables and reports over BLE to ED
 */
void ado_get_devConf() {
  // uint8_t evt_msg[2] = {0};
  uint8_t evt_msg[3] = {0};
  // config_pwm_percent = (((Gear_motor_params.end_RPM-40)*(100-30))/(Gear_motor_params.fw_speed_cap-40))+30;
  evt_msg[0] = config_pwm_percent;
  evt_msg[1] = intent.isIntentEnabled;
  evt_msg[2] = auto_close_timer / 2;

  // Notify Over BLE
  ADO_notify_ble(ble_notify_thrd_id, ADO_CONFIG_RESPONSE,
      REQ_ADO_CONFIG, RESP_ADO_CONFIG, evt_msg, 3);
}

bool is_thread_dead(char *ptr) {
  if ((strcmp(ptr, "dead") == 0) || (strcmp(ptr, "unknown") == 0) || (strcmp(ptr, "\0") == 0)) {
    return true;
  }
  return false;
}

void ado_toggle_display(bool display_cmd) {
  if (display_cmd) {
    printk(" display on");
    // Toggle the display state
    // printk("\n==========display_activity = %d=========\n",display_activity);

    if (display_activity == false) {
      // printk("\n==========display is off and making it is on =========\n",display_activity);
      display_power_control(DIS_TURN_ON);
      display_activity = true;
    } else {
      // printk("\n============display is already on =============\n");
    }
  } else {
    printk(" display off");
    // Toggle the display state
    display_activity = false;
    display_power_control(DIS_TURN_OFF);
  }
}

/*
 *  brief: This function enables or disables the ADO display as per the received boolean argument
 */
void ado_toggle_motor(bool motor_cmd) {
  // if(motor_cmd)
  //{
  //   printk(" motor on");
  //   motor_driver_power_relay_control(MOTOR_RELAY_TURN_ON);  // active low GPIO
  //   motor_activity = true;
  // }
  // else
  //{
  //   printk(" motor off");
  //   motor_driver_power_relay_control(MOTOR_RELAY_TURN_OFF);  // active low GPIO
  //   motor_activity = false;
  // }
}

/*
 *  brief: This function enables or disables the ADO display as per the received boolean argument
 */
void ado_toggle_wifi(bool wifi_cmd) {
  if (wifi_cmd) {
    printk(" wifi on");
    wifi_activity = true;
  } else {
    printk(" wifi off");
    wifi_activity = false;
  }
}

/*
 *  brief: This function enables or disables the ADO display as per the received boolean argument
 */
void ado_toggle_stm32(bool stm32_cmd) {
  // if(stm32_cmd)
  //{
  //   printk(" STM32 on");
  //   stm_activity = true;
  //   STM32_sleep_control(STM32_RUN_MODE);
  // }
  // else
  //{
  //   printk(" STM32 off");
  //   stm_activity = false;
  //   STM32_sleep_control(STM32_SLEEP_MODE);
  // }
}

void power_optimization(ado_power_modes mode) {
  printk(" prev %d ", current_power_mode);
  printk(" current %d ", mode);

  switch (mode) {
  case CONNECTED:
    current_power_mode = mode;
    k_timer_start(&timer, K_MSEC(25000), K_MSEC(0));
    break;

  case DISCONNECTED:
    if (current_power_mode != OPERATION && current_power_mode != OTA) {
      current_power_mode = mode;
      k_timer_start(&timer, K_MSEC(0), K_MSEC(0));
    }
    break;

  case INSTALLATION:
    current_power_mode = mode;
    k_timer_start(&timer, K_MSEC(0), K_MSEC(0));
    break;

  case INSTALLATION_STOP:
    current_power_mode = mode;
    k_timer_start(&timer, K_MSEC(10000), K_MSEC(0));
    break;

  case CALIBRATION:
    current_power_mode = mode;
    k_timer_start(&timer, K_MSEC(0), K_MSEC(0));
    break;

  case CALIBRATION_STOP:
    current_power_mode = mode;
    k_timer_start(&timer, K_MSEC(10000), K_MSEC(0));
    break;

  case OPERATION:
    current_power_mode = mode;
    k_timer_start(&timer, K_MSEC(0), K_MSEC(0));
    break;

  case OPERATION_STOP:
    if (current_power_mode != OTA && current_power_mode != CALIBRATION &&
        current_power_mode != INSTALLATION) {
      current_power_mode = mode;
      k_timer_start(&timer, K_MSEC(10000), K_MSEC(0));
    }
    break;

  case PIR:
    if (current_power_mode != OPERATION && current_power_mode != CALIBRATION &&
        current_power_mode != INSTALLATION) {
      current_power_mode = mode;
      k_timer_start(&timer, K_MSEC(0), K_MSEC(0));
      k_sleep(K_MSEC(1));
      current_power_mode = PIR_STOP;
      k_timer_start(&timer, K_MSEC(20000), K_MSEC(0));
    }
    break;

  case OTA:
    current_power_mode = mode;
    k_timer_start(&timer, K_MSEC(0), K_MSEC(0));
    break;

  case OTA_STOP:
    current_power_mode = mode;
    k_timer_start(&timer, K_MSEC(5000), K_MSEC(0));
    break;

  case DUMPING:
    current_power_mode = mode;
    k_timer_start(&timer, K_MSEC(0), K_MSEC(0));
    break;

  case DUMPING_STOP:
    current_power_mode = mode;
    k_timer_start(&timer, K_MSEC(5000), K_MSEC(0));
    break;
  }
}

void timer_expire(struct k_timer *timer) {
  printk("%d", current_power_mode);

  switch (current_power_mode) {
  case CONNECTED:
    ado_toggle_display(false);
    ado_toggle_stm32(false);
    ado_toggle_wifi(false);
    ado_toggle_motor(false);
    break;

  case DISCONNECTED:
    ado_toggle_display(true);
    ado_toggle_stm32(false);
    ado_toggle_wifi(false);
    ado_toggle_motor(false);
    break;

  case INSTALLATION:
    ado_toggle_display(true);
    ado_toggle_stm32(false);
    ado_toggle_wifi(false);
    ado_toggle_motor(true);
    break;

  case INSTALLATION_STOP:
    ado_toggle_display(false);
    ado_toggle_stm32(false);
    ado_toggle_wifi(false);
    ado_toggle_motor(false);
    break;

  case CALIBRATION:
    ado_toggle_display(true);
    ado_toggle_stm32(false);
    ado_toggle_wifi(false);
    ado_toggle_motor(true);
    break;

  case CALIBRATION_STOP:
    ado_toggle_display(false);
    ado_toggle_stm32(false);
    ado_toggle_wifi(false);
    ado_toggle_motor(false);
    break;

  case OPERATION:
    ado_toggle_display(true);
    ado_toggle_stm32(true);
    ado_toggle_wifi(false);
    ado_toggle_motor(true);
    break;

  case OPERATION_STOP:
    ado_toggle_display(false);
    ado_toggle_stm32(false);
    ado_toggle_wifi(false);
    ado_toggle_motor(false);
    run_in_intent_mode = false;
    enable_pir_1();
    break;

  case PIR:
    ado_toggle_display(true);
    ado_toggle_stm32(true);
    ado_toggle_wifi(false);
    ado_toggle_motor(false);
    break;

  case PIR_STOP:
    if (battery_low == true) {
      ado_toggle_display(true);
    } else {
      ado_toggle_display(false);
    }

    ado_toggle_stm32(false);
    ado_toggle_wifi(false);
    ado_toggle_motor(false);
    run_in_intent_mode = false;
    enable_pir_1();
    break;

  case OTA:
    ado_toggle_display(true);
    ado_toggle_stm32(true);
    ado_toggle_wifi(true);
    ado_toggle_motor(false);
    // run_in_intent_mode = false;
    disable_pir_1();
    break;

  case OTA_STOP:
    ado_toggle_display(false);
    ado_toggle_stm32(false);
    ado_toggle_wifi(false);
    ado_toggle_motor(false);
    run_in_intent_mode = false;
    enable_pir_1();
    break;

  case DUMPING:
    ado_toggle_display(true);
    ado_toggle_stm32(true);
    ado_toggle_wifi(true);
    ado_toggle_motor(false);
    disable_pir_1();
    break;

  case DUMPING_STOP:
    ado_toggle_display(false);
    ado_toggle_stm32(false);
    ado_toggle_wifi(false);
    ado_toggle_motor(false);
    run_in_intent_mode = false;
    enable_pir_1();
    break;
  }
  k_timer_stop(timer); // call timer_stop
}

bool WIFI_ACTIVE() {
  return wifi_activity;
}

bool Motor_active() {
  return motor_activity;
}

bool STM_active() {
  return stm_activity;
}

bool Display_active() {
  return display_activity;
}

int speedmap(long x, long in_min, long in_max, long out_min, long out_max) {
  return (((x - in_min) * (out_max - out_min)) / (in_max - in_min)) + out_min;
}