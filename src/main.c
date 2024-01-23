// System Includes
#include <dk_buttons_and_leds.h>
#include <zephyr/drivers/gpio.h>
#include <errno.h>
#include <hal/nrf_gpio.h>
#include <zephyr/kernel.h>
#include <soc.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/types.h>

// Custom Includes
#include "Includes/ADO_Audio_files.h"
#include "Includes/ADO_Flash_Module.h"
#include "Includes/ADO_ICM20948_Module.h"
#include "Includes/ADO_Operations_Module.h"
#include "Includes/ADO_PIR_Module.h"
#include "Includes/ADO_Reset_Calib_Module.h"
#include "Includes/ADO_STM32H7_WW.h"
#include "Includes/ADO_Ultrasonic_Sensors.h"
#include "Includes/ADO_WiFi_Module.h"
#include "Includes/AnalogIn.h"
#include "Includes/BLE_Module.h"
#include "Includes/BNO080.h"
#include "Includes/CH101_USS.h"
#include "Includes/Calibration_Module.h"
#include "Includes/Command_Manager_Module.h"
#include "Includes/Device_Status_Module.h"
#include "Includes/Display_Module.h"
#include "Includes/EEPROM_Module.h"
#include "Includes/GPIO_Expander.h"
#include "Includes/I2C_Module.h"
#include "Includes/IMU_Module.h"
#include "Includes/Install_ADO.h"
#include "Includes/Intent_Module.h"
#include "Includes/Motor_Control_Module.h"
#include "Includes/SPI_Module.h"
#include "Includes/UART_Module.h"
#include "Includes/si5351.h"

#define SLEEP_INTERVAL_MS 500U
#define OTA_UPDATE_PAUSE -6

extern int auto_close_timer;
extern bool is_update_to_auto_ai;

#ifdef CONFIG_USS_I2C1_ENABLE // For Ultasonic Sensor Enable
extern chirp_data_t chirp_data[CHIRP_MAX_NUM_SENSORS];
#define USS_OPEN_RANGE 2000 // Auto Open using USS
#endif

#define SEND_DOOR_OPEN_CMD 0U
//#define SEND_DOOR_CLOSE_CMD   50U
//#define INT_DOOR_OPERATIONS auto_close_timer + 20 // 250U  //change 2 min to auto_close_timer + 5 sec
//#define INT_DOOR_OPERATIONS auto_close_timer + 1 // 250U  //change 2 min to auto_close_timer + 5 sec
#define SEND_DOOR_CLOSE_CMD auto_close_timer
//#define SEND_DOOR_CLOSE_CMD_2_MIN 120    // 240 // Each count 500ms
#define SEND_DOOR_CLOSE_CMD_2_MIN 20     // 20 // Each count 500ms
#define ADO_SEND_DOOR_CLOSE_EMERGENCY 60 // 240 // Each count 500ms

void initializing_all_modules(void);
int8_t DO_OTA_Update();
int8_t Set_door_to_open_position();
int8_t Set_door_to_close_position();
void send_old_new_ota_version_to_edge_device();
size_t WiFiSpiClient_write_in(const char *str, bool *stop_flag); // Added by Uttam

void connect_to_wifi(void);

void main(void) {
  initializing_all_modules();
  printk("Starting ADO Device\n");
  int err;
  uint8_t str[100];
  memset(str, 0, 100);
  uint8_t intent_detected = 0;
  uint16_t time_count = 0U, time_count2 = 0U;
  uint8_t evt_msg[1] = {0};
  uint8_t evt_msg_len = 0, count = 0;

  // printk("*********erasing flash memory location***************\n");
  //   int error = external_flash_block_erase(0x0000, 0);
  //   if(error)
  //   {
  //     printk("Error erasing flash memory location\n");
  //     return error;
  //   }

  bool send_door_operation_cmd = false, doorclose = false;
  err = WiFi_Module_int();
  // if(!err)
  //{
  //   WiFi_Network_Connect(NULL);
  // }  //This if condition was previously. Uttam

  check_ssid(); // For autocomnect the WiFi
  stm32_audio_files_read();

  // Check for pending OTA updates
  if (download_the_ota_update || send_and_flash_ota_update) {
    OTA_update_pending = true;
  }

  /* ...........................Main loop..............................*/
  for (;;) {
    // poll the voice AI chip for any wake word command detected

    int INT_DOOR_OPERATIONS = auto_close_timer + 1; // Added by Uttam
// printk("\nAuto close timer in main =%d\n", auto_close_timer / 2);

// For Ultrasonic start from here
#ifdef CONFIG_USS_I2C1_ENABLE
// Ultrasonic_arrary_data_read(chirp_data);
// printf("\nLive USS distance: %d mm\n",  chirp_data[1].range);
#endif
    // Upto this for ultrasonic

    // printk("\ncheck_stm32_ww called from main");
    if (!dump_audio_file_to_server) {
      intent_detected = check_stm32_ww();
    }
    // intent_detected = check_stm32_ww();
    // printf("\n Intent Detected in main %d",intent_detected);

    switch (intent_detected) {
      is_update_to_auto_ai = false;
    case NON_WAKE_WORD:
      ww_from_server = 0;
      // if(!send_door_operation_cmd)
      //{
      //   stm32_audio_files_read();
      //   stm32_version_numbers_read();
      // }

      if (!send_door_operation_cmd) {
        send_door_operation_cmd = true;
        time_count = INT_DOOR_OPERATIONS - 1;
      }

      evt_msg[0] = NON_WAKE_WORD;
      evt_msg_len = sizeof(evt_msg);
      ADO_notify_ble(cmd_mgr_thrd_id, ADO_INTENT_RESPONSE, INT_START, INT_DETECTED,
          evt_msg, evt_msg_len);
      break;

    case DOOR_OPEN:
      ww_from_server = 0;
#ifdef CONFIG_USS_I2C1_ENABLE
      // Ultrasonic range check.
      Ultrasonic_arrary_data_read(chirp_data);
      printf("\nUSS distance during voice command: %d mm\n", chirp_data[1].range);

      // Check the USS range / check response is from server.
      if ((chirp_data[1].range < USS_OPEN_RANGE) || (ww_from_server == DOOR_OPEN)) {
        printf("\nDoor is Opening");

        if ((ww_from_server == DOOR_OPEN)) {
          printf(" from server response\n");
        }

        chg_display_page(DISP_DOOR_OPENING_PAGE); // Added for temp testing, need to remove.
        send_door_operation_cmd = true;
        new_wifi_cred = false;

        if (run_in_intent_mode) {
          time_count = SEND_DOOR_OPEN_CMD;
        } else {
          time_count = INT_DOOR_OPERATIONS;
        }

        evt_msg[0] = DOOR_OPEN;
        evt_msg_len = sizeof(evt_msg);
        ADO_notify_ble(cmd_mgr_thrd_id, ADO_INTENT_RESPONSE, INT_START, INT_DETECTED,
            evt_msg, evt_msg_len);
        printf("\nDOOR_OPEN in main send_door_operation_cmd %d emergency_exit %d settings_open %d battery_low %d \n",
            send_door_operation_cmd ? 1 : 0, emergency_exit ? 1 : 0, settings_open ? 1 : 0, battery_low ? 1 : 0);
      } else {
        // time_count = SEND_DOOR_CLOSE_CMD + 1;
        time_count = INT_DOOR_OPERATIONS;
        printf("\nYour distance is more than %d mm, So Door will not open, Please come close to door.\n", UWB_OPEN_RANGE);
      }

#else
      printf("\nDoor is Opening");
      chg_display_page(DISP_DOOR_OPENING_PAGE);
      send_door_operation_cmd = true;
      new_wifi_cred = false;

      if (run_in_intent_mode) {
        time_count = SEND_DOOR_OPEN_CMD;
      } else {
        // time_count = SEND_DOOR_CLOSE_CMD + 1;
        time_count = INT_DOOR_OPERATIONS;
      }

      evt_msg[0] = DOOR_OPEN;
      evt_msg_len = sizeof(evt_msg);
      ADO_notify_cmd_mgr(COMMAND, ADO_OPERATIONS, OPEN, NULL, 0);
      ADO_notify_ble(cmd_mgr_thrd_id, ADO_INTENT_RESPONSE, INT_START, INT_DETECTED,
          evt_msg, evt_msg_len);
      printf("\nDOOR_OPEN in main send_door_operation_cmd %d emergency_exit %d settings_open %d battery_low %d \n",
          send_door_operation_cmd ? 1 : 0, emergency_exit ? 1 : 0, settings_open ? 1 : 0, battery_low ? 1 : 0);
#endif
      break;

    case DOOR_CLOSE:
      ww_from_server = 0;
      send_door_operation_cmd = true;
      new_wifi_cred = false;

      if (is_bot_staginated2) {
        doorclose = true;
      }
      // time_count = SEND_DOOR_CLOSE_CMD;  //Uttam commented for testing
      time_count = auto_close_timer;

      evt_msg[0] = DOOR_CLOSE;
      evt_msg_len = sizeof(evt_msg);
      ADO_notify_cmd_mgr(COMMAND, ADO_OPERATIONS, CLOSE, NULL, 0);
      ADO_notify_ble(cmd_mgr_thrd_id, ADO_INTENT_RESPONSE, INT_START, INT_DETECTED,
          evt_msg, evt_msg_len);
      printf("\nDOOR_CLOSE in main send_door_operation_cmd %d emergency_exit %d settings_open %d battery_low %d \n",
          send_door_operation_cmd ? 1 : 0, !emergency_exit ? 1 : 0, !settings_open ? 1 : 0, !battery_low ? 1 : 0);
      break;

    case DOOR_STOP:
      ww_from_server = 0;
      send_door_operation_cmd = true;
      new_wifi_cred = false;

      // time_count = SEND_DOOR_CLOSE_CMD + 1; //Uttam commented for testing
      time_count = auto_close_timer + 1;

      evt_msg[0] = DOOR_STOP;
      evt_msg_len = sizeof(evt_msg);
      ADO_notify_ble(cmd_mgr_thrd_id, ADO_INTENT_RESPONSE, INT_START, INT_DETECTED,
          evt_msg, evt_msg_len);

      // Send the operation stop command to cmd_mgr to stop door.
      ADO_notify_cmd_mgr(COMMAND, ADO_OPERATIONS, OPR_STOP, NULL, 0);
      break;

    default:
      break;
    }

    if (not_confident_sent && count < 5) {
      if (stm32_interrupt_flag == true) {
        printf("\n----------------------- Break Not_Confident loop \n");
        continue;
      }

      Get_data_from_auto_ai(false);

      if (stm32_interrupt_flag == true) {
        printf("\n----------------------- Break Not_Confident loop \n");
        continue;
      }

      printf("\n ===> Read notconfident response from main");
      int ret = Get_data_from_auto_ai(false);

      if (ww_from_server) {
        not_confident_sent = false;
        printf("\n===> Read notconfident from main ww_from_server %d \n", ww_from_server);
        continue;
      }

      count++;
    } else {
      not_confident_sent = false;
      count = 0;
    }

    switch (ww_from_server) {
      is_update_to_auto_ai = false;
    case 201:
      printk("\nMatch -> Door_close from server\n");
      ww_from_server = 0; // break the switch case.
      ww_from_server = DOOR_CLOSE;
      ADO_notify_cmd_mgr(COMMAND, ADO_OPERATIONS, CLOSE, NULL, 0);
      break;

    case 202:
      if(run_in_intent_mode) {
      printk("\nMatch -> Door_Open  from server\n");
      ww_from_server = 0; // break the switch case.
      ww_from_server = DOOR_OPEN;
      ADO_notify_cmd_mgr(COMMAND, ADO_OPERATIONS, OPEN, NULL, 0);
      // Added for temp testing, need to remove.
      // chg_display_page(DISP_DOOR_OPENING_PAGE);  // Change the display page to door opening
      }
      else printk(".");
      break;

    case 203:
      printk("\nMatch -> Door_Stop  from server\n");
      ww_from_server = 0; // break the switch case.
      ww_from_server = DOOR_STOP;
      break;

    case 204:
      printk("\nMatch -> Unknown  from server\n");
      ww_from_server = 0; // break the switch case.
      break;

    case 400:
      printk("Match -> Server is Stopped from server\n");
      ww_from_server = 0; // break the switch case.
      break;

    case 500:
      printk("Match -> Server issue from server\n");
      ww_from_server = 0; // break the switch case.
      break;

    default:
      break;
    }

    if (intent_detected == 0) {
      intent_detected = ww_from_server;
    }

    // Check if the doorbot is installed and calibrated
    if (!((is_doorbot_uninstalled_uncalibrated() == true) || (ADO_get_restart_state() == true))) {
      if (doorPositionFromClose < 0.0f && close_opr_done) {
        printf("\n@@@@@@@@@@@@@@@@@@@@@@@ doorPositionFromClose < 0 true in main\n\n");
        doorPositionFromClose = 0.0f;
      }

      // 1- is in open   2- is stoped by cmd
      if ((is_door_in_open == 1 ||
              (is_thread_dead((char *)k_thread_state_str(ado_opr_thrd_id,0,0)) && is_door_in_open != 2)) &&
          (!settings_open)) {
        // this will true if door staginated or door in between close and open position without operation cmd
        door_close_to_open_percentage();
        //   printf("\n============================================> current_door_percentage =%d\n",current_door_percentage);
        if (current_door_percentage > 5 || (doorPositionFromClose < 2.0 && emergency_exit)) {
          if(Auto_close_timer_off)
            time_count2++; // This condition will be used for auto door close. Uttam
          // LOG_DBG("============================================> time_count2 =%d\n",time_count2);
          // printf("============================================> time_count2 =%d\n",time_count2);

          if ((emergency_exit) && (ADO_SEND_DOOR_CLOSE_EMERGENCY == time_count2 && instal_start_cmd == 0)) {
            printf("=====================emergency_exit ENABLED=======================> time_count2 =%d\n",
                time_count2);
            ADO_notify_cmd_mgr(COMMAND, ADO_OPERATIONS, OPEN, NULL, 0);
            time_count2 = 0;
            //} else if (SEND_DOOR_CLOSE_CMD_2_MIN == time_count2 && instal_start_cmd == 0 &&
            //           (current_door_percentage > 5)) {
          } else if (auto_close_timer == time_count2 && instal_start_cmd == 0 &&
                     (current_door_percentage > 5)) {
            printf("============================================> time_count2 =%d\n", time_count2);
            ADO_notify_cmd_mgr(COMMAND, ADO_OPERATIONS, CLOSE, NULL, 0);
            time_count2 = 0;
          }
        } else {
          time_count2 = 0U;
          is_door_in_open = 0;
        }
      } else {
        time_count2 = 0U;
      }
    }

    if (instal_start_cmd > 0) {
      printf("\n==============> ADO_INSTALLATION cmd sent to cmd_mngr");
      ADO_notify_cmd_mgr(COMMAND, ADO_INSTALLATION, instal_start_cmd, NULL, 0);
    }

    if (send_door_operation_cmd && !emergency_exit && !settings_open && !battery_low) {
      if (SEND_DOOR_OPEN_CMD == time_count) {
        // Send the Open-Door command to cmd_mgr
        ADO_notify_cmd_mgr(COMMAND, ADO_OPERATIONS, OPEN, NULL, 0);
        printk("SEND_DOOR_CLOSE_CMD = %d and auto_close_timer =%d",
            SEND_DOOR_CLOSE_CMD, auto_close_timer);
      } else if (SEND_DOOR_CLOSE_CMD == time_count || doorclose) {
        // Send the Close-Door command to cmd_mgr
        if (!is_bot_staginated2 || doorclose) {
          doorclose = false;
          ADO_notify_cmd_mgr(COMMAND, ADO_OPERATIONS, CLOSE, NULL, 0);
        }
      }

      else if (INT_DOOR_OPERATIONS <= time_count && !is_bot_staginated2) {
        stm32_audio_files_read();
        stm32_version_numbers_read();
        send_door_operation_cmd = false;
      }
      if(Auto_close_timer_off)
      ++time_count;
    } else {
      // Check is any thread running. if not running send data to auto ai.
      if (stop_wifi_send) {
        // Check if the operation thread is already running
        if ((is_thread_dead((char *)k_thread_state_str(my_tid_install,0,0))) &&      // check installation thread
            (is_thread_dead((char *)k_thread_state_str(ado_calib_thrd_id,0,0))) &&   // check calibration thread
            (is_thread_dead((char *)k_thread_state_str(reset_calib_thrd_id,0,0))) && // check reset IMU thread
            (is_thread_dead((char *)k_thread_state_str(ado_intent_thrd_id,0,0))) &&  // check intent thread
            (is_thread_dead((char *)k_thread_state_str(ado_opr_thrd_id,0,0))))       // check operations thread
        {
          printk("\nNo threads are running... \n");
          stop_wifi_send = false;
          Check_Audio_files_update_status(); // Require for Auto-Ai
          if (!stop_wifi_send) {
            Send_Audio_files_to_Server(&stop_wifi_send); // pass NULL arguments if you want to make wifi libraries blocking.
            server_connected = false;
          }
        } else {
          // Nothing here
        }
      }
    }
    if(ota_info)
    {
      if (Check_stm_OTA_update_availble()) {
        printf("New STM firmware update available......\n"); 
        evt_msg[0] = 2;
        evt_msg_len = sizeof(evt_msg);
        ADO_notify_ble(cmd_mgr_thrd_id, ADO_OTA_UPDATE_RESPONSE, UPDATE_INFO,
            UPDATE_INFO_RESP, evt_msg, evt_msg_len);      
        send_old_new_ota_version_to_edge_device();
        store_stm_ota_update_status(UPDATE_FOUND, false);
        ota_info=false;
        }
      else {
        printf("STM firmware update not available......\n");
        download_the_ota_update = false;
        send_old_new_ota_version_to_edge_device();
        evt_msg[0] = 1;
        evt_msg_len = sizeof(evt_msg);
        ADO_notify_ble(cmd_mgr_thrd_id, ADO_OTA_UPDATE_RESPONSE, UPDATE_INFO,
            UPDATE_INFO_RESP, evt_msg, evt_msg_len);
        ota_info=false;
      }

    }

    if (check_for_ota_update) {
      // TODO: call check new firmware update function.
      if (Check_stm_OTA_update_availble()) {
        printf("New STM firmware update available......\n");
        store_stm_ota_update_status(UPDATE_FOUND, true);
        download_the_ota_update = true;
        OTA_update_pending = true;

        evt_msg[0] = 2;
        evt_msg_len = sizeof(evt_msg);

        ADO_notify_ble(cmd_mgr_thrd_id, ADO_OTA_UPDATE_RESPONSE, CHECK_UPDATE,
            CHECK_UPDATE_RESP, evt_msg, evt_msg_len);
        k_msleep(500U);
        send_old_new_ota_version_to_edge_device();
        send_door_operation_cmd = false;
      } else {
        if (3 != ADO_WiFi_status(NULL)) { // WL_CONNECTED=3
          evt_msg[0] = 5;
          evt_msg_len = sizeof(evt_msg);
          ADO_notify_ble(cmd_mgr_thrd_id,
              ADO_WIFI_CONTROL_RESPONSE,
              WIFI_CONTROL_CONNECTION_STATUS_CONN,
              WIFI_CONTROL_CONNECTION_STATUS_RESP_CONN,
              evt_msg, evt_msg_len);
          printf("\n WIFI not connected \n");
        } else {
          printf("STM firmware update not available......\n");
          // WiFi_module_on_off(WIFI_MODULE_OFF);
          download_the_ota_update = false;
          send_old_new_ota_version_to_edge_device();
          //evt_msg[0] = 1;
          //evt_msg_len = sizeof(evt_msg);
          //ADO_notify_ble(cmd_mgr_thrd_id, ADO_OTA_UPDATE_RESPONSE, CHECK_UPDATE,
          //    CHECK_UPDATE_RESP, evt_msg, evt_msg_len);
        }
      }
      check_for_ota_update = false;
    }

    if (OTA_update_pending) {
      Set_door_to_open_position();
      k_msleep(1000);
      ota_updating = true;
      DO_OTA_Update();
      ota_updating = false;
      k_msleep(1000);
      Set_door_to_close_position();
      OTA_update_pending = false;
    }

    // Dump Audio file to server.
    if (dump_audio_file_to_server) {
      printk("\nStart sending file to server................\n");

      // dump_audio_file_to_server = true;
      // stm32_interrupt_flag = false;     //disable STM interrupt
      send_door_operation_cmd = false;
      stop_wifi_send = false;
      Check_Audio_files_update_status();
      Send_Audio_files_to_Server(&stop_wifi_send); // pass NULL arguments if you want to make wifi libraries blocking.
      dump_audio_file_to_server = false;
      k_msleep(1000);
      send_door_operation_cmd = true;
      server_connected = false;
    }

    // if (!dump_audio_file_to_server) {
    //   //dump_audio_file_to_server = false;
    //   stm32_interrupt_flag = true;
    //   send_door_operation_cmd = true;
    // }
    // Upto this

#ifdef ESP32
    if (esp32_OTA_start) {
      ota_updating = true;

      printk("----------Starting ESP32 OTA ---------\n");
      WiFi_module_on_off(WIFI_MODULE_ON);
      k_msleep(500);
      WiFi_Network_Connect(NULL);

      ESP32_SelfOTA(&esp32_OTA_start);
      printk("----------ESP32 OTA done---------\n");

      // WiFi_module_on_off(WIFI_MODULE_OFF);
      ota_updating = false;
      esp32_OTA_start = false;
    }
#endif
    // TODO: Add any recurring work here, may be in future it can work as an Idle thread to put the system in low power mode
  }
}

/*
This function attempts to connect to a WiFi network and handles different
scenarios such as successful connection, failure due to new credentials,
and failure due to operations. It also sends notifications about the connection status via BLE.
*/

void connect_to_wifi(void) {
  int err;
  uint8_t evt_msg[1] = {0};
  uint8_t evt_msg_len = 0;

  printf("==============================   main--> connecting from main ========================================\n");
  stop_wifi_send = false;
  err = WiFi_Network_Connect(&stop_wifi_send); // added for operations working while wifi connecting
  printf("==============================   main--> err = %d ========================================\n", err);

  if ((err != 3)) { // if err=3 (connected), err=4 (connection failure), err=-6 (wifi stop due to some operations)
    if ((new_wifi_cred == true) && (stop_wifi_send == true)) {
      printf("=============new wifi cred + stop wifi are true=====================\n");
      new_wifi_cred = false;
    } else {
      printf("cred stop while due to operations\n");
      evt_msg[0] = 5; // STATUS = connection failure
      evt_msg_len = sizeof(evt_msg);
      ADO_notify_ble(cmd_mgr_thrd_id, ADO_WIFI_CONTROL_RESPONSE, WIFI_CONTROL_CONNECTION_STATUS_CONN,
          WIFI_CONTROL_CONNECTION_STATUS_RESP_CONN, evt_msg, evt_msg_len);
    }
  }
}

/* this function initializes various communication interfaces, devices, sensors,
and modules required for the operation of a system. It handles the initialization
of I2C, SPI, UART, LEDs, motors, Bluetooth, and other components, as well as
loading data from EEPROM and performing specific actions based on the device's
installation and calibration status.
*/

void initializing_all_modules(void) {
  int err;
  i2c_init();   // initialize i2c communication device tree
  i2c_1_init(); // initialize i2c1 communication device tree

  spi_init();  // initialize SPI communication device tree
  spi2_init(); // spi2 for external flash storage
  external_flash_init();
  load_audio_file_address();

  gpio_expander_init(); // added for gpio expander

#ifdef CONFIG_USS_I2C1_ENABLE
  CH101_USS_INIT();
#endif
  init_STM32H7();

  // intialise UART_1
  err = initUart1();
  if (err != 0) {
    printk("Failed to initialise UART_1 (err %d)\n", err);
  }

  // Show Startup page on display
  chg_display_page(DISP_START_PAGE);

  // initialise PIR sensor gpio
  init_PIR_sensor();

  // initialise board LED GPIOs
  err = dk_leds_init();
  if (err) {
    printk("LEDs init failed (err %d)\n", err);
  }

  init_display_control(); // Initialize respected GPIO required to control display power.
  display_power_control(DIS_TURN_ON);

  // initialise all motors
  err = initAllMotors();

  if (err != 0) {
    printk("Motor init failed (err %d)\n", err);
  }
  printk("motor intialized*******\n");
  motor_driver_power_relay_control(MOTOR_RELAY_TURN_ON);

  // initialise BLE
  err = initBLE();
  if (err != 0) {
    printk("Failed to initialise BLE (err %d)\n", err);
  }

  // ************************************ initialise BNO-080 IMU **************************************
  // if(initIMU_BNO080() == true)
  //{
  //  enableGameRotationVector(50);
  //  printk("IMU_BNO080 initialization done\n");
  //}
  // else
  //{
  //  printk("Failed to initialise IMU\n");
  //}

  // ************************************ initialise ICM-20948 IMU **************************************
  if (init_ICM20948_setup() == true) {
    printk("ICM-20948 initialization done\n");
  } else {
    printk("Failed to initialise IMU\n");
  }

  // optionally added.
  nrf_gpio_cfg_output(P_BAL_B);  // added for product balancer
  nrf_gpio_cfg_output(P_BAL_F);  // added for product balancer
  nrf_gpio_cfg_output(CLUTCH_F); // added for product balancer
  nrf_gpio_cfg_output(CLUTCH_B); // added for product balancer

  nrf_gpio_pin_set(P_BAL_F);
  nrf_gpio_pin_set(P_BAL_B);
  nrf_gpio_pin_set(CLUTCH_F);
  nrf_gpio_pin_set(CLUTCH_B);

  // added for power logs from ADO
  printk("-------------- ADO ");
  // pre_battery_percentage = ADO_Battery_Percen_Read(BATTERY_VOLTAGE_SENSOR);
  // Update_Battery_Percentage(pre_battery_percentage);

  load_install_data(); // loads the stored eeprom installation data into global variables
  load_calib_data();   // loads the stored eeprom calibration data into global variables
  load_config_data();
  load_operation_data(); // loads the stored eeprom operation data into global variable basicalli clutch for now

  // Check if the ADO is installed on the door or not
  if ((is_ADO_Installed(&clamp_data) == true) && (is_ADO_Calibrated(&calib) == true)) {
    // Check and load the values from EEPROM if ADO is calibrated and Intent is enabled in device configurations
    if (is_Intent_Enabled() == true) {
      // Enable the PIR sensor to start intent listening
      enable_pir_1();
    } else {
      printk("Intent disabled\n");
    }

    // Send a mailbox message to cmd_mgr thread to start the reset calibration sequence
    // Commented as we will now be querying whether the doorbot has reset or not from ED.
    // ADO_notify_cmd_mgr(COMMAND, ADO_RESET_REQUEST, RESET_START, NULL, 0);
    ADO_set_restart_state(ADO_RESTARTED);
  } else {
    printk("Doorbot not installed on door or uncalibrated\n");
  }
  gpio_expander_port_A_pin_set(LED_CONTROL); // Trun ON LED strip power
}

/* This function performs OTA updates by downloading the firmware file,
validating it, and then sending and flashing it to an STM32 device.
It handles the download, validation, installation, and status notifications
to the BLE module. The function also includes error handling and attempts
to retry the OTA update process a certain number of times if errors occur.
*/

int8_t DO_OTA_Update() {
  uint8_t ota_update_try_attempts = 0;
  uint8_t evt_msg[1] = {0};
  uint8_t evt_msg_len = sizeof(evt_msg);

  while (1) {
    if (download_the_ota_update == true) {
      printk("Calling the Download_files_from_server function...\n");

      power_optimization(OTA);
      k_msleep(1000); // delay to turn on dispaly.

      // Change the display page to firmware updating
      chg_display_page(DISP_UPDATING_PAGE);

      evt_msg[0] = DOWNLOAD_STARTED;
      evt_msg_len = sizeof(evt_msg);
      ADO_notify_ble(cmd_mgr_thrd_id, ADO_OTA_UPDATE_RESPONSE, UPDATE_PROGRESS,
          UPDATE_PROGRESS_RESP, evt_msg, evt_msg_len);

      if (!Download_stm_hex_file()) {
        // Validate hex file.
        if (validate_stm_new_hex_file()) {
          evt_msg[0] = ERROR_IN_DOWNLOAD_HEX_FILE;
          evt_msg_len = sizeof(evt_msg);
          ADO_notify_ble(cmd_mgr_thrd_id, ADO_OTA_UPDATE_RESPONSE, UPDATE_PROGRESS,
              UPDATE_PROGRESS_RESP, evt_msg, evt_msg_len);
          printf("****Error in hex file****\n");

          // Change the display page to firmware updating failed
          chg_display_page(DISP_FAILED_TO_UPDATE);
          send_and_flash_ota_update = false;
          ota_update_try_attempts++;

          if (ota_update_try_attempts == 10) {
            download_the_ota_update = false;
            send_and_flash_ota_update = false;
            // NOTE: revert back with OLD fw string.
            store_stm_ota_update_status(UPDATE_DOWNLOADED, false);
            evt_msg[0] = OTA_FAILED;
            evt_msg_len = sizeof(evt_msg);
            ADO_notify_ble(cmd_mgr_thrd_id, ADO_OTA_UPDATE_RESPONSE, UPDATE_PROGRESS,
                UPDATE_PROGRESS_RESP, evt_msg, evt_msg_len);
            return -1;
          }
          k_msleep(1000);
        } else {
          printf("---No error in hex file---\n");
          // WiFi_module_on_off(WIFI_MODULE_OFF);
          store_stm_ota_update_status(UPDATE_DOWNLOADED, true);
          download_the_ota_update = false;
          send_and_flash_ota_update = true;
          ota_update_try_attempts = 0;

          evt_msg[0] = DOWNLOAD_COMPLETED;
          evt_msg_len = sizeof(evt_msg);
          ADO_notify_ble(cmd_mgr_thrd_id, ADO_OTA_UPDATE_RESPONSE, UPDATE_PROGRESS,
              UPDATE_PROGRESS_RESP, evt_msg, evt_msg_len);
        }
      } else {
        evt_msg[0] = DOWNLOAD_FAILED_DUE_TO_WIFI;
        evt_msg_len = sizeof(evt_msg);
        ADO_notify_ble(cmd_mgr_thrd_id, ADO_OTA_UPDATE_RESPONSE, UPDATE_PROGRESS,
            UPDATE_PROGRESS_RESP, evt_msg, evt_msg_len);
        ota_update_try_attempts++;

        if (ota_update_try_attempts == 3) {
          printk("\nOTA  uptade 3 attempts are completed. download is stopped.\n");
          download_the_ota_update = false;
          send_and_flash_ota_update = false;
          // NOTE: revert back with OLD fw string.
          store_stm_ota_update_status(UPDATE_DOWNLOADED, false);
          evt_msg[0] = OTA_FAILED;
          evt_msg_len = sizeof(evt_msg);
          ADO_notify_ble(cmd_mgr_thrd_id, ADO_OTA_UPDATE_RESPONSE, UPDATE_PROGRESS,
              UPDATE_PROGRESS_RESP, evt_msg, evt_msg_len);
          return -2;
        }
      }
    }

    if (send_and_flash_ota_update == true) {
      // Change the display page to firmware updating
      chg_display_page(DISP_UPDATING_PAGE);

      evt_msg[0] = INSTALLATION_STARTED;
      evt_msg_len = sizeof(evt_msg);
      ADO_notify_ble(cmd_mgr_thrd_id, ADO_OTA_UPDATE_RESPONSE, UPDATE_PROGRESS,
          UPDATE_PROGRESS_RESP, evt_msg, evt_msg_len);

      // Send Valid New Hex File to STM over SPI.
      if (!send_nex_hex_file_to_stm32()) {
        // wait untill nex hel file flash into STM32
        if (!flash_nex_hex_file_to_stm32()) {
          evt_msg[0] = INSTALLATION_COMPLETED;
          evt_msg_len = sizeof(evt_msg);
          ADO_notify_ble(cmd_mgr_thrd_id, ADO_OTA_UPDATE_RESPONSE, UPDATE_PROGRESS,
              UPDATE_PROGRESS_RESP, evt_msg, evt_msg_len);

          ota_update_try_attempts = 0;
          send_and_flash_ota_update = false;
          store_stm_ota_update_status(UPDATE_SEND_TO_STM, true);
          power_optimization(OTA_STOP);
          k_msleep(4000);

          evt_msg[0] = OTA_SUCCESS;
          evt_msg_len = sizeof(evt_msg);
          ADO_notify_ble(cmd_mgr_thrd_id, ADO_OTA_UPDATE_RESPONSE, UPDATE_PROGRESS,
              UPDATE_PROGRESS_RESP, evt_msg, evt_msg_len);
          break;
        }
      } else {
        evt_msg[0] = INSTALLATION_FAILED;
        evt_msg_len = sizeof(evt_msg);
        ADO_notify_ble(cmd_mgr_thrd_id, ADO_OTA_UPDATE_RESPONSE, UPDATE_PROGRESS,
            UPDATE_PROGRESS_RESP, evt_msg, evt_msg_len);

        // Change the display page to firmware updating failed
        chg_display_page(DISP_FAILED_TO_UPDATE);
        k_msleep(1000);
        ota_update_try_attempts++;

        if (ota_update_try_attempts == 10) {
          download_the_ota_update = false;
          send_and_flash_ota_update = false;
          // NOTE: revert back with OLD fw string.
          store_stm_ota_update_status(UPDATE_SEND_TO_STM, false);
          evt_msg[0] = OTA_FAILED;
          evt_msg_len = sizeof(evt_msg);
          ADO_notify_ble(cmd_mgr_thrd_id, ADO_OTA_UPDATE_RESPONSE, UPDATE_PROGRESS,
              UPDATE_PROGRESS_RESP, evt_msg, evt_msg_len);
          return -3;
        }
      }
    }
    check_stm32_ww();
  }
  return 0;
}

/* This function is responsible for opening the door to prepare for an OTA update.
It checks if the door is calibrated and reset, sends the "Open Door" command to the
command manager module, and waits until the door reaches the open position or until
a maximum number of iterations is reached. This function is typically used before
initiating an OTA update to ensure that the door is in the correct position for the
update process.
*/

int8_t Set_door_to_open_position() {
  printk("\nOpening door for OTA\n");

  if (reset_option() != 2) {
    printk("Not Calibrated or Not Reseted\n");
    return 0;
  }
  int i = 0;

  if (OTA_check_operation_status() != OPENED_ANDROID) {
    printk("Opening door for OTA update\n");
    // Send the Open-Door command to cmd_mgr
    ADO_notify_cmd_mgr(COMMAND, ADO_OPERATIONS, OPEN, NULL, 0);
    is_door_in_open = 2; // it pevent general close the door without cmd  after 2 min
    k_msleep(3000);

    while (OTA_check_operation_status() != OPENED_ANDROID) {
      k_msleep(500);
      printk("*");
      i++;
      if (20 == i) {
        break;
      }
    }
    printk("Door opend and starting OTA update\n");
  }
  return 0;
}

/* This function is responsible for closing the door after an OTA update.
It checks if the door is calibrated and reset, sends the "Close Door" command
to the command manager module, and waits until the door reaches the closed
position or until a maximum number of iterations is reached. This function
is typically used to finalize the OTA update process by closing the door.
*/

int8_t Set_door_to_close_position() {
  printk("\nClosing door for OTA\n");

  if (reset_option() != 2) {
    printk("Not Calibrated or Not Reseted\n");
    return 0;
  }
  int i = 0;

  if (OTA_check_operation_status() != CLOSED_ANDROID) {
    printk("Closing door for OTA update\n");
    // Send the Open-Door command to cmd_mgr
    ADO_notify_cmd_mgr(COMMAND, ADO_OPERATIONS, CLOSE, NULL, 0);
    k_msleep(3000);

    while (OTA_check_operation_status() != CLOSED_ANDROID) {
      k_msleep(500);
      printk("*");
      i++;
      if (20 == i) {
        break;
      }
    }
    printk("Door closed and completed OTA update\n");
  }
  return 0;
}

/* This function is responsible for sending the old and new firmware version
numbers to the edge device. It retrieves the version numbers using the
"stm_old_new_fw_version_to_edge_device" function and notifies the BLE module
with the version numbers using ADO_notify_ble.
*/

void send_old_new_ota_version_to_edge_device() {
  uint8_t stm_old_new_ver_numbers[6] = {0};
  stm_old_new_fw_version_to_edge_device(stm_old_new_ver_numbers);
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_OTA_UPDATE_RESPONSE, UPDATE_INFO,
      UPDATE_INFO_RESP, stm_old_new_ver_numbers, 6);
}