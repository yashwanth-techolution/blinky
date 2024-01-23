#include <math.h>
#include <stdio.h>

#include "Includes/ADO_Configurations.h"
#include "Includes/ADO_ICM20948_Module.h"
#include "Includes/ADO_Operations_Module.h"
#include "Includes/ADO_PIR_Module.h"
#include "Includes/ADO_Ultrasonic_Sensors.h"
#include "Includes/AnalogIn.h"
#include "Includes/BNO080.h"
#include "Includes/CH101_USS.h"
#include "Includes/Calibration_Module.h"
#include "Includes/Command_Manager_Module.h"
#include "Includes/Device_Status_Module.h"
#include "Includes/Display_Module.h"
#include "Includes/IMU_Module.h"
#include "Includes/Install_ADO.h"
#include "Includes/Intent_Module.h"
#include "Includes/UART_Module.h"

// Mailbox declaration
struct k_mbox dev_stat_mb;
ado_power_modes current_power_mode;
int8_t curr_open_percent = 0U;

// battery percentage store variable
uint8_t pre_battery_percentage = 100;
float doorPositionFromClose = 0;
float G_IMU = 0;
float G_IMU1[] = {0, 0};
float tempIMU = 180.0f;
bool run_in_intent_mode = false;
bool battery_low = false;
bool display_page_change = false;

/* Array of structs to hold measurement data, one for each possible device */
chirp_data_t chirp_data[CHIRP_MAX_NUM_SENSORS];

// Variable initializations for moving average
uint16_t mov_avg[MOV_AVG_WINDOW_SIZE] = {0};
int32_t gUssDistanceMM = 0U;
ado_dev_state ado_state = {.has_ADO_restarted = false, .is_battery_charging = false};

uint8_t ultrasonic_detection_type(uint32_t distance);
uint8_t battery_percent_update();

// Define a thread to keep checking the device status of doorbot
// so that it needs not to be invoked separately by any
// function but by scheduler immediately after power ON
K_THREAD_DEFINE(dev_stat_thrd_id, DEV_STAT_THRD_STACK_SIZE, ADO_DevStat_Thread,
    NULL, NULL, NULL, DEV_STAT_THRD_PRIORITY, K_ESSENTIAL, 0);

/*
 * brief: This thread is an essential thread to respond to any device
 * status commands as well as periodically report the door position to ED
 */
void ADO_DevStat_Thread() {
  uint16_t wait_count = 0; // variable to store wait cycels count to update battery percent and device status.
  // uint16_t New_IMU = 0;
  uint32_t Dummy_ch101_value = 0;
  float tempYawVal_BNO080;

  ////battery percentage store variable
  // uint8_t battery_percentage;

  // Mailbox variables
  struct k_mbox_msg recv_msg;
  struct k_timer my_timer;
  bool timerstart = false;
  // Response data variables
  // uint8_t evt_msg[2] = {0};
  // uint8_t evt_msg_len = 0;

  // Device status variables
  uint8_t dev_stat_cmd = 0U;

  // Initialize dev_stat Mailbox
  k_mbox_init(&dev_stat_mb);
  char *thread_state_operation = NULL;
  k_msleep(4000U);

  if (display_activity == false) {
    // printk("\n========== Device status --> (before enter into loop) display is off and making it is on =========\n");
    display_power_control(DIS_TURN_ON);
    display_activity = true;
    k_msleep(1000U);
  }

  // printk("\n========== Device status --> (before enter into loop) Checking battery percentage =========\n");
  battery_percent_update(); // testing battery percentage before reading the PIR, IMU and other device status requests
  k_timer_init(&my_timer, NULL, NULL);

  while (1) {
#if UWB
    if (distance_lessthan200mtr == 1) {
      send_open();
    } else if (distance_lessthan200mtr == 0) {
      send_close();
    }
#endif
    // prepare the mailbox receive buffer
    recv_msg.info = 0;
    recv_msg.size = sizeof(dev_stat_cmd);
    recv_msg.rx_source_thread = cmd_mgr_thrd_id;
    // retrieve and delete the message if recieved, keep waiting for the mailbox message to arrive till DEV_STAT_POLL_INTERVAL
    k_mbox_get(&cmd_mgr_mb, &recv_msg, &dev_stat_cmd, DEV_STAT_POLL_INTERVAL);

    // check if valid PIR interrupt detected
    // if(valid_PIR == Detected_PIR)
    if ((valid_PIR == Detected_PIR)) {
      valid_PIR = Not_Detected_PIR;

      // Check if the ADO is installed and calibrated
      if ((is_ADO_Installed(&clamp_data) == true) && (is_ADO_Calibrated(&calib) == true)) {
        printk("PIR detected\n");
        power_optimization(PIR);

        if (timerstart == false) {
          k_timer_start(&my_timer, K_MSEC(1000), K_MSEC(0)); // only set if timer not yet started
          timerstart = true;
        }

        // Change the Display Page to "say-Open-Door".
        // chg_display_page(DISP_SAY_DOOR_OPEN_PAGE);
        run_in_intent_mode = true;
        // disable_pir_1();
      } else {
        // Debug print
        printk("doorbot is uninstalled or uncalibrated\n");
        // disable further PIR interupts, they will be enabled when installation or calibration done.
        disable_pir_1();
      }
    }

    if ((k_timer_status_get(&my_timer) > 0) && timerstart) {
      timerstart = false;
      // printf("\n\n @@@@@@@@@timer stop in dev");
      k_timer_stop(&my_timer);

      if ((operation_flag == false) && (battery_low == false)) {
        // Change the Display Page to "say-Open-Door".
        chg_display_page(DISP_SAY_DOOR_OPEN_PAGE);
      }
    }

    if ((battery_low == true)) {
      //  printk("\n========== Device status -> Display page = low battery =========\n");
      if (display_activity == false) {
        // printk("\n==========display is off and making it is on =========\n",display_activity);
        display_power_control(DIS_TURN_ON);
        display_activity = true;
        // k_mbox_get(&cmd_mgr_mb, &recv_msg, &dev_stat_cmd, K_SECONDS(1)); // To make it non - blocking
      }

      // Show critical low battery page on dispaly
      chg_display_page(DISP_LOW_BATT_RED_PAGE);
      // display_page_change=true;
    }

    // tempYawVal_BNO080 = IMU_READRawBNO080();  // for BNO080
    tempYawVal_BNO080 = Read_ICM20948_YAW_Data(); // for ICM - 20948

    if (tempYawVal_BNO080 >= 0.0f) {
      // To avoid any zero values when the sensor is not ready and data is not available to read
      if (fabs(tempYawVal_BNO080 - G_IMU) > 0.1f) {
        if (calib.is_direction_cw) {
          printf("Device status -> DS_CW \n");
          if (fabs(tempYawVal_BNO080 - G_IMU) > 180.0f) {
            if ((tempYawVal_BNO080 - G_IMU) < 0.0f) {
              doorPositionFromClose += 360 + (tempYawVal_BNO080 - G_IMU);
            } else {
              doorPositionFromClose += 360 - (tempYawVal_BNO080 - G_IMU);
            }
          } else {
            doorPositionFromClose += tempYawVal_BNO080 - G_IMU;
          }
        } else {
          printf("Device status -> DS_ACW \n");
          if (fabs(tempYawVal_BNO080 - G_IMU) > 180.0f) {
            if ((tempYawVal_BNO080 - G_IMU) < 0.0f) {
              doorPositionFromClose += -(360 + (tempYawVal_BNO080 - G_IMU));
            } else {
              if ((tempYawVal_BNO080 - G_IMU) > 350) {
                doorPositionFromClose += +(360 - (tempYawVal_BNO080 - G_IMU));
              } else {
                doorPositionFromClose += -(360 - (tempYawVal_BNO080 - G_IMU));
              }
            }
          } else {
            doorPositionFromClose += -(tempYawVal_BNO080 - G_IMU);
          }
        }
        // printf("IMU_avg = %f      %f          %f \n", G_IMU,doorPositionFromClose,(tempYawVal_BNO080-G_IMU));
        printf("Device Status -> IMU_tempYawVal = %f, From Close = %f, Difference = %f\n",
            tempYawVal_BNO080, doorPositionFromClose, (tempYawVal_BNO080 - G_IMU));
        G_IMU = tempYawVal_BNO080;
        G_IMU1[0] = tempYawVal_BNO080;
      }
    } else {
      printf("Device Status -> IMU = %f, IMU_tempYawVal = %f\n", G_IMU, tempYawVal_BNO080);
    }

    G_IMU1[1] = chirp_data[0].range;
    thread_state_operation = k_thread_state_str(ado_opr_thrd_id,0,0);

    // printk("%s\n",thread_state_operation );
    if (!is_thread_dead(thread_state_operation)) {
      // printk("Operation going on...\n");
      ADO_notify_short(ado_opr_thrd_id, G_IMU1, sizeof(G_IMU1[0]) * 2); // 2 --> As we are sending two floats
    }

    switch (dev_stat_cmd) {
    case ADO_DEVICE_VERSION:
      // evt_msg[0]  = curr_open_percent;
      // evt_msg_len = 1U;
      ado_device_version();
      dev_stat_cmd = 0U; // Imp: Reset the command to 0 to avoid multiple executions in loop
      break;

    case PERIPHERAL_VERSIONS:
      // evt_msg[0]  = curr_open_percent;
      // evt_msg_len = 1U;
      peripheral_version();
      dev_stat_cmd = 0U; // Imp: Reset the command to 0 to avoid multiple executions in loop
      break;

    case MODEL_VERSIONS:
      // evt_msg[0]  = curr_open_percent;
      // evt_msg_len = 1U;
      model_version();
      dev_stat_cmd = 0U; // Imp: Reset the command to 0 to avoid multiple executions in loop
      break;

    case CURRENT_DOOR_POS:
      // evt_msg[0]  = curr_open_percent;
      // evt_msg_len = 1U;
      current_door_pos();
      dev_stat_cmd = 0U; // Imp: Reset the command to 0 to avoid multiple executions in loop
      break;

    case ADO_BATTERY_PERCENT:
      // evt_msg[0]  = curr_open_percent;
      // evt_msg_len = 1U;
      ado_battery_percentage();
      dev_stat_cmd = 0U; // Imp: Reset the command to 0 to avoid multiple executions in loop
      break;

    case ADO_RESTART_STATUS:
      // evt_msg[0]  = curr_open_percent;
      // evt_msg_len = 1U;
      ado_restart_status();
      dev_stat_cmd = 0U; // Imp: Reset the command to 0 to avoid multiple executions in loop
      break;

    case DEV_STATUS_REG_UPDATE:
      // evt_msg[0]  = curr_open_percent;
      // evt_msg_len = 1U;
      device_status_reg_update();
      dev_stat_cmd = 0U; // Imp: Reset the command to 0 to avoid multiple executions in loop
      break;

    case WIFI_CONNECTION_STATUS:
      // evt_msg[0]  = curr_open_percent;
      // evt_msg_len = 1U;
      // wifi_status();
      printk("\n-------- Wifi connection status request from edge device device status ------ \n");
      check_ado_wifi_status();
      dev_stat_cmd = 0U; // Imp: Reset the command to 0 to avoid multiple executions in loop
      break;

    case INSTALLATION_STATUS:
      // evt_msg[0]  = curr_open_percent;
      // evt_msg_len = 1U;
      installation_status();
      dev_stat_cmd = 0U; // Imp: Reset the command to 0 to avoid multiple executions in loop
      break;

    case CALIBRATION_STATUS:
      // evt_msg[0]  = curr_open_percent;
      // evt_msg_len = 1U;
      calibration_status();
      dev_stat_cmd = 0U; // Imp: Reset the command to 0 to avoid multiple executions in loop
      break;

    case OPERATION_STATUS:
      // evt_msg[0]  = curr_open_percent;
      // evt_msg_len = 1U;
      operation_status();
      dev_stat_cmd = 0U; // Imp: Reset the command to 0 to avoid multiple executions in loop
      break;

    case INTENT_STATUS_DS:
      // evt_msg[0]  = curr_open_percent;
      // evt_msg_len = 1U;
      intent_status();
      dev_stat_cmd = 0U; // Imp: Reset the command to 0 to avoid multiple executions in loop
      break;             // RED Right now

    case SENSORS_STATUS:
      // evt_msg[0]  = curr_open_percent;
      // evt_msg_len = 1U;
      sensors_status();
      dev_stat_cmd = 0U; // Imp: Reset the command to 0 to avoid multiple executions in loop
      break;

    case CLUTCH_STATUS:
      // evt_msg[0]  = curr_open_percent;
      // evt_msg_len = 1U;
      clutch_status();
      dev_stat_cmd = 0U; // Imp: Reset the command to 0 to avoid multiple executions in loop
      break;

    case SLEEP_MODE:
      // evt_msg[0]  = curr_open_percent;
      // evt_msg_len = 1U;
      sleep_mode();
      dev_stat_cmd = 0U; // Imp: Reset the command to 0 to avoid multiple executions in loop
      break;

    case ERROR_STATUS:
      // evt_msg[0]  = curr_open_percent;
      // evt_msg_len = 1U;
      error_status();
      dev_stat_cmd = 0U; // Imp: Reset the command to 0 to avoid multiple executions in loop
      break;

    case EM_LOCK_STATUS:
      em_lock_status();
      dev_stat_cmd = 0U;
      break;

    default:
      wait_count++;

      if (wait_count == DEV_STAT_POLL_COUNT) {
        battery_percent_update();
        wait_count = 0;
      }
      break;
    }
  }
}

/*
 * brief: This function will read and return the current IMU sensor reading
 */
int8_t get_door_pos(void) {
  // uint16_t curr_IMU = 0;
  // int8_t   curr_pos = 0;

  //// Read the present value of IMU Sensor
  // curr_IMU = IMU_READMedian();

  //// validate the read IMU Position
  // if((curr_IMU >= 0) && (curr_IMU < 360)) {
  //   // calculate the percentage of door opened WRT calibrated close position
  //   // TODO: Optimise the door_close_to_open_percentage() to return the result
  //   //       instead of modifying global variable. For now, call it
  //   //       and read the global variable
  //   //door_close_to_open_percentage(curr_IMU, true, false);
  //   curr_pos = current_door_percentage;
  // } else {
  //   // Debug print
  //   printk("Invalid IMU Value Read\n");
  //   return -1;
  // }

  //// return the door percentage
  // return curr_pos;

  return current_door_percentage;
}

/*
 * brief: This function will read the ADO Firmware version string and sends it back to ED
 */
/*
void ado_get_FW_Ver() {
  uint8_t evt_msg[MAX_FW_VER_STR_LEN] = {0};
  uint8_t evt_msg_len = 0U;

  // Prepare the Firmware Version String
  strcpy(evt_msg, ADO_FW_VER_STR);

  evt_msg_len = strlen(evt_msg);
  if ((evt_msg_len < MAX_FW_VER_STR_LEN) && (evt_msg_len > 0)) {
    printk("FW_Ver: %s\r\n", evt_msg);

    // Notify Over BLE
    ADO_notify_ble(cmd_mgr_thrd_id, ADO_DEVICE_STATUS_RESPONSE,
        REQ_FW_VER, RESP_FW_VER, evt_msg, evt_msg_len);
  } else {
    // debug print
    printk("Can't get FW version string\r\n");
  }
}
*/

uint8_t ultrasonic_detection_type(uint32_t distance) {
  // printk("Device Status -> Range = %d\n", distance);
  return 0;
}

uint8_t battery_percent_update() {
  // battery percentage store variable
  uint8_t battery_percentage;

  // added for power logs from ADO
  battery_percentage = ADO_Battery_Percen_Read(BATTERY_VOLTAGE_SENSOR);

  if (abs(pre_battery_percentage - battery_percentage) > 2) {
    if (battery_percentage <= CUT_OFF_BATT_PERCENT) {
      // Show critical low battery page on dispaly
      chg_display_page(DISP_LOW_BATT_RED_PAGE);
      battery_low = true;
      printk("\n---------------battery is low = %d-----\n", battery_percentage);
      // TODO:  write any shutdown logic here.
    } else {
      battery_low = false;
    }
    Update_Battery_Percentage(battery_percentage);
    pre_battery_percentage = battery_percentage;
  }

  printk("---- ADO Battery percentage = %d \n", pre_battery_percentage);
  // chg_display_page(DISP_UPDATING_PAGE);
  return pre_battery_percentage;
}

/*
 * brief: Function to check whether ADO device has been restarted
 */
bool ADO_get_restart_state(void) {
  return (bool)ado_state.has_ADO_restarted;
}

/*
 * brief: Function to set/reset the ADO device state if it has been restarted
 */
void ADO_set_restart_state(bool state) {
  // Set the global structure variable as per the state received
  ado_state.has_ADO_restarted = state;
}

/*
 * brief: Function to set/reset the ADO device state if it has been restarted
 */
bool ADO_get_battery_state() {
  // TODO: Implement the logic to determine whether the battery is charging onboard, presently it will return 'false' always
  //  return whether the device battery is charging
  return (bool)ado_state.is_battery_charging;
}

void ado_device_version() {
  uint8_t evt_msg[MAX_DEV_VER_STR_LEN] = {ADO_FW_VER_MAJOR, ADO_FW_VER_MINOR,
      ADO_FW_VER_REVISION, ADO_HW_VER_MAJOR, ADO_HW_VER_MINOR, ADO_HW_VER_REVISION};
  uint8_t evt_msg_len = 0U;

  // Prepare the Firmware Version String
  // strcpy(evt_msg, "N" );          //ADO_FW_VER_STR
  // strcpy(evt_msg, "I");

  evt_msg_len = sizeof(evt_msg);
  if ((evt_msg_len <= MAX_DEV_VER_STR_LEN) && (evt_msg_len > 0)) {
    printk("Device_Ver: %s\r\n", evt_msg);

    // Notify Over BLE
    ADO_notify_ble(cmd_mgr_thrd_id, ADO_DEVICE_STATUS_RESPONSE,
        ADO_DEVICE_VERSION, ADO_DEVICE_VERSION_RESP, evt_msg, evt_msg_len);
  } else {
    // debug print
    printk("Can't get Device version string\r\n");
  }
}

void peripheral_version() {
  uint8_t evt_msg[MAX_PER_VER_STR_LEN] = {ADO_WIFI_VER_MAJOR, ADO_WIFI_VER_MINOR,
      ADO_WIFI_VER_REVISION, ADO_STM_VER_MAJOR, ADO_STM_VER_MINOR, ADO_STM_VER_REVISION};
  uint8_t evt_msg_len = 0U;

  // Prepare the Firmware Version String
  evt_msg_len = sizeof(evt_msg);

  if ((evt_msg_len <= MAX_PER_VER_STR_LEN) && (evt_msg_len > 0)) {
    printk("PW_Ver: %s\r\n", evt_msg);
    // Notify Over BLE
    ADO_notify_ble(cmd_mgr_thrd_id, ADO_DEVICE_STATUS_RESPONSE, PERIPHERAL_VERSIONS,
        PERIPHERAL_FW_VERSIONS_RESP, evt_msg, evt_msg_len);
  } else {
    // debug print
    printk("Can't get PW version string\r\n");
  }
}

void model_version() {
  uint8_t evt_msg[MAX_MODEL_VER_STR_LEN] = {ADO_VOICE_MODEL_VER_MAJOR,
      ADO_VOICE_MODEL_VER_MINOR, ADO_VOICE_MODEL_VER_REVISION, ADO_MOTION_INTENT_VER_MAJOR,
      ADO_MOTION_INTENT_VER_MINOR, ADO_MOTION_INTENT_VER_REVISION};
  uint8_t evt_msg_len = 0U;

  // Prepare the Firmware Version String
  evt_msg_len = sizeof(evt_msg);
  if ((evt_msg_len <= MAX_MODEL_VER_STR_LEN) && (evt_msg_len > 0)) {
    printk("MW_Ver: %s\r\n", evt_msg);
    // Notify Over BLE
    ADO_notify_ble(cmd_mgr_thrd_id, ADO_DEVICE_STATUS_RESPONSE, MODEL_VERSIONS,
        MODEL_VERSIONS_RESP, evt_msg, evt_msg_len);
  } else {
    // debug print
    printk("Can't get MW version string\r\n");
  }
}

void current_door_pos() {
  curr_open_percent = get_door_pos();
  if ((curr_open_percent >= 0) && (curr_open_percent <= 100)) {
    // Prepare the notification message
    uint8_t evt_msg[MAX_CUR_DOOR_POS_STR_LEN] = {0};
    evt_msg[0] = curr_open_percent;
    uint8_t evt_msg_len = 1U;
    printk("\nDevice status for door position called, and the value is: %d\n", evt_msg[0]);

    // Notify to BLE via cmd_mgr
    ADO_notify_ble(cmd_mgr_thrd_id, ADO_DEVICE_STATUS_RESPONSE, CURRENT_DOOR_POS,
        CURRENT_DOOR_POS_RESP, evt_msg, evt_msg_len);
  } else {
    // Debug prints
    printk("\nError getting door position, curr_open_percent = %d\n", curr_open_percent);
  }
}

void ado_battery_percentage() {
  uint8_t evt_msg[MAX_BAT_PER_STR_LEN] = {0};
  evt_msg[0] = battery_percent_update();
  evt_msg[1] = ADO_get_battery_state();
  uint8_t evt_msg_len = 2U;
  printk("\nDevice Status called for battery (%). Its returning two values."
         "\n Battery percentage: %d, battery state: %d\n",
      evt_msg[0], evt_msg[1]);

  // report over BLE, the current battery percentage
  // TODO: When the ADO has capability of direct charging, check for the battery
  // charging/discharging status, presently it only discharges
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_DEVICE_STATUS_RESPONSE, ADO_BATTERY_PERCENT,
      ADO_BATTERY_PERCENT_RESP, evt_msg, evt_msg_len);
}

void ado_restart_status() {
  uint8_t evt_msg[MAX_RESTART_STATUS_STR_LEN] = {0};
  uint8_t evt_msg_len = 0;
  evt_msg[0] = ADO_get_restart_state();
  evt_msg_len = sizeof(evt_msg);
  printk("\nDevice Status called for restart state, and the value is: %d\n", evt_msg[0]);
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_DEVICE_STATUS_RESPONSE, ADO_RESTART_STATUS,
      ADO_RESTART_STATUS_RESP, evt_msg, evt_msg_len);
}

void device_status_reg_update() {
  uint8_t evt_msg[MAX_DS_REG_UPDATE_VER_STR_LEN] = {0, 11, 2, 5, 6, 7};
  uint8_t evt_msg_len = sizeof(evt_msg);
  // evt_msg[0] = 56;
  // TODO: Dynamic data needs to be sent according to the need.
  evt_msg_len = sizeof(evt_msg);
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_DEVICE_STATUS_RESPONSE, DEV_STATUS_REG_UPDATE,
      DEV_STATUS_REG_UPDATE_RESP, evt_msg, evt_msg_len);
}

void installation_status() {
  uint8_t evt_msg[MAX_INST_STATUS_STR_LEN] = {0};
  uint8_t evt_msg_len = 0;
  evt_msg[0] = check_installation();
  evt_msg_len = sizeof(evt_msg);
  printk("\n  Device status called for installation.\n And the value is: %d\n\n", evt_msg[0]);
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_DEVICE_STATUS_RESPONSE, INSTALLATION_STATUS,
      INSTALLATION_STATUS_RESP, evt_msg, evt_msg_len);
}

void calibration_status() {
  uint8_t evt_msg[MAX_CALIB_STATUS_STR_LEN] = {0};
  uint8_t evt_msg_len = 0;
  evt_msg[0] = check_calibration();
  evt_msg_len = sizeof(evt_msg);
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_DEVICE_STATUS_RESPONSE, CALIBRATION_STATUS,
      CALIBRATION_STATUS_RESP, evt_msg, evt_msg_len);
}

void em_lock_status() {
  uint8_t evt_msg[MAX_CALIB_STATUS_STR_LEN] = {0};
  uint8_t evt_msg_len = 0;

  if (is_lock_config) {
    evt_msg[0] = 1;
  } else {
    evt_msg[0] = 0;
  }

  evt_msg_len = sizeof(evt_msg);
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_DEVICE_STATUS_RESPONSE, EM_LOCK_STATUS,
      EM_LOCK_STATUS_RESP, evt_msg, evt_msg_len);
}

void operation_status() {
  uint8_t evt_msg[MAX_OP_STATUS_STR_LEN] = {0};
  uint8_t evt_msg_len = 0;
  evt_msg[0] = check_operation_status();
  evt_msg_len = sizeof(evt_msg);
  printk("\n Device status called for operation, and the value is: %d\n", evt_msg[0]);
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_DEVICE_STATUS_RESPONSE, OPERATION_STATUS,
      OPERATION_STATUS_RESP, evt_msg, evt_msg_len);
}

void intent_status() {
  uint8_t evt_msg[MAX_INTENT_STATUS_STR_LEN] = {0};
  uint8_t evt_msg_len = 0;
  // marked red
  // TODO: Need to be implemented
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_DEVICE_STATUS_RESPONSE, INTENT_STATUS_DS,
      INTENT_STATUS_RESP, evt_msg, evt_msg_len);
}

void sensors_status() {
  uint8_t evt_msg[MAX_SENSOR_STATUS_STR_LEN] = {1, 0, 1, 1};
  uint8_t evt_msg_len = 0;
  evt_msg_len = sizeof(evt_msg);
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_DEVICE_STATUS_RESPONSE, SENSORS_STATUS,
      SENSORS_STATUS_RESP, evt_msg, evt_msg_len);
}

void clutch_status() {
  uint8_t evt_msg[MAX_CLUTCH_STATUS_STR_LEN] = {0};
  uint8_t evt_msg_len = 0;
  evt_msg[0] = 1;
  // check_clutch(); //check_clutch function not using in stepper. by default clutch is engaged with value 1
  evt_msg_len = sizeof(evt_msg);
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_DEVICE_STATUS_RESPONSE, CLUTCH_STATUS,
      CLUTCH_STATUS_RESP, evt_msg, evt_msg_len);
}

void sleep_mode() {
  uint8_t evt_msg[MAX_SLEEP_MODE_STR_LEN] = {WIFI_ACTIVE(), Motor_active(), STM_active(), Display_active()};
  uint8_t evt_msg_len = sizeof(evt_msg);
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_DEVICE_STATUS_RESPONSE, SLEEP_MODE,
      SLEEP_MODE_RESP, evt_msg, evt_msg_len);
}

void error_status() {
  uint8_t evt_msg[MAX_ERROR_STATUS_STR_LEN] = {0};
  uint8_t evt_msg_len = sizeof(evt_msg);
  evt_msg[0] = 0;
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_DEVICE_STATUS_RESPONSE, ERROR_STATUS,
      ERROR_STATUS_RESP, evt_msg, evt_msg_len);
}