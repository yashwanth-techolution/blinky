#include <zephyr/kernel.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/sys/printk.h>

#include "Includes/ADO_PIR_Module.h"
#include "Includes/BLE_Module.h"
#include "Includes/Calibration_Module.h"
#include "Includes/Command_Manager_Module.h"
#include "Includes/Device_Status_Module.h"
#include "Includes/Display_Module.h"
#include "Includes/EEPROM_Module.h"
#include "Includes/IMU_Module.h"
#include "Includes/IPC_Manager_Module.h"
#include "Includes/Intent_Module.h"
#include "Includes/Motor_Control_Module.h"
#include "Includes/ADO_Reset_Calib_Module.h"
#include "stdint.h"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <errno.h>
#include <hal/nrf_gpio.h>
#include <inttypes.h>
#include <zephyr/kernel.h>
#include <math.h>
#include <stdio.h>
#include <zephyr/sys/util.h>

#define Valid_calibrated_data 1
// Global ADO Calibration structure
ado_calib_status_t calib;
uint8_t calib_stat_check = ERROR_IN_CALIBRATION;
uint8_t track_calibration_process = 0;
ado_calib_cmd_t calib_cmd = 0;
struct k_mbox_msg recv_msg;
uint8_t calib_buf[10]; // Allocate a 10 bytes buffer to hold calib command data
int8_t calib_command = 0;
bool pending_opr = false;

/*
 * brief: Wrapper Thread to Initiate and execute all calibration Routines
 *        Verifies the calibration command and executes the respective routine.
 */

float Cur_IMU = 0.0f, Prev_IMU = 0.0f, delta = 0.0f;
bool calib_stag = false;

void InitCalibrationProcess() {
  // Mailbox variables
  // struct k_mbox_msg recv_msg;
  // uint8_t calib_buf[10];        // Allocate a 10 bytes buffer to hold calib command data

  // ado_calib_cmd_t calib_cmd = 0;
  char *thread_state = NULL;
  int quit = 0, staginate_count = 0;
  uint8_t lockopen = 3, lock_msg = 0;
  power_optimization(CALIBRATION);

  // Disable PIR
  disable_pir_1();
  uint8_t ado_moving_direction = STOP; // stores the current moving direction of ADO.

  // Check if the intent thread was already running, and suspend it
  thread_state = k_thread_state_str(ado_intent_thrd_id,0,0);

  // debug prints
  printk("Intent Thread is in %s state\n", thread_state);

  if (!is_thread_dead(thread_state)) {
    k_thread_abort(ado_intent_thrd_id);
  }

  while (1) {
    // prepare the mailbox receive buffer
    recv_msg.info = 0;
    recv_msg.size = sizeof(uint8_t) * 10; // Received 10 bytes as it is sufficient for all the calib commands
    recv_msg.rx_source_thread = cmd_mgr_thrd_id;

    if (calib_cmd == 0) {
      k_mbox_get(&cmd_mgr_mb, &recv_msg, calib_buf, MAX_MAILBOX_MSG_RECV_TIMEOUT);
      calib_cmd = calib_buf[0];
    }

    switch (calib_cmd) {
    case CAL_START:
      lock_msg = EM_LOCK_DISABLE;
      Lock_sendCmdACK(&lock_msg, 1);
      calib_cmd = CalibrationStart();
      // calib_cmd = 0;
      calib_stat_check = CALIBRATING_ANDROID;
      track_calibration_process = 0; // IMP: Reset the clibration command to 0 to avoid multiple execution of same command
      Speed(Gear_motor_params.start_RPM, 35, Gear_motor_params.gearbox_ratio, Gear_motor_params.micro_steps,
          Gear_motor_params.step_angle, Gear_motor_params.bw_start_RPM, 35);
      printk("\ncalib_cmd in cal start= %d\n", calib_cmd);

      if (calib_cmd < 0) {
        calib_cmd = 0;
      } else {
        // pending_opr=true;
      }
      break;

    case FORWARD:
      ado_moving_direction = FORWARD;
      // pending_opr=false;
      calib_cmd = Forward();
      printk("\n calib_cmd  in forward= %d\n", calib_cmd);
      // door_refrence= G_IMU;
      Cur_IMU = G_IMU;

      if (calib_cmd < 0) {
        calib_cmd = 0;
      } else {
        // pending_opr=true;
      }
      calib_stag = true;
      break;

    case BACKWARD:
      ado_moving_direction = BACKWARD;
      // pending_opr=false;
      calib_cmd = Backward();
      printk("\n calib_cmd  in backward= %d\n", calib_cmd);
      //   door_refrence= G_IMU;
      Cur_IMU = G_IMU;

      if (calib_cmd < 0) {
        calib_cmd = 0;
      } else {
        // pending_opr=true;
      }
      calib_stag = true;
      break;

    case STOP:
      ado_moving_direction = STOP;
      pending_opr = false;
      printk("STOP Command is detected from ED\r\n");
      calib_cmd = StopMotor();
      if (calib_cmd < 0) {
        calib_cmd = 0;
      }
      // calib_cmd = 0;
      break;

    case REC_CLOSE:
      RecordClosePosition();
      // Update YAW crossing default to false
      calib.is_yaw_crossed = false;
      calib.direction_angle_count = 0;
      calib_cmd = 0;
      break;

    case REC_OPEN:
      // Before recording open call yaw cross function finally
      final_YawCrossCalibration(&calib, ado_moving_direction); // added extra argument to this function
      RecordOpenPosition();
      DirectionCalibration(&calib);
      calib_cmd = 0;
      calib.state = NONE_REC_POS;
      break;

    case REQ_INFO:
      RequestInfo();
      calib_cmd = 0;
      break;

    case SAVE_YAW_DIR:
      SaveYawDirection(calib_buf);
      calib_cmd = 0;
      break;

    case CAL_DONE:
      CalibrationDone();
      Speed(Gear_motor_params.start_RPM, Gear_motor_params.end_RPM, Gear_motor_params.gearbox_ratio, Gear_motor_params.micro_steps, Gear_motor_params.step_angle, Gear_motor_params.bw_start_RPM, Gear_motor_params.bw_end_RPM);
      calib_stat_check = CALIBRATED_ANDROID;
      track_calibration_process = 0;
      calib_cmd = 0;
      calib.state = NONE_REC_POS;
      // TODO: Free any resources before exit in future if any

      // Enable the PIR
      if (is_Intent_Enabled() == true) {
        enable_pir_1();
      }
      quit = 1;
      // TODO: added temp
      reset_variable = 2;
      calib.is_calibrated = true;

      // chg_display_page(DISP_SAY_DOOR_OPEN_PAGE);    //commented by ashok (state change)
      // printk("\n\n-----------------From Calibration --> Say door open ------------\n\n");

      if (!(battery_low)) {
        chg_display_page(DISP_SAY_DOOR_OPEN_PAGE);
      }
      break;

    case CAL_TERMINATE:
      CalibrationTerminate();
      calib_cmd = 0;
      calib_stat_check = UNCALIBRATED_ANDROID;
      calib.state = NONE_REC_POS;
      // TODO: Free any resources before exit in future if any

      // Enable the PIR
      if (is_Intent_Enabled() == true) {
        enable_pir_1();
      }
      quit = 1;
      break;
    }

    // Check the calib states to run in loop
    if (calib.state == CHECK_YAW_CROSSING) {
      YawCrossCalibration(&calib, ado_moving_direction); // added extra argument to this function
    }

    // Reset the received command buffer
    calib_buf[0] = 0; // Since in all the command cases except SAVE_YAW_DIR, we are expecting only first byte so cleaning 1 byte only.
    if (quit) {
      quit = 0;
      break;
    }
  }
  power_optimization(CALIBRATION_STOP);
  // TODO: Clean up the calibration structure when it is written to EEPROM (after interfacing)
}

/*
 * brief: This function should be called in response with CAL_START command from ED
 */
int CalibrationStart(void) {
  printk("CAL_START\r\n");

  // Get the DC Motor State
  if (isMotorRunning(&Gear_motor_params)) {
    // STOP the motor if it is in running condition.
    Stop_Motor(&Gear_motor_params, DRIVE_MOTOR_forward, DRIVE_MOTOR_disable, Disable_TruePWM);
  }

  // Notify  over BLE that the calibration has started
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_CALIBRATION_RESPONSE, CAL_START, CAL_STARTED, NULL, 0);
  // Show Calibration page on display
  // chg_display_page(DISP_CALIBRATION_PAGE);      //commented by ashok state change

  if (!(battery_low)) {
    chg_display_page(DISP_CALIBRATION_PAGE);
  }

  // Keep showing the Calibration page for 1 Sec on display
  // k_sleep(K_MSEC(1000));

  k_mbox_get(&cmd_mgr_mb, &recv_msg, calib_buf, K_SECONDS(1));
  calib_cmd = calib_buf[0];
  printk("calib_cmd :- %d", calib_cmd);

  if ((calib_cmd == STOP) || (calib_cmd == FORWARD) || (calib_cmd == BACKWARD)) {
    printk("\n Stop or forward or backward is called when in calibration start\n");
    return calib_cmd;
  }

  if (!(battery_low)) {
    chg_display_page(DISP_CLOSE_DOOR_PAGE);
  }

  // Set the dir/yaw calibration state to None
  calib.state = NONE_REC_POS;
  return 0;
}

/*
 * brief:This function should be called when a FORWARD command is received from ED
 */
int Forward(void) {
  printk("FORWARD\r\n");
  int res = -1;

  // Get the DC Motor State
  if (isMotorRunning(&Gear_motor_params)) {
    // STOP the motor if it is in running condition for protection from damage specially if it was running backward
    res = Stop_Motor_c(&Gear_motor_params, DRIVE_MOTOR_forward, DRIVE_MOTOR_disable, Disable_TruePWM);
  }

  // Move ADO forward
  res = rotateMotor_c(&Gear_motor_params, DEF_CAL_MOTOR_SPEED_PERCENT, DRIVE_MOTOR_forward,
      DRIVE_MOTOR_enable, Disable_TruePWM);
  printk("\n res=%d in forward\n", res);

  // Notify  over BLE that the bot started moving
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_CALIBRATION_RESPONSE, FORWARD, MOVING_FORWARD, NULL, 0);
  return res; // added by ashok
}

/*
 * brief:This function should be called when a BACKWARD command is received from ED
 */
int Backward(void) {
  int res = -1;
  printk("BACKWARD\r\n");

  // Get the DC Motor State
  if (isMotorRunning(&Gear_motor_params)) {
    // STOP the motor if it is in running condition for protection from damage specially if it was running forward
    Stop_Motor(&Gear_motor_params, DRIVE_MOTOR_backward, DRIVE_MOTOR_disable, Disable_TruePWM);
  }

  // Move ADO backward
  res = rotateMotor_c(&Gear_motor_params, DEF_CAL_MOTOR_SPEED_PERCENT, DRIVE_MOTOR_backward,
      DRIVE_MOTOR_enable, Disable_TruePWM);
  // TODO: Handle any errors
  printk("\n res=%d in backward\n", res);

  // Notify  over BLE that the bot started moving
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_CALIBRATION_RESPONSE, BACKWARD, MOVING_BACKWARD, NULL, 0);
  return res; // added by ashok
}

/*
 * brief:This function should be called in response with STOP command from ED
 */
int StopMotor(void) {
  int res = 0;
  // Debug Prints
  printk("STOP\r\n");

  // Get the DC Motor State
  if (isMotorRunning(&Gear_motor_params)) {
    // STOP the motor if it is in running condition.
    res = motor_lock_o(&Gear_motor_params);
    Stop_Motor_c(&Gear_motor_params, DRIVE_MOTOR_forward, DRIVE_MOTOR_disable, Disable_TruePWM);
    printk("\n res=%d in after motor lock\n", res);

    if (res > 0) {
      return res;
    }
    // TODO: Handle any errors
  }
  printk("\n res=%d after motor lock and stop motor\n", res);
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_CALIBRATION_RESPONSE, STOP, CAL_STOPPED, NULL, 0);
  return res;
}

/*
 * brief: This function should be called in response with REC_OPEN command from ED
 */
void RecordOpenPosition(void) {
  // Debug Prints
  printk("REC_OPEN\r\n");

  // set the temporary IMU value to open position for direction calc
  calib.pos_open = calib.imu_val;

  // Notify over BLE
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_CALIBRATION_RESPONSE, REC_OPEN, OPEN_RECORDED, NULL, 0);

  // Show close door page on display
  // chg_display_page(DISP_CLOSE_DOOR_PAGE);   // commented by ashok (state change )
  // printk("\n------------------ Display from calibration = door close page for calibration -------------------------\n");

  if (!(battery_low)) {
    chg_display_page(DISP_CLOSE_DOOR_PAGE); // added by ashok
  }
  printf("Recorded open: %f\n", calib.pos_open);
  // TODO: Handle errors in Reading IMU if required
}

/*
 * brief: This function should be called when a REC_CLOSE command is received from ED
 */
void RecordClosePosition(void) {
  // Debug Prints
  printk("REC_CLOSE\r\n");

  // Notify Over BLE
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_CALIBRATION_RESPONSE, REC_CLOSE, CLOSE_RECORDED, NULL, 0);
  // printk("\n------------------ Display from calibration = door open page for calibration -------------------------\n");
  // Show record open position page on display
  // chg_display_page(DISP_OPEN_DOOR_PAGE);

  if (!(battery_low)) {
    chg_display_page(DISP_OPEN_DOOR_PAGE);
  }

  // Record the close position in a global structure variable
  // calib.pos_close = IMU_READMedian_2();
  calib.pos_close = G_IMU;
  // TODO: Handle errors in Reading IMU if required

  // Update imu value variable to current position IMU value
  calib.imu_val = calib.pos_close;

  // Debug Prints
  printf("Recorded close: %f\n", calib.pos_close);

  // Start checking if the door movement is crossing 360 degrees
  calib.state = CHECK_YAW_CROSSING;
}

/*
 * brief: This function should be called  when REQ_INFO command is received from ED
 */
void RequestInfo(void) {
  uint8_t evt_msg[10] = {0};
  uint8_t evt_msg_len = 0;

  // Debug Prints
  printk("REQ_INFO\r\n");

  // Prepare the information byte array to notify over BLE
  evt_msg[0] = (int)calib.pos_open >> 8;    // open value msb
  evt_msg[1] = (int)calib.pos_open;         // open value lsb
  evt_msg[2] = (int)calib.pos_close >> 8;   // close value msb
  evt_msg[3] = (int)calib.pos_close;        // close value lsb
  evt_msg[4] = (bool)calib.is_yaw_crossed;  // yaw crossing byte
  evt_msg[5] = (bool)calib.is_direction_cw; // dir byte

  evt_msg_len = 6U; // 6 bytes of extra data has been added in evt_msg

  // Notify Over BLE
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_CALIBRATION_RESPONSE, REQ_INFO, RESP_INFO, evt_msg, evt_msg_len);
  // Set the dir/yaw calibration state to None
  calib.state = NONE_REC_POS;
  // debug print
  printf("Close = %f, Open = %f \n", calib.pos_close, calib.pos_open);
}

/*
 * brief: This function should be called in response with CAL_DONE command from ED
 */
void CalibrationDone(void) {
  printk("CAL_DONE\r\n");
  doorPositionFromClose = calib.close_to_open_angle;
  // TODO: Add the logic to write the recorded calibration values to EEPROM when interfaced
  save_calib_data(); // added by samuel -- saves all calibration related data into eeprom

  // Get the DC Motor State
  if (isMotorRunning(&Gear_motor_params)) {
    // STOP the motor if it is in running condition.
    Stop_Motor(&Gear_motor_params, DRIVE_MOTOR_forward, DRIVE_MOTOR_disable, Disable_TruePWM);
    // TODO: Handle any errors
  }

  // Notify Over BLE
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_CALIBRATION_RESPONSE, CAL_DONE, CAL_COMPLETED, NULL, 0);
  // Show Calibration success page on display
  // chg_display_page(DISP_SUCCESS_PAGE);

  if (!(battery_low)) {
    chg_display_page(DISP_SUCCESS_PAGE);
  }

  // Update the ado state that it is not restarted now
  ADO_set_restart_state(ADO_NOT_RESTARTED);

  // chg_display_page(DISP_SAY_DOOR_OPEN_PAGE);      //commented by ashok (change state)
  // printk("\n\n-----------------From Calibration --> Say door open ------------\n\n");

  if (!(battery_low)) {
    chg_display_page(DISP_SAY_DOOR_OPEN_PAGE); // added by ashok
  }

  // Set the dir/yaw calibration state to None
  calib.state = NONE_REC_POS;
}

/*
 * brief: This function should be called in response with CAL_TERMINATE command from ED
 */
void CalibrationTerminate(void) {
  printk("CAL_TERMINATE\r\n");

  // Get the DC Motor State
  if (isMotorRunning(&Gear_motor_params)) {
    // STOP the motor if it is in running condition.
    Stop_Motor(&Gear_motor_params, DRIVE_MOTOR_forward, DRIVE_MOTOR_disable, Disable_TruePWM);
    // TODO: Handle any errors
  }

  // Clean up the calibrated values so far
  // memset(&calib, 0, sizeof(calib));    // added for future perpose if required
  // TODO load the previous calib from EEPROM if we discard the current calib values

  // Notify Over BLE
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_CALIBRATION_RESPONSE, CAL_TERMINATE, CAL_TERMINATED, NULL, 0);

  // TODO: Show Calibration Failed/Terminated page on display
  // chg_display_page(DISP_START_PAGE);      // commented by ashok (state change)

  if (!(battery_low)) {
    chg_display_page(DISP_START_PAGE); // added by ashok
  }

  // printk("\n-------------------- Display Start page from Calibration Terminate -------------------------\n");
}

/*  New logic for direction find.
 * brief: This function calculates the door direction.
 */
void DirectionCalibration(ado_calib_status_t *calib_status) {
  // check YAW crosed or not
  if (calib_status->is_yaw_crossed == true) {
    if (calib_status->pos_close > calib_status->pos_open) {
      calib_status->is_direction_cw = true;
      calib_status->close_to_open_angle = (360 - (calib_status->pos_close - calib_status->pos_open));
    } else {
      calib_status->is_direction_cw = false;
      calib_status->close_to_open_angle = (360 - (calib_status->pos_open - calib_status->pos_close));
    }
  } else {
    if (calib_status->pos_close < calib_status->pos_open) {
      calib_status->is_direction_cw = true;
      calib_status->close_to_open_angle = (calib_status->pos_open - calib_status->pos_close);
    } else {
      calib_status->is_direction_cw = false;
      calib_status->close_to_open_angle = (calib_status->pos_close - calib_status->pos_open);
    }
  }

  // debug print
  // printk("count ----> %d \n", calib_status->direction_angle_count);

  if (calib_status->direction_angle_count > 0) {
    calib_status->close_to_open_motor_direction = MOTOR_CW;
    // debug print
    printk("-------> Predicted FORWARD\n");
  } else if (calib_status->direction_angle_count < 0) {
    calib_status->close_to_open_motor_direction = MOTOR_ACW;
    // debug print
    printk("-------> Predicted BACKWARD\n");
  }
}

/*
 * brief: This function marks yaw crossing condition.
 */
void YawCrossCalibration(ado_calib_status_t *calib_status, uint8_t direction) {
  float IMU_Prev = 0;
  float IMU_Curr = 0;
  float IMU_abs = 0;

  // get the previously read IMU sensor median value
  IMU_Prev = calib_status->imu_val;

  // read the current IMU value
  IMU_Curr = G_IMU;

  IMU_abs = abs(IMU_Prev - IMU_Curr);

  if (IMU_abs >= 180) {
    // Toggle YAW cross when it get 90 degrees difference
    // check previous YAW cross condition
    if (calib_status->is_yaw_crossed == true) {
      calib_status->is_yaw_crossed = false;

      // Debug prints
      printk("Yaw Not crossed\n");
    } else {
      calib_status->is_yaw_crossed = true;

      // Debug prints
      printk("Yaw crossed\n");
    }
  } else { // extra logic added by samuel -> it will increments or dicrements the angle count value depends on forward and backward commands
    // printk("count ----> %d \n", calib_status->direction_angle_count);
    if (direction == FORWARD) {
      calib_status->direction_angle_count = calib_status->direction_angle_count + IMU_abs;
    } else if (direction == BACKWARD) {
      calib_status->direction_angle_count = calib_status->direction_angle_count - IMU_abs;
    }
  }
  // update the global variable
  calib_status->imu_val = IMU_Curr;
}

/*
 * brief: This function marks yaw crossing condition.
 */
void final_YawCrossCalibration(ado_calib_status_t *calib_status, uint8_t direction) {
  float IMU_Prev = 0;
  float IMU_Curr = 0;
  float IMU_abs = 0;

  // get the previously read IMU sensor median value
  IMU_Prev = calib_status->imu_val;

  // read the current IMU value
  IMU_Curr = G_IMU;
  IMU_abs = abs(IMU_Prev - IMU_Curr);

  if (IMU_abs >= 180) {
    // Toggle YAW cross when it get 90 degrees difference
    // check previous YAW cross condition
    if (calib_status->is_yaw_crossed == true) {
      calib_status->is_yaw_crossed = false;

      // Debug prints
      printk("Yaw Not crossed\n");
    } else {
      calib_status->is_yaw_crossed = true;

      // Debug prints
      printk("Yaw crossed\n");
    }
  } else { // extra logic added by samuel -> it will increments or dicrements the angle count value depends on forward and backward commands
    // printk("count ----> %d \n", calib_status->direction_angle_count);
    if (direction == FORWARD) {
      calib_status->direction_angle_count = calib_status->direction_angle_count + IMU_abs;
    } else if (direction == BACKWARD) {
      calib_status->direction_angle_count = calib_status->direction_angle_count - IMU_abs;
    }
  }
  // update the global variable
  calib_status->imu_val = IMU_Curr;
}

/*
 * brief: This function receives the yaw and driection from ED and overwrites ADO memory.
 */
void SaveYawDirection(uint8_t *calib_data) {
  // Set the yaw crossed and direction flags as received from ED
  calib.is_yaw_crossed = (bool)calib_data[1];
  calib.is_direction_cw = (bool)calib_data[2];

  // Notify over BLE that the recived values are saved
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_CALIBRATION_RESPONSE, SAVE_YAW_DIR, SAVED_YAW_DIR, NULL, 0);

  // Set the dir/yaw calibration state to None
  calib.state = NONE_REC_POS;
}

/*
 * brief: This function writes the calibrated params to ADO EEPROM memory.
 */
void save_calib_data() {
  int error = 0;
  uint8_t data[16];
  data[0] = Valid_calibrated_data;
  memcpy(&data[1], (unsigned char *)(&calib.pos_close), sizeof(calib.pos_close));
  // data[1] = ((int)calib.pos_close >> 24) & 0xFF;
  // data[2] = ((int)calib.pos_close >> 16) & 0xFF;
  // data[3] = ((int)calib.pos_close >> 8) & 0xFF;
  // data[4] = (int)calib.pos_close & 0xFF;
  memcpy(&data[5], (unsigned char *)(&calib.pos_open), sizeof(calib.pos_open));
  // data[5] = ((int)calib.pos_open >> 24) & 0xFF;
  // data[6] = ((int)calib.pos_open >> 16) & 0xFF;
  // data[7] = ((int)calib.pos_open >> 8) & 0xFF;
  // data[8] = (int)calib.pos_open & 0xFF;
  memcpy(&data[9], (unsigned char *)(&calib.close_to_open_angle), sizeof(calib.close_to_open_angle));
  // data[9] = ((int)calib.close_to_open_angle >> 24) & 0xFF;
  // data[10] = ((int)calib.close_to_open_angle >> 16) & 0xFF;
  // data[11] = ((int)calib.close_to_open_angle >> 8) & 0xFF;
  // data[12] = (int)calib.close_to_open_angle & 0xFF;

  // TODO: Validate the calibration values before writing to EEPROM, return error otherwise

  // data[1] = ((int)calib.pos_close >> 24) & 0xFF;
  // data[2] = ((int)calib.pos_close >> 16) & 0xFF;
  // data[3] = ((int)calib.pos_close >> 8) & 0xFF;
  // data[4] = (int)calib.pos_close & 0xFF;
  // data[5] = ((int)calib.pos_open >> 24) & 0xFF;
  // data[6] = ((int)calib.pos_open >> 16) & 0xFF;
  // data[7] = ((int)calib.pos_open >> 8) & 0xFF;
  // data[8] = (int)calib.pos_open & 0xFF;
  // data[9] = ((int)calib.close_to_open_angle >> 24) & 0xFF;
  // data[10] = ((int)calib.close_to_open_angle >> 16) & 0xFF;
  // data[11] = ((int)calib.close_to_open_angle >> 8) & 0xFF;
  // data[12] = (int)calib.close_to_open_angle & 0xFF;
  data[13] = calib.is_direction_cw ? 1 : 0;
  data[14] = calib.is_yaw_crossed ? 1 : 0;
  data[15] = calib.close_to_open_motor_direction ? 1 : 0;

  error = Write_EEPROM(CALIBRATION_MEMORY_LOCATION, &data[0], 16);
  // TODO: Error Handling if EEPROM Write fails
  if (error) {
    printk("error saving calib data to EEPROM trying one more time");
    error = Write_EEPROM(CALIBRATION_MEMORY_LOCATION, &data[0], 16);
  }
}

/*
 * brief: This function loads the saved calibration params from ADO EEPROM memory.
 */
int8_t load_calib_data() {
  int error = 0;
  uint8_t data[16] = {0};

  error = Read_EEPROM(CALIBRATION_MEMORY_LOCATION, &data[0], 16);
  if (error) {
    printk("error loading calib data from EEPROM trying one more time");
    error = Read_EEPROM(CALIBRATION_MEMORY_LOCATION, &data[0], 16);
  }

  if (data[0] == 1) {
    calib.is_calibrated = true;
    memcpy(&calib.pos_close, &data[1], sizeof(calib.pos_close));
    // calib.pos_close = (data[1] << 24) | (data[2] << 18) | (data[3] << 8) | data[4];
    memcpy(&calib.pos_open, &data[5], sizeof(calib.pos_open));
    // calib.pos_open = (data[5] << 24) | (data[6] << 18) | (data[7] << 8) | data[8];
    memcpy(&calib.close_to_open_angle, &data[9], sizeof(calib.close_to_open_angle));
    // calib.close_to_open_angle = (data[9] << 24) | (data[10] << 18) | (data[11] << :sunglasses: | data[12];

    // calib.pos_close = (data[1] << 24) | (data[2] << 18) | (data[3] << 8) | data[4];
    // calib.pos_open = (data[5] << 24) | (data[6] << 18) | (data[7] << 8) | data[8];
    // calib.close_to_open_angle = (data[9] << 24) | (data[10] << 18) | (data[11] << 8) | data[12];
    calib.is_direction_cw = data[13] ? true : false;
    calib.is_yaw_crossed = data[14] ? true : false;
    calib.close_to_open_motor_direction = data[15] ? MOTOR_CW : MOTOR_ACW;
  } else {
    calib.is_calibrated = false;
    printk("Door bot not calibrated\n");
    return -1;
  }

  // Debug print
  printk("Close IMU ------> %f\n", calib.pos_close);
  // Debug print
  printk("Open IMU ------> %f\n", calib.pos_open);
  printk("Open Angle ------> %f\n", calib.close_to_open_angle);
  return error;
}

/*
 * brief: This function Checks if the ADO is calibrated
 */
bool is_ADO_Calibrated(ado_calib_status_t *pCalib) {
  // Input check
  if (pCalib != NULL) {
    return pCalib->is_calibrated;
  }
  return false;
}

uint8_t check_calibration() {
  if (calib_stat_check == CALIBRATING_ANDROID) {
    return calib_stat_check;
  } else
    return is_ADO_Calibrated(&calib);
  // check for possibilities of error
}

/*
 * brief: This function is for invalidating ADO's calibrated values.
 */
void erase_Calibration_data() {
  int error = 0;
  uint8_t data[16];
  data[0] = 0;
  memcpy(&data[1], (unsigned char *)(&calib.pos_close), sizeof(calib.pos_close));
  memcpy(&data[5], (unsigned char *)(&calib.pos_open), sizeof(calib.pos_open));
  memcpy(&data[9], (unsigned char *)(&calib.close_to_open_angle), sizeof(calib.close_to_open_angle));

  data[13] = calib.is_direction_cw ? 1 : 0;
  data[14] = calib.is_yaw_crossed ? 1 : 0;
  data[15] = calib.close_to_open_motor_direction ? 1 : 0;

  error = Write_EEPROM(CALIBRATION_MEMORY_LOCATION, &data[0], 16);
  // TODO: Error Handling if EEPROM Write fails
  if (error) {
    printk("error saving calib data to EEPROM trying one more time");
    error = Write_EEPROM(CALIBRATION_MEMORY_LOCATION, &data[0], 16);
  }
  calib.is_calibrated = false; // invalidate calibration status flag
  reset_variable = 0;          // invalidate reset calibration variable data.

  // Update the ado state that it is not restarted now
  ADO_set_restart_state(ADO_NOT_RESTARTED);
}

void reset_accelation_values() {
  printk("\n===>reset_accelation_values called");
  Gear_motor_params.current_interval = 0;
  Gear_motor_params.current_speed_percentage = 0;
  Gear_motor_params.steps = 0;
  Gear_motor_params.skip_steps = 250;
  Gear_motor_params.speed_change_percentage = 10;
  Gear_motor_params.bw_current_interval = 0;
  Gear_motor_params.bw_current_speed_percentage = 0;
  Gear_motor_params.bw_steps = 0;
  Gear_motor_params.bw_skip_steps = 150;             // 250;// 325;
  Gear_motor_params.bw_speed_change_percentage = 12; // 10;
}

float fw_accrelation_interval(void) {
  // printk("Forward acceleration interval is called\r\n");
  if ((Gear_motor_params.steps % Gear_motor_params.skip_steps) == 0) {
    if (Gear_motor_params.current_speed_percentage >= 60 && Gear_motor_params.current_speed_percentage <= 100) {
      Gear_motor_params.skip_steps = 250;
      Gear_motor_params.speed_change_percentage = 1;
    }

    if (Gear_motor_params.current_speed_percentage < 100) {
      // printk("current_speed_percentage = %d speed_change_percentage %d",Gear_motor_params.current_speed_percentage,Gear_motor_params.speed_change_percentage);
      Gear_motor_params.current_speed_percentage += Gear_motor_params.speed_change_percentage;
      Gear_motor_params.current_interval = Gear_motor_params.start_interval - ((Gear_motor_params.start_interval - Gear_motor_params.end_interval) * Gear_motor_params.current_speed_percentage / 100);
      // printk("\n  start_interval=%d end_interval=%d current_speed_percentage=%d",Gear_motor_params.current_interval,Gear_motor_params.start_interval,Gear_motor_params.end_interval,Gear_motor_params.current_speed_percentage);
    } else {
      printk("**************************\n");
      printk("%d", Gear_motor_params.current_interval);
    }

    if (Gear_motor_params.steps >= Gear_motor_params.skip_steps * 20) {
      Gear_motor_params.steps = 0;
    }
  }
  Gear_motor_params.steps += 1;
  // printk("\nsteps = %d",Gear_motor_params.steps);
  return Gear_motor_params.current_interval;
}

float bw_accrelation_interval(void) {
  // printk("backward acceleration interval is called\r\n");
  if ((Gear_motor_params.bw_steps % Gear_motor_params.bw_skip_steps) == 0) {
    if (Gear_motor_params.bw_current_speed_percentage >= 60 && Gear_motor_params.bw_current_speed_percentage <= 100) {
      Gear_motor_params.bw_skip_steps = 250;
      Gear_motor_params.bw_speed_change_percentage = 1;
    }
    
    if (Gear_motor_params.bw_current_speed_percentage < 100) {
      // printk("current_speed_percentage = %d speed_change_percentage %d",Gear_motor_params.bw_current_speed_percentage,Gear_motor_params.bw_speed_change_percentage);
      Gear_motor_params.bw_current_speed_percentage += Gear_motor_params.bw_speed_change_percentage;
      Gear_motor_params.bw_current_interval = Gear_motor_params.bw_start_interval - ((Gear_motor_params.bw_start_interval - Gear_motor_params.bw_end_interval) * Gear_motor_params.bw_current_speed_percentage / 100);
      // printk("\n  start_interval=%d end_interval=%d current_speed_percentage=%d",Gear_motor_params.bw_current_interval,Gear_motor_params.bw_start_interval,Gear_motor_params.bw_end_interval,Gear_motor_params.bw_current_speed_percentage);
    } else {
      printk("**************************\n");
      printk("%d", Gear_motor_params.bw_current_interval);
    }

    if (Gear_motor_params.bw_steps >= Gear_motor_params.bw_skip_steps * 20) {
      Gear_motor_params.bw_steps = 0;
    }
  }
  Gear_motor_params.bw_steps += 1;
  // printk("\nsteps = %d",Gear_motor_params.bw_steps);
  return Gear_motor_params.bw_current_interval;
}

void deceleration_interval(struct motor_params_t *motor_params, bool dir) {
  // printk("deceleration is called\r\n");
  int t = 0, ret = 0;
  bool flag = true;
  struct k_mbox_msg recv_msg;
  uint8_t calib_buf[10]; // Allocate a 10 bytes buffer to hold calib command data
                         // ado_calib_cmd_t calib_cmd = 0;
  recv_msg.info = 0;
  recv_msg.size = sizeof(uint8_t) * 10; // Received 10 bytes as it is sufficient for all the calib commands
  recv_msg.rx_source_thread = cmd_mgr_thrd_id;

  while (flag) {
    k_mbox_get(&cmd_mgr_mb, &recv_msg, calib_buf, K_MSEC(50));
    calib_cmd = calib_buf[0];

    if ((calib_cmd == STOP)) {
      break;
    }

    if (dir == true) {
      // printk("\n==current_interval= %d",Gear_motor_params.current_interval);
      t += 1;
      if (t % 250 == 0) {
        Gear_motor_params.current_interval += 5; ////// deceleration adjust value backward///////////////
      }
      ret = pwm_pin_set_usec(motor_params->pwm, motor_params->settings.pwm_channel, Gear_motor_params.current_interval, Gear_motor_params.current_interval / 2U, motor_params->settings.pwm_flags);

      if (ret != 0) {
        printk("Error %d: failed to set pulse width\n", ret);
      }

      unsigned long int time_stamp = k_uptime_get();

      while (((k_uptime_get() - time_stamp) <= Gear_motor_params.current_interval)) {
        k_mbox_get(&cmd_mgr_mb, &recv_msg, calib_buf, K_USEC(Gear_motor_params.current_interval));
        calib_cmd = calib_buf[0];

        if ((calib_cmd == STOP)) {
          return calib_cmd;
        }
      }

      if (Gear_motor_params.current_interval >= Gear_motor_params.decel_stop_interval || door_close_to_open_percentage() >= 99) {
        printk("\n==break current_interval= %d", Gear_motor_params.current_interval);
        flag = false;
        break;
      }
    } else {
      t += 1;
      if (t % 250 == 0) {
        Gear_motor_params.bw_current_interval += 5; ////// deceleration adjust value backward///////////////
      }
      ret = pwm_pin_set_usec(motor_params->pwm, motor_params->settings.pwm_channel, Gear_motor_params.bw_current_interval, Gear_motor_params.bw_current_interval / 2U, motor_params->settings.pwm_flags);
      
      if (ret != 0) {
        printk("Error %d: failed to set pulse width\n", ret);
      }

      unsigned long int time_stamp = k_uptime_get();

      while (((k_uptime_get() - time_stamp) <= Gear_motor_params.bw_current_interval)) {
        k_mbox_get(&cmd_mgr_mb, &recv_msg, calib_buf, K_USEC(Gear_motor_params.bw_current_interval));
        calib_cmd = calib_buf[0];

        if ((calib_cmd == STOP)) {
          return calib_cmd;
        }
      }

      // door_close_to_open_percentage();

      // door_percentage = 100 - door_percentage;
      if (Gear_motor_params.bw_current_interval >= Gear_motor_params.bw_decel_stop_interval || (100 - door_close_to_open_percentage()) >= 99) {
        printk("\n==break current_interval= %d", Gear_motor_params.bw_current_interval);
        flag = false;
        break;
      }
    }
  }
}

/*
 * brief: Rotate or Stop the motor as per the provided parameters, the motor_param must be valid
 */
int rotateMotor_c(struct motor_params_t *motor_params, uint8_t pwm_percent, bool dir, bool enable, bool true_pwm) {
  // int32_t ret = 0, u_sec = 0, cnt = 0; // temp
  printk("\n rotate motor_c is called\n");
  int32_t u_sec = 0, cnt = 0; // temp
  int ret = 0;
  // Limit PWM percent to 100%

  if (pwm_percent > MAX_OPR_MOTOR_SPEED_PERCENT) {
    pwm_percent = MAX_OPR_MOTOR_SPEED_PERCENT;
  }

  // Limit PWM percent to not fall beyond MIN_OPR_MOTOR_SPEED_PERCENT in operating conditions
  if (!true_pwm) {
    if (pwm_percent < MIN_OPR_MOTOR_SPEED_PERCENT)
      pwm_percent = MIN_OPR_MOTOR_SPEED_PERCENT;
  }

  // Set the motor state
  // since the DC door motor enable pin is set to active low,
  // we are tasking opposite state to depict motor running
  // motor_params->isRunning = !enable;

  // Toggle the PWM duty cycle percent to ON timings i.e. 70% Pwm eq to 30% ON time
  pwm_percent = 100U - pwm_percent;

  if (motor_params->type == CLAMP_MOTOR) {
    motor_params->isRunning = enable;
    enable = !enable;
    dir = !dir;

    // Set the Motor Enable GPIO
    gpio_pin_set(motor_params->enable, motor_params->settings.en_pin, enable);

    // Set the Motor direction GPIO
    gpio_pin_set(motor_params->direction, motor_params->settings.dir_pin, dir);
  } else {
    // Set the Motor Enable GPIO
    motor_params->isRunning = !enable;
    if (motor_params->isRunning) {
      gpio_pin_set(motor_params->enable, motor_params->settings.en_pin, 0);
    } else {
      gpio_pin_set(motor_params->enable, motor_params->settings.en_pin, 1);
    }

    if (enable == false) {
      // Set the PWM Period
      // TODO: Add the functionality of turning off PWM if motor is disabled
      if (dir) {
        ret = gpio_pin_set(motor_params->direction, motor_params->settings.dir_pin, GPIO_ACTIVE_HIGH);
        if (ret != 0) {
          printk("Error %d: failed to set  direction\n", ret);
        }

        while (Gear_motor_params.current_speed_percentage < 100) { // 11825 cnt <= 13325
          u_sec = fw_accrelation_interval();
          // printf("\nforward speed = %d direction = %d msec = %f enable = %d\n",pwm_percent,dir,u_sec,enable);
          ret = pwm_pin_set_usec(motor_params->pwm, motor_params->settings.pwm_channel, u_sec, u_sec / 2U, motor_params->settings.pwm_flags);
          if (ret != 0) {
            printk("Error %d: failed to set pulse width\n", ret);
          }

          k_mbox_get(&cmd_mgr_mb, &recv_msg, calib_buf, K_USEC(u_sec));
          calib_cmd = calib_buf[0];

          if ((calib_cmd == STOP) || (calib_cmd == FORWARD) || (calib_cmd == BACKWARD)) {
            printk("\nSTOP or forward or backward Command is detected in rotate motor\n");
            printk("calib_cmd = %d", calib_cmd);
            return calib_cmd;
          }
        }
      } else {
        ret = gpio_pin_set(motor_params->direction, motor_params->settings.dir_pin, GPIO_ACTIVE_LOW);
        if (ret != 0) {
          printk("Error %d: failed to set  direction\n", ret);
        }

        while (Gear_motor_params.bw_current_speed_percentage < 100) { // 9000 cnt <= 13325
          u_sec = bw_accrelation_interval();
          // printk("\nbackward speed = %d direction = %d msec = %d enable = %d\n", pwm_percent, dir, u_sec, enable);
          ret = pwm_pin_set_usec(motor_params->pwm, motor_params->settings.pwm_channel, u_sec, u_sec / 2U, motor_params->settings.pwm_flags);
          if (ret != 0) {
            printk("Error %d: failed to set pulse width\n", ret);
          }

          k_mbox_get(&cmd_mgr_mb, &recv_msg, calib_buf, K_USEC(u_sec));
          calib_cmd = calib_buf[0];

          if ((calib_cmd == STOP) || (calib_cmd == FORWARD) || (calib_cmd == BACKWARD)) {
            printk("\nSTOP or forward or backward Command is detected in rotate motor\n");
            printk("calib_cmd = %d", calib_cmd);
            return calib_cmd;
          }
        }
      }
    }
    printk("\n============>enable = %d isRunning = %d\n", enable, motor_params->isRunning);
  }
  return 0;
}

int Stop_Motor_c(struct motor_params_t *motor_params, bool dir, bool enable_disable, bool true_PWM) {
  int ret;
  printk("\n ===> Stop_Motor called");
  reset_accelation_values();
  ret = rotateMotor_c(motor_params, PERCENTAGE_ToStop_Motor, dir, enable_disable, true_PWM);
  return ret;
}