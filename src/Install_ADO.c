#include <zephyr/kernel.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/printk.h>

#include "Includes/ADO_PIR_Module.h"
#include "Includes/AnalogIn.h"
#include "Includes/Calibration_Module.h"
#include "Includes/Command_Manager_Module.h"
#include "Includes/Display_Module.h"
#include "Includes/EEPROM_Module.h"
#include "Includes/IPC_Manager_Module.h"
#include "Includes/Install_ADO.h"
#include "Includes/Intent_Module.h"
#include "Includes/Motor_Control_Module.h"

// Global variables for event update.
ado_event_status_t evt_status;

// timer variable
struct k_timer instalation_motors_timer;
bool ins_motors_timer_expired_flag = false;

uint8_t check_installation();
uint8_t installation_status_check;
uint8_t pb_status;

// ADO installation status to check and suppress any unwanted
// installation commands if it is already installed
// TODO: to store/read back from EEPROM while in operation
uint8_t ado_inst_state = 0; // For now, initialise it to 0 to depict 'installation not done'

// Global structure initalization for storing installation clamp related data.
// Initialising it to the values ASSUMING device in FULLY Clamped position
struct auto_clamp_data clamp_data = {
        .hor_pos = 1, // 1=hor clamped, 2=hor unclamped, 0= inermediate
        .ver_pos = 2, // 1=ver clamped, 2=ver unclamped, 0= inermediate
        .hor_per = 100,
        .ver_per = 100,
        .state_pos = 1, // 1=auto clamped, 2= auto unclamped
        .p_balancer = A_POSITION,
        .p_balancer_per = 0,
        .run_pos = 0,
        .response = 0
};

/***** all functions declarations *****/

void horizontal_clamp(uint8_t command);
void horizontal_unclamp(uint8_t command);
void vertical_clamp(uint8_t command);
void vertical_unclamp(uint8_t command);
uint8_t auto_clamp(void);
uint8_t auto_unclamp(void);

int current_offset(int channel);

// global variable to hold current sensor offset voltage
int CUR_SENSOR_CUT_OFF_V;

uint8_t ado_install_uninstall(void) {
  char lock_msg = 0;
  // Mailbox variables
  struct k_mbox_msg recv_msg;

  char *thread_state = NULL;
  power_optimization(INSTALLATION);

  // Disable PIR
  disable_pir_1();

  // Check if the intent thread was already running, and suspend it
  thread_state = k_thread_state_str(ado_intent_thrd_id,0,0);

  // debug prints
  printk("Intent Thread is in %s state\n", thread_state);

  if (!is_thread_dead(thread_state)) {
    k_thread_abort(ado_intent_thrd_id);
  }

  // init timers
  printk("++++++++++++++");
  k_timer_init(&instalation_motors_timer, clamp_motors_timer_expire, NULL);

  // Installation thread.
  while (clamp_data.run_pos != INS_COMPLETE) {
    // prepare the mailbox receive buffer
    recv_msg.info = 0;
    recv_msg.size = sizeof(clamp_data.run_pos);
    recv_msg.rx_source_thread = cmd_mgr_thrd_id;

    // retrieve and delete the message if received
    if (clamp_data.run_pos == INS_IDLE) {
      k_mbox_get(&cmd_mgr_mb, &recv_msg, &clamp_data.run_pos, K_FOREVER);
      printk("received ins_cmd %d\n", clamp_data.run_pos);
    } else {
      printk("skipped reading ins_cmd %d\n", clamp_data.run_pos);
    }

    switch (clamp_data.run_pos) {
    case INS_START:
      lock_msg = EM_LOCK_DISABLE;
      Lock_sendCmdACK(&lock_msg, 1);
      // invalidate the ADO installation state
      // clamp_data.state_pos=ADO_UNINSTALLED;
      //  optional
      if ((clamp_data.p_balancer == A_POSITION) && ((clamp_data.ver_pos == ADO_UNINSTALLED)
            || (clamp_data.state_pos == ADO_INSTALLED))) {
        product_balancer(G_POSITION); // temparly commented for android.
      }

      // 1. Prepare the INS_STARTED response and Notify over BLE
      ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE, INS_START,
          INS_STARTED, NULL, 0);
      clamp_data.run_pos = INS_IDLE;

      // Debug prints
      printk("Installation started\r\n");

      // Show installation page on display
      // chg_display_page(DISP_ENTER_SETUP_PAGE);

      if (!(battery_low)) {
        chg_display_page(DISP_ENTER_SETUP_PAGE);
      }
      break;

    case AUTO_CLAMP: // auto clamping start.
      // Check if already auto clamped
      if (clamp_data.state_pos == 1) {
        // Auto clamp already done, notify this over BLE
        ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE, AUTO_CLAMP,
            CLAMPED_VER, NULL, 0);
        k_sleep(K_MSEC(200)); // Some delay in between two consecutive event notifications
        ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE, AUTO_CLAMP,
            CLAMPED_HOR, NULL, 0);
        // Debug prints
        printk("Already Auto-Clamped\r\n");
        clamp_data.run_pos = INS_IDLE;
        break;
      }
      printk("AUTO_CLAMP Started..\r\n");
      CUR_SENSOR_CUT_OFF_V = current_offset(CLAMP_AND_MICRO_MOTOR_CURRENT_SENSOR);
      clamp_data.run_pos = auto_clamp();
      save_install_data(); // save installation data into eeprom
      break;

    case AUTO_UNCLAMP: // auto unclamping start.
      // Check if already auto unclamped
      if (clamp_data.state_pos == 2) {
        // Auto unclamp already done, notify this over BLE
        ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE, AUTO_UNCLAMP,
            UNCLAMPED_HOR, NULL, 0);
        k_sleep(K_MSEC(200)); // Some delay in between two consecutive event notifications
        ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE, AUTO_UNCLAMP,
            UNCLAMPED_VER, NULL, 0);

        // Debug prints
        printk("Already Auto-Unclamped\r\n");
        clamp_data.run_pos = INS_IDLE;
        break;
      }
      printk("AUTO_UNCLAMP Started..\r\n");
      CUR_SENSOR_CUT_OFF_V = current_offset(CLAMP_AND_MICRO_MOTOR_CURRENT_SENSOR);
      clamp_data.run_pos = auto_unclamp();
      save_install_data(); // save installation data into eeprom
      break;

    case MAN_HOR_CLAMP: // Manual horizontal clamp control
      if (clamp_data.hor_pos == 1) {
        ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE,
            MAN_HOR_CLAMP, CLAMPED_HOR, NULL, 0);

        // Debug prints
        printk("Already Clamped\r\n");
        clamp_data.run_pos = INS_IDLE;
        break;
      }

      printk("MANUAL_HORIZONTAL_CLAMP\r\n");
      CUR_SENSOR_CUT_OFF_V = current_offset(CLAMP_AND_MICRO_MOTOR_CURRENT_SENSOR);
      horizontal_clamp(MAN_HOR_CLAMP);
      break;

    case MAN_HOR_UNCLAMP: // Manual horizontal unclamp control
      if (clamp_data.hor_pos == 2) {
        ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE,
            MAN_HOR_UNCLAMP, UNCLAMPED_HOR, NULL, 0);

        // Debug prints
        printk("Already UnClamped\r\n");
        clamp_data.run_pos = INS_IDLE;
        break;
      }
      printk("MANUAL_HORIZONTAL_UN-CLAMP\r\n");

      CUR_SENSOR_CUT_OFF_V = current_offset(CLAMP_AND_MICRO_MOTOR_CURRENT_SENSOR);
      horizontal_unclamp(MAN_HOR_UNCLAMP);
      break;

    case MAN_VER_CLAMP: // Manual vertical clamp control
      if (clamp_data.ver_pos == 1) {
        ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE,
            MAN_VER_CLAMP, CLAMPED_VER, NULL, 0);

        // Debug prints
        printk("Already Clamped\r\n");
        clamp_data.run_pos = INS_IDLE;
        break;
      }
      printk("MANUAL_VERTICAL_CLAMP\r\n");

      CUR_SENSOR_CUT_OFF_V = current_offset(CLAMP_AND_MICRO_MOTOR_CURRENT_SENSOR);
      vertical_clamp(MAN_VER_CLAMP);
      break;

    case MAN_VER_UNCLAMP: // Manual vertical unclamp control
      if (clamp_data.ver_pos == 2) {
        ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE,
            MAN_VER_UNCLAMP, UNCLAMPED_VER, NULL, 0);

        // Debug prints
        printk("Already UnClamped\r\n");
        clamp_data.run_pos = INS_IDLE;
        break;
      }
      printk("MANUAL_VERTICAL_UN-CLAMP\r\n");

      CUR_SENSOR_CUT_OFF_V = current_offset(CLAMP_AND_MICRO_MOTOR_CURRENT_SENSOR);
      vertical_unclamp(MAN_VER_UNCLAMP);
      break;

    case STOP_CLAMP:
      // 1. Prepare the INS_STOP response and Notify over BLE
      ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE, STOP_CLAMP,
          CLAMP_STOPPED, NULL, 0);
      clamp_data.run_pos = INS_IDLE;

      // Debug prints
      printk("Clamp stopped\r\n");

      // save_install_data();  // save installation data into eeprom
      break;

    case PRODUCT_BALANCER_UP:
      if (clamp_data.p_balancer != A_POSITION) {
        product_balancer(A_POSITION);
      }
      clamp_data.run_pos = INS_IDLE;
      break;

    case PRODUCT_BALANCER_DOWN:
      if (clamp_data.p_balancer != G_POSITION) {
        product_balancer(G_POSITION);
      }
      clamp_data.run_pos = INS_IDLE;
      break;

    default:
      break;
    }

    if ((clamp_data.ver_pos == 1) && (clamp_data.hor_pos == 1)) {
      // Cheking clamp based on manual control
      clamp_data.state_pos = ADO_INSTALLED;           // ADO installed
    } else if ((clamp_data.ver_pos == 2) && (clamp_data.hor_pos == 2)) {
      // Cheking unclamp based on manual control
      clamp_data.state_pos = ADO_UNINSTALLED;
    } else {
      clamp_data.state_pos = 0; // state position
    }
  }

  // optionally product balancer code added here, later need to modify if required.
  if ((clamp_data.state_pos == 1) || (clamp_data.ver_pos == 2)) {
    // optional
    if (clamp_data.p_balancer != A_POSITION) {
      product_balancer(A_POSITION); // temapararly commented for android
    }
  }

  // Notify over BLE that installation is now complete.
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE, INS_COMPLETE, INS_COMPLETED, NULL, 0);

  // TODO: Store any relevant information to EEPROM
  save_install_data(); // added by samuel --> save installation data into eeprom

  clamp_data.run_pos = 0;
  power_optimization(INSTALLATION_STOP);
  return clamp_data.response;
}

/*
 * brief: This function receives a percentage of traveled data of respected stepper motor in
    respected direction. adds or subtractes this percent into a global structure of respected
    motor percentage variable.
 */
uint8_t present_pos(uint8_t motor_state, int i) { // percentage calculation for two stepper motors.
  switch (motor_state) {
  case MAN_HOR_CLAMP:
    clamp_data.hor_per += i;
    printk("------------------------------------------------HORIZONTAL Position = %d %% \r\n",
        clamp_data.hor_per);
    return clamp_data.hor_per;
    break;

  case MAN_HOR_UNCLAMP:
    clamp_data.hor_per -= i;
    printk("------------------------------------------------HORIZONTAL Position = %d %% \r\n",
        clamp_data.hor_per);
    return clamp_data.hor_per;
    break;

  case MAN_VER_CLAMP:
    clamp_data.ver_per += i;
    printk("------------------------------------------------VERTICAL Position %d %% \r\n",
        clamp_data.ver_per);
    return clamp_data.ver_per;
    break;

  case MAN_VER_UNCLAMP:
    clamp_data.ver_per -= i;
    printk("------------------------------------------------VERTICAL Position %d %% \r\n",
        clamp_data.ver_per);
    return clamp_data.ver_per;
    break;

  default:
    break;
  }
  return 0;
}

/*
 * brief: This function takes respected ADC channel number.
 *        returns calculated average current data as offset to in float.
 */
int current_offset(int channel) {
  double raw_voltage = 0;
  for (int i = 0; i < NO_OFF_ADC_SAMPLES; i++) {
    raw_voltage += AnalogRead(channel);
  }
  int avg_voltage = (raw_voltage / NO_OFF_ADC_SAMPLES) * 1000;
  printk("--------------voltage = %d -----------\n", avg_voltage);
  return avg_voltage;
}

/*
 * brief: This function takes respected ADC channel number.
 *        returns calculated average current data in float.
 */
float current_in_amp(int channel) {
  double raw_voltage = 0;
  for (int i = 0; i < NO_OFF_ADC_SAMPLES; i++) {
    raw_voltage += AnalogRead(channel);
  }
  int avg_voltage = (raw_voltage / NO_OFF_ADC_SAMPLES) * 1000;
  float current_amp = ((avg_voltage - CUR_SENSOR_CUT_OFF_V) * 2.5) / 1000;
  // printk("voltage = %d\n", avg_voltage);
  return current_amp;
}

/*
 * brief: This function called by direct manual control or auto clamp control.
 *        and controls horizontal motor in clamp direction and stores clamp state in to global variable.
 */
void horizontal_clamp(uint8_t command) {
  // Buffer to hold extra bytes for notifying percentage values
  uint8_t evt_msg[2];
  uint8_t evt_msg_len;

  float current_a = 0;
  int rise_percent = 0;
  float least_current_value = START_CURRENT_PEAK_VALUE;
  // Mailbox variables
  struct k_mbox_msg recv_msg;

  // Amount of Percentage change hold variable
  int i = 0;
  unsigned long int time_stamp, prev_time_stamp;

  rotateMotor(&hz_stepper_motor_params, CLAMP_MOTOR_PER, CLAMP_MOTOR_clamp,
      CLAMP_MOTOR_enable, Disable_TruePWM);
  time_stamp = k_uptime_get();

  evt_msg[0] = present_pos(MAN_HOR_CLAMP, i);
  evt_msg_len = 1U;

  // Notify to ED over BLE
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE, command,
      RUNNING_HOR, evt_msg, evt_msg_len);

  ins_motors_timer_expired_flag = false;
  k_timer_start(&instalation_motors_timer, HOR_TRAVEL_TIME, K_MSEC(0));

  while (clamp_data.hor_pos != 1) { // horizontal == 1 means clamped
    // prepare the mailbox receive buffer
    recv_msg.info = 0;
    recv_msg.size = sizeof(clamp_data.run_pos);
    recv_msg.rx_source_thread = cmd_mgr_thrd_id;

    // retrieve and delete the message if received
    k_mbox_get(&cmd_mgr_mb, &recv_msg, &clamp_data.run_pos, K_MSEC(200U));

    current_a = current_in_amp(CLAMP_AND_MICRO_MOTOR_CURRENT_SENSOR);
    printf("current in Amp = %f \n", current_a);
    if (current_a > NOMINAL_CUT_OFF_VALUE) {
      if (current_a < least_current_value) {
        least_current_value = current_a;
      }
    }

    rise_percent = (int)(((current_a - least_current_value) / least_current_value) * 100);
    printk("horizontal clamp rise_percent = %d \n", rise_percent);

    if ((rise_percent >= H_C_CUT_OFF_PERCENT) || (ins_motors_timer_expired_flag)) {
      if (!ins_motors_timer_expired_flag) {
        k_timer_start(&instalation_motors_timer, K_MSEC(0), K_MSEC(0));
      }
      clamp_data.hor_pos = 1;

      i = (k_uptime_get() - time_stamp) / HOR_LEN_CAP_TIME_MILLIS;

      if (i > 0) {
        evt_msg[0] = present_pos(MAN_HOR_CLAMP, i);
      }
      evt_msg_len = 1U;

      // Delay for horizontal clamp to settle in AUTO_CLAMP case
      if (clamp_data.run_pos == AUTO_CLAMP) {
        k_msleep(1000U);

        // TODO : added for testing
        // bringing product balancer up....
        nrf_gpio_pin_set(P_BAL_F);
        nrf_gpio_pin_clear(P_BAL_B);
        // end

        k_msleep(3000U);

        // TODO : added for testing
        // stop product balancer motors.
        nrf_gpio_pin_set(P_BAL_F);
        nrf_gpio_pin_set(P_BAL_B);
        // end
      }

      // Notify to ED over BLE
      ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE, command,
          CLAMPED_HOR, evt_msg, evt_msg_len);

      while (clamp_data.run_pos == MAN_HOR_CLAMP) {
        // prepare the mailbox receive buffer
        recv_msg.info = 0;
        recv_msg.size = sizeof(clamp_data.run_pos);
        recv_msg.rx_source_thread = cmd_mgr_thrd_id;

        // retrieve and delete the message if received
        k_mbox_get(&cmd_mgr_mb, &recv_msg, &clamp_data.run_pos, K_NO_WAIT);
        k_msleep(20U);
      }
    } else {
      clamp_data.hor_pos = 0;
      prev_time_stamp = k_uptime_get();
      i = (prev_time_stamp - time_stamp) / HOR_LEN_CAP_TIME_MILLIS;
      if (i > 0) {
        time_stamp = prev_time_stamp - ((prev_time_stamp - time_stamp) % HOR_LEN_CAP_TIME_MILLIS);
        evt_msg[0] = present_pos(MAN_HOR_CLAMP, i);
        evt_msg_len = 1U;

        // Notify to ED over BLE
        ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE, command,
            RUNNING_HOR, evt_msg, evt_msg_len);
      }
    }

    if (clamp_data.run_pos == STOP_CLAMP) {
      k_timer_start(&instalation_motors_timer, K_MSEC(0), K_MSEC(0));
      rotateMotor(&hz_stepper_motor_params, CLAMP_MOTOR_PER, CLAMP_MOTOR_clamp,
          CLAMP_MOTOR_disable, Disable_TruePWM);
      rotateMotor(&vr_stepper_motor_params, CLAMP_MOTOR_PER, CLAMP_MOTOR_clamp,
          CLAMP_MOTOR_disable, Disable_TruePWM);
      return;
    }
  }
  rotateMotor(&hz_stepper_motor_params, CLAMP_MOTOR_PER, CLAMP_MOTOR_clamp,
      CLAMP_MOTOR_disable, Disable_TruePWM);
  rotateMotor(&vr_stepper_motor_params, CLAMP_MOTOR_PER, CLAMP_MOTOR_clamp,
      CLAMP_MOTOR_disable, Disable_TruePWM);
}

/*
 * brief: This function called by direct manual control or auto unclamp control.
 * and controls horizontal motor in unclamp direction and stores unclamp state in to global variable.
 */
void horizontal_unclamp(uint8_t command) {
  // Buffer to hold extra bytes for notifying percentage values
  uint8_t evt_msg[2];
  uint8_t evt_msg_len;
  uint8_t l_count = 0; // added by samuel for unclamp fail safe
  float current_a = 0;
  int rise_percent = 0;
  float least_current_value = START_CURRENT_PEAK_VALUE;

  // Mailbox variables
  struct k_mbox_msg recv_msg;

  // Amount of Percentage change hold variable
  int i = 0;
  unsigned long int time_stamp, prev_time_stamp;

  // NOTE: erase previous calibration values.
  // TODO: call erase or invalidate previous calibrates values function.
  erase_Calibration_data();

  rotateMotor(&hz_stepper_motor_params, CLAMP_MOTOR_PER, CLAMP_MOTOR_unclamp,
      CLAMP_MOTOR_enable, Disable_TruePWM);
  time_stamp = k_uptime_get();
  evt_msg[0] = present_pos(MAN_HOR_UNCLAMP, i);
  evt_msg_len = 1U;

  // Notify to ED over BLE
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE, command, RUNNING_HOR, evt_msg, evt_msg_len);
  ins_motors_timer_expired_flag = false;
  k_timer_start(&instalation_motors_timer, HOR_TRAVEL_TIME, K_MSEC(0));

  /*added to avoid initial clamped spike*/
  if (clamp_data.hor_pos == 1) {
    k_msleep(2500);
  }

  while (clamp_data.hor_pos != 2) { // hrznt == 2 means horizontal unclamped already.
    // prepare the mailbox receive buffer
    recv_msg.info = 0;
    recv_msg.size = sizeof(clamp_data.run_pos);
    recv_msg.rx_source_thread = cmd_mgr_thrd_id;

    // retrieve and delete the message if received
    k_mbox_get(&cmd_mgr_mb, &recv_msg, &clamp_data.run_pos, K_MSEC(200U));

    current_a = current_in_amp(CLAMP_AND_MICRO_MOTOR_CURRENT_SENSOR);
    printf("current in Amp = %f \n", current_a);
    if (current_a > NOMINAL_CUT_OFF_VALUE) {
      if (current_a < least_current_value) {
        least_current_value = current_a;
      }
    }
    rise_percent = (int)(((current_a - least_current_value) / least_current_value) * 100);
    printk("horizontal unclamp rise_percent = %d \n", rise_percent);

    /**added by samuel**/
    if (clamp_data.hor_per <= 5) {
      l_count++;
      clamp_data.hor_per = 6;
      if (l_count == 2) {
        rise_percent = (rise_percent > H_U_CUT_OFF_PERCENT) ? rise_percent : (H_U_CUT_OFF_PERCENT + 1);
      }
    }
    /*** ---- ***/

    if ((rise_percent >= H_U_CUT_OFF_PERCENT) || (ins_motors_timer_expired_flag)) {
      if (!ins_motors_timer_expired_flag) {
        k_timer_start(&instalation_motors_timer, K_MSEC(0), K_MSEC(0));
      }
      clamp_data.hor_pos = 2;
      clamp_data.hor_per = 0; // 0 percentage

      evt_msg[0] = clamp_data.hor_per;
      evt_msg_len = 1U;

      // Notify to ED over BLE
      ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE, command,
          UNCLAMPED_HOR, evt_msg, evt_msg_len);

      while (clamp_data.run_pos == MAN_HOR_UNCLAMP) {
        // prepare the mailbox receive buffer
        recv_msg.info = 0;
        recv_msg.size = sizeof(clamp_data.run_pos);
        recv_msg.rx_source_thread = cmd_mgr_thrd_id;

        // retrieve and delete the message if received
        k_mbox_get(&cmd_mgr_mb, &recv_msg, &clamp_data.run_pos, K_NO_WAIT);
        k_msleep(20U);
      }
    } else {
      clamp_data.hor_pos = 0;
      prev_time_stamp = k_uptime_get();
      i = (prev_time_stamp - time_stamp) / HOR_LEN_CAP_TIME_MILLIS;
      if (i > 0) {
        time_stamp = prev_time_stamp - ((prev_time_stamp - time_stamp) % HOR_LEN_CAP_TIME_MILLIS);
        evt_msg[0] = present_pos(MAN_HOR_UNCLAMP, i);
        evt_msg_len = 1U;

        // Notify to ED over BLE
        ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE,
            command, RUNNING_HOR, evt_msg, evt_msg_len);
      }
    }

    if (clamp_data.run_pos == STOP_CLAMP) {
      k_timer_start(&instalation_motors_timer, K_MSEC(0), K_MSEC(0));
      rotateMotor(&hz_stepper_motor_params, CLAMP_MOTOR_PER, CLAMP_MOTOR_clamp,
          CLAMP_MOTOR_disable, Disable_TruePWM);
      rotateMotor(&vr_stepper_motor_params, CLAMP_MOTOR_PER, CLAMP_MOTOR_clamp,
          CLAMP_MOTOR_disable, Disable_TruePWM);
      return;
    }
  }
  rotateMotor(&hz_stepper_motor_params, CLAMP_MOTOR_PER, CLAMP_MOTOR_clamp,
      CLAMP_MOTOR_disable, Disable_TruePWM);
  rotateMotor(&vr_stepper_motor_params, CLAMP_MOTOR_PER, CLAMP_MOTOR_clamp,
      CLAMP_MOTOR_disable, Disable_TruePWM);
}

/*
 * brief: This function called by direct manual control or auto clamp control.
 *        and controls vertical motor in clamp direction and stores clamp state in to global variable.
 */
void vertical_clamp(uint8_t command) {
  // Buffer to hold extra bytes for notifying percentage values
  uint8_t evt_msg[2];
  uint8_t evt_msg_len;

  float current_a = 0;
  int rise_percent = 0;
  float least_current_value = 0.375;
  // Mailbox variables
  struct k_mbox_msg recv_msg;

  // Amount of Percentage change hold variable
  int i = 0;
  unsigned long int time_stamp, prev_time_stamp;

  rotateMotor(&vr_stepper_motor_params, CLAMP_MOTOR_PER, CLAMP_MOTOR_clamp,
      CLAMP_MOTOR_enable, Disable_TruePWM);
  time_stamp = k_uptime_get();
  evt_msg[0] = present_pos(MAN_VER_CLAMP, i);
  evt_msg_len = 1U;

  // Notify to ED over BLE
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE, command,
      RUNNING_VER, evt_msg, evt_msg_len);

  ins_motors_timer_expired_flag = false;
  k_timer_start(&instalation_motors_timer, VER_TRAVEL_TIME, K_MSEC(0));

  while (clamp_data.ver_pos != 1) { // vrtl_pos == 1 means vertical motor at clamp pos
    // prepare the mailbox receive buffer
    recv_msg.info = 0;
    recv_msg.size = sizeof(clamp_data.run_pos);
    recv_msg.rx_source_thread = cmd_mgr_thrd_id;

    // retrieve and delete the message if received
    k_mbox_get(&cmd_mgr_mb, &recv_msg, &clamp_data.run_pos, K_MSEC(200U));

    current_a = current_in_amp(CLAMP_AND_MICRO_MOTOR_CURRENT_SENSOR);
    printf("current in Amp = %f \n", current_a);
    if (current_a > NOMINAL_CUT_OFF_VALUE) {
      if (current_a < least_current_value) {
        least_current_value = current_a;
      }
    }
    rise_percent = (int)(((current_a - least_current_value) / least_current_value) * 100);
    printk("vertical clamp rise_percent = %d \n", rise_percent);

    if ((rise_percent >= V_C_CUT_OFF_PERCENT) || (ins_motors_timer_expired_flag)) {
      if (!ins_motors_timer_expired_flag) {
        k_timer_start(&instalation_motors_timer, K_MSEC(0), K_MSEC(0));
      }
      clamp_data.ver_pos = 1;

      i = (k_uptime_get() - time_stamp) / VER_LEN_CAP_TIME_MILLIS;
      if (i > 0) {
        evt_msg[0] = present_pos(MAN_VER_CLAMP, i);
      }
      evt_msg_len = 1U;

      // Notify to ED over BLE
      ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE, command,
          CLAMPED_VER, evt_msg, evt_msg_len);

      while (clamp_data.run_pos == MAN_VER_CLAMP) {
        // prepare the mailbox receive buffer
        recv_msg.info = 0;
        recv_msg.size = sizeof(clamp_data.run_pos);
        recv_msg.rx_source_thread = cmd_mgr_thrd_id;

        // retrieve and delete the message if received
        k_mbox_get(&cmd_mgr_mb, &recv_msg, &clamp_data.run_pos, K_NO_WAIT);
        k_msleep(20U);
      }
    } else {
      clamp_data.ver_pos = 0;
      prev_time_stamp = k_uptime_get();
      i = (prev_time_stamp - time_stamp) / VER_LEN_CAP_TIME_MILLIS;
      if (i > 0) {
        time_stamp = prev_time_stamp - ((prev_time_stamp - time_stamp) % VER_LEN_CAP_TIME_MILLIS);
        evt_msg[0] = present_pos(MAN_VER_CLAMP, i);
        evt_msg_len = 1U;

        // Notify to ED over BLE
        ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE, command,
            RUNNING_VER, evt_msg, evt_msg_len);
      }
    }

    if (clamp_data.run_pos == STOP_CLAMP) {
      k_timer_start(&instalation_motors_timer, K_MSEC(0), K_MSEC(0));
      rotateMotor(&hz_stepper_motor_params, CLAMP_MOTOR_PER, CLAMP_MOTOR_clamp,
          CLAMP_MOTOR_disable, Disable_TruePWM);
      rotateMotor(&vr_stepper_motor_params, CLAMP_MOTOR_PER, CLAMP_MOTOR_clamp,
          CLAMP_MOTOR_disable, Disable_TruePWM);
      return;
    }
  }
  rotateMotor(&hz_stepper_motor_params, CLAMP_MOTOR_PER, CLAMP_MOTOR_clamp,
      CLAMP_MOTOR_disable, Disable_TruePWM);
  rotateMotor(&vr_stepper_motor_params, CLAMP_MOTOR_PER, CLAMP_MOTOR_clamp,
      CLAMP_MOTOR_disable, Disable_TruePWM);
}

/*
 * brief: This function called by direct manual control or auto unclamp control.
 *        and controls vertical motor in unclamp direction and stores clamp state in to global variable.
 */
void vertical_unclamp(uint8_t command) {
  // Buffer to hold extra bytes for notifying percentage values
  uint8_t evt_msg[2];
  uint8_t evt_msg_len;

  uint8_t l_count = 0; // added unclamp fail safe

  float current_a = 0;
  int rise_percent = 0;
  float least_current_value = 0.375;
  // Mailbox variables
  struct k_mbox_msg recv_msg;

  // Amount of Percentage change hold variable
  int i = 0;
  unsigned long int time_stamp, prev_time_stamp;

  rotateMotor(&vr_stepper_motor_params, CLAMP_MOTOR_PER, CLAMP_MOTOR_unclamp,
      CLAMP_MOTOR_enable, Disable_TruePWM);
  time_stamp = k_uptime_get();

  evt_msg[0] = present_pos(MAN_VER_UNCLAMP, i);
  evt_msg_len = 1U;

  // Notify to ED over BLE
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE, command,
      RUNNING_VER, evt_msg, evt_msg_len);

  ins_motors_timer_expired_flag = false;
  k_timer_start(&instalation_motors_timer, VER_TRAVEL_TIME, K_MSEC(0));

  while (clamp_data.ver_pos != 2) { // vrtl == 2 means vertically unclamped only
                                    // prepare the mailbox receive buffer
    recv_msg.info = 0;
    recv_msg.size = sizeof(clamp_data.run_pos);
    recv_msg.rx_source_thread = cmd_mgr_thrd_id;

    // retrieve and delete the message if received
    k_mbox_get(&cmd_mgr_mb, &recv_msg, &clamp_data.run_pos, K_MSEC(200U));

    current_a = current_in_amp(CLAMP_AND_MICRO_MOTOR_CURRENT_SENSOR);
    printf("current in Amp = %f \n", current_a);
    if (current_a > NOMINAL_CUT_OFF_VALUE) {
      if (current_a < least_current_value) {
        least_current_value = current_a;
      }
    }
    rise_percent = (int)(((current_a - least_current_value) / least_current_value) * 100);
    printk("vertical unclamp rise_percent = %d \n", rise_percent);

    if (clamp_data.ver_per <= 5) {
      l_count++;
      clamp_data.ver_per = 6;
      if (l_count == 2) {
        rise_percent = (rise_percent > V_U_CUT_OFF_PERCENT) ? rise_percent : (V_U_CUT_OFF_PERCENT + 1);
      }
    }

    if ((rise_percent >= V_U_CUT_OFF_PERCENT) || (ins_motors_timer_expired_flag)) {
      if (!ins_motors_timer_expired_flag) {
        k_timer_start(&instalation_motors_timer, K_MSEC(0), K_MSEC(0));
      }
      clamp_data.ver_pos = 2;
      clamp_data.ver_per = 0; // 0 percentage

      evt_msg[0] = clamp_data.ver_per;
      evt_msg_len = 1U;

      // Notify to ED over BLE
      ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE, command,
          UNCLAMPED_VER, evt_msg, evt_msg_len);

      while (clamp_data.run_pos == MAN_VER_UNCLAMP) {
        // prepare the mailbox receive buffer
        recv_msg.info = 0;
        recv_msg.size = sizeof(clamp_data.run_pos);
        recv_msg.rx_source_thread = cmd_mgr_thrd_id;

        // retrieve and delete the message if received
        k_mbox_get(&cmd_mgr_mb, &recv_msg, &clamp_data.run_pos, K_NO_WAIT);
        k_msleep(20U);
      }
    } else {
      clamp_data.ver_pos = 0;
      prev_time_stamp = k_uptime_get();
      i = (prev_time_stamp - time_stamp) / VER_LEN_CAP_TIME_MILLIS;
      if (i > 0) {
        time_stamp = prev_time_stamp - ((prev_time_stamp - time_stamp) % VER_LEN_CAP_TIME_MILLIS);
        evt_msg[0] = present_pos(MAN_VER_UNCLAMP, i);
        evt_msg_len = 1U;

        // Notify to ED over BLE
        ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE, command,
            RUNNING_VER, evt_msg, evt_msg_len);
      }
    }
    if (clamp_data.run_pos == STOP_CLAMP) {
      k_timer_start(&instalation_motors_timer, K_MSEC(0), K_MSEC(0));
      rotateMotor(&hz_stepper_motor_params, CLAMP_MOTOR_PER, CLAMP_MOTOR_clamp,
          CLAMP_MOTOR_disable, Disable_TruePWM);
      rotateMotor(&vr_stepper_motor_params, CLAMP_MOTOR_PER, CLAMP_MOTOR_clamp,
          CLAMP_MOTOR_disable, Disable_TruePWM);
      return;
    }
  }
  rotateMotor(&hz_stepper_motor_params, CLAMP_MOTOR_PER, CLAMP_MOTOR_clamp,
      CLAMP_MOTOR_disable, Disable_TruePWM);
  rotateMotor(&vr_stepper_motor_params, CLAMP_MOTOR_PER, CLAMP_MOTOR_clamp,
      CLAMP_MOTOR_disable, Disable_TruePWM);
}

/*
 * brief: This function called inside installation thread for auto clamp.
 *        and it calls sub functions to complete auto clamp
 *        and stores clamp state in to global variable.
 */
uint8_t auto_clamp(void) {
  current_in_amp(CLAMP_AND_MICRO_MOTOR_CURRENT_SENSOR);
  if (clamp_data.state_pos != 1) {
    if (clamp_data.hor_pos == 1) {
      horizontal_unclamp(AUTO_CLAMP);
      k_msleep(20U);
    }

    if (clamp_data.run_pos == STOP_CLAMP) {
      return STOP_CLAMP;
    }

    // call vertical motor clamp first......
    if (clamp_data.ver_pos == 1) {
      ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE,
          AUTO_CLAMP, CLAMPED_VER, NULL, 0);
      k_msleep(20U);
    } else {
      vertical_clamp(AUTO_CLAMP);
      k_msleep(20U);
    }

    if (clamp_data.run_pos == STOP_CLAMP) {
      return STOP_CLAMP;
    }

    // call horizontal motor clamp next......
    if (clamp_data.hor_pos == 1) {
      ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE,
          AUTO_CLAMP, CLAMPED_HOR, NULL, 0);
      k_msleep(20U);
    } else {
      horizontal_clamp(AUTO_CLAMP);
      if (clamp_data.run_pos == STOP_CLAMP) {
        return STOP_CLAMP;
      }

      // Stop torating stepper motors
      rotateMotor(&hz_stepper_motor_params, CLAMP_MOTOR_PER, CLAMP_MOTOR_clamp,
          CLAMP_MOTOR_disable, Disable_TruePWM);
      rotateMotor(&vr_stepper_motor_params, CLAMP_MOTOR_PER, CLAMP_MOTOR_clamp,
          CLAMP_MOTOR_disable, Disable_TruePWM);
      k_msleep(20U);
    }

    if ((clamp_data.ver_pos == 1) && (clamp_data.hor_pos == 1)) {
      clamp_data.state_pos = 1;
    }
  }
  return INS_IDLE;
}

/*
 * brief: This function called inside installation thread for auto unclamp.
 *        and it calls sub functions to complete auto unclamp
 *        and stores unclamp state in to global variable.
 */
uint8_t auto_unclamp(void) {
  current_in_amp(CLAMP_AND_MICRO_MOTOR_CURRENT_SENSOR);
  if (clamp_data.state_pos != 2) {
    // call horizontal motor unclamp first.......
    if (clamp_data.hor_pos == 2) {
      ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE,
          AUTO_UNCLAMP, UNCLAMPED_HOR, NULL, 0);
      k_msleep(20U);
    } else {
      horizontal_unclamp(AUTO_UNCLAMP);
      k_msleep(20U);
    }

    if (clamp_data.run_pos == STOP_CLAMP) {
      return STOP_CLAMP;
    }

    k_msleep(20U);
    // call vertical motor unclamp next.......
    if (clamp_data.ver_pos == 2) {
      ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE,
          AUTO_UNCLAMP, UNCLAMPED_VER, NULL, 0);
      k_msleep(20U);
    } else {
      vertical_unclamp(AUTO_UNCLAMP);
      k_msleep(20U);
    }

    if (clamp_data.run_pos == STOP_CLAMP) {
      return STOP_CLAMP;
    }

    k_msleep(20U);
    if ((clamp_data.ver_pos == 2) && (clamp_data.hor_pos == 2)) {
      clamp_data.state_pos = 2;
    }
  }
  return INS_IDLE;
}

/*
 * brief: This function called by system thread when ever it 
   started and going for installation of product.
 */
int8_t product_balancer(uint8_t command) {
  float current_a = 0;
  int rise_percent = 0;
  float least_current_value = 0.40;
  uint8_t next_cmd = 0;

  // Mailbox variables
  struct k_mbox_msg recv_msg;

  // prepare the mailbox receive buffer
  recv_msg.info = 0;
  recv_msg.size = sizeof(next_cmd);
  recv_msg.rx_source_thread = cmd_mgr_thrd_id;

  CUR_SENSOR_CUT_OFF_V = current_offset(CLAMP_AND_MICRO_MOTOR_CURRENT_SENSOR);

  switch (command) {
  case A_POSITION:
    printk("Setting Product Balancer to Home Position --- \n");

    // Notify over BLE that Product balancer is moving up
    ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE,
        PRODUCT_BALANCER_UP, PB_RUNNING_UP, NULL, 0);

    // optionally added later need to change
    nrf_gpio_pin_set(P_BAL_F);
    nrf_gpio_pin_clear(P_BAL_B);

    ins_motors_timer_expired_flag = false;
    k_timer_start(&instalation_motors_timer, PB_UP_TRAVEL_TIME, K_MSEC(0));
    k_msleep(4000U); // added for USA setups.

    while ((rise_percent < P_A_CUT_OFF_PERCENT) && (next_cmd != STOP_CLAMP)) {
      current_a = current_in_amp(CLAMP_AND_MICRO_MOTOR_CURRENT_SENSOR);

      if (current_a > 0.04) {
        if (current_a < least_current_value) {
          least_current_value = current_a;
        }
      }

      rise_percent = (int)(((current_a - least_current_value) / least_current_value) * 100);
      printk("product balancer rise_percent = %d \n", rise_percent);

      // retrieve and delete the message if received
      k_mbox_get(&cmd_mgr_mb, &recv_msg, &next_cmd, K_MSEC(100));

      if (ins_motors_timer_expired_flag) {
        break;
      }
    }

    // check if the product balancer wasn't interrupted while moving and notify the same over BLE
    if (next_cmd == 0) {
      k_msleep(2500);
      // Notify over BLE that Product balancer is moved Extreme UP position
      pb_status = PB_UP;
      ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE, PRODUCT_BALANCER_UP,
          PB_EXTREME_UP, NULL, 0);
    } else if (next_cmd == STOP_CLAMP) {
      // Notify over BLE that Product balancer movemen is stopped
      pb_status = STOPPED_IN_BETWEEN;
      ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE, PRODUCT_BALANCER_UP,
          CLAMP_STOPPED, NULL, 0);
    }

    clamp_data.p_balancer = A_POSITION;

    if (!ins_motors_timer_expired_flag) {
      k_timer_start(&instalation_motors_timer, K_MSEC(0), K_MSEC(0));
    }
    break;

  case B_POSITION:
    printk("Setting Product Balancer to Extream Position --- \n");

    // optionally added later need to change
    nrf_gpio_pin_clear(P_BAL_F);
    nrf_gpio_pin_set(P_BAL_B);

    while (rise_percent < P_B_CUT_OFF_PERCENT) {
      current_a = current_in_amp(CLAMP_AND_MICRO_MOTOR_CURRENT_SENSOR);

      if (current_a > 0.04) {
        if (current_a < least_current_value) {
          least_current_value = current_a;
        }
      }

      rise_percent = (int)(((current_a - least_current_value) / least_current_value) * 100);
      printk("product balancer rise_percent = %d \n", rise_percent);
      k_msleep(100U);
    }

    clamp_data.p_balancer = B_POSITION;
    break;

  case G_POSITION:
    printk("Setting Product Balancer to Equal Position --- \n");

    // Notify over BLE that Product balancer is moving up
    ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE, PRODUCT_BALANCER_DOWN,
        PB_RUNNING_DOWN, NULL, 0);

    // optionally added later need to change
    nrf_gpio_pin_clear(P_BAL_F);
    nrf_gpio_pin_set(P_BAL_B);

    ins_motors_timer_expired_flag = false;
    k_timer_start(&instalation_motors_timer, PB_DW_TRAVEL_TIME, K_MSEC(0));

    k_msleep(4000U); // added for USA setups.

    while ((rise_percent < P_G_CUT_OFF_PERCENT) && (next_cmd != STOP_CLAMP)) {
      current_a = current_in_amp(CLAMP_AND_MICRO_MOTOR_CURRENT_SENSOR);

      if (current_a > 0.04) {
        if (current_a < least_current_value) {
          least_current_value = current_a;
        }
      }

      rise_percent = (int)(((current_a - least_current_value) / least_current_value) * 100);
      printk("product balancer rise_percent = %d \n", rise_percent);

      // retrieve and delete the message if received
      k_mbox_get(&cmd_mgr_mb, &recv_msg, &next_cmd, K_MSEC(100));

      if (ins_motors_timer_expired_flag) {
        break;
      }
    }

    if (next_cmd == 0) {
      k_msleep(1000);
      // Notify over BLE that Product balancer is moved Extreme UP position
      pb_status = PB_DOWN;
      ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE, PRODUCT_BALANCER_DOWN,
          PB_EXTREME_DOWN, NULL, 0);
    } else if (next_cmd == STOP_CLAMP) {
      // Notify over BLE that Product balancer is moved Extreme UP position
      pb_status = STOPPED_IN_BETWEEN;
      ADO_notify_ble(cmd_mgr_thrd_id, ADO_INSTALLATION_RESPONSE, PRODUCT_BALANCER_DOWN,
          CLAMP_STOPPED, NULL, 0);
    }
    clamp_data.p_balancer = G_POSITION;

    if (!ins_motors_timer_expired_flag) {
      k_timer_start(&instalation_motors_timer, K_MSEC(0), K_MSEC(0));
    }

    break;

  default:
    break;
  }
  // optionally added later need to change
  nrf_gpio_pin_set(P_BAL_F);
  nrf_gpio_pin_set(P_BAL_B);
  save_install_data(); //save installation data into eeprom
  return 0;
}

/*
 * brief: This function is used to write the Installation parameters to EEPROM.
 */
int8_t save_install_data() {
  int error = 0;
  uint8_t data[7];

  data[0] = clamp_data.hor_pos; // 1=hor clamped, 2=hor unclamped, 0= inermediate
  data[1] = clamp_data.ver_pos; // 1=ver clamped, 2=ver unclamped, 0= inermediate
  data[2] = clamp_data.hor_per;
  data[3] = clamp_data.ver_per;
  data[4] = clamp_data.state_pos; // 1=auto clamped, 2= auto unclamped
  data[5] = clamp_data.p_balancer;
  data[6] = clamp_data.p_balancer_per;

  error = Write_EEPROM(INSTALLATION_MEMORY_LOCATION, &data[0], 7);

  if (error) {
    printk("error saving install data to EEPROM trying one more time");
    error = Write_EEPROM(INSTALLATION_MEMORY_LOCATION, &data[0], 7);
  }
  return error;
}

/*
 * brief: This function is used to read back the saved Installation parameters from
    EEPROM to global installation structure.
 */
int8_t load_install_data() {
  int error = 0;
  uint8_t data[7];

  error = Read_EEPROM(INSTALLATION_MEMORY_LOCATION, &data[0], 7);

  if (error) {
    printk("error loading install data from EEPROM trying one more time");
    error = Read_EEPROM(INSTALLATION_MEMORY_LOCATION, &data[0], 7);
  }

  clamp_data.hor_pos = data[0]; // 1=hor clamped, 2=hor unclamped, 0= inermediate
  clamp_data.ver_pos = data[1]; // 1=ver clamped, 2=ver unclamped, 0= inermediate
  clamp_data.hor_per = data[2];
  clamp_data.ver_per = data[3];
  clamp_data.state_pos = data[4]; // 1=auto clamped, 2= auto unclamped
  clamp_data.p_balancer = data[5];

  if (clamp_data.p_balancer == 3) {
    pb_status = PB_DOWN;
  } else if (clamp_data.p_balancer == 1) {
    pb_status = PB_UP;
  }
  clamp_data.p_balancer_per = data[6];
  return error;
}

/*
 * brief: This function is used to check whether the ADO is installed on the door
 */
bool is_ADO_Installed(struct auto_clamp_data *pClamp_data) {
  bool retval = false;
  // Input check
  if (pClamp_data != NULL) {
    // Check if the ADO is installed
    if (pClamp_data->state_pos == ADO_INSTALLED) {
      retval = true;
    }
  }
  // TODO: Handle the NULL case and print error if required.

  return retval;
}

void clamp_motors_timer_expire(struct k_timer *timer) {
  ins_motors_timer_expired_flag = true;
  printk("Clamp timer completed..........\n");
  k_timer_stop(&instalation_motors_timer); // call timer_stop
}

uint8_t check_installation() {
  bool temp = is_ADO_Installed(&clamp_data);
  printk("\nis_ADO_installed is returning: %d, pb_status is: %d\n", temp, pb_status);
  if (temp == true) {
    if (pb_status == PB_UP) {
      return installation_status_check = INSTALLED_BOT;
    } else if (pb_status == PB_DOWN || pb_status == STOPPED_IN_BETWEEN) {
      return installation_status_check = PROCESS_GOING_ON;
    } else {
      return installation_status_check = ERROR_IN_INSTALLING;
    }
  } else if (temp == false) {
    if (pb_status == PB_UP) {
      return installation_status_check = UNINSTALLED_BOT;
    } else if (pb_status == PB_DOWN || pb_status == STOPPED_IN_BETWEEN) {
      return installation_status_check = PROCESS_GOING_ON;
    } else {
      return installation_status_check = ERROR_IN_INSTALLING;
    }
  }
}