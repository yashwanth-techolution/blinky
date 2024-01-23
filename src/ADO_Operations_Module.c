#include <math.h>
#include "Includes/AnalogIn.h"
#include "Includes/BLE_Module.h"
#include "Includes/IMU_Module.h"
#include "Includes/Install_ADO.h"
#include "Includes/Intent_Module.h"
#include "Includes/GPIO_Expander.h"
#include "Includes/Display_Module.h"
#include "Includes/IPC_Manager_Module.h"
#include "Includes/Calibration_Module.h"
#include "Includes/Device_Status_Module.h"
#include "Includes/Motor_Control_Module.h"
#include "Includes/ADO_Operations_Module.h"
#include "Includes/ADO_Reset_Calib_Module.h"
#include "EEPROM_Module.h"
#include "Includes/ADO_PIR_Module.h"

// Global variables for holding linearised IMU values
float linear_IMUOpen;
float linear_IMUClose;
float pev_IMU_Val;
float curr_IMU_Val;
float uss_range;
int imu_correction=0;
bool Close_YAW = false;
bool Open_YAW = false;
#define motor_min_speed 50
#define Enable_open true
#define Enable_close false

// Variable to hold the ADO Operation state
ado_operations_t operation;
ado_operations_t operation_status_check;
bool restart_or_not = 1;
uint8_t opr_cmd = 0U;

void timer_expire_dooropen(struct k_timer *timer);
void timer_expire_doorclose(struct k_timer *timer);
int execution_time_close = 3, execution_time_open=3;
struct k_timer timer_open;
struct k_timer timer_close;
int flag_close=0, flag_open=0;

uint8_t check_operation_status();
uint8_t check_clutch();
uint8_t clutch_check_var;

bool accelaration=false;
bool deceration_flag=false;
bool operation_flag = false;

/*
 * Brief: The Thread definition for ADO Operation commands, 
          this thread handles Normal in operation commands 
          like OPEN, CLOSE and OPR_STOP commands
 */
void ADO_Operations_Thread() {
  // Mailbox variables
  printk("timer initialization for open the door.. \n");
  k_timer_init(&timer_open, timer_expire_dooropen, NULL);
  
  printk("timer initialization for close the door. \n");
  k_timer_init(&timer_close, timer_expire_doorclose, NULL); 

  struct k_mbox_msg recv_msg;
  power_optimization(OPERATION);
  uint16_t thread_idle_cycle_cnt = 0U;

  while (thread_idle_cycle_cnt < MAX_OPR_THREAD_IDLE_CYCLES) {
    // prepare the mailbox receive buffer
    recv_msg.info = 0;
    recv_msg.size = sizeof(opr_cmd);
    recv_msg.rx_source_thread = cmd_mgr_thrd_id;

    // retrieve and delete the message if received
    if (opr_cmd == 0) {                                       
      // Don't read if opr_cmd has a valid value
      k_mbox_get(&cmd_mgr_mb, &recv_msg, &opr_cmd, K_MSEC(50));
    }

    if(flag_close==1 && opr_cmd!=0) {
      k_timer_stop(&timer_close);
      flag_close=0;
      execution_time_close=3;
    }

    // check for the received command type
    if(opr_cmd)
      printk("\ncheck firmware version_204");
    switch (opr_cmd) {
    case OPEN:
      // reset the counter
      thread_idle_cycle_cnt = 0U;

      // Notify Over BLE that the door is opening
      //ADO_notify_ble(cmd_mgr_thrd_id, ADO_OPERATIONS_RESPONSE, OPEN,
      //    OPENING, NULL, 0);      //Uttam added this for testing

      if(is_lock_config && is_door_locked) {
         enum_lock_status = EM_LOCKED_FALSE;
         Lock_sendCmdACK(&enum_lock_status, 1);
         printk("\n waiting for response from lock");
         k_sleep(K_MSEC(300)); //Delay 300ms Between EM Lock open and Door Open
      }

      printk("\n lock status %d",is_door_locked);
      opr_cmd = OpenTheDoor(OPEN);

      // TODO: Add the else case to notify open and close are too near.
      break;

    case CLOSE:
      // reset the counter
      thread_idle_cycle_cnt = 0U;

      // Notify Over BLE that the door is closing
     // ADO_notify_ble(cmd_mgr_thrd_id, ADO_OPERATIONS_RESPONSE, CLOSE,
      //    CLOSING, NULL, 0);    //Uttam added this for testing.

      if(execution_time_close>=0) {
        opr_cmd=CloseTheDoor(CLOSE);

        if(flag_close==-1) {
          execution_time_close=3;
          flag_close=0;
        }
      } else {
        flag_close=0;
        execution_time_close=3;
        opr_cmd=0;
      }

      // TODO: Add the else case to notify open and close are too near.
      break;

    case OPR_STOP:
      // reset the counter
      thread_idle_cycle_cnt = 0U;
      is_door_in_open = 2;
      opr_cmd=StopTheDoor(OPR_STOP);

      if(opr_cmd<0) {
          opr_cmd = 0U;
      }
      
      break;

    case CLUTCH_ENGAGE:
      // reset the counter
      thread_idle_cycle_cnt = 0U;

      //Engage the Clutch
      opr_cmd = engageClutch();

      // added for clutch function
      // IMP : Stop the motor
      Stop_Motor(&Gear_motor_params, DRIVE_MOTOR_backward, DRIVE_MOTOR_disable,
          Disable_TruePWM);

      /* Added temporarily for testing mechanical aspects of clutch,
      but it must be removed in later releases */
      if (opr_cmd == OPR_STOP) {
        // TODO Stop clutch motion
        nrf_gpio_pin_set(CLUTCH_F);
        nrf_gpio_pin_set(CLUTCH_B);

        // Notify over BLE that the Clutch operation
        ADO_notify_ble(cmd_mgr_thrd_id, ADO_OPERATIONS_RESPONSE, 
            CLUTCH_ENGAGE, OPR_STOPPED, NULL, 0);
        opr_cmd = 0U;
      }
      break;

    case CLUTCH_DISENGAGE:
      // reset the counter
      thread_idle_cycle_cnt = 0U;

      // 1. Disengage the Clutch
      opr_cmd = disengageClutch();

      // added for clutch function
      // IMP : Stop the motor
      Stop_Motor(&Gear_motor_params, DRIVE_MOTOR_backward, 
          DRIVE_MOTOR_disable, Disable_TruePWM);

      /* Added temporarily for testing mechanical aspects of clutch, 
        but it must be removed in later releases */
      if (opr_cmd == OPR_STOP) {
        // TODO Stop clutch motion
        nrf_gpio_pin_set(CLUTCH_F);
        nrf_gpio_pin_set(CLUTCH_B);

        // Notify over BLE that the Clutch operation
        ADO_notify_ble(cmd_mgr_thrd_id, ADO_OPERATIONS_RESPONSE,
            CLUTCH_DISENGAGE, OPR_STOPPED, NULL, 0);
        opr_cmd = 0U;
      }
      break;

    case ON_LED:
      // reset the counter
      thread_idle_cycle_cnt = 0U;

      // For Testing purposes, this can be received from ED, otherwise any other thread.
      setLEDStrip(ON_LED);
      opr_cmd = 0U;
      break;

    case OFF_LED:
      // reset the counter
      thread_idle_cycle_cnt = 0U;

      // For Testing purposes, this can be received from ED, otherwise any other thread.
      setLEDStrip(OFF_LED);

      // Notify over BLE that LED is OFF
      opr_cmd = 0U;
      break;

    default:
      // Increment the counter to terminate the thread after MAX_OPR_THREAD_IDLE_CYCLES
      thread_idle_cycle_cnt = thread_idle_cycle_cnt + 1;
      opr_cmd = 0U; // Imp: Reset the command to 0 if received any invalid command
      break;
    }
  }
  power_optimization(OPERATION_STOP);
}


float calculate_IMU_val_1(float X1_Val, float Y1_Val, float IMU_Value) {
  X1_Val -= (X1_Val > 359) ? 360 : 0;

  if (Y1_Val < 0) {
    Y1_Val += 360;
    IMU_Value -= ((IMU_Value > X1_Val) && (IMU_Value > Y1_Val)) ? 360 : 0;
  } else {
    IMU_Value += ((IMU_Value < X1_Val) && (IMU_Value < Y1_Val)) ? 360 : 0;
  }
  return IMU_Value;
}

float calculate_IMU_val_0(float X1_Val, float Y1_Val, float IMU_Value) {
  X1_Val += (X1_Val < 0) ? 360 : 0;

  if (Y1_Val > 359) {
    Y1_Val -= 360;
    IMU_Value += ((IMU_Value < X1_Val) && (IMU_Value < Y1_Val)) ? 360 : 0;
  } else {
    IMU_Value -= ((IMU_Value > X1_Val) && (IMU_Value > Y1_Val)) ? 360 : 0;
  }
  return IMU_Value;
}

/*
 * Brief: This routine takes care of Opening the Door when OPEN 
    command is received from ED.
 */
uint8_t OpenTheDoor(uint8_t command) {
  int uss_count = 0;
  int staginate_count = 0, door_refrence = 0;
  bool deceleration = true;
  float speed = config_pwm_percent;

  // Mailbox variables
  struct k_mbox_msg recv_msg;
  bool  stop_motor = true, reverse_move = false;
  uint8_t operation_cmd = OPEN, cmd = 0, uc_staginate_in_accel = 0;
  is_bot_staginated = false;
  is_bot_staginated2 = false;
  is_door_in_open = 0;

  // Set initial percentage
  current_door_percentage = 0;

  float openValue = 0;
  accelaration = false;
  int t = 0, open_try_count = 0, open_try_time = 0;

  // Response data variables
  uint8_t evt_msg[2] = {0};
  uint8_t evt_msg_len = 0;
  operation_flag = true;

  // Check if the doorbot is installed and calibrated
  if ((is_doorbot_uninstalled_uncalibrated() == true) || 
      (ADO_get_restart_state() == true)) {
    // debug print
    restart_or_not = 0;
    operation_status_check.state = ERROR_OPERATION_ANDROID;
    printk("doorbot is uninstalled or uncalibrated\n");
    return error_code;
  }

  // Get the corrected Door Open Position IMU readings based on calibrated Yaw and Direction
  linear_IMUOpen = getlinear_OPEN_CLOSE(Enable_open, 999);
  uint8_t prev_door_percent = 0;
  float startIMMUValue = linear_IMUOpen;

  // Set the current state of operations
  operation.state = OPENING_DOOR;

  // Notify Over BLE that the door is opening
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_OPERATIONS_RESPONSE, OPEN,
      OPENING, NULL, 0);

  // Change the display page to door opening
  chg_display_page(DISP_DOOR_OPENING_PAGE);

  // prepare the mailbox receive buffer
  recv_msg.info = 0;
  recv_msg.size = sizeof(operation_cmd);
  recv_msg.rx_source_thread = cmd_mgr_thrd_id;

  // verify Open YAW Cross if cross set flag.
  pev_IMU_Val = linear_IMUOpen;
  if (abs(pev_IMU_Val - calib.pos_close) <= 30) {
    Open_YAW = false;
  } else {
    if ((360 - abs(pev_IMU_Val - calib.pos_close)) <= 30) {
      Open_YAW = true;
    } else {
      // TODO genenrate error report to edge device
    }
  }

  // Rotate the motor CW till the calibrated open position
  if (calib.is_direction_cw == true) {
    if (doorPositionFromClose < 0) {
      doorPositionFromClose = 0;
    }
    printf("operations----> from close = %f\n", doorPositionFromClose);
    printf("operations ----> close_to_open_angle =%f\n ", calib.close_to_open_angle);
    printf("operations ----> startIMMUValue =%f\n ", startIMMUValue);
    printf("operations ----> imu_correction =%d\n ", imu_correction);

    openValue = startIMMUValue + (calib.close_to_open_angle - doorPositionFromClose)
        - imu_correction;
    printf("operations ----> openValue =%f\n ", openValue);

    if (linear_IMUOpen < openValue) {
      printk("Opening \n");

      if (current_door_percentage > 50) {
        printk("---------speed = %d------------\n",Gear_motor_params.end_RPM/2);
        Speed(Gear_motor_params.start_RPM, Gear_motor_params.end_RPM/2,
        Gear_motor_params.gearbox_ratio, Gear_motor_params.micro_steps, 
        Gear_motor_params.step_angle, Gear_motor_params.bw_start_RPM, 
        Gear_motor_params.end_RPM/2);
      }

      uc_staginate_in_accel = operation_cmd = rotateMotor_opr(command, 
          &Gear_motor_params, config_pwm_percent, calib.close_to_open_motor_direction,
          DRIVE_MOTOR_enable, Disable_TruePWM);
    }
  } else {
    if (doorPositionFromClose < 0) {
      doorPositionFromClose = 0;
    }
    printf("operations----> from close = %f\n", doorPositionFromClose);
    printf("operations ----> close_to_open_angle =%f \n", calib.close_to_open_angle);
    printf("operations ----> startIMMUValue =%f \n", startIMMUValue);

    openValue = startIMMUValue - (calib.close_to_open_angle - doorPositionFromClose) + imu_correction;

    printf("operations ----> openValue =%f \n", openValue);
    printf("operations ----> imu_correction =%d\n ", imu_correction);

    if (linear_IMUOpen > openValue) {
      printk("Opening \n");

      if (current_door_percentage > 50) {
        printk("----------speed = %d----------------\n",Gear_motor_params.end_RPM/2);
        Speed(Gear_motor_params.start_RPM, Gear_motor_params.end_RPM/2,
            Gear_motor_params.gearbox_ratio, Gear_motor_params.micro_steps,
            Gear_motor_params.step_angle, Gear_motor_params.bw_start_RPM,
            Gear_motor_params.end_RPM/2);
      }

      uc_staginate_in_accel = operation_cmd = rotateMotor_opr(command,
          &Gear_motor_params, config_pwm_percent,
          calib.close_to_open_motor_direction,
          DRIVE_MOTOR_enable, Disable_TruePWM);
    }
  }

  restart_or_not = 0;
  operation_status_check.state = OPENING_ANDROID;

  if (operation_cmd == 0 || operation_cmd == 200 ) {
    operation_cmd = OPEN;
  }

  // Change the display page to door opening
  chg_display_page(DISP_DOOR_OPENING_PAGE);

  // Loop until the open_position is reached

  uint8_t ret=0,u_sec=0;    //added by ashok
  unsigned long int time_stamp=0;
  while (true)
  {

    if (calib.is_direction_cw)
    {
      if (!((linear_IMUOpen <= (float)(openValue)) && (operation_cmd == OPEN)))
      {
        //flag_open=-1;
        printf("----------------------loop is breaking here--------------------------- linear_IMUOpen %f openValue %f is_direction_cw true\n",linear_IMUOpen,openValue);
        is_bot_staginated = false;
        break;
      }
    }
    else
    {
      if (!((linear_IMUOpen >= (float)(openValue)) && (operation_cmd == OPEN))) 
      {
        //flag_open=-1;
        printf("----------------------loop is breaking here---------------------------- linear_IMUOpen %f openValue %f is_direction_cw false\n",linear_IMUOpen,openValue);
         is_bot_staginated = false;
        break;
      }
    }
    curr_IMU_Val = linear_IMUOpen = getlinear_OPEN_CLOSE(Enable_open, openValue);

    // Debug print
    if (!is_bot_staginated)
      printk("\nChecking current_door_percentage: %d\n", current_door_percentage);

    if ((abs(prev_door_percent - current_door_percentage) < 1)) {
      staginate_count++;
    } else {
      if (staginate_count > 2 ) {
        // printf("\n\\\\staginate count reset");
        staginate_count = 0;
      }
    }
    if (staginate_count > 3 || uc_staginate_in_accel == 200 || reverse_move) { 
      /* 200 taken as to dicide staginate happen during acceleration.
          value should not be match with operation cpmmand values so taken 200 */
      if (current_door_percentage >= FW_DOOR_DECEL_START_PERCNT && staginate_count < 10) {
        // staginate after count > 9
      } else {
        reverse_move = false;
        staginate_count = 0;
        is_bot_staginated = true;
        is_bot_staginated2 = true;

        if ( instal_start_cmd > 0) {
          break;
        }; // recived installation start cmd while door in stagination position
      }
    }

    //------------------------OPEN TRY LOGIC START----------------------------//

    if (is_bot_staginated ||  current_door_percentage <= 2) {
      open_try_time++;//it will increment every 50ms
      if ((abs(door_refrence - current_door_percentage) > 2) && door_refrence != 0) { 
      // reset retry time if door moving by hand after staginate
        open_try_time = 0;
        door_refrence = current_door_percentage;
        //printf("\n auto retry timer resetted\n");
      }

      if (stop_motor) {
        printf("\n====current_door_percentage after accel = %d open_try_count = %d ======\n\n",
            current_door_percentage, open_try_count + 1);
        Stop_Motor_o(&Gear_motor_params, DRIVE_MOTOR_forward, DRIVE_MOTOR_disable, Disable_TruePWM);
        door_refrence = current_door_percentage;
        open_try_count++;
        stop_motor = false;

        if (open_try_count >= 3) {
          is_bot_staginated = true;
          is_bot_staginated2 = true;
          is_door_in_open = 1;  // used for autoclose after 2 min
          printf("\n==========================staginate after open_try_count reaches \n");
          break;
        }
      }
      if (open_try_time >= 100) {     //open_try_time reaches for 5 sec with 100 count
        if (open_try_count < 3) {     //try to open config times
          curr_IMU_Val = linear_IMUOpen = getlinear_OPEN_CLOSE(Enable_open, openValue);
          is_bot_staginated = false;
          if (current_door_percentage > 50) {
            printk("-----------speed = %d--------------------\n",Gear_motor_params.end_RPM/2);
            Speed(Gear_motor_params.start_RPM, Gear_motor_params.end_RPM/2,
              Gear_motor_params.gearbox_ratio, Gear_motor_params.micro_steps,
              Gear_motor_params.step_angle, Gear_motor_params.bw_start_RPM,
              Gear_motor_params.end_RPM/2);
          }
          deceleration = true;
          if (is_lock_config && is_door_locked) {
            enum_lock_status = EM_LOCKED_FALSE;
            Lock_sendCmdACK(&enum_lock_status, 1);
            printk("\n waiting for response from lock");
            k_sleep(K_MSEC(500));
          }

          uc_staginate_in_accel = operation_cmd = rotateMotor_opr(command,
              &Gear_motor_params, config_pwm_percent, calib.close_to_open_motor_direction,
              DRIVE_MOTOR_enable, Disable_TruePWM);
          staginate_count = 0;

          if (operation_cmd == 0 || operation_cmd == 200) {
            operation_cmd = OPEN;
          }
          stop_motor = true;
          open_try_time = 0;
        } else {
          stop_motor = true;
          is_bot_staginated = false;
          staginate_count = 0;
        }
      }
    }
    //------------------------OPEN TRY LOGIC END-------------------------//

    if (current_door_percentage >= FW_DOOR_DECEL_START_PERCNT && deceleration) {
      staginate_count = 0;
      printk("\n==> deceleration_interval start");
      deceleration_interval_opr(&Gear_motor_params, true);
      deceleration = false;
      Gear_motor_params.decel_start_interval = Gear_motor_params.current_interval;
    }

    if (!is_bot_staginated) //debug
      printf("Operations -> IMU = %f, USS Range = %f, Speed = %f,"
          "openValue = %f current_door_percentage= %d\n",
          linear_IMUOpen, uss_range, speed, openValue, 
          current_door_percentage);  //Uttam

    //retrieve and delete the message if received
    k_mbox_get(&cmd_mgr_mb, &recv_msg, &cmd, K_MSEC(50));
    if (cmd > 0) {
      operation_cmd = cmd;
      if (is_bot_staginated && operation_cmd == OPEN ) {
        printf("\n=>same cmd recived in open in stagination");
        break;
      }
      cmd = 0;
    }

    if (prev_door_percent > current_door_percentage) {
      printf("\n=>reverse_move true  prev_door_percent %d current_door_percentage %d\n",
          prev_door_percent, current_door_percentage);
      reverse_move = true;
    }
    prev_door_percent = current_door_percentage;
  }
  operation_flag = false;

  // IMP : Stop the motor

  if (uc_staginate_in_accel == 200) {
    printk("\n==>stagination in acceleration");
    is_bot_staginated = true;
    is_bot_staginated2 = true;
  }

  if (operation_cmd == OPEN || (operation_cmd == OPEN && uc_staginate_in_accel == 200)) { 
    //if other command it should not lock in opening
    operation_cmd = motor_lock_o(&Gear_motor_params);
    operation_cmd = OPEN;
    Stop_Motor_o(&Gear_motor_params, DRIVE_MOTOR_forward, DRIVE_MOTOR_disable,
        Disable_TruePWM);
    printk("*****************************************************Stoped\n");
  } else {
    // gpio_pin_set(motor_params->enable, motor_params->settings.en_pin, 0);
    ret = pwm_pin_set_usec(Gear_motor_params.pwm, Gear_motor_params.settings.pwm_channel, 
        0, 0, Gear_motor_params.settings.pwm_flags);
    k_msleep(10);
  }

  // printk("\n*************operation_cmd %d  **********************\n",operation_cmd);

  // Show startup page on display
  chg_display_page(DISP_START_PAGE);
  Speed(Gear_motor_params.start_RPM, Gear_motor_params.end_RPM,
      Gear_motor_params.gearbox_ratio, Gear_motor_params.micro_steps,
      Gear_motor_params.step_angle, Gear_motor_params.bw_start_RPM,
      Gear_motor_params.bw_end_RPM);

  // Check if the open was interrupted in between by any other received operation command
  if (operation_cmd != OPEN || cmd == OPEN) {
    // Change the state of operation
    //operation.state = OPEN_INTERRUPTED;
    is_bot_staginated = false;   // if cmd == OPEN  recived same cmd while staginate
    is_bot_staginated2 = false;
    return operation_cmd;
  }

  // Report if the door open was interrupted by USS detected any obstacle
  if (operation.state == OPEN_INTERRUPTED) {
    // Notify that the Open Operation was interrupted in between
    // 1. Prepare the notify message
    evt_msg[0] = (int8_t) current_door_percentage;
    evt_msg[1] = OPR_USS_INTRPT;
    evt_msg_len = 2;

    //2. Notify Over BLE that the door is opened
    ADO_notify_ble(cmd_mgr_thrd_id, ADO_OPERATIONS_RESPONSE, OPEN, OPR_INTERRUPTED,
        evt_msg, evt_msg_len);
    return 0;
    //TODO: Define a proper error code and handle it while working on retry logic
  }

  // Change the state of operation
  operation.state = FULLY_OPEN;
  restart_or_not = 0;
  operation_status_check.state = OPENED_ANDROID;
  // Notify Over BLE that the door is opened
  if (!is_bot_staginated) {
    ADO_notify_ble(cmd_mgr_thrd_id, ADO_OPERATIONS_RESPONSE, OPEN,
    OPENED, NULL, 0);
    #if UWB
    distance_lessthan200mtr=4;
    #endif
  }
  return 0;
}

/*
 * Brief: This routine takes care of Closing the Door when CLOSE 
   command is received from ED.
 */
uint8_t CloseTheDoor(uint8_t command) {
  int i = 0,  x = 0;
  bool deceleration = true, stag_at_close = false;
  float speed = config_pwm_percent;
  char *thread_state = NULL;
  int staginate_count = 0, door_refrence = 0;
  //bool stag=true;
  is_bot_staginated = false;
  is_door_in_open = 0;
  close_opr_done = false;
  //accelaration=false;
  int ret = 0, u_sec = 0, t = 0;

  float Cur_IMU = 0.0f, Prev_IMU = 0.0f, temp = 0.0f;
  bool  stop_motor = true, reverse_move = false;
  int close_try_count = 0, close_try_time = 0;

  is_bot_staginated = false;
  is_door_in_open = 0;
  //Initialise to CLOSE to keep checking if the new command received is something else
  uint8_t operation_cmd = CLOSE, cmd = 0, uc_staginate_in_accel = 0;
  operation_flag = true;

  // Mailbox variables
  struct k_mbox_msg recv_msg;

  // Response data variables
  uint8_t evt_msg[2] = {0};
  uint8_t evt_msg_len = 0;

  // Set initial percentage
  current_door_percentage = 100;

  // added for deceleration speed
  float closeValue = 0;
  // percentage stagnant variables
  float scaleFactor = 100.0 / calib.close_to_open_angle;
  scaleFactor = DOOR_DIFF_PERCENT * scaleFactor;

  // Check if the doorbot is installed and calibrated
  if (is_doorbot_uninstalled_uncalibrated() || (ADO_get_restart_state() == true)) {
    // debug print
    restart_or_not = 0;
    operation_status_check.state = ERROR_OPERATION_ANDROID;
    printk("doorbot is uninstalled or uncalibrated\n");
    return error_code; // TODO Return some error code instead
  }

  // Get the corrected Door Close Position IMU readings
  linear_IMUClose = getlinear_OPEN_CLOSE(Enable_close, 999);
  int8_t prev_door_percent = 0;
  float startIMMUValue = linear_IMUClose;
  pev_IMU_Val = linear_IMUOpen;

  // Set the current state of operations
  operation.state = CLOSING_DOOR;

  // Notify Over BLE that the door is closing
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_OPERATIONS_RESPONSE, CLOSE,
      CLOSING, NULL, 0);

  // Change the display page to door closing
  chg_display_page(DISP_DOOR_CLOSING_PAGE);

  // prepare the mailbox receive buffer
  recv_msg.info = 0;
  recv_msg.size = sizeof(operation_cmd);
  recv_msg.rx_source_thread = cmd_mgr_thrd_id;

  // verify Open YAW Cross if cross set flag.
  pev_IMU_Val = linear_IMUClose;
  if (abs(pev_IMU_Val - calib.pos_open) <= 30) {
    Close_YAW = false;
  } else {
    if ((360 - abs(pev_IMU_Val - calib.pos_open)) <= 30) {
      Close_YAW = true;
    } else {
      // TODO genenrate error report to edge device
    }
  }
  current_door_percentage = 100 - current_door_percentage;
  if (calib.is_direction_cw == true) {
    printf("operations----> from close = %f\n", doorPositionFromClose);
    printf("operations ----> close_to_open_angle =%f ", calib.close_to_open_angle);
    printf("operations ----> startIMMUValue =%f ", startIMMUValue);
    closeValue = startIMMUValue - (doorPositionFromClose);
    printf("operations ----> closeValue =%f ", closeValue);
    if (linear_IMUClose > closeValue) {
      printk("Closing\n");

      if (current_door_percentage > 50) {
        printk("---------speed = %d--------\n",Gear_motor_params.end_RPM/2);
        Speed(Gear_motor_params.start_RPM, Gear_motor_params.end_RPM/2,
            Gear_motor_params.gearbox_ratio, Gear_motor_params.micro_steps,
            Gear_motor_params.step_angle, Gear_motor_params.bw_start_RPM,
            Gear_motor_params.end_RPM/2);
      }
      uc_staginate_in_accel = operation_cmd = rotateMotor_opr(command,
          &Gear_motor_params, speed, !calib.close_to_open_motor_direction,
          DRIVE_MOTOR_enable, Disable_TruePWM);
    }
  } else {
    printf("operations----> from close = %f", doorPositionFromClose);
    printf("operations ----> close_to_open_angle =%f ", calib.close_to_open_angle);
    printf("operations ----> startIMMUValue =%f ", startIMMUValue);
    closeValue = startIMMUValue + (doorPositionFromClose);
    printf("operations ----> closeValue =%f ", closeValue);
    if (linear_IMUClose < closeValue) {
      printk("Closing\n");

      if (current_door_percentage > 50) {
        printk("-------------speed = %d-------------\n",Gear_motor_params.end_RPM/2);
        Speed(Gear_motor_params.start_RPM, Gear_motor_params.end_RPM/2,
            Gear_motor_params.gearbox_ratio, Gear_motor_params.micro_steps,
            Gear_motor_params.step_angle, Gear_motor_params.bw_start_RPM,
            Gear_motor_params.end_RPM/2);
      }
      uc_staginate_in_accel =  operation_cmd = rotateMotor_opr(command,
          &Gear_motor_params, speed, !calib.close_to_open_motor_direction,
          DRIVE_MOTOR_enable, Disable_TruePWM);
    }
  }

  restart_or_not = 0;
  operation_status_check.state = CLOSING_ANDROID;
  
  if (operation_cmd == 0 || operation_cmd == 200) {
    operation_cmd = CLOSE;
  }
  
  // Change the display page to door closing
  chg_display_page(DISP_DOOR_CLOSING_PAGE);
  // Loop untill the close_position is reached
  unsigned long int time_stamp = 0;
  Prev_IMU = Cur_IMU = G_IMU;
  
  while (true) {
    if (calib.is_direction_cw) {
      if (!((linear_IMUClose >= (float)(closeValue)) && (operation_cmd == CLOSE))
          || doorPositionFromClose < 2.0f) {
        flag_close = -1;
        if (doorPositionFromClose < 2.0f) {
          door_close_to_open_percentage();
          current_door_percentage = 100 - current_door_percentage;
          
          if (current_door_percentage >= 0 && current_door_percentage <= 99) {
            printf("\n @@@closed at current_door_percentage = %d", current_door_percentage);
            imu_correction = 100 - current_door_percentage;
          }
          doorPositionFromClose = 0;
        }
        printf("----------------------loop is breaking here--------------------------"
            "linear_IMUClose %f t closeValue %f is_direction_cw true\n",
            linear_IMUClose, closeValue); //Uttam
        is_bot_staginated = false;
        break;
      }
    } else {
      if (!((linear_IMUClose <= (float)(closeValue)) &&
          (operation_cmd == CLOSE)) || doorPositionFromClose < 2.0f) {
        flag_close = -1;
        if (doorPositionFromClose < 2.0f) {
          door_close_to_open_percentage();
          current_door_percentage = 100 - current_door_percentage;
          if (current_door_percentage >= 0) {
            printf("\n @@@closed at current_door_percentage = %d", current_door_percentage);
            imu_correction = 100 - current_door_percentage;
          }
          doorPositionFromClose = 0;
        }
        printf("----------------------loop is breaking here--------------------------"
            "linear_IMUClose %f t closeValue %f is_direction_cw false\n",
            linear_IMUClose, closeValue); //Uttam
        is_bot_staginated = false;
        break;
      }
    }

    curr_IMU_Val = linear_IMUClose = getlinear_OPEN_CLOSE(Enable_close, closeValue);

    if (!is_bot_staginated)
      printk("\nChecking current_door_percentage: %d\n", current_door_percentage);
    current_door_percentage = 100 - current_door_percentage;

    if (current_door_percentage >= 99)
      gpio_pin_set(Gear_motor_params.enable, Gear_motor_params.settings.en_pin, 1);

    if (doorPositionFromClose < 0) {
      doorPositionFromClose = 0;
    }
    // Debug print
    //------------------------STAGINATION LOGIC START--------------------------//
    Cur_IMU = G_IMU;
    temp = fabs(Prev_IMU - Cur_IMU);
    if ((temp > 0) && (temp <= 0.5)) {
      staginate_count++;
      //  printf("\n////////> staginate count %d\n",staginate_count);
    } else {
      if (staginate_count > 2 || (abs(prev_door_percent - current_door_percentage) > 0)) {
        //   printf("\n\\\\staginate count reset");
        staginate_count = 0;
      }
    }
    if (staginate_count > 3 || uc_staginate_in_accel == 200 || reverse_move) {
      /* 200 taken as to dicide staginate happen during acceleration.
          value should not be match with operation cpmmand values so taken 200 */
      staginate_count = 0;
      reverse_move = false;
      is_bot_staginated = true;
      is_bot_staginated2 = true;
      
      if ( instal_start_cmd > 0) {
        break;
      };
    }

    if (((current_door_percentage >= 96 && current_door_percentage < 99)
        && staginate_count >= 2)) {
      printf("\n\n ***************staginate when doo % > 96\n");//debug
      is_bot_staginated = false; // to send close  notification
      stag_at_close = true;
      break;
    }

    //--------------------------STAGINATION LOGIC END-------------------------//
    //--------------------------CLOSE TRY LOGIC START-------------------------//
    if (is_bot_staginated) {
      close_try_time++;//it will increment every 50ms
      if ((abs(door_refrence - current_door_percentage) > 2) && door_refrence != 0) {
        // reset retry time if door moving by hand after staginate
        close_try_time = 0;
        door_refrence = current_door_percentage;
      }

      if (stop_motor) {
        printf("\n================================current_door_percentage after accel"
            " = %d close_try_count = %d ======================\n\n",
            current_door_percentage, close_try_count + 1); //Uttam
        Stop_Motor_o(&Gear_motor_params, DRIVE_MOTOR_backward, DRIVE_MOTOR_disable,
            Disable_TruePWM);
        door_refrence = current_door_percentage;
        close_try_count++;
        stop_motor = false;
        if (close_try_count >= 3) {
          is_bot_staginated = true;
          is_door_in_open = 1;
          printf("\n====================staginate after close_try_count reaches \n");
          break;
        }
      }
      if (close_try_time >= 100) {
        //close_try_time reaches for 10 sec with 100 count // each loop 100 ms
        if (close_try_count < 3) {
          //try to open config times
          curr_IMU_Val = linear_IMUClose = getlinear_OPEN_CLOSE(Enable_close, closeValue);
          current_door_percentage = 100 - current_door_percentage;
          is_bot_staginated = false;

          if (current_door_percentage > 50) {
            printk("------------speed = %d----------\n",Gear_motor_params.end_RPM/2);
            Speed(Gear_motor_params.start_RPM, Gear_motor_params.end_RPM/2,
                Gear_motor_params.gearbox_ratio, Gear_motor_params.micro_steps,
                Gear_motor_params.step_angle, Gear_motor_params.bw_start_RPM,
                Gear_motor_params.end_RPM/2);
          }
          deceleration = true;
          uc_staginate_in_accel =  operation_cmd = rotateMotor_opr(command,
              &Gear_motor_params, speed, !calib.close_to_open_motor_direction,
              DRIVE_MOTOR_enable, Disable_TruePWM);
          staginate_count = 0;

          if (operation_cmd == 0 || operation_cmd == 200 ) {
            operation_cmd = CLOSE;
          }
          stop_motor = true;
          close_try_time = 0;
        } else {
          stop_motor = true;
          is_bot_staginated = false;
          staginate_count = 0;
        }
      }
    }
    //--------------------CLOSE TRY LOGIC END---------------------------//
    if (current_door_percentage >= BW_DOOR_DECEL_START_PERCNT && deceleration) {
      staginate_count = 0;
      printk("\n==>deceleration_interval start\n");
      deceleration_interval_opr(&Gear_motor_params, false);
      Gear_motor_params.decel_start_interval = Gear_motor_params.current_interval;
      deceleration = false;
      current_door_percentage = 100 - current_door_percentage;
      //current_door_percentage is updating in deceleration_interval_opr
    }
    if (!is_bot_staginated)
      printf("Operations -> IMU = %f, USS Range = %f, Speed = %f, closeValue"
          " = %f current_door_percentage =%d,doorPositionFromClose= %f\n",
          linear_IMUClose, uss_range, speed, closeValue, current_door_percentage,
          doorPositionFromClose); //Uttam

    k_mbox_get(&cmd_mgr_mb, &recv_msg, &cmd, K_MSEC(50));
    if (cmd > 0) {
      operation_cmd = cmd;
      if (is_bot_staginated && operation_cmd == CLOSE ) {
        printf("\n @@@@@@@@ same cmd recived in close");
        break;
      }
      cmd = 0;
    }
    if ((prev_door_percent > current_door_percentage) && current_door_percentage < 90) {
      printf("\n===============================reverse_move true  prev_door_percent %d"
          " current_door_percentage %d\n", prev_door_percent, current_door_percentage);
      reverse_move = true;
    }
    prev_door_percent = current_door_percentage;
    Prev_IMU = Cur_IMU; //check staginate end
  }
  operation_flag = false;
  // IMP : Stop the motor
  if (uc_staginate_in_accel == 200) {
    printk("\n==>stagination in acceleration");
    is_bot_staginated = true;
  }
  if (operation_cmd == CLOSE || (operation_cmd == CLOSE && uc_staginate_in_accel == 200)) { 
    //if other command it should not lock in closing
    operation_cmd = motor_lock_o(&Gear_motor_params);
    operation_cmd = CLOSE;
    Stop_Motor_o(&Gear_motor_params, DRIVE_MOTOR_backward, DRIVE_MOTOR_disable, Disable_TruePWM);
    printk("********************************************************Stoped\n");
  } else {
    //  gpio_pin_set(motor_params->enable, motor_params->settings.en_pin, 0);
    ret = pwm_pin_set_usec(Gear_motor_params.pwm, Gear_motor_params.settings.pwm_channel,
        0, 0, Gear_motor_params.settings.pwm_flags);
    k_msleep(10);
  }

  // Show startup page on display
  //chg_display_page(DISP_START_PAGE);
  Speed(Gear_motor_params.start_RPM, Gear_motor_params.end_RPM,
      Gear_motor_params.gearbox_ratio, Gear_motor_params.micro_steps,
      Gear_motor_params.step_angle, Gear_motor_params.bw_start_RPM,
      Gear_motor_params.bw_end_RPM);
  // Check if the open was interrupted in between
  // k_mbox_get(&cmd_mgr_mb, &recv_msg, &operation_cmd, K_MSEC(50));
  // Commented since it is wrong here as we already checked the MB Message in loop

  if (doorPositionFromClose < 0.0f || stag_at_close) {
    doorPositionFromClose = 0.0f;
  }
  chg_display_page(DISP_SAY_DOOR_OPEN_PAGE);

  // Check if the open was interrupted in between
  if (operation_cmd != CLOSE || cmd == CLOSE) {
    // Change the state of operation
    //operation.state = CLOSE_INTERRUPTED;
    //   // Change the state of operation
    is_bot_staginated = false;
    return operation_cmd;
  }

  // Report if the door close was interrupted by ADO stagnation
  if (operation.state == CLOSE_INTERRUPTED) {
    // Notify that the Close Operation was interrupted in between
    // 1. Prepare the notify message
    evt_msg[0] = (int8_t) current_door_percentage;
    evt_msg[1] = OPR_STAGNANT_INTRPT;
    evt_msg_len = 2;

    //2. Notify Over BLE that the door is NOT Completely closed
    ADO_notify_ble(cmd_mgr_thrd_id, ADO_OPERATIONS_RESPONSE, CLOSE,
        OPR_INTERRUPTED, evt_msg, evt_msg_len);
    flag_close = 1;
    k_timer_start(&timer_close, K_MSEC(3000), K_MSEC(0));
    return 0; //TODO: Define a proper error code and handle it while working on retry logic
  }

  // Change the state of operation
  operation.state = FULLY_CLOSED;
  restart_or_not = 0;
  operation_status_check.state = CLOSED_ANDROID;
  // Notify Over BLE that the door is closed
  if (!is_bot_staginated) {
    ADO_notify_ble(cmd_mgr_thrd_id, ADO_OPERATIONS_RESPONSE, CLOSE, CLOSED, NULL, 0);
    #if UWB
    distance_lessthan200mtr=5;
    #endif
  }
  // Prevent opening the door via intent thread when it was 'just closed'.
  if (is_Intent_Enabled() == true) {
    // Check if the intent thread is running, if so send it an INT_STOP command
    thread_state = k_thread_state_str(ado_intent_thrd_id,0,0);
    //debug prints
    //printk("Intent thread state: %s\n", thread_state);

    if (!is_thread_dead(thread_state)) {
      //send an INT_STOP command to cmd_mgr
      //ADO_notify_cmd_mgr(COMMAND, ADO_INTENT_REQUEST, INT_STOP, NULL, 0 );

      // temporary added
      // Change the Display Page to "say-Open-Door".
      chg_display_page(DISP_SAY_DOOR_OPEN_PAGE);
    }
  }
  //wait for 1 sec to fully drop down inertia to correct negtive values in main thread
  cmd = 0 ;
  k_mbox_get(&cmd_mgr_mb, &recv_msg, &cmd, K_MSEC(1000));
  if (doorPositionFromClose < 0.0f) {
    doorPositionFromClose = 0.0f;
  }
  if (cmd > 0) {
    return cmd;
  }

  close_opr_done = true;
  return 0;
}

/*
 * Brief: This routine takes care of making sure drive motor was stoped
   when STOP command is received from ED.
 */
uint8_t StopTheDoor() {
  int res=0;
  // Check if motor is running then stop it.
  if (isMotorRunning(&Gear_motor_params) == true) {
    // decelerate the profile then stop
     res = decelerate_for_stop(&Gear_motor_params);

     if(res > 0) {
       return res;
     }
    // Stop the motor
    res=motor_lock_o(&Gear_motor_params);
    Stop_Motor_o(&Gear_motor_params, DRIVE_MOTOR_backward, DRIVE_MOTOR_disable,
        Disable_TruePWM);
    
    printk("\n res=%d in after motor lock\n",res);
    if(res>0) {
      return res;
    }
  }

  // Notify Over BLE that the door is stopped
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_OPERATIONS_RESPONSE, OPR_STOP,
      OPR_STOPPED, NULL, 0);
  return 0;
}

/*
 *  Dynamic door percentage calculation.
 *  Brief: This function calculates current open/close
    percentage position of door from close to open.
 */
void door_close_to_open_percentage() {
  // Check door close to open position angle
  if (calib.close_to_open_angle <= 0) {
    //printk(" Open angle zero. Calibrate door again...\n");
    return;
  }
  current_door_percentage = (doorPositionFromClose / calib.close_to_open_angle) * 100;
  //door_percentage=current_door_percentage;
}

/*
 *  Brief: This function is to move the clutch motor to engage it 
    for the drive motor to run wheel.
 */
int8_t engageClutch() {
  uint8_t next_cmd = 0;
  uint8_t clutch_wait_cycle_cnt = 0U;

  // Mailbox variables
  struct k_mbox_msg recv_msg;

  // Notify over BLE that clutch is Engaging
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_OPERATIONS_RESPONSE, CLUTCH_ENGAGE,
      CLUTCH_ENGAGE_RUNNING, NULL, 0);

  // prepare the mailbox receive buffer
  recv_msg.info = 0;
  recv_msg.size = sizeof(next_cmd);
  recv_msg.rx_source_thread = cmd_mgr_thrd_id;

  // added for proper clutch engage functionality
  rotateMotor(&Gear_motor_params, DRIVE_MOTOR_CLUTCH_ENGAGE_PWM,
      DRV_MOTOR_FORWARD, DRIVE_MOTOR_enable, Enable_TruePWM);
  clutch_check_var = CLUTCH_ENGAGING_DEV_STAT;
  printk("Engaging Clutch\n");
  // TODO: Add the logic to precisely stop the clutch motor based on time or current spike,
  while ((clutch_wait_cycle_cnt < MAX_CLUTCH_ENG_WAIT_CYCLE)) {
    // retrieve and delete the message if received
    k_mbox_get(&cmd_mgr_mb, &recv_msg, &next_cmd, K_MSEC(50));

    // optionally added later need to change
    nrf_gpio_pin_clear(CLUTCH_F);
    nrf_gpio_pin_set(CLUTCH_B);

    ++clutch_wait_cycle_cnt;
    if(next_cmd == OPR_STOP) {
      break;
    }
  }
  
  Stop_Motor(&Gear_motor_params, DRIVE_MOTOR_backward,
      DRIVE_MOTOR_disable, Disable_TruePWM);

  // TODO Stop clutch motion
  nrf_gpio_pin_set(CLUTCH_F);
  nrf_gpio_pin_set(CLUTCH_B);
  
  printk("Stoped=======\n");
  clutch_check_var = CLUTCH_ENGAGED_DEV_STAT;
  save_operation_data();
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_OPERATIONS_RESPONSE, CLUTCH_ENGAGE,
      CLUTCH_ENGAGED, NULL, 0);
    
  if (next_cmd == 0) {
    // Notify over BLE that clutch is Engaged
    return OPR_STOP;
  }
  return next_cmd; // return the newly read cmd (if any) otherwise 0
}

/*
 *  Brief: This function is to move the clutch motor to disengage it 
    from the drive motor to free the wheel.
 */
int8_t disengageClutch() {
  uint8_t next_cmd = 0;
  uint8_t clutch_wait_cycle_cnt = 0U;

  bool motor_direction = DRV_MOTOR_FORWARD;
  int i = 0;

  // Mailbox variables
  struct k_mbox_msg recv_msg;

  // Notify over BLE that clutch is Disengaging
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_OPERATIONS_RESPONSE, CLUTCH_DISENGAGE,
      CLUTCH_DISENGAGE_RUNNING, NULL, 0);

  // prepare the mailbox receive buffer
  recv_msg.info = 0;
  recv_msg.size = sizeof(next_cmd);
  recv_msg.rx_source_thread = cmd_mgr_thrd_id;

#ifdef ENABLE_DIS_ANGAGE_PWM
  // added for proper clutch engage functionality
  rotateMotor(&Gear_motor_params, DRIVE_MOTOR_CLUTCH_DIS_ENGAGE_PWM,
      DRV_MOTOR_FORWARD, DRIVE_MOTOR_enable, Enable_TruePWM);
#endif

  // NOTE: added clutch testing.
  k_msleep(400U);
  Stop_Motor(&Gear_motor_params, DRIVE_MOTOR_backward, DRIVE_MOTOR_disable, Disable_TruePWM);

  printk("Dis-Engaging Clutch\n");
  clutch_check_var = CLUTCH_DISENGAGING_DEV_STAT;
  /* Add the logic to precisely stop the clutch motor based on time or current spike, 
    presently stop it only on STOP cmd */
  while ((clutch_wait_cycle_cnt < MAX_CLUTCH_DIS_ENG_WAIT_CYCLE)) {
    // retrieve and delete the message if received
    k_mbox_get(&cmd_mgr_mb, &recv_msg, &next_cmd, K_MSEC(50));

    // TODO: Code to Engage Clutch motor
    // optionally added later need to change
    nrf_gpio_pin_clear(CLUTCH_B);
    nrf_gpio_pin_set(CLUTCH_F);
    ++clutch_wait_cycle_cnt;

    if(next_cmd == OPR_STOP) {
      break;
    }
  }

  Stop_Motor(&Gear_motor_params, DRIVE_MOTOR_backward, DRIVE_MOTOR_disable, Disable_TruePWM);

  // TODO Stop clutch motion
  nrf_gpio_pin_set(CLUTCH_F);
  nrf_gpio_pin_set(CLUTCH_B);
  
  clutch_check_var = CLUTCH_DISENGAGED_DEV_STAT;
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_OPERATIONS_RESPONSE, CLUTCH_DISENGAGE,
      CLUTCH_DISENGAGED, NULL, 0);
  save_operation_data();
  printk("Stoped=======\n");
  
  if (next_cmd == 0) {
    // Notify over BLE that clutch is Engaged
    return OPR_STOP; 
  }
  return next_cmd; // return the newly read cmd (if any) otherwise 0
}

/*
 *  Brief: This function is to Set the LED Strip ON/OFF as per received led_cmd,
    It's just a placeholder now.
 */
void setLEDStrip(uint8_t led_cmd) {
  // TODO: Write the LED Strip toggling code, just notify over BLE now
  if (led_cmd == ON_LED) { 
    printk("\n------LED ON-----\n");
    //gpio_expander_port_A_pin_set(LED_CONTROL); // Trun ON LED strip power
    
    //WiFi_module_on_off(WIFI_MODULE_ON);

    // Notify over BLE that LED is ON
    ADO_notify_ble(cmd_mgr_thrd_id, ADO_OPERATIONS_RESPONSE, ON_LED, LED_ON, NULL, 0);
  } else if (led_cmd == OFF_LED) {
    printk("\n------LED OFF-----\n");
    // gpio_expander_port_A_pin_clear(LED_CONTROL); // Trun OFF LED strip power
    // WiFi_module_on_off(WIFI_MODULE_OFF);
    // Notify over BLE that LED is ON
    ADO_notify_ble(cmd_mgr_thrd_id, ADO_OPERATIONS_RESPONSE, OFF_LED, LED_OFF, NULL, 0);
  } else {
    printk("Invalid LED strip cmd %d\n", led_cmd);
  }
}

bool is_doorbot_uninstalled_uncalibrated(void) {
  if ((is_ADO_Installed(&clamp_data) == false) || (is_ADO_Calibrated(&calib) == false)) {
    return true;
  }
  return false;
}

/// New logic to solve near 360 or 0 IMU calibration.
///*
// * Brief: This routine takes care of Reading Median IMU Value While the Door Moving
// *        towards Calibrated Closing position or opening position.
// */

float getlinear_OPEN_CLOSE(bool open_close, float destinationIMU) {
  float IMU_Val;
  struct k_mbox_msg temp_msg;
  float temp_data[] = {0, 0};

  temp_msg.info = 0;
  temp_msg.size = sizeof(temp_data);
  temp_msg.rx_source_thread = dev_stat_thrd_id;
  k_mbox_get(&cmd_mgr_mb, &temp_msg, temp_data, K_MSEC(50));
  //printf("mailbox -> IMU = %f, USS = %f\n",temp_data[0],temp_data[1]);
  uss_range = temp_data[1];
 
  if (temp_data[0]) {
    IMU_Val = temp_data[0];
    temp_data[0] = 0;
  } else {
    IMU_Val = G_IMU;   // take directly from global variable
  }

  door_close_to_open_percentage();

  if (destinationIMU == 999) {
    return IMU_Val;
  }

  if (abs(IMU_Val - pev_IMU_Val) >= 180) {
    if (destinationIMU < 0) {
      IMU_Val = IMU_Val - 360;
    } else {
      IMU_Val = 360 + IMU_Val;
    }
  }
  return IMU_Val;
}

int8_t save_operation_data() {
  int error = 0;
  uint8_t data[1];
  data[0] = clutch_check_var;   //1=hor clamped, 2=hor unclamped, 0= inermediate
  
  error = Write_EEPROM(OPERATION_MEMORY_LOCATION, &data[0], 1);

  if(error) {
    printk("\nerror saving operation data to EEPROM trying one more time");
    error = Write_EEPROM(OPERATION_MEMORY_LOCATION, &data[0], 1);
  }
  return error;
}

/*
 * brief: This function is used to read back the saved Installation
   parameters from EEPROM to global installation structure.
 */
int8_t load_operation_data() {
  int error = 0;
  uint8_t data[1];

  error = Read_EEPROM(OPERATION_MEMORY_LOCATION, &data[0], 1);

  if(error) {
    printk("\nerror loading operation data from EEPROM trying one more time");
    error = Read_EEPROM(OPERATION_MEMORY_LOCATION, &data[0], 1);
  }

  clutch_check_var = data[0]; //0=disengaged, 1=engaged, 2=disengaging, 3=engaging, 255=error
  return error;
}

uint8_t OTA_check_operation_status() {
    printk("operation_status_check.state is: %d\n", operation_status_check.state);
    return operation_status_check.state;
}

uint8_t check_operation_status() {
  uint8_t temp = reset_option();
  printk("\nreset state idea: %d, restart_or_not is: %d\n", temp, restart_or_not);
  printk("operation_status_check id: %d\n", operation_status_check.state);

  if(restart_or_not==1) {
    if(temp == 2) {
      return CLOSED_ANDROID;
    }
    
    if(track_calibration_process == 1) {
      return OPENED_ANDROID;
    }
    return OPENED_ANDROID;  //was supposed to return error
  } else {
    printk("operation_status_check.state is: %d\n", operation_status_check.state);
    
    if((operation_status_check.state != OPENED_ANDROID) &&
        (operation_status_check.state != CLOSED_ANDROID)) {
      return OPENED_ANDROID;
    } else {
      return operation_status_check.state;
    }
  }
}

uint8_t check_clutch() {
  return clutch_check_var;
}

void timer_expire_dooropen(struct k_timer *timer) {
  printk("---###   Timer for INTERRUPTING THE DOOR OPENING expired,"
      "initiate the function again   ###---\n");
  flag_open=0;
  opr_cmd= OPEN;
}

void timer_expire_doorclose(struct k_timer *timer) {
  printk("---###   Timer for INTERRUPTING THE DOOR CLOSING expired,"
      "initiate the function again   ###---\n");
  flag_close=0;
  opr_cmd = CLOSE;  
}

/*
 * brief: Rotate or Stop the motor as per the provided parameters,
   the motor_param must be valid
 */
int8_t rotateMotor_opr(uint8_t command, struct motor_params_t *motor_params, 
    uint8_t pwm_percent, bool dir, bool enable, bool true_pwm) {
  int32_t ret = 0, u_sec = 0, cnt = 0, door_refrence = 0;
  float Cur_IMU = 0, Prev_IMU = 0, temp = 0;
  bool stag_flag = true ; //firstime = true,
  uint8_t operation_cmd = 0, staginate_count = 0;
  struct k_mbox_msg recv_msg;

  recv_msg.info = 0;
  recv_msg.size = sizeof(opr_cmd);
  recv_msg.rx_source_thread = cmd_mgr_thrd_id;
  printf("\nfw_Gear_motor_params.end_RPM %d bw_Gear_motor_params.end_RPM %d\n", 
      Gear_motor_params.end_RPM, Gear_motor_params.bw_end_RPM);
  
  // Limit PWM percent to 100%
  if (pwm_percent > MAX_OPR_MOTOR_SPEED_PERCENT) {
    pwm_percent = MAX_OPR_MOTOR_SPEED_PERCENT;
  }

  // Limit PWM percent to not fall beyond MIN_OPR_MOTOR_SPEED_PERCENT in operating conditions
  if (!true_pwm) {
    if (pwm_percent < MIN_OPR_MOTOR_SPEED_PERCENT) {
      pwm_percent = MIN_OPR_MOTOR_SPEED_PERCENT;
    }
  }

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
      printf("\n  fw_Gear_motor_params.end_RPM %d  bw_Gear_motor_params.end_RPM %d\n", 
          Gear_motor_params.end_RPM, Gear_motor_params.bw_end_RPM);
      // Set the PWM Period
      // TODO: Add the functionality of turning off PWM if motor is disabled
      Prev_IMU = G_IMU;

      if (dir) {
        door_refrence = current_door_percentage;
        ret = gpio_pin_set(motor_params->direction, motor_params->settings.dir_pin, 
            GPIO_ACTIVE_HIGH);

        if (ret != 0) {
          printk("Error %d: failed to set  direction\n", ret);
          ret = 0;
        }

        printf("\n==>Acceleration_interval start\n");
        while (Gear_motor_params.current_speed_percentage < 100) {
          u_sec = fw_accrelation_interval_opr();
          ret = pwm_pin_set_usec(motor_params->pwm, motor_params->settings.pwm_channel, 
              u_sec, u_sec / 2U, motor_params->settings.pwm_flags);

          if (ret != 0) {
            printk("Error %d: failed to set pulse width\n", ret);
            ret = 0;
          }

          //chek stag start
          door_close_to_open_percentage();
          Cur_IMU = G_IMU;
          temp = fabs(Prev_IMU - Cur_IMU);

          if ((temp > 0 && temp <= 0.5) && (abs(door_refrence - current_door_percentage) > 5)) {
            staginate_count++;
            printf("\n ===========Accelerate staginate cnt %d\n ", staginate_count );
          }

          if (staginate_count >= 3) {
            staginate_count = 0;
            Gear_motor_params.current_interval = u_sec;
            Gear_motor_params.decel_start_interval = u_sec;
            return 200;
          }

          Prev_IMU = Cur_IMU; //chek stag end
          //chek kmsg start
          //k_mbox_get(&cmd_mgr_mb, &recv_msg, &operation_cmd,K_USEC(3));
          k_mbox_get(&cmd_mgr_mb, &recv_msg, &operation_cmd, K_USEC(u_sec));

          if (operation_cmd != 0 && operation_cmd != command) {
            printk("\nCOMMAND is detected in rotate motor\n");
            printk("operation_cmd = %d", operation_cmd);
            Gear_motor_params.decel_start_interval = u_sec;
            return operation_cmd;
          }  //chek kmsg end
          
          if (current_door_percentage >= 100) {
            printf("\n in accel current_door_percentage reaches to 100\n");
            Gear_motor_params.decel_start_interval = u_sec;
            return operation_cmd;
          }
        }
      } else {
        door_refrence = 100 - current_door_percentage;
        ret = gpio_pin_set(motor_params->direction, motor_params->settings.dir_pin, GPIO_ACTIVE_LOW);
        
        if (ret != 0) {
          printk("Error %d: failed to set  direction\n", ret);
        }
        printf("\n==>Acceleration_interval start\n");
        
        while (Gear_motor_params.bw_current_speed_percentage < 100) {
          u_sec = bw_accrelation_interval_opr();
          ret = pwm_pin_set_usec(motor_params->pwm, motor_params->settings.pwm_channel, 
              u_sec, u_sec / 2U, motor_params->settings.pwm_flags);

          if (ret != 0) {
            printk("Error %d: failed to set pulse width\n", ret);
          }
          //check staginate start
          door_close_to_open_percentage();
          current_door_percentage = 100 - current_door_percentage;
          Cur_IMU = G_IMU;
          temp = fabs(Prev_IMU - Cur_IMU);
          
          if ((temp > 0 && temp <= 0.5) && (abs(door_refrence - current_door_percentage) > 5)) {
            staginate_count++;
            printf("\n ===========Accelerate staginate cnt %d\n ", staginate_count );
          } else {
            if (staginate_count > 2) {
              printf("\n\\\\staginate count reset");
              staginate_count = 0;
            }
          }
          
          if (staginate_count > 3) {
            staginate_count = 0;
            Gear_motor_params.current_interval = u_sec;
            Gear_motor_params.decel_start_interval = u_sec;
            return 200;
          }
          Prev_IMU = Cur_IMU; //check staginate end
          //check k_msg start
          k_mbox_get(&cmd_mgr_mb, &recv_msg, &operation_cmd, K_USEC(u_sec));

          if (operation_cmd != 0 && operation_cmd != command) {
            printk("\nCOMMAND is detected in rotate motor\n");
            printk("operation_cmd = %d", operation_cmd);
            Gear_motor_params.decel_start_interval = u_sec;
            return operation_cmd;
          }   //check k_msg end

          if (current_door_percentage >= 100) {
            printf("\n in accel current_door_percentage reaches to 100\n");
            Gear_motor_params.decel_start_interval = u_sec;
            return operation_cmd;
          }
        }
      }
      Gear_motor_params.current_interval = u_sec;
      Gear_motor_params.decel_start_interval = u_sec;
    }
    printk("\n============>enable = %d isRunning = %d\n", enable, motor_params->isRunning);
  }
  printk("return value before exit rotate motor in operation is %d\n", ret);
  return 0;
}


uint8_t deceleration_interval_opr(struct motor_params_t *motor_params, bool dir) {
  int t = 0, ret = 0;
  bool flag = true;
  uint8_t operation_cmd = 0, staginate_count = 0;
  float Cur_IMU = 0, Prev_IMU = 0, temp = 0;
  struct k_mbox_msg recv_msg;
  recv_msg.info = 0;
  recv_msg.size = sizeof(opr_cmd);
  recv_msg.rx_source_thread = cmd_mgr_thrd_id;
  Prev_IMU = G_IMU;
  
  while (flag) {
    if (dir == true) {
      t += 1;
      
      if (t % 250 == 0) {
        Gear_motor_params.current_interval += 5;
      }
      
      ret = pwm_pin_set_usec(motor_params->pwm, motor_params->settings.pwm_channel,
          Gear_motor_params.current_interval, Gear_motor_params.current_interval / 2U,
          motor_params->settings.pwm_flags);
      
      if (ret != 0) {
        printk("Error %d: failed to set pulse width\n", ret);
        ret = 0;
      }

      k_mbox_get(&cmd_mgr_mb, &recv_msg, &operation_cmd,
          K_USEC(Gear_motor_params.current_interval));

      if (operation_cmd != 0) {
        return operation_cmd;
      }

      door_close_to_open_percentage();

      if (Gear_motor_params.current_interval >= Gear_motor_params.decel_stop_interval
          || current_door_percentage >= 99) {
        flag = false;
        break;
      }
    } else {
      t += 1;
      
      if (t % 250 == 0) {
        Gear_motor_params.bw_current_interval += 5;
      }
      
      ret = pwm_pin_set_usec(motor_params->pwm, motor_params->settings.pwm_channel,
          Gear_motor_params.bw_current_interval, Gear_motor_params.bw_current_interval / 2U,
          motor_params->settings.pwm_flags);
      
      if (ret != 0) {
        printk("Error %d: failed to set pulse width\n", ret);
        ret = 0;
      }

      k_mbox_get(&cmd_mgr_mb, &recv_msg, &operation_cmd,
          K_USEC(Gear_motor_params.bw_current_interval));
      
      if (operation_cmd != 0) {
        return operation_cmd;
      }
      
      door_close_to_open_percentage();
      
      if (Gear_motor_params.bw_current_interval >= Gear_motor_params.bw_decel_stop_interval
          || ( 100 - current_door_percentage) >= 99) {
        flag = false;
        break;
      }
    }
  }
}


int motor_lock_o(struct motor_params_t *motor_params) {
  uint8_t operation_cmd = 0, i = 0; //,cnt=0;
  bool lock = false;
  struct k_timer my_timer;
  struct k_mbox_msg recv_msg;

  recv_msg.info = 0;
  recv_msg.size = sizeof(opr_cmd);
  recv_msg.rx_source_thread = cmd_mgr_thrd_id;

  printk("\nmotor_lock_o is called\n");
  int ret = 0, enable = 0;
  k_timer_init(&my_timer, NULL, NULL);

  for (i = 0; i < 2; i++) {
    k_timer_start(&my_timer, K_MSEC(250), K_MSEC(0));
    if (lock) {
      gpio_pin_set(motor_params->enable, motor_params->settings.en_pin, 0);
      pwm_pin_set_usec(motor_params->pwm, motor_params->settings.pwm_channel,
          0, 0, motor_params->settings.pwm_flags);
      lock = false;
    } else {
      gpio_pin_set(motor_params->enable, motor_params->settings.en_pin, 1);
      lock = true;
    }

    while (!(k_timer_status_get(&my_timer) > 0)) {
      k_mbox_get(&cmd_mgr_mb, &recv_msg, &operation_cmd, K_USEC(50) );

      if (operation_cmd != 0) {
        printk("\nCOMMAND is detected in MOTOR lock\n");
        printk("operation_cmd = %d", operation_cmd);
        k_timer_stop(&my_timer);
        return operation_cmd;
      }
    }
    k_timer_stop(&my_timer);
  }
  k_timer_stop(&my_timer);
  return 0;
}

int Stop_Motor_o(struct motor_params_t *motor_params, bool dir,
    bool enable_disable, bool true_PWM) {
  int ret;
  // printk("\n ===> Stop_Motor called in operations\n");
  reset_accelation_values();
  gpio_pin_set(motor_params->enable, motor_params->settings.en_pin, 1);
  motor_params->isRunning = false;
  return ret;
}

void reset_accelation_values_opr() {
 //// printk("\n===>reset_accelation_values called");
 // Gear_motor_params.current_interval = 0;
 // Gear_motor_params.current_speed_percentage = 0;
 // Gear_motor_params.steps = 0;
 // Gear_motor_params.skip_steps = 200; //forward skip steps (0 to 60%)
 // Gear_motor_params.speed_change_percentage = 12; // forward increment % (0-60%)

 // Gear_motor_params.bw_current_interval = 0;
 // Gear_motor_params.bw_current_speed_percentage = 0;
 // Gear_motor_params.bw_steps = 0;
 // Gear_motor_params.bw_skip_steps =200;// 325; //backward skip steps (0 to 60%)
 // Gear_motor_params.bw_speed_change_percentage = 10;  // backward increment % (0-60%)
}

float fw_accrelation_interval_opr(void) {
  //printk("Forward acceleration interval is called\r\n");
  if ((Gear_motor_params.steps % Gear_motor_params.skip_steps) == 0) {
    if (Gear_motor_params.current_speed_percentage >= 60 &&
        Gear_motor_params.current_speed_percentage <= 100 ) {
      Gear_motor_params.skip_steps = 250;
      Gear_motor_params.speed_change_percentage = 1;
    }
    
    if (Gear_motor_params.current_speed_percentage < 100) {
      //  count_skips_fw++;
      /* printk("current_speed_percentage = %d speed_change_percentage %d",
          Gear_motor_params.current_speed_percentage,Gear_motor_params.speed_change_percentage); */
      Gear_motor_params.current_speed_percentage += Gear_motor_params.speed_change_percentage;
      Gear_motor_params.current_interval = Gear_motor_params.start_interval -
          ((Gear_motor_params.start_interval - Gear_motor_params.end_interval)
           * Gear_motor_params.current_speed_percentage / 100);
      /* printk("\n  start_interval=%d end_interval=%d current_speed_percentage=%d",
          Gear_motor_params.current_interval,Gear_motor_params.start_interval,
          Gear_motor_params.end_interval,Gear_motor_params.current_speed_percentage); */
    } else {
      printk("**************************\n");
      printk("%d", Gear_motor_params.current_interval);
    }
    
    if (Gear_motor_params.steps >= Gear_motor_params.skip_steps * 20) {
      Gear_motor_params.steps = 0;
    }
  }
  Gear_motor_params.steps += 1;
  //printk(", steps = %d",Gear_motor_params.steps);
  return Gear_motor_params.current_interval;
}

float bw_accrelation_interval_opr(void) {
  //printk("backward acceleration interval is called\r\n");
  if ((Gear_motor_params.bw_steps % Gear_motor_params.bw_skip_steps) == 0) {
    if (Gear_motor_params.bw_current_speed_percentage >= 60 &&
        Gear_motor_params.bw_current_speed_percentage <= 100  ) {
      Gear_motor_params.bw_skip_steps = 250;
      Gear_motor_params.bw_speed_change_percentage = 1;
    }

    if (Gear_motor_params.bw_current_speed_percentage < 100) {
      /* printk("current_speed_percentage = %d speed_change_percentage %d",
          Gear_motor_params.bw_current_speed_percentage,
          Gear_motor_params.bw_speed_change_percentage); */
      //count_skips_bw++;
      Gear_motor_params.bw_current_speed_percentage += Gear_motor_params.bw_speed_change_percentage;
      Gear_motor_params.bw_current_interval = Gear_motor_params.bw_start_interval -
          ((Gear_motor_params.bw_start_interval - Gear_motor_params.bw_end_interval) *
          Gear_motor_params.bw_current_speed_percentage / 100);
      /* printk("\n  start_interval=%d end_interval=%d current_speed_percentage=%d",
          Gear_motor_params.bw_current_interval,Gear_motor_params.bw_start_interval,
          Gear_motor_params.bw_end_interval,Gear_motor_params.bw_current_speed_percentage); */
    } else {
      printk("**************************\n");
      printk("%d", Gear_motor_params.bw_current_interval);
    }

    if (Gear_motor_params.bw_steps >= Gear_motor_params.bw_skip_steps * 20) {
      Gear_motor_params.bw_steps = 0;
    }
  }
  Gear_motor_params.bw_steps += 1;
  //printk(", steps = %d",Gear_motor_params.bw_steps);
  return Gear_motor_params.bw_current_interval;
}


uint8_t decelerate_for_stop(struct motor_params_t *motor_params) {
  int t = 0;
  bool flag = true;
  uint8_t operation_cmd = 0;
  struct k_mbox_msg recv_msg;

  recv_msg.info = 0;
  recv_msg.size = sizeof(opr_cmd);
  recv_msg.rx_source_thread = cmd_mgr_thrd_id;
  printf("\nfor_stop decel Gear_motor_params.decel_start_interval %d",
      Gear_motor_params.decel_start_interval);

  while (flag) {
    t += 1;
    if (t % 250 == 0) {
      Gear_motor_params.decel_start_interval += 10;
    }
    pwm_pin_set_usec(motor_params->pwm, motor_params->settings.pwm_channel,
        Gear_motor_params.decel_start_interval,
        Gear_motor_params.decel_start_interval / 2U,
        motor_params->settings.pwm_flags);
    k_mbox_get(&cmd_mgr_mb, &recv_msg, &operation_cmd,
        K_USEC(Gear_motor_params.decel_start_interval));
    
    if (operation_cmd != 0) {
      return operation_cmd;
    }
    
    if (Gear_motor_params.decel_start_interval >= 181 ) {
      printk("\n==> decelerate_for_stop\n %d", Gear_motor_params.decel_start_interval);
      Gear_motor_params.decel_start_interval = 0;
      break;
    }
  }
  return 0;
}