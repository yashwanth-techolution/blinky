#include <math.h>
#include <zephyr/kernel.h>
#include <stdio.h>

#include "Includes/Motor_Control_Module.h"
#include "Includes/ADO_Reset_Calib_Module.h"
#include "Includes/Command_Manager_Module.h"
#include "Includes/Intent_Module.h"
#include "Includes/Device_Status_Module.h"
#include "Includes/Calibration_Module.h"
#include "Includes/ADO_PIR_Module.h"
#include "Includes/ADO_Operations_Module.h"

uint8_t reset_option();
uint8_t reset_variable = 255;

// Mailbox variables
struct k_mbox_msg rst_calib_msg;
ado_reset_calib_cmd_t res_calib_cmd = 0;  // important to initialize

/*
 *  brief: Definition to the reset calibration thread
 */
void ADO_Reset_Calib_Thread(void) {
  //// Mailbox variables
  // struct k_mbox_msg recv_msg;
  // ado_reset_calib_cmd_t res_calib_cmd = 0;  // important to initialize
  int quit=0;

  power_optimization(CALIBRATION);
  const char * thread_state = NULL;
  float new_close_IMU_val = -1.0f;          // initialise it to invalid IMU value first
  
  // Check and abort intent thread if it is enabled
  if(is_Intent_Enabled() == true) { 
    // Check if the intent thread was already running, and suspend it
    thread_state = k_thread_state_str(ado_intent_thrd_id,0,0);
    
    if(!is_thread_dead(thread_state)) {
      k_thread_abort(ado_intent_thrd_id);
    }

    // Disable PIR
    disable_pir_1(); 
  }  

  while(1) {
    // prepare the mailbox receive buffer
    rst_calib_msg.info = 0;
    rst_calib_msg.size = sizeof(uint8_t);
    rst_calib_msg.rx_source_thread = cmd_mgr_thrd_id; 

    if(res_calib_cmd == 0) {
      // retrieve and delete the message if received
      k_mbox_get(&cmd_mgr_mb, &rst_calib_msg, &res_calib_cmd, K_FOREVER);
    }

    switch (res_calib_cmd) {
      case RESET_START:
        printk("RESET_START\n\r");
        resetCalibStart();
        enum_lock_status=EM_LOCK_DISABLE;
        Lock_sendCmdACK(&enum_lock_status,1);
        Speed(25, 25, Gear_motor_params.gearbox_ratio, Gear_motor_params.micro_steps,
            Gear_motor_params.step_angle, 25, 25); 
        Gear_motor_params.speed_change_percentage = 10;
        Gear_motor_params.skip_steps = 250;
        Gear_motor_params.Bward_speed_change_percentage = 10;
        Gear_motor_params.bw_skip_steps = 250;
     
        res_calib_cmd = 0;
        reset_variable = 0;
        break;

      case RESET_AUTO:
        printk("RESET_AUTO\n\r");// when click NO option RESET_AUTO call again with out RESET_START for that re initlization profile values for accel again
        Speed(25, 25, Gear_motor_params.gearbox_ratio, Gear_motor_params.micro_steps,
            Gear_motor_params.step_angle, 25, 25); 
        Gear_motor_params.Bward_speed_change_percentage = 10;
        Gear_motor_params.bw_skip_steps = 250;

        if(reset_variable==0) {
          reset_variable = 1;
        }        
        res_calib_cmd = resetCalibAuto(&new_close_IMU_val);
        break;
      
      case RESET_FORWARD:
        printk("RESET_FORWARD\n\r");
        Speed(25, 25, Gear_motor_params.gearbox_ratio, Gear_motor_params.micro_steps,
            Gear_motor_params.step_angle, 25, 25); 
        Gear_motor_params.speed_change_percentage = 10;
        Gear_motor_params.skip_steps = 250;
        Gear_motor_params.Bward_speed_change_percentage = 10;
        Gear_motor_params.bw_skip_steps = 250;
        res_calib_cmd = resetCalib_RunADO(RESET_FORWARD, DRIVE_MOTOR_DIR_FORWARD, &new_close_IMU_val);
        break;

      case RESET_BACKWARD:
        printk("RESET_BACKWARD\n\r");
        Speed(25, 25, Gear_motor_params.gearbox_ratio, Gear_motor_params.micro_steps,
            Gear_motor_params.step_angle, 25, 25); 
        Gear_motor_params.speed_change_percentage = 10;
        Gear_motor_params.skip_steps = 250;
        Gear_motor_params.Bward_speed_change_percentage = 10;
        Gear_motor_params.bw_skip_steps = 250;
        res_calib_cmd = resetCalib_RunADO(RESET_BACKWARD, DRIVE_MOTOR_DIR_BACKWARD, &new_close_IMU_val);
        break;

      case RESET_STOP:
        printk("RESET_STOP\n\r");
        res_calib_cmd = resetCalibStop(RESET_STOP);
        reset_variable = 255;
        break;

      case RESET_DONE:
        printk("RESET_DONE\n\r");
        if(reset_variable==1) {
          reset_variable = 2;
        }
        resetCalibDone( new_close_IMU_val);
        Speed(Gear_motor_params.start_RPM, Gear_motor_params.end_RPM, Gear_motor_params.gearbox_ratio,
            Gear_motor_params.micro_steps, Gear_motor_params.step_angle, Gear_motor_params.bw_start_RPM,
            Gear_motor_params.bw_end_RPM);     
        res_calib_cmd = 0;
        quit =1;
        break;
              
      default:
        res_calib_cmd = 0;
        break;
    }
    if(quit) {
    quit =0;
      break;
    }
  }
  enable_pir_1(); 
  power_optimization(CALIBRATION_STOP);
}

/*
 *  brief: Function to start reset calibration
 */
void resetCalibStart(void) {
  // Notify  over BLE that the reset calibration has started
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_RESET_RESPONSE, RESET_START, RESET_STARTED, NULL, 0);
}

/*
 *  brief: Function to automatically move ADO in close direction, stop based on stagnation and respond to ED that door is closed
 */
int8_t resetCalibAuto(float * pnew_close_IMU) {
  int8_t ret;
  // Check if ADO is calibrated, to avoid reading any wrong value from EEPROM
  if(is_ADO_Calibrated(&calib) == false) {
    //Debug print
    printk("ADO not calibrated\r\n");
    return -1;
  }

  // Read the previously calibrated params as read from EEPROM
  // Assign the opposite of close_to_open direction to open_to_close direction 
  bool open_to_close_dir = (calib.close_to_open_motor_direction == DRIVE_MOTOR_DIR_FORWARD) ? DRIVE_MOTOR_DIR_BACKWARD : DRIVE_MOTOR_DIR_FORWARD;
  
  // debug msg
  //printk("open_to_close_dir: %d, close_to_open:  %d\r\n", open_to_close_dir, calib.close_to_open_motor_direction);
  
  // Move the ADO in the open_to_close direction
  ret = resetCalib_RunADO(RESET_AUTO, open_to_close_dir, pnew_close_IMU);
  if(ret != 0) {
    return ret;
  }
  
  // Reached here means ADO stopped due to stagnation, can be treated as 'door is closed'
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_RESET_RESPONSE, RESET_AUTO, RESET_CLOSED, NULL, 0);
  return ret;
}


/*
 *  brief: Function to move ADO in provided direction, stop based on stagnation and respond to ED that door is stopped
 */
uint8_t resetCalib_RunADO(uint8_t cmd, bool direction, float * new_close_IMU) {
  // Mailbox variables
  struct k_mbox_msg recv_msg;

  // prepare the mailbox receive buffer
  recv_msg.info = 0;
  recv_msg.size = sizeof(uint8_t);
  recv_msg.rx_source_thread = cmd_mgr_thrd_id;

  uint8_t recvd_cmd = 0;
  uint8_t cmd_response = 0;
    
  float prev_IMU_val = 0.0;
  float curr_IMU_val = G_IMU;
  float delta = 0.0;
  uint8_t stag_cycle_cnt = 0;

  int8_t ret = -1;

  //Get the Drive Motor State
  if (isMotorRunning(&Gear_motor_params)) {
    //STOP the motor if it is in running condition for protection from damage
    //rotateMotor(&Gear_motor_params, 0U, DRIVE_MOTOR_DIR_FORWARD, DRIVE_MOTOR_OFF, Disable_TruePWM);

    gpio_pin_set(Gear_motor_params.enable, Gear_motor_params.settings.en_pin, 1);
    Gear_motor_params.isRunning = false;
  }

  // Move ADO in given 'direction'
  ret = rotateMotor_autoreset(&Gear_motor_params, DEF_CAL_MOTOR_SPEED_PERCENT, direction, DRIVE_MOTOR_ON,Disable_TruePWM);
  
  // get the received command and it's response to report over BLE
  cmd_response = (direction == DRIVE_MOTOR_DIR_FORWARD)? RESET_MOVING_FORWARD : RESET_MOVING_BACKWARD;

  // Notify  over BLE that the ADO is moving in 'direction'
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_RESET_RESPONSE, cmd, cmd_response, NULL, 0);

  // set the new calibrated close position for further use.
  if(new_close_IMU != NULL) {
    *new_close_IMU = curr_IMU_val;
  }

  // Check if the loop is broken due to any new command
  if(ret > 0) {
    return ret;
  }
  printf("\n===========> before reset calib while");
  // Logic to stop the ADO based on stagnation
  
  while((stag_cycle_cnt < ADO_MAX_STAGNANT_CYCLE_CNT) && (recvd_cmd == 0)) {
    // check the difference between new and previous IMU values
    delta = fabs(curr_IMU_val - prev_IMU_val);
    printf("\ncurr_IMU_val = %f prev_IMU_val = %f  delta = %f\n",curr_IMU_val,prev_IMU_val,delta);

    if(delta <= ADO_STAGNANT_THRESHOLD_DEG) {
      ++stag_cycle_cnt;
     // printf(" \nstag_cycle_cnt %d\n",stag_cycle_cnt);
    } else {
      if(stag_cycle_cnt >=1) {
      //  printf("\n stag_cycle_cnt reset\n");
      stag_cycle_cnt = 0; // reset the counter
      }
    }    // Debug print
    //printk("stag_cycle_cnt = %d\r\n", stag_cycle_cnt);
    // read for any new message receieved over mailbox
    k_mbox_get(&cmd_mgr_mb, &recv_msg, &recvd_cmd, ADO_MAX_STAGNANT_CYCLE_PERIOD_MS);
    prev_IMU_val = curr_IMU_val;
    curr_IMU_val = G_IMU;
  }
  //printf("\n after while stag_cycle_cnt %d\n",stag_cycle_cnt);
  // set the new calibrated close position for further use.
  if(new_close_IMU != NULL) {
    *new_close_IMU = curr_IMU_val;
    printf("\n ==> imu val\n %f    %f",*new_close_IMU,curr_IMU_val);
  }

  // Check if the loop is broken due to any new command
  if(recvd_cmd != 0) {
    return recvd_cmd;
  }
  printf("\n===========> after reset calib while exit");
  // Reached here as ADO was stagnant at one point, stop the ADO
  ret  = resetCalibStop(cmd);
  return ret;
}

/*
 *  brief: Function to stop ADO if it was moving in any direction and respond to ED that door is stopped
 */
int resetCalibStop(uint8_t cmd) {
  int ret = 0;

  // Notify  over BLE that the reset calibration has stopped
  ADO_notify_ble(cmd_mgr_thrd_id, ADO_RESET_RESPONSE, cmd, RESET_STOPPED, NULL, 0);
 
   //Get the DC Motor State
  if (isMotorRunning(&Gear_motor_params)) {
    // printk("\n==>stagination called");
    ret = motor_lock_o(&Gear_motor_params);
    //STOP the motor if it is in running condition.
    // rotateMotor(&Gear_motor_params, 0U,  DRIVE_MOTOR_DIR_FORWARD,DRIVE_MOTOR_OFF,Disable_TruePWM);
    gpio_pin_set(Gear_motor_params.enable, Gear_motor_params.settings.en_pin, 1);
    Gear_motor_params.isRunning = false;
    //TODO: Handle any errors
  } 
  return ret;
}

/*
 *  brief: Function to complete the reset calibration procedure, it updates the new open and close positions into global calib structure
 */
int8_t resetCalibDone(float newIMUCloseVal) {
  int8_t ret = -1;
   
  // Save the new close position and update the open position based on previous close to open angle
  ret = updateCalib(newIMUCloseVal);
  if(ret == 0) {
    //Debug print
    printk("Calibration values updated!\r\n");

    // Notify  over BLE that the reset calibration has completed
    ADO_notify_ble(cmd_mgr_thrd_id, ADO_RESET_RESPONSE, RESET_DONE, RESET_COMPLETE, NULL, 0);

    // Update the ado state that it is not restarted now
    ADO_set_restart_state(ADO_NOT_RESTARTED);
  }
  return ret;
}

/*
 *  brief: Function to update the new open and close positions into global calib structure based on previous calibration values
 */
int8_t updateCalib( float newIMUCloseVal) {
    int8_t ret = -1;
    doorPositionFromClose = 0;

    // Check the new IMU value is having any valid IMU value or not
    if(newIMUCloseVal < 0) {
      printf("Invalid new IMU Close value %f\r\n", newIMUCloseVal);
      return ret;
    }
        
    // set the new open position w.r.t new close position and previously calibrated close and open values
    if(calib.pos_open > calib.pos_close) {
      calib.pos_open = newIMUCloseVal + calib.close_to_open_angle;
    } else if(calib.pos_open < calib.pos_close) {
      calib.pos_open = newIMUCloseVal - calib.close_to_open_angle;
    }
        
    // set the new close position
    calib.pos_close = newIMUCloseVal;

    // debug print
    printf("new close: %f, new open: %f\r\n", calib.pos_close, calib.pos_open);
    return 0;
}

uint8_t reset_option() {
  //uint8_t temp = reset_variable;
  //reset_variable = 255;
  //return temp;
  return reset_variable;
}

float fw_accrelation_interval_resetcalib(void) {
  //printk("Forward acceleration interval is called\r\n");
  if ((Gear_motor_params.steps % Gear_motor_params.skip_steps) == 0) {
    if (Gear_motor_params.current_speed_percentage < 100) {
      Gear_motor_params.current_speed_percentage += Gear_motor_params.speed_change_percentage;
      Gear_motor_params.current_interval = Gear_motor_params.start_interval - ((Gear_motor_params.start_interval - Gear_motor_params.end_interval) * Gear_motor_params.current_speed_percentage / 100);
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


float bw_accrelation_interval_resetcalib(void) {
  if ((Gear_motor_params.bw_steps % Gear_motor_params.bw_skip_steps) == 0) {
    if (Gear_motor_params.bw_current_speed_percentage < 100) {
      Gear_motor_params.bw_current_speed_percentage += Gear_motor_params.bw_speed_change_percentage;
      Gear_motor_params.bw_current_interval = Gear_motor_params.bw_start_interval - ((Gear_motor_params.bw_start_interval - Gear_motor_params.bw_end_interval) * Gear_motor_params.bw_current_speed_percentage / 100);
    } else {
      printk("**************************\n");
      printk("%d", Gear_motor_params.bw_current_interval);
    }

    if (Gear_motor_params.bw_steps >= Gear_motor_params.bw_skip_steps * 20) {
      Gear_motor_params.bw_steps = 0;
    }
  }
  Gear_motor_params.bw_steps += 1;
  return Gear_motor_params.bw_current_interval;
}


/*
 * brief: Rotate or Stop the motor as per the provided parameters, the motor_param must be valid
 */
int8_t rotateMotor_autoreset(struct motor_params_t *motor_params, uint8_t pwm_percent,
    bool dir, bool enable, bool true_pwm) {
  //int32_t ret = 0, u_sec = 0, cnt = 0; // temp
  res_calib_cmd=0;
  printk("\n rotate rotateMotor_autoreset is called\n");
  int32_t u_sec = 0, cnt = 0; // temp

  int ret =0;
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

  // Set the motor state
  // since the DC door motor enable pin is set to active low,
  // we are tasking opposite state to depict motor running
  //motor_params->isRunning = !enable;

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
    if(motor_params->isRunning) {
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

        while (Gear_motor_params.current_speed_percentage < 100  ) { //11825 cnt <= 13325 
          u_sec = fw_accrelation_interval_resetcalib();
          // printf("\nforward speed = %d direction = %d msec = %f enable = %d\n",pwm_percent,dir,u_sec,enable);
          ret = pwm_pin_set_usec(motor_params->pwm, motor_params->settings.pwm_channel, u_sec, u_sec / 2U, motor_params->settings.pwm_flags);
          
          if (ret != 0) {
            printk("Error %d: failed to set pulse width\n", ret);
          }

          k_mbox_get(&cmd_mgr_mb, &rst_calib_msg, &res_calib_cmd, K_USEC(u_sec));

          if((res_calib_cmd==RESET_STOP)  || (res_calib_cmd==RESET_FORWARD) || (res_calib_cmd==RESET_BACKWARD)) {
            printk("\nSTOP or forward or backward Command is detected in rotate motor\n");
            printk("res_calib_cmd = %d",res_calib_cmd);
            return res_calib_cmd;
            //return 4;
          }
        }
      } else {
        ret = gpio_pin_set(motor_params->direction, motor_params->settings.dir_pin, GPIO_ACTIVE_LOW);
        if (ret != 0) {
          printk("Error %d: failed to set  direction\n", ret);
        }

        while (Gear_motor_params.bw_current_speed_percentage < 100 ) { //9000 cnt <= 13325 
          u_sec = bw_accrelation_interval_resetcalib();
          // printk("\nbackward speed = %d direction = %d msec = %d enable = %d\n", pwm_percent, dir, u_sec, enable);
          ret = pwm_pin_set_usec(motor_params->pwm, motor_params->settings.pwm_channel, u_sec, u_sec / 2U,
              motor_params->settings.pwm_flags);
          if (ret != 0) {
            printk("Error %d: failed to set pulse width\n", ret);
          }

          k_mbox_get(&cmd_mgr_mb, &rst_calib_msg, &res_calib_cmd, K_USEC(u_sec));

          if((res_calib_cmd==RESET_STOP) || (res_calib_cmd==RESET_FORWARD) || (res_calib_cmd==RESET_BACKWARD)) {
            printk("\nSTOP or forward or backward Command is detected in rotate motor\n");
            printk("res_calib_cmd = %d",res_calib_cmd);
            return res_calib_cmd;
          }
        }
      }
    }
    printk("\n============>enable = %d isRunning = %d\n", enable,motor_params->isRunning);
  }
  return ret;
}
