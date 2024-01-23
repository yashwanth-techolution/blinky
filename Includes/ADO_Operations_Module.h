#ifndef __ADO_OPERATIONS_MODULE_H_
#define __ADO_OPERATIONS_MODULE_H_

#include <stdint.h>
#include <zephyr/kernel.h>
#include "Includes/Motor_Control_Module.h"


// The ADO operation thread will be idle for 400 cycle of 50 ms i.e 400 * 50 = 20000 ms = 20 sec
#define MAX_OPR_THREAD_IDLE_CYCLES 100U


// Clutch motor maximum parameters for M5.
#define MAX_CLUTCH_DIS_ENG_WAIT_CYCLE      (1650/50)    // (millis/50) timer count.
#define MAX_CLUTCH_ENG_WAIT_CYCLE          (1650/50)    // (millis/50) timer count.
#define DRIVE_MOTOR_CLUTCH_ENGAGE_PWM      25U          // Motor PWM percent.
#define DRIVE_MOTOR_CLUTCH_DIS_ENGAGE_PWM  25U          // Motor PWM percent.
#define DRV_MOTOR_FORWARD                  true
#define FORWARD_BACKWARD_TIME              (500/50)    
#define ENABLE_DIS_ANGAGE_PWM


// IMU Percentage stagnent stop predefines
#define DOOR_DIFF_PERCENT             1U        // percentage delta for counting stop cycles. 
#define DOOR_CLOSE_CUTT_OFF_PERCENT   3U 
#define error_code                    0
// deceleration logic predefines
#define FW_DOOR_DECEL_START_PERCNT       80U   // deceleration start percentage
#define BW_DOOR_DECEL_START_PERCNT       75U   // deceleration start percentage

// Enum containing the ADO operation commands
typedef enum {
  OPEN = 1,
  CLOSE,
  OPR_STOP,
  CLUTCH_ENGAGE,
  CLUTCH_DISENGAGE,
  ON_LED,
  OFF_LED
} ado_operation_cmd_t;

// Enum containing the ADO operation response
typedef enum {
  OPENING = 1,
  OPENED,
  CLOSING,
  CLOSED,
  OPR_INTERRUPTED,
  OPR_UNSAFE_OPERATION,
  OPR_BATTERY_LOW,
  OPR_STOPPED,
  CLUTCH_ENGAGE_RUNNING,
  CLUTCH_DISENGAGE_RUNNING,
  CLUTCH_ENGAGED,
  CLUTCH_DISENGAGED,
  LED_ON,
  LED_OFF
} ado_operation_resp;

// Enum depicting different states of operation, this is meant for device's internal use.
typedef enum {
  OPR_NOT_STARTED = 0,
  OPR_STARTED,
  MOTOR_RUNNING,
  OPENING_DOOR,
  OPEN_INTERRUPTED,
  FULLY_OPEN,
  CLOSING_DOOR,
  CLOSE_INTERRUPTED,
  FULLY_CLOSED,
  OPR_THRD_EXITED
} ado_operations_status_t;

// Enum depicting the causes of interrupt in open/close operation, to be reported to ED.
typedef enum {
  OPR_USS_INTRPT = 1, 
  OPR_STAGNANT_INTRPT
}ado_operation_intrpt_t;

typedef enum {
  OPENING_ANDROID = 0,
  OPENED_ANDROID,
  OPEN_INTERRUPTED_ANDROID,
  CLOSING_ANDROID,
  CLOSED_ANDROID,
  CLOSE_INTERRUPTED_ANDROID,
  ERROR_OPERATION_ANDROID = 255
} operation_check_android;

typedef enum {
  CLUTCH_DISENGAGED_DEV_STAT = 0,
  CLUTCH_ENGAGED_DEV_STAT,
  CLUTCH_DISENGAGING_DEV_STAT,
  CLUTCH_ENGAGING_DEV_STAT,
  ERROR_IN_CLUTCH_OPERATION = 255
} clutch_state_dev_stat; 

// Structure to contain ADO Operations related data
typedef struct {
  // Track keeping variables
  uint8_t state;
  
  uint8_t cmd;
} ado_operations_t;

extern ado_operations_t operation;
extern k_tid_t ado_opr_thrd_id;
extern int execution_time_close;
extern int flag_close;

// Function Declarations

bool is_doorbot_uninstalled_uncalibrated(void);

/**********************************************************************************************************
 * Function name  :   ADO_Operations_Thread()
 *
 * Description    :   1. The Thread definition for ADO Operation commands, this thread handles Normal in-
 *                       operation commands like OPEN, CLOSE, ENGAGE/DISENGAGE CLUTCH, LED ON/OFF and OPR_STOP commands.
 *
 *                    2. This thread depends upon the of member variables of a global structure variable of
 *                       type ado_operations_t and also sets them during the operation of thread. Therefore,
 *                       we need to be careful with its usage.
 *
 * Params         :   None
 *
 * Returns        :   Nothing
 ***********************************************************************************************************/
void ADO_Operations_Thread();


/**********************************************************************************************************
 * Function name  :   OpenTheDoor()
 *
 * Description    :   1. This function handles the request to open the door to the calibrated open position. 
 *                       It checks the current door position and linearises it if the door crosses 360". Its
 *                       working also depends on the calibrated door close->open direction i.e. clock-wise or
 *                       anti clock-wise.
 *  
 *                    2. Once it starts moving the door towards open position, it will keep checking the IMU
 *                       readings until the door reaches the Open Position. But while in transit to open the 
 *                       door, if it receives another command say CLOSE or STOP, it will break the current 
 *                       door open operation and will return indicating the incomplete open via a state called
 *                       OPEN_INTERRUPTED.
 *                  
 *                    3. It is important to note that this function stops the motor before returning irrespective
 *                       to the door is closed completely or interrupted in between.
 *
 * Params         :   None
 *
 * Returns        :   <uint8_t> 0 if the door completely opened, the new command value if open was interrupted
 ***********************************************************************************************************/
uint8_t OpenTheDoor(uint8_t command);


/**********************************************************************************************************
 * Function name  :   CloseTheDoor()
 *
 * Description    :   1. This function handles the request to close the door to the calibrated close position. 
 *                       It checks the current door position and linearises it if the door crosses 360". Its
 *                       working also depends on the calibrated door close->open direction i.e. clock-wise or
 *                       anti clock-wise.
 *  
 *                    2. Once it starts moving the door towards close position, it will keep checking the IMU
 *                       readings until the door reaches the close Position. But while in transit to close the 
 *                       door, if it receives another command say OPEN or STOP, it will break the current 
 *                       door close operation and will return indicating the incomplete close via a state called
 *                       OPEN_INTERRUPTED.
 *
 *                    3. It is important to note that this function stops the motor before returning irrespective
 *                       to the door is closed completely or interrupted in between.
 *
 * Params         :   None
 *
 * Returns        :   <uint8_t> 0 if the door completely closed, the new command value if close was interrupted
 ***********************************************************************************************************/
uint8_t CloseTheDoor(uint8_t command);


/**********************************************************************************************************
 * Function name  :   StopTheDoor()
 *
 * Description    :   1. This function handles the request to stop the door immediately if it is moving in
 *                       any direction and notifies the ED for the close operation status i.e. OPR_STOPPED
 *
 *                    2. It is important to note that this function should check if the motor is running
 *                       before attempting to stop the motor, but presently it just stops it, it could be 
 *                       taken as a future enhancement.
 *
 * Params         :   None
 *
 * Returns        :   <uint8_t> 0 if the door stopped.
 ***********************************************************************************************************/
uint8_t StopTheDoor(); 


/**********************************************************************************************************
 * Function name  :   door_close_to_open_percentage()
 *
 * Description    :   1. This function caluclates the current percentage position of door from close point 
 *                       based on current IMU position value.
 *
 * Params         :   uint16_t current_IMU: Current IMU position value to calculate percentage.
 *
 * Returns        :   none.
 ***********************************************************************************************************/
void door_close_to_open_percentage();

/**********************************************************************************************************
 * Function name  :   engageClutch()
 *
 * Description    :   1. This function controls the clutch motor to engage the drive motor gear to the wheel 
 *                       so that when the drive motor runs, the wheel runs along with it. This function takes
 *                       care of notifying the clutch status to ED over BLE. The clutch motor is stopped if
 *                       an OPR_STOP command is received over BLE or a fixed time MAX_CLUTCH_WAIT_CYCLE * 50MS
 *                       is elapsed. It can be optimised in future for better control.
 *
 * Params         :   None.
 *
 * Returns        :   <int8_t> OPR_STOP when no new command is arrived while engaging otherwise, will return
 *                    new command received.
 ***********************************************************************************************************/
int8_t engageClutch();

/**********************************************************************************************************
 * Function name  :   disengageClutch()
 *
 * Description    :   This function controls the clutch motor to disengage the drive motor gear to the wheel 
 *                    so that the wheel becomes free from drive motor and door can be pushed to open. This
 *                    function takes, care of notifying the clutch status to ED over BLE. The clutch motor
 *                    is stopped if, an OPR_STOP command is received over BLE or a fixed time 
 *                    MAX_CLUTCH_WAIT_CYCLE * 50MS is elapsed. It can be optimised in future for better control.
 *
 * Params         :   None.
 *
 * Returns        :   <int8_t> OPR_STOP when no new command is arrived while disengaging otherwise, will return
 *                    new command received.
 ***********************************************************************************************************/
int8_t disengageClutch();

/**********************************************************************************************************
 * Function name  :   setLEDStrip()
 *
 * Description    :   This function is a placeholder function for now for handling LED Strip commands, it 
 *                    just notifies over BLE of the LED state. Later need to add LED control  logic.    
 *
 * Params         :   None.
 *
 * Returns        :   <none> 
 ***********************************************************************************************************/
void  setLEDStrip(uint8_t led_cmd);
float getlinear_OPEN_CLOSE(bool open_close,float destinationIMU);
float calculate_IMU_val_1(float X1_Val,float Y1_Val,float IMU_Value);
float calculate_IMU_val_0(float X1_Val,float Y1_Val,float IMU_Value);
float getlinearopen(void);
float getlinearclose(void);

uint8_t OTA_check_operation_status();

int8_t save_operation_data();
int8_t load_operation_data();

int8_t rotateMotor_opr(uint8_t command,struct motor_params_t *motor_params, uint8_t pwm_percent, bool dir, bool enable, bool true_pwm);
uint8_t deceleration_interval_opr(struct motor_params_t *motor_params,bool dir);
int door_close_to_open_percentage_m(void);
int Stop_Motor_o(struct motor_params_t *motor_params, bool dir, bool enable_disable, bool true_PWM);
int motor_lock_o(struct motor_params_t *motor_params);

float fw_accrelation_interval_opr(void);
float bw_accrelation_interval_opr(void);
uint8_t decelerate_for_stop(struct motor_params_t *motor_params);

extern bool operation_flag;

#endif