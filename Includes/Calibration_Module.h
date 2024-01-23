
#ifndef __CALIBRATION_MODULE_H__
#define __CALIBRATION_MODULE_H__

#include <stdint.h>
#include <stdbool.h>

#include "Includes/Command_Manager_Module.h"
#include "Includes/Motor_Control_Module.h"

#define MOTOR_CW    true
#define MOTOR_ACW   false

// Enum containing the CALIBRATION commands
typedef enum
{
  CAL_START = 1,
  FORWARD,
  BACKWARD,
  STOP,
  REC_CLOSE,
  REC_OPEN,
  REQ_INFO,
  SAVE_YAW_DIR,
  CAL_DONE,
  CAL_TERMINATE
}ado_calib_cmd_t;

// Enum containing the CALIBRATION command responses
typedef enum
{
  CAL_PENDING = 0,
  CAL_STARTED,
  MOVING_FORWARD,
  MOVING_BACKWARD,
  CAL_STOPPED,
  CLOSE_RECORDED,
  OPEN_RECORDED,
  RESP_INFO,
  SAVED_YAW_DIR,
  CAL_COMPLETED,
  CAL_JAMMED,
  CAL_UNSAFE_OPERATION,
  CAL_NOT_STARTED,
  CAL_BATTERY_LOW,
  CAL_IN_PROGRESS,
  CAL_TERMINATED
} ado_calib_resp_t;

// Enum containing the calibration states
typedef enum
{
  NONE_REC_POS = 0,
  CHECK_YAW_CROSSING,
  CHECK_DIRECTION,
}ado_calib_state_t;

typedef enum
{
  UNCALIBRATED_ANDROID=0,
  CALIBRATED_ANDROID,
  CALIBRATING_ANDROID,
  ERROR_IN_CALIBRATION =255
} calib_state_dev_stat;

// Structure to depict the calibration related parameters
typedef struct
{
  // Track keeping variables
  ado_calib_resp_t    status;
  ado_calib_state_t   state;
  uint8_t             cmd;
  float            imu_val;
  
  // Calibration results
  float            pos_open;
  float            pos_close;
  
  // calculated results
  float               close_to_open_angle;    // Stores total opening angle of door
  int16_t             direction_angle_count;  // counter variable to identify close to open motor direction (value < 0 = backward, value > = forward)

  bool                is_yaw_crossed;
  bool                is_direction_cw;               // false is eq. to anti-cw
  bool                close_to_open_motor_direction; // motor wheel rotation from close position to open position (true = forward, false = backward)
  bool                is_calibrated;                 // To keep the status of doorbot calibration in EEPROM
}ado_calib_status_t;

extern ado_calib_status_t calib;
extern k_tid_t ado_calib_thrd_id;
extern uint8_t track_calibration_process;


/**********************************************************************************************************
 * Thread name    :   InitCalibrationProcess()
 *
 * Description    :   This Thread executes all the calibration commands
 *
 * Params         :   None
 *
 * Return         :   Nothing.
 ***********************************************************************************************************/
void InitCalibrationProcess();


/**********************************************************************************************************
 * Function name  :   CalibrationStart(void)
 *
 * Description    :   This Function process the CAL1 START Command from BLE and forms the response packet.
                      and sets the event flag to true to inform the main routine.
 *
 * Params         :   None
 *
 * Return         :   Nothing.
 ***********************************************************************************************************/
//void CalibrationStart(void);        //commented by ashok
int CalibrationStart(void);

/**********************************************************************************************************
 * Function name  :   RecordOpenPosition()
 *
 * Description    :   This Function process the REC_OPEN Command from BLE and forms the response packet.
                      and sets the event flag to true to inform the main routine.
 *
 * Params         :   None
 *
 * Return         :   Nothing.
 ***********************************************************************************************************/
void RecordOpenPosition(void);


/**********************************************************************************************************
 * Function name  :   RecordClosePosition()
 *
 * Description    :   This Function process the REC_CLOSE Command from BLE and forms the response packet.
                      and sets the event flag to true to inform the main routine.
 *
 * Params         :   None
 *
 * Return         :   Nothing.
 ***********************************************************************************************************/
void RecordClosePosition(void);


/**********************************************************************************************************
 * Function name  :   RequestINFO()
 *
 * Description    :   This Function process the REQ_INFO Command from BLE and forms the response packet.
                      and sets the event flag to true to inform the main routine.
 *
 * Params         :   None
 *
 * Return         :   Nothing.
 ***********************************************************************************************************/
void RequestInfo(void);


/**********************************************************************************************************
 * Function name  :   CalibrationDone()
 *
 * Description    :   This Function process the CAL1_DONE Command from BLE and forms the response packet.
                      and sets the event flag to true to inform the main routine.
 *
 * Params         :   None
 *
 * Return         :   Nothing.
 ***********************************************************************************************************/
void CalibrationDone(void);


/**********************************************************************************************************
 * Function name  :   FORWARD()
 *
 * Description    :   This Function process the FORWARD Command from BLE and forms the response packet.
                      and sets the event flag to true to inform the main routine.
 *
 * Params         :   None
 *
 * Return         :   Nothing.
 ***********************************************************************************************************/
int Forward(void);       //void Forward(void);


/**********************************************************************************************************
 * Function name  :   BACKWARD()
 *
 * Description    :   This Function process the BACKWARD Command from BLE and forms the response packet.
                      and sets the event flag to true to inform the main routine.
 *
 * Params         :   None
 *
 * Return         :   Nothing.
 ***********************************************************************************************************/
int Backward(void);      //void Backward(void);


/**********************************************************************************************************
 * Function name  :   StopMotor()
 *
 * Description    :   This Function process the STOP Command from BLE and forms the response packet.
                      and sets the event flag to true to inform the main routine.
 *
 * Params         :   None
 *
 * Return         :   Nothing.
 ***********************************************************************************************************/
int StopMotor(void);            //added by ashok

//void StopMotor(void);         //commented by ashok


/**********************************************************************************************************
 * Function name  :   DirectionCalibration()
 *
 * Description    :   This Function Calculates the door direction whether clockwise or anti-clockwise
 * Params         :   1. <in & out> ado_calib_status_t *calib_status: A reference to calibration structure
 *
 * Return         :   Nothing.
 ***********************************************************************************************************/
void DirectionCalibration(ado_calib_status_t *calib_status);


/**********************************************************************************************************
 * Function name  :   YawCrossCalibration()
 *
 * Description    :   This Function captures the yaw 0/360 crossing condition. It should be called continuously
 *                    after recording close position while the user moves the door to record open position and
 *                    should stop calling once user has recorded open position.
 *
 * Params         :   1. <in & out> ado_calib_status_t *calib_status: A reference to calibration structure
 *                    2. uint8_t direction: It passes current motor rotation of FORWARD, BACKWARD, STOP commands
 *
 * Return         :   Nothing.
 ***********************************************************************************************************/
void YawCrossCalibration(ado_calib_status_t *calib_status, uint8_t direction);
void final_YawCrossCalibration(ado_calib_status_t *calib_status, uint8_t direction);


/**********************************************************************************************************
 * Function name  :   SaveYawDirection()
 *
 * Description    :   This Function reads the yaw and driection information from ED and overwrites to ADO Memeory
 *
 * Params         :   uint8_t * calib_data:
 *                       Byte array containing the values received from ED to be saved/modified with new values
 *
 * Return         :   Nothing.
 ***********************************************************************************************************/
void SaveYawDirection(uint8_t * calib_data);

/**********************************************************************************************************
 * Function name  :   CalibrationTerminate()
 *
 * Description    :   This Function will stop any calibration task going ON, clears the calibrated data and 
 *                    terminates the thread.
 *
 * Params         :   None
 *
 * Return         :   Nothing.
 ***********************************************************************************************************/
void CalibrationTerminate(void);

/**********************************************************************************************************
 * Function name  :   ado_calibration()
 *
 * Description    :   1. This function is required to execute calibration commands of doorbot already
 *                       installed. It creates a new thread to run all the calibration related commands
 *                       and, sends a mailbox message to it with the received command and related data.
 *                        
 *                    2. If the thread is already running, it just sends a mailbox message to it.
 *                       but if the thread has not started but we received any other command than
 *                       CAL_START, it will notify over BLE that the Calibration thread is not 
 *                       started.
 *
 * Params         :   1. <in> uint8_t * calib_data:  
 *                            A byte array buffer containing the command values for calibration process.
 *
 * Returns        :   Nothing.
 ***********************************************************************************************************/
void ado_calibration(uint8_t * calib_data);

/**********************************************************************************************************
 * Function name  :   save_calib_data()
 *
 * Description    :   1. This function is required to write the calibration parameters to the ADO EEPROM
 *                       Memory. It depends upon the global calib structure variable and it assumes that
 *                       the calib structure variable is having valid calibration parameter values.
 *
 *                    2. No validation is performed on the values to be written at present, it needs to be
 *                       called wisely as any wrong calibration writing may cause the hardware damage.
 *
 * Params         :   None
 *
 * Returns        :   Nothing.
 ***********************************************************************************************************/
void save_calib_data();

/**********************************************************************************************************
 * Function name  :   load_calib_data()
 *
 * Description    :   1. This function is required to read back the calibration parameters from the ADO EEPROM
 *                       Memory. It keeps the read values in global calib structure variable.
 *
 *                    2. No validation is performed on the read values at present, it may yield unwanted results
 *                       for the first time when EEPROM has no calib data saved, one needs to ensure calibrating
 *                       the ADO device before using it for the first time.
 *
 * Params         :   None
 *
 * Returns        :   Nothing.
 ***********************************************************************************************************/
int8_t load_calib_data();

/**********************************************************************************************************
 * Function name  :   is_ADO_Calibrated()
 *
 * Description    :   1. This function Checks the global calib structure variable for the isCalibrated flag,
 *                       and returns whether the device is calibrated or not
 *
 *                    2. Presently, it just relies on the flag value, later can be optimised with better 
 *                       validation logic.
 *
 * Params         :   <ado_calib_status_t *> pCalib: Pointer to calibration structure member, it is global.
 *
 * Returns        :   Nothing.
 ***********************************************************************************************************/
bool is_ADO_Calibrated(ado_calib_status_t * pCalib);

//added by ashok
int rotateMotor_c(struct motor_params_t *motor_params, uint8_t pwm_percent, bool dir, bool enable, bool true_pwm);
int Stop_Motor_c(struct motor_params_t *motor_params, bool dir, bool enable_disable, bool true_PWM);

int motor_lock_c(struct motor_params_t *motor_params,int seconds);


void erase_Calibration_data();
#endif