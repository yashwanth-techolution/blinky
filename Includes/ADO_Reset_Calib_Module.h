// Header guards
#ifndef __ADO_RESET_CALIB_MODULE_H_
#define __ADO_RESET_CALIB_MODULE_H_

// System includes
#include <stdint.h>
#include <zephyr/kernel.h>

// Private includes

// Private Defines
#define ADO_STAGNANT_THRESHOLD_DEG        0.5f
#define ADO_MAX_STAGNANT_CYCLE_CNT        3U
#define ADO_MAX_STAGNANT_CYCLE_PERIOD_MS  K_MSEC(100)


// Enum containing the ADO Reset Calibration commands
typedef enum
{
   RESET_START = 1,
   RESET_AUTO,
   RESET_FORWARD,
   RESET_BACKWARD,
   RESET_STOP,
   RESET_DONE,
}ado_reset_calib_cmd_t;

// Enum containing the ADO Reset Calibration response
typedef enum
{
   RESET_STARTED = 1,
   RESET_MOVING_FORWARD,
   RESET_MOVING_BACKWARD,
   RESET_CLOSED,
   RESET_STOPPED,
   RESET_COMPLETE,
}ado_reset_calib_res_t;

// Extern variables
extern k_tid_t reset_calib_thrd_id;
extern uint8_t reset_variable;

// Thread Declaration
/**********************************************************************************************************
 * Thread name    :  ADO_Reset_Calib_Thread()
 *
 * Description    :  This Thread executes all the reset calibration commands, it is a temporary thread and
 *                   will be started by the command manager if it receives RESET_START command. Presently,
 *                   this thread starts when the ADO restarts post all initialisations. The ADO must be
 *                   installed and calibrated before starting this thread. This thread will return (terminate)
 *                   on receiving a RESET_DONE command.
 *
 * Params         :  None
 *
 * Return         :  Nothing.
 ***********************************************************************************************************/
void ADO_Reset_Calib_Thread(void);

// Function Declarations
/**********************************************************************************************************
 * Function name  :  ado_reset_calib()
 *
 * Description    :  This Function should be called by cmd_mgr thread when RESET_START Command is received from 
 *                   BLE. It creates a temporary thread which will handle the further RESET CALIBRATION commands.
 *
 * Params         :  1. <in> uint8_t reset_calib_cmd : The RESET CALIBRATION command received ex. RESET_START.
 *
 * Return         :  Nothing.
 ***********************************************************************************************************/
void ado_reset_calib(uint8_t reset_calib_cmd );

/**********************************************************************************************************
 * Function name  :  resetCalibStart()
 *
 * Description    :  This Function will be called in the ADO_Reset_Calib_Thread thread when RESET_START Command  
 *                   is received. Presently it just responds back that RESET_STARTED once the thread is created.
 *
 * Params         :  None.
 *
 * Return         :  Nothing.
 ***********************************************************************************************************/
void resetCalibStart(void);

/**********************************************************************************************************
 * Function name  :  resetCalibAuto()
 *
 * Description    :  This Function should be called when RESET_AUTO Command is received from the ED over BLE. 
 *                   will move the ADO in direction of previously calibrated closed position, will stop ADO
 *                   due to stagnation for ADO_MAX_STAGNANT_CYCLE_CNT * ADO_MAX_STAGNANT_CYCLE_PERIOD_MS sec.
 *                   It will respond back that the door is closed, subject to confirmation by the user on ED.
 *
 * Params         :  1. <out> float * pnew_close_IMU : A pointer to float variable which holds new close IMU.
 *
 * Return         :  <int8_t> It will return the new command received while moving ADO, 0 on success or error code.
 ***********************************************************************************************************/
int8_t resetCalibAuto(float * pnew_close_IMU);

/**********************************************************************************************************
 * Function name  :  resetCalib_RunADO()
 *
 * Description    :  This Function should be called if ANY Command to move ADO in provided direction is received. 
 *                   It takes the direction as argument and moves the ADO in that direction. It will respond back
 *                   to ED over BLE as per the current command execution status. The ADO will be stopped if the
 *                   door is stagnant for ADO_MAX_STAGNANT_CYCLE_CNT * ADO_MAX_STAGNANT_CYCLE_PERIOD_MS seconds,
 *                   irrespective of the direction of motion.
 *
 * Params         :  1. <in>  uint8_t cmd            : The RESET calib command received
 *                   2. <in>  bool direction         : The direction to move ADO ex. DRIVE_MOTOR_DIR_FORWARD.
 *                   3. <out> float * pnew_close_IMU : A pointer to float variable which holds new close IMU.
 *
 * Return         :  <uint8_t> It will return the new command received while moving ADO, or 0 on success.
 ***********************************************************************************************************/
uint8_t resetCalib_RunADO(uint8_t cmd, bool direction, float * new_close_IMU);

/**********************************************************************************************************
 * Function name  :  resetCalibStop()
 *
 * Description    :  This Function should be called to stop the ADO irrespective of the direction of motion.
 *
 * Params         :  1. <in>  uint8_t cmd            : The RESET calib command received
 *                   
 * Return         :  Nothing.
 ***********************************************************************************************************/
int resetCalibStop(uint8_t cmd);

/**********************************************************************************************************
 * Function name  :  resetCalibDone()
 *
 * Description    :  This Function should be called if RESET_DONE command is received from the ED over BLE. 
 *                   It takes the new IMU value for close position and checks for the previously calibrated
 *                   parameters like open_to_close angle etc. If everything is fine, it will set the new close
 *                   and open values in global calib structure for further use in ADO_operations.Please note
 *                   that the new values are NOT written back to EEPROM presently.
 *
 * Params         :  1. <in> float  newIMUCloseVal : A float variable holding new close IMU.
 *
 * Return         :  <int8_t> 0 on success or error code on failure.
 ***********************************************************************************************************/
int8_t resetCalibDone(float newIMUCloseVal);


/**********************************************************************************************************
 * Function name  :  updateCalib()
 *
 * Description    :  This Function is used to updatet the global calib structure with new open and close values. 
 *                   It takes the new IMU value for close position and checks for the previously calibrated
 *                   parameters like open_to_close angle etc. If everything is fine, it will set the new close
 *                   and open values in global calib structure for further use in ADO_operations.Please note
 *                   that the new values are NOT written back to EEPROM presently.
 *
 * Params         :  1. <in> float  newIMUCloseVal : A float variable holding new close IMU.
 *
 * Return         :  <int8_t> 0 on success or error code on failure.
 ***********************************************************************************************************/
int8_t updateCalib( float newIMUCloseVal);
int8_t rotateMotor_autoreset(struct motor_params_t * motor_params, uint8_t pwm_percent, bool dir, bool enable,bool true_pwm);
#endif
 