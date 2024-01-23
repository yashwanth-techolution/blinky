#ifndef __DEV_STAT_MODULE_H__
#define __DEV_STAT_MODULE_H__

#include <zephyr/kernel.h>

// provide the least priority to this thread
#define DEV_STAT_THRD_PRIORITY    10
#define DEV_STAT_THRD_STACK_SIZE  1024

// Moving average window size
#define MOV_AVG_WINDOW_SIZE 3U

// The interval at which the device status thread will refresh device status
#define DEV_STAT_POLL_INTERVAL    K_MSEC(47)   // Presently 100 Milli Seconds

// Battery percentage and device status update count cycle
#define DEV_STAT_POLL_COUNT       1000    // 1000 * 54 = 54000 millis -> 54 Seconds

#define MAX_OPEN_POS_OFFSET_PERCENT  5U           // + or - 5% change in door position will not be notified

#define MAX_DEV_VER_STR_LEN               6U           // Ex: v12.34.56 --> vMajorNum.MinorNum.Iteration
#define MAX_PER_VER_STR_LEN               6U
#define MAX_MODEL_VER_STR_LEN             6U
#define MAX_CUR_DOOR_POS_STR_LEN          1U
#define MAX_BAT_PER_STR_LEN               2U

#define MAX_RESTART_STATUS_STR_LEN        1U
#define MAX_DS_REG_UPDATE_VER_STR_LEN     6U
#define MAX_WIFI_CON_STR_LEN              1U
#define MAX_INST_STATUS_STR_LEN           1U
#define MAX_CALIB_STATUS_STR_LEN          1U

#define MAX_OP_STATUS_STR_LEN             1U
#define MAX_INTENT_STATUS_STR_LEN         1U
#define MAX_SENSOR_STATUS_STR_LEN         4U
#define MAX_CLUTCH_STATUS_STR_LEN         1U
#define MAX_SLEEP_MODE_STR_LEN            4U

#define MAX_ERROR_STATUS_STR_LEN          1U


#define ADO_FW_VER_MAJOR          2
#define ADO_FW_VER_MINOR          2
#define ADO_FW_VER_REVISION       15

#define ADO_HW_VER_MAJOR          2
#define ADO_HW_VER_MINOR          2
#define ADO_HW_VER_REVISION       0

#define ADO_WIFI_VER_MAJOR        0
#define ADO_WIFI_VER_MINOR        1
#define ADO_WIFI_VER_REVISION     0

#define ADO_STM_VER_MAJOR         0
#define ADO_STM_VER_MINOR         1
#define ADO_STM_VER_REVISION      0


#define ADO_VOICE_MODEL_VER_MAJOR          18
#define ADO_VOICE_MODEL_VER_MINOR          7
#define ADO_VOICE_MODEL_VER_REVISION       3

#define ADO_MOTION_INTENT_VER_MAJOR          0
#define ADO_MOTION_INTENT_VER_MINOR          0
#define ADO_MOTION_INTENT_VER_REVISION       0


// Device firmware version string, must be updated with every iteration or release
#define ADO_FW_VER_STR              "v2.2.9"      // updated with wifi module & sending data to Auto-AI server.


//#define ADO_RAYTAC_P2  // for ADO P2 Hardware with Raytac Module
#define ADO_RAYTAC_P2_1  // for ADO P2.1 Hardware with Raytac Module


#define MAX_USS_VAL_OFFSET              40U
#define USS_SAFE_OPEN_CUTOFF_DIST_MM    40U

#define ADO_RESTARTED       true
#define ADO_NOT_RESTARTED   false

typedef enum
{
  ADO_DEVICE_VERSION=1,
  PERIPHERAL_VERSIONS,
  MODEL_VERSIONS,
  CURRENT_DOOR_POS,
  ADO_BATTERY_PERCENT,

  ADO_RESTART_STATUS,
  DEV_STATUS_REG_UPDATE,
  WIFI_CONNECTION_STATUS,
  INSTALLATION_STATUS,
  CALIBRATION_STATUS,

  OPERATION_STATUS,
  INTENT_STATUS_DS,
  SENSORS_STATUS,
  CLUTCH_STATUS,
  SLEEP_MODE,

  ERROR_STATUS,
  EM_LOCK_STATUS
}ado_dev_stat_cmd_t;

typedef enum
{
  ADO_DEVICE_VERSION_RESP=1,
  PERIPHERAL_FW_VERSIONS_RESP,
  MODEL_VERSIONS_RESP,
  CURRENT_DOOR_POS_RESP,
  ADO_BATTERY_PERCENT_RESP,
  ADO_RESTART_STATUS_RESP,
  DEV_STATUS_REG_UPDATE_RESP,
  WIFI_CONNECTION_STATUS_RESP,
  INSTALLATION_STATUS_RESP,
  CALIBRATION_STATUS_RESP,
  OPERATION_STATUS_RESP,
  INTENT_STATUS_RESP,
  SENSORS_STATUS_RESP,
  CLUTCH_STATUS_RESP,
  SLEEP_MODE_RESP,
  ERROR_STATUS_RESP,
  EM_LOCK_STATUS_RESP
  //CURRENT_DOOR_POS = 1,
  //RESP_BATTERY_STATUS,
  //RESP_FW_VER,
  //RESP_DEV_STATUS,
  //DEV_STAT_REG_UPD
}ado_dev_stat_res_t;


typedef enum
{
  CONNECTED = 1,
  DISCONNECTED,
  INSTALLATION,
  INSTALLATION_STOP,
  CALIBRATION,
  CALIBRATION_STOP,
  OPERATION,
  OPERATION_STOP,
  PIR,
  PIR_STOP,
  OTA,
  OTA_STOP,
  DUMPING,
  DUMPING_STOP,
}ado_power_modes;

typedef struct 
{
   bool has_ADO_restarted;
   bool is_battery_charging;
}ado_dev_state;

extern ado_power_modes current_power_mode;

extern const k_tid_t   dev_stat_thrd_id;

extern float G_IMU;
extern float doorPositionFromClose;

extern ado_dev_state  ado_state;

extern bool run_in_intent_mode;
//battery percentage store variable
//extern uint8_t battery_percentage;
//battery percentage store variable
extern uint8_t pre_battery_percentage;


extern bool battery_low;

/**********************************************************************************************************
 * Function name  :    ADO_DevStat_Thread()
 *
 * Description    :   1. This is definition of ADO Device Status thread, which will be created immediately after
 *                       power ON and will keep listening to any commands received to its mailbox. 
 *                        
 *                    2. Once a command is received from the ED it will respond back with the respective data
 *                       Also it will keep posting the current door position to ED Periodically
 *
 * Params         :   None.
 *
 * Returns        :   Nothing.
 ***********************************************************************************************************/
void ADO_DevStat_Thread(void);

/**********************************************************************************************************
 * Function name  :   get_door_pos()
 *
 * Description    :   This function reads the current IMU sensor value 
 *               
 *
 * Params         :   None.
 *
 * Returns        :   <int8_t> IMU Sensor value, or error code.
 ***********************************************************************************************************/
int8_t get_door_pos(void);

/**********************************************************************************************************
 * Function name  :   ado_get_FW_Ver()
 *
 * Description    :   This function reads the current ADO firmware version string and reports to ED over BLE.
 *
 * Params         :   None.
 *
 * Returns        :   Nothing.
 ***********************************************************************************************************/
void ado_get_FW_Ver(void);

/**********************************************************************************************************
 * Function name  :   ADO_get_battery_state()
 *
 * Description    :   This function returns whether the ADO battery is charging or Not. It reads a global
 *                    ado_state structure variable for the same.
 *
 * Params         :   None.
 *
 * Returns        :   <bool> 'true' if the battery is charging otherwise 'false'.
 ***********************************************************************************************************/
bool ADO_get_battery_state(void);

/**********************************************************************************************************
 * Function name  :   ADO_get_restart_state()
 *
 * Description    :   This function will return whether the ADO has restarted or NOT.
 *
 * Params         :   None.
 *
 * Returns        :   <bool> 'true' if the ADO is restarted otherwise 'false'.
 ***********************************************************************************************************/
bool ADO_get_restart_state(void);

/**********************************************************************************************************
 * Function name  :   ADO_set_restart_state()
 *
 * Description    :   This function will set the restart state of ADO in a global ado_state structure variable.
 *
 * Params         :   <bool> state : The restarte state in which the ADO is to be set with.
 *
 * Returns        :   Nothing.
 ***********************************************************************************************************/
void ADO_set_restart_state( bool state);



void ado_device_version();
void peripheral_version();
void model_version();
void current_door_pos();
void ado_battery_percentage();
void ado_restart_status();
void device_status_reg_update();
void wifi_status();
void installation_status();
void calibration_status();
void operation_status();
void intent_status();
void sensors_status();
void clutch_status();
void sleep_mode();
void error_status();
void em_lock_status();
#endif
