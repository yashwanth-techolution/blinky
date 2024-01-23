// ADO command manager Header file 

#ifndef __COMMAND_MANAGER_MODULE_H__
#define __COMMAND_MANAGER_MODULE_H__

#include<stdint.h>
#include<stdbool.h>
#include <zephyr/kernel.h>

#include "Includes/BLE_Module.h"
#include "Includes/Device_Status_Module.h"

extern int auto_close_timer;
extern bool Auto_close_timer_off;

// Enum containing the type of commands
typedef enum
{
  ADO_CALIBRATION = 1,
  ADO_OPERATIONS,
  ADO_DEVICE_STATUS,
  ADO_COMMAND_RESPONSE,
  ADO_EVENT_REPORT,
  ADO_DEVICE_STATUS_RESPONSE,
  ADO_PARAMETER_RESPONSE,
  ADO_CONFIG,
  ADO_INSTALLATION,
  ADO_INSTALLATION_RESPONSE,
  ADO_CALIBRATION_RESPONSE,
  ADO_OPERATIONS_RESPONSE,
  ADO_CONFIG_RESPONSE,
  ADO_INTENT_REQUEST,
  ADO_INTENT_RESPONSE,
  ADO_RESET_REQUEST,
  ADO_RESET_RESPONSE,
  ADO_WIFI_CONTROL,
  ADO_WIFI_CONTROL_RESPONSE,
  ADO_OTA_UPDATE,
  ADO_OTA_UPDATE_RESPONSE,
  DOORBOT_EM_LOCK,
  UWB_dummy,
  UWB_PARAMS,
  UWB_PRAMS_RESPONSE
}ado_cmd_type_t;


/* added by samuel */
// Enum containing the ADO wi-fi control commands
//typedef enum
//{
//  WIFI_MODULE_ENABLE = 1,
//  WIFI_MODULE_DISABLE,
//  WIFI_CONNECT,
//  WIFI_DISCONNECT,
//  WIFI_SSID,
//  WIFI_PASSWORD,
//  WIFI_AUTO_CONNECT,
//  WIFI_NETWORK_SCAN
//}ado_wifi_control_cmd_t;

///* added by samuel */
//// Enum containing the ADO wi-fi control response
//typedef enum
//{
//  WIFI_NO_SHIELD = 1,
//  WIFI_MODULE_ENABLED,
//  WIFI_MODULE_DISABLED,
//  WIFI_CONNECTED_RSSI,
//  WIFI_CONNECT_FAIL,
//  WIFI_CONNECTION_LOST,
//  WIFI_DISCONNECTED,
//  WIFI_NO_SSID_AVAILABLE,
//  WIFI_SCAN_COMPLETED
//}ado_wifi_control_resp_t;



//-----------------------------added by ashok for wifi_cred ----------//

// Enum containing the ADO wi-fi control commands
typedef enum
{
  WIFI_MODULE_STATE = 1,
  WIFI_SCAN,
  WIFI_SSID,
  WIFI_PASSWORD,
  WIFI_CONNECT,
  WIFI_DISCONNECT,
  WIFI_CONTROL_CONNECTION_STATUS_CONN,
  WIFI_SIGNAL_STRENGTH,
  WIFI_RECONNECT,
  WIFI_CONNECTED_SSID           //--testing
 // WIFI_STATUS

}ado_wifi_control_cmd_t;

// Enum containing the ADO wi-fi control response
typedef enum
{
  WIFI_MODULE_STATE_RESP = 1,
  WIFI_SCAN_STATE_RESP,
  WIFI_SSID_RESP,
  WIFI_PASS_RESP,
  WIFI_CONNECT_RESP,
  WIFI_DISCONNECT_RESP,
  WIFI_CONTROL_CONNECTION_STATUS_RESP_CONN,
  WIFI_SIGNAL_STRENGTH_RESP,
  WIFI_RECONNECT_RESP,
  WIFI_CONNECTED_SSID_RESP        //-- testing
}ado_wifi_control_resp_t;


//--------------------------------------end-----------------------------//



/* added by Aijaz */
//*************************ADO_OTA_UPDATE***************//
typedef enum
{
  CHECK_UPDATE = 1,
  UPDATE_TARGET_TYPE,
  UPDATE_INFO,
  START_UPDATE,
  CANCEL_UPDATE,
  UPDATE_PROGRESS,
  STM_UPDATE_VERSION,
  NORDIC_UPDATE_VERSION,
  WIFI_MOD_OTA_START,
  WIFI_MOD_OTA_STOP,
  WIFI_MOD_OTA_STATUS
}ado_ota_update_cmd_t;

typedef enum
{
  CHECK_UPDATE_RESP = 1,
  UPDATE_TARGET_TYPE_RESP,
  UPDATE_INFO_RESP,
  START_UPDATE_RESP,
  CANCEL_UPDATE_RESP,
  UPDATE_PROGRESS_RESP,
  STM_UPDATE_VERSION_RESP,
  NORDIC_UPDATE_VERSION_RESP,
  WIFI_MOD_OTA_START_RESP,
  WIFI_MOD_OTA_STOP_RESP,
  WIFI_MOD_OTA_STATUS_RESP
}ado_ota_update_resp;
//****************************ADO_OTA_UPDATE_END**********************// 


// Enum containing the ADO configuration commands
typedef enum
{
  SET_SPEED = 1,
  INTENT_CONTROL,
  REQ_ADO_CONFIG,
  DISPLAY_CONTROL,
  CLUTCH_CONTROL,
  AUTO_CLOSE_TIME,
  AUTO_CLOSE_TIMER_OFF
}ado_config_cmd_t;

// Enum containing the ADO configuration command response
typedef enum
{
  SPEED_SET = 1,
  INTENT_STATUS,
  RESP_ADO_CONFIG,
  DISPLAY_STATE,
  CLUTCH_STATE,
  AUTO_CLOSE_TIME_RESP,
  AUTO_CLOSE_TIMER_OFF_RESP
}ado_config_resp_t;

// Enum containing the ADO command types
typedef enum
{
  COMMAND = 1,
  RESPONSE,
}ado_cmd_msg_type_t;

// Enum containing the ADO response message types
typedef enum
{
  BLE_NOTIFY = 1,
  DISPLAY,
}ado_res_msg_type_t;

typedef struct
{
  uint16_t  msg_buf_len;
  uint8_t   msg_type;
  uint8_t   msg_cmd;
  uint8_t   msg_buf[MAX_BLE_PCKT_MSG_LEN];
  //k_tid_t   src_thrd_id;    // for future use
  //k_tid_t   dst_thrd_id;    // for future use
}ado_cmd_mgr_msg_t;

extern bool ssid_flag;
extern bool pass_flag;

extern uint8_t         config_pwm_percent;
extern const k_tid_t   cmd_mgr_thrd_id; 
extern struct k_mbox   cmd_mgr_mb;
extern bool stop_wifi_send;  // added by samuel
extern bool esp32_OTA_start;

//added by ashok for wifi-cred and different operations
extern bool emergency_exit;      //added by ashok
extern bool new_wifi;
extern bool reconnect_to_wifi;
extern bool new_wifi_cred;
extern uint8_t instal_start_cmd;
extern bool ota_info;


//temp
extern ble_cmd_packet_info_t cmd_info;


// Funtion Declarations

/**********************************************************************************************************
 * Function name  :   ado_operation()
 *
 * Description    :   1. This function called by edge device to execute open/close operation of door
 *                       when it recorgonized with  valid face. And this function called from intent
 *                       detection thread also to execute open/close operation of door, when valid 
 *                       intent detected by PIR sensor.
 *                    2. This function will creates the ADO operation thread if it not created and passes
 *                       open, close, stop door command to created thread.
 *
 * Params         :   1. <in> uint8_t operate_cmd :  
 *                            type define stucture that contains door operation executable commands of
 *                            open, close, stop.
 *
 ***********************************************************************************************************/
extern void ado_operation(uint8_t operate_cmd);

/**********************************************************************************************************
 * Function name  :   ado_installation()
 *
 * Description    :   1. This function is required to execute installation of doorbot to the door.
 *                       It creates a new thread to run all the installation related commands and,
 *                       sends a mailbox message to it with the received command and related data.
 *                        
 *                    2. If the thread is already running, it just sends a mailbox message to it.
 *                       but if the thread has not started but we received any other command than
 *                       INS_START, it will notify over BLE that the Installation thread is not 
 *                       started.
 *
 * Params         :   1. <in> uint8_t installation_cmd:  
 *                            Installation/clampng command for the doorbot.
 *
 * Returns        :   Nothing.
 ***********************************************************************************************************/
void ado_installation(uint8_t installation_cmd);

/**********************************************************************************************************
 * Function name  :    ado_intent()
 *
 * Description    :   1. This function is required to execute the user intent detection logic to either
 *                       open/close the door.
 *                        
 *                    2. Once a command is received from the ED to start the intent detection, a new thread
 *                       will be created and it will keep listening to any PIR sensor detection of a human
 *                       presence/signal to open the door from inside of room. Once opened, it will be open
 *                       for a set timeout, and will close if the timeout is elapsed.
 *
 * Params         :   1. <in> uint8_t intent_detection_cmd:  
 *                            Intent detection start/stop command.
 *
 * Returns        :   Nothing.
 ***********************************************************************************************************/
void ado_intent(uint8_t intent_detection_cmd);

/**********************************************************************************************************
 * Function name  :    ado_intent()
 *
 * Description    :   1. This function is called when there is any door status request received from ED.
 *                      
 *                    2. Once a command is received from the ED a mailbox message will be passed to dev_stat
 *                       thread and that thread will be executing the same.
 *
 * Params         :   1. <in> uint8_t dev_status_cmd:  
 *                            Device status query command.
 *
 * Returns        :   Nothing.
 ***********************************************************************************************************/
void ado_dev_status(uint8_t  dev_status_cmd);

/**********************************************************************************************************
 * Function name  :    ADO_CmdMgr_Thread()
 *
 * Description    :   1. This is definition of command manager thread, which will be created immediately after
 *                       power ON and will keep listening to any commands received to its mailbox. 
 *                        
 *                    2. Once a command is received from the ED or any source in future, as per it's type
 *                       whether it is a command from external source like BLE, Display, Voice etc. Or it
 *                       is a response message from any internal thread which is to be conveyed to some 
 *                       other thread.
 *
 * Params         :   None.
 *
 * Returns        :   Nothing.
 ***********************************************************************************************************/
void ADO_CmdMgr_Thread();

/**********************************************************************************************************
 * Function name  :   ADO_notify_cmd_mgr()
 *
 * Description    :   1. This function takes the required parameters to be notified from any thread to cmd_mgr,
 *                       prepares a message to the cmd_mgr thread and posts it to cmd_mgr's mailbox. 
 *
 * Params         :   1. <in> uint8_t msg_type :  
 *                            The type of message we are sending to cmd_mgr. It can be a COMMAND or RESPONSE
 *
 *                    2. <in> uint8_t cmd_type:   
 *                            The type of command or response for example. ADO_CALIBRATION
 *
 *                    3. <in>  uint8_t cmd :   
 *                            The command or operation that needs to be performed by ADO. As in our example
 *                            of ADO_CALIBRATION case, the value of this variable could be CAL_TERMINATE.
 *        
 *                    4. <in> uint8_t *cmd_msg :   
 *                            A pointer to byte array of extra bytes that needs to be sent as a part of sent command.
 *                            This pointer must be passed with an address of a valid cmd_msg[] buffer with respective values.
 *                            
 *                    5. <in> uint8_t cmd_msg_len :
 *                            A variable containing the number of extra bytes in cmd_msg            
 *
 * Return         :    Nothing. 
 ***********************************************************************************************************/
void ADO_notify_cmd_mgr(uint8_t msg_type, uint8_t cmd_type, uint8_t cmd, uint8_t *cmd_msg, uint8_t cmd_msg_len);

/**********************************************************************************************************
 * Function name  :    save_config_data()
 *
 * Description    :   This function reads the global configuration variables and stores their values to EEPROM
 *
 * Params         :   None.
 *
 * Returns        :   Nothing.
 ***********************************************************************************************************/
void save_config_data();

/**********************************************************************************************************
 * Function name  :   load_config_data()
 *
 * Description    :   This function reads the saved configuration variables from EEPROM to respective global
 *                    variables.
 *
 * Params         :   None.
 *
 * Returns        :   Nothing.
 ***********************************************************************************************************/
void load_config_data();

/**********************************************************************************************************
 * Function name  :   ado_toggle_display()
 *
 * Description    :   This function enables or disables the ADO display as per the received boolean argument
 *                    and notifies over BLE for the current state, it has been added just for testing.
 *
 * Params         :   <in> bool display_cmd: Command depicting whether to turn Display ON/OFF, true -> ON
 *
 * Returns        :   Nothing.
 ***********************************************************************************************************/
void ado_toggle_display(bool display_cmd);
void ADO_notify_short( k_tid_t dest_thrd_id, float *msg, uint8_t len);
bool is_thread_dead(char *ptr);
void ado_toggle_motor( bool motor_cmd);
void ado_toggle_wifi( bool wifi_cmd);
void ado_toggle_stm32( bool stm32_cmd);
void power_optimization(ado_power_modes mode);
void timer_expire(struct k_timer *timer);
void abort_all_thread();
void timer_expire_intent(struct k_timer *timer);
int speedmap(long x, long in_min, long in_max, long out_min, long out_max);
//-------------------------added by ashok ----------------------------------//
void save_auto_close_data(void);
void load_auto_close_data(void);
void  ado_wifi_status(uint8_t);
extern bool settings_open;      //tsi added by ashok

extern bool display_activity; // added by ashok

void  wifi_connect_cmd_call(void);

#endif
