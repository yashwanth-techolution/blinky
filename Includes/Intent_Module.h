#ifndef INTENT_MODULE_H_
#define INTENT_MODULE_H_

#include <stdint.h>
#include <zephyr/kernel.h>

// The ADO Intent detection thread will be idle for 300 cycle of  MAX_MAILBOX_MSG_RECV_TIMEOUT ms 
// for ex 300 * 100 = 30000 ms = 30 sec, where  MAX_MAILBOX_MSG_RECV_TIMEOUT = 100ms
#define MAX_INTENT_THRD_IDLE_CYCL_CNT       350U


// Enum containing the ADO intent Wake word commands
typedef enum {
  DOOR_OPEN = 1,
  DOOR_CLOSE,
  DOOR_STOP,
  NON_WAKE_WORD,
  NOT_CONFIDENT,
}ado_intent_wake_word_cmds_t;

// Enum containing the ADO intent detection start/stop command
typedef enum {
  INT_START = 1,
  INT_STOP,
  INTENT_DETECT,
  INTENT_APPROVE,
  INTENT_START_DUMP,
  INTENT_STOP_DUMP,
}ado_intent_detection_cmd_t;

// Enum containing the ADO intent detection start/stop response
typedef enum {
  INT_STARTED = 1,
  INT_STOPPED,
  INT_DETECTED
}ado_intent_detection_res_t;

// Structure to contain ADO Intent related data
typedef struct {
  // Track keeping variables
  bool    isIntentEnabled;
  uint8_t thread_status;
  uint8_t cmd;
}ado_intent_t;

extern ado_intent_wake_word_cmds_t wake_words;
extern ado_intent_t intent;
extern k_tid_t ado_intent_thrd_id;
extern bool is_bot_staginated;
extern bool is_bot_staginated2,close_opr_done;
extern uint8_t is_door_in_open;
extern int16_t  current_door_percentage;
// Function Declarations

/**********************************************************************************************************
 * Function name  :   ADO_intent_detect_thread()
 *
 * Description    :   1. The Thread definition for detecting intent from PIR sensor when ever User try to 
 *                       Reach door from inner side. And calls Operation thread to open and close the door,
    *                    once it detects intent.
 *
 * Params         :   None
 *
 * Returns        :   Nothing
 ***********************************************************************************************************/
void ADO_intent_detect_thread();

/**********************************************************************************************************
 * Function name  :   execute_intent()
 *
 * Description    :   This function takes the intent_flag as input and checks if the intent is already detected,
 *                    if yes it sends a mailbox msg  to open the door to command manager and after a predefined
 *                    time, it will send a close door command.                  
 *
 * Params         :   None
 *
 * Returns        :   Nothing
 ***********************************************************************************************************/
void execute_intent();

/**********************************************************************************************************
 * Function name  :   is_Intent_Enabled()
 *
 * Description    :   This function checks the global intent structure variable for the intent enabled flag,
 *                    and will return the boolean state.
 *
 * Params         :   None
 *
 * Returns        :   <bool> true if enabled, false if disabled.
 ***********************************************************************************************************/
bool is_Intent_Enabled();

/**********************************************************************************************************
 * Function name  :   ado_toggle_intent()
 *
 * Description    :   This function will take the command whether to enable or disable the intent detection 
 *                    feature as received over BLE from the Edge device                 
 *
 * Params         :   <bool> intent_cmd : boolean value to which the intent feature is to be set
 *
 * Returns        :   Nothing
 ***********************************************************************************************************/
void ado_toggle_intent(bool intent_cmd);

#endif