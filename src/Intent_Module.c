#include "Includes/Install_ADO.h"
#include "Includes/Intent_Module.h"
#include "Includes/ADO_PIR_Module.h"
#include "Includes/Display_Module.h"
#include "Includes/ADO_STM32H7_WW.h"
#include "Includes/IPC_Manager_Module.h"
#include "Includes/Calibration_Module.h"
#include "Includes/Motor_Control_Module.h"
#include "Includes/ADO_Operations_Module.h"
#include "Includes/Command_Manager_Module.h"

// Global variable to hold the ADO Operation state
ado_intent_t intent;
ado_intent_wake_word_cmds_t wake_words;
bool is_bot_staginated = false;
bool is_bot_staginated2 = false,close_opr_done = false;
uint8_t is_door_in_open = 0;

// Global variable for holding door open/close percentage.
int16_t current_door_percentage;

/*
 *  thread function to start detecting intent through PIR sensor
 */
void ADO_intent_detect_thread() {
  uint8_t intent_cmd;
  uint8_t intent_detected;
  uint16_t thrd_idle_cycle_cnt = 0U;
  uint16_t time_count = 0U;
  bool send_door_operation_cmd = false;
 
  // Mailbox variables
  struct k_mbox_msg recv_msg;
 
  // Disable PIR     
 
  //run_syntiant();
  //check_stm32_ww();

  // Run thread untill receive intent detection stop command or MAX_INTENT_THRD_IDLE_CYCL_CNT cycles
  //while(MAX_INTENT_THRD_IDLE_CYCL_CNT >= thrd_idle_cycle_cnt)     // modified by samuel
  while(1) {
    // prepare the mailbox receive buffer
    recv_msg.info = 0;
    recv_msg.size = sizeof(intent_cmd);
    recv_msg.rx_source_thread = cmd_mgr_thrd_id;

    // retrieve and delete the message if received
    k_mbox_get(&cmd_mgr_mb, &recv_msg, &intent_cmd, MAX_MAILBOX_MSG_RECV_TIMEOUT);
    
    switch(intent_cmd) {
      case INT_START:
        // Debug print
        printk("Intent detection started\n");
        
        // Notify over BLE that intent detection started
        //ADO_notify_ble(cmd_mgr_thrd_id, ADO_INTENT_RESPONSE, INT_START, INT_STARTED, NULL, 0);

        // Change the Display Page to "say-Open-Door".
        //chg_display_page(DISP_SAY_DOOR_OPEN_PAGE);      //state change

        if(!(battery_low)) {
            chg_display_page(DISP_SAY_DOOR_OPEN_PAGE);
        }

        intent_cmd = 0;
        thrd_idle_cycle_cnt = 0U;      
        break;

      case INT_STOP:
        // Debug print
        printk("Intent detection stopped\n");

        // Notify over BLE that intent detection is now stopped
        //ADO_notify_ble(cmd_mgr_thrd_id, ADO_INTENT_RESPONSE, INT_STOP, INT_STOPPED, NULL, 0);
                
        // Debug print
        printk("Exiting Intent thread\n");
        if(is_Intent_Enabled() == true) {
          k_sleep(K_MSEC(2000U));
          //enable_pir_1();
        }
        return;

      default:
        // Increment the idle count
        ++thrd_idle_cycle_cnt;
        // Debug print
        intent_cmd = 0;
        // printk("-");
        break;
    }
   
    // poll the voice AI chip for any wake word command detected
    //intent_detected = run_syntiant(); 
    intent_detected = check_stm32_ww();

    switch(intent_detected) {
      case DOOR_OPEN:
        //stm32_audio_files_read();
        // Notify over BLE that intent detected
        ADO_notify_ble(cmd_mgr_thrd_id, ADO_INTENT_RESPONSE, INT_START, INT_DETECTED, NULL, 0);
        send_door_operation_cmd = true;
        time_count = 0;
        break;

      case DOOR_CLOSE:
        // Notify over BLE that intent detected
        ADO_notify_ble(cmd_mgr_thrd_id, ADO_INTENT_RESPONSE, INT_START, INT_DETECTED, NULL, 0);
        send_door_operation_cmd = true;
        time_count = 30;
        break;

      case NON_WAKE_WORD:
        stm32_audio_files_read();
        stm32_version_numbers_read();
        break;
    
      default:
        break;
    }

    if(send_door_operation_cmd) { 
      if(0 == time_count) {   
        // Debug print
        printk("Door Open Intent Detected\n");

        // Notify over BLE that intent detected
        ADO_notify_ble(cmd_mgr_thrd_id, ADO_INTENT_RESPONSE, INT_START, INT_DETECTED, NULL, 0);
        
        // Send the Open-Door command to cmd_mgr
        ADO_notify_cmd_mgr(COMMAND, ADO_OPERATIONS, OPEN, NULL, 0);
      } else if(150 == time_count) {  
        stm32_audio_files_read();
        printk("== Door close send from intent module ==\n");

        // Send the Close-Door command to cmd_mgr
        ADO_notify_cmd_mgr(COMMAND, ADO_OPERATIONS, CLOSE, NULL, 0);
      } else if(300 <= time_count) {
        send_door_operation_cmd = false;
      }
      ++time_count;
    }
  }
  printk("Exiting Intent thread\n");

  // Change the Display Page back to Default 
  //chg_display_page(DISP_START_PAGE);
  //enable_pir_1();  
}

/*
 *  brief: This function will return whether the intent has been enabled in device configurations
 */
bool is_Intent_Enabled() {
  // TODO: Read the Intent config setting from EEPROM when device starts and return the same, for now return true
   return intent.isIntentEnabled;
}

/*
 *  brief: This function will enable/disable the intent as received in configuration command 
 *         from ED and will save to EEPROM 
 */
void ado_toggle_intent(bool intent_cmd) {
   char * thread_state = NULL;

   if(intent_cmd == true) { //Enable the intent
      //check if it is NOT already enabled
      if(is_Intent_Enabled() == false) {
        // check if device is installed and calibrated
        if((is_ADO_Installed(&clamp_data) == true) && (is_ADO_Calibrated(&calib) == true)) {
          // Enable the PIR sensor and update in EEPROM
          enable_pir_1();

          // TODO: Update in EEPROM, for now write into a global variable
          intent.isIntentEnabled = true;
          save_config_data();
        } else {
          // Debug prints
          //printk("ado_toggle_intent(): ADO not installed or uncalibrated\r\n");
          // TODO: notify over BLE for the error
          return;
        }
      }
      // Notify over BLE that intent is now/already enabled
      ADO_notify_ble(ble_notify_thrd_id, ADO_CONFIG_RESPONSE, INTENT_CONTROL, INTENT_STATUS, (uint8_t *)&intent.isIntentEnabled, 1);
   } else if(intent_cmd == false) { //Disable the intent
      //check if it is NOT already disabled
      if(is_Intent_Enabled() == true) {
        // Check if the intent thread was already running, and abort it
        thread_state = k_thread_state_str(ado_intent_thrd_id,0,0);

        // debug prints
        //printk("Intent Thread is in %s state\n", thread_state);

        if(!is_thread_dead(thread_state)) {
           k_thread_abort(ado_intent_thrd_id);

           // The display might be showing "Say-door-open", remove that screen
           //chg_display_page(DISP_START_PAGE);     //commented by ashok
        }
        printk("Intnent 2");
        // Disable the PIR sensor, so that intent can't be enabled untill INTENT_START is received over BLE
        disable_pir_1();

        // TODO: Update in EEPROM, for now write into a global variable:Done
        intent.isIntentEnabled = false;
        save_config_data();
      }
      // Report over BLE that Intent is now/already disabled
      ADO_notify_ble(ble_notify_thrd_id, ADO_CONFIG_RESPONSE, INTENT_CONTROL, INTENT_STATUS, &intent.isIntentEnabled, 1);  
   } else {
      // Debug prints
      //printk("ado_toggle_intent(): Invalid command\r\n");
   }  
}

