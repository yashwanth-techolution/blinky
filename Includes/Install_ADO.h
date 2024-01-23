// ADO command manager Header file

#ifndef ADO_INSTALL_MODULE_H__
#define ADO_INSTALL_MODULE_H__

#include "Device_Status_Module.h"
#include <stdbool.h>
#include <stdint.h>

#define CLAMP_MOTOR_PER 50U

#define A_POSITION 1
#define B_POSITION 2
#define G_POSITION 3

#ifdef ADO_RAYTAC_P2
// Defining all cutt off current percentages for stepper motors.
#define V_U_CUT_OFF_PERCENT 20 // Vertical stepper current limit percentage in Unclamping.
#define V_C_CUT_OFF_PERCENT 20 // Vertical stepper current limit precentage in Clamping.
#define H_U_CUT_OFF_PERCENT 15 // Horizontal stepper current limit percentage in Unclamping.
#define H_C_CUT_OFF_PERCENT 30 // Horizontal stepper current limit perecentage in Clamping.

#define P_A_CUT_OFF_PERCENT 250 // Product balancer A point current limit percentage.
#define P_B_CUT_OFF_PERCENT 50  // Product balancer B point current limit percentage.
#define P_G_CUT_OFF_PERCENT 25  // Product balancer Ground touch point current limit percentage.

#define START_CURRENT_PEAK_VALUE 0.375f
#endif

#ifdef ADO_RAYTAC_P2_1
// Defining all cutt off current percentages for stepper motors.
#define V_U_CUT_OFF_PERCENT 22 // Vertical stepper current limit percentage in Unclamping.
#define V_C_CUT_OFF_PERCENT 22 // Vertical stepper current limit precentage in Clamping.
#define H_U_CUT_OFF_PERCENT 29 // Horizontal stepper current limit percentage in Unclamping.
#define H_C_CUT_OFF_PERCENT 40 // Horizontal stepper current limit perecentage in Clamping.

#define P_A_CUT_OFF_PERCENT 150 // Product balancer A point current limit percentage.
#define P_B_CUT_OFF_PERCENT 50  // Product balancer B point current limit percentage.
#define P_G_CUT_OFF_PERCENT 35  // Product balancer Ground touch point current limit percentage.

#define START_CURRENT_PEAK_VALUE 0.52f
#endif

// Cutt off limit to start claculating current percentages.
#define NOMINAL_CUT_OFF_VALUE 0.10f // minimum motor current threshold to start claculation of current percentage.

//#define CUR_SENSOR_CUT_OFF_V  2315U     // Current sensor Analog cutt of voltage in mV

// tempary commanted
//#define NO_OFF_ADC_SAMPLES    500U      // No.of Analog read input samples to calculate propere current values.

// Lead Screws (or) clamp travel length of Horizontal and Vertical Stepper Motors direction in (mm).
#define HOR_TRAV_LEN 72.5f // Clamp moving length in Horizontal direction in (mm).
#define VER_TRAV_LEN 50.5f // Clamp moving length in vertical direction in (mm).

// installation motors(hor, ver, product balancer) travel time with buffer time of Horizontal and Vertical clamps.
#define HOR_TRAVEL_TIME K_MSEC(7500)    // Horizontal Clamp max travel time in millis
#define VER_TRAVEL_TIME K_MSEC(9000)    // Vertical Clamp max travel time in millis.
#define PB_UP_TRAVEL_TIME K_MSEC(10000) // Vertical Clamp max travel time in millis.
#define PB_DW_TRAVEL_TIME K_MSEC(8000)
// Time delay constants to percentage calculation for stepper motors.
#define HOR_LEN_CAP_TIME_MILLIS 65U // Horizontal stepper time delay in (ms)
#define VER_LEN_CAP_TIME_MILLIS 65U // Vertical stepper time delay in (ms)

// ADO Installation state definitions
#define ADO_INSTALLED 1U
#define ADO_UNINSTALLED 2U

// Enum containing the ADO installation commands
typedef enum {
  INS_IDLE = 0,
  AUTO_CLAMP,
  AUTO_UNCLAMP,
  MAN_HOR_CLAMP,
  MAN_HOR_UNCLAMP,
  MAN_VER_CLAMP,
  MAN_VER_UNCLAMP,
  STOP_CLAMP,
  INS_START,
  INS_COMPLETE,
  PRODUCT_BALANCER_UP,
  PRODUCT_BALANCER_DOWN
} ado_installation_cmd_t;

// Enum containing the clampimng status values
typedef enum {
  EXTREME_POS = 1,
  RUNNING_HOR,
  RUNNING_VER,
  CLAMPED_HOR,
  UNCLAMPED_HOR,
  CLAMPED_VER,
  UNCLAMPED_VER,
  MAX_POS_HOR,
  MAX_POS_VER,
  JAMMED, // For future use
  UNSAFE_OPERATION,
  INSTALLATION_NOT_STARTED,
  BATTERY_LOW,
  INS_STARTED,
  INS_COMPLETED,
  CLAMP_STOPPED,
  IN_PROGRESS,
  PB_RUNNING_UP,
  PB_RUNNING_DOWN,
  PB_EXTREME_UP,
  PB_EXTREME_DOWN
} ado_installation_status_t;

typedef enum {
  PB_DOWN = 0,
  PB_UP,
  STOPPED_IN_BETWEEN
} pb_postions;

typedef enum {
  UNINSTALLED_BOT = 0,
  INSTALLED_BOT,
  PROCESS_GOING_ON,
  ERROR_IN_INSTALLING = 255
} intallation_enum;

// Structure to contain the installation related data
struct auto_clamp_data {
  uint8_t hor_pos;
  uint8_t ver_pos;
  uint8_t hor_per;
  uint8_t ver_per;
  uint8_t state_pos;
  uint8_t p_balancer;     // added by samuel --> stores the status of product balancer.
  uint8_t p_balancer_per; // added by samuel --> stores the status of product balancer.
  uint8_t run_pos;
  uint8_t response;
};

// Declaration of structure with variable name of "clamp_data"
extern struct auto_clamp_data clamp_data;
extern k_tid_t my_tid_install;

// Funtion Declarations
/****************************************************************************************************************
 * Function name  :   ado_install_uninstall()
 *
 * Description    :   1. It Starts as thread, with input data of installation command from Command_Manager_Module
 *                       type(uint8_t) and is responsible to translate it into respective ADO commands.
 *
 *                    2. Depending upon the input command, it will call the required functions with in this thread.
 *
 *                    3. This thread will be auto terminated when ever it receives "stop" command from
 *                       Command_Manager_Module or self terminated when this respected command task completed.
 *
 * Params         :   1. <in> uint8_t clamp_data.run_pos:
 *                             this is gloabal variable comming from command manager.
 *
 *                    2. <out> uint8_t clamp_data.hor_pos:
 *                            Global variable to store data of clamped or unclamped or other,
 *                            related to horizontal stepper.
 *
 *                    3. <out> uint8_t clamp_data.ver_pos:
 *                            Global variable to store data of clamped or unclamped or other,
 *                            related to vertical stepper.
 *
 *                    4. <out> uint8_t clamp_data.hor_per:
 *                            Global variable to store data of moving percentage,
 *                            related to horizontal stepper.
 *
 *                    5. <out> uint8_t clamp_data.ver_per:
 *                            Global variable to store data of moving percentage,
 *                            related to vertical stepper.
 *
 *                    6. <out> uint8_t clamp_data.state_pos:
 *                            Global variable to store data of Installed or Uninstalled or other,
 *                            related to ADO.
 *
 * Return         :     <int8_t>  0 on success, non-zero on failure.
 ***********************************************************************************************************/
extern uint8_t ado_install_uninstall(void);

/**********************************************************************************************************
 * Function name  :   product_balancer()
 *
 * Description    :   1. This function is used to control the product balancer motors while installing the
 *                       ADO on the door. It receives the command from ED over BLE and responds back.
 *
 * Params         :   1. <in> uint8_t command: Product balancer control commands
 *
 * Returns        :   0 on success, error code on failure.
 *
 ***********************************************************************************************************/
int8_t product_balancer(uint8_t command);

/**********************************************************************************************************
 * Function name  :   save_install_data()
 *
 * Description    :   Write the installation data to the ADO EEPROM Memory
 *
 * Params         :   None
 *
 * Returns        :   0 on success, error code on failure.
 *
 ***********************************************************************************************************/
int8_t save_install_data();

/**********************************************************************************************************
 * Function name  :   load_install_data()
 *
 * Description    :   Read the saved installation data from the ADO EEPROM Memory to global installation structure
 *
 * Params         :   None
 *
 * Returns        :   0 on success, error code on failure.
 *
 ***********************************************************************************************************/
int8_t load_install_data();

/**********************************************************************************************************
 * Function name  :   is_ADO_Installed()
 *
 * Description    :   Check whether the ADO is installed on the door or not
 *
 * Params         :   1. <in> struct auto_clamp_data * pClamp_data : Pointer to global installation structure.
 *
 * Returns        :   0 on success, error code on failure.
 *
 ***********************************************************************************************************/
bool is_ADO_Installed(struct auto_clamp_data *pClamp_data);
void clamp_motors_timer_expire(struct k_timer *timer);

#endif