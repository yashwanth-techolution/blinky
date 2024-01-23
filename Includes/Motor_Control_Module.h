// ADO motor control module header file

#ifndef MOTOR_CONTROL_TASK_H__
#define MOTOR_CONTROL_TASK_H__

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <hal/nrf_gpio.h> // for stepper motor GPIOs initialisation
#include <stdbool.h>
#include <stdint.h>

#include "Device_Status_Module.h"

// Custom Defines

#ifdef ADO_RAYTAC_P2
#define P_BAL_F 14
#define P_BAL_B 13
#define CLUTCH_F 8
#define CLUTCH_B 6
#endif

#ifdef ADO_RAYTAC_P2_1

#define P_BAL_F 6
#define P_BAL_B 8
#define CLUTCH_F 13
#define CLUTCH_B 14

#endif

// Motor Type Definitions
#define DRIVE_MOTOR 0
#define CLAMP_MOTOR 1

#define Enable_TruePWM true
#define Disable_TruePWM false

#define DRIVE_MOTOR_enable false
#define DRIVE_MOTOR_disable true
#define DRIVE_MOTOR_forward true
#define DRIVE_MOTOR_backward false

// directives of horizontal and vertical stepper motor
#define CLAMP_MOTOR_enable true
#define CLAMP_MOTOR_disable false
#define CLAMP_MOTOR_unclamp true
#define CLAMP_MOTOR_clamp false

#define PERCENTAGE_ToStop_Motor 0U

#define MOTOR_RELAY_TURN_OFF true // Active low GPIO pin requires true.
#define MOTOR_RELAY_TURN_ON false // Active low GPIO pin requires false.

#define MOTOR_RELAY_TURN_OFF true // Active low GPIO pin requires true.
#define MOTOR_RELAY_TURN_ON false // Active low GPIO pin requires false.

// Macro defining the default speed percent of motor in calibration process
#define DEF_CAL_MOTOR_SPEED_PERCENT 50U  // Default DC motor PWM duty cycle percent in calibration mode
#define MIN_OPR_MOTOR_SPEED_PERCENT 30U  // Minimum
#define MAX_OPR_MOTOR_SPEED_PERCENT 100U // 100U // Maximum

#define DRIVE_MOTOR_ON false
#define DRIVE_MOTOR_OFF true
#define DRIVE_MOTOR_DIR_FORWARD true
#define DRIVE_MOTOR_DIR_BACKWARD false

//#define DC_MOTOR_PWM_PERIOD_USEC      (USEC_PER_SEC / 20000U) // Main Motor
#define STPR_MOTOR_PWM_PERIOD_USEC (USEC_PER_SEC / 5000U) // Stepper motor Pulse Period

#ifdef ADO_RAYTAC_P2
// 1. For horizontal clamp stepper motor
//  a. Get an instance of Horizontal Clamp motor Pulse (PWM) Pin from the device tree (dts)
//     Note: It must be defined in dts file prior using it here, pls check if error.
#define HZ_STPR_PWM_NODE DT_ALIAS(stpr_pulse) // p0.13:stale

#if DT_NODE_HAS_STATUS(HZ_STPR_PWM_NODE, okay)

#define HZ_STPR_PWM_LABEL DT_PWMS_LABEL(HZ_STPR_PWM_NODE)
#define HZ_STPR_PWM_CHANNEL DT_PWMS_CHANNEL(HZ_STPR_PWM_NODE)
#define HZ_STPR_PWM_FLAGS DT_PWMS_FLAGS(HZ_STPR_PWM_NODE)

#else

#error "Error: 'hz_stpr_pulse' devicetree alias is not defined"
#define HZ_STPR_PWM_LABEL ""
#define HZ_STPR_PWM_CHANNEL 0
#define HZ_STPR_PWM_FLAGS 0

#endif

// b. Get an instance of Stepper motors driver sleep pin from the device tree (dts)
//    Note: It must be defined in dts file prior using it here
#define STPRS_SLP_NODE DT_ALIAS(stprs_slp)
#if DT_NODE_HAS_STATUS(STPRS_SLP_NODE, okay) && DT_NODE_HAS_PROP(STPRS_SLP_NODE, gpios)
#define STPRS_SLP_LABEL DT_GPIO_LABEL(STPRS_SLP_NODE, gpios)
#define STPRS_SLP_PIN DT_GPIO_PIN(STPRS_SLP_NODE, gpios)
#define STPRS_SLP_FLAGS (GPIO_OUTPUT | DT_GPIO_FLAGS(STPRS_SLP_NODE, gpios))
#endif

// b. Get an instance of Horizontal Clamp motor direction pin from the device tree (dts)
//    Note: It must be defined in dts file prior using it here
#define HZ_STPR_DIR_NODE DT_ALIAS(hz_stpr_dir)

#if DT_NODE_HAS_STATUS(HZ_STPR_DIR_NODE, okay) && DT_NODE_HAS_PROP(HZ_STPR_DIR_NODE, gpios)

#define HZ_STPR_DIR_LABEL DT_GPIO_LABEL(HZ_STPR_DIR_NODE, gpios)
#define HZ_STPR_DIR_PIN DT_GPIO_PIN(HZ_STPR_DIR_NODE, gpios)
#define HZ_STPR_DIR_FLAGS (GPIO_OUTPUT | DT_GPIO_FLAGS(HZ_STPR_DIR_NODE, gpios))

#endif

// c. Get an instance of Horizontal Clamp motor Enable pin from the device tree (dts)
//    Note: It must be defined in dts file prior using it here
#define HZ_STPR_EN_NODE DT_ALIAS(hz_stpr_en)
#if DT_NODE_HAS_STATUS(HZ_STPR_EN_NODE, okay) && DT_NODE_HAS_PROP(HZ_STPR_EN_NODE, gpios)

#define HZ_STPR_EN_LABEL DT_GPIO_LABEL(HZ_STPR_EN_NODE, gpios)
#define HZ_STPR_EN_PIN DT_GPIO_PIN(HZ_STPR_EN_NODE, gpios)
#define HZ_STPR_EN_FLAGS (GPIO_OUTPUT | DT_GPIO_FLAGS(HZ_STPR_EN_NODE, gpios))

#endif

// 2. For Vertical clamp stepper motor
//  a. Get an instance of Vertical Clamp motor Pulse (PWM) Pin from the device tree (dts)
//     Note: It must be defined in dts file prior using it here, pls check if error.
#define VR_STPR_PWM_NODE DT_ALIAS(stpr_pulse) // p0.13:stale

#if DT_NODE_HAS_STATUS(VR_STPR_PWM_NODE, okay)

#define VR_STPR_PWM_LABEL DT_PWMS_LABEL(VR_STPR_PWM_NODE)
#define VR_STPR_PWM_CHANNEL DT_PWMS_CHANNEL(VR_STPR_PWM_NODE)
#define VR_STPR_PWM_FLAGS DT_PWMS_FLAGS(VR_STPR_PWM_NODE)

#else
#error "Error: 'vr_stpr_pulse' devicetree alias is not defined"
#define VR_STPR_PWM_LABEL ""
#define VR_STPR_PWM_CHANNEL 0
#define VR_STPR_PWM_FLAGS 0

#endif

// b. Get an instance of Vertical Clamp motor direction pin from the device tree (dts)
//    Note: It must be defined in dts file prior using it here
#define VR_STPR_DIR_NODE DT_ALIAS(vr_stpr_dir)
#if DT_NODE_HAS_STATUS(VR_STPR_DIR_NODE, okay) && DT_NODE_HAS_PROP(VR_STPR_DIR_NODE, gpios)
#define VR_STPR_DIR_LABEL DT_GPIO_LABEL(VR_STPR_DIR_NODE, gpios)
#define VR_STPR_DIR_PIN DT_GPIO_PIN(VR_STPR_DIR_NODE, gpios)
#define VR_STPR_DIR_FLAGS (GPIO_OUTPUT | DT_GPIO_FLAGS(VR_STPR_DIR_NODE, gpios))
#endif

// c. Get an instance of Horizontal Clamp motor Enable pin from the device tree (dts)
//    Note: It must be defined in dts file prior using it here
#define VR_STPR_EN_NODE DT_ALIAS(vr_stpr_en)
#if DT_NODE_HAS_STATUS(VR_STPR_EN_NODE, okay) && DT_NODE_HAS_PROP(VR_STPR_EN_NODE, gpios)
#define VR_STPR_EN_LABEL DT_GPIO_LABEL(VR_STPR_EN_NODE, gpios)
#define VR_STPR_EN_PIN DT_GPIO_PIN(VR_STPR_EN_NODE, gpios)
#define VR_STPR_EN_FLAGS (GPIO_OUTPUT | DT_GPIO_FLAGS(VR_STPR_EN_NODE, gpios))
#endif

// 3. For O-Bot main-wheel DC motor
// a. Get an alias of PWM Pin
#define DC_MOTOR_PWM_NODE DT_ALIAS(m_motor_pwm)

#if DT_NODE_HAS_STATUS(DC_MOTOR_PWM_NODE, okay)
#define DC_MOTOR_PWM_LABEL DT_PWMS_LABEL(DC_MOTOR_PWM_NODE)
#define DC_MOTOR_PWM_CHANNEL DT_PWMS_CHANNEL(DC_MOTOR_PWM_NODE)
#define DC_MOTOR_PWM_FLAGS DT_PWMS_FLAGS(DC_MOTOR_PWM_NODE)
#else
#error "Error: 'm_motor_pwm' devicetree alias is not defined"
#define DC_MOTOR_PWM_LABEL ""
#define DC_MOTOR_PWM_CHANNEL 0
#define DC_MOTOR_PWM_FLAGS 0
#endif

// b. Get an alias of Motor direction GPIO Pin
#define DC_MOTOR_DIR_NODE DT_ALIAS(m_motor_dir)
#if DT_NODE_HAS_STATUS(DC_MOTOR_DIR_NODE, okay) && DT_NODE_HAS_PROP(DC_MOTOR_DIR_NODE, gpios)
#define DC_MOTOR_DIR_LABEL DT_GPIO_LABEL(DC_MOTOR_DIR_NODE, gpios)
#define DC_MOTOR_DIR_PIN DT_GPIO_PIN(DC_MOTOR_DIR_NODE, gpios)
#define DC_MOTOR_DIR_FLAGS (GPIO_OUTPUT | DT_GPIO_FLAGS(DC_MOTOR_DIR_NODE, gpios))
#endif

// c. Get an alias of Motor enable GPIO Pin
#define DC_MOTOR_EN_NODE DT_ALIAS(m_motor_en)
#if DT_NODE_HAS_STATUS(DC_MOTOR_EN_NODE, okay) && DT_NODE_HAS_PROP(DC_MOTOR_EN_NODE, gpios)
#define DC_MOTOR_EN_LABEL DT_GPIO_LABEL(DC_MOTOR_EN_NODE, gpios)
#define DC_MOTOR_EN_PIN DT_GPIO_PIN(DC_MOTOR_EN_NODE, gpios)
#define DC_MOTOR_EN_FLAGS (GPIO_OUTPUT | DT_GPIO_FLAGS(DC_MOTOR_EN_NODE, gpios))
#endif

// c. Get an alias of Motor's clutch GPIO Pin
#define DC_MOTOR_CLUTCH_NODE DT_ALIAS(m_motor_clutch)
#if DT_NODE_HAS_STATUS(DC_MOTOR_CLUTCH_NODE, okay) && DT_NODE_HAS_PROP(DC_MOTOR_CLUTCH_NODE, gpios)
#define DC_MOTOR_CLUTCH_LABEL DT_GPIO_LABEL(DC_MOTOR_CLUTCH_NODE, gpios)
#define DC_MOTOR_CLUTCH_PIN DT_GPIO_PIN(DC_MOTOR_CLUTCH_NODE, gpios)
#define DC_MOTOR_CLUTCH_FLAGS (GPIO_OUTPUT | DT_GPIO_FLAGS(DC_MOTOR_CLUTCH_NODE, gpios))
#endif

// b. Get an instance of motor driver relay control pin from the device tree (dts)
//    Note: It must be defined in dts file prior using it here
#define MOTOR_DRIVER_RELAY_NODE DT_ALIAS(motor_driver_relay)
#if DT_NODE_HAS_STATUS(MOTOR_DRIVER_RELAY_NODE, okay) && DT_NODE_HAS_PROP(MOTOR_DRIVER_RELAY_NODE, gpios)
#define DRIVER_RELAY_LABEL DT_GPIO_LABEL(MOTOR_DRIVER_RELAY_NODE, gpios)
#define DRIVER_RELAY_PIN DT_GPIO_PIN(MOTOR_DRIVER_RELAY_NODE, gpios)
#define DRIVER_RELAY_FLAGS (GPIO_OUTPUT | DT_GPIO_FLAGS(MOTOR_DRIVER_RELAY_NODE, gpios))
#endif

// Structure to be initialised with motor specific values.
struct motor_settings_t {
  char *pwm_label;
  char *dir_label;
  char *en_label;
  uint8_t pwm_channel;
  uint8_t dir_pin;
  uint8_t en_pin;
  uint32_t pwm_flags;
  uint32_t dir_flags;
  uint32_t en_flags;
  uint32_t pwm_period;
};

// Structure to contain the DTS instances of motor GPIOs
struct motor_params_t {
  const struct device *direction;
  const struct device *pwm;
  const struct device *enable;
  struct motor_settings_t settings;
  bool type;
  bool isRunning;
};
#endif

#ifdef ADO_RAYTAC_P2_1

// 1. For horizontal clamp stepper motor
//  a. Get an instance of Horizontal Clamp motor Pulse (PWM) Pin from the device tree (dts)
//     Note: It must be defined in dts file prior using it here, pls check if error.
#define STPR_PWM_NODE DT_ALIAS(stpr_pulse) // p0.13:stale

#if DT_NODE_HAS_STATUS(STPR_PWM_NODE, okay)
#define STPR_PWM_LABEL DT_PWMS_LABEL(STPR_PWM_NODE)
#define STPR_PWM_CHANNEL DT_PWMS_CHANNEL(STPR_PWM_NODE)
#define STPR_PWM_FLAGS DT_PWMS_FLAGS(STPR_PWM_NODE)
#else
#error "Error: 'hz_stpr_pulse' devicetree alias is not defined"
#define HZ_STPR_PWM_LABEL ""
#define HZ_STPR_PWM_CHANNEL 0
#define HZ_STPR_PWM_FLAGS 0
#endif

// b. Get an instance of Horizontal Clamp motor direction pin from the device tree (dts)
//    Note: It must be defined in dts file prior using it here
#define STPR_DIR_NODE DT_ALIAS(stpr_dir)
#if DT_NODE_HAS_STATUS(STPR_DIR_NODE, okay) && DT_NODE_HAS_PROP(STPR_DIR_NODE, gpios)
#define STPR_DIR_LABEL DT_GPIO_LABEL(STPR_DIR_NODE, gpios)
#define STPR_DIR_PIN DT_GPIO_PIN(STPR_DIR_NODE, gpios)
#define STPR_DIR_FLAGS (GPIO_OUTPUT | DT_GPIO_FLAGS(STPR_DIR_NODE, gpios))
#endif

// c. Get an instance of Horizontal Clamp motor Enable pin from the device tree (dts)
//    Note: It must be defined in dts file prior using it here
#define HZ_STPR_EN_NODE DT_ALIAS(hz_stpr_en)
#if DT_NODE_HAS_STATUS(HZ_STPR_EN_NODE, okay) && DT_NODE_HAS_PROP(HZ_STPR_EN_NODE, gpios)
#define HZ_STPR_EN_LABEL DT_GPIO_LABEL(HZ_STPR_EN_NODE, gpios)
#define HZ_STPR_EN_PIN DT_GPIO_PIN(HZ_STPR_EN_NODE, gpios)
#define HZ_STPR_EN_FLAGS (GPIO_OUTPUT | DT_GPIO_FLAGS(HZ_STPR_EN_NODE, gpios))
#endif

// c. Get an instance of Horizontal Clamp motor Enable pin from the device tree (dts)
//    Note: It must be defined in dts file prior using it here
#define VR_STPR_EN_NODE DT_ALIAS(vr_stpr_en)
#if DT_NODE_HAS_STATUS(VR_STPR_EN_NODE, okay) && DT_NODE_HAS_PROP(VR_STPR_EN_NODE, gpios)
#define VR_STPR_EN_LABEL DT_GPIO_LABEL(VR_STPR_EN_NODE, gpios)
#define VR_STPR_EN_PIN DT_GPIO_PIN(VR_STPR_EN_NODE, gpios)
#define VR_STPR_EN_FLAGS (GPIO_OUTPUT | DT_GPIO_FLAGS(VR_STPR_EN_NODE, gpios))
#endif

// 3. For O-Bot main-wheel DC motor
// a. Get an alias of PWM Pin
#define DC_MOTOR_PWM_NODE DT_ALIAS(m_motor_pwm)

#if DT_NODE_HAS_STATUS(DC_MOTOR_PWM_NODE, okay)
#define DC_MOTOR_PWM_LABEL DT_PWMS_LABEL(DC_MOTOR_PWM_NODE)
#define DC_MOTOR_PWM_CHANNEL DT_PWMS_CHANNEL(DC_MOTOR_PWM_NODE)
#define DC_MOTOR_PWM_FLAGS DT_PWMS_FLAGS(DC_MOTOR_PWM_NODE)
#else
#error "Error: 'm_motor_pwm' devicetree alias is not defined"
#define DC_MOTOR_PWM_LABEL ""
#define DC_MOTOR_PWM_CHANNEL 0
#define DC_MOTOR_PWM_FLAGS 0
#endif

// 3. For O-Bot main-wheel DC motor
// a. Get an alias of PWM Pin
//#define DC_MOTOR_PWM_B_NODE	DT_ALIAS(m_motor_pwm_b)

//#if DT_NODE_HAS_STATUS(DC_MOTOR_PWM_B_NODE, okay)
//#define DC_MOTOR_PWM_B_LABEL	DT_PWMS_LABEL(DC_MOTOR_PWM_B_NODE)
//#define DC_MOTOR_PWM_B_CHANNEL	DT_PWMS_CHANNEL(DC_MOTOR_PWM_B_NODE)
//#define DC_MOTOR_PWM_B_FLAGS	DT_PWMS_FLAGS(DC_MOTOR_PWM_B_NODE)
//#else
//#error "Error: 'm_motor_pwm' devicetree alias is not defined"
//#define DC_MOTOR_PWM_LABEL	""
//#define DC_MOTOR_PWM_CHANNEL	0
//#define DC_MOTOR_PWM_FLAGS	0
//#endif

#define DC_MOTOR_PWM_B_NODE DT_ALIAS(gearmotor_dir)
#if DT_NODE_HAS_STATUS(DC_MOTOR_PWM_B_NODE, okay) && DT_NODE_HAS_PROP(DC_MOTOR_PWM_B_NODE, gpios)
#define DC_MOTOR_PWM_B_LABEL DT_GPIO_LABEL(DC_MOTOR_PWM_B_NODE, gpios)
#define DC_MOTOR_PWM_B_CHANNEL DT_GPIO_PIN(DC_MOTOR_PWM_B_NODE, gpios)
#define DC_MOTOR_PWM_B_FLAGS (GPIO_OUTPUT | DT_GPIO_FLAGS(DC_MOTOR_PWM_B_NODE, gpios))
#endif

// c. Get an alias of Motor enable GPIO Pin
#define DC_MOTOR_EN_NODE DT_ALIAS(m_motor_en)
#if DT_NODE_HAS_STATUS(DC_MOTOR_EN_NODE, okay) && DT_NODE_HAS_PROP(DC_MOTOR_EN_NODE, gpios)
#define DC_MOTOR_EN_LABEL DT_GPIO_LABEL(DC_MOTOR_EN_NODE, gpios)
#define DC_MOTOR_EN_PIN DT_GPIO_PIN(DC_MOTOR_EN_NODE, gpios)
#define DC_MOTOR_EN_FLAGS (GPIO_OUTPUT | DT_GPIO_FLAGS(DC_MOTOR_EN_NODE, gpios))
#endif

// b. Get an instance of motor driver relay control pin from the device tree (dts)
//    Note: It must be defined in dts file prior using it here
#define MOTOR_DRIVER_RELAY_NODE DT_ALIAS(motor_driver_relay)
#if DT_NODE_HAS_STATUS(MOTOR_DRIVER_RELAY_NODE, okay) && DT_NODE_HAS_PROP(MOTOR_DRIVER_RELAY_NODE, gpios)
#define DRIVER_RELAY_LABEL DT_GPIO_LABEL(MOTOR_DRIVER_RELAY_NODE, gpios)
#define DRIVER_RELAY_PIN DT_GPIO_PIN(MOTOR_DRIVER_RELAY_NODE, gpios)
#define DRIVER_RELAY_FLAGS (GPIO_OUTPUT | DT_GPIO_FLAGS(MOTOR_DRIVER_RELAY_NODE, gpios))
#endif

// Structure to be initialised with motor specific values.
// extern int16_t door_percentage;
struct motor_settings_t {
  char *pwm_label;
  uint8_t pwm_channel;
  uint32_t pwm_flags;

  char *dir_label;
  uint8_t dir_pin;
  uint32_t dir_flags;

  char *en_label;
  uint8_t en_pin;
  uint32_t en_flags;
  uint32_t pwm_period;
};

// Structure to contain the DTS instances of motor GPIOs
struct motor_params_t {
  const struct device *direction;
  const struct device *pwm;
  const struct device *enable;
  struct motor_settings_t settings;
  bool type;
  bool isRunning;

  int start_interval;
  int end_interval;
  int decel_stop_interval;
  int fw_decel_stop_rpm;
  float gearbox_ratio;
  int micro_steps;
  float step_angle;
  int start_RPM;
  int fw_speed_cap;
  int bw_speed_cap;
  int end_RPM;
  int current_interval;
  int decel_start_interval; // for operation stop cmd

  // bool working;
  // bool working1;

  int current_speed_percentage;
  int steps;
  int skip_steps;
  int speed_change_percentage;
  int Bward_speed_change_percentage;

  int bw_start_interval;
  int bw_end_interval;
  int bw_decel_stop_interval;
  int bw_decel_stop_rpm;

  int bw_start_RPM;
  int bw_end_RPM;
  int bw_current_interval;
  int bw_current_speed_percentage;
  int bw_steps;
  int bw_skip_steps;
  int bw_speed_change_percentage;
};

#endif

// Declaration of motor_params to avail it to all files
extern struct motor_params_t Gear_motor_params;
extern struct motor_params_t hz_stepper_motor_params;
extern struct motor_params_t vr_stepper_motor_params;

// Function Declarations

/**********************************************************************************************************
 * Function name  :   motor_driver_power_relay_control()
 *
 * Description    :   1. This function turns ON/OFF particular Motor driver realy GPIO based on bool input.
 *
 * Params         :   1. <in> bool on:
 *                             boolean parameter to set motor driver power state.
 * Returns        :   Returns 0 if success
 *
 ***********************************************************************************************************/
int8_t motor_driver_power_relay_control(bool on);

/**********************************************************************************************************
 * Function name  :   initMotor()
 *
 * Description    :   1. Creates an instance of GPIO device tree bindings for PWM, Motor direction and
 *                       enable pins.
 *                    2. Initialises them to their initial state flags
 *                    3. Set the PWM Pin period in prescribed values as provided in the parameters
 *
 * Params         :   1. <out> struct motor_params_t * motor_params :
 *                            Pointer to a structure containing the motor initialisation settings, instances
 *                            of PWM, motor direction and motor enable pin device handles.These handles will
 *                            be required to control the motor post initialisation, therefore this param
 *                            needs to be pointing to a global instance of motor_params_t structure and can
 *                            be used throughout the program. Please note that it is an 'out' param i.e. will
 *                            be initialised inside this function.
 *
 *
 * Return         :     <int8_t>  0 on success, non-zero on failure.
 ***********************************************************************************************************/
int8_t initMotor(struct motor_params_t *motor_params);

/**********************************************************************************************************
 * Function name  :   rotateMotor()
 *
 * Description    :   1. It is responsible to set motor direction, PWM duty cycle percent and enable pin
 *                       of the motor whose motor_params have been supplied in the function call.
 *                    2. It expects a valid motor_params values set to the respective device tree
 *                       bindings of each of member variables i.e. PWM, direction and enable pins. Therefore
 *                       this function must be called after initMotor() succeeds.
 *
 * Params         :   1. <in> struct motor_params_t * motor_params :
 *                            Pointer to a structure containing instances of PWM, motor direction and motor
 *                            enable pin device handles.This parameter needs to be pointing to a global
 *                            instance of motor_params_t structure initialised by successfull initMotor().
 *
 *                    2. <in> uint8_t pwm_percent :
 *                            It will contain the unsigned value upto 100U to be set for the PWM Duty cycle
 *                            for the respective motor under operation as per provided motor_params. The 4th
 *                            param i.e enable must be 'true' for the motor to rotate else it will not effect
 *
 *                    3. <in> bool dir :
 *                            The boolean variable depicting the direction of rotation of the motor. Its
 *                            value could be 'true' or 'false' depending upon which the motor will rotate
 *                            either clockwise or anti-clockwise as per the motor driver attached.The 4th
 *                            param i.e enable must be 'true' for the motor to rotate else it will not effect
 *
 *                    4. <in> bool enable :
 *                            The boolean variable depicting the enable pin of the motor. Its value could be
 *                            'true' or 'false' depending upon which the motor will be enabled or disabled.
 *                            Setting this to 'false' will stop the motor irrespective of the values of 2nd
 *                            and 3rd params, therefore this can be used to stop the motor if it is rotating.
 *
 * Return         :     <int8_t>  0 on success, non-zero on failure.
 ***********************************************************************************************************/
int8_t rotateMotor(struct motor_params_t *motor_params, uint8_t pwm_percent, bool dir, bool enable, bool true_pwm);

// int8_t rotateMotor_true_pwm(struct motor_params_t * motor_params, uint8_t pwm_percent, bool dir, bool enable);

/**********************************************************************************************************
 * Function name  :   initAllMotors()
 *
 * Description    :   1. Wrapper function to initialise all motors i.e ADO wheel drive motor, horizontal
 *                       and vertical clamp motors at once.
 *
 * Params         :   None
 *
 * Return         :   <int8_t>  0 on success, non-zero on failure.
 ***********************************************************************************************************/
int8_t initAllMotors(void);

/**********************************************************************************************************
 * Function name  :   isMotorRunning(MotorType)
 *
 * Description    :   1. Warapper function to read the status of Motor whether running or stopped.
 *
 * Params         :   <in>  struct motor_params_t * motor_params: A referencr to motor params structure
 *
 * Return         :   <bool>  True if motor is running, False if motor is not running.
 ***********************************************************************************************************/
bool isMotorRunning(struct motor_params_t *motor_params);
void stopAllMotors(void);
int8_t Stop_Motor(struct motor_params_t *motor_params, bool dir, bool enable_disable, bool PWM_true);
void reset_accelation_values();
float fw_accrelation_interval(void);
float bw_accrelation_interval(void);
void deceleration_interval(struct motor_params_t *motor_params, bool dir); //,float value);
void Speed(int Int_RPM, int RPM, float GB, int uStep, float StepAngle, int bw_Int_RPM, int bw_RPM);
void motor_lock(struct motor_params_t *motor_params);
void staginate(void);
#endif