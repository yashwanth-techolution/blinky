#include "Includes/Motor_Control_Module.h"
#include <dk_buttons_and_leds.h>
#include <zephyr/sys/printk.h>

#include "Includes/Calibration_Module.h"
// Initialise the motor settings

#ifdef ADO_RAYTAC_P2_1
// Motor control variables
// 1. Main DC Motor settings
// int16_t door_percentage=0;

struct motor_params_t Gear_motor_params = {
    .type = DRIVE_MOTOR,
    .isRunning = false,
    .settings.pwm_label = DC_MOTOR_PWM_LABEL,
    .settings.pwm_channel = DC_MOTOR_PWM_CHANNEL,
    .settings.pwm_flags = DC_MOTOR_PWM_FLAGS,

    .settings.pwm_period = 120, // usec

    .settings.dir_label = DC_MOTOR_PWM_B_LABEL,
    .settings.dir_pin = DC_MOTOR_PWM_B_CHANNEL,
    .settings.dir_flags = DC_MOTOR_PWM_B_FLAGS,

    .settings.en_label = DC_MOTOR_EN_LABEL,
    .settings.en_flags = DC_MOTOR_EN_FLAGS,
    .settings.en_pin = DC_MOTOR_EN_PIN,

    .decel_start_interval = 0, // deceleration srat interval for stop cmd
    .gearbox_ratio = 5.18f,    // 5.18f,
    .micro_steps = 32,         // 16,
    .step_angle = 1.8f,

    .start_interval = 0,
    .end_interval = 0,
    .current_interval = 0,
    .current_speed_percentage = 0,
    .steps = 0,
    .end_RPM = 0,
    .decel_stop_interval = 0,

    .bw_start_interval = 0,
    .bw_end_interval = 0,
    .bw_current_interval = 0,
    .bw_current_speed_percentage = 0,
    .bw_steps = 0,
    .bw_end_RPM = 45,
    .bw_decel_stop_interval = 0,
    //----------------------forward cofig parameters------------------------------------
    .start_RPM = 15, // 20, // forward start RPM
    .fw_speed_cap = 65,
    .fw_decel_stop_rpm = 15, //  121 ==  15 rpm 72,//45,//52,// 45,//52,//60,// 36,//60,// 90, // 181 =  20rpm
    //----------------------backward cofig parameters------------------------------------
    .bw_start_RPM = 6,
    .bw_speed_cap = 55,
    .bw_decel_stop_rpm = 25,
};

// 2. Horizontal Stepper Motor settings
struct motor_params_t hz_stepper_motor_params = {
    .type = CLAMP_MOTOR,
    .isRunning = false,
    .settings.pwm_label = STPR_PWM_LABEL,
    .settings.pwm_channel = STPR_PWM_CHANNEL,
    .settings.pwm_flags = STPR_PWM_FLAGS,
    .settings.pwm_period = STPR_MOTOR_PWM_PERIOD_USEC,

    .settings.dir_label = STPR_DIR_LABEL,
    .settings.dir_flags = STPR_DIR_FLAGS,
    .settings.dir_pin = STPR_DIR_PIN,

    .settings.en_label = HZ_STPR_EN_LABEL,
    .settings.en_flags = HZ_STPR_EN_FLAGS,
    .settings.en_pin = HZ_STPR_EN_PIN,
};

// 3. Vertical Stepper Motor settings
struct motor_params_t vr_stepper_motor_params = {
    .type = CLAMP_MOTOR,
    .isRunning = false,
    .settings.pwm_label = STPR_PWM_LABEL,
    .settings.pwm_channel = STPR_PWM_CHANNEL,
    .settings.pwm_flags = STPR_PWM_FLAGS,
    .settings.pwm_period = STPR_MOTOR_PWM_PERIOD_USEC,

    .settings.dir_label = STPR_DIR_LABEL,
    .settings.dir_flags = STPR_DIR_FLAGS,
    .settings.dir_pin = STPR_DIR_PIN,

    .settings.en_label = VR_STPR_EN_LABEL,
    .settings.en_flags = VR_STPR_EN_FLAGS,
    .settings.en_pin = VR_STPR_EN_PIN,
};
#endif

#ifdef ADO_RAYTAC_P2
// Motor control variables
// 1. Main DC Motor settings
struct motor_params_t Gear_motor_params = {
    .type = DRIVE_MOTOR,
    .isRunning = false,
    .settings.pwm_label = DC_MOTOR_PWM_LABEL,
    .settings.pwm_channel = DC_MOTOR_PWM_CHANNEL,
    .settings.pwm_flags = DC_MOTOR_PWM_FLAGS,
    .settings.pwm_period = DC_MOTOR_PWM_PERIOD_USEC,

    .settings.dir_label = DC_MOTOR_DIR_LABEL,
    .settings.dir_flags = DC_MOTOR_DIR_FLAGS,
    .settings.dir_pin = DC_MOTOR_DIR_PIN,

    .settings.en_label = DC_MOTOR_EN_LABEL,
    .settings.en_flags = DC_MOTOR_EN_FLAGS,
    .settings.en_pin = DC_MOTOR_EN_PIN,
};

// 2. Horizontal Stepper Motor settings
struct motor_params_t hz_stepper_motor_params = {
    .type = CLAMP_MOTOR,
    .isRunning = false,
    .settings.pwm_label = HZ_STPR_PWM_LABEL,
    .settings.pwm_channel = HZ_STPR_PWM_CHANNEL,
    .settings.pwm_flags = HZ_STPR_PWM_FLAGS,
    .settings.pwm_period = STPR_MOTOR_PWM_PERIOD_USEC,

    .settings.dir_label = HZ_STPR_DIR_LABEL,
    .settings.dir_flags = HZ_STPR_DIR_FLAGS,
    .settings.dir_pin = HZ_STPR_DIR_PIN,

    .settings.en_label = HZ_STPR_EN_LABEL,
    .settings.en_flags = HZ_STPR_EN_FLAGS,
    .settings.en_pin = HZ_STPR_EN_PIN,
};

// 3. Vertical Stepper Motor settings
struct motor_params_t vr_stepper_motor_params = {
    .type = CLAMP_MOTOR,
    .isRunning = false,
    .settings.pwm_label = VR_STPR_PWM_LABEL,
    .settings.pwm_channel = VR_STPR_PWM_CHANNEL,
    .settings.pwm_flags = VR_STPR_PWM_FLAGS,
    .settings.pwm_period = STPR_MOTOR_PWM_PERIOD_USEC,

    .settings.dir_label = VR_STPR_DIR_LABEL,
    .settings.dir_flags = VR_STPR_DIR_FLAGS,
    .settings.dir_pin = VR_STPR_DIR_PIN,

    .settings.en_label = VR_STPR_EN_LABEL,
    .settings.en_flags = VR_STPR_EN_FLAGS,
    .settings.en_pin = VR_STPR_EN_PIN,
};
#endif

const struct device *driver_relay;

/*
 * brief: This function initializes the respected GPIO required to turn ON/OFF motor driver.
 */
void init_motor_driver_relay_control() {
  int8_t err;

  // Motor driver relay control GPIO pin Initialisations
  driver_relay = device_get_binding(DRIVER_RELAY_LABEL);
  if (driver_relay == NULL) {
    printk("Couldn't find motor driver relay control GPIO device%s\n", DRIVER_RELAY_LABEL);
    return;
  }

  // 2. Configure the Motor Driver relay control GPIO pin with its flags
  err = gpio_pin_configure(driver_relay, DRIVER_RELAY_PIN, DRIVER_RELAY_FLAGS);
  if (err != 0) {
    printk("Error %d in configuring motor driver relay control GPIO device %s pin %d\n",
        err, DRIVER_RELAY_LABEL, DRIVER_RELAY_PIN);
    return;
  }
}

/*
 * brief: This function turns ON or OFF Motor driver power through relay
    control based on called argument command.
 */
int8_t motor_driver_power_relay_control(bool on) {
  // Set the motor driver relay GPIO
  gpio_pin_set(driver_relay, DRIVER_RELAY_PIN, on);
}

/*
 * brief: Stop the motor as per the provided parameters, the motor_param must be valid
 */

int8_t Stop_Motor(struct motor_params_t *motor_params, bool dir, bool enable_disable, bool true_PWM) {
  int8_t ret;
  printk("\n ===> Stop_Motor called");
  reset_accelation_values();
  ret = rotateMotor(motor_params, PERCENTAGE_ToStop_Motor, dir, enable_disable, true_PWM);
  return ret;
}

#ifdef ADO_RAYTAC_P2
// Added by samuel
void enable_stepper_sleep() {
  const struct device *dev;
  int8_t err;
  // Motor direction GPIO pin Initialisations
  // 1. Initialise the Stepper driver sleep pin GPIO device
  dev = device_get_binding(STPRS_SLP_LABEL);
  if (dev == NULL) {
    printk("Couldn't find motor direction GPIO device%s\n", STPRS_SLP_LABEL);
    return;
  }

  // 2. Configure the Motor Direction GPIO pin with its flags
  err = gpio_pin_configure(dev, STPRS_SLP_PIN, STPRS_SLP_FLAGS);
  if (err != 0) {
    printk("Error %d in configuring motor direction GPIO device %s pin %d\n",
        err, STPRS_SLP_LABEL, STPRS_SLP_PIN);
    return;
  }

  // Set the stepper sleep GPIO
  gpio_pin_set(dev, STPRS_SLP_PIN, false);
}

/*
 * brief: Initialise motor PWM, direction and enable pins and set the motor_params
 */
int8_t initMotor(struct motor_params_t *motor_params) {
  int8_t ret = 0;

  // Motor PWM GPIO pin Initialisations
  // 1. Initialise the Motor PWM GPIO Device
  motor_params->pwm = device_get_binding(motor_params->settings.pwm_label);
  if (!motor_params->pwm) {
    printk("Couldn't find motor PWM GPIO device%s\n", motor_params->settings.pwm_label);
    return -2;
  }

  // 2. Set the PWM cycle period
  // Check the motor type first
  if (motor_params->type != CLAMP_MOTOR) {
    pwm_pin_set_usec(motor_params->pwm, motor_params->settings.pwm_channel,
        motor_params->settings.pwm_period, motor_params->settings.pwm_period,
        motor_params->settings.pwm_flags);
  } else {
    pwm_pin_set_usec(motor_params->pwm, motor_params->settings.pwm_channel,
        motor_params->settings.pwm_period, motor_params->settings.pwm_period / 2U,
        motor_params->settings.pwm_flags);
  }

  // Motor direction GPIO pin Initialisations
  // 1. Initialise the Motor Direction GPIO device
  motor_params->direction = device_get_binding(motor_params->settings.dir_label);
  if (motor_params->direction == NULL) {
    printk("Couldn't find motor direction GPIO device%s\n",
        motor_params->settings.dir_label);
    return -1;
  }

  // 2. Configure the Motor Direction GPIO pin with its flags
  ret = gpio_pin_configure(motor_params->direction, motor_params->settings.dir_pin,
      motor_params->settings.dir_flags);
  if (ret != 0) {
    printk("Error %d in configuring motor direction GPIO device %s pin %d\n",
        ret, motor_params->settings.dir_label, motor_params->settings.dir_pin);
    return ret;
  }

  printk("Set up Motor Direction at %s pin %d\n",
      motor_params->settings.dir_label, motor_params->settings.dir_pin);

  // Motor Enable GPIO Settings
  // 1. Initialise the Motor Enable GPIO device
  motor_params->enable = device_get_binding(motor_params->settings.en_label);
  if (motor_params->enable == NULL) {
    printk("Couldn't find motor enable GPIO device%s\n", motor_params->settings.en_label);
    return -1;
  }

  // 2. Configure the Motor Enable GPIO pin with its flags
  ret = gpio_pin_configure(motor_params->enable, motor_params->settings.en_pin,
      motor_params->settings.en_flags);
  if (ret != 0) {
    printk("Error %d in configuring motor enable GPIO device %s pin %d\n",
        ret, motor_params->settings.en_label, motor_params->settings.en_pin);
    return ret;
  }

  printk("Set up Motor enable at %s pin %d\n", motor_params->settings.en_label,
      motor_params->settings.en_pin);
  return ret;
}

/*
 * brief: rotate the motor as per the provided parameters, the motor_param must be valid
 */

int8_t rotateMotor(struct motor_params_t *motor_params, uint8_t pwm_percent,
    bool dir, bool enable, bool true_pwm) {
  int8_t ret = 0; // temp

  // Input validations
  if (motor_params->direction == NULL && motor_params->pwm == NULL) {
    printk("Unable to get motor GPIO device instances\n");
    return -1;
  }

  // Limit PWM percent to 100%
  if (pwm_percent > MAX_OPR_MOTOR_SPEED_PERCENT) {
    pwm_percent = MAX_OPR_MOTOR_SPEED_PERCENT;
  }

  // TODO   // uncommented after speed testing.
  //  Limit PWM percent to not fall beyond MIN_OPR_MOTOR_SPEED_PERCENT in operating conditions
  if (!true_pwm) {
    if (pwm_percent < MIN_OPR_MOTOR_SPEED_PERCENT)
      pwm_percent = MIN_OPR_MOTOR_SPEED_PERCENT;
  }
  // Set the motor state
  // since the DC door motor enable pin is set to active low,
  // we are tasking opposite state to depict motor running
  motor_params->isRunning = !enable;
  if (motor_params->type == CLAMP_MOTOR) {
    motor_params->isRunning = enable;
    enable = !enable;
    dir = !dir;
  }

  // Toggle the PWM duty cycle percent to ON timings i.e. 70% Pwm eq to 30% ON time
  pwm_percent = 100U - pwm_percent;

  // Set the Motor Enable GPIO
  gpio_pin_set(motor_params->enable, motor_params->settings.en_pin, enable);

  // Set the Motor direction GPIO
  gpio_pin_set(motor_params->direction, motor_params->settings.dir_pin, dir);

  // Set the PWM Period
  // TODO: Add the functionality of turning off PWM if motor is disabled
  if (motor_params->type != CLAMP_MOTOR) {
    ret = pwm_pin_set_usec(motor_params->pwm,
        motor_params->settings.pwm_channel,
        motor_params->settings.pwm_period,
        motor_params->settings.pwm_period * pwm_percent / 100U,
        motor_params->settings.pwm_flags);
    if (ret != 0) {
      printk("Error %d: failed to set pulse width\n", ret);
    }
  }
  return ret;
}
#endif

#ifdef ADO_RAYTAC_P2_1
/*
 * brief: Initialise motor PWM, direction and enable pins and set the motor_params
 */
int8_t initMotor(struct motor_params_t *motor_params) {
  int8_t ret = 0;

  // 2. Set the PWM cycle period
  // Check the motor type first
  if (motor_params->type != CLAMP_MOTOR) {
    // Motor PWM GPIO pin Initialisations
    // 1. Initialise the Motor PWM GPIO Device
    motor_params->pwm = device_get_binding(motor_params->settings.pwm_label);
  
    if (!motor_params->pwm) {
      printk("Couldn't find motor dir GPIO device%s\n", motor_params->settings.pwm_label);
      return -2;
    }

    //// Motor PWM GPIO pin Initialisations
    //// 1. Initialise the Motor PWM GPIO Device
    // motor_params->direction = device_get_binding(motor_params->settings.dir_label);
    // if(!motor_params->direction)
    //{
    //   printk("Couldn't find motor dir GPIO device%s\n", motor_params->settings.dir_label);
    //   return -2;
    // }

    pwm_pin_set_usec(motor_params->pwm, motor_params->settings.pwm_channel,
        motor_params->settings.pwm_period, motor_params->settings.pwm_period / 2U,
        motor_params->settings.pwm_flags);
 
    /* pwm_pin_set_usec(motor_params->direction, motor_params->settings.dir_pin,
        motor_params->settings.pwm_period, motor_params->settings.pwm_period,
        motor_params->settings.dir_flags); */

    // Motor direction GPIO pin Initialisations
    // 1. Initialise the Motor Direction GPIO device
    motor_params->direction = device_get_binding(motor_params->settings.dir_label);

    if (motor_params->direction == NULL) {
      printk("Couldn't find motor direction GPIO device%s\n",
          motor_params->settings.dir_label);
      return -1;
    }

    // 2. Configure the Motor Direction GPIO pin with its flags
    ret = gpio_pin_configure(motor_params->direction, motor_params->settings.dir_pin,
        motor_params->settings.dir_flags);

    if (ret != 0) {
      printk("Error %d in configuring motor direction GPIO device %s pin %d\n",
          ret, motor_params->settings.dir_label, motor_params->settings.dir_pin);
      return ret;
    }

    printk("Set up Motor Direction at %s pin %d\n",
        motor_params->settings.dir_label, motor_params->settings.dir_pin);

    gpio_pin_set(motor_params->direction, motor_params->settings.dir_pin, GPIO_ACTIVE_HIGH);
    // Motor Enable GPIO Settings
    // 1. Initialise the Motor Enable GPIO device
    motor_params->enable = device_get_binding(motor_params->settings.en_label);
    if (motor_params->enable == NULL) {
      printk("Couldn't find motor enable GPIO device%s\n", motor_params->settings.en_label);
      return -1;
    }

    // 2. Configure the Motor Enable GPIO pin with its flags
    ret = gpio_pin_configure(motor_params->enable, motor_params->settings.en_pin,
        motor_params->settings.en_flags);
    if (ret != 0) {
      printk("Error %d in configuring motor enable GPIO device %s pin %d\n",
          ret, motor_params->settings.en_label, motor_params->settings.en_pin);
      return ret;
    }

    printk("Set up Motor enable at %s pin %d\n", motor_params->settings.en_label,
        motor_params->settings.en_pin);
    gpio_pin_set(motor_params->enable, motor_params->settings.en_pin, GPIO_ACTIVE_HIGH);

    Speed(motor_params->start_RPM, motor_params->end_RPM, motor_params->gearbox_ratio,
        motor_params->micro_steps, motor_params->step_angle, motor_params->bw_start_RPM,
        motor_params->bw_end_RPM);
    // Speed(motor_params->start_RPM, motor_params->end_RPM);
  } else {
    // Motor PWM GPIO pin Initialisations
    // 1. Initialise the Motor PWM GPIO Device
    motor_params->pwm = device_get_binding(motor_params->settings.pwm_label);
    if (!motor_params->pwm) {
      printk("Couldn't find motor PWM GPIO device%s\n", motor_params->settings.pwm_label);
      return -2;
    }

    pwm_pin_set_usec(motor_params->pwm, motor_params->settings.pwm_channel,
        motor_params->settings.pwm_period, motor_params->settings.pwm_period / 2U,
        motor_params->settings.pwm_flags);

    // Motor direction GPIO pin Initialisations
    // 1. Initialise the Motor Direction GPIO device
    motor_params->direction = device_get_binding(motor_params->settings.dir_label);

    if (motor_params->direction == NULL) {
      printk("Couldn't find motor direction GPIO device%s\n", motor_params->settings.dir_label);
      return -1;
    }

    // 2. Configure the Motor Direction GPIO pin with its flags
    ret = gpio_pin_configure(motor_params->direction, motor_params->settings.dir_pin,
        motor_params->settings.dir_flags);

    if (ret != 0) {
      printk("Error %d in configuring motor direction GPIO device %s pin %d\n",
          ret, motor_params->settings.dir_label, motor_params->settings.dir_pin);
      return ret;
    }

    printk("Set up Motor Direction at %s pin %d\n", motor_params->settings.dir_label,
        motor_params->settings.dir_pin);

    // Motor Enable GPIO Settings
    // 1. Initialise the Motor Enable GPIO device
    motor_params->enable = device_get_binding(motor_params->settings.en_label);

    if (motor_params->enable == NULL) {
      printk("Couldn't find motor enable GPIO device%s\n", motor_params->settings.en_label);
      return -1;
    }

    // 2. Configure the Motor Enable GPIO pin with its flags
    ret = gpio_pin_configure(motor_params->enable, motor_params->settings.en_pin,
        motor_params->settings.en_flags);

    if (ret != 0) {
      printk("Error %d in configuring motor enable GPIO device %s pin %d\n",
          ret, motor_params->settings.en_label, motor_params->settings.en_pin);
      return ret;
    }
    printk("Set up Motor enable at %s pin %d\n", motor_params->settings.en_label,
        motor_params->settings.en_pin);
  }
  return ret;
}

/*
 * brief: Rotate or Stop the motor as per the provided parameters, the motor_param must be valid
 */
int8_t rotateMotor(struct motor_params_t *motor_params, uint8_t pwm_percent, bool dir, bool enable, bool true_pwm) {
  int32_t ret = 0, u_sec = 0, cnt = 0; // temp

  float Cur_IMU = 0, Prev_IMU = 0, temp = 0;
  bool stag_flag = true; // firstime = true,
  uint8_t staginate_count = 0;

  //// Input validations
  //if (motor_params->direction == NULL && motor_params->pwm == NULL) {
  //   printk("Unable to get motor GPIO device instances\n");
  //   return -1;
  //}

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
  // motor_params->isRunning = !enable;

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
    if (motor_params->isRunning) {
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

        while (Gear_motor_params.current_speed_percentage < 100) { // 11825 cnt <= 13325
          u_sec = fw_accrelation_interval();

          printf("\nforward speed = %d direction = %d msec = %f enable = %d\n",pwm_percent,dir,u_sec,enable);  //Uncommented by Uttam, just for test
          ret = pwm_pin_set_usec(motor_params->pwm, motor_params->settings.pwm_channel,
              u_sec, u_sec / 2U, motor_params->settings.pwm_flags);

          if (ret != 0) {
            printk("Error %d: failed to set pulse width\n", ret);
          }
          k_usleep(u_sec);
          // u_sec = fw_accrelation_interval();
          // printf("\n==>prev_u_sec = %f u_sec=%f ",prev_u_sec,u_sec);
          // cnt++;
        }
      } else {
        ret = gpio_pin_set(motor_params->direction, motor_params->settings.dir_pin, GPIO_ACTIVE_LOW);
        if (ret != 0) {
          printk("Error %d: failed to set  direction\n", ret);
        }

        while (Gear_motor_params.bw_current_speed_percentage < 100) { // 9000 cnt <= 13325
          u_sec = bw_accrelation_interval();
          printk("\nbackward speed = %d direction = %d msec = %d enable = %d\n", pwm_percent, dir, u_sec, enable); //Uncommented by Uttam, just for test
          ret = pwm_pin_set_usec(motor_params->pwm, motor_params->settings.pwm_channel,
              u_sec, u_sec / 2U, motor_params->settings.pwm_flags);
          if (ret != 0) {
            printk("Error %d: failed to set pulse width\n", ret);
          }

          ////check staginate start

          // Cur_IMU=G_IMU;
          // temp=fabs(Prev_IMU - Cur_IMU);
          // if(G_IMU>0) {
          //   printf("\n temp = %f ",G_IMU);
          // }
          // if(temp > 0 && temp<= 1.0f) {
          //   staginate_count++;
          //   printf("\n ===========Accelerate staginate cnt %d\n ",staginate_count );
          // }

          // if(staginate_count >= 3) {
          //   staginate_count=0;
          //   Gear_motor_params.current_interval=u_sec;
          //   return 200;
          // }
          // Prev_IMU=Cur_IMU; //check staginate end
          k_usleep(u_sec);
          // u_sec = bw_accrelation_interval();
          // printf("\n==>prev_u_sec = %f u_sec=%f ",prev_u_sec,u_sec);
          // cnt++;
        }
      }
    }
    printk("\n============>enable = %d isRunning = %d\n", enable, motor_params->isRunning);
  }
  return ret;
}

#endif

/*
 * brief: Wrapper function to initialise all motors at once
 */
int8_t initAllMotors(void) {
  int8_t err = 0;

#ifdef ADO_RAYTAC_P2
  // initialize Stepper driver sleep pin.
  enable_stepper_sleep();
#endif
  // initialize Motor driver relay control pin.
  init_motor_driver_relay_control();

  // initialise DC motor
  err = initMotor(&Gear_motor_params);
  if (err) {
    printk("Failed to initialise DC motor (err %d)\n", err);
    return err;
  } else {
    // Stop the motor first
    Stop_Motor(&Gear_motor_params, DRIVE_MOTOR_forward, DRIVE_MOTOR_disable, Disable_TruePWM);
  }
  printk("after stop motor function\n");
  // initialise horizontal clamp motor
  err = initMotor(&hz_stepper_motor_params);
  if (err) {
    printk("Failed to initialise Horizontal Clamp motor (err %d)\n", err);
    return err;
  } else {
    // Stop the motor first
    Stop_Motor(&hz_stepper_motor_params, CLAMP_MOTOR_unclamp, CLAMP_MOTOR_disable, Disable_TruePWM);
  }

  // initialise vertical clamp motor
  err = initMotor(&vr_stepper_motor_params);
  if (err) {
    printk("Failed to initialise Vertical Clamp motor (err %d)\n", err);
    return err;
  } else {
    // Stop the motor first
    Stop_Motor(&vr_stepper_motor_params, CLAMP_MOTOR_unclamp, CLAMP_MOTOR_disable, Disable_TruePWM);
  }
  return err;
}

/*
 * brief: Function to Check whether the motor is running presently
 */
bool isMotorRunning(struct motor_params_t *motor_params) {
  return motor_params->isRunning;
}

/*
 * brief: Function to Check if motor is running presently, and stop it.
 */
void stopAllMotors(void) {
  if (isMotorRunning(&Gear_motor_params)) {
    // Stop the motor
    Stop_Motor(&Gear_motor_params, DRIVE_MOTOR_backward, DRIVE_MOTOR_disable, Disable_TruePWM);
  }

  if (isMotorRunning(&hz_stepper_motor_params)) {
    // Stop the motor
    Stop_Motor(&hz_stepper_motor_params, CLAMP_MOTOR_clamp, CLAMP_MOTOR_disable, Disable_TruePWM);
  }

  if (isMotorRunning(&vr_stepper_motor_params)) {
    // Stop the motor
    Stop_Motor(&vr_stepper_motor_params, CLAMP_MOTOR_clamp, CLAMP_MOTOR_disable, Disable_TruePWM);
  }
}


void Speed(int Int_RPM, int RPM, float GB, int uStep, float StepAngle, int bw_Int_RPM, int bw_RPM) {
  printk("\n==>");

  Gear_motor_params.start_interval = (1000000 / (((Int_RPM * GB) * 360 * uStep) / (60 * StepAngle)));
  Gear_motor_params.end_interval = (1000000 / (((RPM * GB) * 360 * uStep) / (60 * StepAngle)));

  Gear_motor_params.bw_start_interval = (1000000 / (((bw_Int_RPM * GB) * 360 * uStep) / (60 * StepAngle)));
  Gear_motor_params.bw_end_interval = (1000000 / (((bw_RPM * GB) * 360 * uStep) / (60 * StepAngle)));

  Gear_motor_params.decel_stop_interval = (1000000 / (((Gear_motor_params.fw_decel_stop_rpm * GB) * 360 * uStep) / (60 * StepAngle)));
  Gear_motor_params.bw_decel_stop_interval = (1000000 / (((Gear_motor_params.bw_decel_stop_rpm * GB) * 360 * uStep) / (60 * StepAngle)));

  printk("RPM : ");
  printk("%d", RPM);
  printk("  ");
  printk("GB  : ");
  printk("%f", GB);
  printk("  ");
  printk("uStep : ");
  printk("%d", uStep);
  printk("  ");
  printk("Step Size : ");
  printk("%f", StepAngle / uStep);
  printk("  ");
  printk("FW Start Interval : ");
  printk("%d", Gear_motor_params.start_interval);
  printk("  ");
  printk("FW End Interval : ");
  printk("%d\n", Gear_motor_params.end_interval);

  printk("BW Start Interval : ");
  printk("%d", Gear_motor_params.bw_start_interval);
  printk("  ");
  printk("BW End Interval : ");
  printk("%d\n", Gear_motor_params.bw_end_interval);

  printk("FW decel_stop_interval: ");
  printk("%d", Gear_motor_params.decel_stop_interval);
  printk("  ");
  printk("BW decel_stop_interval : ");
  printk("%d\n", Gear_motor_params.bw_decel_stop_interval);
  reset_accelation_values();
}