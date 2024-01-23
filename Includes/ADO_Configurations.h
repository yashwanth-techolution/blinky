#ifndef ADO_CONFIGURATIONS_H
#define ADO_CONFIGURATIONS_H

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>


/* Includes */
#include <stdio.h>
#include "soniclib.h"			// Chirp SonicLib sensor API definitions
#include "chirp_board_config.h"	// required header with basic device counts etc.
#include "app_config.h"
#include "app_version.h"
#include "chirp_bsp.h"			// board support package function definitions
#include "chirp_smartsonic.h"
#include "ultrasound_display_config_info.h"
#include "Includes/GPIO_Expander.h"

/* Bit flags used in main loop to check for completion of sensor I/O.  */
//#define DATA_READY_FLAG		(1 << 0)
//#define IQ_READY_FLAG		(1 << 1)
//#define NO_TARGET_VALUE  200


//#define PCB_1_CH101   BOARD_2
//#define PCB_2_CH201   BOARD_1

#define CH201 2
#define CH101 1

// CHx01 Output Data Rate(ODR) setting. 
// ODR = (1000/MEASUREMENT_INTERVAL_MS)    Ex: (1000/100) = 10Hz
#define	MEASUREMENT_INTERVAL_MS		50		// 100ms interval = 10Hz sampling

// Sensor 1
#define SENSOR_1_TYPE            CH101
#define SENSOR_1_FIRMWARE        ch101_gpr_init
//#define SENSOR_1_FIRMWARE        ch201_gprmt_init
//#define SENSOR_1_FIRMWARE        ch201_gprstr_init
#define SENSOR_1_RANGE_MM        750

// Sensor 2
#define SENSOR_2_TYPE            CH201
//#define SENSOR_2_FIRMWARE        ch101_gpr_init
//#define SENSOR_2_FIRMWARE        ch201_gprmt_init
#define SENSOR_2_FIRMWARE        ch201_gprstr_init
#define SENSOR_2_RANGE_MM        3000

// Sensor 3
#define SENSOR_3_TYPE            CH201
#define SENSOR_3_FIRMWARE        ch201_gprstr_init
#define SENSOR_3_RANGE_MM        3000

// Sensor 4
#define SENSOR_4_TYPE            NULL
#define SENSOR_4_FIRMWARE        ch201_gprmt_init
#define SENSOR_4_RANGE_MM        0

#define CHIRP_SENSOR_TYPE        {SENSOR_1_TYPE,     SENSOR_2_TYPE,     SENSOR_3_TYPE,     SENSOR_4_TYPE}
#define CHIRP_SENSOR_FIRMWARE    {SENSOR_1_FIRMWARE, SENSOR_2_FIRMWARE, SENSOR_3_FIRMWARE, SENSOR_4_FIRMWARE}
#define CHIRP_SENSOR_MAX_RANGE   {SENSOR_1_RANGE_MM, SENSOR_2_RANGE_MM, SENSOR_3_RANGE_MM, SENSOR_4_RANGE_MM}



#endif //ADO_CONFIGURATIONS_H