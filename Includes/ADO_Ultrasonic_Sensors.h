#ifndef CH201_STR_H_
#define CH201_STR_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>


//extern float feet;

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

//Enable Ultrasonic Sensor by enabling this macro.
//If this macro is disable then Ultrasonic sensor will be disable.
//#define CONFIG_USS_I2C1_ENABLE

/* chirp_data_t - Structure to hold measurement data for one sensor
 *   This structure is used to hold the data from one measurement cycle from 
 *   a sensor.  The data values include the measured range, the ultrasonic 
 *   signal amplitude, the number of valid samples (I/Q data pairs) in the 
 *   measurement, and (optionally) the raw I/Q data from the measurement.
 *
 *  The format of this data structure is specific to this application, so 
 *  you may change it as desired.
 *
 *  A "chirp_data[]" array of these structures, one for each possible sensor, 
 *  is declared in the main.c file.  The sensor's device number is 
 *  used to index the array.
 */
struct chirp_data_t{
	uint32_t		range;							// from ch_get_range()
	uint16_t		amplitude;						// from ch_get_amplitude()
	uint16_t		num_samples;					// from ch_get_num_samples()
#ifdef READ_AMPLITUDE_DATA
	uint16_t		amp_data[DATA_MAX_NUM_SAMPLES];	
												// from ch_get_amplitude_data()
#endif
#ifdef READ_IQ_DATA
	ch_iq_sample_t	iq_data[IQ_DATA_MAX_NUM_SAMPLES];	
												// from ch_get_iq_data()
#endif
};

typedef struct chirp_data_t chirp_data_t;

/* CH101_USS_INIT() - 
 *
 * This function contains the initialization sequence for the application
 * on the board, including system hardware initialization, sensor discovery
 * and configuration, callback routine registration, and timer setup.  After 
 * the initialization sequence completes, this routine enters an infinite 
 * loop that will run for the remainder of the application execution.
 */

void CH101_USS_INIT(void);


int8_t Ultrasonic_arrary_data_read(struct chirp_data_t * chirp_data);
//uint32_t CH101_USS_DATA_READ(void);
//uint32_t low_passFilter(uint32_t range);
#endif