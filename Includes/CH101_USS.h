//#ifndef CH101_USS_H_
//#define CH101_USS_H_

//#include <device.h>
//#include <drivers/i2c.h>
//#include <errno.h>
//#include <sys/printk.h>
//#include <zephyr.h>


///* Includes */
//#include <stdio.h>
//#include "soniclib.h"			// Chirp SonicLib sensor API definitions
//#include "chirp_board_config.h"	// required header with basic device counts etc.
//#include "app_config.h"
//#include "app_version.h"
//#include "chirp_bsp.h"			// board support package function definitions
//#include "chirp_smartsonic.h"
//#include "ultrasound_display_config_info.h"

///* Bit flags used in main loop to check for completion of sensor I/O.  */
//#define DATA_READY_FLAG		(1 << 0)
//#define IQ_READY_FLAG		(1 << 1)
//#define NO_TARGET_VALUE  200


///* CH101_USS_INIT() - 
// *
// * This function contains the initialization sequence for the application
// * on the board, including system hardware initialization, sensor discovery
// * and configuration, callback routine registration, and timer setup.  After 
// * the initialization sequence completes, this routine enters an infinite 
// * loop that will run for the remainder of the application execution.
// */

//void CH101_USS_INIT(void);


//uint32_t CH101_USS_DATA_READ(void);
//uint32_t low_passFilter(uint32_t range);
//#endif