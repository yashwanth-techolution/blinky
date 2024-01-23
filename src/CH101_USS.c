//#include <stdio.h>
//#include <math.h>

//#include "Includes/CH101_USS.h"


///* chirp_data_t - Structure to hold measurement data for one sensor
// *   This structure is used to hold the data from one measurement cycle from 
// *   a sensor.  The data values include the measured range, the ultrasonic 
// *   signal amplitude, the number of valid samples (I/Q data pairs) in the 
// *   measurement, and (optionally) the full amplitude data and/or raw I/Q data 
// *   from the measurement.
// *
// *  The format of this data structure is specific to this application, so 
// *  you may change it as desired.
// *
// *  A "chirp_data[]" array of these structures, one for each possible sensor, 
// *  is declared in the main.c file.  The sensor's device number is 
// *  used to index the array.
// */

//typedef struct {
//	uint32_t		range;						// from ch_get_range()
//	uint16_t		amplitude;					// from ch_get_amplitude()
//	uint16_t		num_samples;				// from ch_get_num_samples()
//#ifdef READ_AMPLITUDE_DATA
//	uint16_t		amp_data[DATA_MAX_NUM_SAMPLES];	
//												// from ch_get_amplitude_data()
//#endif
//#ifdef READ_IQ_DATA
//	ch_iq_sample_t	iq_data[DATA_MAX_NUM_SAMPLES];	
//												// from ch_get_iq_data()
//#endif
//} chirp_data_t;

////uint32_t present_range = 0;   // added by samuel
////uint32_t prev_range = 1000;   // added by samuel


///* added By Samuel */
///* Array of structs to hold measurement data, one for each possible device */
//chirp_data_t	chirp_data;		

//float         multification_factor;                  // added by samuel for matching physical distance.

///* Array of ch_dev_t device descriptors, one for each possible device */
//ch_dev_t	chirp_devices;	

///* Configuration structure for group of sensors */
//ch_group_t 	chirp_group;							

///* Detection level settings - for CH201 sensors only
// *   Each threshold entry includes the starting sample number & threshold level.
// */
//ch_thresholds_t chirp_ch201_thresholds = {
//	.threshold = {
//		{0, 	5000},		/* threshold 0 */
//		{26,	2000},		/* threshold 1 */
//		{39,	800},		/* threshold 2 */
//		{56,	400},		/* threshold 3 */
//		{79,	250},		/* threshold 4 */
//		{89,	175}		/* threshold 5 */
//	}
//};


///* Task flag word
// *   This variable contains the DATA_READY_FLAG and IQ_READY_FLAG bit flags 
// *   that are set in I/O processing routines.  The flags are checked in the 
// *   main() loop and, if set, will cause an appropriate handler function to 
// *   be called to process sensor data.  
// */
//volatile uint32_t taskflags = 0;

//static uint32_t active_devices;
//static uint32_t data_ready_devices;

///* Number of connected sensors */
//static uint8_t	num_connected_sensors = 0;

///* Number of sensors that use h/w triggering to start measurement */
//static uint8_t	num_triggered_devices = 0;

//#if (defined(READ_IQ_DATA) && defined(READ_IQ_NONBLOCKING))
///* Count of non-blocking I/Q reads queued */
//static uint8_t	num_io_queued = 0;
//#endif


///* Forward declarations */
//static void    sensor_int_callback(ch_group_t *grp_ptr, uint8_t dev_num);
//static void    io_complete_callback(ch_group_t *grp_ptr);
//static uint8_t handle_data_ready(ch_group_t *grp_ptr);

//#ifdef READ_IQ_DATA
//static uint8_t display_iq_data(ch_dev_t *dev_ptr);
//#ifdef READ_IQ_NONBLOCKING
//static uint8_t handle_iq_data_done(ch_group_t *grp_ptr);
//#endif
//#endif


//ch_group_t	*grp_ptr = &chirp_group;

        
//uint8_t CH101_INIT(void)
//{
	
//	ch_config_t 	read_config_data;   // added by samuel for multification scale factor

//        uint8_t chirp_error = 0;
//	uint8_t num_ports;  // only one sensor interfacing. by Samuel
//	uint8_t dev_num;

//	chbsp_board_init(grp_ptr);   // Cmnd by samuel

//	printk("\n\nHello Chirp! - Chirp SonicLib Example Application\n");
//	//printf("    Compile time:  %s %s\n", __DATE__, __TIME__);
//	printf("    Version: %u.%u.%u", APP_VERSION_MAJOR, APP_VERSION_MINOR, APP_VERSION_REV);
//	printf("    SonicLib version: %u.%u.%u\n", SONICLIB_VER_MAJOR, SONICLIB_VER_MINOR, SONICLIB_VER_REV);
//	printf("\n");


//	/* Get the number of (possible) sensor devices on the board
//	 *   Set by the BSP during chbsp_board_init() 
//	 */
//	num_ports = ch_get_num_ports(grp_ptr);  // cmnd samuel 
//        printk("Number of ports = %d \n", num_ports);

//	printk("Initializing sensor(s)... ");

//        // added by Samuel
//        ch_dev_t *dev_ptr = &chirp_devices;	// init struct in array
//        chirp_error = ch_init(dev_ptr, grp_ptr, 0, CHIRP_SENSOR_FW_INIT_FUNC);

//	if (chirp_error == 0) {
//		printk("starting group... ");
//		chirp_error = ch_group_start(grp_ptr);
//	}

//	if (chirp_error == 0) {
//		printk("OK\n");
//	} else {
//		printk(" FAILED: %d \n", chirp_error);
//	}
//	printf("\n");

//	/* Get and display the initialization results for each connected sensor.
//	 *   This loop checks each device number in the sensor group to determine 
//	 *   if a sensor is actually connected.  If so, it makes a series of 
//	 *   function calls to get different operating values, including the 
//	 *   operating frequency, clock calibration values, and firmware version.
// 	 */
//	printf("Sensor\tType \t   Freq\t\t RTC Cal \tFirmware\n");

//	for (dev_num = 0; dev_num < num_ports; dev_num++) {
//		ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

//		if (ch_sensor_is_connected(dev_ptr)) {
//			printf("%d\tCH%d\t %u Hz\t%u@%ums\t%s\n", dev_num,ch_get_part_number(dev_ptr),(unsigned int) ch_get_frequency(dev_ptr),ch_get_rtc_cal_result(dev_ptr),ch_get_rtc_cal_pulselength(dev_ptr),ch_get_fw_version_string(dev_ptr));
//		}
//	}
//	printf("\n");

//	/* Configure each sensor with its operating parameters 
//	 *   Initialize a ch_config_t structure with values defined in the
//	 *   app_config.h header file, then write the configuration to the 
//	 *   sensor using ch_set_config().
//	 */
//	printf ("Configuring sensor(s)...\n");

//        /* Modified and added by Samuel */
//        ch_config_t dev_config;

//        if (ch_sensor_is_connected(dev_ptr)) 
//        {
//	  /* Select sensor mode 
//	   *   All connected sensors are placed in hardware triggered mode.
// 	   *   The first connected (lowest numbered) sensor will transmit and 
//	   *   receive, all others will only receive.
// 	  */

//	    dev_config.mode = CH_MODE_FREERUN;   // cmd by Samuel

//	  if (dev_config.mode != CH_MODE_FREERUN) 
//          {	// unless free-running
//	    num_triggered_devices++;				// will be triggered
//	  }

//	  /* Init config structure with values from app_config.h */
//	  dev_config.max_range       = CHIRP_SENSOR_MAX_RANGE_MM;
//	  dev_config.static_range    = CHIRP_SENSOR_STATIC_RANGE;

//	  /* If sensor will be free-running, set internal sample interval */
//	  if (dev_config.mode == CH_MODE_FREERUN) 
//          {
//	    dev_config.sample_interval = MEASUREMENT_INTERVAL_MS;
//	  } 
//          else 
//          {
//	    dev_config.sample_interval = 0;
//	  }

//	  /* Set detection thresholds (CH201 only) */
//	  if (ch_get_part_number(dev_ptr) == CH201_PART_NUMBER) 
//          {
//	    /* Set pointer to struct containing detection thresholds */
//	    dev_config.thresh_ptr = &chirp_ch201_thresholds;	
//	  }
//          else 
//          {
//	    dev_config.thresh_ptr = 0;							
//	  }

//          /* Apply sensor configuration */
//	  chirp_error = ch_set_config(dev_ptr, &dev_config);

//	  /* Enable sensor interrupt if using free-running mode 
//	   *   Note that interrupt is automatically enabled if using 
//	   *   triggered modes.
//	  */
//	  if ((!chirp_error) && (dev_config.mode == CH_MODE_FREERUN)) 
//          {
//	    chbsp_set_io_dir_in(dev_ptr);
//	    chbsp_io_interrupt_enable(dev_ptr);
//	  }

//	  /* Read back and display config settings */
//	  if (!chirp_error) 
//          {
//	    ultrasound_display_config_info(dev_ptr);

//            /* added by samuel for multification factor
//            */ 
//	    /* Read configuration values for the device into ch_config_t structure */
//	    chirp_error = ch_get_config(dev_ptr, &read_config_data);
//            multification_factor = (float)dev_config.max_range/(float)read_config_data.max_range;
//            /* Display sensor number, mode and max range */
//            printf("max_range = %dmm,   max_range set = %dmm, multifactor = %.4f \n", dev_config.max_range, read_config_data.max_range, multification_factor);
    
//            if((multification_factor <= (float)1.1) || (multification_factor >= (float)0.9))
//            {
//              printk("-------------------------------------------------\n");
//              return 1;
//            }
//          }
//          else 
//          {
//	    printf("Device %d: Error during ch_set_config()\n", dev_num);
//	  }
//	}

//	printf("\n");

//	/* Enable receive sensor pre-triggering, if specified */
//	ch_set_rx_pretrigger(grp_ptr, RX_PRETRIGGER_ENABLE);       // Cmd by Samuel

//	/* Initialize the periodic timer we'll use to trigger the measurements.
// 	 *   This function initializes a timer that will interrupt every time it 
//	 *   expires, after the specified measurement interval.  The function also 
//	 *   registers a callback function that will be called from the timer 
//	 *   handler when the interrupt occurs.  The callback function will be 
//	 *   used to trigger a measurement cycle on the group of sensors.
//	 */
//	if (num_triggered_devices > 0) 
//        {
//		printf("Initializing sample timer for %dms interval... ",MEASUREMENT_INTERVAL_MS);

//		chbsp_periodic_timer_init(MEASUREMENT_INTERVAL_MS,periodic_timer_callback);

//		/* Enable interrupt and start timer to trigger sensor sampling */
//		chbsp_periodic_timer_irq_enable();
//		chbsp_periodic_timer_start();
//                printf("OK\n");
//	}
//     return 0;
//}

//void CH101_USS_INIT(void)
//{
//  int i = 0;
//  while(CH101_INIT() == 0)
//  {
//    k_msleep(10);
//    i++;
//    if(i >= 5)
//    {
//      printk("-----------Failed to initialize Ultrasonic Sensor----------");
//      break;
//    }
//  }
//}


//uint32_t pev_range=0;
//uint32_t low_passFilter(uint32_t range){
//  if(pev_range==0){
//    pev_range = range;
//  }else{
//    pev_range = pev_range*.50 + range*.50;
//  }
//  return pev_range;
//}

//uint32_t CH101_USS_DATA_READ(void)
//{
//      uint8_t 	dev_num = 0;
//	int 		num_samples = 0;
//	uint8_t 	ret_val = 0;
//        float present_range = 0;   // added by samuel

//	ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
        
//	if (ch_sensor_is_connected(dev_ptr)) 
//        {	
//		if (ch_get_mode(dev_ptr) == CH_MODE_TRIGGERED_RX_ONLY)
//                {
//		  chirp_data.range = ch_get_range(dev_ptr, CH_RANGE_DIRECT);
//		} 
//                else 
//                {
//		  chirp_data.range = ch_get_range(dev_ptr, CH_RANGE_ECHO_ONE_WAY);  // cmd by samuel
//		}

//		if (chirp_data.range == CH_NO_TARGET)
//                {
//		  /* No target object was detected - no range value */

//		  chirp_data.amplitude = 0;  /* no updated amplitude */

//                  chirp_data.range = NO_TARGET_VALUE;
//                  present_range = chirp_data.range;
//		}
//                else 
//                {
//		  //chirp_data.amplitude = ch_get_amplitude(dev_ptr);
//                  /* added by Samuel */
//                    present_range = (float)chirp_data.range+((float)chirp_data.range*0.3);
//                    present_range = present_range/32;
//		}
//        }
//	return low_passFilter((uint32_t)present_range);
//}






////uint16_t CH101_USS_DATA_READ(void)
////{
////    uint8_t ret_val = 0;
////    //k_msleep(100U);
////    ret_val = handle_data_ready(grp_ptr);			// read and display measurement

////#if (defined(READ_IQ_DATA) && defined(READ_IQ_NONBLOCKING))
////		/* Start any pending non-blocking I/Q reads */
////		if (num_io_queued != 0) {
////			ch_io_start_nb(grp_ptr);
////			num_io_queued = 0;
////		}

////		/* Check for non-blocking I/Q read complete */
////		if (taskflags & IQ_READY_FLAG) {

////			/* All non-blocking I/Q readouts have completed */
////			taskflags &= ~IQ_READY_FLAG;		// clear flag
////			handle_iq_data_done(grp_ptr);		// display I/Q data
////		}
////#endif
////return ret_val;
////}

/////*
//// * handle_data_ready() - get and display data from all sensors
//// *
//// * This routine is called from the main() loop after all sensors have 
//// * interrupted. It shows how to read the sensor data once a measurement is 
//// * complete.  This routine always reads out the range and amplitude, and 
//// * optionally will read out the amplitude data or raw I/Q for all samples
//// * in the measurement.
//// *
//// * See the comments in app_config.h for information about the amplitude data
//// * and I/Q readout build options.
//// *
//// */
/////* Modified and added by Samuel */
////static uint8_t handle_data_ready(ch_group_t *grp_ptr) {
////	uint8_t 	dev_num = 0;
////	int 		num_samples = 0;
////	uint8_t 	ret_val = 0;

////	/* Read and display data from each connected sensor 
////	 *   This loop will write the sensor data to this application's "chirp_data"
////	 *   array.  Each sensor has a separate chirp_data_t structure in that 
////	 *   array, so the device number is used as an index.
////	 */

////	ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
        
////	if (ch_sensor_is_connected(dev_ptr)) 
////        {
////		/* Get measurement results from each connected sensor 
////		 *   For sensor in transmit/receive mode, report one-way echo 
////		 *   distance,  For sensor(s) in receive-only mode, report direct 
////		 *   one-way distance from transmitting sensor 
////		 */
			
////		if (ch_get_mode(dev_ptr) == CH_MODE_TRIGGERED_RX_ONLY)
////                {
////		  chirp_data.range = ch_get_range(dev_ptr, CH_RANGE_DIRECT);
////		} 
////                else 
////                {
////		  chirp_data.range = ch_get_range(dev_ptr, CH_RANGE_ECHO_ONE_WAY);  // cmd by samuel
////		}

////		if (chirp_data.range == CH_NO_TARGET)
////                {
////		  /* No target object was detected - no range value */

////		  chirp_data.amplitude = 0;  /* no updated amplitude */
                  
////                  if(prev_range != 0)   // added by samuel
////                  {
////		    printk("Port %d:          no target found        \n", dev_num);
////                  }
////                  prev_range = 0;  // added by samuel
////		}
////                else 
////                {
////		  /* Target object was successfully detected (range available) */

////		  /* Get the new amplitude value - it's only updated if range 
////		   * was successfully measured.  */
////		  chirp_data.amplitude = ch_get_amplitude(dev_ptr);
                  
////		/* Store number of active samples in this measurement */
////		//num_samples = ch_get_num_samples(dev_ptr);
////		//chirp_data.num_samples = num_samples;

////		  //printf("Port %d:  Range: %0.1f mm  Amp: %u  ", dev_num, 
////				//		(float) chirp_data.range/32.0f,
////				//	   	chirp_data.amplitude);
////		  //printf("Port %d:  Range: %0.1f mm  Amp: %u  \n", dev_num, 
////				//		(float) (chirp_data.range/32.0f)+30.0,
////				//	   	chirp_data.amplitude);

////                   /* added by Samuel */
////                  pret_range =  (chirp_data.range/32)+((chirp_data.range/32)*0.4);
////                  if(prev_range != pret_range)
////                  {
////                    printk("Range = %d \t", pret_range);
////                    prev_range = pret_range;
////                  }
////		}  

////		/* Optionally read amplitude values for all samples */
////#ifdef READ_AMPLITUDE_DATA
////			uint16_t 	start_sample = 0;
////			ch_get_amplitude_data(dev_ptr, chirp_data.amp_data, 
////								  start_sample, num_samples, CH_IO_MODE_BLOCK);

////#ifdef OUTPUT_AMPLITUDE_DATA
////			printf("\n");
////			for (uint8_t count = 0; count < num_samples; count++) {

////				printk("%d\n",  chirp_data.amp_data[count]);
////			}
////#endif
////#endif
////   }
////	return ret_val;
////}