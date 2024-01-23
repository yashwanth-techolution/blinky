///*
// * _____________________________________________________________________________
// * Copyright (c) 2020-2021 InvenSense Inc. All rights reserved.
// *
// * This software, related documentation and any modifications thereto 
// * (collectively "Software") is subject to InvenSense and its licensors' 
// * intellectual property rights under U.S. and international copyright
// * and other intellectual property rights laws.
// *
// * InvenSense and its licensors retain all intellectual property and proprietary
// * rights in and to the Software and any use, reproduction, disclosure or 
// * distribution of the Software without an express license agreement from 
// * InvenSense is strictly prohibited.
// *
// * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE 
// * SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
// * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
// * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.  EXCEPT AS OTHERWISE 
// * PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
// * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR 
// * CONSEQUENTIAL DAMAGES, OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF 
// * USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER 
// * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
// * OF THE SOFTWARE.
// * _____________________________________________________________________________
// */

///*! \file app_config.h */
//#ifndef APP_CONFIG_H
//#define APP_CONFIG_H

//#include "soniclib.h"

///*========================= Sensor Firmware Selection ===========================*/

///* Select sensor firmware to use 
// *   The sensor firmware type is specified during the call to ch_init(), by
// *   giving the name (address) of the firmware initialization function that will
// *   be called.  The CHIRP_SENSOR_FW_INIT_FUNC symbol is used to specify the 
// *   init routine for the sensor firmware to be used.
// *
// *   Uncomment ONE of the following lines to use that sensor firmware type.
// *   You must choose a firmware type that is appropriate for the sensor model 
// *   you are using (CH101 or CH201).  
// *
// *   To use a different sensor firmware type than those listed here (for 
// *   example, a new distribution from Chirp), simply define 
// *   CHIRP_SENSOR_FW_INIT_FUNC to equal the name of the init routine for 
// *   the new firmware.
// *
// * 	 Short Range Firmware:
// *   CH101 sensor firmware with "sr" in the name, e.g. ch101_gpr_sr, is 
// *   optimized for short range. The short range firmware has 4 times the 
// *   resolution, but only 1/4 the maximum range.  If you use this option, you 
// *   should redefine the CHIRP_SENSOR_MAX_RANGE_MM symbol, below, to 250mm or 
// *   less.
// */

//	/* CH101 GPR - general purpose rangefinding, standard range */
////#define	 CHIRP_SENSOR_FW_INIT_FUNC	ch101_gpr_init   // Cmd by Samuel
////#define	 CHIRP_SENSOR_FW_INIT_FUNC      ch101_gpr_open_init  // added by Samuel
////#define	 CHIRP_SENSOR_FW_INIT_FUNC	ch101_gpr_sr_open_init   // added by samuel

//	/* CH101 GPR NARROW - general purpose rangefinding, narrow FoV */
// //#define	 CHIRP_SENSOR_FW_INIT_FUNC	ch101_gpr_narrow_init
	
//	/* CH101 GPR SR - general purpose rangefinding, short range */
// #define	 CHIRP_SENSOR_FW_INIT_FUNC	ch101_gpr_sr_init
 
//	/* CH101 GPR SR NARROW - general purpose rangefinding, short range, narrow FoV */ 
//  //#define	 CHIRP_SENSOR_FW_INIT_FUNC	ch101_gpr_sr_narrow_init

//	/* CH201 GPRMT - general purpose rangefinding / multi threshold */
//// #define	 CHIRP_SENSOR_FW_INIT_FUNC	ch201_gprmt_init	


///*=========================== Sensor Configuration ===========================*/

///* Operating mode for the sensor(s)
// *	 These two values define the mode in which the sensor(s) will operate.  
// *
// *	 The CHIRP_FIRST_SENSOR_MODE value specifies the mode for the first sensor
// *	 (lowest numbered) that is present.  If only one sensor is attached, this
// *	 value must be either CH_MODE_TRIGGERED_TX_RX or CH_MODE_FREERUN.
// *
// *	 The CHIRP_OTHER_SENSOR_MODE value specifies the mode for all other sensors
// *	 that are present.
// *
// * 	 For typical Pitch-Catch operation using two or more sensors, set 
// * 	 CHIRP_FIRST_SENSOR_MODE to CH_MODE_TRIGGERED_TX_RX and set 
// * 	 CHIRP_OTHER_SENSOR_MODE to CH_MODE_TRIGGERED_RX_ONLY.
// */
//#define CHIRP_FIRST_SENSOR_MODE		CH_MODE_TRIGGERED_TX_RX
//#define CHIRP_OTHER_SENSOR_MODE		CH_MODE_TRIGGERED_RX_ONLY

///* Maximum detection range for the sensor
// * This value will determine how long the sensor "listens" for an ultrasound 
// * signal.  Note that the maximum possible range will vary depending on sensor 
// * model (CH101 vs. CH201) and sensor firmware type.  If the value specified 
// * here is greater than the maximum possible range, the maximum possible range 
// * will be used.
// */
////#define	       CHIRP_SENSOR_MAX_RANGE_MM		(750)	/* maximum range, in mm */
//#define	        CHIRP_SENSOR_MAX_RANGE_MM		(245)	/* maximum range, in mm */  // added by Samuel

///* Static target rejection range
// * This value specifies if static target rejection (STR) will be used.  If
// * CHIRP_SENSOR_STATIC_RANGE is non-zero, STR will be enabled and will apply
// * to the specified number of samples at the beginning of a measurement.
// */
//#define	CHIRP_SENSOR_STATIC_RANGE		(0)	/* static target rejection sample 
//											   range, in samples (0=disabled) */

///* Receive sensor pre-triggering
// * This value specifieds if receive (rx) sensor pre-triggering will be used.  
// * This setting only applies if more than one sensor is used, and one or more 
// * sensor is operating in CH_MODE_TRIGGERED_RX_ONLY.  
// *
// * Receive pre-triggering improves performance in pitch-catch operation at 
// * short distances, by triggering the receive-only sensor(s) slightly before 
// * the transmitting sensor.  However, this setting will reduce maximum range of 
// * rx-only sensors approximately 200mm, relative to the 
// * CHIRP_SENSOR_MAX_RANGE_MM setting, above.
// *
// * Set RX_PRETRIGGER_ENABLE to non-zero to enable rx pretriggering, or zero 
// * to disable.
// */

//#define RX_PRETRIGGER_ENABLE	 1  // cmd by samuel


///*============================ Application Timing ============================*/

///* Define how often the application will get a new sample from the sensor(s) 
// *   This macro defines the sensor measurement interval, in milliseconds.  
// *
// *   For sensors in triggered mode (CH_MODE_TRIGGERED_TX_RX or 
// *   CH_MODE_TRIGGERED_RX_ONLY), the application will use a periodic timer to 
// *   trigger a sensor measurement each time this period elapses.
// *
// *   For sensors in free-running mode (CH_MODE_FREERUN), the application will
// *   set this period as the sensor's internal sample interval.
// */

//#define	MEASUREMENT_INTERVAL_MS (100)     // cmd by Samuel


///*==================  Application Storage for Sensor Data ====================*/

///* Define how many samples per measurement are expected by this application
// *   The following macro is used to allocate array storage in the "chirp_data_t"
// *   structure, defined in main.c.  That structure contains arrays for 
// *   individual data values (I/Q or amplitude) that describe the raw samples 
// *   within an ultrasound measurement.  
// *
// *   Because a Chirp CH201 sensor has more samples in each measurement than a 
// *   CH101 device, the CH201 sample count is used here by default.  If you are 
// *   ONLY using CH101 devices with this application, you may redefine the 
// *   following symbol to CH101_MAX_NUM_SAMPLES to use less memory.
// */

//#define DATA_MAX_NUM_SAMPLES  CH201_MAX_NUM_SAMPLES		// use CH201 max


///*===============  Build Options for Amplitude Data Handling =================*/

///* The following build options control if and how the full amplitude data for 
// * all internal samples within an ultrasound measurement will be read and 
// * displayed.  This data is separate from the standard range and simple target 
// * amplitude values that are normally output.
// * 
// * Note that reading the full amplitude data is not required for most basic 
// * sensing applications - the reported range value, possibly combined with the 
// * simple target amplitude value, is typically all that is required.  However, 
// * the full set of amplitude values may be read and analyzed for more advanced 
// * sensing or data capture needs.
// *
// * Comment or un-comment the various definitions, as appropriate.
// *
// * Define READ_AMPLITUDE_DATA to enable readout of the amplitude data.
// * Define OUTPUT_AMPLITUDE_DATA to enable output of the amplitude data via the
// * serial port, as ascii values, one per line.
// */

// //#define READ_AMPLITUDE_DATA		/* uncomment to readout amplitude data */

// //#define OUTPUT_AMPLITUDE_DATA	/* uncomment to output data in ascii */


///*==================  Build Options for I/Q Data Handling ====================*/

///* The following build options control if and how the raw I/Q data is read 
// * from the device after each measurement cycle, in addition to the standard 
// * range and amplitude.  
// *
// * Note that reading the I/Q data is not required for most basic sensing 
// * applications - the reported range value is typically all that is required. 
// * However, the full data set may be read and analyzed for more advanced 
// * sensing or data capture needs.
// *
// * Comment or un-comment the various definitions, as appropriate.
// *
// * Define READ_IQ_DATA to enable readout of the I/Q data.
// *
// * By default, this application will read the I/Q data in blocking mode 
// * (i.e. READ_IQ_BLOCKING is defined by default).  The data will be read from 
// * the device and placed in the I/Q data array field in the application's 
// * chirp_data structure.  Because the I/Q data is read in blocking mode, the 
// * call to ch_get_iq_data() will not return until the data has actually 
// * been copied from the device.  
// *
// * If, however, READ_IQ_NONBLOCKING is defined instead, the I/Q data will be 
// * read in non-blocking mode. The ch_get_iq_data() call will return immediately,
// * and a separate callback function will be called to notify the application 
// * when the read operation is complete.
// *
// * Finally, if OUTPUT_IQ_DATA_CSV is defined, the application will write the 
// * I/Q data values out through the serial port in ascii form as comma-separated 
// * numeric value pairs (Q,I).  This can make it easier to take the data 
// * from the application and analyze it in a spreadsheet or other program.
// */

// //#define READ_IQ_DATA			/* uncomment this line to readout I/Q data */

//#define READ_IQ_BLOCKING 		/* use blocking mode when reading I/Q */
//// #define READ_IQ_NONBLOCKING 	/* use non-blocking mode when reading I/Q */

//// #define OUTPUT_IQ_DATA_CSV	/* uncomment to output I/Q data in CSV format */
											

//#endif /* APP_CONFIG_H */



























/*
 * _____________________________________________________________________________
 * Copyright (c) 2020-2021 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto 
 * (collectively "Software") is subject to InvenSense and its licensors' 
 * intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary
 * rights in and to the Software and any use, reproduction, disclosure or 
 * distribution of the Software without an express license agreement from 
 * InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE 
 * SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.  EXCEPT AS OTHERWISE 
 * PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR 
 * CONSEQUENTIAL DAMAGES, OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF 
 * USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER 
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * _____________________________________________________________________________
 */

/*! \file app_config.h */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "soniclib.h"

///*========================= Sensor Firmware Selection ===========================*/

///* Select sensor firmware to use 
// *   The sensor firmware type is specified during the call to ch_init(), by
// *   giving the name (address) of the firmware initialization function that will
// *   be called.  The CHIRP_SENSOR_FW_INIT_FUNC symbol is used to specify the 
// *   init routine for the sensor firmware to be used.
// *
// *   Uncomment ONE of the following lines to use that sensor firmware type.
// *   You must choose a firmware type that is appropriate for the sensor model 
// *   you are using (CH101 or CH201).  
// *
// *   To use a different sensor firmware type than those listed here (for 
// *   example, a new distribution from Chirp), simply define 
// *   CHIRP_SENSOR_FW_INIT_FUNC to equal the name of the init routine for 
// *   the new firmware.
// *
// * 	 Short Range Firmware:
// *   CH101 sensor firmware with "sr" in the name, e.g. ch101_gpr_sr, is 
// *   optimized for short range. The short range firmware has 4 times the 
// *   resolution, but only 1/4 the maximum range.  If you use this option, you 
// *   should redefine the CHIRP_SENSOR_MAX_RANGE_MM symbol, below, to 250mm or 
// *   less.
// */

//	/* CH101 GPR - general purpose rangefinding, standard range */
//#define	 CHIRP_SENSOR_FW_INIT_FUNC_101 	    ch101_gpr_init   // Cmd by Samuel
////#define	 CHIRP_SENSOR_FW_INIT_FUNC_101      ch101_gpr_open_init  // added by Samuel
////#define	 CHIRP_SENSOR_FW_INIT_FUNC_101	    ch101_gpr_sr_open_init   // added by samuel

//	/* CH101 GPR NARROW - general purpose rangefinding, narrow FoV */
// //#define	 CHIRP_SENSOR_FW_INIT_FUNC_101	    ch101_gpr_narrow_init
	
//	/* CH101 GPR SR - general purpose rangefinding, short range */
// //#define	 CHIRP_SENSOR_FW_INIT_FUNC_101	    ch101_gpr_sr_init
 
//	/* CH101 GPR SR NARROW - general purpose rangefinding, short range, narrow FoV */ 
//  //#define	 CHIRP_SENSOR_FW_INIT_FUNC_101	    ch101_gpr_sr_narrow_init

//	/* CH201 GPRMT - general purpose rangefinding / multi threshold */
// #define	 CHIRP_SENSOR_FW_INIT_FUNC_201	    ch201_gprmt_init	


///*=========================== Sensor Configuration ===========================*/

///* Operating mode for the sensor(s)
// *	 These two values define the mode in which the sensor(s) will operate.  
// *
// *	 The CHIRP_FIRST_SENSOR_MODE value specifies the mode for the first sensor
// *	 (lowest numbered) that is present.  If only one sensor is attached, this
// *	 value must be either CH_MODE_TRIGGERED_TX_RX or CH_MODE_FREERUN.
// *
// *	 The CHIRP_OTHER_SENSOR_MODE value specifies the mode for all other sensors
// *	 that are present.
// *
// * 	 For typical Pitch-Catch operation using two or more sensors, set 
// * 	 CHIRP_FIRST_SENSOR_MODE to CH_MODE_TRIGGERED_TX_RX and set 
// * 	 CHIRP_OTHER_SENSOR_MODE to CH_MODE_TRIGGERED_RX_ONLY.
// */
//#define CHIRP_FIRST_SENSOR_MODE		CH_MODE_TRIGGERED_TX_RX
//#define CHIRP_OTHER_SENSOR_MODE		CH_MODE_TRIGGERED_RX_ONLY

///* Maximum detection range for the sensor
// * This value will determine how long the sensor "listens" for an ultrasound 
// * signal.  Note that the maximum possible range will vary depending on sensor 
// * model (CH101 vs. CH201) and sensor firmware type.  If the value specified 
// * here is greater than the maximum possible range, the maximum possible range 
// * will be used.
// */
//#define	        CHIRP_SENSOR_MAX_RANGE_MM_101		(750)	/* maximum range, in mm */
//#define	        CHIRP_SENSOR_MAX_RANGE_MM_201		(3600)	/* maximum range, in mm */  // added by Samuel
////#define	        CHIRP_SENSOR_MAX_RANGE_MM_101		(245)	/* maximum range, in mm */  // added by Samuel

///* Static target rejection range
// * This value specifies if static target rejection (STR) will be used.  If
// * CHIRP_SENSOR_STATIC_RANGE is non-zero, STR will be enabled and will apply
// * to the specified number of samples at the beginning of a measurement.
// */
//#define	CHIRP_SENSOR_STATIC_RANGE		(0)	/* static target rejection sample range, in samples (0=disabled) */

///* Receive sensor pre-triggering
// * This value specifieds if receive (rx) sensor pre-triggering will be used.  
// * This setting only applies if more than one sensor is used, and one or more 
// * sensor is operating in CH_MODE_TRIGGERED_RX_ONLY.  
// *
// * Receive pre-triggering improves performance in pitch-catch operation at 
// * short distances, by triggering the receive-only sensor(s) slightly before 
// * the transmitting sensor.  However, this setting will reduce maximum range of 
// * rx-only sensors approximately 200mm, relative to the 
// * CHIRP_SENSOR_MAX_RANGE_MM setting, above.
// *
// * Set RX_PRETRIGGER_ENABLE to non-zero to enable rx pretriggering, or zero 
// * to disable.
// */

//#define RX_PRETRIGGER_ENABLE	 1  // cmd by samuel


///*============================ Application Timing ============================*/

///* Define how often the application will get a new sample from the sensor(s) 
// *   This macro defines the sensor measurement interval, in milliseconds.  
// *
// *   For sensors in triggered mode (CH_MODE_TRIGGERED_TX_RX or 
// *   CH_MODE_TRIGGERED_RX_ONLY), the application will use a periodic timer to 
// *   trigger a sensor measurement each time this period elapses.
// *
// *   For sensors in free-running mode (CH_MODE_FREERUN), the application will
// *   set this period as the sensor's internal sample interval.
// */

//#define	MEASUREMENT_INTERVAL_MS (100)     // cmd by Samuel


///*==================  Application Storage for Sensor Data ====================*/

///* Define how many samples per measurement are expected by this application
// *   The following macro is used to allocate array storage in the "chirp_data_t"
// *   structure, defined in main.c.  That structure contains arrays for 
// *   individual data values (I/Q or amplitude) that describe the raw samples 
// *   within an ultrasound measurement.  
// *
// *   Because a Chirp CH201 sensor has more samples in each measurement than a 
// *   CH101 device, the CH201 sample count is used here by default.  If you are 
// *   ONLY using CH101 devices with this application, you may redefine the 
// *   following symbol to CH101_MAX_NUM_SAMPLES to use less memory.
// */

//#define DATA_MAX_NUM_SAMPLES  CH201_MAX_NUM_SAMPLES		// use CH201 max


///*===============  Build Options for Amplitude Data Handling =================*/

///* The following build options control if and how the full amplitude data for 
// * all internal samples within an ultrasound measurement will be read and 
// * displayed.  This data is separate from the standard range and simple target 
// * amplitude values that are normally output.
// * 
// * Note that reading the full amplitude data is not required for most basic 
// * sensing applications - the reported range value, possibly combined with the 
// * simple target amplitude value, is typically all that is required.  However, 
// * the full set of amplitude values may be read and analyzed for more advanced 
// * sensing or data capture needs.
// *
// * Comment or un-comment the various definitions, as appropriate.
// *
// * Define READ_AMPLITUDE_DATA to enable readout of the amplitude data.
// * Define OUTPUT_AMPLITUDE_DATA to enable output of the amplitude data via the
// * serial port, as ascii values, one per line.
// */

// //#define READ_AMPLITUDE_DATA		/* uncomment to readout amplitude data */

// //#define OUTPUT_AMPLITUDE_DATA	/* uncomment to output data in ascii */


///*==================  Build Options for I/Q Data Handling ====================*/

///* The following build options control if and how the raw I/Q data is read 
// * from the device after each measurement cycle, in addition to the standard 
// * range and amplitude.  
// *
// * Note that reading the I/Q data is not required for most basic sensing 
// * applications - the reported range value is typically all that is required. 
// * However, the full data set may be read and analyzed for more advanced 
// * sensing or data capture needs.
// *
// * Comment or un-comment the various definitions, as appropriate.
// *
// * Define READ_IQ_DATA to enable readout of the I/Q data.
// *
// * By default, this application will read the I/Q data in blocking mode 
// * (i.e. READ_IQ_BLOCKING is defined by default).  The data will be read from 
// * the device and placed in the I/Q data array field in the application's 
// * chirp_data structure.  Because the I/Q data is read in blocking mode, the 
// * call to ch_get_iq_data() will not return until the data has actually 
// * been copied from the device.  
// *
// * If, however, READ_IQ_NONBLOCKING is defined instead, the I/Q data will be 
// * read in non-blocking mode. The ch_get_iq_data() call will return immediately,
// * and a separate callback function will be called to notify the application 
// * when the read operation is complete.
// *
// * Finally, if OUTPUT_IQ_DATA_CSV is defined, the application will write the 
// * I/Q data values out through the serial port in ascii form as comma-separated 
// * numeric value pairs (Q,I).  This can make it easier to take the data 
// * from the application and analyze it in a spreadsheet or other program.
// */

// //#define READ_IQ_DATA			/* uncomment this line to readout I/Q data */

//#define READ_IQ_BLOCKING 		/* use blocking mode when reading I/Q */
//// #define READ_IQ_NONBLOCKING 	/* use non-blocking mode when reading I/Q */

//// #define OUTPUT_IQ_DATA_CSV	/* uncomment to output I/Q data in CSV format */
											




//----------------------------------------------------------------------------------------------------
//CH201 Satic traget Rejection (STR) Mode Code App configurations
//----------------------------------------------------------------------------------------------------

/*========================= Sensor Firmware Selection ===========================*/

/* Select sensor firmware to use 
 *   The sensor firmware is specified during the call to ch_init(), by
 *   giving the name (address) of the firmware initialization function 
 *   that will be called.
 *
 *   To use a different sensor firmware type (e.g. a new distribution from
 *   Chirp), simply define CHIRP_SENSOR_FW_INIT_FUNC to equal the name of
 *   the init routine for the new firmware.
 */
#define	 CHIRP_SENSOR_FW_INIT_FUNC	ch201_gprstr_init	/* CH201 GPR STR firmware */


/*============================ Sensor Configuration =============================*/

/* Operating mode for the sensor
 *	 This value defines the mode in which the sensor will operate.  For this
 *	 example application, the choices are CH_MODE_FREERUN or CH_MODE_TRIGGERED_TX_RX.
 *
 *   This application uses CH_MODE_FREERUN by default, in which the sensor's internal
 *   clock will initiate the measurement cycles.  Triggered mode can be enabled instead, 
 *   to demonstrate controlling the sensor's measurement cycles from an external source
 *   (the periodic timer set up by the board support package). 
 */
#define CHIRP_SENSOR_MODE		CH_MODE_FREERUN

/* Maximum detection range for the sensor
 * This value will determine how long the sensor "listens" for an ultrasound 
 * signal (echo).  If the value specified here is greater than the maximum possible 
 * range for the sensor, the maximum possible range will be used.
 */
#define	CHIRP_SENSOR_MAX_RANGE_MM		3000	/* maximum range, in mm */

/* Detection threshold settings
 * These values set the required amplitude for an ultrasound signal to be
 * reported as a detected object.  In the CH201 STR sensor firmware, two
 * thresholds may be set.  Threshold 0 is used for samples very close to the sensor.
 * Threshold 1 is used for samples at more typical operating distances.
 *
 * If either value is set to zero, the sensor's default value for that threshold
 * will not be changed.
 */
#define	CHIRP_SENSOR_THRESHOLD_0		0	/* close range threshold (0 = use default) */
#define	CHIRP_SENSOR_THRESHOLD_1		50	/* standard threshold (0 = use default) */

/* Receive holdoff range
 * This value specifies the number of samples at the start (closest end) of the measurement 
 * that will be ignored when detecting a target, regardless of the signal amplitude.  This
 * setting can be combined with the above threshold values to tune the responsiveness of 
 * the sensor over the desired measurement range.  The default is zero.
 */
#define	CHIRP_SENSOR_RX_HOLDOFF			0	/* # of samples to ignore at start of meas */

/* Receive low-gain range
 * This value specifies the number of samples within the measurement that will use
 * a lower gain setting than is used for farther samples.
 * 
 * If this value is set to zero, the sensor's default number of samples for the low-gain 
 * range will not be changed.  Otherwise, it will be set to the specified number of samples.
 * In CH201 GPRSTR firmware, the default is 60 samples.
 */
#define	CHIRP_SENSOR_RX_LOW_GAIN		0	/* # of samples (0 = use default) */

/* Transmit length
 * This value controls the duration of the ultrasound pulse that is generated by the sensor
 * at the start of a measurement.  The units are the number of internal PMUT (transducer) 
 * cycles per pulse.  In general, a pulse using a longer transmit length will have a higher 
 * amplitude when the echo is received.
 *
 * If this value is set to zero, the sensor's default transmit length will not be changed.  
 * For CH201 GPRSTR firmware, the default is 30 cycles.  The default value used by the sensor
 * is appropriate for most situations.
 */
#define	CHIRP_SENSOR_TX_LENGTH			0	/* Tx pulse length, in cycles (0 = use default) */


/*============================ STR Configuration =============================*/

/* Range for static target rejection (STR)
 * This value specifies how many samples within the measurement will be filtered
 * for static target rejection.  Stationary objects that are beyond this limit will
 * be reported.
 *
 * If zero (0) is specified, STR will be enabled for the entire measurement range 
 * specified in CHIRP_SENSOR_MAX_RANGE_MM.
 */
#define	CHIRP_SENSOR_STR_RANGE		0		/* STR range, in samples (0 = entire meas range) */

/* These definitions set parameters affecting the application STR output. */
#define SHOW_PRESENCE_INDICATORS			/* COMMENT OUT THIS LINE TO DISABLE "Present" INDICATORS */

#define PRESENCE_HOLD_SECONDS      	1		/* time until end of presence is reported, in seconds */

#define NUM_RANGE_HISTORY_VALUES    7 		/* number of range history values used in filtering */ 


/*============================ Application Timing ============================*/

/* Define how often the application will get a new sample from the sensor(s) 
 *   This macro defines the sensor measurement interval, in milliseconds.  
 *
 *   For sensors in free-running mode (CH_MODE_FREERUN), the application will
 *   set this period as the sensor's internal sample interval.
 *
 *   For sensors in triggered mode (CH_MODE_TRIGGERED_TX_RX or 
 *   CH_MODE_TRIGGERED_RX_ONLY), the application will use a periodic timer to 
 *   trigger a sensor measurement each time this period elapses.
 */
//#define	MEASUREMENT_INTERVAL_MS		50		// 100ms interval = 10Hz sampling


/*===================  Application Storage for Sensor Data ======================*/

/* Define how many I/Q samples are expected by this application
 *   The following macro is used to allocate space for I/Q data in the 
 *   "chirp_data_t" structure, defined below.  It reserves enough space to
 *   hold I/Q samples for the maximum range of the sensor.
 */
#define IQ_DATA_MAX_NUM_SAMPLES  CH201_GPRSTR_MAX_SAMPLES	// use max for ch201_gprstr


/*===================  Build Options for I/Q Data Handling ======================*/

/* The following build options control if and how the raw I/Q data is read 
 * from the device after each measurement cycle, in addition to the standard 
 * range and amplitude.  
 *
 * Note that reading the I/Q data is not required for most basic sensing 
 * applications - the reported range value is typically all that is required. 
 * However, the full data set may be read and analyzed for more advanced 
 * sensing or data capture needs.
 *
 * Comment or un-comment the various definitions, as appropriate.
 *
 * Define READ_IQ_DATA to enable readout of the I/Q data.
 *
 * By default, this application will read the I/Q data in blocking mode 
 * (i.e. READ_IQ_BLOCKING is defined by default).  The data will be read from 
 * the device and placed in the I/Q data array field in the application's 
 * chirp_data structure.  Because the I/Q data is read in blocking mode, the 
 * call to ch_get_iq_data() will not return until the data has actually 
 * been copied from the device.  
 *
 * If, however, READ_IQ_NONBLOCKING is defined instead, the I/Q data will be 
 * read in non-blocking mode. The ch_get_iq_data() call will return immediately,
 * and a separate callback function will be called to notify the application 
 * when the read operation is complete.
 *
 * If OUTPUT_IQ_DATA_CSV is defined, the application will write the raw
 * I/Q data values out through the serial port in ascii form as comma-separated 
 * numeric value pairs (Q,I), each on its own line.  The data can then be
 * captured and imported into a spreadsheet or other analysis tool.
 *
 * If OUTPUT_AMP_DATA_CSV is defined, the application will read the I/Q data from
 * the sensor, then calculate the amplitude value for each I/Q pair and output
 * those values in a CSV format.  The amplitude data is displayed as a continuation
 * of the same output line that contains the basic range information, with comma
 * separators.
 */

 //#define READ_IQ_DATA			/* uncomment this line to readout I/Q data */

#define READ_IQ_BLOCKING 		/* use blocking mode when reading I/Q */
// #define READ_IQ_NONBLOCKING 	/* use non-blocking mode when reading I/Q */

// #define OUTPUT_IQ_DATA_CSV	/* uncomment to output I/Q data in CSV format */
// #define OUTPUT_AMP_DATA_CSV	/* uncomment to output calculated amplitudes in CSV */
											

#endif /* APP_CONFIG_H */

