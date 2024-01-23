#ifndef ANALOG_IN_H
#define ANALOG_IN_H

#include <stdint.h>
#include <stdbool.h>

#include "Device_Status_Module.h"


#ifdef __cplusplus
extern "C" {
#endif


#ifdef ADO_RAYTAC_P2
// Defining ADO current sensor connected ADC channe 0 number. 
#define ADO_CURRENT_SENSOR 2U

// Defining Stepper motor's and Micro DC motors current sensor connected ADC channel number. 
#define CLAMP_AND_MICRO_MOTOR_CURRENT_SENSOR 1U

// Defining Battery voltage sensing circuit connected ADC. 
#define BATTERY_VOLTAGE_SENSOR 0U
#endif



#ifdef ADO_RAYTAC_P2_1
// Defining ADO current sensor connected ADC channe 0 number. 
#define ADO_CURRENT_SENSOR 0U

// Defining Wheel Drive motor's current sensor connected ADC channel number. 
#define DRIVE_MOTOR_CURRENT_SENSOR 1U

// Defining Stepper motor's and Micro DC motors current sensor connected ADC channel number. 
#define CLAMP_AND_MICRO_MOTOR_CURRENT_SENSOR 2U

// Defining Battery voltage sensing circuit connected ADC. 
#define BATTERY_VOLTAGE_SENSOR 3U
#endif


#define ADC_08_BIT_VALUE 256
#define ADC_10_BIT_VALUE 1024
#define ADC_12_BIT_VALUE 4096
#define ADC_14_BIT_VALUE 16384

#define Denominator_value_percent_cal  35  //1/(max_batt_vol - min_batt-vol))x 100     i.e   (1/12500-9000)*100=35
#define battery_min_voltage            9000
#define sensitivity_factor             5
#define scalingFactor_BatVoltage       4.2
#define start_current_representation   2500
#define analog_correction_Vol          217   
#define volTomv_converter              1000     
#define Battery_max_Percentage        100
#define Battery_min_Percentage        0
#define Current_offset_value         205

#define ADC_DEVICE_NAME		DT_LABEL(DT_ALIAS(adcctrl))
#define ADC_RESOLUTION		12
#define ADC_GAIN                ADC_GAIN_1_6
#define ADC_REFERENCE		ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME	ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40)
#define BUFFER_SIZE             6
#define ADC_VDD                 3.3

#define NO_OFF_ADC_SAMPLES    500U      // No.of Analog read input samples to calculate propere current values. 
#define BAD_ANALOG_READ       -123    // Predefined analog value if ADC fails to read.


/**********************************************************************************************************
 * Function name  :   AnalogRead()
 *
 * Description    :   To read ADC out from respected ADC channel of controller. 
 *
 * Params         :   1. <in> int channel: Respected sensor connected ADC channel ID
 *
 * Returns        :   Caluclated Analog float value.
 ***********************************************************************************************************/ 
float AnalogRead(int channel);


/**********************************************************************************************************
 * Function name  :   average_adc()
 *
 * Description    :   To read average ADC out from respected ADC channel of controller in mV.
 *
 * Params         :   1. <in> int channel: Respected sensor connected ADC channel ID

 * Returns        :   Caluclated Analog average mV in int.
 ***********************************************************************************************************/ 
int average_adc(int channel);


/**********************************************************************************************************
 * Function name  :   ADO_Battery_Voltage_mV()
 *
 * Description    :   It will tells actual battery voltage in mV formate.
 *
 * Params         :   1. <in> int channel: Respected sensor connected ADC channel ID

 * Returns        :   Battery voltage in mV
 ***********************************************************************************************************/ 
uint16_t ADO_Battery_Voltage_mV(int channel);


/**********************************************************************************************************
 * Function name  :   ADO_Battery_Percentage()
 *
 * Description    :   Caluclates battery percentage based on battery voltage.
 *
 * Params         :   1. <in> int channel: Respected sensor connected ADC channel ID

 * Returns        :   Caluclated battery percentage.
 ***********************************************************************************************************/ 
uint8_t ADO_Battery_Percen_Read(int channel);


/**********************************************************************************************************
 * Function name  :   ADO_Current_mA()
 *
 * Description    :   Tells about entire ADO current cusumption in mA at particulat instant. 
 *
 * Params         :   1. <in> int channel: Respected sensor connected ADC channel ID

 * Returns        :   caluculated ADO current in mA
 ***********************************************************************************************************/ 
uint16_t ADO_Current_mA(int channel);


#ifdef ADO_RAYTAC_P2_1
/**********************************************************************************************************
 * Function name  :   ADO_drive_motor_Current_mA()
 *
 * Description    :   Tells about ADO wheel drive motor current cusumption in mA at particulat instant. 
 *
 * Params         :   1. <in> int channel: Respected sensor connected ADC channel ID

 * Returns        :   caluculated ADO current in mA
 ***********************************************************************************************************/
uint16_t ADO_drive_motor_Current_mA(int channel);
#endif


#ifdef __cplusplus
}
#endif


#endif