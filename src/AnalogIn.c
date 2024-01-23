#include "Includes/AnalogIn.h"
#include <zephyr/drivers/adc.h>
#include <string.h>

// Simple analog input method
// this just reads a sample then waits then returns it

// ADC Sampling Settings
// doc says that impedance of 800K == 40usec sample time

static bool _IsInitialized = false;
static uint8_t _LastChannel = 250;
static int16_t m_sample_buffer[BUFFER_SIZE];

// the channel configuration with channel not yet filled in
static struct adc_channel_cfg m_1st_channel_cfg =
    {
        .gain = ADC_GAIN,
        .reference = ADC_REFERENCE,
        .acquisition_time = ADC_ACQUISITION_TIME,
        .channel_id = 4, // gets set during init
        .differential = 0,
#if CONFIG_ADC_CONFIGURABLE_INPUTS
        .input_positive = 0, // gets set during init
#endif
};

// return device* for the adc
static const struct device *getAdcDevice(void) {
  return device_get_binding(ADC_DEVICE_NAME);
}

// initialize the adc channel
static const struct device *init_adc(int channel) {
  int ret;
  const struct device *adc_dev = getAdcDevice();
  if (_LastChannel != channel) {
    _IsInitialized = false;
    _LastChannel = channel;
  }

  if (adc_dev != NULL && !_IsInitialized) {
    // strangely channel_id gets the channel id and input_positive gets id+1
    m_1st_channel_cfg.channel_id = channel;
#if CONFIG_ADC_CONFIGURABLE_INPUTS
    m_1st_channel_cfg.input_positive = channel + 1,
#endif
    ret = adc_channel_setup(adc_dev, &m_1st_channel_cfg);
    if (ret != 0) {
      // LOG_INF("Setting up of the first channel failed with code %d", ret);
      adc_dev = NULL;
    } else {
      _IsInitialized = true; // we don't have any other analog users
    }
  }
  memset(m_sample_buffer, 0, sizeof(m_sample_buffer));
  return adc_dev;
}

// ------------------------------------------------
// read one channel of adc
// ------------------------------------------------
static int16_t readOneChannel(int channel) {
  const struct adc_sequence sequence =
      {
          .options = NULL,           // extra samples and callback
          .channels = BIT(channel),  // bit mask of channels to read
          .buffer = m_sample_buffer, // where to put samples read
          .buffer_size = sizeof(m_sample_buffer),
          .resolution = ADC_RESOLUTION, // desired resolution
          .oversampling = 0,            // don't oversample
          .calibrate = 0                // don't calibrate
      };

  int ret;
  int16_t sample_value = BAD_ANALOG_READ;
  const struct device *adc_dev = init_adc(channel);
  if (adc_dev) {
    ret = adc_read(adc_dev, &sequence);
    if (ret == 0) {
      sample_value = m_sample_buffer[0];
    }
  }
  return sample_value;
}

// ------------------------------------------------
// high level read adc channel and convert to float voltage
// ------------------------------------------------
float AnalogRead(int channel) {
  int16_t serial_voltage = readOneChannel(channel);
  if (serial_voltage == BAD_ANALOG_READ) {
    return serial_voltage;
  }

  int multip = ADC_08_BIT_VALUE;
  // find 2**adc_resolution
  switch (ADC_RESOLUTION) {
  default:
  case 8:
    multip = ADC_08_BIT_VALUE;
    break;
  case 10:
    multip = ADC_10_BIT_VALUE;
    break;
  case 12:
    multip = ADC_12_BIT_VALUE;
    break;
  case 14:
    multip = ADC_14_BIT_VALUE;
    break;
  }

  // the 3.3 relates to the voltage divider.
  float fout = (serial_voltage * ADC_VDD / multip);
  return fout;
}

/*
 * brief: This function takes respected ADC channel number.
 *        returns average analog voltage from sensors by taking no.of samples = 500.
 */
int average_adc(int channel) {
  double raw_voltage = 0;
  for (int i = 0; i < NO_OFF_ADC_SAMPLES; i++) {
    raw_voltage += AnalogRead(channel);
  }
  int avg_voltage = (raw_voltage / NO_OFF_ADC_SAMPLES) * volTomv_converter;
  printk("voltage = %d -----------\n", avg_voltage);
  return avg_voltage;
}

/*
 * brief: This function takes respected ADC channel number.
 *        returns calculated ADO battery voltage in mV.
 */
uint16_t ADO_Battery_Voltage_mV(int channel) {
  double raw_voltage = 0;
  for (int i = 0; i < NO_OFF_ADC_SAMPLES; i++) {
    raw_voltage += AnalogRead(channel);
  }
  int avg_voltage = (raw_voltage / NO_OFF_ADC_SAMPLES) * volTomv_converter;
  uint16_t battery_Voltage_mV = (avg_voltage + analog_correction_Vol) * scalingFactor_BatVoltage;
  // printk("battery voltage (mV) = %d,  ", battery_Voltage_mV);

  if (battery_Voltage_mV <= battery_min_voltage) {
    battery_Voltage_mV = battery_min_voltage + 10;
  }
  return battery_Voltage_mV;
}

/*
 * brief: This function takes respected ADC channel number.
 *        returns present Battery percentage of ADO based on reading analog battery voltage.
 */
uint8_t ADO_Battery_Percen_Read(int channel) {
  int8_t batt_percent;
  batt_percent = (int8_t)((ADO_Battery_Voltage_mV(channel) - battery_min_voltage) / Denominator_value_percent_cal); //  ((present_batt_volt - min_batt_volt)/(max_batt_vol - min_batt-vol))x 100 // battery percentage calculation
  if (batt_percent >= Battery_max_Percentage) {
    batt_percent = Battery_max_Percentage;
  } else if (batt_percent <= Battery_min_Percentage) {
    batt_percent = Battery_min_Percentage;
  }
  // printk("battery percentage (%) = %d\n", batt_percent);
  return batt_percent;
}

/*
 * brief: This function takes respected ADC channel number.
 *        returns calculated current data entire ADO in mA.
 */
uint16_t ADO_Current_mA(int channel) {
  double raw_voltage = 0;
  for (int i = 0; i < NO_OFF_ADC_SAMPLES; i++) {
    raw_voltage += AnalogRead(channel);
  }
  int avg_voltage = (raw_voltage / NO_OFF_ADC_SAMPLES) * volTomv_converter;
  uint16_t current_mA = ((avg_voltage + Current_offset_value) - start_current_representation) * sensitivity_factor;
  // printk("ADO Current (mA) = %d, analog read mV = %d\n", current_mA, avg_voltage);
  printf("%.1f\n", (float)current_mA);
  return current_mA;
}

#ifdef ADO_RAYTAC_P2_1
/*
 * brief: This function takes respected ADC channel number.
 *        returns calculated current data entire ADO in mA.
 */
uint16_t ADO_drive_motor_Current_mA(int channel) {
  double raw_voltage = 0;
  for (int i = 0; i < NO_OFF_ADC_SAMPLES; i++) {
    raw_voltage += AnalogRead(channel);
  }
  int avg_voltage = (raw_voltage / NO_OFF_ADC_SAMPLES) * 1000;
  uint16_t current_mA = ((avg_voltage + 205) - 2500) * 5;
  printk("ADO Current (mA) = %d, analog read mV = %d\n", current_mA, avg_voltage);
  return current_mA;
}
#endif