#ifndef ADO_PIR_MODULE_H__
#define ADO_PIR_MODULE_H__

#include<stdint.h>
#include<stdbool.h>

#include<zephyr/device.h>
#include<zephyr/devicetree.h>
#include<zephyr/drivers/gpio.h>

#include "Device_Status_Module.h"


#ifdef ADO_RAYTAC_P2
/*
 * Get PIR configuration from the devicetree pir_1 alias.
 *
 * At least a GPIO device and pin number must be provided. The 'flags'
 * cell is optional.
 */
#define PIR_1_NODE	DT_ALIAS(pir_1)

#if DT_NODE_HAS_STATUS(PIR_1_NODE, okay)
#define PIR1_GPIO_LABEL	DT_GPIO_LABEL(PIR_1_NODE, gpios)
#define PIR1_GPIO_PIN	DT_GPIO_PIN(PIR_1_NODE, gpios)
#define PIR1_GPIO_FLAGS	(GPIO_INPUT | DT_GPIO_FLAGS(PIR_1_NODE, gpios))
#else
#error "Unsupported board: sw0 devicetree alias is not defined"
#define SW0_GPIO_LABEL	""
#define SW0_GPIO_PIN	0
#define SW0_GPIO_FLAGS	0
#endif
#endif


#ifdef ADO_RAYTAC_P2_1
/*
 * Get PIR configuration from the devicetree pir_1 alias.
 *
 * At least a GPIO device and pin number must be provided. The 'flags'
 * cell is optional.
 */
#define PIR_1_NODE	DT_ALIAS(pir_sensor_1)

#if DT_NODE_HAS_STATUS(PIR_1_NODE, okay)
#define PIR1_GPIO_LABEL	DT_GPIO_LABEL(PIR_1_NODE, gpios)
#define PIR1_GPIO_PIN	DT_GPIO_PIN(PIR_1_NODE, gpios)
#define PIR1_GPIO_FLAGS	(GPIO_INPUT | DT_GPIO_FLAGS(PIR_1_NODE, gpios))
#else
#error "Unsupported board: sw0 devicetree alias is not defined"
#define SW0_GPIO_LABEL	""
#define SW0_GPIO_PIN	0
#define SW0_GPIO_FLAGS	0
#endif
#endif

#define Detected_PIR       1
#define Not_Detected_PIR   0

#define errorcode1  -1   //didn't find %s device
#define errorcode2  -2   //failed to configure
#define errorcode3  -3   //failed to configure interrupt

extern bool valid_PIR;


extern int8_t init_PIR_sensor();

/**********************************************************************************************************
 * Function name  :   pir_1_intent_detect()
 *
 * Description    :   Disables PIR Sensor interrupt callback function
 *
 * Params         :   None

 * Returns        :   Nothing
 ***********************************************************************************************************/ 
extern void disable_pir_1();

/**********************************************************************************************************
 * Function name  :   enable_pir_1()
 *
 * Description    :   Enables PIR Sensor interrupt callback function
 *
 * Params         :   None

 * Returns        :   Nothing
 ***********************************************************************************************************/ 
extern void enable_pir_1();
#endif
