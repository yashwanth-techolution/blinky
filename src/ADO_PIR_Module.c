#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <inttypes.h>
#include <zephyr/kernel.h>

#include "Includes/ADO_PIR_Module.h"
#include "Includes/Intent_Module.h"
#include "Includes/Command_Manager_Module.h"

static struct gpio_callback pir_1_cb_data;

const struct device *pir_1;
bool valid_PIR = Not_Detected_PIR;


/*
 * brief: It is a callback function triggered on an PIR sensor GPIO interrupt,
 *        when there is a person detected.
 */
void pir_1_detected(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
  // Debug print
  printk("PIR Detected.....\n");
  valid_PIR = Detected_PIR;
}

/*
 * brief: Assign a GPIO Pin to PIR sensor as an Input Pin and register its interrupt.
 */
int8_t init_PIR_sensor()
{
  int ret;

  pir_1 = device_get_binding(PIR1_GPIO_LABEL);
  if (pir_1 == NULL) 
  {
    printk("Error: didn't find %s device\n", PIR1_GPIO_LABEL);
    return errorcode1;
  }

  ret = gpio_pin_configure(pir_1, PIR1_GPIO_PIN, PIR1_GPIO_FLAGS);
  if (ret != 0) 
  {
    printk("Error %d: failed to configure %s pin %d\n", ret, PIR1_GPIO_LABEL, PIR1_GPIO_PIN);
    return errorcode2;
  }

  ret = gpio_pin_interrupt_configure(pir_1, PIR1_GPIO_PIN, GPIO_INT_EDGE_RISING);
  if (ret != 0) 
  {
    printk("Error %d: failed to configure interrupt on %s pin %d\n", ret, PIR1_GPIO_LABEL, PIR1_GPIO_PIN);
    return errorcode3;
  }

  gpio_init_callback(&pir_1_cb_data, pir_1_detected, BIT(PIR1_GPIO_PIN));
       
  // BSP: Removed adding the callback to make the PIR, 'Disabled' by default.
  gpio_add_callback(pir_1, &pir_1_cb_data);
  printk("Set up button at %s pin %d\n", PIR1_GPIO_LABEL, PIR1_GPIO_PIN);
  
  return 0;
}

/*
 * brief: Enables PIR Sensor interrupt callback function
 */
void enable_pir_1()
{
  printk("PIR Enabled\r\n");
  gpio_add_callback(pir_1, &pir_1_cb_data);
  
  // Just set a global flag to notify intent is detected, and main thread will check and notify cmd_mgr
  valid_PIR = Not_Detected_PIR;        // this will be checked and served in main thread
}

/*
 * brief: Disables PIR Sensor interrupt callback function
 */
void disable_pir_1()
{
  printk("PIR Disabled\r\n");
  gpio_remove_callback(pir_1, &pir_1_cb_data);
}