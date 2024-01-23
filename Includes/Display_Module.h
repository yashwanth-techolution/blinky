#ifndef __DISPLAY_MODULE_H
#define __DISPLAY_MODULE_H

#include <stdint.h>
#include <zephyr/device.h>
#include <stdbool.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <hal/nrf_gpio.h>  // for stepper motor GPIOs initialisation
#include "UART_Module.h"

#define MAX_DISPLAY_PAGE_CHG_CMD_LEN 10U
#define MAX_BATTERY_PERCENT_UPDATE_CMD_LEN 8U

#define DIS_TURN_OFF true  // Active low GPIO pin requires true.
#define DIS_TURN_ON false  // Active low GPIO pin requires false.

#define LOW_CUT_OFF_BATT_PERCENT_2  5U  // Low battery warning indication level 2 cutt-off
#define LOW_CUT_OFF_BATT_PERCENT_1  10U // Low battery warning indication level 1 percentage
#define CUT_OFF_BATT_PERCENT    15U     // Low battery warning indication

// Page number definitions
#define DISP_START_PAGE         0x0000
#define DISP_WELCOME_PAGE       0x0001
#define DISP_ENTER_SETUP_PAGE   0x0002
#define DISP_WAIT_CONN_PAGE     0x0003
#define DISP_BT_PAIRED_PAGE     0x0004
#define DISP_CALIBRATION_PAGE   0x0005
#define DISP_OPEN_DOOR_PAGE     0x0006
#define DISP_CLOSE_DOOR_PAGE    0x0007
#define DISP_SUCCESS_PAGE       0x0008
#define DISP_DOOR_OPENING_PAGE  0x0009
#define DISP_DOOR_CLOSING_PAGE  0x000A
#define DISP_LOW_BATT_RED_PAGE  0x000B
#define DISP_DOOR_STOP          0x000C
#define DISP_SAY_DOOR_OPEN_PAGE 0x000D
#define DISP_UPDATING_PAGE      0X000E
#define DISP_FAILED_TO_UPDATE   0X000F

extern uint8_t disp_status;
#if UWB
extern const struct device *uwb_device;
extern const struct device *out;
extern void send_open(void);
#endif

// b. Get an instance of Display sleep pin from the device tree (dts)
//    Note: It must be defined in dts file prior using it here
#define DISPLAY_SLP_NODE          DT_ALIAS(display_on)
//#if DT_NODE_HAS_STATUS(DISPLAY_SLP_NODE, okay) && DT_NODE_HAS_PROP(DISPLAY_SLP_NODE, gpios)
  #define DISPLAY_SLP_LABEL       DT_GPIO_LABEL(DISPLAY_SLP_NODE, gpios)
  #define DISPLAY_SLP_PIN         DT_GPIO_PIN(DISPLAY_SLP_NODE, gpios)
  #define DISPLAY_SLP_FLAGS       (GPIO_OUTPUT | DT_GPIO_FLAGS(DISPLAY_SLP_NODE, gpios))
//#endif

#if UWB
#define UART_OUTSIG_NODE DT_ALIAS(uart_gpio)
//#if DT_NODE_HAS_STATUS(UART_OUTSIG_NODE, okay) && DT_NODE_HAS_PROP(UART_OUTSIG_NODE, gpios)
#define UART_OUTSIG_LABEL DT_GPIO_LABEL(UART_OUTSIG_NODE, gpios)
#define UART_OUTSIG_PIN DT_GPIO_PIN(UART_OUTSIG_NODE, gpios)
#define UART_OUTSIG_FLAGS (GPIO_OUTPUT | DT_GPIO_FLAGS(UART_OUTSIG_NODE, gpios))
//#endif

#endif

/**********************************************************************************************************
 * Function name  :   init_display_control()
 *
 * Description    :   1. This function intializes and binds dispaly enable GPIO pin to device structure 
 *                        pointer with respected port name, pin number and Flags.
 *
 * Params         :   Nothing
 * Returns        :   Nothing
 *
 ***********************************************************************************************************/
void init_display_control();


/**********************************************************************************************************
 * Function name  :   display_power_control()
 *
 * Description    :   1. This function turns ON/OFF particular display enable GPIO based on bool input.
 *
 * Params         :   1. <in> bool on:
 *                             boolean parameter to set display power state.
 * Returns        :   Returns 0 if success 
 *
 ***********************************************************************************************************/
int8_t display_power_control(bool on);


/**********************************************************************************************************
 * Function name  :   chg_display_page()
 *
 * Description    :   1. This function takes the page number in Hexadecimal notation, prepares the whole
 *                       page change command and writes it over the UART_1 interface to which the ADO display
 *                       is connected.
 *
 * Params         :   1. <in> uint16_t page_num :  
 *                            page number in hexadecimal notation, pls refer Display_Module.h for all page no.
 *                            definitions.
 * Returns        :   Nothing
 *
 ***********************************************************************************************************/
void chg_display_page(uint16_t page_num);


/**********************************************************************************************************
 * Function name  :   Update_Battery_Percentage()
 *
 * Description    :   1. This function takes the battery percentage in Hexadecimal notation, prepares the whole
 *                       battery percentage command and writes it over the UART_1 interface to which the ADO display
 *                       is connected.
 *
 * Params         :   1. <in> uint8_t batt_percent :  
 *                            Battery percentage in hexadecimal notation.
 * Returns        :   Nothing
 *
 ***********************************************************************************************************/
void Update_Battery_Percentage(uint8_t batt_percent);
void display_tx_enable(uint8_t status);
#endif