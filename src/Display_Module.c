#include "Includes/Display_Module.h"
#include "Includes/Intent_Module.h"
#include "Includes/ADO_Operations_Module.h"

// Global variables
struct uart_data_t tx_data;
const struct device *dis_en;
#if UWB
const struct device *out;
const struct device *uwb_device;
#endif
uint8_t battery_percent = 100, disp_status;

//#define TOUCH_DISPLAY

/*
 * brief: This function initializes the respected GPIO required to turn ON/OFF display.
 */
void init_display_control() {
  int8_t err;

  // Display sleep GPIO pin Initialisations
  dis_en = device_get_binding(DISPLAY_SLP_LABEL);
  if(dis_en == NULL) {
    printk("Couldn't find dispaly control GPIO device%s\n", DISPLAY_SLP_LABEL);
    return;  
  }

  // 2. Configure the Display sleep enable GPIO pin with its flags
  err = gpio_pin_configure(dis_en, DISPLAY_SLP_PIN, DISPLAY_SLP_FLAGS);
  if(err != 0) {
    printk("Error %d in configuring display enable GPIO device %s pin %d\n", err, DISPLAY_SLP_LABEL, DISPLAY_SLP_PIN);
    return;
  }
#if UWB
  //uwb pin
 out = device_get_binding(UART_OUTSIG_LABEL);
  if (out == NULL)
    printk("Couldn't find motor driver relay control GPIO device%s\n", UART_OUTSIG_LABEL);


    err = gpio_pin_configure(out, UART_OUTSIG_PIN, UART_OUTSIG_FLAGS);
  if (err != 0)
    printk("Error %d in configuring motor driver relay control GPIO device %s pin %d\n", err, UART_OUTSIG_LABEL, UART_OUTSIG_PIN);
    gpio_pin_set(out, UART_OUTSIG_PIN, GPIO_ACTIVE_LOW);
#endif
}


/*
 * brief: This function turns ON or OFF display power based on called argument command.
 */
int8_t display_power_control(bool on) {
  // Set the display enable GPIO
  door_close_to_open_percentage();
  if((current_door_percentage>10)&&(on==DIS_TURN_OFF))
      gpio_pin_set(dis_en, DISPLAY_SLP_PIN, DIS_TURN_ON);
  else gpio_pin_set(dis_en, DISPLAY_SLP_PIN, on);
   return 0;
}


#ifdef TOUCH_DISPLAY
// Function Definitions
/*
 * brief: This function will change the display page as per the page number passed.
 */
void chg_display_page(uint16_t page_num) {
  uint8_t chg_disp_page_cmd[MAX_DISPLAY_PAGE_CHG_CMD_LEN] = {0};
  
  // 5A A5 07 82 0084 5A01 0006  (0006) is the page to switch		
  
  // Prepare the command string to change display page
  chg_disp_page_cmd[0] = 0x5A;  // frame header1
  chg_disp_page_cmd[1] = 0xA5;  // frame header2
  chg_disp_page_cmd[2] = 0x07;  // data length
  chg_disp_page_cmd[3] = 0x82;  // command
  chg_disp_page_cmd[4] = (uint8_t)((0x0084 >> 8) & 0xFF);  // MSB of 0x0084
  chg_disp_page_cmd[5] = (uint8_t)((0x0084) & 0xFF);       // LSB of 0x0084
  chg_disp_page_cmd[6] = (uint8_t)((0x5A01 >> 8) & 0xFF);  // MSB of 0x5A01
  chg_disp_page_cmd[7] = (uint8_t)((0x5A01) & 0xFF);       // LSB of 0x5A01
  chg_disp_page_cmd[8] = (uint8_t)((page_num >> 8) & 0xFF);  // MSB of page_num
  chg_disp_page_cmd[9] = (uint8_t)((page_num) & 0xFF);       // LSB of page_num

  // Clean up the uart_buffer
  if(tx_data.len != 0) {
    memset(tx_data.data, 0, tx_data.len);
    tx_data.len = 0;
  }

  // prepare the uart_data buffer
  tx_data.len = MAX_DISPLAY_PAGE_CHG_CMD_LEN;
  memcpy(tx_data.data, chg_disp_page_cmd, tx_data.len);
  
  //// Debug print
  //printk("Display cmd:");
  //  for(int i = 0; i < 10; ++i)
  //    printk("Byte %d : %X\n", i, tx_data.data[i]);

  // Send the prepared command to display over UART_1 interface
  uart_write_data(&tx_data);
}

// Function Definitions
/*
 * brief: This function Updateds Battery percentage into dispaly page as per the current percentage.
 */
void Update_Battery_Percentage(uint8_t batt_percent) {
  uint8_t update_batt_per_cmd[MAX_BATTERY_PERCENT_UPDATE_CMD_LEN] = {0};
  
  // 5A A5 05 82 2000 xxxx  (xxxx) is the battery percentage in HEX	
  
  // Prepare the command string to update battery percentage
  update_batt_per_cmd[0] = 0x5A;  // frame header1
  update_batt_per_cmd[1] = 0xA5;  // frame header2
  update_batt_per_cmd[2] = 0x05;  // data length
  update_batt_per_cmd[3] = 0x82;  // command
  update_batt_per_cmd[4] = (uint8_t)((0x2000 >> 8) & 0xFF);  // MSB of 0x2000
  update_batt_per_cmd[5] = (uint8_t)((0x2000) & 0xFF);       // LSB of 0x2000
  update_batt_per_cmd[6] = (uint8_t)((batt_percent >> 8) & 0xFF);  // MSB of batt_percent
  update_batt_per_cmd[7] = (uint8_t)((batt_percent) & 0xFF);       // LSB of batt_percent

  // Clean up the uart_buffer
  if(tx_data.len != 0) {
    memset(tx_data.data, 0, tx_data.len);
    tx_data.len = 0;
  }

  // prepare the uart_data buffer
  tx_data.len = MAX_BATTERY_PERCENT_UPDATE_CMD_LEN;
  memcpy(tx_data.data, update_batt_per_cmd, tx_data.len);
  
  //// Debug print
  //printk("Display cmd:");
  //  for(int i = 0; i < 8; ++i)
  //    printk("Byte %d : %X\n", i, tx_data.data[i]);

  // Send the prepared command to display over UART_1 interface
  uart_write_data(&tx_data);
}

#else

//****************************************Non - touch display******************************************//
void chg_display_page(uint16_t page_num) {
  ////uint8_t chg_disp_page_cmd[MAX_DISPLAY_PAGE_CHG_CMD_LEN] = {0};
  uint8_t chg_disp_page_cmd[11] = {0};
  static bool timer2start = false;
  static struct k_timer my_timer2_sec;
  
  // 01 00 08 AA 22 00 00 CC 33 C3 3C  (00 00) is the page to switch
  
  // Prepare the command string to change display page

  chg_disp_page_cmd[0] = 0x01;
  chg_disp_page_cmd[1] = 0x00;
  chg_disp_page_cmd[2] = 0x08;
  chg_disp_page_cmd[3] = 0xAA;
  chg_disp_page_cmd[4] = 0x22;
  chg_disp_page_cmd[5] = (uint8_t)((page_num >> 8) & 0xFF);
  chg_disp_page_cmd[6] = (uint8_t)((page_num) & 0xFF);
  chg_disp_page_cmd[7] = 0xCC;
  chg_disp_page_cmd[8] = 0x33;
  chg_disp_page_cmd[9] = 0xC3;
  chg_disp_page_cmd[10] = 0x3C;

  disp_status=chg_disp_page_cmd[6];


  // Clean up the uart_buffer
  if(tx_data.len != 0) {
    memset(tx_data.data, 0, 11);
    tx_data.len = 0;
  }

  // prepare the uart_data buffer
  tx_data.len = 11;
  memcpy(tx_data.data, chg_disp_page_cmd, tx_data.len);
  
  //// Debug print
  //printk("Display cmd:");
  //  for(int i = 0; i < 10; ++i)
  //    printk("Byte %d : %X\n", i, tx_data.data[i]);

  // Send the prepared command to display over UART_1 interface
  uart_write_data(&tx_data);
  k_msleep(10);
   Update_Battery_Percentage(battery_percent);
}

// Function Definitions
/*
 * brief: This function Updateds Battery percentage into dispaly page as per the current percentage.
 */
void Update_Battery_Percentage(uint8_t batt_percent) {
  battery_percent = batt_percent;

  //uint8_t update_batt_per_cmd[MAX_BATTERY_PERCENT_UPDATE_CMD_LEN] = {0};
  uint8_t update_batt_per_cmd[23] = {0};
  
  // 01 00 14 AA 11 41 FF FF 00 00 00 35 00 15 31 30 30 20 25 CC 33 C3 3C  (31 30 30 is the battery percentage)
  // Prepare the command string to update battery percentage

  uint8_t batt_digit_arr[3] = {0x20, 0x20, 0x20};
  int i = 0;
  uint8_t batt_percent_value = batt_percent;
  
  while(batt_percent_value != 0) {
    uint8_t value =  batt_percent_value%10;
    batt_digit_arr[i] = value + 48;
    batt_percent_value = batt_percent_value/10;
    i++;
  }

  update_batt_per_cmd[0] = 0x01;
  update_batt_per_cmd[1] = 0x00;
  update_batt_per_cmd[2] = 0x14;
  update_batt_per_cmd[3] = 0xAA;
  update_batt_per_cmd[4] = 0x11;
  update_batt_per_cmd[5] = 0x41;
  update_batt_per_cmd[6] = 0xFF;
  update_batt_per_cmd[7] = 0xFF;
  update_batt_per_cmd[8] = 0x00;
  update_batt_per_cmd[9] = 0x00;
  update_batt_per_cmd[10] = 0x00;
  update_batt_per_cmd[11] = 0x35;
  update_batt_per_cmd[12] = 0x00;
  update_batt_per_cmd[13] = 0x15;
  update_batt_per_cmd[14] = batt_digit_arr[2];
  update_batt_per_cmd[15] = batt_digit_arr[1];
  update_batt_per_cmd[16] = batt_digit_arr[0];
  update_batt_per_cmd[17] = 0x20;
  update_batt_per_cmd[18] = 0x25;
  update_batt_per_cmd[19] = 0xCC;
  update_batt_per_cmd[20] = 0x33;
  update_batt_per_cmd[21] = 0xC3;
  update_batt_per_cmd[22] = 0x3C;


  // Clean up the uart_buffer
  if(tx_data.len != 0) {
    memset(tx_data.data, 0, 23);
    tx_data.len = 0;
  }

  // prepare the uart_data buffer
  tx_data.len = 23;
  memcpy(tx_data.data, update_batt_per_cmd, tx_data.len);
  
  //// Debug print
  //printk("Display cmd:");
  //  for(int i = 0; i < 8; ++i)
  //    printk("Byte %d : %X\n", i, tx_data.data[i]);

  // Send the prepared command to display over UART_1 interface
  uart_write_data(&tx_data);
}
#endif