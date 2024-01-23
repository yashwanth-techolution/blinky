#include "Includes/UART_Module.h"
#include "Includes/BLE_Module.h"
#include "Includes/Command_Manager_Module.h"
#include "Includes/Display_Module.h"
#include <stdio.h>
#include <string.h>

// Global variables
const struct device *uart_1;
#if UWB
const struct device *uart_0;
uint8_t uart_1_rx_buf[RECEIVE_BUFF_SIZE];
uint8_t uart_1_rx_buffer[RECEIVE_BUFF_SIZE];
uint8_t uart_1_rx_data[RECEIVE_BUFF_SIZE];
uint8_t *udata;
struct uart_data_t u_RX_data;
uint8_t uart_0_rx_buf[RECEIVE_BUFF0_SIZE];
uint8_t uart_0_rx_buffer[RECEIVE_BUFF0_SIZE];
uint8_t uart_0_rx_data[RECEIVE_BUFF0_SIZE];
uint8_t *u0data;
struct uart0_data_t u0_RX_data;
bool received = true;
static struct k_mbox_msg uwb_send_msg;
static ado_cmd_mgr_msg_t uwb_cmd_mgr_msg;
typedef enum {
  // UwbInitializedData    = 0x01,
  // UwbDidStart           = 0x02,
  // UwbDidStop            = 0x03,
  // Uwb_notinitialized    = 0x04
  Walking_towords_door = 0x01,
  Walking_away_from_door = 0x02,
} ResponseId_t;
// ResponseId_t  uwb_status=Uwb_notinitialized;
volatile uint16_t distance = 0;
uint16_t uwbmac_id = 0;
bool uwb_uart_sending = false;
bool uart_excuting = false;
uint8_t distance_lessthan200mtr = 3;
#endif
struct k_sem uart_sem;
K_SEM_DEFINE(uart_sem, 0, 1);

// Function definitions
/*
 * brief: This will initialise the UART1 Peripheral to which we have connected the display
 */

#if UWB
void send_open(void) {
  uwb_cmd_mgr_msg.msg_buf[0] = 1;
  k_mbox_put(&cmd_mgr_mb, &uwb_send_msg, K_MSEC(50));
  distance_lessthan200mtr = 3;
}
void send_close(void) {
  uwb_cmd_mgr_msg.msg_buf[0] = 2;
  k_mbox_put(&cmd_mgr_mb, &uwb_send_msg, K_MSEC(50));
  distance_lessthan200mtr = 3;
}

/* STEP 6 - Define the callback function for UART */
uint8_t enable_uart_rx(void) {
  int err = 0;
  err = uart_rx_enable(uart_1, uart_1_rx_buf, sizeof uart_1_rx_buf, RECEIVE_TIMEOUT);
  if (err) {
    printk("uart rx enable error\n");
    return 1;
  }
}
uint8_t enable_uart0_rx(void) {
  int err = 0;
  err = uart_rx_enable(uart_0, uart_0_rx_buf, sizeof uart_0_rx_buf, RECEIVE_TIMEOUT);
  if (err) {
    printk("uart rx enable error\n");
    return 1;
  }
}
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data) {
  switch (evt->type) {

  case UART_RX_RDY: {

    // u_RX_data.len = evt->data.rx.len;
    uint8_t *data = evt->data.rx.buf;
    memset(u_RX_data.data, 0, RECEIVE_BUFF0_SIZE);
    memcpy(u_RX_data.data, data, RECEIVE_BUFF0_SIZE);
    for (int i = 0; i < RECEIVE_BUFF0_SIZE; i++)
      printk(" %x,", data[i]);
    if ((u_RX_data.data[2] == 19) && (received == true)) // 19 android, 38 ios
    {
      uwbmac_id = (u_RX_data.data[20] << 8) + u_RX_data.data[21];
      if (uwbmac_id != 0)
        received = false;
      distance = 1000;
    } else if ((u_RX_data.data[2] == 38) && (received == true)) {
      uwbmac_id = (u_RX_data.data[37] << 8) + u_RX_data.data[38];
      if (uwbmac_id != 0)
        received = false;
      distance = 1000;
    } else if (u_RX_data.data[2] == 8) {
      distance = (u_RX_data.data[5] << 8) + u_RX_data.data[6];
      printk(" \n uwbmac_id & disatnce   %X - %d  %d  %d\n", uwbmac_id, distance, u_RX_data.data[8], u_RX_data.data[10]);
    }
    else if (u_RX_data.data[2] == 1) {
      if (Walking_away_from_door == u_RX_data.data[3]) {
        if(distance_lessthan200mtr!=5)
          distance_lessthan200mtr = 0;
          if(auto_close_timer>250)
          auto_close_timer=auto_close_timer-250;
        printk("\n###### uwb CLOSE ########\n");

      } else if ( Walking_towords_door == u_RX_data.data[3]) {
        if(distance_lessthan200mtr!=4)
          distance_lessthan200mtr = 1;
          if(auto_close_timer<250)
          auto_close_timer=auto_close_timer+250;
        printk("\n###### uwb OPEN ########\n");
      }
    }

    // printk("UART_RX_RDY\n");
    uart_rx_disable(uart_1);
    memset(evt->data.rx.buf, 0, RECEIVE_BUFF_SIZE);
    enable_uart_rx();
    break;
  }
  case UART_RX_BUF_REQUEST:
    memset(evt->data.rx.buf, 0, RECEIVE_BUFF_SIZE);
    uart_rx_buf_rsp(dev, uart_1_rx_buffer, RECEIVE_BUFF_SIZE);
    // printk("UART_RX_BUF_REQUEST\n");
    break;
    //}
  case UART_RX_DISABLED:
    uart_rx_enable(dev, uart_1_rx_buf, sizeof uart_1_rx_buf, RECEIVE_TIMEOUT);
    // printk("UART_RX_DISABLED\n");
    break;
  case UART_TX_DONE:
    // printk("UART_TX_DONE\n");
    break;

  default:
    break;
  }
}
static void uart_cb0(const struct device *dev, struct uart_event *evt, void *user_data) {
  switch (evt->type) {

  case UART_RX_RDY: {
    uint8_t *data = evt->data.rx.buf;
    memset(u0_RX_data.data, 0, 5);
    memcpy(u0_RX_data.data, data, 4);
    for (int i = 0; i < 4; i++)
      printk(" %x,", data[i]);

    if (Walking_away_from_door == u0_RX_data.data[0]) {
      //distance_lessthan200mtr = 0;

      if(distance_lessthan200mtr!=5)
          distance_lessthan200mtr = 0;
          if(auto_close_timer>250)
          auto_close_timer=auto_close_timer-250;

      printk("\n###### uwb CLOSE ########\n");

    } else if (Walking_towords_door == u0_RX_data.data[0]) {
      //distance_lessthan200mtr = 1;

      if(distance_lessthan200mtr!=4)
          distance_lessthan200mtr = 1;
          if(auto_close_timer<250)
          auto_close_timer=auto_close_timer+250;

      printk("\n###### uwb OPEN ########\n");
    } else {
      distance_lessthan200mtr = 3;
    }
    // printk("UART_RX_RDY\n");
    uart_rx_disable(uart_0);
    memset(evt->data.rx.buf, 0, RECEIVE_BUFF0_SIZE);
    enable_uart0_rx();
    break;
  }
  case UART_RX_BUF_REQUEST:
    memset(evt->data.rx.buf, 0, RECEIVE_BUFF0_SIZE);
    uart_rx_buf_rsp(dev, uart_0_rx_buffer, RECEIVE_BUFF0_SIZE);
    // printk("UART_RX_BUF_REQUEST\n");
    break;
  case UART_RX_DISABLED:
    uart_rx_enable(dev, uart_0_rx_buf, sizeof uart_0_rx_buf, RECEIVE_TIMEOUT);
    // printk("UART_RX_DISABLED\n");
    break;
  case UART_TX_DONE:
    // printk("UART_TX_DONE\n");
    break;

  default:
    break;
  }
}
#endif
int8_t initUart1(void) {
  int8_t err;
  uart_1 = device_get_binding(ADO_DISPLAY_UART_LABEL);

  if (!uart_1) {
    // Debug prints
    printk("Err: Unable to get UART_1 DTS binding\n");
    return -ENXIO;
  }
#if UWB
  err = uart_callback_set(uart_1, uart_cb, NULL);
  if (err) {
    printk("callback set error\n");
    return 1;
  }
  enable_uart_rx();
  //initUart0();
  uwb_cmd_mgr_msg.msg_type = COMMAND;
  uwb_cmd_mgr_msg.msg_cmd = 2;
  uwb_cmd_mgr_msg.msg_buf_len = 2;
  uwb_cmd_mgr_msg.msg_buf[0] = 0;
  uwb_send_msg.info = 0;
  uwb_send_msg.size = sizeof(uwb_cmd_mgr_msg);
  uwb_send_msg.tx_data = &uwb_cmd_mgr_msg;
  uwb_send_msg.tx_target_thread = cmd_mgr_thrd_id;
#endif
  return 0;
}
#if UWB
int8_t initUart0(void) {
  int8_t err;
  uart_0 = device_get_binding(ADO_DISPLAY_UART0_LABEL);

  if (!uart_0) {
    // Debug prints
    printk("Err: Unable to get uart_0 DTS binding\n");
    return -ENXIO;
  }
  err = uart_callback_set(uart_0, uart_cb0, NULL);
  if (err) {
    printk("callback set error\n");
    return 1;
  }
  enable_uart0_rx();
  return 0;
}
#endif
/*
 * brief: This function will write data to uart_1
 */
void uart_write_data(struct uart_data_t *tx) {
  if (k_sem_take(&uart_sem, K_MSEC(50)) != 0) {
    printk("uart Semaphore not released Input data not available!\n");
  } else {
    // TODO: Do any precondition check or validation here
    uart_tx(uart_1, tx->data, tx->len, SYS_FOREVER_MS);
  }
  k_sem_give(&uart_sem);
}

#if UWB
void ado_send_uwb_devConf() {
  if (received == false) {
    printk("ado_send_uwb_devConf()\n");
    ADO_notify_ble(ble_notify_thrd_id, UWB_PRAMS_RESPONSE, 1, 1, u_RX_data.data + 3, 19);
    memset(u_RX_data.data, 0, UART_BUF_SIZE);
    received = true;
  }
}

void ado_send_iosuwb_devConf() {
  if (received == false) {
    printk("ado_send_iosuwb_devConf()\n");
    ADO_notify_ble(ble_notify_thrd_id, UWB_PRAMS_RESPONSE, 1, 1, u_RX_data.data + 3, 38);
    uwb_sendCmdACK(u_RX_data.data + 4,37);
    memset(u_RX_data.data, 0, UART_BUF_SIZE);
    received = true;
  }
}

void Uwb_init_AND() {
  gpio_pin_set(out, UART_OUTSIG_PIN, GPIO_ACTIVE_HIGH);
  k_msleep(100);
  gpio_pin_set(out, UART_OUTSIG_PIN, GPIO_ACTIVE_LOW);
  printk("Uwb_init_AND()\n");
  struct uart_data_t u_data;
  uint8_t uwbtx_data[4] = {0x01, 0x00, 0x01, 0xA5};
  u_data.len = 4;
  memcpy(u_data.data, uwbtx_data, u_data.len);
  k_msleep(3000);
  uart_write_data(&u_data);
  k_msleep(1000);
  ado_send_uwb_devConf();
}
void Uwb_init_IOs() {
  gpio_pin_set(out, UART_OUTSIG_PIN, GPIO_ACTIVE_HIGH);
  k_msleep(100);
  gpio_pin_set(out, UART_OUTSIG_PIN, GPIO_ACTIVE_LOW);
  printk("Uwb_init_IOs()\n");
  struct uart_data_t u_data;
  uint8_t uwbtx_data[4] = {0x01, 0x00, 0x01, 0x0A};
  u_data.len = 4;
  memcpy(u_data.data, uwbtx_data, u_data.len);
  k_msleep(3000);
  uart_write_data(&u_data);
  k_msleep(1000);
  ado_send_iosuwb_devConf();
}

void Uwb_stop() {
  printk("Uwb_stop()\n");
  struct uart_data_t u_data;
  uint8_t uwbtx_data[4] = {0x01, 0x00, 0x01, 0x0C};
  u_data.len = 4;
  memcpy(u_data.data, uwbtx_data, u_data.len);
  uart_write_data(&u_data);
  k_msleep(500);
  gpio_pin_set(out, UART_OUTSIG_PIN, GPIO_ACTIVE_HIGH);
  k_msleep(100);
  gpio_pin_set(out, UART_OUTSIG_PIN, GPIO_ACTIVE_LOW);
  if(auto_close_timer>250)
     auto_close_timer=auto_close_timer-250;
}

void Uwb_phone_config(uint8_t *data) {
  // Uwb_stop();
  printk("Uwb_phone_config()\n");
  struct uart_data_t u_data;
  uint8_t uwbtx_data[18] = {0x01, 0x00, 0x0F}; //,0x0B,0x01,0x0,0x0,0x0,0x1B,0x70,0x56,0x53,0xA,0x9,0x1,0x1,0x18,0x2B};
  u_data.len = 18;
  memcpy(u_data.data, uwbtx_data, 3);
  memcpy(u_data.data + 3, data, 15);
  uart_write_data(&u_data);
  k_msleep(500);
}

void Uwb_iphone_config(uint8_t *data) {
  // Uwb_stop();
  printk("Uwb_phone_config()\n");
  struct uart_data_t u_data;
  //  <ESC>[0mreceived config data (uint8_t)evt.eMsgType 0 evt.Size 31<LF>
  // B, 1, 0, 1, 0, 19, 45, 55, BA, A2, 0, 0, B, 9, 6, 0, 10, E, 9E, 1, 3, BF, 28, 13, 91, A7, BD, CF, B3, C8, 0,
  uint8_t uwbtx_data[34] = {0x01, 0x00, 0x1F}; //,0x0B,0x01,0x0,0x0,0x0,0x1B,0x70,0x56,0x53,0xA,0x9,0x1,0x1,0x18,0x2B};
  u_data.len = 34;
  memcpy(u_data.data, uwbtx_data, 3);
  memcpy(u_data.data + 3, data, 31);
  uart_write_data(&u_data);
  k_msleep(500);
}
#endif