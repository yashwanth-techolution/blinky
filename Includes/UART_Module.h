#ifndef _UART_MODULE_H
#define _UART_MODULE_H

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/types.h>
#include <zephyr/kernel.h>

#define UWB 1
// The maximum size of the UART Tx/Rx buffer
#define UART_BUF_SIZE 45U
#define UART_BUF0_SIZE 5U
// Get the UART1 Device tree node label
#define ADO_DISPLAY_UART_LABEL DT_LABEL(DT_NODELABEL(uart1))
#define ADO_DISPLAY_UART0_LABEL DT_LABEL(DT_NODELABEL(uart0))

/* STEP 9.1.1 - Define the size of the receive buffer */
#define RECEIVE_BUFF_SIZE 256
#define RECEIVE_BUFF0_SIZE 45
/* STEP 9.2 - Define the receiving timeout period */
#define RECEIVE_TIMEOUT 100
// Structure for the UART data
struct uart_data_t {
  uint8_t data[UART_BUF_SIZE];
  uint16_t len;
};
struct uart0_data_t {
  uint8_t data[UART_BUF0_SIZE];
  uint16_t len;
};

extern const struct device *uart_1;
extern const struct device *uart_0;
#if UWB
extern struct uart_data_t u_RX_data;
extern struct uart0_data_t u0_RX_data;
extern uint8_t distance_lessthan200mtr;
extern volatile uint16_t distance;
#endif
// Function declarations
int8_t initUart1(void);
int8_t initUart0(void);
void uart_write_data(struct uart_data_t *tx);

#if UWB
uint8_t enable_uart_rx(void);
uint8_t enable_uart0_rx(void);
void ado_send_uwb_devConf();
void ado_send_iosuwb_devConf();
void Uwb_init_AND();
void Uwb_init_IOs();
void Uwb_stop();
void Uwb_phone_config(uint8_t *data);
void Uwb_iphone_config(uint8_t *data);
#endif

#endif