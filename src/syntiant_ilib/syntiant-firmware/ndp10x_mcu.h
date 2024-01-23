/*
 * SYNTIANT CONFIDENTIAL
 *
 * _____________________
 *
 * Copyright (c) 2017-2020 Syntiant Corporation
 * All Rights Reserved.
 *
 * NOTICE:  All information contained herein is, and remains the property of
 * Syntiant Corporation and its suppliers, if any.  The intellectual and
 * technical concepts contained herein are proprietary to Syntiant Corporation
 * and its suppliers and may be covered by U.S. and Foreign Patents, patents in
 * process, and are protected by trade secret or copyright law.  Dissemination
 * of this information or reproduction of this material is strictly forbidden
 * unless prior written permission is obtained from Syntiant Corporation.
 *
 */

#ifndef NDP10X_MCU_H
#define NDP10X_MCU_H

#ifdef __cplusplus
extern "C" {
#endif

/* MCU Interrupt vectors */
#define INTERRUPT_REG_BASE (0xE000E100UL)
#define INTERRUPT_REG_ENABLE (INTERRUPT_REG_BASE + 0x00)
#define INTERRUPT_REG_DISABLE (INTERRUPT_REG_BASE + 0x80)
#define INTERRUPT_PREG_ENABLE (INTERRUPT_REG_BASE + 0x100)
#define INTERRUPT_PREG_DISABLE (INTERRUPT_REG_BASE + 0x180)
#define INTERRUPT_IPREG (INTERRUPT_REG_BASE + 0x300)

/* MCU GPIO */
#define MCU_GPIO_REG_BASE (0x40011000UL)
#define MCU_GPIO_FUNC_SET (MCU_GPIO_REG_BASE + 0x18)
#define MCU_GPIO_FUNC_CLEAR (MCU_GPIO_REG_BASE + 0x1C)
#define MCU_GPIO_OUT_ENABLE (MCU_GPIO_REG_BASE + 0x10)
#define MCU_GPIO_INTEN_SET (MCU_GPIO_REG_BASE + 0x20)
#define MCU_GPIO_INTEN_CLR (MCU_GPIO_REG_BASE + 0x24)
#define MCU_GPIO_INTTYPE_SET (MCU_GPIO_REG_BASE + 0x28)
#define MCU_GPIO_INTTYPE_CLR (MCU_GPIO_REG_BASE + 0x2C)
#define MCU_GPIO_INTPOL_SET (MCU_GPIO_REG_BASE + 0x30)
#define MCU_GPIO_INTPOL_CLR (MCU_GPIO_REG_BASE + 0x34)
#define MCU_GPIO_INT_STATUS (MCU_GPIO_REG_BASE + 0x38)

/* MCU TIMER  */
#define MCU_APB_DUALTIMER1_TIMER0_BASE      (0x40002000UL)
#define MCU_APB_DUALTIMER1_TIMER0_LOAD      (MCU_APB_DUALTIMER1_TIMER0_BASE + 0x00)
#define MCU_APB_DUALTIMER1_TIMER0_VALUE     (MCU_APB_DUALTIMER1_TIMER0_BASE + 0x04)
#define MCU_APB_DUALTIMER1_TIMER0_CTRL      (MCU_APB_DUALTIMER1_TIMER0_BASE + 0x08)
#define MCU_APB_DUALTIMER1_TIMER0_INTCLR    (MCU_APB_DUALTIMER1_TIMER0_BASE + 0x0C)
#define MCU_APB_DUALTIMER1_TIMER0_MIS       (MCU_APB_DUALTIMER1_TIMER0_BASE + 0x14)
#define MCU_APB_DUALTIMER1_TIMER0_BGLOAD    (MCU_APB_DUALTIMER1_TIMER0_BASE + 0x18)

#define MCU_APB_DUALTIMER1_TIMER1_BASE      (0x40002020UL)
#define MCU_APB_DUALTIMER1_TIMER1_LOAD      (MCU_APB_DUALTIMER1_TIMER1_BASE + 0x00)
#define MCU_APB_DUALTIMER1_TIMER1_VALUE     (MCU_APB_DUALTIMER1_TIMER1_BASE + 0x04)
#define MCU_APB_DUALTIMER1_TIMER1_CTRL      (MCU_APB_DUALTIMER1_TIMER1_BASE + 0x08)
#define MCU_APB_DUALTIMER1_TIMER1_INTCLR    (MCU_APB_DUALTIMER1_TIMER1_BASE + 0x0C)
#define MCU_APB_DUALTIMER1_TIMER1_MIS       (MCU_APB_DUALTIMER1_TIMER1_BASE + 0x14)
#define MCU_APB_DUALTIMER1_TIMER1_BGLOAD    (MCU_APB_DUALTIMER1_TIMER1_BASE + 0x18)

#define MCU_APB_TIMER_ENABLE        (0x00000080UL)
#define MCU_APB_TIMER_INT_CLEAR     (0x00000001UL)
#define MCU_APB_TIMER_CONTROL_SIZE  (0x00000002UL)
#define MCU_APB_TIMER_CONTROL_MODE  (0x00000040UL)
#define MCU_APB_TIMER_CONTROL_INTEN (0x00000020UL)

/* MCU UART */
#define MCU_APB_UART0_BASE                  (0x40004000UL)
#define MCU_APB_UART0_DATA            (MCU_APB_UART0_BASE + 0x00)
#define MCU_APB_UART0_STATE           (MCU_APB_UART0_BASE + 0x04)
#define MCU_APB_UART0_CTRL            (MCU_APB_UART0_BASE + 0x08)
#define MCU_APB_UART0_INTSTATCLR      (MCU_APB_UART0_BASE + 0x0C)
#define MCU_APB_UART0_BAUDDIV         (MCU_APB_UART0_BASE + 0x10)

#define MCU_APB_UART_RX_IRQ_ENABLE    (0x00000008UL)
#define MCU_APB_UART_TX_IRQ_ENABLE    (0x00000004UL)
#define MCU_APB_UART_RX_ENABLE        (0x00000002UL)
#define MCU_APB_UART_TX_ENABLE        (0x00000001UL)
#define MCU_APB_UART_TX_INTCLR        (0x00000001UL)
#define MCU_APB_UART_RX_INTCLR        (0x00000002UL)

#define INT_VECTOR_0 (16u)
#define INT_VECTOR_1 (17u)
#define INT_VECTOR_2 (18u)
#define INT_VECTOR_PORT1 (7u)
#define INT_VECTOR_DUALTIMER (10u)

#define NDP_GPIO0_BIT 0x00000001UL
#define NDP_GPIO1_BIT 0x00000002UL
#define NDP_GPIO2_BIT 0x00000004UL
#define NDP_GPIO3_BIT 0x00000008UL
#define NDP_GPIO4_BIT 0x00000010UL
#define NDP_GPIO5_BIT 0x00000020UL
#define NDP_GPIO6_BIT 0x00000040UL
#define NDP_GPIO7_BIT 0x00000080UL

/**
 * @brief Data Structure for storing UART BB state.
 *
 */
#define M301_READ_RES 0
#define M301_COMMAND  1
#define M301_REG      2
#define M301_DATA     3

struct ndp10x_uart_bb_state_s {
  uint32_t gpio_tx;                   /* integer 0-7 */
  uint32_t gpio_rx;                   /* integer 0-7 */
  uint32_t timer_used;
  uint32_t timer_int_used;
  uint32_t baud_rate;
  uint32_t freq_div;                  /* divider for 4 M clock */
  uint32_t current_state;
  uint8_t m301_message[4];
  uint32_t avd_on_off;
  uint32_t avd_low_power_div;
  uint32_t avd_full_power_div;
};

/**
 * @brief enable/disables the interrupt on MCU
 *
 * @param irq interrupt number
 */
void ndp10x_mcu_interrupt_set(uint32_t irq);

/**
 * @brief clear the interrupt on MCU
 *
 * @param irq interrupt number
 */
void ndp10x_mcu_interrupt_clear(uint32_t irq);

/**
 * @brief checks if a interrupt is pending on MCU
 *
 * @param irq interrupt number
 * @return uint32_t  0 if Interrupt is not pending else 1.
 */
uint32_t ndp10x_mcu_interrupt_get_pending(uint32_t irq);

/**
 * @brief clears the pending interrupt on MCU
 *
 * @param irq interrupt number
 *
 */
void ndp10x_mcu_interrupt_clear_pending(uint32_t irq);

/**
 * @brief enable pin for gpio
 *
 * @param value value of gpio to be set
 * @param gpio_outputs gpios that are ouputs, 0 if none
 * @param gpio_inputs gpios that are inputs, 0 if none
 *
 */
void ndp10x_mcu_gpio_enable(uint32_t value, uint32_t gpio_outputs, uint32_t gpio_inputs);

/**
 * @brief clear a value on gpio
 *
 * @param value value of gpio to be clear
 * 
 */
void ndp10x_mcu_gpio_clr(uint32_t value);

/**
 * @brief read gpio input register
 *
 * @param value value of gpio to be read
 * @return uint32_t gpio input values
 * 
 */
uint32_t ndp10x_mcu_gpio_read(uint32_t value);

/**
 * @brief set a value on gpio
 *
 * @param value value of gpio to be set
 * 
 */
void ndp10x_mcu_gpio_set(uint32_t value);

void ndp10x_mcu_gpio_intenable(uint32_t value, uint32_t edge, uint32_t polarity);
void ndp10x_mcu_gpio_intdisable(uint32_t value);
uint32_t ndp10x_mcu_gpio_intstat(uint32_t value);
void ndp10x_mcu_gpio_intclr(uint32_t value);

/**
 * @brief spin delay
 *
 * @param delay delay for 500ns * delay
 * 
 */
void spin_delay(int delay);

/**
 * @brief Blocking wait for interrupt.
 *
 */

#define ndp10x_wait_for_interrupt() __asm volatile("wfi")

/**
 * @brief sets the Priority Mask register
 *
 * @param mask if 1 it prevents the activation of all exceptions with
 * configurable priority
 */


#define ndp10x_priority_mask_set(mask) __asm volatile("MSR primask, %0" : : "r"(mask) : "memory");

void ndp10x_mcu_uart_rx_init(uint32_t baud_rate_div);

void ndp10x_mcu_uart_init(uint32_t baud_rate_div, uint32_t tx_enable, \
                          uint32_t rx_enable, uint32_t tx_irq_en, uint32_t rx_irq_en);

void ndp10x_mcu_uart_rx_clr(void);

uint32_t ndp10x_mcu_uart_rx_status(void);

uint32_t ndp10x_mcu_uart_rx_data(void);

void ndp10x_mcu_uart_tx_clr(void);

uint32_t ndp10x_mcu_uart_tx_status(void);

void ndp10x_mcu_uart_tx_data(uint32_t);


/**
 * @brief UART over a normal GPIO TX function
 *
 * @param *uart_bb structure for uart bit bang
 * @param byte_to_send byte to transmit
 *
 */
void ndp10x_mcu_bb_uart_tx(struct ndp10x_uart_bb_state_s *uart_bb, uint8_t byte_to_send);

void ndp10x_mcu_timer_enable(uint32_t timer_num);

void ndp10x_mcu_timer_disable(uint32_t timer_num);

void ndp10x_mcu_timer_reset(uint32_t timer_num, uint32_t value);

void ndp10x_mcu_timer_intclr(uint32_t timer_num);

uint32_t  ndp10x_mcu_timer_value(uint32_t timer_num);

uint32_t ndp10x_mcu_timer_intstat(uint32_t timer_num);

void ndp10x_mcu_lowpower_wake(uint32_t full_power_pdm_clk_divider);

void ndp10x_mcu_lowpower_enable(uint32_t low_power_pdm_clk_divider);

#ifdef __cplusplus
}
#endif

#endif
