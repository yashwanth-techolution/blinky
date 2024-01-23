/*
 * SYNTIANT CONFIDENTIAL
 *
 * _____________________
 *
 * Copyright (c) 2017-2018 Syntiant Corporation
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

#ifndef __NDP120_MCU_H__
#define __NDP120_MCU_H__


typedef enum IRQn
{
  /* internal interrupt */
  NMI_IRQn                      = -14,
  HardFault_IRQn                = -13,
  SVCall_IRQn                   = -5,
  PendSV_IRQn                   = -2,
  SysTick_IRQn                  = -1,

  /* external interrupt*/
  UARTRX0_IRQn                  = 0,
  UARTTX0_IRQn                  = 1,
  UARTRX1_IRQn                  = 2,
  UARTTX1_IRQn                  = 3,
  UARTRX2_IRQn                  = 4,
  UARTTX2_IRQn                  = 5,
  PORT0_COMB_IRQn               = 6,
  PORT1_COMB_IRQn               = 7,
  TIMER0_IRQn                   = 8,
  TIMER1_IRQn                   = 9,
  DUALTIMER_IRQn                = 10,
  UARTOVF_IRQn                  = 12,
  MBOUT_IRQn                    = 15,
  MBIN_IRQn                     = 16,
  FREQ_IRQn                     = 17,
  DNN_IRQn                      = 18,
  MATCH_IRQn                    = 19,
  WM_IRQn                       = 20,
  MSPITX_IRQn                   = 21,
  DSP2MCU_IRQn                  = 23,
} IRQn_Type;


/**
 * @brief enable/disables the interrupt on MCU
 *
 * @param irq interrupt number
 */
void ndp_mcu_interrupt_set(uint32_t irq);

/**
 * @brief clear the interrupt on MCU
 *
 * @param irq interrupt number
 */
void ndp_mcu_interrupt_clear(uint32_t irq);

/**
 * @brief checks if a interrupt is pending on MCU
 *
 * @param irq interrupt number
 * @return uint32_t  0 if Interrupt is not pending else 1.
 */
uint32_t ndp_mcu_interrupt_get_pending(uint32_t irq);

/**
 * @brief clears the pending interrupt on MCU
 *
 * @param irq interrupt number
 *
 */
void ndp_mcu_interrupt_clear_pending(uint32_t irq);

/**
 * @brief set GPIO input/output direction
 *
 * @param gpio_pin    GPIO pin number
 * @param out_enable  1: output enable, 0:input enable
 */
void
ndp_mcu_gpio_set_dir(uint32_t gpio_pin, uint32_t out_enable);


/**
 * @brief set GPIO value (for output pin)
 *
 * @param gpio_pin  GPIO pin number
 * @param val       output value
 */
void
ndp_mcu_gpio_set(uint32_t gpio_pin, uint32_t val);



/**
 * @brief Blocking wait for interrupt.
 *
 */
inline void
ndp_wait_for_interrupt(void)
{
    __asm volatile("wfi");
}

/**
 * @brief sets the Priority Mask register
 *
 * @param mask if 1 it prevents the activation of all exceptions with
 * configurable priority
 */
inline void
ndp_priority_mask_set(uint32_t mask)
{
    __asm volatile("MSR primask, %0" : : "r"(mask) : "memory");
}

#endif