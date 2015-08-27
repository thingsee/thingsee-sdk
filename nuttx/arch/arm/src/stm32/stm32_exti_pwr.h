#ifndef STM32_EXTI_PWR_H_
#define STM32_EXTI_PWR_H_

/****************************************************************************
 * Name: stm32_exti_pvd
 *
 * Description:
 *   Sets/clears EXTI PVD interrupt.
 *
 * Parameters:
 *  - rising/falling edge: enables interrupt on rising/falling edge
 *  - event:  generate event when set
 *  - func:   when non-NULL, generate interrupt
 *
 * Returns:
 *   The previous value of the interrupt handler function pointer.  This
 *   value may, for example, be used to restore the previous handler when
 *   multiple handlers are used.
 *
 ****************************************************************************/

xcpt_t stm32_exti_pvd(bool risingedge, bool fallingedge, bool event,
                      xcpt_t func);

#endif /* STM32_EXTI_PWR_H_ */
