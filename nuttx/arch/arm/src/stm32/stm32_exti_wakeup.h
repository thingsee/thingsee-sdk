#ifndef STM32_EXTI_WAKEUP_H_
#define STM32_EXTI_WAKEUP_H_

xcpt_t stm32_exti_wakeup(bool risingedge, bool fallingedge, bool event,
                        xcpt_t func);

#endif /* STM32_EXTI_WAKEUP_H_ */
