/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <arch/irq.h>

#include "up_arch.h"
#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_exti.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Interrupt handlers attached to the ALARM EXTI */

static xcpt_t stm32_exti_callback;

/****************************************************************************
 * Public Data
 ****************************************************************************/

 /****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_exti_wakeup_isr
 *
 * Description:
 *   EXTI WAKEUP interrupt service routine/dispatcher
 *
 ****************************************************************************/

static int stm32_exti_wakeup_isr(int irq, void *context)
{
  int ret = OK;

  /* Clear the pending EXTI interrupt */

  putreg32(EXTI_RTC_WAKEUP, STM32_EXTI_PR);

  /* And dispatch the interrupt to the handler */

  if (stm32_exti_callback)
    {
      ret = stm32_exti_callback(irq, context);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_exti_wakeup
 *
 * Description:
 *   Sets/clears EXTI wakeup interrupt.
 *
 * Parameters:
 *  - rising/falling edge: enables interrupt on rising/falling edget
 *  - event:  generate event when set
 *  - func:   when non-NULL, generate interrupt
 *
 * Returns:
 *   The previous value of the interrupt handler function pointer.  This
 *   value may, for example, be used to restore the previous handler when
 *   multiple handlers are used.
 *
 ****************************************************************************/

xcpt_t stm32_exti_wakeup(bool risingedge, bool fallingedge, bool event,
                        xcpt_t func)
{
  xcpt_t oldhandler;

  /* Get the previous GPIO IRQ handler; Save the new IRQ handler. */

  oldhandler          = stm32_exti_callback;
  stm32_exti_callback = func;

  /* Install external interrupt handlers (if not already attached) */

  if (func)
    {
      irq_attach(STM32_IRQ_RTC_WKUP, stm32_exti_wakeup_isr);
      up_enable_irq(STM32_IRQ_RTC_WKUP);
    }
  else
    {
      up_disable_irq(STM32_IRQ_RTC_WKUP);
    }

  /* Configure rising/falling edges */

  modifyreg32(STM32_EXTI_RTSR,
              risingedge ? 0 : EXTI_RTC_WAKEUP,
              risingedge ? EXTI_RTC_WAKEUP : 0);
  modifyreg32(STM32_EXTI_FTSR,
              fallingedge ? 0 : EXTI_RTC_WAKEUP,
              fallingedge ? EXTI_RTC_WAKEUP : 0);

  /* Enable Events and Interrupts */

  modifyreg32(STM32_EXTI_EMR,
              event ? 0 : EXTI_RTC_WAKEUP,
              event ? EXTI_RTC_WAKEUP : 0);
  modifyreg32(STM32_EXTI_IMR,
              func ? 0 : EXTI_RTC_WAKEUP,
              func ? EXTI_RTC_WAKEUP : 0);

  /* Return the old IRQ handler */

  return oldhandler;
}
