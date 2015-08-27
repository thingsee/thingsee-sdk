/*
 * Since Cortex-M does not have a real DCC, we use this protocol through a debug
 * register for talking to target/cortex_m.c:cortex_m_dcc_read() in OpenOCD sources.
 *
 * Use 'target_request debugmsgs charmsg' to enable.
 */

#include <nuttx/config.h>
#include <nuttx/irq.h>

#include "stm32_dcc.h"

#ifdef CONFIG_DEBUG_DCC

/* Cortex-M Debug Core Register Data Register */
#define DCB_DCRDR_REG (*(volatile unsigned short *)0xE000EDF8)

/* Set this from debugger to enable dcc_putc() output */
volatile int dcc_enable_debug = 0;

void dcc_init(void)
{
  /* Use explicit assignment so we can use this before BSS initialization */

  dcc_enable_debug = 0;
}

int dcc_putc(unsigned int c)
{
  irqstate_t saved_state;
  unsigned char buf[] = { c & 0xff, 0, 0, 0 };
  int i;

  if (dcc_enable_debug)
    {
      saved_state = irqsave();
      for (i = 0; i < 4; i++)
        {
          while (DCB_DCRDR_REG & 1)
              ;
          DCB_DCRDR_REG = (unsigned short)((buf[i] << 8) | 1);
        }
      irqrestore(saved_state);
    }
  return (unsigned char)c;
}

int dcc_getc(void)
{
  return -1;
}

#endif /* CONFIG_DEBUG_DCC */
