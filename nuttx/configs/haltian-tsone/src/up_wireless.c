/************************************************************************************
 * configs/haltian-tsone/src/up_wireless.c
 *
 *   Copyright (C) 2009, 2013 Gregory Nutt. All rights reserved.
 *   Author: Laurent Latil <laurent@latil.nom.fr>
 *           David Sidrane <david_s5@nscdg.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <debug.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/spi/spi.h>
#include <nuttx/wireless/wireless.h>
#include <nuttx/wireless/cc3000.h>
#include <nuttx/wireless/cc3000/include/cc3000_upif.h>
#include <nuttx/wireless/cc3000/nvmem.h>
#include <nuttx/wireless/cc3000/include/sys/socket.h>
#include <nuttx/wireless/cc3000/wlan.h>
#include <nuttx/wireless/cc3000/hci.h>
#include <nuttx/wireless/cc3000/security.h>
#include <nuttx/wireless/cc3000/netapp.h>

#include <arch/board/board-pwrctl.h>
#include <arch/board/board-wireless.h>

#include "stm32.h"
#include "haltian-tsone.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#define NETAPP_IPCONFIG_IP_OFFSET (0)
#define DISABLE (0)

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) ({ \
                             if (cc3000_debug_printf) \
                               cc3000_debug_printf(__VA_ARGS__); \
                             else \
                               lowsyslog(LOG_DEBUG, __VA_ARGS__); \
                         })
#  else
#    define message(...) printf(__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message printf
#  else
#    define message printf
#  endif
#endif

/* Configuration ************************************************************/

#ifdef CONFIG_WL_CC3000
#ifndef CONFIG_WIRELESS
#  error "Wireless support requires CONFIG_WIRELESS"
#endif

#ifndef CONFIG_STM32_SPI1
#  error "CC3000 Wireless support requires CONFIG_STM32_SPI1"
#endif

#ifndef CONFIG_CC3000_SPI_FREQUENCY
#  define CONFIG_CC3000_SPI_FREQUENCY 16000000
#endif

#ifndef CONFIG_CC3000_SPI_MODE
#  define CONFIG_CC3000_SPI_MODE 1
#endif

#if CONFIG_CC3000_SPI_MODE != 1
#  error "SPI mode for CC3000 is '1'! (CPOL=0, CPHA=1)"
#endif

#ifndef CONFIG_CC3000_SPIDEV
#  define CONFIG_CC3000_SPIDEV 1
#endif

#if CONFIG_CC3000_SPIDEV != 1
#  error "CC3000_SPIDEV must be 1"
#endif

#ifndef CONFIG_CC3000_DEVMINOR
#  define CONFIG_CC3000_DEVMINOR 0
#endif

#ifndef CONFIG_CC3000_RX_BUFFER_SIZE
#define CONFIG_CC3000_RX_BUFFER_SIZE 132
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_config_s
{
  struct cc3000_config_s dev;
  xcpt_t handler;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind callbacks
 * to isolate the CC3000 driver from differences in GPIO interrupt handling
 * by varying boards and MCUs.  If possible, interrupts should be configured
 * on falling edges to detect the Ready Condition At T2: The normal master
 * SPI write sequence is SPI_CS low, followed by SPI_IRQ low CC3000 to host,
 * indicating that the CC3000 core module is ready to accept data. T2
 * duration is approximately 7 ms.
 *
 *   irq_attach         - Attach the CC3000 interrupt handler to the GPIO interrupt
 *   irq_enable         - Enable or disable the GPIO interrupt
 *   clear_irq          - Acknowledge/clear any pending GPIO interrupt
 *   power_enable       - Enable or disable Module enable.
 *   chip_chip_select   - The Chip Select
 *   wl_read_irq        - Return the state of the interrupt GPIO input
 */

static int  wl_attach_irq(FAR struct cc3000_config_s *state, xcpt_t handler);
static void wl_enable_irq(FAR struct cc3000_config_s *state, bool enable);
static void wl_clear_irq(FAR struct cc3000_config_s *state);
static void wl_select(FAR struct cc3000_config_s *state, bool enable);
static void wl_enable_power(FAR struct cc3000_config_s *state, bool enable);
static bool wl_read_irq(FAR struct cc3000_config_s *state);
#ifdef CONFIG_CC3000_PROBES
static bool probe(FAR struct cc3000_config_s *state,int n, bool s);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the CC3000
 * driver.  This structure provides information about the configuration
 * of the CC3000 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writable because, under certain circumstances, the driver
 * may modify frequency or X plate resistance values.
 */

static struct stm32_config_s g_cc3000_info =
{
  .dev.spi_frequency    = CONFIG_CC3000_SPI_FREQUENCY,
  .dev.spi_mode         = CONFIG_CC3000_SPI_MODE,
  .dev.max_rx_size      = 0,
  .dev.irq_attach       = wl_attach_irq,
  .dev.irq_enable       = wl_enable_irq,
  .dev.irq_clear        = wl_clear_irq,
  .dev.power_enable     = wl_enable_power,
  .dev.chip_chip_select = wl_select,
  .dev.irq_read         = wl_read_irq,
#ifdef CONFIG_CC3000_PROBES
  .dev.probe            = probe, /* This is used for debugging */
#endif
  .handler              = NULL,
};

static bool is_initialized = false;
static bool cc3000_is_connected = false;
static bool cc3000_has_ipaddress = false;
static sem_t sem_cc3000_event;
static int (*cc3000_debug_printf)(const char *fmt, ...);
static void (*cc3000_user_async_callback)(long event_type, char *data,
    uint8_t length);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the CC3000 driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 * attach  - Attach the CC3000 interrupt handler to the GPIO interrupt
 * enable  - Enable or disable the GPIO interrupt
 * clear   - Acknowledge/clear any pending GPIO interrupt
 * pendown - Return the state of the pen down GPIO input
 */

static int wl_attach_irq(FAR struct cc3000_config_s *state, xcpt_t handler)
{
  FAR struct stm32_config_s *priv = (FAR struct stm32_config_s *)state;

  /* Just save the handler for use when the interrupt is enabled */

  priv->handler = handler;
  return OK;
}

static void wl_enable_irq(FAR struct cc3000_config_s *state, bool enable)
{
  FAR struct stm32_config_s *priv = (FAR struct stm32_config_s *)state;

  /* The caller should not attempt to enable interrupts if the handler
   * has not yet been 'attached'
   */

  DEBUGASSERT(priv->handler || !enable);

  /* Attach and enable, or detach and disable */

  ivdbg("enable:%d\n", enable);
  if (enable)
    {
      (void)stm32_gpiosetevent(GPIO_WIFI_INT, false, true, false, priv->handler);
    }
  else
    {
      (void)stm32_gpiosetevent(GPIO_WIFI_INT, false, false, false, NULL);
    }
}

static void wl_enable_power(FAR struct cc3000_config_s *state, bool enable)
{
  ivdbg("enable:%d\n", enable);

  /* Active high enable */

  stm32_gpiowrite(GPIO_WIFI_EN, enable);
}

static void wl_select(FAR struct cc3000_config_s *state, bool enable)
{
  /* not used. */
  for (;;);
}

static void wl_clear_irq(FAR struct cc3000_config_s *state)
{
  /* Does nothing */
}

static bool wl_read_irq(FAR struct cc3000_config_s *state)
{
  /* Active low*/

  return stm32_gpioread(GPIO_WIFI_INT) ? false : true;
}

#ifdef CONFIG_CC3000_PROBES
static bool probe(FAR struct cc3000_config_s *state,int n, bool s)
{
  if (n == 0)
    {
      stm32_gpiowrite(GPIO_D0, s);
    }

  if (n == 1)
    {
      stm32_gpiowrite(GPIO_D1, s);
    }

  return true;
}
#endif

/****************************************************************************
 * Name: arch_wlinitialize
 *
 * Description:
 *   Each board that supports a wireless device must provide this function.
 *   This function is called by application-specific, setup logic to
 *   configure the wireless device.  This function will register the driver
 *   as /dev/wirelessN where N is the minor device number.
 *
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int wireless_archinitialize(size_t max_rx_size)
{
  FAR struct spi_dev_s *spi;
  static bool once = false;

  if (once)
    return OK;

  sem_init(&sem_cc3000_event, 0, 0);

  /* Init SPI bus */

  idbg("minor %d\n", minor);
  DEBUGASSERT(CONFIG_CC3000_DEVMINOR == 0);

#ifdef CONFIG_CC3000_PROBES
  stm32_configgpio(GPIO_D0);
  stm32_configgpio(GPIO_D1);
  stm32_gpiowrite(GPIO_D0, 1);
  stm32_gpiowrite(GPIO_D1, 1);
#endif

  /* Configure CC3000 EN & IRQ line IOs */

  stm32_configgpio(GPIO_WIFI_EN);
  stm32_configgpio(GPIO_WIFI_INT);

  /* Get an instance of the SPI interface */

  spi = up_spiinitialize(CONFIG_CC3000_SPIDEV);
  if (!spi)
    {
      idbg("Failed to initialize SPI bus %d\n", CONFIG_CC3000_SPIDEV);
      return -ENODEV;
    }

  /* Initialize and register the SPI CC3000 device */
  g_cc3000_info.dev.max_rx_size = max_rx_size ? max_rx_size : CONFIG_CC3000_RX_BUFFER_SIZE;
  int ret = cc3000_register(spi, &g_cc3000_info.dev, CONFIG_CC3000_DEVMINOR);
  if (ret < 0)
    {
      idbg("Failed to initialize SPI bus %d\n", CONFIG_CC3000_SPIDEV);
      return -ENODEV;
    }

  once = true;

  return OK;
}

/*****************************************************************************
 * Name: C3000_wlan_init
 *
 * Description:
 *   Initialize wlan driver
 *
 *   Warning: This function must be called before ANY other wlan driver
 *   function
 *
 * Input Parmeters:
 *   sWlanCB   Asynchronous events callback.
 *             0 no event call back.
 *             - Call back parameters:
 *               1) event_type: HCI_EVNT_WLAN_UNSOL_CONNECT connect event,
 *                  HCI_EVNT_WLAN_UNSOL_DISCONNECT disconnect event,
 *                  HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE config done,
 *                  HCI_EVNT_WLAN_UNSOL_DHCP dhcp report,
 *                  HCI_EVNT_WLAN_ASYNC_PING_REPORT ping report OR
 *                  HCI_EVNT_WLAN_KEEPALIVE keepalive.
 *               2) data: pointer to extra data that received by the event
 *                  (NULL no data).
 *               3) length: data length.
 *              - Events with extra data:
 *                  HCI_EVNT_WLAN_UNSOL_DHCP: 4 bytes IP, 4 bytes Mask,
 *                    4 bytes default gateway, 4 bytes DHCP server and 4 bytes
 *                    for DNS server.
 *                  HCI_EVNT_WLAN_ASYNC_PING_REPORT: 4 bytes Packets sent,
 *                    4 bytes Packets received, 4 bytes Min round time,
 *                    4 bytes Max round time and 4 bytes for Avg round time.
 *
 *     sFWPatches  0 no patch or pointer to FW patches
 *     sDriverPatches  0 no patch or pointer to driver patches
 *     sBootLoaderPatches  0 no patch or pointer to bootloader patches
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void cc3000_wlan_init(size_t max_tx_len,
                      tWlanCB sWlanCB,
                      tFWPatches sFWPatches, tDriverPatches
                      sDriverPatches,  tBootLoaderPatches sBootLoaderPatches)
{
  wlan_init(max_tx_len, sWlanCB, sFWPatches, sDriverPatches, sBootLoaderPatches);
}

/* CC3000 Driver callbacks */

/*
 * The TI library calls this routine when asynchronous events happen.
 *
 *   For example you tell the CC3000 to turn itself on and connect
 *   to an access point then your code can go on to do its own thing.
 *   When the CC3000 is done configuring itself (e.g. it gets an IP
 *   address from the DHCP server) it will call this routine so you
 *   can take appropriate action.
 */

static void cc3000_async_callback(long event_type, char *data, uint8_t length)
{
  bool event_wakeup = false;

  if (event_type != HCI_EVNT_WLAN_UNSOL_CONNECT &&
      event_type != HCI_EVNT_WLAN_UNSOL_DISCONNECT &&
      event_type != HCI_EVNT_WLAN_UNSOL_DHCP)
    goto skip;

  switch (event_type)
    {
    case HCI_EVNT_WLAN_UNSOL_CONNECT:
      cc3000_is_connected = true;
      event_wakeup = true;
      break;

    case HCI_EVNT_WLAN_UNSOL_DISCONNECT:
      cc3000_is_connected = false;
      cc3000_has_ipaddress = false;
      event_wakeup = true;
      break;

    case HCI_EVNT_WLAN_UNSOL_DHCP:
      /* Notes:
       * 1) IP config parameters are received swapped
       * 2) IP config parameters are valid only if status is OK, i.e. ulCC3000DHCP becomes 1
       * only if status is OK, the flag is set to 1 and the addresses are valid
       */

      if (*(uint32_t *)(data + NETAPP_IPCONFIG_IP_OFFSET) != 0)
        {
          cc3000_has_ipaddress = true;
          event_wakeup = true;
        }
      else
        {
          cc3000_has_ipaddress = false;
          event_wakeup = true;
        }
      break;

    default:
      break;
    }

skip:
  if (cc3000_user_async_callback)
    cc3000_user_async_callback(event_type, data, length);

  if (event_wakeup)
    sem_post(&sem_cc3000_event);
}

/* The TI library calls these routines on CC3000 startup.
 *
 *  This library does not send firmware, driver, or bootloader patches
 *  so we do nothing and we return NULL.
 */

static char *cc3000_send_firmware_patch(unsigned long *length)
{
  *length = 0;
  return NULL;
}

static char *cc3000_send_driver_patch(unsigned long *length)
{
  *length = 0;
  return NULL;
}

static char *cc3000_send_bootloader_patch(unsigned long *length)
{
  *length = 0;
  return NULL;
}

/*****************************************************************************
 * Name: wait_cc3000_event
 *
 * Description:
 *   Wait for event from CC3000 driver through cc3000_async_callback
 *
 * Input Parameters:
 *   timeout_msecs   Relative timeout value in milliseconds.
 *   event_flag      Pointer to bool variable that is checked for expected value.
 *   wait_for_value  Expected value for *event_flag.
 *
 * Returned Value:
 *   Return true if wait completed, and false if timed-out or catch error.
 *
 ****************************************************************************/

static bool wait_cc3000_event(uint32_t timeout_msecs, volatile bool *event_flag,
                              const bool wait_for_value,
                              bool (*stopfn)(void *priv),
                              void *priv)
{
  struct timespec abstime;
  int ret;
  uint32_t curr_msecs;

  /* Get the current time */

  (void)clock_gettime(CLOCK_REALTIME, &abstime);

  while (timeout_msecs > 0)
    {
      curr_msecs = timeout_msecs;
      if (curr_msecs > 200)
        curr_msecs = 200;
      timeout_msecs -= curr_msecs;

      /* Setup absolute timeout time */

      while (curr_msecs >= 1000)
        {
          curr_msecs -= 1000;
          abstime.tv_sec++;
        }
      abstime.tv_nsec += curr_msecs * 1000 * 1000;
      if (abstime.tv_nsec >= 1000 * 1000 * 1000)
        {
          abstime.tv_sec++;
          abstime.tv_nsec -= 1000 * 1000 * 1000;
        }

      while (*event_flag != wait_for_value)
        {
          if (stopfn && stopfn(priv))
            {
              errno = EINTR;
              return false;
            }

          ret = sem_timedwait(&sem_cc3000_event, &abstime);
          if (ret != OK)
            {
              if (errno == EINTR)
                {
                  return false;
                }

              DEBUGASSERT(errno == ETIMEDOUT);
              break;
            }
        }

      if (*event_flag == wait_for_value)
        break;
    }

  if (*event_flag == wait_for_value)
    {
      return true;
    }

  errno = ETIMEDOUT;
  return false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*****************************************************************************
 * Name: cc3000_set_power
 *
 * Description:
 *   Set power ON/OFF for CC3000 module
 *
 ****************************************************************************/

void cc3000_set_power(bool on)
{
  if (on)
    {
      board_pwrctl_get(PWRCTL_REGULATOR_WLAN);
      usleep(250 * 1000);
    }
  else
    {
      wl_enable_irq(&g_cc3000_info.dev, false);
      stm32_unconfiggpio(GPIO_WIFI_INT);
      board_pwrctl_put(PWRCTL_REGULATOR_WLAN);
    }
}

/*****************************************************************************
 * Name: cc3000_disconnect_from_accesspoint
 *
 * Description:
 *   Disconnect form access-point.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   True when successfully executed
 *
 ****************************************************************************/

bool cc3000_disconnect_from_accesspoint(void)
{
  wlan_disconnect();
  return true;
}

/*****************************************************************************
 * Name: cc3000_connect_to_accesspoint_with_stopfn
 *
 * Description:
 *   Attempt connection to access-point.
 *
 * Input Parameters:
 *   security Security type for accesspoint
 *   ssid     SSID for AP
 *   wpa_key  Plaintext WPA2-PSK key for AP
 *
 * Returned Value:
 *   True when successfully executed
 *
 ****************************************************************************/

bool cc3000_connect_to_accesspoint_with_stopfn(
    int security, const char *ssid, const char *key, unsigned int timeout_msec,
    int flags, bool (*stopfn)(void *priv), void *priv)
{
  int ssid_len = ssid ? strlen(ssid) : 0;
  int key_len = key ? strlen(key) : 0;
  const uint8_t *key_unsigned = (const uint8_t *)key;
  bool ret;
  uint8_t rval;

  message("CC3000: Preparing connection to SSID:'%s' ...\n", ssid);

  if (!is_initialized)
    {
      message("  CC3000 not initialized; can't run connect.\n");
      return false;
    }

  message("  Disabling auto-connect policy...\n");
  (void)wlan_ioctl_set_connection_policy(DISABLE, DISABLE, DISABLE);

  message("  Deleting all existing profiles...\n");
  (void)wlan_ioctl_del_profile(255);

  message("  Waiting until disconnected...\n");

  ret = wait_cc3000_event(timeout_msec /*msec*/, &cc3000_is_connected, false,
                          stopfn, priv);
  if (!ret)
    {
      message(" FAILED: %s!\n", (errno == EINTR) ? "Got signal" : "Timeout");
      return false;
    }
  else
    {
      message(" Succeed\n");
    }

  message("  Connecting...\n");

  rval = wlan_connect(security,
        ssid, ssid_len,
        NULL,
        key_unsigned, key_len);

  if (rval == 0)
    {
      message("  Connect success.\n");
    }
  else
    {
      message("  Unusual return value: %d\n", rval);
      return false;
    }

  message("  Waiting for connection...\n");

  ret = wait_cc3000_event(timeout_msec /*msec*/, &cc3000_is_connected, true,
                          stopfn, priv);
  if (!ret)
    {
      if (errno == EINTR)
        message("    Wait cancelled!\n");
      else
        message("    Wait timed out!\n");
      return false;
    }
  else
    {
      message(" Succeed\n");
    }

  if (flags & CC3000_CONN_FLG_NO_IP_ADDR_WAIT)
    {
      return true;
    }

  message("  Waiting for IP address...\n");

  ret = wait_cc3000_event(timeout_msec /*msec*/, &cc3000_has_ipaddress, true,
                          stopfn, priv);
  if (!ret)
    {
      if (errno == EINTR)
        message("    Wait cancelled!\n");
      else
        message("    Wait timed out!\n");
      return false;
    }
  else
    {
      message(" Succeed\n");
      return true;
    }
}

/*****************************************************************************
 * Name: cc3000_uninitialize
 *
 * Description:
 *   Uniitialize CC3000 wifi module
 *
 * Returned Value:
 *   True when successfully executed
 *
 ****************************************************************************/

bool cc3000_uninitialize(void)
{
  if (!is_initialized)
    return true;

  message("Stopping CC3000 WiFi module...\n");

  /* If already initialized, stop wifi module first. */
  wlan_stop();
  sleep(1);
  is_initialized = false;

  /* Power off WiFi module. */

  cc3000_set_power(false);

  cc3000_is_connected = false;
  cc3000_has_ipaddress = false;
  cc3000_user_async_callback = NULL;
  cc3000_debug_printf = NULL;

  return true;
}

/*****************************************************************************
 * Name: cc3000_initialize
 *
 * Description:
 *   Initialize CC3000 wifi module (hw, then sw)
 *
 * Returned Value:
 *   True when successfully executed
 *
 ****************************************************************************/

bool cc3000_initialize(int (*debugout_fn)(const char *fmt, ...),
                       void (*async_callback)(long event_type, char *data, uint8_t length))
{
  uint8_t buf[MAC_ADDR_LEN] = {};

  cc3000_debug_printf = debugout_fn;

  if (is_initialized)
    {
      cc3000_uninitialize();
      is_initialized = false;
    }

  message("Initializing CC3000 WiFi module...\n");

  cc3000_set_power(true);

  cc3000_user_async_callback = async_callback;

  cc3000_wlan_init(CONFIG_CC3000_RX_BUFFER_SIZE, cc3000_async_callback,
      cc3000_send_firmware_patch,
      cc3000_send_driver_patch,
      cc3000_send_bootloader_patch);

  wlan_start(0);

  if (nvmem_read_sp_version(buf) == 0)
    {
      message("  CC3000 Firmware version is: ");
      message("%d", buf[0]);
      message(".");
      message("%d\n", buf[1]);
    }
  else
    {
      message("  Unable to get firmware version. Can't continue.\n");
      return false;
    }

  is_initialized = true;

  if (nvmem_get_mac_address(buf) == MAC_ADDR_LEN)
    {
      message("  CC3000 MAC address: %02X:%02X:%02X:%02X:%02X:%02X\n",
              buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
    }
  else
    {
      message("  Unable to get MAC address for CC3000!\n");
    }

  return true;
}

/*****************************************************************************
 * Name: nsh_cc3000_start
 *
 * Description:
 *   Initialize CC3000 WiFi module.
 *
 ****************************************************************************/

void nsh_cc3000_start(void)
{
  wireless_archinitialize(CONFIG_CC3000_RX_BUFFER_SIZE);
}

#endif /* CONFIG_WL_CC3000 */
