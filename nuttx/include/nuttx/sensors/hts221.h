/****************************************************************************
 * include/nuttx/sensors/hts221.h
 *
 *   Copyright (C) 2014 Haltian Ltd. All rights reserved.
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
 ****************************************************************************/

#ifndef HTS221_H_
#define HTS221_H_

#define HTS221_TEMPERATURE_PRECISION  100
#define HTS221_HUMIDITY_PRECISION     10

typedef enum hts221_ioc_cmds
  {
    HTS221_IOC_GET_DEV_ID = 0x0,
    HTS221_IOC_CFGR,
    HTS221_IOC_START_CONVERSION,
    HTS221_IOC_CHECK_STATUS_REG,
    HTS221_IOC_READ_RAW_DATA,
    HTS221_IOC_READ_CONVERT_DATA,
#if CONFIG_TRACES_HTS221_HUMIDITY
    HTS221_IOC_DUMP_REGS,
#endif
  } hts221_ioc_cmds;

/* Number of temperature samples */

typedef enum hts221_avrg_temp
  {
    HTS221_AVGT2 = 0x0,
    HTS221_AVGT4,
    HTS221_AVGT8,
    HTS221_AVGT16,              /* Default value */
    HTS221_AVGT32,
    HTS221_AVGT64,
    HTS221_AVGT128,
    HTS221_AVGT256
  } hts221_avrg_temp;

/* Number of humidity samples */

typedef enum hts221_avrg_humid
  {
    HTS221_AVGH4 = 0x0,
    HTS221_AVGH8,
    HTS221_AVGH16,
    HTS221_AVGH32,              /* Default value */
    HTS221_AVGH64,
    HTS221_AVGH128,
    HTS221_AVGH256,
    HTS221_AVGH512
  } hts221_avrg_humid;

/* Output data rate configuration */
typedef enum hts221_odr_t
  {
    HTS221_ODR_ONESHOT = 0x0,
    HTS221_ODR_1HZ,
    HTS221_ODR_7HZ,
    HTS221_ODR_12_5HZ
  } hts221_odr_t;

/* Configuration structure */

typedef struct hts221_settings_t
  {
    hts221_avrg_temp temp_resol;        /* Temperature resolution. The more
                                         * samples sensor takes, the more power
                                         * it uses */
    hts221_avrg_humid humid_resol;      /* Humidity resolution. The more
                                         * samples sensor takes, the more power
                                         * it uses */
    hts221_odr_t odr;           /* Output data rate */
    bool is_bdu;                /* If read operation is not faster than output
                                 * operation, then this variable must be set to true */
    bool is_data_rdy;           /* Must be set to true, if interrupt needed.
                                 * Default is 0, disabled */
    bool is_high_edge;          /* High or low interrupt signal from device.
                                 * Default is high, 0 */
    bool is_open_drain;         /* Open drain or push-pull on data-ready pin.
                                 * Default is push-pull, 0 */
    bool is_boot;               /* Refresh the content of the internal registers */
  } hts221_settings_t;

/* Interrupt configuration data structure */

typedef struct hts221_config_t
  {
    int irq;
    int (*irq_attach) (FAR struct hts221_config_t * state, xcpt_t isr);
    void (*irq_enable) (FAR struct hts221_config_t * state, bool enable);
    void (*irq_clear) (FAR struct hts221_config_t * state);
  } hts221_config_t;

/* Raw data structure */

typedef struct hts221_raw_data_t
  {
    uint8_t humid_low_bits;
    uint8_t humid_high_bits;
    uint8_t temp_low_bits;
    uint8_t temp_high_bits;
  } hts221_raw_data_t;

typedef struct hts221_conv_data_t
  {
    int temperature;
    unsigned int humidity;
  } hts221_conv_data_t;

/* Status register data */

typedef struct hts221_status_t
  {
    bool is_humid_ready;
    bool is_temp_ready;
  } hts221_status_t;

int hts221_register(FAR const char *devpath, FAR struct i2c_dev_s *i2c,
                    uint8_t addr, hts221_config_t * config);

#endif /* HTS221_H_ */
