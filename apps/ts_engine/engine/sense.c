/****************************************************************************
 * apps/thingsee/engine/sense.c
 *
 * Copyright (C) 2014-2016 Haltian Ltd. All rights reserved.
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
 * Authors:
 *   Pekka Ervasti <pekka.ervasti@haltian.com>
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <assert.h>

#include <stdlib.h>
#include <stdio.h>

#include "eng_dbg.h"
#include "value.h"
#include "parse.h"
#include "sense.h"
#include "eng_error.h"

#ifndef CONFIG_ARCH_SIM
#include "sense_group_hw_keys.h"
#include "sense_group_acceleration.h"
#include "sense_group_orientation.h"
#include "sense_group_location.h"
#include "sense_group_energy.h"
#include "sense_group_time.h"
#include "sense_ambient_light.h"
#include "sense_hts221.h"
#include "sense_lps25h.h"
#else
#include "sense_sim.h"
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#define MAX_SENSES_PER_SENSOR 64
#define DEEP_SLEEP_RATE_LIMIT 1000

typedef int (*sense_read_t)(struct ts_cause *cause);

struct ts_sense_lookup
{
  dq_entry_t entry;

  const struct ts_sense_info *sense_info;
  uint8_t index;
};

struct ts_module
{
  const char *id;
  const char *name;

  int number_of_senses;
  const struct ts_sense_info *sense_info;
};

#ifndef CONFIG_ARCH_SIM

static const char const g_moduleId[] = "ThingseeOne"; /* TODO */
static const char const g_moduleName[] = "ThingseeOne";

#ifdef CONFIG_HTS221_HUMIDITY
static const struct ts_sense_info sense_info_hts221[] =
    {
#ifdef CONFIG_THINGSEE_ENGINE_SENSE_TEMPERATURE
	{
	    .sId = SENSE_ID_TEMPERATURE,
	    .name = g_temperature_str,
	    .min = { .valuetype = VALUEDOUBLE, .valuedouble = -40 },
	    .max = { .valuetype = VALUEDOUBLE, .valuedouble = 85 },
	    .min_interval = 100,
	    .ops.irq.init = NULL,
	    .ops.irq.uninit = NULL,
	    .ops.active.init = sense_hts221_active_init,
	    .ops.active.uninit = sense_hts221_active_uninit,
	    .ops.active.read = sense_hts221_active_read,
	},
#endif
#ifdef CONFIG_THINGSEE_ENGINE_SENSE_HUMIDITY
	{
	    .sId = SENSE_ID_HUMIDITY,
	    .name = g_humidity_str,
	    .min = { .valuetype = VALUEDOUBLE, .valuedouble = 0 },
	    .max = { .valuetype = VALUEDOUBLE, .valuedouble = 100 },
	    .min_interval = 100,
	    .ops.irq.init = NULL,
	    .ops.irq.uninit = NULL,
	    .ops.active.init = sense_hts221_active_init,
	    .ops.active.uninit = sense_hts221_active_uninit,
	    .ops.active.read = sense_hts221_active_read,
	}
#endif
    };
#endif

#ifdef CONFIG_I2C_HUMIDITY_DEV_SHT25
static const struct ts_sense_info sense_info_sht25[] =
    {
	{
	    .sId = SENSE_ID_TEMPERATURE,
	    .name = g_temperature_str,
	    .min = { .valuetype = VALUEDOUBLE, .valuedouble = -40 },
	    .max = { .valuetype = VALUEDOUBLE, .valuedouble = 125 },
	    .min_interval = 100,
	    .ops.irq.init = NULL,
	    .ops.irq.uninit = NULL,
	    .ops.active.init = NULL,
	    .ops.active.uninit = NULL,
	    .ops.active.read = NULL,
	},
	{
	    .sId = SENSE_ID_HUMIDITY,
	    .name = g_humidity_str,
	    .min = { .valuetype = VALUEDOUBLE, .valuedouble = 0 },
	    .max = { .valuetype = VALUEDOUBLE, .valuedouble = 100 },
	    .min_interval = 100,
	    .ops.irq.init = NULL,
	    .ops.irq.uninit = NULL,
	    .ops.active.init = NULL,
	    .ops.active.uninit = NULL,
	    .ops.active.read = NULL,
	}
    };
#endif

#ifdef CONFIG_I2C_PRESSURE_DEV_LPS25H
static const struct ts_sense_info sense_info_lps25h[] =
    {
#ifdef CONFIG_THINGSEE_ENGINE_SENSE_PRESSURE
	{
	    .sId = SENSE_ID_PRESSURE,
	    .name = g_pressure_str,
	    .min = { .valuetype = VALUEDOUBLE, .valuedouble = 260 },
	    .max = { .valuetype = VALUEDOUBLE, .valuedouble = 1260 },
	    .min_interval = 100,
	    .ops.irq.init = NULL,
	    .ops.irq.uninit = NULL,
	    .ops.active.init = NULL,
	    .ops.active.uninit = NULL,
	    .ops.active.read = sense_lps25h_active_read_pressure,
	},
#endif
#ifdef CONFIG_THINGSEE_ENGINE_SENSE_TEMPERATURE
	{
	    .sId = SENSE_ID_TEMPERATURE,
	    .name = g_temperature_1_str,
	    .min = { .valuetype = VALUEDOUBLE, .valuedouble = -40 },
	    .max = { .valuetype = VALUEDOUBLE, .valuedouble = 100 },
	    .min_interval = 100,
	    .ops.irq.init = NULL,
	    .ops.irq.uninit = NULL,
	    .ops.active.init = NULL,
	    .ops.active.uninit = NULL,
	    .ops.active.read = sense_lps25h_active_read_temperature,
	},
#endif
    };
#endif

static const struct ts_sense_info sense_info_power_button[] =
    {
	{
	    .sId = SENSE_ID_POWER_BUTTON_PRESSED,
	    .name = g_power_button_pressed_str,
	    .min = { .valuetype = VALUEUINT32, .valueuint32 = 0 },
	    .max = { .valuetype = VALUEUINT32, .valueuint32 = 1 },
	    .min_interval = -1,
	    .ops.irq.init = sense_power_button_pressed_irq_init,
	    .ops.irq.uninit = NULL,
	    .ops.active.init = sense_power_button_pressed_active_init,
	    .ops.active.uninit = NULL,
	    .ops.active.read = sense_power_button_pressed_active_read,
	}
    };

#if CONFIG_LIS2DH
static const struct ts_sense_info sense_info_lis2dh[] =
    {
#ifdef CONFIG_THINGSEE_ENGINE_SENSE_ACCELERATION
	{
	    .sId = SENSE_ID_LONGITUDINAL,
	    .name = g_acceleration_longitudinal_str,
	    .min = { .valuetype = VALUEDOUBLE, .valuedouble = -4 },
	    .max = { .valuetype = VALUEDOUBLE, .valuedouble = 4 },
	    .min_interval = -1,
	    .ops.irq.init = sense_acceleration_init,
	    .ops.irq.uninit = sense_acceleration_uninit,
	    .ops.active.init = NULL,
	    .ops.active.uninit = NULL,
	    .ops.active.read = NULL,
	},
	{
	    .sId = SENSE_ID_LATERAL,
	    .name = g_acceleration_lateral_str,
	    .min = { .valuetype = VALUEDOUBLE, .valuedouble = -4 },
	    .max = { .valuetype = VALUEDOUBLE, .valuedouble = 4 },
	    .min_interval = -1,
	    .ops.irq.init = sense_acceleration_init,
	    .ops.irq.uninit = sense_acceleration_uninit,
	    .ops.active.init = NULL,
	    .ops.active.uninit = NULL,
	    .ops.active.read = NULL,
	},
	{
	    .sId = SENSE_ID_VERTICAL,
	    .name = g_acceleration_vertical_str,
	    .min = { .valuetype = VALUEDOUBLE, .valuedouble = -4 },
	    .max = { .valuetype = VALUEDOUBLE, .valuedouble = 4 },
	    .min_interval = -1,
	    .ops.irq.init = sense_acceleration_init,
	    .ops.irq.uninit = sense_acceleration_uninit,
	    .ops.active.init = NULL,
	    .ops.active.uninit = NULL,
	    .ops.active.read = NULL,
	},
#endif
#ifdef CONFIG_THINGSEE_ENGINE_SENSE_IMPACT
	{
	    .sId = SENSE_ID_IMPACT,
	    .name = g_acceleration_impact_str,
	    .min = { .valuetype = VALUEDOUBLE, .valuedouble = 0 },
	    .max = { .valuetype = VALUEDOUBLE, .valuedouble = 5.656 },
	    .min_interval = -1,
	    .ops.irq.init = sense_acceleration_init,
	    .ops.irq.uninit = sense_acceleration_uninit,
	    .ops.active.init = NULL,
	    .ops.active.uninit = NULL,
	    .ops.active.read = NULL,
	},
#endif
    };
#endif

#ifdef CONFIG_LSM9DS1_SENS
static const struct ts_sense_info sense_info_lsm9ds1[] =
    {
#ifdef CONFIG_THINGSEE_ENGINE_SENSE_ORIENTATION
	{
	    .sId = SENSE_ID_HEADING,
	    .name = g_heading_str,
	    .min = { .valuetype = VALUEDOUBLE, .valuedouble = 0 },
	    .max = { .valuetype = VALUEDOUBLE, .valuedouble = 360 },
	    .min_interval = 100,
	    .ops.irq.init = NULL,
	    .ops.irq.uninit = NULL,
	    .ops.active.init = sense_orientation_init,
	    .ops.active.uninit = sense_orientation_uninit,
	    .ops.active.read = sense_orientation,
	},
	{
	    .sId = SENSE_ID_YAW,
	    .name = g_yaw_str,
	    .min = { .valuetype = VALUEDOUBLE, .valuedouble = -180 },
	    .max = { .valuetype = VALUEDOUBLE, .valuedouble = 180 },
	    .min_interval = 100,
	    .ops.irq.init = NULL,
	    .ops.irq.uninit = NULL,
	    .ops.active.init = sense_orientation_init,
	    .ops.active.uninit = sense_orientation_uninit,
	    .ops.active.read = sense_orientation, /* fusion */
	},
	{
	    .sId = SENSE_ID_PITCH,
	    .name = g_pitch_str,
	    .min = { .valuetype = VALUEDOUBLE, .valuedouble = -90 },
	    .max = { .valuetype = VALUEDOUBLE, .valuedouble = 90 },
	    .min_interval = 100,
	    .ops.irq.init = NULL,
	    .ops.irq.uninit = NULL,
	    .ops.active.init = sense_orientation_init,
	    .ops.active.uninit = sense_orientation_uninit,
	    .ops.active.read = sense_orientation, /* fusion */
	},
	{
	    .sId = SENSE_ID_ROLL,
	    .name = g_roll_str,
	    .min = { .valuetype = VALUEDOUBLE, .valuedouble = -180 },
	    .max = { .valuetype = VALUEDOUBLE, .valuedouble = 180 },
	    .min_interval = 100,
	    .ops.irq.init = NULL,
	    .ops.irq.uninit = NULL,
	    .ops.active.init = sense_orientation_init,
	    .ops.active.uninit = sense_orientation_uninit,
	    .ops.active.read = sense_orientation, /* fusion */
	},
#endif
#ifdef CONFIG_THINGSEE_ENGINE_SENSE_ACCELERATION
	{
	    .sId = SENSE_ID_LONGITUDINAL,
	    .name = g_acceleration_longitudinal_str,
	    .min = { .valuetype = VALUEDOUBLE, .valuedouble = -16 },
	    .max = { .valuetype = VALUEDOUBLE, .valuedouble = 16 },
	    .min_interval = 100,
	    .ops.irq.init = NULL,
	    .ops.irq.uninit = NULL,
	    .ops.active.init = sense_orientation_init,
	    .ops.active.uninit = sense_orientation_uninit,
	    .ops.active.read = sense_lsm9ds1_acceleration,
	},
	{
	    .sId = SENSE_ID_LATERAL,
	    .name = g_acceleration_lateral_str,
	    .min = { .valuetype = VALUEDOUBLE, .valuedouble = -16 },
	    .max = { .valuetype = VALUEDOUBLE, .valuedouble = 16 },
	    .min_interval = 100,
	    .ops.irq.init = NULL,
	    .ops.irq.uninit = NULL,
	    .ops.active.init = sense_orientation_init,
	    .ops.active.uninit = sense_orientation_uninit,
	    .ops.active.read = sense_lsm9ds1_acceleration,
	},
	{
	    .sId = SENSE_ID_VERTICAL,
	    .name = g_acceleration_vertical_str,
	    .min = { .valuetype = VALUEDOUBLE, .valuedouble = -16 },
	    .max = { .valuetype = VALUEDOUBLE, .valuedouble = 16 },
	    .min_interval = 100,
	    .ops.irq.init = NULL,
	    .ops.irq.uninit = NULL,
	    .ops.active.init = sense_orientation_init,
	    .ops.active.uninit = sense_orientation_uninit,
	    .ops.active.read = sense_lsm9ds1_acceleration,
	},
#endif
#ifdef CONFIG_THINGSEE_ENGINE_SENSE_MAGNETIC_FIELD
	{
	    .sId = SENSE_ID_MAGNETIC_FIELD_LONGITUDINAL,
	    .name = g_magnetic_field_longitudinal_str,
	    .min = { .valuetype = VALUEDOUBLE, .valuedouble = -4 },
	    .max = { .valuetype = VALUEDOUBLE, .valuedouble = 4 },
	    .min_interval = 100,
	    .ops.irq.init = NULL,
	    .ops.irq.uninit = NULL,
	    .ops.active.init = sense_orientation_init,
	    .ops.active.uninit = sense_orientation_uninit,
	    .ops.active.read = sense_lsm9ds1_magnetic_field,
	},
	{
	    .sId = SENSE_ID_MAGNETIC_FIELD_LATERAL,
	    .name = g_magnetic_field_lateral_str,
	    .min = { .valuetype = VALUEDOUBLE, .valuedouble = -4 },
	    .max = { .valuetype = VALUEDOUBLE, .valuedouble = 4 },
	    .min_interval = 100,
	    .ops.irq.init = NULL,
	    .ops.irq.uninit = NULL,
	    .ops.active.init = sense_orientation_init,
	    .ops.active.uninit = sense_orientation_uninit,
	    .ops.active.read = sense_lsm9ds1_magnetic_field,
	},
	{
	    .sId = SENSE_ID_MAGNETIC_FIELD_VERTICAL,
	    .name = g_magnetic_field_vertical_str,
	    .min = { .valuetype = VALUEDOUBLE, .valuedouble = -4 },
	    .max = { .valuetype = VALUEDOUBLE, .valuedouble = 4 },
	    .min_interval = 100,
	    .ops.irq.init = NULL,
	    .ops.irq.uninit = NULL,
	    .ops.active.init = sense_orientation_init,
	    .ops.active.uninit = sense_orientation_init,
	    .ops.active.read = sense_lsm9ds1_magnetic_field,
	},
#endif
#ifdef CONFIG_THINGSEE_ENGINE_SENSE_ORIENTATION
	{
	    .sId = SENSE_ID_ANGLE_SPEED_YAW,
	    .name = g_angle_speed_yaw_str,
	    .min = { .valuetype = VALUEDOUBLE, .valuedouble = -245 },
	    .max = { .valuetype = VALUEDOUBLE, .valuedouble = 245 },
	    .min_interval = 100,
	    .ops.irq.init = NULL,
	    .ops.irq.uninit = NULL,
	    .ops.active.init = sense_orientation_init,
	    .ops.active.uninit = sense_orientation_uninit,
	    .ops.active.read = sense_lsm9ds1_angle_speed, /* gyro */
	},
	{
	    .sId = SENSE_ID_ANGLE_SPEED_PITCH,
	    .name = g_angle_speed_pitch_str,
	    .min = { .valuetype = VALUEDOUBLE, .valuedouble = -245 },
	    .max = { .valuetype = VALUEDOUBLE, .valuedouble = 245 },
	    .min_interval = 100,
	    .ops.irq.init = NULL,
	    .ops.irq.uninit = NULL,
	    .ops.active.init = sense_orientation_init,
	    .ops.active.uninit = sense_orientation_uninit,
	    .ops.active.read = sense_lsm9ds1_angle_speed, /* gyro */
	},
	{
	    .sId = SENSE_ID_ANGLE_SPEED_ROLL,
	    .name = g_angle_speed_roll_str,
	    .min = { .valuetype = VALUEDOUBLE, .valuedouble = -245 },
	    .max = { .valuetype = VALUEDOUBLE, .valuedouble = 245 },
	    .min_interval = 100,
	    .ops.irq.init = NULL,
	    .ops.irq.uninit = NULL,
	    .ops.active.init = sense_orientation_init,
	    .ops.active.uninit = sense_orientation_uninit,
	    .ops.active.read = sense_lsm9ds1_angle_speed, /* gyro */
	},
#endif
    };
#endif

#ifdef CONFIG_THINGSEE_GPS_MODULE
static const struct ts_sense_info sense_info_ublox_gps[] =
    {
#ifdef CONFIG_THINGSEE_ENGINE_SENSE_LOCATION
        {
            .sId = SENSE_ID_IS_INSIDE_GEOFENCE,
            .name = g_gps_is_inside_geofence_str,
            .min = { .valuetype = VALUEBOOL, .valuebool = false },
            .max = { .valuetype = VALUEBOOL, .valuebool = true },
            .min_interval = 100,
            .ops.irq.init = sense_location_init,
            .ops.irq.uninit = sense_location_uninit,
            .ops.active.init = NULL,
            .ops.active.uninit = NULL,
            .ops.active.read = NULL,
        },
        {
            .sId = SENSE_ID_LATITUDE,
            .name = g_gps_latitude_str,
            .min = { .valuetype = VALUEDOUBLE, .valuedouble = -90 },
            .max = { .valuetype = VALUEDOUBLE, .valuedouble = 90 },
            .min_interval = 100,
            .ops.irq.init = sense_location_init,
            .ops.irq.uninit = sense_location_uninit,
            .ops.active.init = NULL,
            .ops.active.uninit = NULL,
            .ops.active.read = NULL,
        },
	{
	    .sId = SENSE_ID_LONGITUDE,
	    .name = g_gps_longitude_str,
	    .min = { .valuetype = VALUEDOUBLE, .valuedouble = -180 },
	    .max = { .valuetype = VALUEDOUBLE, .valuedouble = 180 },
	    .min_interval = 100,
	    .ops.irq.init = sense_location_init,
	    .ops.irq.uninit = sense_location_uninit,
	    .ops.active.init = NULL,
	    .ops.active.uninit = NULL,
	    .ops.active.read = NULL,
	},
	{
	    .sId = SENSE_ID_ALTITUDE,
	    .name = g_gps_altitude_str,
	    .min = { .valuetype = VALUEINT32, .valueint32 = -10000 },
	    .max = { .valuetype = VALUEINT32, .valueint32 = 10000 },
	    .min_interval = 100,
	    .ops.irq.init = sense_location_init,
	    .ops.irq.uninit = sense_location_uninit,
	    .ops.active.init = NULL,
	    .ops.active.uninit = NULL,
	    .ops.active.read = NULL,
	},
	{
	    .sId = SENSE_ID_ACCURACY,
	    .name = g_gps_accuracy_str,
	    .min = { .valuetype = VALUEUINT32, .valueuint32 = 0 },
	    .max = { .valuetype = VALUEUINT32, .valueuint32 = 1000 },
	    .min_interval = 100,
	    .ops.irq.init = sense_location_init,
	    .ops.irq.uninit = sense_location_uninit,
	    .ops.active.init = NULL,
	    .ops.active.uninit = NULL,
	    .ops.active.read = NULL,
	},
	{
	    .sId = SENSE_ID_GPS_TIMESTAMP,
	    .name = g_gps_timestamp_str,
	    .min = { .valuetype = VALUEUINT32, .valueuint32 = 0 },
	    .max = { .valuetype = VALUEUINT32, .valueuint32 = UINT32_MAX },
	    .min_interval = 100,
	    .ops.irq.init = sense_location_init,
	    .ops.irq.uninit = sense_location_uninit,
	    .ops.active.init = NULL,
	    .ops.active.uninit = NULL,
	    .ops.active.read = NULL,
	},
	{
	    .sId = SENSE_ID_LATLON,
	    .name = g_gps_latlon_str,
	    .min = { .valuetype = VALUESTRING, .valuestring = NULL },
	    .max = { .valuetype = VALUESTRING, .valuestring = NULL },
	    .min_interval = 100,
	    .ops.irq.init = sense_location_init,
	    .ops.irq.uninit = sense_location_uninit,
	    .ops.active.init = NULL,
	    .ops.active.uninit = NULL,
	    .ops.active.read = NULL,
	},
#endif
#ifdef CONFIG_THINGSEE_ENGINE_SENSE_SPEED
	{
	    .sId = SENSE_ID_GROUND_SPEED,
	    .name = g_ground_speed_str,
	    .min = { .valuetype = VALUEDOUBLE, .valuedouble = 0 },
	    .max = { .valuetype = VALUEDOUBLE, .valuedouble = 1000 },
	    .min_interval = 100,
	    .ops.irq.init = sense_location_init,
	    .ops.irq.uninit = sense_location_uninit,
	    .ops.active.init = NULL,
	    .ops.active.uninit = NULL,
	    .ops.active.read = NULL,
	},
#endif
#ifdef CONFIG_THINGSEE_ENGINE_SENSE_ORIENTATION
	{
	    .sId = SENSE_ID_HEADING,
	    .name = g_heading_str,
	    .min = { .valuetype = VALUEDOUBLE, .valuedouble = 0 },
	    .max = { .valuetype = VALUEDOUBLE, .valuedouble = 360 },
	    .min_interval = 100,
	    .ops.irq.init = sense_location_init,
	    .ops.irq.uninit = sense_location_uninit,
	    .ops.active.init = NULL,
	    .ops.active.uninit = NULL,
	    .ops.active.read = NULL,
	}
#endif
    };
#endif

#ifdef CONFIG_THINGSEE_CHARGER_MODULE
static const struct ts_sense_info sense_info_energy[] =
    {
#ifdef CONFIG_THINGSEE_ENGINE_SENSE_ENERGY
        {
            .sId = SENSE_ID_BATTERY_FULL_CAPACITY,
            .name = g_battery_full_capacity_str,
            .min = { .valuetype = VALUEDOUBLE, .valuedouble = 0 },
            .max = { .valuetype = VALUEDOUBLE, .valuedouble = 999999 },
            .min_interval = 100,
            .ops.irq.init = NULL,
            .ops.irq.uninit = NULL,
	    .ops.active.init = NULL,
	    .ops.active.uninit = NULL,
            .ops.active.read = sense_energy,
        },
        {
            .sId = SENSE_ID_CURRENT_LEVEL,
            .name = g_battery_current_level_str,
            .min = { .valuetype = VALUEDOUBLE, .valuedouble = 0 },
            .max = { .valuetype = VALUEDOUBLE, .valuedouble = 100 },
            .min_interval = 100,
            .ops.irq.init = NULL,
            .ops.irq.uninit = NULL,
	    .ops.active.init = NULL,
	    .ops.active.uninit = NULL,
            .ops.active.read = sense_energy,
        },
        {
            .sId = SENSE_ID_CURRENT_VOLTAGE,
            .name = g_battery_current_voltage_str,
            .min = { .valuetype = VALUEDOUBLE, .valuedouble = 0 },
            .max = { .valuetype = VALUEDOUBLE, .valuedouble = 999999 },
            .min_interval = 100,
            .ops.irq.init = NULL,
            .ops.irq.uninit = NULL,
	    .ops.active.init = NULL,
	    .ops.active.uninit = NULL,
            .ops.active.read = sense_energy,
        },
        {
            .sId = SENSE_ID_CHARGER_IS_CONNECTED,
            .name = g_charger_is_connected_str,
            .min = { .valuetype = VALUEBOOL, .valuebool = false },
            .max = { .valuetype = VALUEBOOL, .valuebool = true },
            .min_interval = 100,
            .ops.irq.init = NULL,
            .ops.irq.uninit = NULL,
	    .ops.active.init = NULL,
	    .ops.active.uninit = NULL,
            .ops.active.read = sense_energy,
        },
#endif
    };
#endif

#ifdef CONFIG_THINGSEE_ENGINE_SENSE_TIME
static const struct ts_sense_info sense_info_time[] =
    {
        {
            .sId = SENSE_ID_UNIX_TIME,
            .name = g_unix_time_str,
            .min = { .valuetype = VALUEUINT32, .valueuint32 = 0 },
            .max = { .valuetype = VALUEUINT32, .valueuint32 = UINT32_MAX },
            .min_interval = -1,
            .ops.irq.init = sense_time_init,
            .ops.irq.uninit = NULL,
	    .ops.active.init = NULL,
	    .ops.active.uninit = NULL,
            .ops.active.read = NULL,
        },
    };
#endif

#ifdef CONFIG_THINGSEE_ENGINE_SENSE_AMBIENT_LIGHT
static const struct ts_sense_info sense_info_als[] =
    {
        {
            .sId = SENSE_ID_AMBIENT_LIGHT,
            .name = g_ambient_light_str,
            .min = { .valuetype = VALUEDOUBLE, .valuedouble = 0 },
            .max = { .valuetype = VALUEDOUBLE, .valuedouble = 187269 },
            .min_interval = -1,
            .ops.irq.init = sense_ambient_light_irq_init,
            .ops.irq.uninit = sense_ambient_light_irq_uninit,
	    .ops.active.init = sense_ambient_light_active_init,
	    .ops.active.uninit = sense_ambient_light_active_uninit,
            .ops.active.read = sense_ambient_light_active_read,
        },
    };
#endif

static const struct ts_module g_modules[] =
    {
#ifdef CONFIG_HTS221_HUMIDITY
	{
	    "hts221",
	    g_moduleName,

	    ARRAY_SIZE(sense_info_hts221),
	    sense_info_hts221,
	},
#endif
#ifdef CONFIG_I2C_HUMIDITY_DEV_SHT25
	{
	    "sht25",
	    g_moduleName,

	    ARRAY_SIZE(sense_info_sht25),
	    sense_info_sht25,
	},
#endif
#ifdef CONFIG_I2C_PRESSURE_DEV_LPS25H
	{
	    "lps25h",
	    g_moduleName,

	    ARRAY_SIZE(sense_info_lps25h),
	    sense_info_lps25h,
	},
#endif
	{
	    "power button",
	    g_moduleName,

	    ARRAY_SIZE(sense_info_power_button),
	    sense_info_power_button,
	},
#ifdef CONFIG_LIS2DH
	{
	    "lis2dh",
	    g_moduleName,

	    ARRAY_SIZE(sense_info_lis2dh),
	    sense_info_lis2dh,
	},
#endif
#ifdef CONFIG_LSM9DS1_SENS
	{
	    "lsm9ds1",
	    g_moduleName,

	    ARRAY_SIZE(sense_info_lsm9ds1),
	    sense_info_lsm9ds1,
	},
#endif
#ifdef CONFIG_THINGSEE_GPS_MODULE
	{
	    "ublox gps",
	    g_moduleName,

	    ARRAY_SIZE(sense_info_ublox_gps),
	    sense_info_ublox_gps,
	},
#endif
#ifdef CONFIG_THINGSEE_CHARGER_MODULE
	{
	    "energy",
	    g_moduleName,

	    ARRAY_SIZE(sense_info_energy),
	    sense_info_energy,
	},
#endif
#ifdef CONFIG_THINGSEE_ENGINE_SENSE_TIME
	{
	    "time",
	    g_moduleName,

	    ARRAY_SIZE(sense_info_time),
	    sense_info_time,
	},
#endif
#ifdef CONFIG_THINGSEE_ENGINE_SENSE_AMBIENT_LIGHT
	{
	    "max44009",
	    g_moduleName,

	    ARRAY_SIZE(sense_info_als),
	    sense_info_als,
	},
#endif
    };
#endif /* !CONFIG_ARCH_SIM */

static dq_queue_t g_group_id_location[PROPERTY_ID_LOCATION_LAST + 1];
static dq_queue_t g_group_id_speed[PROPERTY_ID_SPEED_LAST + 1];
static dq_queue_t g_group_id_energy[PROPERTY_ID_ENERGY_LAST + 1];
static dq_queue_t g_group_id_orientation[PROPERTY_ID_ORIENTATION_LAST + 1];
static dq_queue_t g_group_id_acceleration[PROPERTY_ID_ACCELERATION_LAST + 1];
static dq_queue_t g_group_id_environment[PROPERTY_ID_ENVIRONMENT_LAST + 1];
static dq_queue_t g_group_id_hw_keys[PROPERTY_ID_HW_KEYS_LAST + 1];
static dq_queue_t g_group_id_time[PROPERTY_ID_TIME_LAST + 1];

static struct group {
  dq_queue_t *list;
  int last_property_id;
} g_lookup_tbl[] = {
    [GROUP_ID_LOCATION] = { g_group_id_location,  PROPERTY_ID_LOCATION_LAST, },
    [GROUP_ID_SPEED] = { g_group_id_speed,  PROPERTY_ID_SPEED_LAST, },
    [GROUP_ID_ENERGY] = { g_group_id_energy,  PROPERTY_ID_ENERGY_LAST, },
    [GROUP_ID_ORIENTATION] = { g_group_id_orientation,  PROPERTY_ID_ORIENTATION_LAST, },
    [GROUP_ID_ACCELERATION] = { g_group_id_acceleration,  PROPERTY_ID_ACCELERATION_LAST, },
    [GROUP_ID_ENVIRONMENT] = { g_group_id_environment,  PROPERTY_ID_ENVIRONMENT_LAST, },
    [GROUP_ID_HW_KEYS] = { g_group_id_hw_keys,  PROPERTY_ID_HW_KEYS_LAST, },
    [GROUP_ID_TIME] = { g_group_id_time,  PROPERTY_ID_TIME_LAST, },
};

static int
start_cause_timer (struct ts_cause *cause);

static int
cause_timer_callback (const int timer_id, void * const priv)
{
  struct ts_cause *cause = (struct ts_cause *) priv;

  DEBUGASSERT(timer_id == cause->dyn.timer_id);

  return engine_cause_get_value (cause);
}

static int cause_timer_callback_date(const int timer_id,
    const struct timespec *date, void * const priv)
{
  struct ts_cause *cause = (struct ts_cause *) priv;

  DEBUGASSERT(timer_id == cause->dyn.timer_id);

  start_cause_timer(cause);

  return engine_cause_get_value (cause);
}

static bool cause_timer_in_past(struct timespec *time, struct timespec *currts)
{
  if (time->tv_sec == currts->tv_sec)
    {
      return time->tv_nsec < currts->tv_nsec;
    }

  return time->tv_sec < currts->tv_sec;
}

static int
start_cause_timer (struct ts_cause *cause)
{
  if (cause->conf.measurement.interval < DEEP_SLEEP_RATE_LIMIT)
    {
      cause->dyn.timer_id = ts_core_timer_setup (TS_TIMER_TYPE_INTERVAL,
                                             cause->conf.measurement.interval,
                                             cause_timer_callback, cause);
    }
  else
    {
      struct timespec currts;

      clock_gettime(CLOCK_MONOTONIC, &currts);

      do
        {
          cause->dyn.time.tv_sec += cause->conf.measurement.interval / 1000;
          cause->dyn.time.tv_nsec += (cause->conf.measurement.interval % 1000) * 1000 * 1000;
          while (cause->dyn.time.tv_nsec >= 1000 * 1000 * 1000)
            {
              cause->dyn.time.tv_sec++;
              cause->dyn.time.tv_nsec -= 1000 * 1000 * 1000;
            }

          if (!cause_timer_in_past(&cause->dyn.time, &currts))
            {
              break;
            }

          eng_dbg("cause time in past, adjusting to next step.\n");
        }
      while (true);

      cause->dyn.timer_id = ts_core_timer_setup_date (&cause->dyn.time, cause_timer_callback_date, cause);
    }

  if (cause->dyn.timer_id < 0)
    {
      return -TS_ENGINE_ERROR_SYSTEM;
    }

  return OK;
}

int
engine_cause_get_value (struct ts_cause *cause)
{
  return cause->dyn.sense_info->ops.active.read (cause);
}

int
engine_cause_request_value (struct ts_cause *cause)
{
  int ret;
  const struct ts_sense_info *sense_info = cause->dyn.sense_info;

  cause->dyn.sense_value.value.valuetype = sense_info->min.valuetype;

  if (cause->conf.measurement.interval > 0 && sense_info->ops.active.read)
    {
      if (sense_info->ops.active.init)
        {
          sense_info->ops.active.init (cause);
        }

      ret = clock_gettime (CLOCK_MONOTONIC, &cause->dyn.time);
      if (ret < 0)
        {
          eng_dbg("clock_gettime failed\n");
          return ERROR;
        }

      return start_cause_timer(cause);
    }
  else
    {
      if (!sense_info->ops.irq.init)
        {
          eng_dbg("no ops.irq.init defined for interrupt configuration of sense 0x%08x\n", cause->dyn.sense_value.sId);
          return ERROR;
        }

      return sense_info->ops.irq.init (cause);
    }
}

const struct ts_sense_info *
get_sense_info (sense_id_t sId)
{
  uint8_t group_id;
  uint8_t property_id;
  uint8_t index;
  struct ts_sense_lookup *sense_lookup;

  group_id = (sId >> 16) & 0xff;
  property_id = (sId >> 8) & 0xff;
  index = sId & 0xff;

  if (group_id < 1 || group_id > GROUP_ID_LAST ||
      property_id < 1 || property_id > g_lookup_tbl[group_id].last_property_id ||
      dq_empty(&g_lookup_tbl[group_id].list[property_id]))
    {
      goto fail_out;
    }

  sense_lookup = (struct ts_sense_lookup *)sq_peek (&g_lookup_tbl[group_id].list[property_id]);

  while (sense_lookup)
    {
      if (sense_lookup->index == index)
	{
	  return sense_lookup->sense_info;
	}

      sense_lookup = (struct ts_sense_lookup *)dq_next (&sense_lookup->entry);
    }

fail_out:

  eng_dbg ("sId 0x%08x does not exist\n", sId);

  return NULL;
}

static int
add_sense (const struct ts_sense_info *sense_info)
{
  struct ts_sense_lookup *lookup, *prev;
  uint8_t group_id;
  uint8_t property_id;

  lookup = malloc (sizeof(*lookup));
  if (!lookup)
    {
      eng_dbg ("malloc for ts_sense_lookup failed\n");
      return -TS_ENGINE_ERROR_OOM;
    }

  group_id = (sense_info->sId >> 16) & 0xff;
  property_id = (sense_info->sId >> 8) & 0xff;

  lookup->sense_info = sense_info;

  dq_addlast (&lookup->entry, &g_lookup_tbl[group_id].list[property_id]);

  prev = (struct ts_sense_lookup *)lookup->entry.blink;

  if (prev)
    {
      lookup->index = prev->index + 1;
    }
  else
    {
      lookup->index = 0;
    }

  return OK;
}

#ifdef CONFIG_ARCH_SIM
static int
generate_sense_info_sim (int g, int p)
  {
    struct ts_sense_info *sense_info;

    sense_info = malloc(sizeof(*sense_info));
    if (!sense_info)
      {
	eng_dbg ("malloc failed for sense_info\n");
	return ERROR;
      }

    sense_info->sId = g << 16 | p << 8;
    sense_info->valuetype = VALUEDOUBLE;
    sense_info->ops.irq.init = NULL;
    sense_info->ops.irq.uninit = NULL;
    sense_info->ops.active.read = engine_sense_get_value_sim;

    return add_sense (sense_info);
  }
#endif

int
initialize_sense_lookup (void)
{
  int g, p;
#ifndef CONFIG_ARCH_SIM
  int i, j;
#endif
  int ret;

  for (g = GROUP_ID_LOCATION; g <= GROUP_ID_LAST; g++)
    {
      for (p = 1; p <= g_lookup_tbl[g].last_property_id; p++)
	{
	  dq_init (&g_lookup_tbl[g].list[p]);

#ifdef CONFIG_ARCH_SIM
	  ret = generate_sense_info_sim (g, p);
	  if (ret != OK)
	    {
	      eng_dbg ("generate_sense_info_sim failed\n");
	      return ERROR;
	    }
#endif
	}
    }

#ifndef CONFIG_ARCH_SIM
  for (i = 0; i < ARRAY_SIZE(g_modules); i++)
    {
      eng_dbg ("%s %s\n", g_modules[i].id, g_modules[i].name);

      for (j = 0; j < g_modules[i].number_of_senses; j++)
	{
	  if (g_modules[i].sense_info[j].ops.irq.init ||
	      g_modules[i].sense_info[j].ops.active.read)
	    {
	      ret = add_sense (&g_modules[i].sense_info[j]);
	      if (ret != OK)
		{
		  eng_dbg ("add_sense failed\n");
		  return ret;
		}
	    }

	  eng_dbg ("  0x%08x\n", g_modules[i].sense_info[j].sId);
	}
    }
#endif

  return OK;
}

static uint8_t
find_index (const struct ts_sense_info * const sense_info)
{
  struct ts_sense_lookup *lookup;
  uint8_t group_id;
  uint8_t property_id;

  group_id = (sense_info->sId >> 16) & 0xff;
  property_id = (sense_info->sId >> 8) & 0xff;

  lookup = (struct ts_sense_lookup *)sq_peek (&g_lookup_tbl[group_id].list[property_id]);

  while (lookup)
    {
      if (lookup->sense_info == sense_info)
	{
	  return lookup->index;
	}

      lookup = (struct ts_sense_lookup *) dq_next(&lookup->entry);
    }

  DEBUGASSERT(false);

  return 0;
}

static int
ts_value_to_double (const struct ts_value * const value, double *outval)
{
  switch (value->valuetype)
  {
    case VALUEDOUBLE:
      *outval = value->valuedouble;
      break;
    case VALUEINT16:
      *outval = (double)value->valueint16;
      break;
    case VALUEUINT16:
      *outval = (double)value->valueuint16;
      break;
    case VALUEINT32:
      *outval = (double)value->valueint32;
      break;
    case VALUEUINT32:
      *outval = (double)value->valueuint32;
      break;
    case VALUEBOOL:
      *outval = (double)value->valuebool;
      break;
    case VALUEARRAY:
      *outval = (double)value->valuearray.number_of_items;
      break;
    default:
      return ERROR;
  }

  return OK;
}

cJSON *
generate_module_json (void)
{
#ifdef CONFIG_ARCH_SIM
  return NULL;
#else
  cJSON *module, *modules, *sense, *senses;
  int i, j;
  int sIds[MAX_SENSES_PER_SENSOR];
  int number_of_senses = 0;
  uint8_t index;
  const struct ts_sense_info *sense_info;
  double val;
  int ret;
  char buf[16];

  for (i = 0; i < ARRAY_SIZE(g_modules); i++)
    {
      for (j = 0;
	  j < g_modules[i].number_of_senses && j < MAX_SENSES_PER_SENSOR; j++)
	{
	  if (g_modules[i].sense_info[j].ops.irq.init
	      || g_modules[i].sense_info[j].ops.active.read)
	    {
	      index = find_index (&g_modules[i].sense_info[j]);

	      sIds[number_of_senses++] = g_modules[i].sense_info[j].sId
		  | index;
	    }
	}
    }

  if (!number_of_senses)
    {
      return NULL;
    }

  modules = cJSON_CreateArray ();
  module = cJSON_CreateObject ();

  cJSON_AddStringToObject(module, "moduleId", g_moduleId);
  cJSON_AddStringToObject(module, "moduleName", g_moduleName);

  senses = cJSON_CreateArray ();

  for (i = 0; i < number_of_senses; i++)
    {
      sense = cJSON_CreateObject ();
      if (!sense)
	{
	  eng_dbg("cJSON_CreateObject failed\n");
	  return NULL;
	}

      sense_info = get_sense_info (sIds[i]);
      if (!sense_info)
	{
	  eng_dbg("get_sense_info failed\n");
	  return NULL;
	}

      snprintf (buf, 16, "0x%08x", sIds[i]);
      cJSON_AddStringToObject(sense, "sId", buf);

      if (sense_info->min.valuetype != VALUESTRING)
        {
          if (sense_info->min.valuetype == VALUEARRAY)
            {
              val = 0.0;
            }
          else
            {
              ret = ts_value_to_double (&sense_info->min, &val);
              if (ret != OK)
                {
                  eng_dbg("ts_value_to_double min failed\n");
                  return NULL;
                }
            }

          cJSON_AddNumberToObject(sense, "min", val);
        }

      if (sense_info->max.valuetype != VALUESTRING)
        {
          ret = ts_value_to_double (&sense_info->max, &val);
          if (ret != OK)
            {
              eng_dbg("ts_value_to_double max failed\n");
              return NULL;
            }

          cJSON_AddNumberToObject(sense, "max", val);
        }

      cJSON_AddNumberToObject(sense, "minInterval", sense_info->min_interval);

      cJSON_AddItemToArray (senses, sense);
    }

  cJSON_AddItemToObject (module, "senses", senses);
  cJSON_AddItemToArray (modules, module);

  return modules;
#endif
}
