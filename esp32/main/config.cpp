/*
 * Firmware for a segway-style robot using ESP32.
 * Copyright (C) 2017  Sakari Kapanen
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>

#include "locks.h"
#include "espway.h"

espway_config my_config;

const static espway_config DEFAULT_CONFIG = {
  .pid_coeffs_arr = {
    { ANGLE_KP, ANGLE_KI, ANGLE_KD },
    { VEL_KP, VEL_KI, VEL_KD }
  },
  .gyro_offsets = { GYRO_X_OFFSET, GYRO_Y_OFFSET, GYRO_Z_OFFSET }
};

void pretty_print_config()
{
  MutexLock lock(pid_mutex);
  printf(
      "\n\nESPway current config:\n\n"
      "#define ANGLE_KP %f\n"
      "#define ANGLE_KI %f\n"
      "#define ANGLE_KD %f\n"
      "#define VEL_KP %f\n"
      "#define VEL_KI %f\n"
      "#define VEL_KD %f\n"
      "#define GYRO_X_OFFSET %d\n"
      "#define GYRO_Y_OFFSET %d\n"
      "#define GYRO_Z_OFFSET %d\n"
      "\n\n",
      my_config.pid_coeffs_arr[ANGLE].p,
      my_config.pid_coeffs_arr[ANGLE].i,
      my_config.pid_coeffs_arr[ANGLE].d,
      my_config.pid_coeffs_arr[VEL].p,
      my_config.pid_coeffs_arr[VEL].i,
      my_config.pid_coeffs_arr[VEL].d,
      my_config.gyro_offsets[0],
      my_config.gyro_offsets[1],
      my_config.gyro_offsets[2]
  );
}

void load_hardcoded_config()
{
  my_config = DEFAULT_CONFIG;
}

void load_config()
{
  MutexLock lock(pid_mutex);
  load_hardcoded_config();
#if 0
  sysparam_get_data_static("ANGLE_PID", (uint8_t*) &my_config.pid_coeffs_arr[ANGLE],
                           sizeof(pid_coeffs), NULL, NULL);
  sysparam_get_data_static("VEL_PID", (uint8_t*) &my_config.pid_coeffs_arr[VEL],
                           sizeof(pid_coeffs), NULL, NULL);
  sysparam_get_data_static("GYRO_OFFSETS", (uint8_t*) &my_config.gyro_offsets,
                           3 * sizeof(int16_t), NULL, NULL);
#endif
}

void apply_config_params()
{
  MutexLock lock(pid_mutex);
  pid_initialize(&my_config.pid_coeffs_arr[ANGLE],
                 IMU_SAMPLE_TIME,
                 -1.0, 1.0, false, &pid_settings_arr[ANGLE]);
  pid_initialize(&my_config.pid_coeffs_arr[VEL],
                 IMU_SAMPLE_TIME,
                 STABLE_ANGLE - FALL_LIMIT, STABLE_ANGLE + FALL_LIMIT, true,
                 &pid_settings_arr[VEL]);
}

bool save_flash_config()
{
  bool success = true;

#if 0
  {
    MutexLock lock(pid_mutex);
    success = sysparam_set_data("ANGLE_PID", (uint8_t *)&my_config.pid_coeffs_arr[ANGLE],
                                sizeof(pid_coeffs), true) == SYSPARAM_OK;
    if (success)
        success = sysparam_set_data("VEL_PID",
                                    (uint8_t*) &my_config.pid_coeffs_arr[VEL],
                                    sizeof(pid_coeffs), true) == SYSPARAM_OK;
    if (success)
        success = sysparam_set_data("GYRO_OFFSETS", (uint8_t*) &my_config.gyro_offsets,
                                    3 * sizeof(int16_t), true) == SYSPARAM_OK;
  }
#endif

  return success;
}

bool clear_flash_config()
{
#if 0
  uint32_t base_addr, num_sectors;
  return sysparam_get_info(&base_addr, &num_sectors) == SYSPARAM_OK &&
     sysparam_create_area(base_addr, num_sectors, true) == SYSPARAM_OK &&
     sysparam_init(base_addr, 0) == SYSPARAM_OK;
#endif
  //!!
  return true;
}

void update_pid_controller(pid_controller_index idx, double p, double i, double d)
{
  if (idx > 2) return;

  {
    MutexLock lock(pid_mutex);
    pid_coeffs *p_coeffs = &my_config.pid_coeffs_arr[idx];
    p_coeffs->p = p;
    p_coeffs->i = i;
    p_coeffs->d = d;
    pid_update_params(p_coeffs, &pid_settings_arr[idx]);
  }
}
