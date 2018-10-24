/*
 * Library for calculating orientation from an inertial measurement unit
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

#include "imu_math.h"
#include <cstring>
#include <stdio.h>

void mahony_filter_init(mahony_filter_state* state, float Kp, float Ki,
                        float gyro_factor, float dt)
{
    state->Kp = Kp;
    state->Ki = Ki;
    state->dt = dt;
    state->gyro_conversion_factor = gyro_factor;
    memset(state->integral.data, 0, sizeof(state->integral));
}

void mahony_filter_update(mahony_filter_state* state,
                          const int16_t* raw_accel, const int16_t* raw_gyro, vector3d* gravity)
{
    vector3d omega, accel;
    const double SCALE = 65536.0;
    for (size_t i = 0; i < 3; ++i)
    {
        accel.data[i] = raw_accel[i]/SCALE;
        omega.data[i] = raw_gyro[i]/SCALE;
    }

    omega = v3d_mul(state->gyro_conversion_factor, &omega);

    vector3d verror;
    if (accel.x != 0 || accel.y != 0 || accel.z != 0)
    {
        accel = v3d_normalize(&accel);
        verror = v3d_cross(&accel, gravity);
        state->integral = v3d_add(&state->integral, &verror);
        verror = v3d_mul(state->Kp, &verror);
        omega = v3d_add(&omega, &verror);
    }

    verror = v3d_mul(state->Ki, &state->integral);
    verror = v3d_mul(state->dt, &verror);
    omega = v3d_add(&omega, &verror);

    vector3d vupdate = v3d_cross(gravity, &omega);
    vupdate = v3d_mul(state->dt, &vupdate);
    *gravity = v3d_add(gravity, &vupdate);
    *gravity = v3d_normalize(gravity);
}
