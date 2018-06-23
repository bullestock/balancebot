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

void dump(const char* s, const vector3d& v)
{
    //printf("%s: %.3f %.3f %.3f\n", s, v.data[0], v.data[1], v.data[2]);
}

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

    dump("accel", accel);
    dump("omega raw", omega);
    omega = v3d_mul(state->gyro_conversion_factor, &omega);
    dump("omega radians ", omega);

    vector3d verror;
    if (accel.x != 0 || accel.y != 0 || accel.z != 0)
    {
        accel = v3d_normalize(&accel);
        dump("accel normalized", accel);
        verror = v3d_cross(&accel, gravity);
        dump("verror", verror);
        state->integral = v3d_add(&state->integral, &verror);
        dump("integral", state->integral);
        verror = v3d_mul(state->Kp, &verror);
        dump("verror 1", verror);
        omega = v3d_add(&omega, &verror);
        dump("omega 3", omega);
    }

    verror = v3d_mul(state->Ki, &state->integral);
    dump("verror 2", verror);
    verror = v3d_mul(state->dt, &verror);
    dump("verror 3", verror);
    omega = v3d_add(&omega, &verror);
    dump("omega 4", omega);

    vector3d vupdate = v3d_cross(gravity, &omega);
    dump("vupdate 1", vupdate);
    vupdate = v3d_mul(state->dt, &vupdate);
    dump("vupdate 2", vupdate);
    *gravity = v3d_add(gravity, &vupdate);
    dump("gravity 1", *gravity);
    *gravity = v3d_normalize(gravity);
    dump("gravity 2", *gravity);
}
