/*
 * Vector calculations
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

#pragma once

#include <cmath>

union vector3d
{
    struct
    {
        double x, y, z;
    };
    double data[3];
} ;

static inline vector3d v3d_add(const vector3d& u, const vector3d& v)
{
    vector3d result =
    {
        {
            u.x + v.x,
            u.y + v.y,
            u.z + v.z
        }
    };
    return result;
}

static inline vector3d v3d_sub(const vector3d& u, const vector3d& v)
{
    vector3d result =
    {
        {
            u.x - v.x,
            u.y - v.y,
            u.z - v.z
        }
    };
    return result;
}

static inline vector3d v3d_mul(double a, const vector3d& v)
{
    vector3d result =
    {
        {
            a * v.x,
            a * v.y,
            a * v.z
        }
    };
    return result;
}

static inline vector3d v3d_cross(const vector3d& u, const vector3d& v)
{
    vector3d result =
    {
        {
            u.y * v.z - u.z * v.y,
            u.z * v.x - u.x * v.z,
            u.x * v.y - u.y * v.x
        }
    };
    return result;
}

static inline vector3d v3d_normalize(const vector3d& u)
{
    double rnorm = u.x * u.x + u.y * u.y + u.z * u.z;
    rnorm = 1.0/sqrt(rnorm);
    return v3d_mul(rnorm, u);
}
