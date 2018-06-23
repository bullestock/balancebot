/*
 * A simple PID library
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

#include <stdbool.h>

struct  pidsettings
{
    double Kp;
    double Ki_times_dt;
    double Kd_over_dt;
    double dt;
    double out_min;
    double out_max;
    bool invert;
};

struct pidstate
{
    double i_term;
    double last_input;
};


struct pid_coeffs
{
    double p, i, d;
};

void pid_initialize(const pid_coeffs* coeffs, double dt, double out_min, double out_max,
                    bool invert, pidsettings* settings);
void pid_update_params(const pid_coeffs* coeffs, pidsettings* settings);
double pid_compute(double input, double setpoint, pidsettings* settings, pidstate* state, bool show_debug);
void pid_reset(double input, double output, pidsettings* settings, pidstate* state);
