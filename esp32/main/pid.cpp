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

#include "pid.h"

#include <stdio.h>

static inline double clamp(double x, double a, double b)
{
    return x > a ? (x < b ? x : b) : a;
}

void pid_initialize(const pid_coeffs* coeffs, double dt,
                    double out_min, double out_max, bool invert, pidsettings *settings)
{
    settings->dt = dt;
    settings->out_min = out_min;
    settings->out_max = out_max;
    settings->invert = invert;
    pid_update_params(coeffs, settings);
}

void pid_update_params(const pid_coeffs* coeffs,
                       pidsettings* settings)
{
    settings->Kp = coeffs->p;
    settings->Ki_times_dt = coeffs->i * settings->dt;
    settings->Kd_over_dt = coeffs->d / settings->dt;
    if (settings->invert)
    {
        settings->Kp *= -1;
        settings->Ki_times_dt *= -1;
        settings->Kd_over_dt *= -1;
    }
}

double pid_compute(double input, double setpoint,
                   pidsettings *settings, pidstate *state, bool show_debug)
{
    double error = input - setpoint;
    double p_term = settings->Kp * error;
    state->i_term = clamp(state->i_term + settings->Ki_times_dt * error,
                          settings->out_min, settings->out_max);
    double d_term = settings->Kd_over_dt * (input - state->last_input);
    double output = p_term + state->i_term + d_term;
    //if (show_debug) printf("error %f p_term %f i_term %f d_term %f output %f\n", error, p_term, state->i_term, d_term, output);
    state->last_input = input;
    return clamp(output, settings->out_min, settings->out_max);
}

void pid_reset(double input, double output, pidsettings *settings, pidstate *state)
{
    state->i_term = clamp(output, settings->out_min,
                          settings->out_max);
    state->last_input = input;
}

