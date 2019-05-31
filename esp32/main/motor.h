#pragma once

#include <driver/mcpwm.h>

class Motor
{
public:
    Motor(mcpwm_unit_t unit,
          mcpwm_timer_t timer,
          mcpwm_io_signals_t _pwm_a,
          mcpwm_io_signals_t _pwm_b,
          int _gpio_enable, int _gpio_phase);

    // -1.0 to 1.0
    void set_speed(float speed);

private:
    mcpwm_unit_t unit = (mcpwm_unit_t) 0;
    mcpwm_timer_t timer = (mcpwm_timer_t) 0;
};

void set_motors(double m1, double m2);
