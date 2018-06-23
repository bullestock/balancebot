#include "motor.h"

#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

Motor::Motor(mcpwm_unit_t _unit,
             mcpwm_timer_t _timer,
             mcpwm_io_signals_t _pwm_a,
             mcpwm_io_signals_t _pwm_b,
             int _gpio_a, int _gpio_b)
    : unit(_unit),
      timer(_timer)
{
    mcpwm_gpio_init(unit, _pwm_a, _gpio_a);
    mcpwm_gpio_init(unit, _pwm_b, _gpio_b);
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(unit, timer, &pwm_config);  
}

void Motor::set_speed(float speed)
{
    if (speed >= 0)
    {
        mcpwm_set_signal_low(unit, timer, MCPWM_OPR_B);
        mcpwm_set_duty(unit, timer, MCPWM_OPR_A, speed*100);
        mcpwm_set_duty_type(unit, timer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    }
    else
    {
        speed = -speed;
        mcpwm_set_signal_low(unit, timer, MCPWM_OPR_A);
        mcpwm_set_duty(unit, timer, MCPWM_OPR_B, speed*100);
        mcpwm_set_duty_type(unit, timer, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }
}

