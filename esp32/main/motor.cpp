#include "motor.h"

#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

Motor::Motor(mcpwm_unit_t _unit,
             mcpwm_timer_t _timer,
             mcpwm_io_signals_t _pwm_a,
             mcpwm_io_signals_t _pwm_b,
             int _gpio_enable, int _gpio_phase)
    : unit(_unit),
      timer(_timer)
{
    mcpwm_gpio_init(_unit, _pwm_a, _gpio_enable);
    mcpwm_gpio_init(_unit, _pwm_b, _gpio_phase);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(_unit, _timer, &pwm_config);

}

void Motor::set_speed(float speed)
{
    if (speed < 0)
        mcpwm_set_signal_low(unit, timer, MCPWM_OPR_B);
    else
        mcpwm_set_signal_high(unit, timer, MCPWM_OPR_B);

    mcpwm_set_duty(unit, timer, MCPWM_OPR_A, speed < 0 ? -speed*100 : speed*100);
}

extern Motor* motor_a;
extern Motor* motor_b;

// -1 to +1
void set_motors(double m1, double m2)
{
    motor_a->set_speed(m1);
    motor_b->set_speed(-m2);
}
