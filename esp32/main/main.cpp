#include <stdio.h>
#include <utility>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sdkconfig.h"

#include "led.h"
#include "motor.h"

#define LED_GPIO 13

#define GPIO_PWM0A_OUT 16
#define GPIO_PWM0B_OUT 17
#define GPIO_PWM1A_OUT 18
#define GPIO_PWM1B_OUT 19

void led_task(void* p)
{
    auto led = reinterpret_cast<Led*>(p);
    // Loop
    int col = 0;
    while (1)
    {
        int r = 0, g = 0, b = 0;
        switch (col)
        {
        case 0: // red
            r = 255;
            break;
        case 1: // blue
            b = 255;
            break;
        case 2: // green
            g = 255;
            break;
        case 3: // yellow
            r = g = 255;
            break;
        }
        led->set_color(r, g, b);
        vTaskDelay(1000/portTICK_PERIOD_MS);
        ++col;
        if (col > 3)
            col = 0;
    }
}

void motor_task(void* p)
{
    auto motors = reinterpret_cast<std::pair<Motor*, Motor*>*>(p);
    auto motor_a = motors->first;
    auto motor_b = motors->second;
    
    bool fwd = true;
    float speed1 = 0.0;
    float speed2 = 0.0;
    while (1)
    {
        motor_a->set_speed(fwd ? speed1 : -speed1);
        motor_b->set_speed(fwd ? speed2 : -speed2);
        vTaskDelay(100/portTICK_PERIOD_MS);
        speed1 += 10;
        if (speed1 > 100)
        {
            speed1 = 0;
            speed2 += 10;
            if (speed2 > 100)
            {
                speed1 = speed2 = 0;
                fwd = !fwd;
            }
        }
    }
}

extern "C"
void app_main()
{
    auto led = new Led(LED_GPIO);

    auto motor_a = new Motor(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B, GPIO_PWM0A_OUT, GPIO_PWM0B_OUT);
    auto motor_b = new Motor(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM1A, MCPWM1B, GPIO_PWM1A_OUT, GPIO_PWM1B_OUT);
    auto motors = std::make_pair(motor_a, motor_b);

    xTaskCreate(led_task, "led_task", 2048, led, 5, NULL);
    xTaskCreate(motor_task, "motor_task", 2048, &motors, 5, NULL);
}
