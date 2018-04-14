#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "sdkconfig.h"

#include "esp32_digital_led_lib.h"

#define LED_GPIO 13

#define GPIO_PWM0A_OUT 16
#define GPIO_PWM0B_OUT 17
#define GPIO_PWM1A_OUT 18
#define GPIO_PWM1B_OUT 19

void gpioSetup(int gpioNum, int gpioMode, int gpioVal)
{
    gpio_num_t gpioNumNative = static_cast<gpio_num_t>(gpioNum);
    gpio_mode_t gpioModeNative = static_cast<gpio_mode_t>(gpioMode);
    gpio_pad_select_gpio(gpioNumNative);
    gpio_set_direction(gpioNumNative, gpioModeNative);
    gpio_set_level(gpioNumNative, gpioVal);
}

strand_t STRANDS[] = { // Avoid using any of the strapping pins on the ESP32
  {.rmtChannel = 1, .gpioNum = LED_GPIO, .ledType = LED_WS2812B_V3, .brightLimit = 32, .numPixels =  2,
   .pixels = nullptr, ._stateVars = nullptr}
};

void led_task(void*)
{
    // Setup
    gpioSetup(LED_GPIO, GPIO_MODE_OUTPUT, 0);
    if (digitalLeds_initStrands(STRANDS, 1))
    {
        printf("Init FAILURE\n");
        return;
    }
    auto pStrand = STRANDS;
    digitalLeds_resetPixels(pStrand);

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
        printf("%d %d %d\n", r, g, b);
        pStrand->pixels[0] = pStrand->pixels[1] = pixelFromRGB(g, r, b);
        digitalLeds_updatePixels(pStrand);
        vTaskDelay(1000/portTICK_PERIOD_MS);
        ++col;
        if (col > 3)
            col = 0;
    }
}

void set_motor(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, float duty_cycle)
{
    if (duty_cycle >= 0)
    {
        mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
        mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
        mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    }
    else
    {
        duty_cycle = -duty_cycle;
        mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
        mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle);
        mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }
}

void motor_task(void*)
{
    printf("Init PWM\n");

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_PWM1A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, GPIO_PWM1B_OUT);
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency = 500Hz,
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);  
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);  

    printf("Loop\n");
    
    bool fwd = true;
    float speed1 = 0.0;
    float speed2 = 0.0;
    while (1)
    {
        printf("Speed %f/%f Fwd %d\n", speed1, speed2, fwd);
        set_motor(MCPWM_UNIT_0, MCPWM_TIMER_0, fwd ? speed1 : -speed1);
        set_motor(MCPWM_UNIT_0, MCPWM_TIMER_1, fwd ? speed2 : -speed2);
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
    xTaskCreate(led_task, "led_task", 2048, NULL, 5, NULL);
    xTaskCreate(motor_task, "motor_task", 2048, NULL, 5, NULL);
}
