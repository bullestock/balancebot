#include <stdio.h>
#include <utility>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <driver/pcnt.h>
#include "sdkconfig.h"

#include "encoder.h"
#include "imu.h"
#include "led.h"
#include "motor.h"

#define LED_GPIO 13

#define GPIO_PWM0A_OUT 16
#define GPIO_PWM0B_OUT 17
#define GPIO_PWM1A_OUT 18
#define GPIO_PWM1B_OUT 19

#define GPIO_ENC_A1   22
#define GPIO_ENC_A2   23
#define GPIO_ENC_B1    2
#define GPIO_ENC_B2   15

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

void event_task(void* p)
{
    auto encoders = reinterpret_cast<std::pair<Encoder*, Encoder*>*>(p);
    auto enc_a = encoders->first;
    auto enc_b = encoders->second;
    while (1)
    {
        enc_a->poll();
        enc_b->poll();
    }
}

xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events

void imu_task(void* p)
{
    auto imu = reinterpret_cast<Imu*>(p);
    while (1)
    {
        vTaskDelay(1000/portTICK_PERIOD_MS);
        int16_t data[6];
        if (!imu->read_raw_data(data))
            printf("read error\n");
        else
        {
            printf("Acc X  %d\n", data[0]);
            printf("Acc Y  %d\n", data[1]);
            printf("Acc Z  %d\n", data[2]);
            printf("Rate X %d\n", data[3]);
            printf("Rate Y %d\n", data[4]);
            printf("Rate Z %d\n", data[5]);
        }
    }
}

extern "C"
void app_main()
{
    auto led = new Led(LED_GPIO);
    xTaskCreate(led_task, "led_task", 2048, led, 5, NULL);

    auto motor_a = new Motor(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B, GPIO_PWM0A_OUT, GPIO_PWM0B_OUT);
    auto motor_b = new Motor(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM1A, MCPWM1B, GPIO_PWM1A_OUT, GPIO_PWM1B_OUT);
    static auto motors = std::make_pair(motor_a, motor_b);
    xTaskCreate(motor_task, "motor_task", 2048, &motors, 5, NULL);

    auto enc_a = new Encoder(PCNT_UNIT_0, GPIO_ENC_A1, GPIO_ENC_A2);
    auto enc_b = new Encoder(PCNT_UNIT_1, GPIO_ENC_B1, GPIO_ENC_B2);
    static auto encoders = std::make_pair(enc_a, enc_b);
    xTaskCreate(event_task, "event_task", 2048, &encoders, 5, NULL);

    auto imu = new Imu();
    xTaskCreate(imu_task, "imu_task", 2048, imu, 5, NULL);
}
