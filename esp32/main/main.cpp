#include <stdio.h>
#include <utility>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sdkconfig.h"

#include "freertos/queue.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/pcnt.h"

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

#define PCNT_PULSE_GPIO     22
#define PCNT_CONTROL_GPIO   23

#define PCNT_H_LIM_VAL      1000
#define PCNT_L_LIM_VAL     -1000

xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events

/* A sample structure to pass events from the PCNT
 * interrupt handler to the main program.
 */
typedef struct {
    int unit;  // the PCNT unit that originated an interrupt
    uint32_t status; // information on the event type that caused the interrupt
} pcnt_evt_t;

static void IRAM_ATTR quad_enc_isr(void *arg)
{
    uint32_t intr_status = PCNT.int_st.val;
    int i;
    pcnt_evt_t evt;
    portBASE_TYPE HPTaskAwoken = pdFALSE;

    for (i = 0; i < PCNT_UNIT_MAX; i++)
        if (intr_status & (BIT(i))) {
            evt.unit = i;
            /* Save the PCNT event type that caused an interrupt
               to pass it to the main program */
            evt.status = PCNT.status_unit[i].val;
            PCNT.int_clr.val = BIT(i);
            xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
            if (HPTaskAwoken == pdTRUE)
                portYIELD_FROM_ISR();
        }
}

static void quadrature_encoder_counter_init() {
    pcnt_config_t pcnt_config;
    pcnt_config.pulse_gpio_num = PCNT_PULSE_GPIO;
    pcnt_config.ctrl_gpio_num = PCNT_CONTROL_GPIO;
    pcnt_config.channel = PCNT_CHANNEL_0;
    pcnt_config.unit = PCNT_UNIT_0;
    pcnt_config.pos_mode = PCNT_COUNT_INC;            // Count up on the positive edge
    pcnt_config.neg_mode = PCNT_COUNT_DEC;
    pcnt_config.lctrl_mode = PCNT_MODE_KEEP;          // Reverse counting direction if low
    pcnt_config.hctrl_mode = PCNT_MODE_REVERSE;       // Keep the primary counter mode if high
    pcnt_config.counter_h_lim = PCNT_H_LIM_VAL;
    pcnt_config.counter_l_lim = PCNT_L_LIM_VAL;

    pcnt_unit_config(&pcnt_config);
    pcnt_set_filter_value(PCNT_UNIT_0, 100);
    pcnt_filter_enable(PCNT_UNIT_0);

    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_ZERO);
    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_L_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);

    /* Register ISR handler and enable interrupts for PCNT unit */
    pcnt_isr_register(quad_enc_isr, NULL, 0, NULL);
    pcnt_intr_enable(PCNT_UNIT_0);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(PCNT_UNIT_0);
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
    
    /* Initialize PCNT event queue and PCNT functions */
    pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
    quadrature_encoder_counter_init();

    int16_t count = 0;
    pcnt_evt_t evt;
    portBASE_TYPE res;
    while (1) {
        /* Wait for the event information passed from PCNT's interrupt handler.
         * Once received, decode the event type and print it on the serial monitor.
         */
        res = xQueueReceive(pcnt_evt_queue, &evt, 1000 / portTICK_PERIOD_MS);
        if (res == pdTRUE) {
            pcnt_get_counter_value(PCNT_UNIT_0, &count);
            printf("Event PCNT unit[%d]; cnt: %d\n", evt.unit, count);
            if (evt.status & PCNT_STATUS_THRES1_M) {
                printf("THRES1 EVT\n");
            }
            if (evt.status & PCNT_STATUS_THRES0_M) {
                printf("THRES0 EVT\n");
            }
            if (evt.status & PCNT_STATUS_L_LIM_M) {
                printf("L_LIM EVT\n");
            }
            if (evt.status & PCNT_STATUS_H_LIM_M) {
                printf("H_LIM EVT\n");
            }
            if (evt.status & PCNT_STATUS_ZERO_M) {
                printf("ZERO EVT\n");
            }
        } else {
            pcnt_get_counter_value(PCNT_UNIT_0, &count);
            printf("Current counter value: %d\n", count);
        }
    }

}
