#include "encoder.h"

#include <soc/timer_group_struct.h>
#include <driver/periph_ctrl.h>
#include <driver/timer.h>

#define PCNT_H_LIM_VAL      1000
#define PCNT_L_LIM_VAL     -1000

xQueueHandle Encoder::pcnt_evt_queue = nullptr;

Encoder::Encoder(pcnt_unit_t _unit, int gpio1, int gpio2)
    : unit(_unit)
{
    // Initialize PCNT event queue and PCNT functions
    if (!pcnt_evt_queue)
        pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));

    pcnt_config_t pcnt_config;
    pcnt_config.pulse_gpio_num = gpio1;
    pcnt_config.ctrl_gpio_num = gpio2;
    pcnt_config.channel = PCNT_CHANNEL_0;
    pcnt_config.unit = unit;
    pcnt_config.pos_mode = PCNT_COUNT_INC;            // Count up on the positive edge
    pcnt_config.neg_mode = PCNT_COUNT_DEC;
    pcnt_config.lctrl_mode = PCNT_MODE_KEEP;          // Reverse counting direction if low
    pcnt_config.hctrl_mode = PCNT_MODE_REVERSE;       // Keep the primary counter mode if high
    pcnt_config.counter_h_lim = PCNT_H_LIM_VAL;
    pcnt_config.counter_l_lim = PCNT_L_LIM_VAL;

    pcnt_unit_config(&pcnt_config);
    pcnt_set_filter_value(unit, 100);
    pcnt_filter_enable(unit);

    pcnt_event_enable(unit, PCNT_EVT_ZERO);
    pcnt_event_enable(unit, PCNT_EVT_H_LIM);
    pcnt_event_enable(unit, PCNT_EVT_L_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);

    /* Register ISR handler and enable interrupts for PCNT unit */
    pcnt_isr_register(quad_enc_isr, NULL, 0, NULL);
    pcnt_intr_enable(unit);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(unit);
}

void Encoder::poll()
{
    /* Wait for the event information passed from PCNT's interrupt handler.
     * Once received, decode the event type and print it on the serial monitor.
     */
    pcnt_evt_t evt;
    auto res = xQueueReceive(pcnt_evt_queue, &evt, 1000 / portTICK_PERIOD_MS);
    //printf("PCNT unit %d\n", unit);
    if (res == pdTRUE) {
        pcnt_get_counter_value(unit, &count);
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
        pcnt_get_counter_value(unit, &count);
        //printf("Current counter value: %d\n", count);
    }
}

void IRAM_ATTR Encoder::quad_enc_isr(void *arg)
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
