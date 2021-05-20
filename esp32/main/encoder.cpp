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
    pcnt_config_t pcnt_config;
    pcnt_config.pulse_gpio_num = gpio1;
    pcnt_config.ctrl_gpio_num = gpio2;
    pcnt_config.channel = PCNT_CHANNEL_0;
    pcnt_config.unit = unit;
    pcnt_config.pos_mode = PCNT_COUNT_DEC;
    pcnt_config.neg_mode = PCNT_COUNT_INC;
    pcnt_config.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config.counter_h_lim = PCNT_H_LIM_VAL;
    pcnt_config.counter_l_lim = PCNT_L_LIM_VAL;

    ESP_ERROR_CHECK(pcnt_unit_config(&pcnt_config));

    pcnt_config.pulse_gpio_num = gpio2;
    pcnt_config.ctrl_gpio_num = gpio1;
    pcnt_config.channel = PCNT_CHANNEL_1;
    pcnt_config.pos_mode = PCNT_COUNT_INC;
    pcnt_config.neg_mode = PCNT_COUNT_DEC;

    ESP_ERROR_CHECK(pcnt_unit_config(&pcnt_config));
    
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);

    // Initialize PCNT event queue and PCNT functions
    if (!pcnt_evt_queue)
    {
        pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
        ESP_ERROR_CHECK(pcnt_isr_service_install(0));
    }
    pcnt_isr_handler_add(unit, quad_enc_isr, this);

    pcnt_set_filter_value(unit, 100);
    pcnt_filter_enable(unit);

    pcnt_event_enable(unit, PCNT_EVT_H_LIM);
    pcnt_event_enable(unit, PCNT_EVT_L_LIM);

    pcnt_counter_resume(unit);
}

int64_t Encoder::poll()
{
    /* Wait for the event information passed from PCNT's interrupt handler.
     * Once received, decode the event type and print it on the serial monitor.
     */
    pcnt_evt_t evt;
    auto res = xQueueReceive(pcnt_evt_queue, &evt, 0);
    if (res == pdTRUE)
    {
        auto enc = (Encoder*) evt.enc;
        //printf("Status %d\n", evt.status);
        if (evt.status & PCNT_STATUS_L_LIM_M) {
            enc->accumulated += PCNT_L_LIM_VAL;
        }
        if (evt.status & PCNT_STATUS_H_LIM_M) {
            enc->accumulated += PCNT_H_LIM_VAL;
        }
    }

    int16_t temp_count;
    pcnt_get_counter_value(unit, &temp_count);
    return temp_count + accumulated;
}

void IRAM_ATTR Encoder::quad_enc_isr(void* arg)
{
    auto enc = (Encoder*) arg;

    uint32_t status = 0;
    pcnt_get_event_status(enc->unit, &status);
    if ((status & PCNT_STATUS_L_LIM_M) ||
        (status & PCNT_STATUS_H_LIM_M))
    {
        pcnt_evt_t evt;
        evt.enc = enc;
        evt.status = status;
        portBASE_TYPE HPTaskAwoken = pdFALSE;
        xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
        if (HPTaskAwoken == pdTRUE)
            portYIELD_FROM_ISR();
    }
}
