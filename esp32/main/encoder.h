#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <driver/pcnt.h>

class Encoder
{
public:
    Encoder(pcnt_unit_t unit,
            int gpio1, int gpio2);

    int64_t poll();
    
private:
    struct pcnt_evt_t
    {
        Encoder* enc = 0;
        uint32_t status = 0;
    };

    static void IRAM_ATTR quad_enc_isr(void*);

    pcnt_unit_t unit = (pcnt_unit_t) 0;
    
    // A queue to handle pulse counter events
    static xQueueHandle pcnt_evt_queue;

    int64_t accumulated = 0;
};



