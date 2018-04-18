#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <driver/pcnt.h>

class Encoder
{
public:
    Encoder(pcnt_unit_t unit,
            int gpio1, int gpio2);

    void poll();
    
private:
    struct pcnt_evt_t
    {
        int unit;  // the PCNT unit that originated an interrupt
        uint32_t status; // information on the event type that caused the interrupt
    };

    static void IRAM_ATTR quad_enc_isr(void*);

    pcnt_unit_t unit = (pcnt_unit_t) 0;
    
    // A queue to handle pulse counter events
    static xQueueHandle pcnt_evt_queue;

    int16_t count = 0;
};



