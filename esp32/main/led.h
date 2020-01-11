#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esp32_digital_led_lib.h"

class Led
{
public:
    enum Colour
    {
        Init,
        Stabilized,
        Running,
        Fallen,
        WoundUp,
        None
    };
        
    Led(int gpio);

    void update();
    
    void set_color(int r, int g, int b);
    void set_color(Colour c);
    void set_colors(int ms,
                    int r1, int g1, int b1,
                    int r2, int g2, int b2);
    void set_colors(int ms,
                    Colour c1, Colour c2);

private:
    void do_set_color(int r, int g, int b);

    strand_t strand;
    bool flashing = false;
    bool flashing_state = false;
    bool primary_color = true;
    int period_ms = 0;
    TickType_t  last_tick = 0;
    int r1 = 0;
    int g1 = 0;
    int b1 = 0;
    int r2 = 0;
    int g2 = 0;
    int b2 = 0;
    bool dirty = false;
};
