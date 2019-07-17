#pragma once

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
        WoundUp
    };
        
    Led(int gpio);

    void set_color(int r, int g, int b);
    void set_color(Colour);
    void set_color_right(int r, int g, int b);

private:
    strand_t strand;
    bool right_color_fixed = false;
};
