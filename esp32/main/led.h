#pragma once

#include "esp32_digital_led_lib.h"

class Led
{
public:
    Led(int gpio);

    void set_color(int r, int g, int b);

private:
    strand_t strand;
};
