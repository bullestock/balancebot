#pragma once

#include "esp_adc_cal.h"

class Battery
{
public:
    Battery();

    float read_voltage() const;
};

extern esp_adc_cal_characteristics_t* adc_chars;
