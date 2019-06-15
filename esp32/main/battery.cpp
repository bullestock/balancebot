#include "battery.h"

#include <driver/adc.h>

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate

esp_adc_cal_characteristics_t* adc_chars = nullptr;

Battery::Battery()
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten((adc1_channel_t) ADC_CHANNEL_4, ADC_ATTEN_DB_0);
    adc_chars = (esp_adc_cal_characteristics_t*) calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
}

float Battery::read_voltage() const
{
    auto adc_reading = adc1_get_raw((adc1_channel_t) ADC_CHANNEL_4);
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    return voltage * 0.0128408304498;
}
