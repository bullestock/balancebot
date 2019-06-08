#include "led.h"

#include <driver/gpio.h>

void gpioSetup(int gpioNum, int gpioMode, int gpioVal)
{
    gpio_num_t gpioNumNative = static_cast<gpio_num_t>(gpioNum);
    gpio_mode_t gpioModeNative = static_cast<gpio_mode_t>(gpioMode);
    gpio_pad_select_gpio(gpioNumNative);
    gpio_set_direction(gpioNumNative, gpioModeNative);
    gpio_set_level(gpioNumNative, gpioVal);
}

Led::Led(int gpio)
{
    gpioSetup(gpio, GPIO_MODE_OUTPUT, 0);

    strand.rmtChannel = 1;
    strand.ledType = LED_WS2812B_V3;
    strand.brightLimit = 32;
    strand.numPixels =  2;
    strand.pixels = nullptr;
    strand._stateVars = nullptr;
    strand.gpioNum = gpio;
    if (digitalLeds_initStrands(&strand, 1))
    {
        printf("Init FAILURE\n");
        return;
    }
    digitalLeds_resetPixels(&strand);
}

void Led::set_color(int r, int g, int b)
{
    strand.pixels[0] = strand.pixels[1] = pixelFromRGB(g, r, b);
    digitalLeds_updatePixels(&strand);
}

void Led::set_color(Colour c)
{
    switch (c)
    {
    case Init:
        set_color(0, 64, 64);
        return;
    case Stabilized:
        set_color(255, 255, 255);
        return;
    case Running:
        set_color(0, 255, 0);
        return;
    case Fallen:
        set_color(0, 0, 255);
        return;
    case WoundUp:
        set_color(255, 0, 0);
        return;
    default:
        ESP_ERROR_CHECK(0);
        break;
    }
}
