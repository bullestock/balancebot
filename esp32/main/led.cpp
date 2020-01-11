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
    strand.pixels[0] = pixelFromRGB(0, 0, 0);
}

int elapsed_time_ms(TickType_t t2, TickType_t t1)
{
    return (t2 - t1)*portTICK_PERIOD_MS;
}

int get_red(Led::Colour c)
{
    switch (c)
    {
    case Led::Init:
        return 0;
    case Led::Stabilized:
        return 255;
    case Led::Running:
        return 0;
    case Led::Fallen:
        return 0;
    case Led::WoundUp:
        return 255;
    case Led::None:
        return 0;
    default:
        ESP_ERROR_CHECK(0);
        break;
    }
    return 0;
}

int get_green(Led::Colour c)
{
    switch (c)
    {
    case Led::Init:
        return 64;
    case Led::Stabilized:
        return 255;
    case Led::Running:
        return 255;
    case Led::Fallen:
        return 0;
    case Led::WoundUp:
        return 0;
    case Led::None:
        return 0;
    default:
        ESP_ERROR_CHECK(0);
        break;
    }
    return 0;
}

int get_blue(Led::Colour c)
{
    switch (c)
    {
    case Led::Init:
        return 64;
    case Led::Stabilized:
        return 255;
    case Led::Running:
        return 0;
    case Led::Fallen:
        return 255;
    case Led::WoundUp:
        return 0;
    case Led::None:
        return 0;
    default:
        ESP_ERROR_CHECK(0);
        break;
    }
    return 0;
}

void Led::update()
{
    if (!flashing)
    {
        if (!dirty)
            return;
        dirty = false;
        do_set_color(r1, g1, b1);
        return;
    }
    const auto current_time = xTaskGetTickCount();
    const auto elapsed = elapsed_time_ms(current_time, last_tick);
    if (elapsed < period_ms)
        return;
    last_tick = current_time;
    primary_color = !primary_color;
    if (primary_color)
        do_set_color(r1, g1, b1);
    else
        do_set_color(r2, g2, b2);
}

void Led::set_color(int r, int g, int b)
{
    r1 = r;
    g1 = g;
    b1 = b;
    flashing = false;
    dirty = true;
}

void Led::set_colors(int _ms,
                    int _r1, int _g1, int _b1,
                    int _r2, int _g2, int _b2)
{
    r1 = _r1;
    g1 = _g1;
    b1 = _b1;
    r2 = _r2;
    g2 = _g2;
    b2 = _b2;
    flashing = true;
    period_ms = _ms;
}

void Led::set_color(Colour c)
{
    set_color(get_red(c), get_blue(c), get_green(c));
}

void Led::set_colors(int ms,
                     Colour c1, Colour c2)
{
    set_colors(ms, get_red(c1), get_blue(c1), get_green(c1),
               get_red(c2), get_blue(c2), get_green(c2));
}

void Led::do_set_color(int r, int g, int b)
{
    strand.pixels[1] = pixelFromRGB(g, r, b);
    digitalLeds_updatePixels(&strand);
}
