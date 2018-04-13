#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "esp32_digital_led_lib.h"

#define LED_GPIO 13

void gpioSetup(int gpioNum, int gpioMode, int gpioVal)
{
    gpio_num_t gpioNumNative = static_cast<gpio_num_t>(gpioNum);
    gpio_mode_t gpioModeNative = static_cast<gpio_mode_t>(gpioMode);
    gpio_pad_select_gpio(gpioNumNative);
    gpio_set_direction(gpioNumNative, gpioModeNative);
    gpio_set_level(gpioNumNative, gpioVal);
}

strand_t STRANDS[] = { // Avoid using any of the strapping pins on the ESP32
  {.rmtChannel = 1, .gpioNum = LED_GPIO, .ledType = LED_WS2812B_V3, .brightLimit = 32, .numPixels =  2,
   .pixels = nullptr, ._stateVars = nullptr}
};

void led_task(void*)
{
    // Setup
    gpioSetup(LED_GPIO, GPIO_MODE_OUTPUT, 0);
    if (digitalLeds_initStrands(STRANDS, 1))
    {
        printf("Init FAILURE\n");
        return;
    }
    auto pStrand = STRANDS;
    digitalLeds_resetPixels(pStrand);

    // Loop
    int col = 0;
    while (1)
    {
        int r = 0, g = 0, b = 0;
        switch (col)
        {
        case 0: // red
            r = 255;
            break;
        case 1: // blue
            b = 255;
            break;
        case 2: // green
            g = 255;
            break;
        case 3: // yellow
            r = g = 255;
            break;
        }
        printf("%d %d %d\n", r, g, b);
        pStrand->pixels[0] = pStrand->pixels[1] = pixelFromRGB(g, r, b);
        digitalLeds_updatePixels(pStrand);
        vTaskDelay(1000/portTICK_PERIOD_MS);
        ++col;
        if (col > 3)
            col = 0;
    }
}

extern "C"
void app_main()
{
    xTaskCreate(&led_task, "led_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
}
