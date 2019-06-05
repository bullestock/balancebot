#include "console.h"

#include "espway.h"
#include "motor.h"
#include "imu.h"
#include "imu_math.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"

#include <driver/i2c.h>
#include <driver/uart.h>

#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"

int motor_test(int argc, char** argv)
{
    int count = 1;
    if (argc > 1)
        count = atoi(argv[1]);

    printf("Running motor test (%d)\n", count);
    for (int j = 0; j < count; ++j)
    {
        set_motors(0, 0);
        for (int i = 10; i <= 100; ++i)
        {
            if (!(i % 10))
                printf("A %d\n", i);
            set_motors(i/100.0, 0);
            vTaskDelay(50/portTICK_PERIOD_MS);
        }
        for (int i = 100; i >= -100; --i)
        {
            if (!(i % 10))
                printf("A %d\n", i);
            set_motors(i/100.0, 0);
            vTaskDelay(50/portTICK_PERIOD_MS);
        }
        for (int i = -100; i <= 0; ++i)
        {
            if (!(i % 10))
                printf("A %d\n", i);
            set_motors(i/100.0, 0);
            vTaskDelay(50/portTICK_PERIOD_MS);
        }
        set_motors(0, 0);
        vTaskDelay(1000/portTICK_PERIOD_MS);
        for (int i = 10; i <= 100; ++i)
        {
            if (!(i % 10))
                printf("B %d\n", i);
            set_motors(0, i/100.0);
            vTaskDelay(50/portTICK_PERIOD_MS);
        }
        for (int i = 100; i >= -100; --i)
        {
            if (!(i % 10))
                printf("B %d\n", i);
            set_motors(0, i/100.0);
            vTaskDelay(50/portTICK_PERIOD_MS);
        }
        for (int i = -100; i <= 0; ++i)
        {
            if (!(i % 10))
                printf("B %d\n", i);
            set_motors(0, i/100.0);
            vTaskDelay(50/portTICK_PERIOD_MS);
        }
        set_motors(0, 0);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    return 0;
}

int read_imu(int argc, char** argv)
{
    extern Imu* imu;
    extern mahony_filter_state imuparams;
    extern vector3d gravity;
    
    int count = 10;
    if (argc > 1)
        count = atoi(argv[1]);

    printf("Running IMU test (%d)\n", count);
    for (int j = 0; j < count; ++j)
    {
        int16_t raw_data[6];
        for (int i = 0; i < 100; ++i)
        {
            if (!imu->read_raw_data(raw_data))
            {
                printf("Reading IMU failed\n");
                continue;
            }
            mahony_filter_update(&imuparams, &raw_data[0], &raw_data[3], &gravity);
            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        printf("%d, %d, %d, %d, %d, %d\n",
               raw_data[0], raw_data[1], raw_data[2],
               raw_data[3], raw_data[4], raw_data[5]);
        // Calculate sine of pitch angle from gravity vector
        auto sin_pitch = -gravity.data[IMU_FORWARD_AXIS];
        if (IMU_INVERT_FORWARD_AXIS)
            sin_pitch = -sin_pitch;
        auto sin_roll = -gravity.data[IMU_SIDE_AXIS];
        if (IMU_INVERT_SIDE_AXIS)
            sin_roll = -sin_roll;
        printf("%f, %f, %f -> %f, %f\n",
               gravity.data[0], gravity.data[1], gravity.data[2],
               sin_pitch, sin_roll);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    
    return 0;
}

void initialize_console()
{
    /* Disable buffering on stdin */
    setvbuf(stdin, NULL, _IONBF, 0);

    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    esp_vfs_dev_uart_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    esp_vfs_dev_uart_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

    /* Configure UART. Note that REF_TICK is used so that the baud rate remains
     * correct while APB frequency is changing in light sleep mode.
     */
    uart_config_t uart_config;
    memset(&uart_config, 0, sizeof(uart_config));
    uart_config.baud_rate = CONFIG_CONSOLE_UART_BAUDRATE;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.use_ref_tick = true;
    ESP_ERROR_CHECK(uart_param_config((uart_port_t) CONFIG_CONSOLE_UART_NUM, &uart_config));

    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK(uart_driver_install((uart_port_t) CONFIG_CONSOLE_UART_NUM,
                                         256, 0, 0, NULL, 0));

    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(CONFIG_CONSOLE_UART_NUM);

    /* Initialize the console */
    esp_console_config_t console_config;
    memset(&console_config, 0, sizeof(console_config));
    console_config.max_cmdline_args = 8;
    console_config.max_cmdline_length = 256;
#if CONFIG_LOG_COLORS
    console_config.hint_color = atoi(LOG_COLOR_CYAN);
#endif
    ESP_ERROR_CHECK(esp_console_init(&console_config));

    /* Configure linenoise line completion library */
    /* Enable multiline editing. If not set, long commands will scroll within
     * single line.
     */
    linenoiseSetMultiLine(1);

    /* Tell linenoise where to get command completions and hints */
    linenoiseSetCompletionCallback(&esp_console_get_completion);
    linenoiseSetHintsCallback((linenoiseHintsCallback*) &esp_console_get_hint);

    /* Set command history size */
    linenoiseHistorySetMaxLen(100);
}

void run_console()
{
    initialize_console();

    /* Register commands */
    esp_console_register_help_command();

    const esp_console_cmd_t cmd1 = {
        .command = "motortest",
        .help = "Test the motors",
        .hint = NULL,
        .func = &motor_test,
        .argtable = nullptr
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd1));

    const esp_console_cmd_t cmd2 = {
        .command = "readimu",
        .help = "Read IMU data",
        .hint = NULL,
        .func = &read_imu,
        .argtable = nullptr
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd2));

    /* Prompt to be printed before each line.
     * This can be customized, made dynamic, etc.
     */
    const char* prompt = LOG_COLOR_I "esp32> " LOG_RESET_COLOR;

    printf("\n"
           "Type 'help' to get the list of commands.\n"
           "Use UP/DOWN arrows to navigate through command history.\n"
           "Press TAB when typing command name to auto-complete.\n");

    /* Figure out if the terminal supports escape sequences */
    int probe_status = linenoiseProbe();
    if (probe_status)
    {
        printf("\n"
               "Your terminal application does not support escape sequences.\n"
               "Line editing and history features are disabled.\n"
               "On Windows, try using Putty instead.\n");
        linenoiseSetDumbMode(1);
#if CONFIG_LOG_COLORS
        /* Since the terminal doesn't support escape sequences,
         * don't use color codes in the prompt.
         */
        prompt = "esp32> ";
#endif //CONFIG_LOG_COLORS
    }

    while (true)
    {
        char* line = linenoise(prompt);
        if (line == NULL)
            continue;

        linenoiseHistoryAdd(line);

        int ret;
        esp_err_t err = esp_console_run(line, &ret);
        if (err == ESP_ERR_NOT_FOUND)
            printf("Unrecognized command\n");
        else if (err == ESP_ERR_INVALID_ARG)
            ; // command was empty
        else if (err == ESP_OK && ret != ESP_OK)
            printf("Command returned non-zero error code: 0x%x (%s)\n", ret, esp_err_to_name(err));
        else if (err != ESP_OK)
            printf("Internal error: %s\n", esp_err_to_name(err));

        linenoiseFree(line);
    }
}

// Local Variables:
// compile-command: "(cd ..; make)"
// End:
