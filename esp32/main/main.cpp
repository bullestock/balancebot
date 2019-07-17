#include <stdio.h>
#include <string.h>
#include <utility>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <driver/adc.h>
#include <driver/ledc.h>
#include <driver/pcnt.h>

#include <lwip/sockets.h>

#include "esp_wifi.h"
#include "sdkconfig.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"

#include "websocket_server.h"

#include "battery.h"
#include "console.h"
#include "espway.h"
#include "encoder.h"
#include "imu.h"
#include "imu_math.h"
#include "led.h"
#include "locks.h"
#include "motor.h"

#define LED_GPIO 16
#define LED_PIN 2 // internal

#define GPIO_APHASE_OUT 21
#define GPIO_AENBL_OUT  22
#define GPIO_BPHASE_OUT 25
#define GPIO_BENBL_OUT  27

#define GPIO_ENC_A1    2
#define GPIO_ENC_A2    0
#define GPIO_ENC_B1    4
#define GPIO_ENC_B2   17

#define PRIO_COMMUNICATION  2
#define PRIO_MAIN_LOOP      (TCPIP_THREAD_PRIO + 1)

Led* led = nullptr;
Motor* motor_a = nullptr;
Motor* motor_b = nullptr;
Imu* imu = nullptr;
Battery* battery = nullptr;

#if 0

void event_task(void* p)
{
    auto encoders = reinterpret_cast<std::pair<Encoder*, Encoder*>*>(p);
    auto enc_a = encoders->first;
    auto enc_b = encoders->second;
    while (1)
    {
        enc_a->poll();
        enc_b->poll();
    }
}

#endif

xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events

mahony_filter_state imuparams;
static pidstate vel_pid_state;
static pidstate angle_pid_state;

SemaphoreHandle_t pid_mutex;
pidsettings pid_settings_arr[2];

static double target_speed = 0;
static double steering_bias = 0;

static SemaphoreHandle_t orientation_mutex;
vector3d gravity = {{ 0.0, 0.0, 1.0 }};

enum log_mode { LOG_FREQ, LOG_RAW, LOG_PITCH, LOG_NONE };

constexpr auto LOGMODE = LOG_FREQ;

#define ORIENTATION_STABILIZE_DURATION_US ((ORIENTATION_STABILIZE_DURATION) * 1000)
#define WINDUP_TIMEOUT_US ((WINDUP_TIMEOUT) * 1000)

static double sin_pitch = 0.0, sin_roll = 0.0;

static TaskHandle_t xCalculationTask = nullptr;

orientation get_orientation()
{
  orientation copy_orientation = {};
  {
    MutexLock lock(orientation_mutex);
    copy_orientation.sin_pitch = sin_pitch;
    copy_orientation.sin_roll = sin_roll;
  }
  return copy_orientation;
}

static SemaphoreHandle_t steering_mutex;

void set_steering(double new_target_speed, double new_steering_bias)
{
  {
    MutexLock lock(steering_mutex);
    target_speed = new_target_speed;
    steering_bias = new_steering_bias;
  }

  //!!if (xSteeringWatcher) xTaskNotify(xSteeringWatcher, 0, eNoAction);
}


double exponential_smooth(double prevVal, double newVal, double alpha)
{
    return prevVal + alpha * (newVal - prevVal);
}

int elapsed_time_us(TickType_t t2, TickType_t t1)
{
    return (t2 - t1)*portTICK_PERIOD_MS*1000;
}

void battery_cutoff()
{
    led->set_color(0, 0, 0);
    set_motors(0, 0);
    esp_deep_sleep_start();
}

void battery_task(void*)
{
    float smoothed_battery_value = battery->read_voltage();
    while (1)
    {
        smoothed_battery_value = exponential_smooth(smoothed_battery_value, battery->read_voltage(), 0.05);
        if (smoothed_battery_value < BATTERY_THRESHOLD)
        {
            if (ENABLE_BATTERY_CUTOFF)
            {
                battery_cutoff();
                // Terminate task
                break;
            }
            else
            {
                disable_motors();
                led->set_color_right(255, 0, 0);
            }
        }

        uint8_t buf[3];
        buf[0] = BATTERY;
        uint16_t* payload = (uint16_t*) &buf[1];
        payload[0] = static_cast<uint16_t>(smoothed_battery_value*1000);
        ws_server_send_bin_all(buf, sizeof(buf));

        vTaskDelay(BATTERY_CHECK_INTERVAL / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void main_loop(void* pvParameters)
{
    TickType_t time_last = xTaskGetTickCount();
    TickType_t current_time = 0;
    int n = 0;
    double smoothed_target_speed = 0;
    double travel_speed = 0;
    state my_state = STABILIZING_ORIENTATION;
    TickType_t stage_started = time_last;
    TickType_t last_wind_up = time_last;

    int loopcount = 0;
#define SHOW_DEBUG() 0 // (my_state == RUNNING)// && (loopcount == 0))

    led->set_color(Led::Init);
    while (1)
    {
        ++loopcount;
        if (loopcount >= 100)
            loopcount = 0;
        
        xTaskNotifyWait(0, 0, NULL, 1); // allow other tasks to run
        
        // 700 us
        int16_t raw_data[6];
        if (!imu->read_raw_data(raw_data))
        {
            printf("Reading IMU failed\n");
            continue;
        }

        // Update orientation estimate
        // < 100 us
        {
            MutexLock lock(orientation_mutex);
            mahony_filter_update(&imuparams, &raw_data[0], &raw_data[3], &gravity);
            // Calculate sine of pitch angle from gravity vector
            sin_pitch = -gravity.data[IMU_FORWARD_AXIS];
            if (IMU_INVERT_FORWARD_AXIS)
                sin_pitch = -sin_pitch;
            sin_roll = -gravity.data[IMU_SIDE_AXIS];
            if (IMU_INVERT_SIDE_AXIS)
                sin_roll = -sin_roll;
        }

        // Exponential smoothing of target speed
        
        double motor_bias;
        {
            MutexLock lock(orientation_mutex);
            smoothed_target_speed = exponential_smooth(smoothed_target_speed,
                                                       target_speed, TARGET_SPEED_SMOOTHING);
            motor_bias = steering_bias;
        }

        current_time = xTaskGetTickCount();

        //printf("sin_pitch %f sin_roll %f\n", sin_pitch, sin_roll);
        
        if (my_state == STABILIZING_ORIENTATION &&
            elapsed_time_us(current_time, stage_started) > ORIENTATION_STABILIZE_DURATION_US)
        {
            led->set_color(Led::Stabilized);
            printf("Done stabilizing at %u\n", current_time);
            printf("sin_pitch %f sin_roll %f\n", sin_pitch, sin_roll);
            my_state = RUNNING;
            last_wind_up = current_time;
            imuparams.Kp = MAHONY_FILTER_KP;
        }
        else if (my_state == RUNNING || my_state == WOUND_UP)
        {
            if (fabs(sin_pitch - STABLE_ANGLE) < FALL_LIMIT &&
                fabs(sin_roll) < ROLL_LIMIT)
            {
                // Perform PID update
                double target_angle, motor_speed;
                {
                    MutexLock lock(pid_mutex);
                    target_angle = pid_compute(travel_speed, smoothed_target_speed,
                                               &pid_settings_arr[VEL], &vel_pid_state,
                                               SHOW_DEBUG());
                    if (sin_pitch < (target_angle - HIGH_PID_LIMIT))
                        motor_speed = -1.0;
                    else if (sin_pitch > target_angle + HIGH_PID_LIMIT)
                        motor_speed = 1.0;
                    else
                        //!!
                        motor_speed = pid_compute(sin_pitch, target_angle,
                                                  &pid_settings_arr[ANGLE], &angle_pid_state,
                                                  SHOW_DEBUG());
                    if (SHOW_DEBUG())
                        printf("Angle Kp %f motor_speed %f\n", pid_settings_arr[ANGLE].Kp, motor_speed);
                }

                if (fabs(motor_speed) < MOTOR_DEADBAND)
                    motor_speed = 0;

                if (my_state == WOUND_UP)
                    set_motors(0, 0);
                else
                    set_motors(motor_speed + motor_bias,
                               motor_speed - motor_bias);

                if (motor_speed != 1.0 && motor_speed != -1.0)
                    last_wind_up = current_time;
                else
                {
                    const auto us_since_last_windup = elapsed_time_us(current_time, last_wind_up);
                    if (us_since_last_windup > WINDUP_TIMEOUT_US)
                    {
                        set_motors(0, 0);
                        if (my_state != WOUND_UP)
                            printf("WOUND UP at %u (%d)\n", current_time, us_since_last_windup);
                        my_state = WOUND_UP;
                        set_motors(0, 0);
                        led->set_color(Led::WoundUp);
                        //vTaskDelay(100/portTICK_PERIOD_MS);
                    }
                }

                // Estimate travel speed by exponential smoothing
                travel_speed = exponential_smooth(travel_speed, motor_speed,
                                                  TRAVEL_SPEED_SMOOTHING);
            }
            else
            {
                printf("FALLEN!\n");
                my_state = FALLEN;
                led->set_color(Led::Fallen);
                travel_speed = 0;
                smoothed_target_speed = 0;
                set_motors(0, 0);
            }
        }
        else if (my_state == FALLEN &&
                 fabs(sin_pitch - STABLE_ANGLE) < RECOVER_LIMIT &&
                 fabs(sin_roll) < ROLL_LIMIT)
        {
            printf("Running again!\n");
            my_state = RUNNING;
            last_wind_up = current_time;
            led->set_color(Led::Running);
            pid_reset(sin_pitch, 0, &pid_settings_arr[ANGLE], &angle_pid_state);
            pid_reset(0, STABLE_ANGLE, &pid_settings_arr[VEL],
                      &vel_pid_state);
        }

        if (LOGMODE == LOG_FREQ)
        {
            ++n;
            if (n == 1000)
            {
                n = 0;
                const auto now = xTaskGetTickCount();
                auto looptime = elapsed_time_us(now, time_last);
                printf("Looptime: %u us\n", looptime/1000);
                time_last = now;
            }
        }
        else if (LOGMODE == LOG_RAW)
        {
            printf("%d, %d, %d, %d, %d, %d\n",
                   raw_data[0], raw_data[1], raw_data[2],
                   raw_data[3], raw_data[4], raw_data[5]);
        }
        else if (LOGMODE == LOG_PITCH)
        {
            printf("%f, %f\n", sin_pitch, sin_roll);
        }

        //!!xTaskNotify(xIMUWatcher, 0, eNoAction);
    }
}

esp_err_t event_handler(void* ctx, system_event_t* event)
{
    printf("WiFi event: %d\n", event->event_id);
    return ESP_OK;
}

static void wifi_setup()
{
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, nullptr));

    tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_AP); // Don't run a DHCP client
    tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP);

    tcpip_adapter_ip_info_t ipInfo;
    inet_pton(AF_INET, "10.0.0.1", &ipInfo.ip);
    inet_pton(AF_INET, "10.0.0.1", &ipInfo.gw);
    inet_pton(AF_INET, "255.255.255.0", &ipInfo.netmask);
    tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP, &ipInfo);

    ESP_ERROR_CHECK(tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP));
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    const auto ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK)
    {
        printf("esp_wifi_init: %s\n", esp_err_to_name(ret));
        return;
    }
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    wifi_config_t ap_config = {};
    strcpy((char*) ap_config.ap.ssid, WIFI_SSID);
    ap_config.ap.ssid_len = strlen(WIFI_SSID);
    ap_config.ap.authmode = WIFI_AUTH_OPEN;
    ap_config.ap.max_connection = 1;
    ap_config.ap.beacon_interval = 100;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

extern void server_task(void* pvParameters);
extern void server_handle_task(void* pvParameters);

extern "C"
void app_main()
{
    led = new Led(LED_GPIO);
    led->set_color(0, 0, 0);

    pid_mutex = xSemaphoreCreateMutex();
    orientation_mutex = xSemaphoreCreateMutex();
    steering_mutex = xSemaphoreCreateMutex();

    nvs_flash_init();
    load_config();
    apply_config_params();
    pretty_print_config();

    // Parameter calculation & initialization
    pid_reset(0, 0, &pid_settings_arr[ANGLE], &angle_pid_state);
    pid_reset(0, 0, &pid_settings_arr[VEL], &vel_pid_state);
    mahony_filter_init(&imuparams, 10.0f * MAHONY_FILTER_KP, MAHONY_FILTER_KI,
                       2.0 * 2000.0f * M_PI / 180.0f, IMU_SAMPLE_TIME);

    imu = new Imu();
    motor_a = new Motor(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B, GPIO_AENBL_OUT, GPIO_APHASE_OUT);
    motor_b = new Motor(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM1A, MCPWM1B, GPIO_BENBL_OUT, GPIO_BPHASE_OUT);
    battery = new Battery;
#if 0
    auto enc_a = new Encoder(PCNT_UNIT_0, GPIO_ENC_A1, GPIO_ENC_A2);
    auto enc_b = new Encoder(PCNT_UNIT_1, GPIO_ENC_B1, GPIO_ENC_B2);
    static auto encoders = std::make_pair(enc_a, enc_b);
    xTaskCreate(event_task, "event_task", 2048, &encoders, 5, NULL);
#endif

    printf("Press a key to enter console\n");
    bool debug = false;
    for (int i = 0; i < 20; ++i)
    {
        if (getchar() != EOF)
        {
            debug = true;
            break;
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    if (debug)
        run_console();        // never returns
    printf("\nStarting application\n");
    
    wifi_setup();
    ws_server_start();
    xTaskCreate(&battery_task, "Battery task", 2048, NULL, PRIO_COMMUNICATION, NULL);
    xTaskCreate(&server_task, "HTTP Daemon", 3000, NULL, PRIO_COMMUNICATION, NULL);
    xTaskCreate(&server_handle_task, "server_handle_task", 4000, NULL, PRIO_COMMUNICATION, NULL);
    //xTaskCreate(&count_task, "count_task", 6000, NULL, 2, NULL);

    xTaskCreate(&main_loop, "Main loop", 10240, NULL, PRIO_MAIN_LOOP, &xCalculationTask);
#if 0
    xTaskCreate(&steering_watcher, "Steering watcher", 128, NULL, PRIO_MAIN_LOOP + 1, &xSteeringWatcher);
    xTaskCreate(&imu_watcher, "IMU watcher", 128, NULL, PRIO_MAIN_LOOP + 2, &xIMUWatcher);
    xTaskCreate(&maze_solver_task, "Maze solver", 256, NULL, PRIO_COMMUNICATION + 1, NULL);

    gpio_enable(IMU_INTERRUPT_PIN, GPIO_INPUT);
    gpio_set_interrupt(IMU_INTERRUPT_PIN, GPIO_INTTYPE_EDGE_POS, imu_interrupt_handler);
#endif
}
