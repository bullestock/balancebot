#include <stdio.h>
#include <utility>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <driver/ledc.h>
#include <driver/pcnt.h>

#include <lwip/sockets.h>

#include "esp_wifi.h"
#include "sdkconfig.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"

#include "websocket_server.h"

#include "espway.h"
#include "encoder.h"
#include "imu.h"
#include "imu_math.h"
#include "led.h"
#include "locks.h"
#include "motor.h"

#define LED_GPIO 13
#define LED_PIN 2 // internal

#define GPIO_PWM0A_OUT 16
#define GPIO_PWM0B_OUT 17
#define GPIO_PWM1A_OUT 18
#define GPIO_PWM1B_OUT 19

#define GPIO_ENC_A1   22
#define GPIO_ENC_A2   23
#define GPIO_ENC_B1    2
#define GPIO_ENC_B2   15

#define GREEN   0, 255, 0
#define BLUE    0, 0, 255

#define PRIO_COMMUNICATION  2
#define PRIO_MAIN_LOOP      (TCPIP_THREAD_PRIO + 1)

Led* led = nullptr;
Motor* motor_a = nullptr;
Motor* motor_b = nullptr;
Imu* imu = nullptr;

static ledc_channel_config_t ledc_channel;

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

static mahony_filter_state imuparams;
static pidstate vel_pid_state;
static pidstate angle_pid_state;

SemaphoreHandle_t pid_mutex;
pidsettings pid_settings_arr[2];

static double target_speed = 0;
static double steering_bias = 0;

static SemaphoreHandle_t orientation_mutex;
static vector3d gravity = {{ 0.0, 0.0, 1.0 }};

enum log_mode { LOG_FREQ, LOG_RAW, LOG_PITCH, LOG_NONE };

constexpr auto LOGMODE = LOG_NONE;

#define ORIENTATION_STABILIZE_DURATION_US ((ORIENTATION_STABILIZE_DURATION) * 1000)
#define WINDUP_TIMEOUT_US ((WINDUP_TIMEOUT) * 1000)

static double sin_pitch = 0.0, sin_roll = 0.0;

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

// -1 to +1
void set_motors(double m1, double m2)
{
    motor_a->set_speed(m1);
    motor_b->set_speed(-m2);
}

void main_loop(void* pvParameters)
{
    TickType_t time_old = 0;
    TickType_t current_time = 0;
    int n = 0;
    double smoothed_target_speed = 0;
    double travel_speed = 0;
    state my_state = STABILIZING_ORIENTATION;
    TickType_t stage_started = 0;
    TickType_t last_wind_up = 0;

    int loopcount = 0;
#define SHOW_DEBUG() ((my_state == RUNNING) && (loopcount == 0))
    
    while (1)
    {
        ++loopcount;
        if (loopcount >= 10)
            loopcount = 0;
        
        //vTaskDelay(1/portTICK_PERIOD_MS);

        //xTaskNotifyWait(0, 0, NULL, 1);
        int16_t raw_data[6];
        if (!imu->read_raw_data(raw_data))
        {
            printf("Reading IMU failed with\n");
            continue;
        }
        
        // Update orientation estimate
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

            // Upright ~ 0
            if (SHOW_DEBUG())
                printf("PITCH %f ROLL %f\n", sin_pitch, sin_roll);
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

        if (my_state == STABILIZING_ORIENTATION &&
            elapsed_time_us(current_time, stage_started) > ORIENTATION_STABILIZE_DURATION_US)
        {
            printf("Done stabilizing\n");
            my_state = RUNNING;
            stage_started = current_time;
            imuparams.Kp = MAHONY_FILTER_KP;
        }
        else if (my_state == RUNNING || my_state == WOUND_UP)
        {
            if (SHOW_DEBUG())
                printf("ap %f ar %f\n", fabs(sin_pitch - STABLE_ANGLE), fabs(sin_roll));
            if (fabs(sin_pitch - STABLE_ANGLE) < FALL_LIMIT &&
                fabs(sin_roll) < ROLL_LIMIT)
            {
                // Perform PID update
                double target_angle, motor_speed;
                {
                    MutexLock lock(pid_mutex);
                    if (SHOW_DEBUG())
                        printf("PID update 1: ts %f ss %f\n", travel_speed, smoothed_target_speed);
                    target_angle = pid_compute(travel_speed, smoothed_target_speed,
                                               &pid_settings_arr[VEL], &vel_pid_state,
                                               SHOW_DEBUG());

                    if (sin_pitch < (target_angle - HIGH_PID_LIMIT))
                        motor_speed = -1.0;
                    else if (sin_pitch > target_angle + HIGH_PID_LIMIT)
                        motor_speed = 1.0;
                    else
                        motor_speed = pid_compute(sin_pitch, target_angle,
                                                  &pid_settings_arr[ANGLE], &angle_pid_state,
                                                  SHOW_DEBUG());
                    if (SHOW_DEBUG())
                        printf("PID update: ta %f sp %f bias %f\n", target_angle, motor_speed, motor_bias);
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
                else if (elapsed_time_us(current_time, last_wind_up) > WINDUP_TIMEOUT_US)
                {
                    printf("WOUND UP!\n");
                    my_state = WOUND_UP;
                    led->set_color(BLUE);
                    vTaskDelay(10000/portTICK_PERIOD_MS);
                }

                // Estimate travel speed by exponential smoothing
                travel_speed = exponential_smooth(travel_speed, motor_speed,
                                                  TRAVEL_SPEED_SMOOTHING);

                if (SHOW_DEBUG())
                    printf("trav %f target %f\n", travel_speed, smoothed_target_speed);
            }
            else
            {
                printf("FALLEN!\n");
                my_state = FALLEN;
                led->set_color(BLUE);
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
            led->set_color(GREEN);
            pid_reset(sin_pitch, 0, &pid_settings_arr[ANGLE], &angle_pid_state);
            pid_reset(0, STABLE_ANGLE, &pid_settings_arr[VEL],
                      &vel_pid_state);
        }

        if (LOGMODE == LOG_FREQ)
        {
            n += 1;
            if (n == 1024)
            {
                n = 0;
                auto looptime = elapsed_time_us(current_time, time_old);
                printf("Looptime: %u us\n", looptime);
                time_old = current_time;
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

// sets up the led for pwm
static void led_duty(uint16_t duty) {
  static uint16_t val;
  static uint16_t max = (1L<<10)-1;
  if(duty > 100) return;
  val = (duty * max) / 100;
  ledc_set_duty(ledc_channel.speed_mode,ledc_channel.channel,val);
  ledc_update_duty(ledc_channel.speed_mode,ledc_channel.channel);
}

static void led_setup()
{
  ledc_timer_config_t ledc_timer = {
    LEDC_HIGH_SPEED_MODE,
    LEDC_TIMER_10_BIT,
    LEDC_TIMER_0,
    5000
  };

  ledc_channel.channel = LEDC_CHANNEL_0;
  ledc_channel.duty = 0;
  ledc_channel.gpio_num = LED_PIN,
  ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
  ledc_channel.timer_sel = LEDC_TIMER_0;

  ledc_timer_config(&ledc_timer);
  ledc_channel_config(&ledc_channel);
  led_duty(0);
}

extern void server_task(void* pvParameters);
extern void server_handle_task(void* pvParameters);

extern "C"
void app_main()
{
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

    wifi_setup();
    led_setup();
    ws_server_start();
    xTaskCreate(&server_task, "server_task", 3000, NULL, 9, NULL);
    xTaskCreate(&server_handle_task, "server_handle_task", 4000, NULL, 6, NULL);
    //xTaskCreate(&count_task, "count_task", 6000, NULL, 2, NULL);

    //!!xTaskCreate(&httpd_task, "HTTP Daemon", 256, NULL, PRIO_COMMUNICATION, NULL);
#if 0
    xTaskCreate(&main_loop, "Main loop", 256, NULL, PRIO_MAIN_LOOP, &xCalculationTask);
    xTaskCreate(&steering_watcher, "Steering watcher", 128, NULL, PRIO_MAIN_LOOP + 1, &xSteeringWatcher);
    xTaskCreate(&imu_watcher, "IMU watcher", 128, NULL, PRIO_MAIN_LOOP + 2, &xIMUWatcher);
    xTaskCreate(&maze_solver_task, "Maze solver", 256, NULL, PRIO_COMMUNICATION + 1, NULL);

    gpio_enable(IMU_INTERRUPT_PIN, GPIO_INPUT);
    gpio_set_interrupt(IMU_INTERRUPT_PIN, GPIO_INTTYPE_EDGE_POS, imu_interrupt_handler);
#endif
    
    led = new Led(LED_GPIO);

    motor_a = new Motor(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B, GPIO_PWM0A_OUT, GPIO_PWM0B_OUT);
    motor_b = new Motor(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM1A, MCPWM1B, GPIO_PWM1A_OUT, GPIO_PWM1B_OUT);
#if 0
    static auto motors = std::make_pair(motor_a, motor_b);
    xTaskCreate(motor_test_task, "motor_task", 2048, &motors, 5, NULL);
#endif
    
#if 0
    auto enc_a = new Encoder(PCNT_UNIT_0, GPIO_ENC_A1, GPIO_ENC_A2);
    auto enc_b = new Encoder(PCNT_UNIT_1, GPIO_ENC_B1, GPIO_ENC_B2);
    static auto encoders = std::make_pair(enc_a, enc_b);
    xTaskCreate(event_task, "event_task", 2048, &encoders, 5, NULL);
#endif
    
#if 0
    imu = new Imu();

    xTaskCreate(main_loop, "Main loop", 10240, nullptr, PRIO_MAIN_LOOP, nullptr);
#endif
}
