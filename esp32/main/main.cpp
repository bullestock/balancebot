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

//////// PID //////////

#define MAX_PID_OUTPUT 500

float BASE_Kp = 100.0, BASE_Ki = 5.0, BASE_Kd = 130.0;
float Kp = BASE_Kp, Ki = BASE_Ki, Kd = BASE_Kd;
float angleSetpoint = 0, selfBalanceAngleSetpoint = 0;
float pidOutput, pidError, pidLastError, integralErr, positionErr, serialControlErr, prevSerialControlErr, errorDerivative;

float MAX_CONTROL_OR_POSITION_ERR = MAX_PID_OUTPUT / Kp;
float MAX_CONTROL_ERR_INCREMENT = MAX_CONTROL_OR_POSITION_ERR / 400;
#define MIN_CONTROL_ERR 1

long long loop_timer;
long long print_timer;

float roll = 0.0, pitch = 0.0, rollAcc = 0.0, pitchAcc = 0.0;
float speeed = 0.0;

int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
int16_t gyroX_calibration, gyroY_calibration, gyroZ_calibration;

long long prevSpeedStart;
int16_t prevSpeed;
int32_t currentPos = 0;

int16_t constr(int16_t value, int16_t mini, int16_t maxi) {
  if(value < mini) return mini;
  else if(value > maxi) return maxi;
  return value;
}
float constrf(float value, float mini, float maxi) {
  if(value < mini) return mini;
  else if(value > maxi) return maxi;
  return value;
}

#define MAX_SPEED 20000

void setSpeed(int16_t s, int16_t rotation) {
  int16_t sL = s - rotation;
  int16_t sR = s + rotation;
  
  if(sL > MAX_SPEED) sL = MAX_SPEED;
  if(sL < -MAX_SPEED) sL = -MAX_SPEED;
  if(sR > MAX_SPEED) sR = MAX_SPEED;
  if(sR < -MAX_SPEED) sR = -MAX_SPEED;

  // keep track of the position (in steps forward or backward)
  currentPos += ((esp_timer_get_time() - prevSpeedStart) / (float)1000000)  * prevSpeed;
  prevSpeed = s;
  prevSpeedStart = esp_timer_get_time();

  set_motors(double(sL)/MAX_SPEED, double(sR)/MAX_SPEED);
}

#define PERIOD  4000    // loop period in micros
#define PRINT_PERIOD  100000    // print period in micros

#define GYRO_SCALE_FACTOR             131     // LSB / (degs per seconds)
#define ACC_SCALE_FACTOR              65536    // LSB / g

const double RAD_TO_DEG = 57.295779513082320876798154814105;

static float GYRO_RAW_TO_DEGS = 1.0 / (1000000.0 / PERIOD) / GYRO_SCALE_FACTOR;

void main_loop(void* pvParameters)
{
    const auto start_tick = esp_timer_get_time();
    loop_timer = start_tick + PERIOD;
    print_timer = start_tick + PRINT_PERIOD;
    printf("Start main loop at %lld\n", start_tick);

    //led->set_color(Led::Init);
    while (1)
    {
        xTaskNotifyWait(0, 0, NULL, 1); // allow other tasks to run
        
        // 700 us
        int16_t raw_data[6];
        if (!imu->read_raw_data(raw_data))
        {
            printf("Reading IMU failed\n");
            continue;
        }

        accX = raw_data[0];
        accY = raw_data[1];
        accZ = raw_data[2];
        printf("acc %d %d %d\n", accX, accY, accZ);
        rollAcc = asin((float) accX / ACC_SCALE_FACTOR) * RAD_TO_DEG;
        pitchAcc = asin((float) accY / ACC_SCALE_FACTOR) * RAD_TO_DEG;

        gyroX = raw_data[3];
        gyroY = raw_data[4];
        gyroZ = raw_data[5];

        roll -= gyroY * GYRO_RAW_TO_DEGS;
        pitch += gyroX * GYRO_RAW_TO_DEGS;

        roll = roll * 0.999 + rollAcc * 0.001;
        pitch = pitch * 0.999 + pitchAcc * 0.001;

        printf("roll %f %f pitch %f %f\n", rollAcc, roll, pitchAcc, pitch);
        
        positionErr = constrf(currentPos / (float)1000, -MAX_CONTROL_OR_POSITION_ERR, MAX_CONTROL_OR_POSITION_ERR);

        pidError = pitch - angleSetpoint - selfBalanceAngleSetpoint;

        pidError += positionErr;  

        integralErr = constrf(integralErr + Ki * pidError, -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
        errorDerivative = pidError - pidLastError;
  
        pidOutput = Kp * pidError + integralErr + Kd * errorDerivative;

        if(pidOutput < 5 && pidOutput > -5) pidOutput = 0;  //Create a dead-band to stop the motors when the robot is balanced

        if(pitch > 30 || pitch < -30) {    //If the robot tips over
            pidOutput = 0;     
            integralErr = 0; 
            selfBalanceAngleSetpoint = 0; 
        }
  
        // store error for next loop
        pidLastError = pidError;

        int16_t rotation = 0;

        setSpeed(constrf(pidOutput, -MAX_PID_OUTPUT, MAX_PID_OUTPUT) * (MAX_SPEED / MAX_PID_OUTPUT), rotation);
 
        // The angle calculations are tuned for a loop time of PERIOD milliseconds. 
        // To make sure every loop is exactly that, a wait loop is created by setting the loop_timer 
        const auto now = esp_timer_get_time();
#if 0
        if(loop_timer <= now)
            printf("ERROR: Loop too short: %lld/%lld\n", loop_timer, now);
#endif
        while(loop_timer > esp_timer_get_time());
        loop_timer += PERIOD;

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
