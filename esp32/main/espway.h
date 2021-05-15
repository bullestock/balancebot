#pragma once

#include <cstdint>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/task.h>
#include "esp_event.h"

#include "pid.h"

#include "config.h"

enum ws_msg_t
{
    STEERING = 0,

    REQ_ORIENTATION = 1,
    RES_ORIENTATION = 2,

    BATTERY = 3,
    BATTERY_CUTOFF = 4,

    REQ_SET_PID_PARAMS = 5,
    RES_SET_PID_PARAMS_ACK = 6,
    REQ_GET_PID_PARAMS = 7,
    RES_PID_PARAMS = 8,

    REQ_SET_GYRO_OFFSETS = 9,
    RES_SET_GYRO_OFFSETS_FAILURE = 10,
    RES_SET_GYRO_OFFSETS_SUCCESS = 11,

    REQ_SAVE_CONFIG = 12,
    RES_SAVE_CONFIG_FAILURE = 13,
    RES_SAVE_CONFIG_SUCCESS = 14,

    REQ_CLEAR_CONFIG = 15,
    RES_CLEAR_CONFIG_FAILURE = 16,
    RES_CLEAR_CONFIG_SUCCESS = 17,

    REQ_LOAD_FLASH_CONFIG = 18,
    RES_LOAD_FLASH_CONFIG_DONE = 19,

    REQ_ENABLE_MOTORS = 20,
    REQ_DISABLE_MOTORS = 21,

    REQ_GET_STATS = 22,
    RES_GET_STATS = 23,

    REQ_GET_STATUS = 24,
    RES_GET_STATUS = 25,
};

struct espway_config
{
    pid_coeffs pid_coeffs_arr[2];
    int16_t gyro_offsets[3];
};

enum pid_controller_index
{
    ANGLE = 0,
    VEL
};

struct orientation
{
    double sin_pitch;
    double sin_roll;
};

enum state { STABILIZING_ORIENTATION, RUNNING, FALLEN, WOUND_UP };

extern espway_config my_config;

extern SemaphoreHandle_t pid_mutex;
extern pidsettings pid_settings_arr[2];

void pretty_print_config();
void apply_config_params();
bool save_flash_config();
bool clear_flash_config();
void load_config();
void load_hardcoded_config();
void update_pid_controller(pid_controller_index idx, double p, double i, double d);

void httpd_task(void* pvParameters);

void battery_cutoff();

state get_state();
void set_steering(double new_target_speed, double new_turning_bias);
orientation get_orientation();

