#pragma once

// Hardware config

// Constant indices, don't change
#define IMU_X_AXIS 0
#define IMU_Y_AXIS 1
#define IMU_Z_AXIS 2

// Select these parameters to match the mounting orientation of the IMU
// The IMU axis pointing to the forward direction of the robot
#define IMU_FORWARD_AXIS        IMU_Z_AXIS
// If the positive axis specified above points backward, set this to true
#define IMU_INVERT_FORWARD_AXIS false
#define IMU_INVERT_SIDE_AXIS    true
// The IMU axis pointing horizontally to the "side" of the robot when it is standing
#define IMU_SIDE_AXIS           IMU_Y_AXIS

// PID tunings
// -----------
// Note that these will be overridden by the values saved in flash
// when configured through HTML page

// Angle PID == the PID controller which regulates motor output signal to reach
// the target angle
#define ANGLE_KP 1.0
#define ANGLE_KI 1.0
#define ANGLE_KD 0.1
// Velocity PID == the PID controller which regulates target angle to reach
// the target velocity
#define VEL_KP 2.0
#define VEL_KI 0.5
#define VEL_KD 0.002

// Optional motor drive deadband -- set to 0 to disable
#define MOTOR_DEADBAND 0.0

// Determine how sensitive the robot is to steering
#define STEERING_FACTOR 0.13
#define SPEED_CONTROL_FACTOR 0.67

// Exponential decay parameters
#define TRAVEL_SPEED_SMOOTHING 0.002
#define TARGET_SPEED_SMOOTHING 0.001

// IMU filter parameters
#define MAHONY_FILTER_KP 2.0
#define MAHONY_FILTER_KI 0.01
#define IMU_SAMPLE_TIME 0.001 // (1.0f / 1660.0f)

// The robot will let the sensor fusion settle for
// a defined number of milliseconds
#define ORIENTATION_STABILIZE_DURATION 2000

// Angle limits - sin(alpha)
#define FALL_LIMIT     0.75  // Robot considered fallen
#define ROLL_LIMIT     0.75  // Robot considered fallen (roll direction)
#define RECOVER_LIMIT  0.2   // Robot considered recovered
#define HIGH_PID_LIMIT 0.2   // Limit to boost balance control

#define STABLE_ANGLE   0.0   // Default target angle -- won't matter much as it's adaptive

// Default (hardcoded) gyro calibration offsets
#define GYRO_X_OFFSET -27
#define GYRO_Y_OFFSET -89
#define GYRO_Z_OFFSET 14

// If we don't see reaction to motor control in this time,
// consider the robot fallen
#define WINDUP_TIMEOUT 1000

// Undervoltage cutoff check
#define BATTERY_THRESHOLD 7.4
#define BATTERY_CHECK_INTERVAL 500
#define ENABLE_BATTERY_CHECK true
#define ENABLE_BATTERY_CUTOFF false

#define WIFI_SSID "ESPway"
#define WIFI_CHANNEL 1

// Maze solver config

#define ULTRASONIC_SENSOR_SIDE_GPIO  1
#define ULTRASONIC_SENSOR_FRONT_GPIO 5

// Handedness of the solver and mounting side of the sensor
// 0 is left, 1 is right
#define MAZE_SOLVER_RIGHT_HANDED 1
