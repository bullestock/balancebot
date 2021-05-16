#pragma once

#include <cstdint>

#include "imu_math.h"

class Imu
{
public:
    Imu();

    bool read_raw_data(int16v3& accel, int16v3& gyro);
};
