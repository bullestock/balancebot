#pragma once

#include <cstdint>

class Imu
{
public:
    Imu();

    bool read_raw_data(int16_t* data);
    
};
