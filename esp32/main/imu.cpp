#include "imu.h"

#include <driver/i2c.h>

#include "lsm6ds3_driver.h"

#include <cstring>

#define I2C_MASTER_SCL_IO          GPIO_NUM_18
#define I2C_MASTER_SDA_IO          GPIO_NUM_19
#define I2C_MASTER_NUM             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ         1000000

#define IMU_ADDR 0x6a

#define LSM6DS3_WHO_AM_I 0x0f
#define LSM6DS3_WHO_AM_I_VALUE 0x69

#define INT1_CTRL 0x0d
#define INT1_DRDY_G (0x01 << 1)

#define CTRL1_XL 0x10
#define BW_XL_200Hz 0x01
#define ODR_XL_1K66 (0x08 << 4)
#define ODR_XL_833 (0x07 << 4)

#define CTRL2_G 0x11
#define FS_G_2000_DPS (0x3 << 2)
#define ODR_G_1K66 (0x08 << 4)
#define ODR_G_833 (0x07 << 4)

#define OUTX_L_G 0x22
#define OUTX_L_XL 0x28


struct imu_register_value_t
{
    uint8_t address;
    uint8_t value;
};

static const imu_register_value_t LSM6DS3_CONFIG_VALUES[] = {
    { INT1_CTRL,  INT1_DRDY_G },
    { CTRL1_XL,   ODR_XL_1K66 },
    { CTRL2_G,    FS_G_2000_DPS | ODR_G_1K66 }
};


Imu::Imu()
{
    // Configure I2C

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(I2C_MASTER_NUM, &conf);
    assert(!i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0,
                               0)); //!! interrupt flag ESP_INTR_FLAG_
    
    assert(lsm6ds3_i2c_init(I2C_MASTER_NUM, IMU_ADDR) == 0);
       
    uint8_t data = 0;
    assert(lsm6ds3_i2c_read_chipid(I2C_MASTER_NUM, IMU_ADDR, &data) == 0);
    assert(data == LSM6DS3_WHO_AM_I_VALUE);

    const int N = 6;
    uint8_t tmp[N];
    assert(lsm6ds3_i2c_read_accel(I2C_MASTER_NUM, IMU_ADDR, tmp) == 0);
}

bool Imu::read_raw_data(int16v3& accel, int16v3& gyro)
{
    const int N = 6;
    uint8_t tmp[N];
    // 2 ms
    assert(lsm6ds3_i2c_read_accel(I2C_MASTER_NUM, IMU_ADDR, tmp) == 0);
    memcpy(&accel[0], tmp, N);

    assert(lsm6ds3_i2c_read_gyro(I2C_MASTER_NUM, IMU_ADDR, tmp) == 0);
    memcpy(&gyro[0], tmp, N);

    return true;
}

