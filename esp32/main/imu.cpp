#include "imu.h"

#include <driver/i2c.h>

#include <cstring>

#define I2C_MASTER_SCL_IO          GPIO_NUM_14
#define I2C_MASTER_SDA_IO          GPIO_NUM_27
#define I2C_MASTER_NUM             I2C_NUM_1
#define I2C_MASTER_FREQ_HZ         100000

#define IMU_SAMPLE_TIME (1.0f / 1660.0f)
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
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0,
                       0); //!! interrupt flag ESP_INTR_FLAG_

    // Configure IMU parameters
    
    for (const auto& e : LSM6DS3_CONFIG_VALUES)
    {
        auto cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (IMU_ADDR << 1) | I2C_MASTER_WRITE, 1);
        i2c_master_write_byte(cmd, e.address, 1);
        i2c_master_write_byte(cmd, e.value, 1);
        i2c_master_stop(cmd);
        const auto ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        assert(ret == 0);
    }

    // Check that the WHO_AM_I register reports the correct value
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (IMU_ADDR << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, LSM6DS3_WHO_AM_I, 1);
    i2c_master_stop(cmd);
    auto ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    assert(ret == 0);
    
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (IMU_ADDR << 1) | I2C_MASTER_READ, 1);
    uint8_t whoami = 0;
    i2c_master_read_byte(cmd, &whoami, I2C_MASTER_ACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    assert(ret == 0);
    assert(whoami == LSM6DS3_WHO_AM_I_VALUE);

}

bool Imu::read_raw_data(int16_t* data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (IMU_ADDR << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, OUTX_L_G, 1);
    i2c_master_stop(cmd);
    auto ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != 0)
        return false;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (IMU_ADDR << 1) | I2C_MASTER_READ, 1); // 1 or 0, no difference
    const int N = 12;
    uint8_t tmp[N];
    for (int i = 0; i < N - 1; ++i)
        i2c_master_read_byte(cmd, tmp + i, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, tmp + N - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != 0)
        return false;

    // for (int i = 0; i < sizeof(tmp)/sizeof(tmp[0]); ++i)
    //     printf("IMU %d %d\n", i, tmp[i]);

    auto my_data = (uint8_t*) data;
    memcpy(&my_data[6], &tmp[0], 6);
    memcpy(&my_data[0], &tmp[6], 6);
    /*
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (IMU_ADDR << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, LSM6DS3_WHO_AM_I, 1);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    assert(ret == 0);
    
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (IMU_ADDR << 1) | I2C_MASTER_READ, 1);
    uint8_t whoami = 0;
    i2c_master_read_byte(cmd, &whoami, I2C_MASTER_ACK); // Works with both I2C_MASTER_ACK and I2C_MASTER_NACK
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    assert(ret == 0);
    printf("WHO %d (%d)\n", whoami, LSM6DS3_WHO_AM_I_VALUE);
    */
    return true;
}

