#include "imu.h"

#include <driver/i2c.h>

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
    printf("Imu ctor\n");
    //imu_i2c_init(0);

    const auto i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, 0, 0,
                       0); //!! interrupt flag

  //!!
  // ret = imu_send_config(IMU_ADDR, LSM6DS3_CONFIG_VALUES,
  //                       sizeof(LSM6DS3_CONFIG_VALUES) / sizeof(LSM6DS3_CONFIG_VALUES[0]));
  // if (ret != 0)
  // {
  //   return ret;
  // }

    uint8_t whoami = 0;
    //ret = imu_read_register(IMU_ADDR, LSM6DS3_WHO_AM_I, &whoami);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (IMU_ADDR << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, LSM6DS3_WHO_AM_I, 1);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
    if (ret != 0)
    {
        printf("step 1: err %d\n", ret);
        return;
    }
    i2c_cmd_link_delete(cmd);
    
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (IMU_ADDR << 1) | I2C_MASTER_READ, 1);
    i2c_master_read_byte(cmd, &whoami, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != 0)
    {
        printf("step 2: err %d\n", ret);
        return;
    }
    printf("whoami %d\n", whoami);
    if (whoami != LSM6DS3_WHO_AM_I_VALUE)
    {
        printf("whoami: expected %d, got %d\n", LSM6DS3_WHO_AM_I_VALUE, whoami);
    }
}
