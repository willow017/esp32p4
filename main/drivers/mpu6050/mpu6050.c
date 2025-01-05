/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "unity.h"
#include "driver/i2c.h"
#include "mpu6050.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

#define I2C_MASTER_SCL_IO GPIO_NUM_5      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO GPIO_NUM_4      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

#define ALPHA                       0.99f        /*!< Weight of gyroscope */
#define RAD_TO_DEG                  57.27272727f /*!< Radians to degrees */
#define SAMPLE_RATE 100                              // 数据采样频率，单位为 Hz
#define I2C_MASTER_TIMEOUT_MS       1000

typedef struct {
    float pitch; // 俯仰角
    float roll;  // 翻滚角
    float yaw;   // 偏航角
} imu_angles_t;

// 低通滤波器的 alpha 值
#define ALPHA 0.96

// 全局变量
float gyro_yaw = 0.0f; // 用于累积偏航角

static const char *TAG = "mpu6050 test";
static mpu6050_handle_t mpu6050 = NULL;
TaskHandle_t mpu6050TaskHandle = NULL;

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;
/**
 * @brief i2c master initializationz
 */
static void i2c_bus_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_I2C_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

/**
 * @brief Read a sequence of bytes from a MPU6050 sensor registers
 */
static esp_err_t mpu6050_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to a MPU6050 sensor register
 */
static esp_err_t mpu6050_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}
/**
 * @brief Write two byte to a MPU6050 sensor register
 */
static esp_err_t mpu6050_register_write_two_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data, uint8_t data1)
{
    uint8_t write_buf[3] = {reg_addr, data1<<3, data<<3};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t mpu6050_wakeup(i2c_master_dev_handle_t dev_handle)
{
    esp_err_t ret;
    uint8_t tmp[1];
    ret = mpu6050_register_read(dev_handle, MPU6050_PWR_MGMT_1, tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    tmp[0] &= (~BIT6);
    ret = mpu6050_register_write_byte(dev_handle, MPU6050_PWR_MGMT_1, tmp[0]);
    return ret;
}

/**
 * @brief i2c master initialization
 */
static void i2c_sensor_mpu6050_init(void)
{
    esp_err_t ret;

    i2c_bus_init(&bus_handle, &dev_handle);


    mpu6050_register_write_two_byte(dev_handle, MPU6050_GYRO_CONFIG, ACCE_FS_4G, GYRO_FS_500DPS);

    ret = mpu6050_wakeup(dev_handle);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

esp_err_t mpu6050_get_acce_sensitivitys(i2c_master_dev_handle_t dev_handle, float *const acce_sensitivity)
{
    esp_err_t ret;
    uint8_t acce_fs;
    ret = mpu6050_register_read(dev_handle, MPU6050_ACCEL_CONFIG, &acce_fs, 1);
    acce_fs = (acce_fs >> 3) & 0x03;
    switch (acce_fs) {
    case ACCE_FS_2G:
        *acce_sensitivity = 16384;
        break;

    case ACCE_FS_4G:
        *acce_sensitivity = 8192;
        break;

    case ACCE_FS_8G:
        *acce_sensitivity = 4096;
        break;

    case ACCE_FS_16G:
        *acce_sensitivity = 2048;
        break;

    default:
        break;
    }
    return ret;
}

esp_err_t mpu6050_get_raw_acces(i2c_master_dev_handle_t dev_handle, mpu6050_raw_acce_value_t *const raw_acce_value)
{
    uint8_t data_rd[6];
    esp_err_t ret = mpu6050_register_read(dev_handle, MPU6050_ACCEL_XOUT_H, data_rd, sizeof(data_rd));

    raw_acce_value->raw_acce_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
    raw_acce_value->raw_acce_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
    raw_acce_value->raw_acce_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));
    return ret;
}

esp_err_t mpu6050_get_acces(i2c_master_dev_handle_t dev_handle, mpu6050_acce_value_t *const acce_value)
{
    esp_err_t ret;
    float acce_sensitivity;
    mpu6050_raw_acce_value_t raw_acce;

    ret = mpu6050_get_acce_sensitivitys(dev_handle, &acce_sensitivity);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = mpu6050_get_raw_acces(dev_handle, &raw_acce);
    if (ret != ESP_OK) {
        return ret;
    }

    acce_value->acce_x = raw_acce.raw_acce_x / acce_sensitivity;
    acce_value->acce_y = raw_acce.raw_acce_y / acce_sensitivity;
    acce_value->acce_z = raw_acce.raw_acce_z / acce_sensitivity;
    return ESP_OK;
}

esp_err_t mpu6050_get_temps(i2c_master_dev_handle_t sensor, mpu6050_temp_value_t *const temp_value)
{
    uint8_t data_rd[2];
    esp_err_t ret = mpu6050_register_read(sensor, MPU6050_TEMP_XOUT_H, data_rd, sizeof(data_rd));
    temp_value->temp = (int16_t)((data_rd[0] << 8) | (data_rd[1])) / 340.00 + 36.53;
    return ret;
}

esp_err_t mpu6050_get_gyro_sensitivitys(i2c_master_dev_handle_t sensor, float *const gyro_sensitivity)
{
    esp_err_t ret;
    uint8_t gyro_fs;
    ret = mpu6050_register_read(sensor, MPU6050_GYRO_CONFIG, &gyro_fs, 1);
    gyro_fs = (gyro_fs >> 3) & 0x03;
    switch (gyro_fs) {
    case GYRO_FS_250DPS:
        *gyro_sensitivity = 131;
        break;

    case GYRO_FS_500DPS:
        *gyro_sensitivity = 65.5;
        break;

    case GYRO_FS_1000DPS:
        *gyro_sensitivity = 32.8;
        break;

    case GYRO_FS_2000DPS:
        *gyro_sensitivity = 16.4;
        break;

    default:
        break;
    }
    return ret;
}

esp_err_t mpu6050_get_raw_gyros(i2c_master_dev_handle_t sensor, mpu6050_raw_gyro_value_t *const raw_gyro_value)
{
    uint8_t data_rd[6];
    esp_err_t ret = mpu6050_register_read(sensor, MPU6050_GYRO_XOUT_H, data_rd, sizeof(data_rd));

    raw_gyro_value->raw_gyro_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
    raw_gyro_value->raw_gyro_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
    raw_gyro_value->raw_gyro_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));

    return ret;
}

esp_err_t mpu6050_get_gyros(i2c_master_dev_handle_t sensor, mpu6050_gyro_value_t *const gyro_value)
{
    esp_err_t ret;
    float gyro_sensitivity;
    mpu6050_raw_gyro_value_t raw_gyro;

    ret = mpu6050_get_gyro_sensitivitys(sensor, &gyro_sensitivity);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = mpu6050_get_raw_gyros(sensor, &raw_gyro);
    if (ret != ESP_OK) {
        return ret;
    }

    gyro_value->gyro_x = raw_gyro.raw_gyro_x / gyro_sensitivity;
    gyro_value->gyro_y = raw_gyro.raw_gyro_y / gyro_sensitivity;
    gyro_value->gyro_z = raw_gyro.raw_gyro_z / gyro_sensitivity;
    return ESP_OK;
}

// 计算俯仰角、翻滚角和偏航角
void compute_angles(float ax, float ay, float az, float gx, float gy, float gz, imu_angles_t *angles)
{
    // 基于加速度计计算俯仰角和翻滚角
    float pitch_acc = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
    float roll_acc = atan2(ay, az) * RAD_TO_DEG;

    // 陀螺仪增量角度
    float dt = 1.0 / SAMPLE_RATE; // 时间间隔
    float pitch_gyro = angles->pitch + gx * dt;
    float roll_gyro = angles->roll + gy * dt;
    gyro_yaw += gz * dt;

    // 通过加速度计和陀螺仪融合计算俯仰角和翻滚角
    angles->pitch = ALPHA * pitch_gyro + (1 - ALPHA) * pitch_acc;
    angles->roll = ALPHA * roll_gyro + (1 - ALPHA) * roll_acc;

    // 偏航角由陀螺仪积分计算
    angles->yaw = gyro_yaw;
}

static void mpu6050_main(void *pvParameters)
{
    esp_err_t ret;
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    mpu6050_temp_value_t temp;
    imu_angles_t angles = {0};
    uint8_t data[2];

    i2c_sensor_mpu6050_init();
    ESP_ERROR_CHECK(mpu6050_register_read(dev_handle, MPU6050_WHO_AM_I, data, 1));
    ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

    ret = mpu6050_get_acces(dev_handle, &acce);
    ESP_LOGI(TAG, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f\n", acce.acce_x, acce.acce_y, acce.acce_z);

    ret = mpu6050_get_gyros(dev_handle, &gyro);
    ESP_LOGI(TAG, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f\n", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);

    ret = mpu6050_get_temps(dev_handle, &temp);
    ESP_LOGI(TAG, "t:%.2f \n", temp.temp);

    while (1)
    {
        ret = mpu6050_get_acces(dev_handle, &acce);
        ESP_LOGI(TAG, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f\n", acce.acce_x, acce.acce_y, acce.acce_z);

        ret = mpu6050_get_gyros(dev_handle, &gyro);
        ESP_LOGI(TAG, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f\n", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);

        compute_angles(acce.acce_x,acce.acce_y,acce.acce_z,gyro.gyro_x, gyro.gyro_y, gyro.gyro_z, &angles);
        ESP_LOGI(TAG,"Pitch: %.2f, Roll: %.2f, Yaw: %.2f\n", angles.pitch, angles.roll, angles.yaw);

        vTaskDelay(pdMS_TO_TICKS(1000 / SAMPLE_RATE));
    }
    
}

bool mpu6050_init(void)
{
    xTaskCreatePinnedToCore(mpu6050_main, "mpu6050_main", 4096*2, NULL, 4, &mpu6050TaskHandle, 1);
    return 1;
}