#include "sensors.h"
#include "bmp390.h"
#include "mpu6050.h"
#include <esp_log.h>
#include <cmath>

static const char *TAG = "SensorManager";
static constexpr float SEA_LEVEL_P       = 1012.17f;
static constexpr float SEA_LEVEL_PRESSURE = SEA_LEVEL_P * 100.0f; // Pa

mpu6050_handle_t mpu_handle = nullptr;
bmp390_handle_t bmp_handle = nullptr;

esp_err_t SensorManager::init(i2c_port_t i2c_num,
                              gpio_num_t sda_pin,
                              gpio_num_t scl_pin)
{
    // 1) install the low‑level I2C driver
    i2c_config_t conf{};
    conf.mode           = I2C_MODE_MASTER;
    conf.sda_io_num     = sda_pin;
    conf.scl_io_num     = scl_pin;
    conf.master.clk_speed = 100000;
    ESP_ERROR_CHECK(i2c_param_config(i2c_num, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_num, conf.mode, 0, 0, 0));

    // 2) BMP390 init (same as before) …
    //    (omit here for brevity)

    // 3) MPU6050 init
    mpu_handle = mpu6050_create(i2c_num, MPU6050_I2C_ADDRESS);
    if (mpu_handle == nullptr) {
        ESP_LOGE(TAG, "MPU6050 create failed");
        return ESP_FAIL;
    }
    ESP_ERROR_CHECK( mpu6050_wake_up(mpu_handle) );
    ESP_ERROR_CHECK( mpu6050_config(mpu_handle, ACCE_FS_8G, GYRO_FS_500DPS) );

    ESP_LOGI(TAG, "Sensors initialized");
    return ESP_OK;
}

esp_err_t SensorManager::read(TelemetryData &out)
{
    // timestamp
    out.time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // --- read BMP390 ---
    float temperature, pressure;
    esp_err_t err = bmp390_get_measurements(bmp_handle, &temperature, &pressure);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BMP390 read failed (%d)", err);
        return err;
    }
    out.raw_altitude = 44330.0f * (1.0f - powf(pressure / SEA_LEVEL_PRESSURE, 0.190263f));

    // --- read MPU6050 ---
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    ESP_ERROR_CHECK( mpu6050_get_acce(mpu_handle, &acce) );
    ESP_ERROR_CHECK( mpu6050_get_gyro(mpu_handle, &gyro) );

    float ax = acce.acce_x, ay = acce.acce_y, az = acce.acce_z;
    float pitch_acc = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;
    static float pitch_gyro = pitch_acc;
    static uint32_t last = out.time;
    uint32_t now = out.time;
    float dt = (now - last) * 0.001f;
    last = now;
    pitch_gyro += gyro.gyro_x * dt;
    out.estimated_pitch = 0.96f * pitch_gyro + 0.04f * pitch_acc;

    // TODO: fill out yaw, roll, velocity, apogee…

    return ESP_OK;
}
