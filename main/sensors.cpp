#include "sensors.h"
#include "bmp390.h"
#include "mpu6050.h"
#include <esp_log.h>
#include <cmath>
#include "driver/i2c_master.h"  

static const char *TAG = "SensorManager";
static constexpr float SEA_LEVEL_P        = 1012.17f;
static constexpr float SEA_LEVEL_PRESSURE = SEA_LEVEL_P * 100.0f;

static mpu6050_handle_t mpu_handle = nullptr;
static bmp390_handle_t bmp_handle = nullptr;

esp_err_t SensorManager::init(i2c_master_bus_handle_t bus) {
    // 1) new master bus
    

    // 2) BMP390 stays the same
    bmp390_config_t bmp_cfg = {
        // I2C_BMP390_DEV_ADDR_HI,
        0x76,
        I2C_BMP390_DEV_CLK_SPD,
        BMP390_IIR_FILTER_OFF,
        BMP390_PRESSURE_OVERSAMPLING_8X,
        BMP390_TEMPERATURE_OVERSAMPLING_8X,
        BMP390_ODR_40MS,
        BMP390_POWER_MODE_FORCED
    };
    ESP_ERROR_CHECK( bmp390_init(bus, &bmp_cfg, &bmp_handle) );

    // 3) MPU6050 via new API
    mpu6050_config_t mpu_cfg = I2C_MPU6050_CONFIG_DEFAULT;
    ESP_ERROR_CHECK( mpu6050_init(bus, &mpu_cfg, &mpu_handle) );
    // optional reset/configure interrupts:
    // ESP_ERROR_CHECK( mpu6050_reset(mpu_handle) );
    // ESP_ERROR_CHECK( mpu6050_configure_interrupts(mpu_handle, &mpu_cfg) );

    ESP_LOGI(TAG, "Sensors initialized");
    return ESP_OK;
}


esp_err_t SensorManager::read(TelemetryData &out)
{
    // timestamp
    out.time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // --- BMP390: read temperature & pressure ---
    float temperature, pressure;
    esp_err_t err = bmp390_get_measurements(bmp_handle, &temperature, &pressure);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BMP390 read failed: %d", err);
        return err;
    }
    // barometric formula to meters
    out.raw_altitude = 44330.0f *
        (1.0f - powf(pressure / SEA_LEVEL_PRESSURE, 0.190263f));

    // --- MPU6050: read accelerometer & gyro ---
    mpu6050_accel_data_axes_t acce;
    mpu6050_gyro_data_axes_t  gyro;
    float                     temp;
    esp_err_t mpuErr = mpu6050_get_motion(mpu_handle, &gyro, &acce, &temp);
    if (mpuErr != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 read failed: %d", mpuErr);
        return mpuErr;
    }

    // complementary filter for pitch
    float ax = 0;//acce.acce_x;
    float ay = 0;//acce.acce_y;
    float az = 0;//acce.acce_z;
    float gyro_x = 0;//gyro.gyro_x;


    float pitch_acc = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;
    static float pitch_gyro = pitch_acc;
    static uint32_t last = out.time;
    uint32_t now = out.time;
    float dt = (now - last) * 0.001f;
    last = now;
    pitch_gyro += gyro_x * dt;
    out.estimated_pitch = 0.96f * pitch_gyro + 0.04f * pitch_acc;

    // TODO: yaw, roll, velocity, apogee…

    return ESP_OK;
}
