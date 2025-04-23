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
    bmp390_config_t bmp_cfg = {
        // I2C_BMP390_DEV_ADDR_HI,
        0x76,
        I2C_BMP390_DEV_CLK_SPD,
        BMP390_IIR_FILTER_3,
        BMP390_PRESSURE_OVERSAMPLING_4X,
        BMP390_TEMPERATURE_OVERSAMPLING_8X,
        BMP390_ODR_40MS,
        BMP390_POWER_MODE_FORCED
    };
    ESP_ERROR_CHECK( bmp390_init(bus, &bmp_cfg, &bmp_handle) );

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
    // — timestamp & dt —
    out.time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    static uint32_t lastTime = 0;
    uint32_t now = out.time;
    float dt = (now - lastTime) * 0.001f; // ms → s
    lastTime = now;

    // — BMP390 → raw + Kalman‑filtered altitude —
    float temperature, pressure;
    esp_err_t err = bmp390_get_measurements(bmp_handle, &temperature, &pressure);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BMP390 read failed: %d", err);
        return err;
    }
    out.raw_altitude = 44330.0f * (1.0f - powf(pressure / SEA_LEVEL_PRESSURE, 0.190263f));
    

    // — MPU6050 accel + gyro —
    mpu6050_gyro_data_axes_t  gyro;
    mpu6050_accel_data_axes_t acce;
    float                     tempC;
    esp_err_t mpuErr = mpu6050_get_motion(mpu_handle, &gyro, &acce, &tempC);
    if (mpuErr != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 read failed: %d", mpuErr);
        return mpuErr;
    }

    // — apply your Arduino‑calibration offsets —
    const float ACC_X_OFF =  0.41f;
    const float ACC_Y_OFF = -0.14f;
    const float ACC_Z_OFF = -0.12f;
    const float YAW_BIAS  =  1.7f;
    const float G         =  9.80665f;

    float ax = acce.x_axis - ACC_X_OFF;
    float ay = acce.y_axis - ACC_Y_OFF;
    float az = acce.z_axis - ACC_Z_OFF;

    float gx = gyro.x_axis;
    float gy = gyro.y_axis;
    float gz = gyro.z_axis;

    // — accel‑based angles —
    float angX = atan2f(-ax, sqrtf(ay*ay + az*az)) * (180.0f/M_PI);
    float angZ = atan2f( az, sqrtf(ax*ax + ay*ay)) * (180.0f/M_PI);

    // — integrate gyro rates —
    static float iGX = 0, iGY = 0, iGZ = 0;
    iGX += gx * dt;
    iGY += gy * dt;
    iGZ += gz * dt;

    // — complementary filter on X (yaw) & Z (pitch) axes —
    iGX = iGX * 0.96f + angX * 0.04f;
    iGZ = iGZ * 0.96f + angZ * 0.04f;
    iGZ -= YAW_BIAS;

    out.estimated_yaw   = iGX;
    out.estimated_pitch = iGZ;
    out.estimated_roll  = iGY;

    // — Y‑axis accel → velocity Kalman —
    float raw_vel_ms2 = acce.y_axis * G;
    // out.estimated_altitude = heightKalmanFilter.updateEstimate(out.raw_altitude - 54.0f);
    // out.velocity = velocityKalmanFilter.updateEstimate(raw_vel_ms2);

    // dsps_kalman_f32_predict(&heightKalmanFilter, raw_acceleration_ms2);
    // dsps_kalman_f32_predict(&velocityKalmanFilter, raw_acceleration_ms2);

    // out.estimated_altitude = dsps_kalman_f32_update(&heightKalmanFilter, out.raw_altitude);
    // out.velocity           = dsps_kalman_f32_update(&velocityKalmanFilter, raw_acceleration_ms2);

    return ESP_OK;
}


void SensorManager::initKalman()
{
    const float dt = 0.01f;  // your loop period
    // Q (process noise): [0.001, 0.0005], R (measurement noise): 0.02, P0 (init cov): 1
    // dsps_kalman_f32_init(&heightKalmanFilter, dt, 0.001f, 0.0005f, 0.02f, 1.0f);
    // dsps_kalman_f32_init(&velocityKalmanFilter, dt, 0.001f, 0.0005f, 0.02f, 1.0f);
}