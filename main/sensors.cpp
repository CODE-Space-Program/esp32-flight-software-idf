#include "sensors.h"
#include "bmp390.h"
#include "mpu6050.h"
#include <esp_log.h>
#include "kalman.h"
#include <cmath>
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "SimpleKalmanFilter.h"
#include "datapoint.h"

static constexpr float SEA_LEVEL_P = 1012.17f;
static constexpr float SEA_LEVEL_PRESSURE = SEA_LEVEL_P * 100.0f;

static mpu6050_handle_t mpu_handle = nullptr;
static bmp390_handle_t bmp_handle = nullptr;
static SimpleKalmanFilter pressureKalmanFilter(0.03f, 1.0f, 0.01f);

static constexpr float DEG_TO_RAD = M_PI / 180.0f;
static constexpr float RAD_To_DEG = 180.0f / M_PI;

static constexpr float G = 9.81f;


SensorManager::SensorManager(EKFManager &ekfmanager)
    : ekfmanager(ekfmanager) {}

esp_err_t SensorManager::init(i2c_master_bus_handle_t bus)
{
    bmp390_config_t bmp_cfg = {
        I2C_BMP390_DEV_ADDR_LO, // 0x76
        I2C_BMP390_DEV_CLK_SPD,
        BMP390_IIR_FILTER_3,
        BMP390_PRESSURE_OVERSAMPLING_4X,
        BMP390_TEMPERATURE_OVERSAMPLING_8X,
        BMP390_ODR_40MS,
        BMP390_POWER_MODE_FORCED};
    ESP_ERROR_CHECK(bmp390_init(bus, &bmp_cfg, &bmp_handle));

    mpu6050_config_t mpu_cfg = {
        I2C_MPU6050_DEV_ADDR_L,
        I2C_MPU6050_DEV_CLK_SPD,
        MPU6050_DIGITAL_LP_FILTER_ACCEL_94KHZ_GYRO_98KHZ,
        MPU6050_GYRO_CS_PLL_X_AXIS_REF,
        MPU6050_GYRO_FS_RANGE_2000DPS, 
        MPU6050_ACCEL_FS_RANGE_16G };

    ESP_ERROR_CHECK(mpu6050_init(bus, &mpu_cfg, &mpu_handle));

    ESP_ERROR_CHECK(calibrateMpu());

    // Log calibration results
    ESP_LOGI("SensorManager", "Gyro Bias: [%f, %f, %f]", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
    ESP_LOGI("SensorManager", "Accel Bias: [%f, %f, %f]", accel_bias[0], accel_bias[1], accel_bias[2]);
    // optional reset/configure interrupts:
    // ESP_ERROR_CHECK( mpu6050_reset(mpu_handle) );
    // ESP_ERROR_CHECK( mpu6050_configure_interrupts(mpu_handle, &mpu_cfg) );
    last_bmp_time = esp_timer_get_time();
    last_mpu_time = esp_timer_get_time();

    ESP_LOGI("SensorManager", "Sensors initialized");
    return ESP_OK;
}

esp_err_t SensorManager::calibrateMpu() {
    if (!mpu_handle) {
        ESP_LOGE("sensor", "MPU6050 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    const int num_samples = 2000;
    float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
    float accel_sum[3] = {0.0f, 0.0f, 0.0f};

    // Declare variables outside the loop
    mpu6050_accel_data_axes_t accel_data;
    mpu6050_gyro_data_axes_t gyro_data;
    float temperature;

    ESP_LOGI("sensor", "Starting MPU6050 calibration...");

    for (int i = 0; i < num_samples; i++) {
        // Read raw data from MPU6050
        esp_err_t err = mpu6050_get_motion(mpu_handle, &gyro_data, &accel_data, &temperature);
        if (err != ESP_OK) {
            ESP_LOGE("sensor", "Failed to read MPU6050 data during calibration: %s", esp_err_to_name(err));
            return err; // Return the error immediately
        }

        // Log raw sensor data
        ESP_LOGD("sensor", "Raw Gyro: [%f, %f, %f], Raw Accel: [%f, %f, %f]",
                 gyro_data.x_axis, gyro_data.y_axis, gyro_data.z_axis,
                 accel_data.x_axis, accel_data.y_axis, accel_data.z_axis);

        // Accumulate gyro and accel data
        gyro_sum[0] += gyro_data.x_axis;
        gyro_sum[1] += gyro_data.y_axis;
        gyro_sum[2] += gyro_data.z_axis;

        accel_sum[0] += accel_data.x_axis;
        accel_sum[1] += accel_data.y_axis;
        accel_sum[2] += accel_data.z_axis;

        // Delay between samples
        vTaskDelay(pdMS_TO_TICKS(1)); // 1 ms delay
    }

    // Calculate biases (average of readings)
    gyro_bias[0] = gyro_sum[0] / num_samples;
    gyro_bias[1] = gyro_sum[1] / num_samples;
    gyro_bias[2] = gyro_sum[2] / num_samples;

    accel_bias[0] = accel_sum[0] / num_samples;
    accel_bias[1] = accel_sum[1] / num_samples;
    accel_bias[2] = accel_sum[2] / num_samples;

    // Adjust Y-axis accelerometer bias to account for gravity (1g)
    accel_bias[1] -= 1.0f;

    ESP_LOGI("sensor", "MPU6050 calibration complete");
    return ESP_OK; // Ensure the function always returns a value
}


esp_err_t SensorManager::readBmp() {
    // Get the current time
    int64_t current_time = esp_timer_get_time();

    // Calculate time delta in seconds
    float dt = (current_time - last_bmp_time) / 1e6f; // Convert microseconds to seconds
    last_bmp_time = current_time; // Update the last read time

    // read bmp390 measurements
    float temperature, pressure, estimated_altitude;

    esp_err_t err = bmp390_get_measurements(bmp_handle, &temperature, &pressure);
    if (err != ESP_OK) {
        ESP_LOGE("SensorManager", "BMP390 read failed: %d", err);
        return err;
    }

    // Calculate altitude
    float raw_altitude = 44330.0f * (1.0f - powf(pressure / SEA_LEVEL_PRESSURE, 0.190263f));
    datapoint.raw_altitude = raw_altitude;
    datapoint.pressure = pressure;
    datapoint.temperature = temperature;

    estimated_altitude = (pressureKalmanFilter.updateEstimate(raw_altitude - 16.0f)); // code 1st floor
    datapoint.estimated_altitude = estimated_altitude;

    ESP_LOGI("SensorManager", "BMP390 Data: Altitude=%.2f m, Pressure=%.2f Pa, Temp=%.2f °C , Estimated Altitude=%.2f",
        datapoint.raw_altitude, datapoint.pressure, datapoint.temperature, datapoint.estimated_altitude);

    return ESP_OK;
}

double ax = 0, ay = 0,az = 0;
double gx = 0, gy = 0, gz = 0;

esp_err_t SensorManager::readMpu() {
    // Get the current time 
    int64_t current_time = esp_timer_get_time();

    // Calculate time delta in seconds
    float dt = (current_time - last_mpu_time) / 1e6f; // Convert microseconds to seconds
    last_mpu_time = current_time; // Update the last read time

    // Read raw accelerometer and gyroscope data
    mpu6050_accel_data_axes_t accelData;
    mpu6050_gyro_data_axes_t gyroData;
    float temperature;

    esp_err_t err = mpu6050_get_motion(mpu_handle, &gyroData, &accelData, &temperature);
    if (err != ESP_OK) {
        ESP_LOGE("SensorManager", "Failed to read MPU6050 data: %d", err);
        return err;
    };

    // raw acceleromter data in g
    float raw_accX = accelData.x_axis;
    float raw_accY = accelData.y_axis;
    float raw_accZ = accelData.z_axis;

    // Scale raw gyroscope data to rad/s
    float raw_gyrX = gyroData.x_axis * DEG_TO_RAD;
    float raw_gyrY = gyroData.y_axis * DEG_TO_RAD;
    float raw_gyrZ = gyroData.z_axis * DEG_TO_RAD;

    // Apply calibration (bias correction)
    float accX_cal = raw_accX - accel_bias[0];
    float accY_cal = raw_accY - accel_bias[1];
    float accZ_cal = raw_accZ - accel_bias[2];

    float gyrX_cal = raw_gyrX;
    float gyrY_cal = raw_gyrY;
    float gyrZ_cal = raw_gyrZ;

    // angles based on accelerometer
    ax = atan2(-accX_cal, sqrt(pow(accY_cal, 2) + pow(accZ_cal, 2))) * RAD_To_DEG;
    ay = atan2(accY_cal, sqrt(pow(accX_cal, 2) + pow(accZ_cal, 2))) * RAD_To_DEG;
    az = atan2(accZ_cal, sqrt(pow(accX_cal, 2) + pow(accY_cal, 2))) * RAD_To_DEG;

    gx = gx + gyrX_cal * dt;
    gy = gy + gyrY_cal * dt;
    gz = gz + gyrZ_cal * dt;

    float alpha = 0.96f; // Complementary filter coefficient

    // Update pitch (gyroscope + accelerometer)
    gx = alpha * (gx + gyrX_cal * dt) + (1 - alpha) * ax;
    
    // Update roll (gyroscope + accelerometer)
    gz = alpha * (gz + gyrZ_cal * dt) + (1 - alpha) * az;

    datapoint.estimated_pitch = gx;
    datapoint.estimated_yaw = gz;

    ESP_LOGI("SensorManager", "MPU6050 Data: Pitch=%.2f, Yaw=%.2f", datapoint.estimated_pitch, datapoint.estimated_yaw);

    ESP_LOGI("SensorManager", "Raw Gyro: [%f, %f, %f], Raw Accel: [%f, %f, %f]",
        raw_gyrX, raw_gyrY, raw_gyrZ,
        raw_accX, raw_accY, raw_accZ);

    ESP_LOGI("SensorManager", "Cal Gyro: [%f, %f, %f], Cal Accel: [%f, %f, %f]",
        gyrX_cal, gyrY_cal, gyrZ_cal,
        accX_cal, accY_cal, accZ_cal);

    /* Remap axes to match EKF's NED frame
    float accX = accX_cal;         // X remains the same
    float accY = -accZ_cal;        // Z becomes Y (inverted)
    float accZ = accY_cal;         // Y becomes Z

    float gyrX = gyrX_cal;         // X remains the same
    float gyrY = -gyrZ_cal;        // Z becomes Y (inverted)
    float gyrZ = gyrY_cal;         // Y becomes Z


    // retrieve gyro bias from ekf
    //float gyro_bias[3];
    //ekfmanager.getState(gyro_bias);

    float gyro[3] = {gyrX, gyrY, gyrZ};


    float accelNorm[3] = {accX, accY, accZ};
    float norm = std::sqrt(accelNorm[0] * accelNorm[0] +
                           accelNorm[1] * accelNorm[1] +
                           accelNorm[2] * accelNorm[2]);

    if (norm == 0.0f) {
        ESP_LOGE("SensorManager", "Accelerometer norm is zero, cannot normalize");
        accelNorm[0] = accelNorm[1] = accelNorm[2] = 0.0f;
    } else {
        accelNorm[0] /= norm;
        accelNorm[1] /= norm;
        accelNorm[2] /= norm;
    }


    // Pass data to EKF
    ekfmanager.processSensorData(accelNorm, gyro, dt);

    // Retrieve pitch and yaw from EKF
    data.estimated_pitch = ekfmanager.getPitch();
    data.estimated_yaw = ekfmanager.getYaw(); */
    
    return ESP_OK;
}

bool SensorManager::allSystemsCheck() {
    return true;
}

bool SensorManager::detectApogee(float current_altitude) {
    static float maxAltitude = 0.0f;
    static bool apogeeDetected = false;

    if (current_altitude > maxAltitude) {
        maxAltitude = current_altitude;
        apogeeDetected = false;
    }

    if (current_altitude < maxAltitude - 1.0 && !apogeeDetected) {
        apogeeDetected = true;
        ESP_LOGI("SensorManager", "Apogee Detected!, Max Altitude: %.2f", maxAltitude);
    }

    datapoint.apogee = maxAltitude;
    return apogeeDetected;
}