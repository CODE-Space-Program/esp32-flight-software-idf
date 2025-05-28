#pragma once

#include <string>
#include <vector>
#include <functional>

#include "esp_event.h"
#include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "cJSON.h"
#include "kalman.h"

class SensorManager
{
public:
    SensorManager(EKFManager &ekfmanager);

    esp_err_t init(i2c_master_bus_handle_t bus);
    esp_err_t calibrateMpu();
    esp_err_t readMpu();
    esp_err_t readBmp();

    bool allSystemsCheck();
    bool detectApogee(float current_altitude);


private:
    EKFManager &ekfmanager;

    float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
    float accel_bias[3] = {0.0f, 0.0f, 0.0f};

    int64_t last_bmp_time;
    int64_t last_mpu_time;
};