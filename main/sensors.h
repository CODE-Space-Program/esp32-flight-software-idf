#pragma once

#include <string>
#include <vector>
#include <functional>

#include "esp_event.h"
#include "driver/i2c.h"
#include "cJSON.h"

#include "ground_control.h"

class SensorManager
{
public:
    esp_err_t init(i2c_port_t i2c_num = I2C_NUM_0, gpio_num_t sda_pin = GPIO_NUM_21, gpio_num_t scl_pin = GPIO_NUM_22);
    esp_err_t read(TelemetryData &out);

private:
    // todo: kalman
};
    