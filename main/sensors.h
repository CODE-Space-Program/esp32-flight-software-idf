#pragma once

#include <string>
#include <vector>
#include <functional>

#include "esp_event.h"
#include "driver/i2c.h"
#include "cJSON.h"

#include "ground_control.h"

#include "driver/i2c_master.h"

class SensorManager
{
public:
    esp_err_t init(i2c_master_bus_handle_t bus);
    esp_err_t read(TelemetryData &out);

private:
    // todo: kalman
};
    