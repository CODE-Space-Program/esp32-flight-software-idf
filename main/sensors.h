#pragma once

#include <string>
#include <vector>
#include <functional>

#include "esp_event.h"
#include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "cJSON.h"

#include "ground_control.h"
#include "kalman.h"

class SimpleKalman1D : public ekf {
public:
  SimpleKalman1D() : ekf(2,1) {}
  void Init() override {
    // set initial X (2×1), P (2×2), Q (2×2), R (1×1), F (2×2), G (2×1)
  }
  void LinearizeFG(dspm::Mat &x, float *u) override {
    // for linear model, F and G stay constant
  }
};

class SensorManager
{
public:
    esp_err_t init(i2c_master_bus_handle_t bus);
    esp_err_t read(TelemetryData &out);

private:
    KalmanFilter heightKalmanFilter{0.0f, 0.0f, 0.01f};
    KalmanFilter velocityKalmanFilter{0.0f, 0.0f, 0.01f};

    void initKalman();
};