#pragma once

#include "driver/i2c_master.h"
#include "pca9685.hpp"
#include "esp_err.h"

class Servos {
public:
    Servos(i2c_master_bus_handle_t bus,
           uint16_t dev_addr,
           uint16_t servoMinCount,
           uint16_t servoMaxCount);

    ~Servos();

    esp_err_t initialize();
    esp_err_t move(uint8_t channel, float angle);
    void      deinitialize();

private:
    PCA9685* _pca    = nullptr;

    i2c_master_bus_handle_t bus = nullptr;
    uint16_t   addr;
    uint16_t   minCount, maxCount;
};
