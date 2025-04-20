#pragma once

#include "driver/i2c_master.h"
#include "pca9685.hpp"
#include "esp_err.h"

class Servos {
public:
    Servos(i2c_port_t port,
           gpio_num_t sda,
           gpio_num_t scl,
           uint16_t dev_addr,
           uint16_t servoMinCount,
           uint16_t servoMaxCount);

    ~Servos();

    esp_err_t initialize();
    esp_err_t move(uint8_t channel, float angle);
    void      deinitialize();

private:
    i2c_master_dev_handle_t _handle = nullptr;
    PCA9685*                _pca    = nullptr;

    i2c_port_t port;
    gpio_num_t sda_io;
    gpio_num_t scl_io;
    uint16_t   addr;
    uint16_t   minCount, maxCount;
};
