#include "servos.h"
#include <algorithm>

Servos::Servos(i2c_port_t port,
               gpio_num_t sda,
               gpio_num_t scl,
               uint16_t dev_addr,
               uint16_t servoMinCount,
               uint16_t servoMaxCount)
    : port(port)
    , sda_io(sda)
    , scl_io(scl)
    , addr(dev_addr)
    , minCount(servoMinCount)
    , maxCount(servoMaxCount)
{}

Servos::~Servos() {
    deinitialize();
}

esp_err_t Servos::initialize() {
    // create I2C handle for PCA9685
    esp_err_t err = PCA9685::NewI2cHandle(port, sda_io, scl_io, addr, &_handle);
    if (err != ESP_OK) return err;

    // construct driver at 50 Hz (servo rate), no inversion
    _pca = new PCA9685(_handle, nullptr, 50, false, 0);
    _pca->SwitchAllOff();
    _pca->Refresh();
    return ESP_OK;
}

esp_err_t Servos::move(uint8_t channel, float angle) {
    if (!_pca) return ESP_ERR_INVALID_STATE;

    // map 0–180° → [minCount..maxCount]
    float a = std::clamp(angle, 0.0f, 180.0f);
    uint16_t pulse = uint16_t((a/180.0f)*(maxCount - minCount) + minCount);

    // set on=0, off=pulse
    _pca->SetPwm(channel, 0, pulse);
    _pca->Refresh();
    return ESP_OK;
}

void Servos::deinitialize() {
    if (_pca) {
        delete _pca;
        _pca = nullptr;
    }
    _handle = nullptr; // driver stays up for other uses
}
