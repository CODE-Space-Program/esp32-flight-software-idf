#include "servos.h"
#include <algorithm>

Servos::Servos(i2c_master_bus_handle_t bus,
               uint16_t dev_addr,
               uint16_t servoMinCount,
               uint16_t servoMaxCount)
    : bus(bus), addr(dev_addr), minCount(servoMinCount), maxCount(servoMaxCount)
{
}

Servos::~Servos()
{
    deinitialize();
}

esp_err_t Servos::initialize()
{
    i2c_master_dev_handle_t pca_dev;
    i2c_device_config_t dev_cfg = {
        .device_address = addr, // e.g. 0x40
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &pca_dev));

    _pca = new PCA9685(pca_dev, pca_dev, 50, false, 0);
    _pca->SwitchAllOff();
    _pca->Refresh();
    return ESP_OK;
}

esp_err_t Servos::move(uint8_t channel, float angle)
{
    if (!_pca)
        return ESP_ERR_INVALID_STATE;

    // map 0–180° → [minCount..maxCount]
    float a = std::clamp(angle, 0.0f, 180.0f);
    uint16_t pulse = uint16_t((a / 180.0f) * (maxCount - minCount) + minCount);

    // set on=0, off=pulse
    _pca->SetPwm(channel, 0, pulse);
    _pca->Refresh();
    return ESP_OK;
}

void Servos::deinitialize()
{
    if (_pca)
    {
        delete _pca;
        _pca = nullptr;
    }
}
