#include "servos.h"
#include <algorithm>
#include <esp_log.h>

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

esp_err_t Servos::initialize() {
    if (_pca) {
        ESP_LOGW("Servos", "Servos already initialized.");
        return ESP_ERR_INVALID_STATE;
    }

    i2c_master_dev_handle_t pca_dev;
    i2c_device_config_t dev_cfg = {
        .device_address = addr,
        .scl_speed_hz = 400000, // Ensure this matches the I2C bus speed
    };

    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &pca_dev);
    if (err != ESP_OK) {
        ESP_LOGE("Servos", "Failed to add I2C device: %s", esp_err_to_name(err));
        return err;
    }

    _pca = new PCA9685(pca_dev, pca_dev, 50, false, 0);
    _pca->SwitchAllOff();
    _pca->Refresh();
    ESP_LOGI("Servos", "Servos initialized successfully.");
    return ESP_OK;
}


esp_err_t Servos::move(uint8_t channel, float angle) {
    if (!_pca) {
        ESP_LOGE("Servos", "Servo not initialized. Call initialize() first.");
        return ESP_ERR_INVALID_STATE;
    }

    float a = std::clamp(angle, 0.0f, 180.0f);
    uint16_t pulse = uint16_t((a / 180.0f) * (maxCount - minCount) + minCount);

    _pca->SetPwm(channel, 0, pulse);
    _pca->Refresh();
    ESP_LOGI("Servos", "Moved servo on channel %d to angle %.2f (pulse %d).", channel, angle, pulse);
    return ESP_OK;
}


void Servos::deinitialize()
{
    if (!_pca) {
        ESP_LOGW("Servos", "Servo already deinitialized.");
        return;
    }
    delete _pca;
    _pca = nullptr;
}