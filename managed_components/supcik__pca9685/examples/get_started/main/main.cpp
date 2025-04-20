/**
 ******************************************************************************
 * @file        : main.cpp
 * @brief       : Getting Started App
 * @author      : Jacques Supcik <jacques@supcik.net>
 * @date        : 3 April 2024
 ******************************************************************************
 * @copyright   : Copyright (c) 2024 Jacques Supcik
 * @attention   : SPDX-License-Identifier: MIT
 ******************************************************************************
 * @details
 *
 ******************************************************************************
 */

#include <inttypes.h>
#include <stdio.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pca9685.hpp"
#include "sdkconfig.h"

static const char* kTag = "main";

extern "C" {
void app_main(void);
}

void app_main(void) {
    ESP_LOGI(kTag, "Starting PCA9685 example");

    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(PCA9685::NewI2cHandle(I2C_NUM_0, GPIO_NUM_18, GPIO_NUM_16, 0x40, &dev_handle));
    PCA9685 pca9685(dev_handle);
    pca9685.SwitchAllOff();
    pca9685.Refresh();

    int pos = 0;
    while (1) {
        pca9685.SwitchAllOff();
        pca9685.SwitchOn(pos);
        pca9685.Refresh();
        vTaskDelay(pdMS_TO_TICKS(100));
        pos = (pos + 1) % 16;
    }
}
