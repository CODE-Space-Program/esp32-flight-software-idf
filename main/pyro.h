#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "pyro";

#define ASCENDING_MOTOR_IGNITION_PIN   GPIO_NUM_5
#define DESCENDING_MOTOR_IGNITION_PIN  GPIO_NUM_33

static bool landingMotorIgnited = false;

void pyroInit()
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<ASCENDING_MOTOR_IGNITION_PIN) | (1ULL<<DESCENDING_MOTOR_IGNITION_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // ensure both are low
    gpio_set_level(ASCENDING_MOTOR_IGNITION_PIN, 0);
    gpio_set_level(DESCENDING_MOTOR_IGNITION_PIN, 0);

    ESP_LOGI(TAG, "Pyro channels initialized");
}

// quickly pulse the ascending motor ignitor
void ascendingMotorIgnite()
{
    ESP_LOGI(TAG, "IGNITE ASCENDING motor");
    gpio_set_level(ASCENDING_MOTOR_IGNITION_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(ASCENDING_MOTOR_IGNITION_PIN, 0);
    ESP_LOGI(TAG, "Ascending Motor ignited");
}

// quickly pulse the descending (landing) motor ignitor
void descendingMotorIgnite()
{
    ESP_LOGI(TAG, "IGNITE DESCENDING motor");
    gpio_set_level(DESCENDING_MOTOR_IGNITION_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(DESCENDING_MOTOR_IGNITION_PIN, 0);
    ESP_LOGI(TAG, "Descending motor ignited");
}

// call regularly (e.g. from your main loop/task)
void fireLandingBurn(const TelemetryData &datapoint)
{
    float fireLandingMotorAlt = 0.6f * datapoint.apogee;

    if (datapoint.estimated_altitude <= fireLandingMotorAlt
        && !landingMotorIgnited) {
        descendingMotorIgnite();
        landingMotorIgnited = true;
    }
}
