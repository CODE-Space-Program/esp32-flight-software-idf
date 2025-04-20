#include "ground_control.h"
#include <nvs_flash.h>
#include <esp_netif.h>
#include <esp_event.h>

#include "wifi_helper.h"
#include "tvc_test.h"
#include "servos.h"

static const char* TAG = "app_main";

extern "C" void app_main()
{
    connectWifi();

    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();

    GroundControl gc("https://spaceprogram.bolls.dev");
    TvcTest tvcTest;

    Servos servos((i2c_port_t) 0, (gpio_num_t) 0, (gpio_num_t) 0, 0, 110, 480);

    gc.subscribe([&](const std::string &cmd, cJSON *args) {
        if (args) {
            char *args_str = cJSON_PrintUnformatted(args);
            if (args_str) {
                ESP_LOGI(TAG, "  Args: %s", args_str);
                cJSON_free(args_str);
            }
        }
        if (cmd == "start") {
            ESP_LOGI(TAG, "Received start command");
        }
        else if (cmd == "test_tvc") {
            ESP_LOGI(TAG, "Received test_tvc command");

            // default values
            float maxDegrees  = 20.0f;
            float stepDegrees = 10.0f;
            int   duration    = 5000;

            if (args) {
                cJSON *j;

                j = cJSON_GetObjectItem(args, "maxDegrees");
                if (cJSON_IsNumber(j)) maxDegrees = (float)j->valuedouble;

                j = cJSON_GetObjectItem(args, "stepDegrees");
                if (cJSON_IsNumber(j)) stepDegrees = (float)j->valuedouble;

                j = cJSON_GetObjectItem(args, "duration");
                if (cJSON_IsNumber(j)) duration = j->valueint;
            }

            ESP_LOGI(TAG, "  TVC test params: max=%.1f°, step=%.1f°, duration=%dms",
                    maxDegrees, stepDegrees, duration);

            tvcTest.start(maxDegrees, stepDegrees, duration);
        }
        else {
            ESP_LOGI(TAG, "Unhandled command: %s", cmd.c_str());
        }
    });
    gc.connect();

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
