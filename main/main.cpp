#include "ground_control.h"
#include <nvs_flash.h>
#include <esp_netif.h>
#include <esp_event.h>

#include "wifi_helper.h"
#include "tvc_test.h"
#include "servos.h"

static const char* TAG = "app_main";

static int tvcTestTick = 0;

static TelemetryData mockTelemetry {
    "Ready",      // state
    12345678L,        // time (ms)
    1013.25f,         // pressure (mbar)
    22.5f,            // temperature (°C)
    150.0f,           // raw_altitude
    145.0f,           // estimated_altitude
    5.0f,             // velocity
    2.5f,             // estimated_pitch
    -1.0f,            // estimated_yaw
    0.0f,             // estimated_roll
    160.0f,           // apogee
    '\0',             // null_terminator (uses default anyway)
    90.0f,            // nominalYawServoDegrees
    45.0f,            // nominalPitchServoDegrees
    false             // servosLocked
};

extern "C" void app_main()
{

    ESP_LOGI(TAG, "Starting main application");

    connectWifi();

    ESP_LOGI(TAG, "Connected to WiFi");

    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();

    ESP_LOGI(TAG, "Initialized event loop");

    GroundControl gc("https://spaceprogram.bolls.dev");
    TvcTest tvcTest;

    gc.connect();

    gc.sendTelemetry(mockTelemetry);

    ESP_LOGI(TAG, "Sent telemetry");

    Servos servos((i2c_port_t) 0, (gpio_num_t) 0, (gpio_num_t) 0, 0, 110, 480);

    servos.initialize();

    ESP_LOGI(TAG, "Initialized servos");

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
        else if (cmd == "zero_tvc"){
            ESP_LOGI(TAG, "Received zero_tvc command");

            servos.move(0, 90);
            servos.move(1, 90);
        }
        else {
            ESP_LOGI(TAG, "Unhandled command: %s", cmd.c_str());
        }
    });

    while (true) {
        if (tvcTest.isInProgress()) {
            ESP_LOGI(TAG, "Tvc test in progress");

            tvcTestTick++;

            if (tvcTestTick % 100 == 0) {
                float newPitch = tvcTest.getNewPitch();
                float newYaw   = tvcTest.getNewYaw();
                servos.move(0, newPitch);
                servos.move(1, newYaw);
            }
            // 1ms delay to simulate Arduino's fast loop rate
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        // not in progress: you could lower CPU use here
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
