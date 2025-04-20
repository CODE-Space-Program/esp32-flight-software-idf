#include "ground_control.h"
#include <nvs_flash.h>
#include <esp_netif.h>
#include <esp_event.h>

// #include "wifi_helper.h"

static const char* TAG = "app_main";

extern "C" void app_main()
{
    // connectWifi();

    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();

    GroundControl gc("https://spaceprogram.bolls.dev");

    gc.subscribe([](const std::string &cmd, cJSON *args) {
        if (cmd == "start") {
            ESP_LOGI(TAG, "Received start command");
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
