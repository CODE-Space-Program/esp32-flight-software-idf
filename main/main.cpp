#include "ground_control.h"
#include <nvs_flash.h>
#include <esp_netif.h>
#include <esp_event.h>
#include "esp_wifi.h"

#include "wifi_helper.h"
#include "tvc_test.h"
#include "servos.h"

#include "driver/gpio.h"

#include "sensors.h"

static constexpr gpio_num_t LED_GPIO = GPIO_NUM_2;

void init_led()
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << LED_GPIO);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    gpio_set_level(LED_GPIO, 0);
}

void led_on() { gpio_set_level(LED_GPIO, 1); }

void led_off() { gpio_set_level(LED_GPIO, 0); }

void led_toggle()
{
    int level = gpio_get_level(LED_GPIO);
    gpio_set_level(LED_GPIO, !level);
}

static const char *TAG = "app_main";

static int tvcTestTick = 0;

static TelemetryData mockTelemetry{
    "Ready",   // state
    12345678L, // time (ms)
    1013.25f,  // pressure (mbar)
    22.5f,     // temperature (°C)
    150.0f,    // raw_altitude
    145.0f,    // estimated_altitude
    5.0f,      // velocity
    2.5f,      // estimated_pitch
    -1.0f,     // estimated_yaw
    0.0f,      // estimated_roll
    160.0f,    // apogee
    '\0',      // null_terminator (uses default anyway)
    90.0f,     // nominalYawServoDegrees
    45.0f,     // nominalPitchServoDegrees
    false      // servosLocked
};

static i2c_master_bus_handle_t bus = nullptr;
static SensorManager sensors;

extern "C" void app_main()
{

    init_led();
    led_on();

    ESP_LOGI(TAG, "Starting main application");

    connectWifi();

    ESP_LOGI(TAG, "Connected to WiFi");

    led_off();

    nvs_flash_init();

    // don't carry over wifi connection info between reboots. this seems to fix an issue where every wifi connection takes longer than the previous one,
    // due to repeatedly failing with "wifi: Disconnected-reconnecting". eventually, this leads to not being able to connect at all.
    // this issue has been observed on the "CODE University" WiFi. For Ava's mobile hotspot, it always connects instantly.
    esp_wifi_set_storage(WIFI_STORAGE_RAM);

    esp_netif_init();
    esp_event_loop_create_default();

    ESP_LOGI(TAG, "Initialized event loop");

    GroundControl gc("https://spaceprogram.bolls.dev");
    TvcTest tvcTest;

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = (i2c_port_t)0,
        .sda_io_num = (gpio_num_t)21,
        .scl_io_num = (gpio_num_t)22,
        .clk_source = I2C_CLK_SRC_DEFAULT,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    SensorManager *sensors = new SensorManager;
    sensors->init(bus);
    sensors->read(mockTelemetry);

    gc.connect();

    TickType_t lastTelemetryTick = xTaskGetTickCount();

    ESP_LOGI(TAG, "Sent telemetry");

    Servos servos(bus, 0x40, 110, 480);

    servos.initialize();

    ESP_LOGI(TAG, "Initialized servos");

    servos.move(0, 90);
    servos.move(1, 90);

    gc.subscribe([&](const std::string &cmd, cJSON *args)
                 {
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
        } });

    while (true)
    {
        TickType_t now = xTaskGetTickCount();
        if (now - lastTelemetryTick >= pdMS_TO_TICKS(100))
        {
            gc.sendTelemetry(mockTelemetry);
            lastTelemetryTick = now;
        }

        if (tvcTest.isInProgress())
        {
            ESP_LOGI(TAG, "Tvc test in progress");

            tvcTestTick++;

            if (tvcTestTick % 100 == 0)
            {
                float newPitch = tvcTest.getNewPitch();
                float newYaw = tvcTest.getNewYaw();

                ESP_LOGI(TAG, "  Pitch: %.1f°, Yaw: %.1f°", newPitch, newYaw);

                servos.move(0, newPitch - 90);
                servos.move(1, newYaw - 90);
            }
            // 1ms delay to simulate Arduino's fast loop rate
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        // not in progress: you could lower CPU use here
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
