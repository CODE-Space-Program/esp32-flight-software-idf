#include "driver/gpio.h"
#include "ground_control.h"
#include <nvs_flash.h>
#include <esp_netif.h>
#include <esp_event.h>
#include "esp_wifi.h"

#include "wifi_helper.h"
#include "tvc_test.h"
#include "servos.h"
#include "sensors.h"
#include "kalman.h"
#include "pyro.h"
#include "tvc.h"
#include "flight_controller.h"
#include "datapoint.h"

// Global variables
static int tvcTestTick = 0;
static i2c_master_bus_handle_t bus = nullptr;
static constexpr gpio_num_t LED_GPIO = GPIO_NUM_2;
static TickType_t lastLEDToggleTick = 0;
static const TickType_t LED_TOGGLE_INTERVAL = pdMS_TO_TICKS(500); // 500ms interval

void init_led() {
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

void led_toggle() {
    int level = gpio_get_level(LED_GPIO);
    gpio_set_level(LED_GPIO, !level);
}

void telemetryTask(void *arg) {
    GroundControl *gc = static_cast<GroundControl *>(arg);
    TickType_t lastTelemetryTick = xTaskGetTickCount();

    while (true) {
        TickType_t now = xTaskGetTickCount();
        if (now - lastTelemetryTick >= pdMS_TO_TICKS(100)) {
            gc->sendTelemetry(datapoint);
            lastTelemetryTick = now;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

extern "C" void app_main() {

    init_led();
    led_on();
    // SECTION 1: Initialization
    ESP_LOGI("main", "Starting main application");

    // Initialize NVS (Non-Volatile Storage)
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // Initialize ESP networking and event loop
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_LOGI("main", "Initialized event loop");


    err = connectWifi();
    if (err != ESP_OK) {
        ESP_LOGE("main", "Failed to connet to WiFi: %s", esp_err_to_name(err));
        return; // abort if Wi-Fi fails to connect
    }

    // Configure WiFi storage to RAM (avoids reconnect issues)
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_LOGI("main", "Connected to WiFi");

    led_off();

    // Initialize Ground Control communication
    GroundControl gc("https://spaceprogram.bolls.dev");

    // Initialize TVC test object
    TvcTest tvcTest;

    // Configure I2C bus
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = (i2c_port_t)0,
        .sda_io_num = (gpio_num_t)21,
        .scl_io_num = (gpio_num_t)22,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 0,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = 0
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    // Initialize sensors and EKF (Extended Kalman Filter)
    EKFManager ekfManager;
    ekfManager.init();
    SensorManager sensors(ekfManager);
    sensors.init(bus);

    // Connect Ground Control
    gc.connect();

    // Initialize servos
    Servos servos(bus, 0x40, 150, 600);

    // Initialize TVC (Thrust Vector Control)
    Tvc tvc(servos, 0, 1, 4.0f);
    ESP_LOGI("main", "Initialized TVC");

    // Initialize flight controller
    FlightController flightController(tvc, sensors);
    flightController.initialize();

    xTaskCreatePinnedToCore(
        telemetryTask,
        "TelemetryTask",
        4096,
        &gc,
        tskIDLE_PRIORITY + 1,
        NULL,
        1
    );

    gc.subscribe([&](const std::string &cmd, cJSON *args) {
        if (cmd == "start") {
            ESP_LOGI("main", "Received start command");
            ascendingMotorIgnite();
            flightController.setState(FlightState::Flight);

        } else if (cmd == "zero_tvc") {
            servos.move(0, 90);
            servos.move(1, 90);
        }
    });

    // SECTION 2: Main Loop
    while (true) {
        // Update flight controller
        flightController.update();

        if (flightController.getState() == FlightState::PreLaunch) {
            TickType_t now = xTaskGetTickCount();
            if (now - lastLEDToggleTick >= LED_TOGGLE_INTERVAL) {
                led_toggle();
                lastLEDToggleTick = now;
            }
            
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Handle TVC test logic
        if (tvcTest.isInProgress()) {
            ESP_LOGI("main", "TVC test in progress");

            tvcTestTick++;
            if (tvcTestTick % 100 == 0) {
                float newPitch = tvcTest.getNewPitch();
                float newYaw = tvcTest.getNewYaw();

                ESP_LOGI("main", "  Pitch: %.1f°, Yaw: %.1f°", newPitch, newYaw);

                servos.move(0, newPitch - 90);
                servos.move(1, newYaw - 90);
            }

            // 1ms delay for fast loop rate
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        // Delay to reduce CPU usage when no TVC test is in progress
        //vTaskDelay(pdMS_TO_TICKS(100));
    }
}