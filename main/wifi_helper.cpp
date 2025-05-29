#include <cstring>

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"

static EventGroupHandle_t wifi_event_group;
static const int CONNECTED_BIT = BIT0;

#define WIFI_SSID "Vodafone-70C0"
#define WIFI_PASS "34h6aMT8NUPJrtct"

// Event handler to catch start, disconnect & got‑IP events
static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGW("wifi", "Disconnected—reconnecting");
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI("wifi", "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
    }
}

void cleanupWifi() {
    ESP_LOGI("wifi", "Cleaning up Wi-Fi");

    // Stop Wi-Fi
    if (esp_wifi_stop() != ESP_OK) {
        ESP_LOGW("wifi", "Wi-Fi stop failed or not initialized.");
    }

    // Deinitialize Wi-Fi
    if (esp_wifi_deinit() != ESP_OK) {
        ESP_LOGW("wifi", "Wi-Fi deinit failed or not initialized.");
    }

    // Check if default Wi-Fi STA netif exists before destroying
    if (esp_netif_get_handle_from_ifkey("WIFI_STA_DEF")) {
        esp_netif_destroy_default_wifi(NULL);
    } else {
        ESP_LOGW("wifi", "No default Wi-Fi STA netif to destroy.");
    }
}



esp_err_t connectWifi() {
    static bool wifi_initialized = false;

    if (wifi_initialized) {
        ESP_LOGW("wifi", "Wi-Fi already initialized, skipping!");
        return ESP_OK;
    }

    // Check if the default STA netif already exists
    if (esp_netif_get_handle_from_ifkey("WIFI_STA_DEF")) {
        ESP_LOGW("wifi", "Default Wi-Fi STA netif already exists. Skipping creation.");
    } else {
        // Create default Wi-Fi STA netif
        esp_netif_t *netif = esp_netif_create_default_wifi_sta();
        if (!netif) {
            ESP_LOGE("wifi", "Failed to create default Wi-Fi STA.");
            return ESP_FAIL;
        }
    }

    // Initialize Wi-Fi driver
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE("wifi", "Wi-Fi initialization failed: %s", esp_err_to_name(err));
        return err;
    }

    wifi_initialized = true;

    // Register event handlers
    wifi_event_group = xEventGroupCreate();
    esp_event_handler_instance_t inst_any_id, inst_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        &wifi_event_handler, NULL, &inst_any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                        &wifi_event_handler, NULL, &inst_got_ip);

    // Configure Wi-Fi
    wifi_config_t wifi_cfg = {};
    strncpy((char *)wifi_cfg.sta.ssid, WIFI_SSID, sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *)wifi_cfg.sta.password, WIFI_PASS, sizeof(wifi_cfg.sta.password) - 1);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    esp_wifi_start();

    ESP_LOGI("wifi", "Connecting to WiFi…");

    // Wait for connection
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                           CONNECTED_BIT,
                                           pdFALSE,
                                           pdTRUE,
                                           portMAX_DELAY);

    if (bits & CONNECTED_BIT) {
        ESP_LOGI("wifi", "Connected to Wi-Fi.");
    } else {
        ESP_LOGE("wifi", "Failed to connect to Wi-Fi.");
        return ESP_FAIL;
    }

    // Get and print RSSI
    wifi_ap_record_t info;
    if (esp_wifi_sta_get_ap_info(&info) == ESP_OK) {
        ESP_LOGI("wifi", "Connected (RSSI %d dBm)", info.rssi);
    } else {
        ESP_LOGW("wifi", "Failed to get AP info.");
    }

    return ESP_OK;
}
