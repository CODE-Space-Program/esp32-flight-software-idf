#include "esp_crt_bundle.h"
#include "ground_control.h"
#include <esp_log.h>
#include <esp_http_client.h>
#include <esp_tls.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <nvs_flash.h>

static const char *TAG = "GroundControl";

struct HttpResponse {
    std::string data;
};

static esp_err_t http_event_handler(esp_http_client_event_t *evt) {
    HttpResponse *resp = static_cast<HttpResponse*>(evt->user_data);
    switch (evt->event_id) {
        case HTTP_EVENT_ON_DATA:
            if (resp && evt->data_len > 0) {
                resp->data.append(reinterpret_cast<char*>(evt->data), evt->data_len);
            }
            break;
        default:
            break;
    }
    return ESP_OK;
}

GroundControl::GroundControl(const std::string &url)
  : baseUrl(url)
{
    requestTimer = xTimerCreate(
        "gc_timer",
        POLL_INTERVAL,
        pdTRUE,
        this,
        &GroundControl::timerCb
    );
}

GroundControl::~GroundControl() {
    if (requestTimer) {
        xTimerDelete(requestTimer, 0);
    }
    for (auto j : telemetryBuffer) {
        cJSON_Delete(j);
    }
}

void GroundControl::timerCb(TimerHandle_t t) {
    auto *self = static_cast<GroundControl*>(pvTimerGetTimerID(t));
    self->makeRequest();
}

void GroundControl::subscribe(Listener cb) {
    listeners.push_back(cb);
}

void GroundControl::connect() {
    if (isConnecting || isConnected) return;
    isConnecting = true;
    if (fetchFlightId() == 0) {
        xTimerStart(requestTimer, 0);
        isConnected = true;
    }
    isConnecting = false;
}

int GroundControl::fetchFlightId() {
    std::string url = baseUrl + "/api/flights";
    ESP_LOGI(TAG, "Fetching flight ID from %s", url.c_str());

    HttpResponse resp{};
    esp_http_client_config_t cfg = {};
    cfg.url = url.c_str();
    cfg.transport_type = HTTP_TRANSPORT_OVER_SSL;
    cfg.crt_bundle_attach = esp_crt_bundle_attach;
    cfg.event_handler = http_event_handler;
    cfg.user_data = &resp;

    auto *c = esp_http_client_init(&cfg);
    esp_http_client_set_method(c, HTTP_METHOD_POST);
    esp_http_client_set_header(c, "Content-Type", "application/json");
    esp_http_client_set_post_field(c, "{}", 2);

    ESP_LOGI(TAG, "Performing request...");
    esp_err_t err = esp_http_client_perform(c);
    int status = esp_http_client_get_status_code(c);
    esp_http_client_cleanup(c);

    if (err != ESP_OK || status != 200) {
        ESP_LOGE(TAG, "HTTP error: %s, code: %d", esp_err_to_name(err), status);
        return -1;
    }

    ESP_LOGI(TAG, "Response (%u bytes): %s", resp.data.size(), resp.data.c_str());

    cJSON *root = cJSON_Parse(resp.data.c_str());
    if (!root) {
        ESP_LOGE(TAG, "cJSON parse failed");
        return -1;
    }
    cJSON *d = cJSON_GetObjectItem(root, "data");
    flightId = cJSON_GetObjectItem(d, "flightId")->valuestring;
    token    = cJSON_GetObjectItem(d, "token")->valuestring;
    cJSON_Delete(root);

    ESP_LOGI(TAG, "Flight ID: %s", flightId.c_str());
    return 0;
}

void GroundControl::makeRequest() {
    if (flightId.empty()) return;

    std::string url = baseUrl + "/api/flights/" + flightId + "/events";
    ESP_LOGI(TAG, "Fetching events from %s", url.c_str());

    HttpResponse resp{};
    esp_http_client_config_t cfg = {};
    cfg.url = url.c_str();
    cfg.transport_type = HTTP_TRANSPORT_OVER_SSL;
    cfg.crt_bundle_attach = esp_crt_bundle_attach;
    cfg.event_handler = http_event_handler;
    cfg.user_data = &resp;

    auto *c = esp_http_client_init(&cfg);
    esp_http_client_set_header(c, "Authorization", ("Bearer " + token).c_str());

    esp_err_t err = esp_http_client_perform(c);
    int status = esp_http_client_get_status_code(c);
    esp_http_client_cleanup(c);

    if (err != ESP_OK || status != 200) {
        ESP_LOGE(TAG, "HTTP error fetching events: %s, code: %d", esp_err_to_name(err), status);
        return;
    }

    ESP_LOGI(TAG, "Response (%u bytes): %s", resp.data.size(), resp.data.c_str());
    processResponse(resp.data);
}

void GroundControl::processResponse(const std::string &p) {
    cJSON *root = cJSON_Parse(p.c_str());
    if (!root) {
        ESP_LOGE(TAG, "cJSON parse events failed");
        return;
    }
    cJSON *arr = cJSON_GetObjectItem(root, "data");
    cJSON *it;
    cJSON_ArrayForEach(it, arr) {
        auto *cmd = cJSON_GetObjectItem(it, "command");
        auto *args = cJSON_GetObjectItem(it, "args");
        emitEvent(cmd->valuestring, args);
    }
    cJSON_Delete(root);
}

void GroundControl::emitEvent(const std::string &cmd, cJSON *args) {
    for (auto &l : listeners) {
        l(cmd, args);
    }
}

void GroundControl::sendTelemetry(const TelemetryData &d) {
    cJSON *entry = cJSON_CreateObject();
    cJSON_AddNumberToObject(entry, "raw_altitude", d.raw_altitude);
    // … add other fields …
    cJSON_AddNumberToObject(entry, "sent", xTaskGetTickCount() * portTICK_PERIOD_MS);
    telemetryBuffer.push_back(entry);
    checkAndSendTelemetry();
}

void GroundControl::checkAndSendTelemetry() {
    if (telemetryBuffer.empty()) return;
    
    // send if oldest entry ≥1 s old
    // … build cJSON array & POST to /logs …
}

void GroundControl::debugNetworkConnection() {
    // mirror your Arduino logic using esp_netif APIs…
}
