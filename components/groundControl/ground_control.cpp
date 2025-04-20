#include "ground_control.h"
#include <esp_log.h>
#include <esp_http_client.h>
#include <esp_tls.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <nvs_flash.h>

static const char *TAG = "GroundControl";

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
    fetchFlightId();
    xTimerStart(requestTimer, 0);
    isConnecting = false;
    isConnected  = true;
}

int GroundControl::fetchFlightId() {
    esp_http_client_config_t cfg = {
        .url = (baseUrl + "/api/flights").c_str()
    };
    auto *c = esp_http_client_init(&cfg);
    esp_http_client_set_method(c, HTTP_METHOD_POST);
    esp_http_client_set_header(c, "Content-Type", "application/json");
    esp_http_client_set_post_field(c, "{}", 2);

    esp_err_t err = esp_http_client_perform(c);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "POST /flights failed: %s", esp_err_to_name(err));
        esp_http_client_cleanup(c);
        debugNetworkConnection();
        return -1;
    }

    int code = esp_http_client_get_status_code(c);
    int len  = esp_http_client_get_content_length(c);
    std::string buf(len, '\0');
    esp_http_client_read_response(c, &buf[0], len);
    esp_http_client_cleanup(c);

    if (code != 200) {
        ESP_LOGE(TAG, "HTTP %d fetching flight ID", code);
        return -1;
    }

    cJSON *root = cJSON_Parse(buf.c_str());
    if (!root) {
        ESP_LOGE(TAG, "cJSON parse failed");
        return -1;
    }
    cJSON *d = cJSON_GetObjectItem(root, "data");
    flightId = cJSON_GetObjectItem(d, "flightId")->valuestring;
    token    = cJSON_GetObjectItem(d, "token")->valuestring;
    ESP_LOGI(TAG, "Flight ID: %s", flightId.c_str());
    cJSON_Delete(root);
    return 0;
}

void GroundControl::makeRequest() {
    if (flightId.empty()) return;

    esp_http_client_config_t cfg = {
        .url = (baseUrl + "/api/flights/" + flightId + "/events").c_str()
    };
    auto *c = esp_http_client_init(&cfg);
    esp_http_client_set_header(c, "Authorization", ("Bearer " + token).c_str());

    if (esp_http_client_perform(c) != ESP_OK) {
        ESP_LOGE(TAG, "GET /events failed");
        esp_http_client_cleanup(c);
        return;
    }

    int len = esp_http_client_get_content_length(c);
    std::string buf(len, '\0');
    esp_http_client_read_response(c, &buf[0], len);
    esp_http_client_cleanup(c);

    processResponse(buf);
}

void GroundControl::processResponse(const std::string &p) {
    cJSON *root = cJSON_Parse(p.c_str());
    if (!root) {
        ESP_LOGE(TAG, "cJSON parse events");
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

void GroundControl::sendTelemetry(const Data &d) {
    cJSON *entry = cJSON_CreateObject();
    cJSON_AddNumberToObject(entry, "raw_altitude", d.raw_altitude);
    // … add other fields …

    cJSON_AddNumberToObject(entry, "sent", xTaskGetTickCount() * portTICK_PERIOD_MS);
    telemetryBuffer.push_back(entry);
    checkAndSendTelemetry();
}

void GroundControl::checkAndSendTelemetry() {
    if (telemetryBuffer.empty()) return;

    // send if oldest entry ≥1 s old
    // … build cJSON array & POST to /logs …
}

void GroundControl::debugNetworkConnection() {
    // mirror your Arduino logic using esp_netif APIs…
}
