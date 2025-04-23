#include "esp_crt_bundle.h"
#include "ground_control.h"
#include <esp_log.h>
#include <esp_http_client.h>
#include <esp_tls.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <nvs_flash.h>
#include <freertos/semphr.h>

static const char *TAG = "GroundControl";

struct HttpResponse
{
    std::string data;
};

// HTTP event handler: collect body chunks
extern "C" esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    HttpResponse *resp = static_cast<HttpResponse *>(evt->user_data);
    if (evt->event_id == HTTP_EVENT_ON_DATA && evt->data_len > 0)
    {
        resp->data.append(reinterpret_cast<char *>(evt->data), evt->data_len);
    }
    return ESP_OK;
}

GroundControl::GroundControl(const std::string &base_url)
    : baseUrl(base_url)
{
    // create auto‑reloading timer (runs in timer service task)
    requestTimer = xTimerCreate(
        "gc_timer",
        POLL_INTERVAL,
        pdTRUE,
        this,
        &GroundControl::timerCb);

    requestSemaphore = xSemaphoreCreateBinary();
    // spawn a request task with a fat 8 KB stack
    xTaskCreate(
        requestTaskFn,
        "GC_req",
        8192, // words ≈ 32 KB
        this,
        tskIDLE_PRIORITY + 1,
        &requestTaskHandle);
}

GroundControl::~GroundControl()
{
    if (requestTimer)
    {
        xTimerDelete(requestTimer, 0);
    }
    for (auto entry : telemetryBuffer)
    {
        cJSON_Delete(entry);
    }
}

void GroundControl::timerCb(TimerHandle_t t)
{
    auto *self = static_cast<GroundControl *>(pvTimerGetTimerID(t));
    BaseType_t hpw = pdFALSE;
    xSemaphoreGiveFromISR(self->requestSemaphore, &hpw);
    portYIELD_FROM_ISR(hpw);
}

void GroundControl::subscribe(Listener listener)
{
    listeners.push_back(listener);
}

void GroundControl::connect()
{
    if (isConnecting || isConnected)
        return;
    isConnecting = true;

    if (fetchFlightId() == 0)
    {
        xTimerStart(requestTimer, 0);
        isConnected = true;
    }

    isConnecting = false;
}

int GroundControl::fetchFlightId()
{
    std::string url = baseUrl + "/api/flights";
    ESP_LOGI(TAG, "Fetching flight ID from %s", url.c_str());

    HttpResponse resp{};
    esp_http_client_config_t cfg = {};
    cfg.url = url.c_str();
    cfg.transport_type = HTTP_TRANSPORT_OVER_SSL;
    cfg.crt_bundle_attach = esp_crt_bundle_attach;
    cfg.event_handler = http_event_handler;
    cfg.user_data = &resp;
    cfg.timeout_ms = 15000;

    auto *client = esp_http_client_init(&cfg);
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, "{}", 2);

    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (err != ESP_OK || status != 200)
    {
        ESP_LOGE(TAG, "HTTP error: %s (code %d)", esp_err_to_name(err), status);
        return -1;
    }

    ESP_LOGI(TAG, "Response (%u bytes): %s", resp.data.size(), resp.data.c_str());
    cJSON *root = cJSON_Parse(resp.data.c_str());
    if (!root)
    {
        ESP_LOGE(TAG, "cJSON parse failed");
        return -1;
    }
    cJSON *d = cJSON_GetObjectItem(root, "data");
    flightId = cJSON_GetObjectItem(d, "flightId")->valuestring;
    token = cJSON_GetObjectItem(d, "token")->valuestring;
    cJSON_Delete(root);

    ESP_LOGI(TAG, "Flight ID: %s", flightId.c_str());
    return 0;
}

void GroundControl::makeRequest()
{
    if (flightId.empty())
        return;

    std::string url = baseUrl + "/api/flights/" + flightId + "/events";
    ESP_LOGI(TAG, "Fetching events from %s", url.c_str());

    HttpResponse resp{};
    esp_http_client_config_t cfg = {};
    cfg.url = url.c_str();
    cfg.transport_type = HTTP_TRANSPORT_OVER_SSL;
    cfg.crt_bundle_attach = esp_crt_bundle_attach;
    cfg.event_handler = http_event_handler;
    cfg.user_data = &resp;
    cfg.timeout_ms = 15000;

    auto *client = esp_http_client_init(&cfg);
    esp_http_client_set_header(client, "Authorization", ("Bearer " + token).c_str());

    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (err != ESP_OK || status != 200)
    {
        ESP_LOGE(TAG, "HTTP error fetching events: %s (code %d)", esp_err_to_name(err), status);
        return;
    }

    ESP_LOGI(TAG, "Events response (%u bytes): %s", resp.data.size(), resp.data.c_str());
    processResponse(resp.data);
}

void GroundControl::processResponse(const std::string &payload)
{
    cJSON *root = cJSON_Parse(payload.c_str());
    if (!root)
    {
        ESP_LOGE(TAG, "cJSON parse events failed");
        return;
    }
    cJSON *arr = cJSON_GetObjectItem(root, "data");
    cJSON *it;
    cJSON_ArrayForEach(it, arr)
    {
        auto *cmd = cJSON_GetObjectItem(it, "command");
        auto *args = cJSON_GetObjectItem(it, "args");
        emitEvent(cmd->valuestring, args);
    }
    cJSON_Delete(root);
}

void GroundControl::emitEvent(const std::string &cmd, cJSON *args)
{
    for (auto &l : listeners)
    {
        l(cmd, args);
    }
}

void GroundControl::sendTelemetry(const TelemetryData &d)
{
    long now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    cJSON *entry = cJSON_CreateObject();
    cJSON_AddNumberToObject(entry, "raw_altitude", d.raw_altitude);
    cJSON_AddNumberToObject(entry, "altitude", d.estimated_altitude);
    cJSON_AddNumberToObject(entry, "velocity", d.velocity);
    cJSON_AddNumberToObject(entry, "pitch", d.estimated_pitch);
    cJSON_AddNumberToObject(entry, "yaw", d.estimated_yaw);
    cJSON_AddNumberToObject(entry, "roll", d.estimated_roll);
    cJSON_AddNumberToObject(entry, "temperature", d.temperature);
    cJSON_AddNumberToObject(entry, "pressure", d.pressure);
    cJSON_AddNumberToObject(entry, "time", d.time);
    cJSON_AddNumberToObject(entry, "apogee", d.apogee);
    cJSON_AddStringToObject(entry, "state", d.state.c_str());
    cJSON_AddNumberToObject(entry, "nominalPitchServoDegrees", d.nominalPitchServoDegrees);
    cJSON_AddNumberToObject(entry, "nominalYawServoDegrees", d.nominalYawServoDegrees);
    cJSON_AddBoolToObject(entry, "servosLocked", d.servosLocked);
    cJSON_AddNumberToObject(entry, "sent", now_ms);
    telemetryBuffer.push_back(entry);
    checkAndSendTelemetry();
}

void GroundControl::checkAndSendTelemetry()
{
    if (telemetryBuffer.empty())
        return;

    long oldest = static_cast<long>(cJSON_GetObjectItem(telemetryBuffer.front(), "sent")->valuedouble);
    long now = static_cast<long>(xTaskGetTickCount() * portTICK_PERIOD_MS);

    if (now - oldest < 1000)
        return;

    // Build cJSON array
    cJSON *arr = cJSON_CreateArray();
    for (auto entry : telemetryBuffer)
    {
        cJSON_AddItemToArray(arr, entry);
    }
    telemetryBuffer.clear();

    char *body = cJSON_PrintUnformatted(arr);
    cJSON_Delete(arr);

    std::string url = baseUrl + "/api/flights/" + flightId + "/logs";
    esp_http_client_config_t cfg = {};
    cfg.url = url.c_str();
    cfg.transport_type = HTTP_TRANSPORT_OVER_SSL;
    cfg.crt_bundle_attach = esp_crt_bundle_attach;
    auto *client = esp_http_client_init(&cfg);
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "Authorization", ("Bearer " + token).c_str());
    esp_http_client_set_post_field(client, body, strlen(body));

    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    if (err != ESP_OK || status != 200)
    {
        ESP_LOGE(TAG, "Telemetry POST failed: code %d", status);
    }
    else
    {
        ESP_LOGI(TAG, "Telemetry sent successfully");
        // Optionally parse response for data.ok
    }
    esp_http_client_cleanup(client);
    free(body);
}

void GroundControl::debugNetworkConnection()
{
    // placeholder: replicate Arduino network checks with esp_netif APIs
}

void GroundControl::requestTaskFn(void *arg)
{
    auto *self = static_cast<GroundControl *>(arg);
    for (;;)
    {
        if (xSemaphoreTake(self->requestSemaphore, portMAX_DELAY) == pdTRUE)
        {
            self->makeRequest();
        }
    }
}
