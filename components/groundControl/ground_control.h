#pragma once

#include <string>
#include <vector>
#include <functional>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <cJSON.h>

#include "esp_http_client.h"   // HTTP client API
#include "esp_tls.h"           // TLS support (for HTTPS)
#include "esp_log.h"           // logging
#include "esp_event.h"         // event loop (required by the client)
#include "esp_netif.h"         // network interface

struct TelemetryData
{
    std::string state = "Ready";

    long time; // time in ms

    float pressure;    // pressure in mbar
    float temperature; // Temperature in Celsius
    float raw_altitude;
    float estimated_altitude; // Filtered height
    float velocity;
    float estimated_pitch;
    float estimated_yaw;
    float estimated_roll;
    float apogee;


    char const null_terminator = 0; // Null terminator to avoid overflow

    float nominalYawServoDegrees = 0;
    float nominalPitchServoDegrees = 0;

    bool servosLocked = true;

};

class GroundControl {
public:
    using Listener = std::function<void(const std::string &command, cJSON *args)>;

    explicit GroundControl(const std::string &base_url);
    ~GroundControl();

    void subscribe(Listener listener);
    void connect();
    void sendTelemetry(const TelemetryData &data);

    bool isConnecting = false;
    bool isConnected = false;

private:
    std::string baseUrl, flightId, token;
    std::vector<Listener> listeners;
    std::vector<cJSON*> telemetryBuffer;

    int    fetchFlightId();
    void   makeRequest();
    void   processResponse(const std::string &payload);
    void   emitEvent(const std::string &cmd, cJSON *args);
    void   debugNetworkConnection();
    void   checkAndSendTelemetry();

    static constexpr TickType_t POLL_INTERVAL = pdMS_TO_TICKS(1000);

    TaskHandle_t requestTaskHandle = nullptr;
    TimerHandle_t requestTimer;
    SemaphoreHandle_t requestSemaphore  = nullptr;

    static void timerCb(TimerHandle_t t);
    static void requestTaskFn(void *arg);
};
