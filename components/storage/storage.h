#pragma once

#include <vector>
#include "nvs_flash.h"
#include "nvs.h"

// Define your telemetry entry structure here
struct TelemetryEntry {
    uint32_t timestamp;
    float    value;
    // add more fields as needed
};

class TelemetryQueue {
public:
    // namespaceName: NVS namespace to use (default "telemetry")
    explicit TelemetryQueue(const char* namespaceName = "telemetry");
    ~TelemetryQueue();

    // Push a new entry onto the queue
    esp_err_t push(const TelemetryEntry& entry);

    // Retrieve up to `n` entries; removed from storage
    esp_err_t pop(size_t n, std::vector<TelemetryEntry>& outEntries);

private:
    const char*    ns_;
    nvs_handle_t   handle_;

    // Key under which the queue blob is stored
    static constexpr const char* QUEUE_KEY = "queue_blob";

    // Open NVS handle (call before any operation)
    esp_err_t openHandle(uint32_t openMode);
    void      closeHandle();
};