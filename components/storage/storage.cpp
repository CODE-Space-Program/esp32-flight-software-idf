#include "storage.h"

TelemetryQueue::TelemetryQueue(const char *namespaceName)
    : ns_(namespaceName), handle_(0)
{
    // Initialize NVS (should be done once in app)
    nvs_flash_init();
}

TelemetryQueue::~TelemetryQueue()
{
    // nothing to do
}

esp_err_t TelemetryQueue::openHandle(uint32_t openMode)
{
    return nvs_open(ns_, (nvs_open_mode_t)openMode, &handle_);
}

void TelemetryQueue::closeHandle()
{
    if (handle_)
    {
        nvs_close(handle_);
        handle_ = 0;
    }
}

esp_err_t TelemetryQueue::push(const TelemetryEntry &entry)
{
    esp_err_t err = openHandle(NVS_READWRITE);
    if (err != ESP_OK)
        return err;

    // read existing blob
    size_t blob_len = 0;
    err = nvs_get_blob(handle_, QUEUE_KEY, nullptr, &blob_len);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
    {
        closeHandle();
        return err;
    }

    std::vector<TelemetryEntry> queue;
    if (blob_len > 0)
    {
        queue.resize(blob_len / sizeof(TelemetryEntry));
        err = nvs_get_blob(handle_, QUEUE_KEY, queue.data(), &blob_len);
        if (err != ESP_OK)
        {
            closeHandle();
            return err;
        }
    }

    // append new entry
    queue.push_back(entry);

    // write back
    err = nvs_set_blob(handle_, QUEUE_KEY, queue.data(), queue.size() * sizeof(TelemetryEntry));
    if (err == ESP_OK)
    {
        err = nvs_commit(handle_);
    }
    closeHandle();
    return err;
}

esp_err_t TelemetryQueue::pop(size_t n, std::vector<TelemetryEntry> &outEntries)
{
    esp_err_t err = openHandle(NVS_READWRITE);
    if (err != ESP_OK)
        return err;

    size_t blob_len = 0;
    err = nvs_get_blob(handle_, QUEUE_KEY, nullptr, &blob_len);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        closeHandle();
        return ESP_OK; // empty queue
    }
    if (err != ESP_OK)
    {
        closeHandle();
        return err;
    }

    size_t count = blob_len / sizeof(TelemetryEntry);
    std::vector<TelemetryEntry> queue(count);
    err = nvs_get_blob(handle_, QUEUE_KEY, queue.data(), &blob_len);
    if (err != ESP_OK)
    {
        closeHandle();
        return err;
    }

    // determine how many to pop
    size_t toPop = std::min(n, queue.size());
    outEntries.assign(queue.begin(), queue.begin() + toPop);

    // remove popped entries and write remaining back
    std::vector<TelemetryEntry> remaining;
    if (queue.size() > toPop)
    {
        remaining.assign(queue.begin() + toPop, queue.end());
        err = nvs_set_blob(handle_, QUEUE_KEY,
                           remaining.data(), remaining.size() * sizeof(TelemetryEntry));
    }
    else
    {
        // clear entire key
        err = nvs_erase_key(handle_, QUEUE_KEY);
    }
    if (err == ESP_OK)
    {
        err = nvs_commit(handle_);
    }

    closeHandle();
    return err;
}
