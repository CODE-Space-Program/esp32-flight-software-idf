#include "unity.h"
#include "storage.h"
#include "nvs_flash.h"

TEST_CASE("Empty queue returns zero entries", "[telemetry]")
{
    nvs_flash_erase();
    TEST_ASSERT_EQUAL(ESP_OK, nvs_flash_init());

    TelemetryQueue queue("telemetry");
    std::vector<TelemetryEntry> out;

    esp_err_t count = queue.pop(10, out);
    TEST_ASSERT_EQUAL(0, count);
    TEST_ASSERT_TRUE(out.empty());
}

TEST_CASE("Push and pop a single entry", "[telemetry]")
{
    nvs_flash_erase();
    TEST_ASSERT_EQUAL(ESP_OK, nvs_flash_init());

    TelemetryQueue queue("telemetry");
    TelemetryEntry e1{.timestamp = 1, .value = 42.0f};
    queue.push(e1);

    std::vector<TelemetryEntry> out;
    esp_err_t count = queue.pop(1, out);
    TEST_ASSERT_EQUAL(0, count);
    TEST_ASSERT_EQUAL_UINT32(e1.timestamp, out[0].timestamp);
    TEST_ASSERT_EQUAL_FLOAT(e1.value, out[0].value);

    // subsequent read should be empty
    count = queue.pop(1, out);
    TEST_ASSERT_EQUAL(0, count);
}

TEST_CASE("Push multiple entries and retrieve in FIFO order", "[telemetry]")
{
    nvs_flash_erase();
    TEST_ASSERT_EQUAL(ESP_OK, nvs_flash_init());

    TelemetryQueue queue("telemetry");
    for (uint32_t i = 0; i < 5; ++i)
    {
        queue.push({.timestamp = i, .value = (float)i * 1.5f});
    }

    std::vector<TelemetryEntry> out;
    esp_err_t count = queue.pop(3, out);
    TEST_ASSERT_EQUAL(0, count);
    for (size_t i = 0; i < 3; ++i)
    {
        TEST_ASSERT_EQUAL_UINT32(i, out[i].timestamp);
    }

    // Next pop should return remaining 2
    out.clear();
    count = queue.pop(10, out);
    TEST_ASSERT_EQUAL(0, count);
    TEST_ASSERT_EQUAL_UINT32(3, out[0].timestamp);
    TEST_ASSERT_EQUAL_UINT32(4, out[1].timestamp);
}
