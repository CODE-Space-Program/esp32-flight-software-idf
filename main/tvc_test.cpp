#include "tvc_test.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// drop-in replacement for millis() in Arduino
static inline uint32_t millis() {
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

bool   DO_CIRCLE    = true;
float  angle        = 0.0f;
float  centerPitch  = 90.0f;
float  centerYaw    = 90.0f;

TvcTest::TvcTest()
  : startedMs(0)
  , durationMs(0)
  , lastYaw(0)
  , lastPitch(0)
  , maxYaw(5)
  , minYaw(-5)
  , maxPitch(5)
  , minPitch(-5)
  , stepSizeYaw(0)
  , stepSizePitch(0)
  , yawDirection(true)
  , pitchDirection(true)
{}

bool TvcTest::isInProgress() {
    return (millis() - startedMs) < (unsigned long)durationMs;
}

void TvcTest::start(double maxDegrees, float stepSize, int duration) {
    startedMs   = millis();
    durationMs  = duration;
    maxPitch    =  (float)maxDegrees;
    minPitch    = -(float)maxDegrees;
    maxYaw      =  (float)maxDegrees;
    minYaw      = -(float)maxDegrees;
    stepSizeYaw   = stepSize;
    stepSizePitch = stepSize;
}

float TvcTest::getNewPitch() {
    if (DO_CIRCLE) {
        float p = centerPitch + maxPitch * sin(angle);
        lastPitch = p;
        return p;
    }
    float p = lastPitch + (pitchDirection ? stepSizePitch : -stepSizePitch);
    if (p >= maxPitch) pitchDirection = false;
    if (p <= minPitch) pitchDirection = true;
    lastPitch = p;
    return p;
}

float TvcTest::getNewYaw() {
    if (DO_CIRCLE) {
        float y = centerYaw + maxPitch * cos(angle);
        lastYaw = y;
        angle  += stepSizeYaw;
        return y;
    }
    float y = lastYaw + (yawDirection ? stepSizeYaw : -stepSizeYaw);
    if (y >= maxYaw) yawDirection = false;
    if (y <= minYaw) yawDirection = true;
    lastYaw = y;
    return y;
}
