#pragma once

#include <cmath>

extern bool   DO_CIRCLE;
extern float  angle;
extern float  centerPitch;
extern float  centerYaw;

class TvcTest {
public:
    TvcTest();

    // returns true while test is running
    bool isInProgress();

    // start a new test: maxDegrees (±), stepSize per update, total duration ms
    void start(double maxDegrees, float stepSize, int duration = 5000);

    // compute next set‑point
    float getNewPitch();
    float getNewYaw();

private:
    unsigned long startedMs;
    int           durationMs;

    float lastYaw, lastPitch;
    float maxYaw, minYaw;
    float maxPitch, minPitch;
    float stepSizeYaw, stepSizePitch;
    bool  yawDirection, pitchDirection;
};
