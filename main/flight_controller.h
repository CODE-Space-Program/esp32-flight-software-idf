#pragma once

#include "tvc.h"
#include "sensors.h"
#include "esp_log.h"
#include "datapoint.h"

enum class FlightState {
    Boot,
    Ready,
    PreLaunch,
    Flight,
    PoweredLanding,
    Landed,
};

class FlightController {
public:
    FlightController(Tvc &tvc, SensorManager &sensors);

    void setState(FlightState newState);
    FlightState getState() const;
    void initialize();
    void update();

private:
    Tvc &tvc;
    SensorManager &sensors;

    void handleBoot();
    void handleReady();
    void handlePreLaunch();
    void handleFlight();
    void handlePoweredLanding();
    void handleLanded();
    FlightState state;
};