#include "flight_controller.h"
#include "pyro.h"
#include "datapoint.h"

static bool istrue = false;

FlightController::FlightController(Tvc &tvc, SensorManager &sensors)
    : tvc(tvc), sensors(sensors), state(FlightState::Boot){}

void FlightController::initialize() {
    ESP_LOGI("flight controller", "Initializing flight controller...");
    tvc.initialize();
    //tvc.moveRaw(90, 90);
    state = FlightState::Boot;
}

void FlightController::setState(FlightState newState) {
    state = newState;
}

FlightState FlightController::getState() const {
    return state;
}

void FlightController::update() {
    switch (state)
    {
    case FlightState::Boot:
        handleBoot();
        break;
    case FlightState::Ready:
        handleReady();
        break;
    case FlightState::PreLaunch:
        handlePreLaunch();
        break;
    case FlightState::Flight:
        handleFlight();
        break;
    case FlightState::PoweredLanding:
        handlePoweredLanding();
        break;
    case FlightState::Landed:
        handleLanded();
        break;
    }
}

void FlightController::handleBoot() {
    ESP_LOGI("flight controller", "Booting...");
    state = FlightState::Ready;
}

void FlightController::handleReady() {
    ESP_LOGI("flight controller", "Ready state: Checking Sensors...");
    if (sensors.allSystemsCheck()) {
        state = FlightState::PreLaunch;
        ESP_LOGI("flight controller", "All systems are go. Transitioning to PreLaunch.");
    }
}

void FlightController::handlePreLaunch() {
    ESP_LOGI("flight controller", "PreLaunch state: Waiting for start command...");
    /*bool detectLaunchCommand = true;
    if (detectLaunchCommand) {
        state = FlightState::Flight;
        ESP_LOGI("flight controller", "Launch command detected, Transitioning to flight.");
    }*/
}

void FlightController::handleFlight() {
    ESP_LOGI("flight controller", "Flight state: Controlling TVC...");
    sensors.readMpu();
    tvc.move(datapoint.estimated_pitch, datapoint.estimated_yaw);
    sensors.readBmp();
    if (sensors.detectApogee(datapoint.estimated_altitude)) {
        state = FlightState::PoweredLanding;
        ESP_LOGI("flight controller", "Apogee detected. Transitioning to PoweredLanding.");
    }
}

void FlightController::handlePoweredLanding() {
    ESP_LOGI("flight controller", "PoweredLanding state: Stabilizing for landing...");
    sensors.readMpu();
    tvc.move(datapoint.estimated_pitch, datapoint.estimated_yaw);
    sensors.readBmp();
    fireLandingBurn();
    if (datapoint.estimated_altitude < 2.0f) {
        state = FlightState::Landed;
        ESP_LOGI("flight controller", "Landing completed. Transitioning to Landed");
    }
}

void FlightController::handleLanded() {
    ESP_LOGI("flight controller", "Landed state: Flight completed.");
    tvc.uninitialize();
}