#include "tvc.h"
#include <algorithm>
#include "esp_timer.h"
#include "esp_log.h"


Tvc::Tvc(Servos &servos, int pitchServoChannel, int yawServoChannel, float servoReduction)
    : pitchServoChannel(pitchServoChannel), yawServoChannel(yawServoChannel), servoReduction(servoReduction), servos(servos) {}

void Tvc::initialize() {
        servos.initialize();

        initializePID(pitchPID, 1.0f, 0.9f, 0.5f, -25.0f, 25.0f, -10.0f, 10.0f, 0.03f, 0.01f);
        initializePID(yawPID, 1.5f, 0.3f, 0.5f, -25.0f, 25.0f, -10.0f, 10.0f, 0.03f, 0.01f);


        ESP_LOGI("TVC", "TVC initialized");
}

void Tvc::uninitialize() {
        servos.deinitialize();

        ESP_LOGI("TVC", "TVC uninitialized");
}

void Tvc::moveRaw(float pitch, float yaw) {
    // Store the current pitch and yaw
    this->pitch = pitch;
    this->yaw = yaw;

    ESP_LOGI("TVC", "pitch and yaw in moveRaw: pitch: %.2f, yaw: %.2f", pitch, yaw);

    // Move servos
    servos.move(pitchServoChannel, pitch);
    servos.move(yawServoChannel, yaw);

}

void Tvc::move(float pitch, float yaw) {
    static int64_t lastTime = esp_timer_get_time(); // Initialize lastTime with the current time in microseconds
    int64_t currentTime = esp_timer_get_time();
    float dt = (currentTime - lastTime) / 1000000.0f; // Convert microseconds to seconds
    lastTime = currentTime;

    pitchPID.T = dt;
    yawPID.T = dt;

    // Scale control signals
    float controlSignalYaw = PIDController_Update(&yawPID, 0.0f, yaw);
    float controlSignalPitch = PIDController_Update(&pitchPID, 0.0f, pitch);

    ESP_LOGI("TVC", "TVC pitch and yaw in move before servoReduction *: pitch: %.2f, yaw: %.2f", controlSignalPitch, controlSignalYaw);

    // Apply servo reduction
    controlSignalPitch *= servoReduction;
    controlSignalYaw *= servoReduction;

    // Clamp to servo range (e.g., 0° to 180°)
    controlSignalPitch = std::clamp(controlSignalPitch, -90.0f, 90.0f);
    controlSignalYaw = std::clamp(controlSignalYaw, -90.0f, 90.0f);


    ESP_LOGI("TVC", "TVC pitch and yaw in move: pitch: %.2f, yaw: %.2f", controlSignalPitch, controlSignalYaw);

    // Move servos
    moveRaw(controlSignalPitch, controlSignalYaw);
}

void Tvc::initializePID(PIDController &pid, float Kp, float Ki, float Kd, float limMin, float limMax, float limMinInt, float limMaxInt, float tau, float T) {
    pid.Kp = Kp;
    pid.Ki = Ki;
    pid.Kd = Kd;
    pid.limMin = limMin;
    pid.limMax = limMax;
    pid.limMinInt = limMinInt;
    pid.limMaxInt = limMaxInt;
    pid.tau = tau;
    pid.T = T;

    PIDController_Init(&pid);
}
