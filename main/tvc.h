#pragma once

#include "servos.h"
#include "esp_log.h"
#include "PID.h"

/**
 * @brief Thrust Vector Control (TVC) class.
 */
class Tvc {
public:
    int pitchServoChannel;
    int yawServoChannel;
    float pitch;
    float yaw;
    float servoReduction;

    Tvc(Servos &servos, int pitchServoChannel, int yawServoChannel, float servoReduction);

    void initialize();
    void uninitialize();

    /**
     * @brief Move the TVC to the specified angles.
     * @param pitch Angle in degrees [-90, 90].
     * @param yaw Angle in degrees [-90, 90].
     */
    void moveRaw(float pitch, float yaw);

    /**
     * @brief Move the TVC using PID control to stabilize around 0°.
     * @param pitch Current pitch angle.
     * @param yaw Current yaw angle.
     */
    void move(float pitch, float yaw);

private:
    Servos &servos;

    PIDController pitchPID;
    PIDController yawPID;

    void initializePID(PIDController &pid, float Kp, float Ki, float Kd, float limMin, float limMax, float limMinInt, float limMaxInt, float tau, float T);
};
