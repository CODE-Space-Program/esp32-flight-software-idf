#include "PID.h"
#include <algorithm>
#include <math.h>

void PIDController_Init(PIDController *pid) {
    // Clear all controller variables
    pid->integrator = 0.0f;
    pid->prevError = 0.0f;

    pid->differentiator = 0.0f;
    pid->prevMeasurement = 0.0f;

    pid->out = 0.0f;
}

float PIDController_Update(PIDController *pid, float setpoint, float measurement) {
    // Calculate error
    float error = setpoint - measurement;

    // Apply deadband
    if (fabs(error) < 0.5f) {
        error = 0.0f;
    }

    // Proportional term
    float proportional = pid->Kp * error;

    // Check if the output is saturated
    bool isSaturated = (pid->out >= pid->limMax && error > 0.0f) ||
                        (pid->out <= pid->limMin && error < 0.0f);

    // Integral term (only update if not saturated)
    if (!isSaturated) {
        if (fabs(error) > 0.5f) {
            pid->integrator += 0.5f * pid->Ki * pid->T * (error + pid->prevError);
        } else {
            pid->integrator *= 0.9f; // decay integrator for small errors
        }

        // Anti-windup: Clamp the integrator state
        if (pid->integrator > pid->limMaxInt) {
            pid->integrator = pid->limMaxInt;
        } else if (pid->integrator < pid->limMinInt) {
            pid->integrator = pid->limMinInt;
        }
    }   

    // Derivative term (band-limited differentiator)
    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement) +
                            (2.0f * pid->tau - pid->T) * pid->differentiator) /
                          (2.0f * pid->tau + pid->T);

    // Compute output and apply limits
    pid->out = proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->limMax) {
        pid->out = pid->limMax;
    } else if (pid->out < pid->limMin) {
        pid->out = pid->limMin;
    }

    // Save error and measurement for the next update
    pid->prevError = error;
    pid->prevMeasurement = measurement;

    return pid->out;
}
