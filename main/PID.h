#pragma once

/**
 * @brief Structure to hold PID controller parameters and state.
 */
typedef struct {
    float Kp;          // Proportional gain
    float Ki;          // Integral gain
    float Kd;          // Derivative gain

    float tau;         // Derivative low-pass filter time constant
    float T;           // Sample time (in seconds)

    float limMin;      // Minimum output limit
    float limMax;      // Maximum output limit

    float limMinInt;   // Minimum integrator state
    float limMaxInt;   // Maximum integrator state

    // Controller state
    float integrator;      // Integrator state
    float prevError;       // Previous error
    float differentiator;  // Differentiator state
    float prevMeasurement; // Previous measurement

    // Output
    float out;             // Controller output
} PIDController;

/**
 * @brief Initialize the PID controller.
 * @param pid Pointer to the PIDController structure.
 */
void PIDController_Init(PIDController *pid);

/**
 * @brief Update the PID controller with a new setpoint and measurement.
 * @param pid Pointer to the PIDController structure.
 * @param setpoint Desired setpoint.
 * @param measurement Current measurement.
 * @return Controller output.
 */
float PIDController_Update(PIDController *pid, float setpoint, float measurement);
