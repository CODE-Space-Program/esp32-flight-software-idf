#pragma once
#include <string>
#include <cstdint>

struct DataPoint {
    // General telemetry data
    //std::string state;          // Current system state (e.g., "Ready")
    //uint32_t time;              // Timestamp in milliseconds
    float pressure;             // Atmospheric pressure (mbar)
    float temperature;          // Temperature (°C)
    float raw_altitude;         // Raw altitude from BMP390 (meters)
    float estimated_altitude;   // Kalman-filtered altitude (meters)
    float velocity;             // Velocity (m/s)
    float estimated_pitch;      // Estimated pitch angle (degrees)
    float estimated_yaw;        // Estimated yaw angle (degrees)
    //float estimated_roll;       // Estimated roll angle (degrees)
    float apogee;               // Maximum altitude reached (meters)

    // Sensor-specific data
    //float accel_data[3];        // Accelerometer data (m/s²) [X, Y, Z]
    //float gyro_data[3];         // Gyroscope data (rad/s) [X, Y, Z]

    // Additional control data
    //float nominalYawServoDegrees;  // Servo yaw position (degrees)
    //float nominalPitchServoDegrees; // Servo pitch position (degrees)
    //bool servosLocked;             // Servo lock status
};

extern DataPoint datapoint;
