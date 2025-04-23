#pragma once

#include "ekf.h"
#include <cmath>

/**
 * 1D Kalman filter for height and velocity.
 * Subclass of the abstract ekf base.
 */
class KalmanFilter : public ekf {
public:
    /**
     * @param init_height  Initial height estimate
     * @param init_velocity Initial velocity estimate
     * @param dt           Time step (s)
     */
    KalmanFilter(float init_height, float init_velocity, float dt);
    ~KalmanFilter() override = default;

    // Pure-virtual overrides (not used in this simple linear KF)
    void Init() override {};
    void LinearizeFG(dspm::Mat &x, float *u) override {};

    /** Predict state forward by one step using control (acceleration). */
    void predict(float acc);
    /** Correct state using a height measurement. */
    void update(float height_measured);

    float get_estimated_height() const;
    float get_estimated_velocity() const;

private:
    float dt_;
    // Matrices and vectors
    float A[2][2];  // state transition
    float B[2];     // control-input
    float H[1][2];  // measurement
    float Q[2][2];  // process noise
    float R[1][1];  // measurement noise
    float P[2][2];  // estimate covariance
    float x_[2];    // state vector [height, velocity]
};