#include "kalman.h"

KalmanFilter::KalmanFilter(float init_height, float init_velocity, float dt)
    : ekf(0, 0), dt_(dt)
{
    // Initial state
    x_[0] = init_height;
    x_[1] = init_velocity;

    // State transition matrix A
    A[0][0] = 1;
    A[0][1] = dt_;
    A[1][0] = 0;
    A[1][1] = 1;
    // Control matrix B
    B[0] = 0.5f * dt_ * dt_;
    B[1] = dt_;
    // Measurement matrix H
    H[0][0] = 1;
    H[0][1] = 0;
    // Process noise Q
    Q[0][0] = 0.001f;
    Q[0][1] = 0;
    Q[1][0] = 0;
    Q[1][1] = 0.0005f;
    // Measurement noise R
    R[0][0] = 0.02f;
    // Initial covariance P
    P[0][0] = 1;
    P[0][1] = 0;
    P[1][0] = 0;
    P[1][1] = 1;
}

void KalmanFilter::predict(float acc)
{
    // Predict state
    float x_pred[2];
    x_pred[0] = A[0][0] * x_[0] + A[0][1] * x_[1] + B[0] * acc;
    x_pred[1] = A[1][0] * x_[0] + A[1][1] * x_[1] + B[1] * acc;

    // Predict covariance
    float P_pred[2][2];
    P_pred[0][0] = A[0][0] * P[0][0] + A[0][1] * P[1][0];
    P_pred[0][1] = A[0][0] * P[0][1] + A[0][1] * P[1][1];
    P_pred[1][0] = A[1][0] * P[0][0] + A[1][1] * P[1][0];
    P_pred[1][1] = A[1][0] * P[0][1] + A[1][1] * P[1][1];

    // Add process noise
    P_pred[0][0] += Q[0][0];
    P_pred[1][1] += Q[1][1];

    // Commit
    x_[0] = x_pred[0];
    x_[1] = x_pred[1];
    P[0][0] = P_pred[0][0];
    P[0][1] = P_pred[0][1];
    P[1][0] = P_pred[1][0];
    P[1][1] = P_pred[1][1];
}

void KalmanFilter::update(float height_measured)
{
    // Innovation
    float y = height_measured - (H[0][0] * x_[0] + H[0][1] * x_[1]);
    // Innovation covariance
    float S = H[0][0] * P[0][0] + H[0][1] * P[1][0] + R[0][0];
    // Kalman gain
    float K[2];
    K[0] = (P[0][0] * H[0][0] + P[0][1] * H[0][1]) / S;
    K[1] = (P[1][0] * H[0][0] + P[1][1] * H[0][1]) / S;

    // Update estimate
    x_[0] += K[0] * y;
    x_[1] += K[1] * y;

    // Update covariance
    P[0][0] -= K[0] * H[0][0] * P[0][0];
    P[0][1] -= K[0] * H[0][1] * P[0][1];
    P[1][0] -= K[1] * H[0][0] * P[1][0];
    P[1][1] -= K[1] * H[0][1] * P[1][1];
}

float KalmanFilter::get_estimated_height() const
{
    return x_[0];
}

float KalmanFilter::get_estimated_velocity() const
{
    return x_[1];
}
