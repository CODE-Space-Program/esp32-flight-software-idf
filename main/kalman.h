#pragma once

#include "ekf_imu13states.h"

class EKFManager {
public:
    EKFManager();
    ~EKFManager();

    void init();
    void processSensorData(float accel[3], float gyro[3], float dt);
    float getAltitude() const;
    void getState(float gyro_data[3]);
    float getPitch() const;
    float getYaw() const;
    dspm::Mat getEularAngles() const;
    

private:
    float state[4];
    ekf_imu13states *ekf13_;
};