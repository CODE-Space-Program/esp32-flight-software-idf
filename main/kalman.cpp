#include "kalman.h"
#include "esp_log.h"
#include "ground_control.h"

EKFManager::EKFManager() : ekf13_(nullptr) {}

EKFManager::~EKFManager() {
    if (ekf13_) {
        delete ekf13_;
    }
}

void EKFManager::init() {
    ekf13_ = new ekf_imu13states();
    ekf13_->Init();
    
    /*// Initialize the state vector (X)
    ekf13_->X(0, 0) = 1.0f;  // Quaternion w
    ekf13_->X(1, 0) = 0.0f;  // Quaternion x
    ekf13_->X(2, 0) = 0.0f;  // Quaternion y
    ekf13_->X(3, 0) = 0.0f;  // Quaternion z
    ekf13_->X(4, 0) = 0.0f;  // Gyro bias x
    ekf13_->X(5, 0) = 0.0f;  // Gyro bias y
    ekf13_->X(6, 0) = 0.0f;  // Gyro bias z*/

    // Initialize the covariance matrix (P) as an identity matrix
    /*const int MATRIX_SIZE = 13;  // Assuming P is a 13x13 matrix
    for (int i = 0; i < MATRIX_SIZE; ++i) {
        for (int j = 0; j < MATRIX_SIZE; ++j) {
            ekf13_->P(i, j) = (i == j) ? 1.0f : 0.0f;  // Diagonal to 1, others to 0
        }
    }*/
    ESP_LOGI("ekf", "EKF Initialized");
}

void EKFManager::processSensorData(float accel[3], float gyro[3], float dt) {

    if (!accel || !gyro) {
        ESP_LOGE("EKFManager", "Null pointer passed to processSensorData: accel=%p, gyro=%p", accel, gyro);
        return;
    }

    if (!ekf13_) {
        ESP_LOGE("EKFManager", "EKF object is not initialized");
        return;
    }
    /*ESP_LOGI("EKFManager", "Processing data: accel=[%f, %f, %f], gyro=[%f, %f, %f], dt=%f",
        accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], dt);*/

    
    ekf13_->Process(gyro, dt);

    // Normalize quaternion after prediction
    dspm::Mat q_norm(ekf13_->X.data, 4, 1);
    float norm = q_norm.norm();
    if (norm > 1e-6) {
        q_norm /= norm;
    } else {
        ESP_LOGE("ekf", "Quaternion norm too small");
    }

    // Measurement noise covariance matrix
    float R_accel[3] = {0.1f, 0.1f, 0.1f};
    
    // Update EKF state with accelerometer data
    ekf13_->UpdateRefMeasurement(accel, nullptr, R_accel);

    ESP_LOGI("EKFManager", "Quaternion: [%f, %f, %f, %f]",
        ekf13_->X(0, 0), ekf13_->X(1, 0), ekf13_->X(2, 0), ekf13_->X(3, 0));
    ESP_LOGI("EKFManager", "Gyro Bias: [%f, %f, %f]",
        ekf13_->X(4, 0), ekf13_->X(5, 0), ekf13_->X(6, 0));


    ESP_LOGI("ekf", "EKF state updated");
}

float EKFManager::getAltitude() const {
    return ekf13_->X(7, 0);
}

void EKFManager::getState(float gyro_bias[3]) {
    dspm::Mat state = ekf13_->X;

    // Extract quaternion (attitude)
    //quaternion[0] = state(0, 0);
    //quaternion[1] = state(1, 0);
    //quaternion[2] = state(2, 0);
    //quaternion[3] = state(3, 0);

    // Extract gyroscope bias
    gyro_bias[0] = state(4, 0);
    gyro_bias[1] = state(5, 0);
    gyro_bias[2] = state(6, 0);

    //ESP_LOGI("ekf", "Quaternion: [%.2f, %.2f, %.2f, %.2f]", quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
    ESP_LOGI("ekf", "Gyro Bias: [%.2f, %.2f, %.2f]", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
}

dspm::Mat EKFManager::getEularAngles() const {
    return ekf::quat2eul(ekf13_->X.data);
}

float EKFManager::getPitch() const {
    dspm::Mat eular_angles = getEularAngles();
    return eular_angles(1, 0) * (180.0f / M_PI); // pitch is the second angle
}

float EKFManager::getYaw() const {
    dspm::Mat eular_angles = getEularAngles();
    return eular_angles(2, 0) * (180.0f / M_PI); // Yaw is the third angle
}