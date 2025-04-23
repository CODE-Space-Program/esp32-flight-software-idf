idf_component_register(
  SRCS
    "main.cpp"
    "wifi_helper.cpp"
    "tvc_test.cpp"
    "servos.cpp"
    "sensors.cpp"
    "pyro.h"
    "kalman.cpp"
  INCLUDE_DIRS
    "."
  REQUIRES
    esp_event
    esp_netif
    esp_wifi
    nvs_flash

    i2cdev
    pca9685
    esp_bmp390
    esp_mpu6050
    esp-dsp

    groundControl
)
