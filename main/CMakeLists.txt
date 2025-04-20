idf_component_register(
  SRCS
    "esp32-flight-software-v2.c"
    "main.cpp"
    "wifi_helper.cpp"
  INCLUDE_DIRS
    "."
  REQUIRES
    groundControl
    
    esp_event
    esp_netif
    esp_wifi
    nvs_flash
)
