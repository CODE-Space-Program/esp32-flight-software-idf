idf_component_register(
    SRCS 
        "ground_control.cpp"
    INCLUDE_DIRS 
        "."
    REQUIRES
        json
        esp_http_client
        esp-tls         # TLS for HTTPS
        esp_event       # event library
        esp_netif       # networking
        nvs_flash       # non‑volatile storage (often needed for Wi‑Fi creds)
        esp_timer
)
