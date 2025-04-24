idf_component_register(
    SRCS 
        "storage.cpp"
    INCLUDE_DIRS 
        "."
    REQUIRES
        nvs_flash       # non‑volatile storage (often needed for Wi‑Fi creds)
)
