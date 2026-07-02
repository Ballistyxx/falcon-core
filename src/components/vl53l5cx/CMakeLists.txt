idf_component_register(
    SRCS "vl53l5cx_api.c" "vl53l5cx_platform.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_i2c
)
