idf_component_register(SRCS "SensorBLE_OLED.c"
        INCLUDE_DIRS "."
        REQUIRES esp_driver_i2c u8g2 u8g2-hal-esp-idf nvs_flash esp_timer bt driver)