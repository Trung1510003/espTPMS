idf_component_register(SRCS "testBLE_Sensor.c"
                    INCLUDE_DIRS "."
                    REQUIRES bt nvs_flash driver esp_timer freertos
                    PRIV_REQUIRES bt nvs_flash)