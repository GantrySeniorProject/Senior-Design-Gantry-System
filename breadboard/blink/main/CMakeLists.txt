idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_i2c esp_driver_ledc
                                esp_driver_uart esp_adc esp_timer)
