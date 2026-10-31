idf_component_register(
    SRCS "motor_test.c"
    REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_uart
)
