idf_component_register(
    SRCS "touch_sens_basic_example_main.c" "motor_test.c"
    REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_pcnt esp_driver_uart
             esp_driver_touch_sens esp_event esp_netif esp_wifi nvs_flash
             touch_sens_examples_common
)
