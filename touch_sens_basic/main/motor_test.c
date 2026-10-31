#include <stdbool.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_err.h"

#define ENA GPIO_NUM_21
#define IN1 GPIO_NUM_22
#define IN2 GPIO_NUM_23

#define TEST_DUTY 220  // Out of 255: approximately 86% drive
#define PULSE_MS 1000

static void set_drive(unsigned duty)
{
    ESP_ERROR_CHECK(ledc_set_duty(
        LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty));
    ESP_ERROR_CHECK(ledc_update_duty(
        LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

static void stop_motor(void)
{
    set_drive(0);
    ESP_ERROR_CHECK(gpio_set_level(IN1, 0));
    ESP_ERROR_CHECK(gpio_set_level(IN2, 0));
}

void app_main(void)
{
    // Disable motor drive before configuring PWM.
    ESP_ERROR_CHECK(gpio_set_level(ENA, 0));
    ESP_ERROR_CHECK(gpio_set_direction(ENA, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(IN1, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(IN2, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(IN1, 0));
    ESP_ERROR_CHECK(gpio_set_level(IN2, 0));

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 20000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t channel = {
        .gpio_num = ENA,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel));

    // Receive commands through the USB serial connection.
    ESP_ERROR_CHECK(uart_driver_install(
        UART_NUM_0, 256, 0, 0, NULL, 0));

    bool running = false;
    bool has_stopped = false;
    TickType_t started = 0;
    TickType_t stopped = 0;

    printf("\nREADY: f=forward pulse, r=reverse pulse, s=stop; "
           "duty=%d/255, pulse=%d ms\n", TEST_DUTY, PULSE_MS);

    while (1) {
        TickType_t now = xTaskGetTickCount();

        if (running && now - started >= pdMS_TO_TICKS(PULSE_MS)) {
            stop_motor();
            running = false;
            has_stopped = true;
            stopped = now;
            printf("Stopped\n");
        }

        unsigned char command;
        if (uart_read_bytes(UART_NUM_0, &command, 1, 0) == 1) {
            if (command == 's') {
                stop_motor();
                running = false;
                has_stopped = true;
                stopped = xTaskGetTickCount();
                printf("Stopped\n");
            } else if (command == 'f' || command == 'r') {
                if (!running &&
                    (!has_stopped ||
                     now - stopped >= pdMS_TO_TICKS(1000))) {
                    gpio_set_level(IN1, command == 'f');
                    gpio_set_level(IN2, command == 'r');
                    set_drive(TEST_DUTY);
                    started = xTaskGetTickCount();
                    running = true;
                    printf("%s pulse\n",
                           command == 'f' ? "Forward" : "Reverse");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
