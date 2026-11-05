#include <stdbool.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "touch_sensors.h"

// Motor harness labels verified on this unit:
// M1 white -> L298N OUT2; the red motor lead -> L298N OUT1.
// Encoder GND blue -> ESP32 GND; VCC black -> ESP32 3V3.
// Encoder C1 green -> GPIO16; C2 yellow -> GPIO17.
#define ENA GPIO_NUM_21
#define IN1 GPIO_NUM_22
#define IN2 GPIO_NUM_23
#define ENCODER_A GPIO_NUM_16
#define ENCODER_B GPIO_NUM_17
#define FORWARD_BUTTON GPIO_NUM_25
#define REVERSE_BUTTON GPIO_NUM_26

#define TEST_DUTY 220  // Out of 255: approximately 86% drive
#define DEFAULT_PULSE_MS 1000U
#define MIN_PULSE_MS 100U
#define MAX_PULSE_MS 2000U
#define PULSE_STEP_MS 100U
#define BUTTON_DEBOUNCE_MS 30U
#define DOUBLE_CLICK_MS 350U
#define MOTOR_COOLDOWN_MS 1000U
#define ENCODER_COUNTS_PER_SCREW_REV 882.0f
#define SCREW_PITCH_MM 0.5f

typedef enum {
    BUTTON_NONE,
    BUTTON_SINGLE,
    BUTTON_DOUBLE,
} button_event_t;

typedef struct {
    gpio_num_t pin;
    bool raw_high;
    bool stable_high;
    bool pending_single;
    bool ignore_release;
    TickType_t raw_changed_at;
    TickType_t first_release_at;
} button_state_t;

typedef struct {
    bool running;
    bool has_stopped;
    TickType_t started;
    TickType_t stopped;
    TickType_t last_position_report;
} motor_state_t;

static pcnt_unit_handle_t encoder_unit;
static pcnt_unit_handle_t encoder_a_edges;
static pcnt_unit_handle_t encoder_b_edges;
static bool encoder_zeroed;
static unsigned pulse_ms = DEFAULT_PULSE_MS;

static button_event_t poll_button(button_state_t *button, TickType_t now)
{
    bool raw_high = gpio_get_level(button->pin) != 0;
    if (raw_high != button->raw_high) {
        button->raw_high = raw_high;
        button->raw_changed_at = now;
    }

    if (raw_high != button->stable_high &&
        now - button->raw_changed_at >= pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS)) {
        button->stable_high = raw_high;
        if (raw_high) {
            if (button->pending_single) {
                button->pending_single = false;
                if (now - button->first_release_at <= pdMS_TO_TICKS(DOUBLE_CLICK_MS)) {
                    button->ignore_release = true;
                    return BUTTON_DOUBLE;
                }
                // This press is too late to make a double click. Its release
                // will become the first click of the next pair.
                return BUTTON_SINGLE;
            }
        } else if (button->ignore_release) {
            button->ignore_release = false;
        } else {
            button->pending_single = true;
            button->first_release_at = now;
        }
    }

    if (button->pending_single && !button->raw_high && !button->stable_high &&
        now - button->first_release_at > pdMS_TO_TICKS(DOUBLE_CLICK_MS)) {
        button->pending_single = false;
        return BUTTON_SINGLE;
    }
    return BUTTON_NONE;
}

// Diagnostic counters count each channel's rising edges independently. If one
// stays at zero, the corresponding signal is not reaching the ESP32 PCNT input.
static void init_edge_counter(gpio_num_t pin, pcnt_unit_handle_t *unit)
{
    pcnt_unit_config_t unit_config = {
        .low_limit = -1,
        .high_limit = 30000,
        .flags.accum_count = true,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, unit));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(*unit, &filter_config));

    pcnt_chan_config_t channel_config = {
        .edge_gpio_num = pin,
        .level_gpio_num = -1,
    };
    pcnt_channel_handle_t channel;
    ESP_ERROR_CHECK(pcnt_new_channel(*unit, &channel_config, &channel));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(channel,
                    PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                    PCNT_CHANNEL_EDGE_ACTION_HOLD));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(channel,
                    PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                    PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(*unit, 30000));
    ESP_ERROR_CHECK(pcnt_unit_enable(*unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(*unit));
    ESP_ERROR_CHECK(pcnt_unit_start(*unit));
}

static void init_encoder(void)
{
    pcnt_unit_config_t unit_config = {
        .low_limit = -30000,
        .high_limit = 30000,
        .flags.accum_count = true,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &encoder_unit));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(encoder_unit, &filter_config));

    // The PCNT driver does not enable input pulls. These keep an open-drain
    // encoder output at a defined 3.3 V logic level between low pulses.
    ESP_ERROR_CHECK(gpio_pullup_en(ENCODER_A));
    ESP_ERROR_CHECK(gpio_pullup_en(ENCODER_B));

    pcnt_chan_config_t channel_a_config = {
        .edge_gpio_num = ENCODER_A,
        .level_gpio_num = ENCODER_B,
    };
    pcnt_chan_config_t channel_b_config = {
        .edge_gpio_num = ENCODER_B,
        .level_gpio_num = ENCODER_A,
    };
    pcnt_channel_handle_t channel_a;
    pcnt_channel_handle_t channel_b;
    ESP_ERROR_CHECK(pcnt_new_channel(encoder_unit, &channel_a_config, &channel_a));
    ESP_ERROR_CHECK(pcnt_new_channel(encoder_unit, &channel_b_config, &channel_b));

    // Count both edges of A and B: four counts per quadrature cycle.
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(channel_a,
                    PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(channel_a,
                    PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(channel_b,
                    PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(channel_b,
                    PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Accumulate past the ESP32's 16-bit hardware counter range.
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(encoder_unit, -30000));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(encoder_unit, 30000));
    ESP_ERROR_CHECK(pcnt_unit_enable(encoder_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(encoder_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(encoder_unit));

    init_edge_counter(ENCODER_A, &encoder_a_edges);
    init_edge_counter(ENCODER_B, &encoder_b_edges);
}

static void print_position(void)
{
    int counts;
    int a_rises;
    int b_rises;
    ESP_ERROR_CHECK(pcnt_unit_get_count(encoder_unit, &counts));
    ESP_ERROR_CHECK(pcnt_unit_get_count(encoder_a_edges, &a_rises));
    ESP_ERROR_CHECK(pcnt_unit_get_count(encoder_b_edges, &b_rises));
    if (encoder_zeroed) {
        float screw_mm = counts * SCREW_PITCH_MM / ENCODER_COUNTS_PER_SCREW_REV;
        printf("Encoder: %d counts, screw travel: %+.3f mm from zero; "
               "A rises=%d B rises=%d; pulse=%u ms\n",
               counts, screw_mm, a_rises, b_rises, pulse_ms);
    } else {
        printf("Encoder: %d counts (unreferenced); A rises=%d B rises=%d; "
               "pulse=%u ms\n", counts, a_rises, b_rises, pulse_ms);
    }
}

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

static void handle_command(unsigned char command, TickType_t now, motor_state_t *motor)
{
    if (command == 's') {
        stop_motor();
        motor->running = false;
        motor->has_stopped = true;
        motor->stopped = now;
        motor->last_position_report = now;
        printf("Stopped\n");
        print_position();
    } else if (command == '+' || command == '=' || command == '-') {
        if (command == '-') {
            if (pulse_ms > MIN_PULSE_MS) {
                pulse_ms -= PULSE_STEP_MS;
            }
        } else if (pulse_ms < MAX_PULSE_MS) {
            pulse_ms += PULSE_STEP_MS;
        }
        printf("Pulse duration: %u ms%s\n", pulse_ms,
               motor->running ? " (applies to current pulse)" : "");
    } else if (command == 'p') {
        print_position();
    } else if (command == 'z') {
        if (motor->running) {
            printf("Stop motor before setting the position zero\n");
        } else {
            ESP_ERROR_CHECK(pcnt_unit_clear_count(encoder_unit));
            encoder_zeroed = true;
            printf("Position zero set at current screw position\n");
            print_position();
        }
    } else if (command == 'f' || command == 'r') {
        if (motor->running) {
            printf("Motor already running\n");
        } else if (motor->has_stopped &&
                   now - motor->stopped < pdMS_TO_TICKS(MOTOR_COOLDOWN_MS)) {
            printf("Motor cooldown; try again in a moment\n");
        } else {
            ESP_ERROR_CHECK(gpio_set_level(IN1, command == 'f'));
            ESP_ERROR_CHECK(gpio_set_level(IN2, command == 'r'));
            set_drive(TEST_DUTY);
            motor->started = now;
            motor->last_position_report = now;
            motor->running = true;
            printf("%s pulse: %u ms\n",
                   command == 'f' ? "Forward" : "Reverse", pulse_ms);
        }
    }
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

    init_encoder();

    // Active-high buttons: each switch connects its GPIO to ESP32 3V3.
    gpio_config_t buttons = {
        .pin_bit_mask = (1ULL << FORWARD_BUTTON) | (1ULL << REVERSE_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&buttons));
    bool forward_held_at_boot = gpio_get_level(FORWARD_BUTTON) != 0;
    bool reverse_held_at_boot = gpio_get_level(REVERSE_BUTTON) != 0;
    button_state_t forward_button = {
        .pin = FORWARD_BUTTON,
        .raw_high = forward_held_at_boot,
        .stable_high = forward_held_at_boot,
        .ignore_release = forward_held_at_boot,
    };
    button_state_t reverse_button = {
        .pin = REVERSE_BUTTON,
        .raw_high = reverse_held_at_boot,
        .stable_high = reverse_held_at_boot,
        .ignore_release = reverse_held_at_boot,
    };

    // Receive commands through the USB serial connection.
    ESP_ERROR_CHECK(uart_driver_install(
        UART_NUM_0, 256, 0, 0, NULL, 0));

    motor_state_t motor = {0};
    motor.last_position_report = xTaskGetTickCount();

    // The sensor task streams ten capacitive readings independently of motor motion.
    if (xTaskCreate(touch_sensor_task, "touch_stream", 8192, NULL,
                    tskIDLE_PRIORITY, NULL) != pdPASS) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }

    printf("\nREADY: f=forward pulse, r=reverse pulse, s=stop, "
           "+/=longer, -=shorter (%u ms steps), "
           "p=position, z=zero at known open position; "
           "duty=%d/255, pulse=%u ms (%u-%u ms)\n",
           PULSE_STEP_MS, TEST_DUTY, pulse_ms, MIN_PULSE_MS, MAX_PULSE_MS);
    printf("Buttons: GPIO%d single=forward, double=longer; "
           "GPIO%d single=reverse, double=shorter (debounce %u ms, double window %u ms)\n",
           FORWARD_BUTTON, REVERSE_BUTTON, BUTTON_DEBOUNCE_MS, DOUBLE_CLICK_MS);

    while (1) {
        TickType_t now = xTaskGetTickCount();

        if (motor.running && now - motor.started >= pdMS_TO_TICKS(pulse_ms)) {
            stop_motor();
            motor.running = false;
            motor.has_stopped = true;
            motor.stopped = now;
            printf("Stopped\n");
            print_position();
        }

        TickType_t report_period = pdMS_TO_TICKS(motor.running ? 100 : 500);
        if (now - motor.last_position_report >= report_period) {
            print_position();
            motor.last_position_report = now;
        }

        button_event_t forward_event = poll_button(&forward_button, now);
        button_event_t reverse_event = poll_button(&reverse_button, now);
        if (forward_event != BUTTON_NONE) {
            handle_command(forward_event == BUTTON_DOUBLE ? '+' : 'f', now, &motor);
        }
        if (reverse_event != BUTTON_NONE) {
            handle_command(reverse_event == BUTTON_DOUBLE ? '-' : 'r', now, &motor);
        }

        unsigned char command;
        if (uart_read_bytes(UART_NUM_0, &command, 1, 0) == 1) {
            handle_command(command, now, &motor);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
