/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/touch_sens.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "touch_sens_example_config.h"

// Touch version 3 supports multiple sample configurations (i.e. supports frequency hopping),
// others only have one set of sample configurations.
#define EXAMPLE_TOUCH_SAMPLE_CFG_NUM        TOUCH_SAMPLE_CFG_NUM  // Up to 'TOUCH_SAMPLE_CFG_NUM'
#define EXAMPLE_TOUCH_CHANNEL_NUM           10
#define EXAMPLE_TOUCH_CHAN_INIT_SCAN_TIMES  3

ESP_STATIC_ASSERT(EXAMPLE_TOUCH_SAMPLE_CFG_NUM <= TOUCH_SAMPLE_CFG_NUM, "sample configuration number exceed the supported number");
ESP_STATIC_ASSERT(EXAMPLE_TOUCH_CHANNEL_NUM <= (TOUCH_MAX_CHAN_ID - TOUCH_MIN_CHAN_ID + 1), "touch channel number exceed the max supported number ");

// Active threshold to benchmark ratio. (i.e., touch will be activated when data >= benchmark * (1 + ratio))
static float s_thresh2bm_ratio[EXAMPLE_TOUCH_CHANNEL_NUM] = {
    [0 ... EXAMPLE_TOUCH_CHANNEL_NUM - 1] = 0.015f,  // 1.5%
};
// The touch channel IDs that used in this example
// For the corresponding GPIOs of these channel, please refer to 'touch_sensor_channel.h'
static int s_channel_id[EXAMPLE_TOUCH_CHANNEL_NUM] = {
    TOUCH_MIN_CHAN_ID,
    TOUCH_MIN_CHAN_ID + 1,
    TOUCH_MIN_CHAN_ID + 2,
    TOUCH_MIN_CHAN_ID + 3,
    TOUCH_MIN_CHAN_ID + 4,
    TOUCH_MIN_CHAN_ID + 5,
    TOUCH_MIN_CHAN_ID + 6,
    TOUCH_MIN_CHAN_ID + 7,
    TOUCH_MIN_CHAN_ID + 8,
    TOUCH_MIN_CHAN_ID + 9
};

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_CONNECT_TIMEOUT_MS 15000
#define SNTP_SYNC_TIMEOUT_MS 10000

static EventGroupHandle_t s_wifi_event_group;
static bool s_sntp_started;

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_data;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        if (!s_sntp_started && esp_netif_sntp_start() == ESP_OK) {
            s_sntp_started = true;
        }
    }
}

static void initialize_wifi_time_sync(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    esp_sntp_config_t sntp_config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    sntp_config.start = false;
    sntp_config.renew_servers_after_new_IP = true;
    ESP_ERROR_CHECK(esp_netif_sntp_init(&sntp_config));

    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL));

    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_TOUCH_WIFI_SSID,
            .password = CONFIG_TOUCH_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE,
                                           pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));
    if ((bits & WIFI_CONNECTED_BIT) == 0) {
        printf("Wi-Fi not connected yet; sensor logging continues and will retry\n");
        return;
    }

    if (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(SNTP_SYNC_TIMEOUT_MS)) != ESP_OK) {
        printf("UTC time not synchronized yet; timestamp field will be 0 until SNTP succeeds\n");
    } else {
        printf("UTC time synchronized with SNTP\n");
    }
}

static uint64_t current_utc_timestamp_us(void)
{
    struct timeval now;
    struct tm utc_time;
    gettimeofday(&now, NULL);
    if (gmtime_r(&now.tv_sec, &utc_time) == NULL || utc_time.tm_year < (2024 - 1900)) {
        return 0;
    }
    return (uint64_t)now.tv_sec * 1000000ULL + (uint64_t)now.tv_usec;
}

void app_main(void)
{
    /* Handles of touch sensor */
    touch_sensor_handle_t sens_handle = NULL;
    touch_channel_handle_t chan_handle[EXAMPLE_TOUCH_CHANNEL_NUM];

    /* Step 1: Create a new touch sensor controller handle with default sample configuration */
    touch_sensor_sample_config_t sample_cfg[TOUCH_SAMPLE_CFG_NUM] = EXAMPLE_TOUCH_SAMPLE_CFG_DEFAULT();
    touch_sensor_config_t sens_cfg = TOUCH_SENSOR_DEFAULT_BASIC_CONFIG(EXAMPLE_TOUCH_SAMPLE_CFG_NUM, sample_cfg);
    ESP_ERROR_CHECK(touch_sensor_new_controller(&sens_cfg, &sens_handle));

    /* Step 2: Create and enable the new touch channel handles with default configurations */
    /** Following is about setting the touch channel active threshold of each sample configuration.
     *
     *  @How to Determine:
     *  As the actual threshold is affected by various factors in real application,
     *  we need to run the touch app first to get the `benchmark` and the `smooth_data` that being touched.
     *
     *  @Formula:
     *  Touch V1 uses absolute threshold, and it has no benchmark, so you can use untouched smooth data instead:
     *      abs_active_thresh = benchmark * (1 - coeff), (coeff for example, 0.1%~20%)
     *  Touch V2/V3 uses relative threshold:
     *      active_thresh = benchmark * coeff, (coeff for example, 0.1%~20%)
     *  Please adjust the coeff to guarantee the threshold < smooth_data - benchmark
     *
     *  @Typical Practice:
     *  Normally, we can't determine a fixed threshold at the beginning,
     *  but we can give them estimated values first and update them after an initial scanning (like this example),
     *  Step1: set an estimated value for each sample configuration first. (i.e., here)
     *  Step2: then reconfig the threshold after the initial scanning.(see `example_touch_do_initial_scanning`)
     *  Step3: adjust the `s_thresh2bm_ratio` to a proper value to trigger the active callback
     */
    touch_channel_config_t chan_cfg = EXAMPLE_TOUCH_CHAN_CFG_DEFAULT();
    /* Allocate new touch channel on the touch controller */
    for (int i = 0; i < EXAMPLE_TOUCH_CHANNEL_NUM; i++) {
        ESP_ERROR_CHECK(touch_sensor_new_channel(sens_handle, s_channel_id[i], &chan_cfg, &chan_handle[i]));
        /* Display the touch channel corresponding GPIO number, you can also know from `touch_sensor_channel.h` */
        touch_chan_info_t chan_info = {};
        ESP_ERROR_CHECK(touch_sensor_get_channel_info(chan_handle[i], &chan_info));
        printf("Touch [CH %d] enabled on GPIO%d\n", s_channel_id[i], chan_info.chan_gpio);
    }
    printf("=================================\n");
    initialize_wifi_time_sync();
    printf("The final numeric field in each sensor row is UTC Unix time in microseconds (0 means unsynchronized)\n");

    /* Step 3: Confiture the default filter for the touch sensor (Note: Touch V1 uses software filter) */
    touch_sensor_filter_config_t filter_cfg = TOUCH_SENSOR_DEFAULT_FILTER_CONFIG();
    ESP_ERROR_CHECK(touch_sensor_config_filter(sens_handle, &filter_cfg));

    /* Step 6: Enable the touch sensor */
    ESP_ERROR_CHECK(touch_sensor_enable(sens_handle));

    /* Step 7: Start continuous scanning, you can also trigger oneshot scanning manually */
    ESP_ERROR_CHECK(touch_sensor_start_continuous_scanning(sens_handle));

    /* Step8: Print the sampled data of each enabled touch channel */
    uint32_t data[EXAMPLE_TOUCH_SAMPLE_CFG_NUM] = {};
    while (1) {
            /* Read and print the channel data of each sample configuration */
            for(int i=0; i<EXAMPLE_TOUCH_CHANNEL_NUM; i++) {
                ESP_ERROR_CHECK(touch_channel_read_data(chan_handle[i], TOUCH_CHAN_DATA_TYPE_SMOOTH, data));
                // printf("smooth    [CH %d]:", s_channel_id[i]);
                for (int j = 0; j < EXAMPLE_TOUCH_SAMPLE_CFG_NUM; j++) {
                    printf(" %"PRIu32",", data[j]);
                }
                
            }
            printf(" %"PRIu64"\n", current_utc_timestamp_us());
        /* Read and display the data every 20 ms */
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
