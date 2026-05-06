/**
 * @file main.c
 * @brief Main application entry point for the PulsePioneer system.
 *
 * This file orchestrates the initialization of hardware peripherals,
 * TensorFlow Lite Micro, and launches the core FreeRTOS tasks that run 
 * the ECG streaming, display rendering (LVGL), and ML inference.
 */
#include <stdio.h>
#include <nvs.h>
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "led_strip.h"
#include "led_strip_rmt.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

#include "ecg.h"
#include "lcd.h"
#include "gui.h"
#include "tflm_wrapper.h"

static const char *TAG = "main.c";

/** @brief GPIO pin for the onboard RGB LED */
#define RGB_LED_PIN 38
/** @brief Default brightness for the RGB LED (0-255) */
#define RGB_LED_BRIGHTNESS 25

/** @brief Flag to enable detailed debug logging */
#define _TESTING 1

/** @brief Handle for the ECG streaming task */
TaskHandle_t ecg_stream_task_handle = NULL;
/** @brief Handle for the LVGL display task */
TaskHandle_t lvgl_task_handle = NULL;
/** @brief Handle for the user input task */
TaskHandle_t input_task_handle = NULL;
/** @brief Handle for the GUI rendering task */
TaskHandle_t gui_task_handle = NULL;
/** @brief Handle for the TFLM inference task */
TaskHandle_t inference_task_handle = NULL;

/** @brief Mutex to protect access to the RGB LED */
SemaphoreHandle_t rgb_led_mutex = NULL;
/** @brief Handle for the RMT-based RGB LED strip */
led_strip_handle_t board_led_handle = NULL;


/**
 * Setup GPIO pins on the ESP32-S3 DevKitC-1.
 * 
 * Initializes outputs, regular inputs, interrupt-enabled inputs, and RTC inputs
 */

void init_gpio() {
    if (_TESTING) ESP_LOGI(TAG, "In init_gpio()");
    gpio_config_t io_conf = {};

    // Configure output pins
    io_conf.pin_bit_mask = ((1ULL << LED_CS_PIN) |
                              (1ULL << LCD_SDI_PIN) |
                              (1ULL << LCD_SCK_PIN) |
                              (1ULL << LCD_DC_RS_PIN) |
                              (1ULL << LCD_RST_PIN) |
                              (1ULL << LCD_CS_PIN) |
                              (1ULL << ECG_SCLK_PIN) |
                              (1ULL << ECG_SDI_PIN) |
                              (1ULL << ECG_CSB_PIN) |
                              (1ULL << RGB_LED_PIN));
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLUP_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    // Configure regular input pins
    io_conf.pin_bit_mask = ((1ULL << LCD_SDO_PIN) |
                            (1ULL << ECG_SDO_PIN) |
                            // (1ULL << ECG_ALAB_PIN) |
                            (1ULL << ECG_DRDB_PIN));
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Configure interrupt-enabled inputs
    io_conf.pin_bit_mask = (1ULL << ECG_ALAB_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Configure RTC inputs
    ESP_ERROR_CHECK(rtc_gpio_init(POWER_BTN_PIN));
    ESP_ERROR_CHECK(rtc_gpio_set_direction(POWER_BTN_PIN, RTC_GPIO_MODE_INPUT_ONLY));
    // Ensure pin will not droop low
    ESP_ERROR_CHECK(rtc_gpio_pullup_en(POWER_BTN_PIN));
    ESP_ERROR_CHECK(rtc_gpio_pulldown_dis(POWER_BTN_PIN));
}

/**
 * @brief Initialize ESP32-S3 DevKitC-1 onboard RGB LED as a new RMT device.
 *
 * @note Sets up the WS2812 LED model parameters and initializes the shared mutex.
 */
void init_rgb_indicator(void) {
    if (_TESTING) ESP_LOGI(TAG, "In init_rgb_indicator()");

    rgb_led_mutex = xSemaphoreCreateMutex();
    if (rgb_led_mutex == NULL) {
        ESP_LOGE("LED", "Failed to create RGB LED mutex!");
    }

    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = RGB_LED_PIN;
    strip_config.max_leds = 1;
    strip_config.led_model = LED_MODEL_WS2812;

    led_strip_rmt_config_t strip_rmt_config = {};
    strip_rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
    strip_rmt_config.resolution_hz = 10 * 1000 * 1000;
    strip_rmt_config.flags.with_dma = 1;

    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &strip_rmt_config, &board_led_handle));
}

/**
 * @brief Initializes critical subsystems before starting system tasks.
 * 
 * @note Initializes GPIO, on-board RGB indicator, ADS1293 (ECG), and the LCD.
 *       - Shows yellow RGB LED after GPIO and RGB init (stays yellow while in deep-sleep).
 *       - Shows green RGB LED after waking up, successful ADS1293 init, and LCD init.
 *       - Sets up the TensorFlow Lite Micro (TFLM) memory arena.
 */
void start_system_boot() {
    if (_TESTING) ESP_LOGI(TAG, "In start_system_boot()");

    init_gpio();
    init_rgb_indicator();
    show_rgb_led(255, 255, 0, RGB_LED_BRIGHTNESS);        // yellow
    vTaskDelay(pdMS_TO_TICKS(1000));              // pause for one second

    // Catch the system upon initial startup and send it to deep sleep (standby) mode
    esp_sleep_wakeup_cause_t deep_wakeup_cause = esp_sleep_get_wakeup_cause();
    if (deep_wakeup_cause != ESP_SLEEP_WAKEUP_EXT0) {       // if initial boot
        start_deep_sleep(false);
    } // else continue

    // Initialize tflm arena before initializing ECG and LVGL
    if (!tflm_init()) {
        ESP_LOGE(TAG, "tflm_init failed during boot.");
    } else {
        ESP_LOGI(TAG, "tflm_init completed successfully during boot.");
    }

    if (INCLUDE_LCD) init_lcd();
    if (INCLUDE_ECG) init_ecg();

    show_rgb_led(0, 255, 0, RGB_LED_BRIGHTNESS);        // green

    return;
}

/**
 * @brief Application entry point.
 * 
 * @details Boots the system, creates and pins all necessary FreeRTOS tasks 
 *          (inference, UI, ECG stream) to their respective cores, and idles indefinitely.
 */
void app_main() {
    if (_TESTING) ESP_LOGI(TAG, "In app_main()");

    start_system_boot();

    // Dev note: stack size is defined in words (4 bytes)
    // Dev note: lower uxPriority is lower priority (0 is Idle)
    //              Use lower uxPriority values for non-critical tasks
    //              Use high uxPriority values for critical tasks
    if (INCLUDE_LCD) {
        // Create the inference task(implemented in tflm_wrapper.cc) with priority=1 on core 0
        xTaskCreatePinnedToCore(inference_task, "inference_task", 4096, NULL, 1, &inference_task_handle, 0);
        // Create the LVGL task (implemented in lcd.c) with priority=3 on core 1
        xTaskCreatePinnedToCore(lvgl_task, "lvgl_task", 8192, NULL, 3, &lvgl_task_handle, 1);
        // Create the user input task (implemented in lcd.c) with priority=10 on core 1
        xTaskCreatePinnedToCore(input_task, "input_task", 4096, NULL, 10, &input_task_handle, 1);
        // Create the GUI task (implemented in gui.c) with priority=5 on core 1
        xTaskCreatePinnedToCore(gui_task, "gui_task", 4096, NULL, 5, &gui_task_handle, 1);

    }  

    // Create the ECG streaming task (implemented in ecg.c) with priority=4 on core 0
    // Dev note: this must have the same priority and core selection as alarm_handler_task (see ecg.c)
    if (INCLUDE_ECG) {
        if (xTaskCreatePinnedToCore(ecg_stream_task, "ecg_stream_task", 4096, NULL, 4, &ecg_stream_task_handle, 0) != pdPASS) {
            ESP_LOGE(TAG, "ECG stream task could not be created.");
        }
    } 

    vTaskDelay(pdMS_TO_TICKS(1000));

    // Print the leftover stack height for each task
    if (INCLUDE_ECG && ecg_stream_task_handle) ESP_LOGI(TAG, "uxTaskGetStackHighWaterMark2(ecg_stream_task_handle) returned %ld", uxTaskGetStackHighWaterMark2(ecg_stream_task_handle));
    if (INCLUDE_LCD && lvgl_task_handle) ESP_LOGI(TAG, "uxTaskGetStackHighWaterMark2(lvgl_task_handle) returned %ld", uxTaskGetStackHighWaterMark2(lvgl_task_handle));
    if (INCLUDE_LCD && input_task_handle) ESP_LOGI(TAG, "uxTaskGetStackHighWaterMark2(input_task_handle) returned %ld", uxTaskGetStackHighWaterMark2(input_task_handle));
    if (INCLUDE_LCD && gui_task_handle) ESP_LOGI(TAG, "uxTaskGetStackHighWaterMark2(gui_task_handle) returned %ld", uxTaskGetStackHighWaterMark2(gui_task_handle));
    if (INCLUDE_LCD && inference_task_handle) ESP_LOGI(TAG, "uxTaskGetStackHighWaterMark2(inference_task_handle) returned %ld", uxTaskGetStackHighWaterMark2(inference_task_handle));

    // Wait indefinitely while tasks run
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    };
}