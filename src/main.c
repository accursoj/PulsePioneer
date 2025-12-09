#include <stdio.h>
#include <nvs.h>
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "ecg.h"
#include "lcd.h"
#include "led_strip.h"
#include "led_strip_rmt.h"
#include "sdkconfig.h"

#define RGB_LED_PIN 38
#define POWER_PIN GPIO_NUM_3
#define RGB_LED_BRIGHTNESS 25

void init_gpio() {
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
    
    // Configure input pins
    io_conf.pin_bit_mask = ((1ULL << LCD_SDO_PIN) |
                            (1ULL << ECG_SDO_PIN) |
                            (1ULL << ECG_ALAB_PIN) |
                            (1ULL << ECG_DRDB_PIN));
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ESP_ERROR_CHECK(rtc_gpio_init(POWER_PIN));
    ESP_ERROR_CHECK(rtc_gpio_set_direction(POWER_PIN, RTC_GPIO_MODE_INPUT_ONLY));
    // Ensure pin will not droop low
    ESP_ERROR_CHECK(rtc_gpio_pullup_en(POWER_PIN));
    ESP_ERROR_CHECK(rtc_gpio_pulldown_dis(POWER_PIN));
}

static led_strip_handle_t board_led_handle;
void init_rgb_indicator(void) {
    
    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = RGB_LED_PIN;
    strip_config.max_leds = 1;
    strip_config.led_model = LED_MODEL_WS2812;

    led_strip_rmt_config_t strip_rmt_config = {};
    strip_rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
    strip_rmt_config.resolution_hz = 10 * 1000 * 1000;
    strip_rmt_config.flags.with_dma = 1;

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &strip_rmt_config, &board_led_handle));
}

void show_rgb_led(uint32_t color_r, uint32_t color_g, uint32_t color_b, uint32_t brightness) {
    ESP_ERROR_CHECK(led_strip_set_pixel(
        board_led_handle,
        (0*brightness)/255,
        (color_r*brightness)/255,
        (color_g*brightness)/255,
        (color_b*brightness)/255)
    );
    ESP_ERROR_CHECK(led_strip_refresh(board_led_handle));
}

void start_deep_sleep() {
    if (esp_sleep_is_valid_wakeup_gpio(POWER_PIN))      // check if GPIO3 can enable ext0 wakeup
    {
        esp_sleep_enable_ext0_wakeup(POWER_PIN, 0);
        esp_deep_sleep_start();
    }
}

void start_system_boot() {
    init_gpio();
    init_rgb_indicator();
    show_rgb_led(255, 255, 0, RGB_LED_BRIGHTNESS);        // yellow
    vTaskDelay(1000 / portTICK_PERIOD_MS);              // pause for one second

    esp_sleep_wakeup_cause_t deep_wakeup_cause = esp_sleep_get_wakeup_cause();
    if (deep_wakeup_cause != ESP_SLEEP_WAKEUP_EXT0) {
        start_deep_sleep();
    }

    if (INCLUDE_ECG) {
        init_ecg();
    }
    if (INCLUDE_LCD) {
        init_lcd();
    }

    show_rgb_led(0, 255, 0, RGB_LED_BRIGHTNESS);        // green
}

// TODO: Update
void power_down() {
    ecg_power_down();
}

void app_main() {
    start_system_boot();

    // Create ECG streaming task with priority=4 on cpu=1
    if (INCLUDE_ECG) {
        xTaskCreatePinnedToCore(ecg_stream_task, "ecg_stream_task", 4096, NULL, 4, NULL, 1);

    }

    // Create LVGL display task with priority=3 on cpu=0
    if (INCLUDE_LCD) {
        xTaskCreatePinnedToCore(lvgl_task, "lvgl_task", 4096, NULL, 3, NULL, 0);
    }
    
    show_rgb_led(0, 0, 255, RGB_LED_BRIGHTNESS); // blue

    // Wait indefinitely while tasks run
    while (1) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    };
    power_down();
}