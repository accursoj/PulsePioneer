#include <stdio.h>
#include <nvs.h>
#include "driver/gpio.h"
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

void init_gpio() {
    gpio_config_t io_conf = {};

    // Configure output pins
    io_conf.pin_bit_mask = ((1ULL << LED_CS_PIN) |
                              (1ULL << LCD_SDO_PIN) |
                              (1ULL << LCD_SCK_PIN) |
                              (1ULL << LCD_DC_RS_PIN) |
                              (1ULL << LCD_RST_PIN) |
                              (1ULL << LCD_CS_PIN) |
                              (1ULL << ECG_SCLK_PIN) |
                              (1ULL << ECG_SDO_PIN) |
                              (1ULL << ECG_CSB_PIN) |
                              (1ULL << RGB_LED_PIN));
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = 0;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    // Configure input pins
    io_conf.pin_bit_mask = ((1ULL << LCD_SDI_PIN) |
                            (1ULL << ECG_SDI_PIN) |
                            (1ULL << ECG_ALAB_PIN) |
                            (1ULL << ECG_DRDB_PIN));
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    gpio_config(&io_conf);
}

void start_deep_sleep() {
    if (esp_sleep_is_valid_wakeup_gpio(POWER_PIN))      // check if GPIO3 can enable ext0 wakeup
    {
        esp_sleep_enable_ext0_wakeup(POWER_PIN, 1);
        esp_deep_sleep_start();
    }
}
static led_strip_handle_t board_led_handle = NULL;
void start_rgb_indicator(void) {
    
    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = RGB_LED_PIN;
    strip_config.max_leds = 1;
    strip_config.led_model = LED_MODEL_WS2812;

    led_strip_rmt_config_t strip_rmt_config = {};
    strip_rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
    strip_rmt_config.resolution_hz = 10 * 1000 * 1000;
    strip_rmt_config.flags.with_dma = 1;

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &strip_rmt_config, &board_led_handle));

    ESP_ERROR_CHECK(led_strip_set_pixel(board_led_handle, 0, 255, 255, 0));

    ESP_ERROR_CHECK(led_strip_refresh(board_led_handle));
    // ledc_timer_config_t ledc_timer = {};
    // ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
    // ledc_timer.timer_num = LEDC_TIMER_0;
    // ledc_timer.duty_resolution = LEDC_TIMER_13_BIT;
    // ledc_timer.freq_hz = 4000;
    // ledc_timer.clk_cfg = LEDC_AUTO_CLK;

    // ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // ledc_channel_config_t ledc_channel = {};
    // ledc_channel.gpio_num = RGB_LED_PIN;
    // ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    // ledc_channel.channel = LEDC_CHANNEL_0;
    // ledc_channel.intr_type = LEDC_INTR_DISABLE;
    // ledc_channel.timer_sel = LEDC_TIMER_0;
    // ledc_channel.duty = 0;
    // ledc_channel.hpoint = 0;

    // ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 4096));
}

void system_boot() {
    init_gpio();
    start_rgb_indicator();
    vTaskDelay(1000 / portTICK_PERIOD_MS);      // pause for one second
    start_deep_sleep();
    init_ecg();
    init_lcd();
    show_boot_screen_lvgl();
}

void show_green_board_led() {
    ESP_ERROR_CHECK(led_strip_set_pixel(board_led_handle, 0, 0, 255, 0));
    ESP_ERROR_CHECK(led_strip_refresh(board_led_handle));
}

void power_down() {
    ecg_power_down();
}

void app_main() {
    system_boot();
    show_boot_screen();
    show_boot_screen_lvgl();
    // TODO: Check ECG alarms
    stream_ecg_data();
    show_green_board_led();
    while (1){}

    power_down();
}