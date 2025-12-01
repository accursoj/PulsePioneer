#include <stdio.h>
#include <nvs.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "ecg.h"
#include "lcd.h"

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

void system_boot() {
    start_deep_sleep();
    init_gpio();
    init_lcd();
    init_ecg();
}

void power_down() {
    ecg_power_down();
}

void app_main() {
    system_boot();
    show_boot_screen();
    // TODO: Check ECG alarms
    stream_ecg_data();
    while(1) {}
    power_down();
}