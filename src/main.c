#include <stdio.h>
#include <nvs.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"


short sys_deep_sleep_en = 0;

void init_gpio() {

    #define LED_CS 1
    #define LCD_SDO 4
    #define LCD_SCK 5
    #define LCD_SDI 6
    #define LCD_DC_RS 7
    #define LCD_RST 15
    #define LCD_CS 16
    #define ECG_SCLK 9
    #define ECG_SDI 10
    #define ECG_SDO 11
    #define ECG_CSB 12
    #define ECG_ALAB 13
    #define ECG_DRDB 14
    #define RGB_LED 38
    #define POWER_PIN GPIO_NUM_3

    gpio_config_t io_conf = {};

    // Configure output pins
    io_conf.pin_bit_mask = ((1ULL << LED_CS) |
                              (1ULL << LCD_SDO) |
                              (1ULL << LCD_SCK) |
                              (1ULL << LCD_DC_RS) |
                              (1ULL << LCD_RST) |
                              (1ULL << LCD_CS) |
                              (1ULL << ECG_SCLK) |
                              (1ULL << ECG_SDO) |
                              (1ULL << ECG_CSB) |
                              (1ULL << ECG_ALAB) |
                              (1ULL << ECG_DRDB) |
                              (1ULL << RGB_LED));
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = 0;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    // Configure input pins
    io_conf.pin_bit_mask = ((1ULL << LCD_SDI) |
                            (1ULL << ECG_SDI) |
                            (1ULL << ECG_ALAB) |
                            (1ULL << ECG_DRDB));
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    gpio_config(&io_conf);
}

void start_modem_sleep() {}

void start_deep_sleep() {
    if (esp_sleep_is_valid_wakeup_gpio(POWER_PIN))      // check if GPIO3 can enable ext0 wakeup
    {
        esp_sleep_enable_ext0_wakeup(POWER_PIN, 1);
    }
}

void app_main() {
    init_gpio();
    start_modem_sleep();
    start_deep_sleep();
}