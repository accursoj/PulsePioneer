#include <stdio.h>
#include <nvs.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "driver/spi_master.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_st7796.h"


short sys_deep_sleep_en = 0;

void init_gpio() {

    #define LED_CS_PIN 1
    #define LCD_SDO_PIN 4
    #define LCD_SCK_PIN 5
    #define LCD_SDI_PIN 6
    #define LCD_DC_RS_PIN 7
    #define LCD_RST_PIN 15
    #define LCD_CS_PIN 16
    #define ECG_SCLK_PIN 9
    #define ECG_SDI_PIN 10
    #define ECG_SDO_PIN 11
    #define ECG_CSB_PIN 12
    #define ECG_ALAB_PIN 13
    #define ECG_DRDB_PIN 14
    #define RGB_LED_PIN 38
    #define POWER_PIN GPIO_NUM_3

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
                              (1ULL << ECG_ALAB_PIN) |
                              (1ULL << ECG_DRDB_PIN) |
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

void init_lcd() {
    #define LCD_H_RES 480
    #define LCD_V_RES 320
    #define LINES_PER_DMA 80
    #define PIXEL_CLK_FREQ 20 * 1000 * 1000
    #define TRANS_QUEUE_DEPTH 10

    spi_host_device_t lcd_host_device = SPI3_HOST;

    spi_bus_config_t lcd_bus_config = {};
    lcd_bus_config.mosi_io_num = LCD_SDI_PIN;
    lcd_bus_config.miso_io_num = LCD_SDO_PIN;
    lcd_bus_config.sclk_io_num = LCD_SCK_PIN;
    lcd_bus_config.max_transfer_sz = LCD_H_RES * LINES_PER_DMA * sizeof(uint16_t);

    // Not used
    lcd_bus_config.quadwp_io_num = -1;
    lcd_bus_config.quadhd_io_num = -1;
    lcd_bus_config.data4_io_num = -1;
    lcd_bus_config.data5_io_num = -1;
    lcd_bus_config.data6_io_num = -1;
    lcd_bus_config.data7_io_num = -1;

    // Defaults
    lcd_bus_config.data_io_default_level = 0;
    lcd_bus_config.flags = 0;
    lcd_bus_config.isr_cpu_id = 0;
    lcd_bus_config.intr_flags = 0;

    spi_dma_chan_t lcd_dma_chan = SPI_DMA_CH_AUTO;

    ESP_ERROR_CHECK(spi_bus_initialize(lcd_host_device, &lcd_bus_config, lcd_dma_chan));

    esp_lcd_panel_io_handle_t lcd_io_handle = NULL;
    esp_lcd_panel_io_spi_config_t lcd_io_config = {};
    lcd_io_config.dc_gpio_num = LCD_DC_RS_PIN;
    lcd_io_config.cs_gpio_num = -1;     // operate the bus exclusively
    lcd_io_config.pclk_hz = PIXEL_CLK_FREQ;
    lcd_io_config.spi_mode = 3;
    lcd_io_config.lcd_cmd_bits = 8;
    lcd_io_config.lcd_param_bits = 8;
    lcd_io_config.trans_queue_depth = TRANS_QUEUE_DEPTH;

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)lcd_host_device, &lcd_io_config, &lcd_io_handle));

    esp_lcd_panel_handle_t lcd_panel_handle = NULL;
    esp_lcd_panel_dev_config_t lcd_panel_config = {};
    lcd_panel_config.reset_gpio_num = LCD_RST_PIN;
    lcd_panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
    lcd_panel_config.bits_per_pixel = 16;       // RGB565

    ESP_ERROR_CHECK(esp_lcd_new_panel_st7796(lcd_io_handle, &lcd_panel_config, &lcd_panel_handle));
}

void init_ecg() {
    spi_host_device_t ecg_host_device = SPI2_HOST;

    spi_bus_config_t ecg_bus_config = {};
    ecg_bus_config.mosi_io_num = ECG_SDI_PIN;
    ecg_bus_config.miso_io_num = ECG_SDO_PIN;
    ecg_bus_config.sclk_io_num = ECG_SCLK_PIN;
    ecg_bus_config.max_transfer_sz = 4096;

    // Not used
    ecg_bus_config.quadwp_io_num = -1;
    ecg_bus_config.quadhd_io_num = -1;
    ecg_bus_config.data4_io_num = -1;
    ecg_bus_config.data5_io_num = -1;
    ecg_bus_config.data6_io_num = -1;
    ecg_bus_config.data7_io_num = -1;

    // Defaults
    ecg_bus_config.data_io_default_level = 0;
    ecg_bus_config.flags = 0;
    ecg_bus_config.isr_cpu_id = 0;
    ecg_bus_config.intr_flags = 0;

    spi_dma_chan_t ecg_dma_chan = SPI_DMA_CH_AUTO;

    ESP_ERROR_CHECK(spi_bus_initialize(ecg_host_device, &ecg_bus_config, ecg_dma_chan));

}

void sys_boot() {
    start_deep_sleep();
    init_gpio();
    init_lcd();
    init_ecg();
}

void app_main() {
    sys_boot();

}