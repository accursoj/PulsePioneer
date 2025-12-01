#include "lcd.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_st7796.h"
#include "esp_lcd_panel_ops.h"
#include "driver/ledc.h"
#include <string.h>

const gpio_num_t LED_CS_PIN = 1;
const gpio_num_t LCD_SDO_PIN = 4;
const gpio_num_t LCD_SCK_PIN = 5;
const gpio_num_t LCD_SDI_PIN = 6;
const gpio_num_t LCD_DC_RS_PIN = 7;
const gpio_num_t LCD_RST_PIN = 15;
const gpio_num_t LCD_CS_PIN = 16;

#define LCD_H_RES 480
#define LCD_V_RES 320
#define LINES_PER_DMA 80
#define PIXEL_CLK_FREQ 20000000
#define TRANS_QUEUE_DEPTH 10

#define BACKLIGHT_PWM_FREQ  5000    // 5 kHz
#define BACKLIGHT_PWM_RES   LEDC_TIMER_13_BIT
#define BACKLIGHT_DUTY      8192    // Adjust duty (0-8191 for 13-bit resolution)

esp_lcd_panel_handle_t lcd_panel_handle;

spi_host_device_t lcd_host_device;
void init_lcd() {

    lcd_host_device = SPI3_HOST;

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

    lcd_panel_handle = NULL;
    esp_lcd_panel_dev_config_t lcd_panel_config = {};
    lcd_panel_config.reset_gpio_num = LCD_RST_PIN;
    lcd_panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
    lcd_panel_config.bits_per_pixel = 16;       // RGB565

    ESP_ERROR_CHECK(esp_lcd_new_panel_st7796(lcd_io_handle, &lcd_panel_config, &lcd_panel_handle));
}

void show_boot_screen_no_dma() {
    // Configure PWM for backlight
    ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = BACKLIGHT_PWM_RES,
        .freq_hz = BACKLIGHT_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&pwm_timer);

    ledc_channel_config_t pwm_channel = {
        .gpio_num = LED_CS_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = BACKLIGHT_DUTY,
        .hpoint = 0
    };
    ledc_channel_config(&pwm_channel);

    // Define each pixel in the row as 'black'
    uint16_t black_line[LCD_H_RES];
    for (int i = 0; i < LCD_H_RES; i++) black_line[i] = 0x0000;
    
    // Draw each row
    for (int y = 0; y < LCD_V_RES; y++) {
        esp_lcd_panel_draw_bitmap(lcd_panel_handle, 0, y, LCD_H_RES, y + 1, black_line);
    }
}

void show_boot_screen() {
    // --- Configure PWM for backlight ---
    ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = BACKLIGHT_PWM_RES,
        .freq_hz = BACKLIGHT_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&pwm_timer);

    ledc_channel_config_t pwm_channel = {
        .gpio_num = LED_CS_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = BACKLIGHT_DUTY,
        .hpoint = 0
    };
    ledc_channel_config(&pwm_channel);

    // --- Prepare DMA-friendly black buffer ---
    // Buffer size: LINES_PER_DMA × LCD_H_RES
    uint16_t black_block[LINES_PER_DMA * LCD_H_RES];
    memset(black_block, 0x00, sizeof(black_block)); // Fill with black pixels

    // --- Send black pixels in chunks of LINES_PER_DMA ---
    for (int y = 0; y < LCD_V_RES; y += LINES_PER_DMA) {
        int lines_to_draw = LINES_PER_DMA;
        if (y + LINES_PER_DMA > LCD_V_RES) {
            lines_to_draw = LCD_V_RES - y; // handle last partial block
        }

        esp_lcd_panel_draw_bitmap(
            lcd_panel_handle,
            0, y,
            LCD_H_RES, y + lines_to_draw,
            black_block
        );
    }
}

