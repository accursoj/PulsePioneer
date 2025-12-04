#include "lcd.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_st7796.h"
#include "esp_lcd_panel_ops.h"
#include "driver/ledc.h"
#include <string.h>
#include <lvgl.h>
#include "esp_timer.h"

const gpio_num_t LED_CS_PIN = 1;
const gpio_num_t LCD_SDO_PIN = 4;
const gpio_num_t LCD_SCK_PIN = 5;
const gpio_num_t LCD_SDI_PIN = 6;
const gpio_num_t LCD_DC_RS_PIN = 7;
const gpio_num_t LCD_RST_PIN = 15;
const gpio_num_t LCD_CS_PIN = 16;

#define INCLUDE_LCD 0

#define LCD_H_RES 480
#define LCD_V_RES 320
#define LINES_PER_DMA 80
#define PIXEL_CLK_FREQ 20000000
#define TRANS_QUEUE_DEPTH 10

#define BACKLIGHT_PWM_FREQ  1000    // 5 kHz
#define BACKLIGHT_PWM_RES   LEDC_TIMER_13_BIT
#define BACKLIGHT_DUTY      8192    // Adjust duty (0-8191 for 13-bit resolution)

esp_lcd_panel_handle_t lcd_panel_handle;

spi_host_device_t lcd_host_device;
void init_lcd() {

    if (!INCLUDE_LCD) {
        return;
    }

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

    ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_panel_handle));

    // ESP_ERROR_CHECK(esp_lcd_panel_invert_color(lcd_panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(lcd_panel_handle, true));     // required for 480x320
    // ESP_ERROR_CHECK(esp_lcd_panel_mirror(lcd_panel_handle, false, false));
}

static uint32_t get_esp_tick(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *color_map) {
    int x1 = area->x1;
    int y1 = area->y1;
    int x2 = area->x2;
    int y2 = area->y2;

    // esp_lcd expects width/height as (end_x, end_y), exclusive
    esp_lcd_panel_draw_bitmap(
        lcd_panel_handle,
        x1, y1,
        x2 + 1, y2 + 1,
        color_map
    );

    lv_display_flush_ready(disp);
}

void init_lvgl() {
    lv_init();
    lv_tick_set_cb(get_esp_tick);

    static lv_draw_buf_t buf1[LCD_H_RES * LINES_PER_DMA];

    lv_display_t *disp = lv_display_create(LCD_H_RES, LCD_V_RES);

    lv_display_set_flush_cb(disp, lvgl_flush_cb);

    lv_display_set_draw_buffers(disp, buf1, NULL);
}

void show_boot_screen_no_dma() {
    if (!INCLUDE_LCD) {
        return;
    }

    // Configure PWM for backlight
    ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_SPEED_MODE_MAX,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = BACKLIGHT_PWM_RES,
        .freq_hz = BACKLIGHT_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&pwm_timer);

    ledc_channel_config_t pwm_channel = {
        .gpio_num = LED_CS_PIN,
        .speed_mode = LEDC_SPEED_MODE_MAX,
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

static lv_style_t *create_black_boot_style(void) {
    static lv_style_t style;
    static bool initialized = false;

    if (!initialized) {
        lv_style_init(&style);
        lv_style_set_bg_color(&style, lv_color_black());
        lv_style_set_bg_opa(&style, LV_OPA_COVER);
        initialized = true;
    }
    return &style;
}

void show_boot_screen_lvgl() {
    if (!INCLUDE_LCD) {
        return;
    }

    // Configure PWM for backlight
    ledc_timer_config_t pwm_timer = {};
    pwm_timer.speed_mode = LEDC_SPEED_MODE_MAX;
    pwm_timer.timer_num = LEDC_TIMER_1;
    pwm_timer.duty_resolution = BACKLIGHT_PWM_RES;
    pwm_timer.freq_hz = BACKLIGHT_PWM_FREQ;
    pwm_timer.clk_cfg = LEDC_AUTO_CLK;

    ledc_timer_config(&pwm_timer);

    ledc_channel_config_t pwm_channel = {};
    pwm_channel.gpio_num = LED_CS_PIN;
    pwm_channel.speed_mode = LEDC_SPEED_MODE_MAX;
    pwm_channel.channel = LEDC_CHANNEL_1;
    pwm_channel.intr_type = LEDC_INTR_DISABLE;
    pwm_channel.timer_sel = LEDC_TIMER_1;
    pwm_channel.duty = BACKLIGHT_DUTY;
    pwm_channel.hpoint = 0;

    ledc_channel_config(&pwm_channel);

    // Create boot screen using LVGL obejcts
    lv_obj_t *screen = lv_obj_create(NULL);            // Create a blank LVGL screen
    lv_obj_remove_style_all(screen);                   // Remove default white styling
    lv_obj_add_style(screen, create_black_boot_style(), 0);

    // Create "booting..." text
    lv_obj_t *label = lv_label_create(screen);
    lv_label_set_text(label, "Booting Pulse Pioneer...");

    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_set_style_text_font(label, LV_FONT_DEFAULT, 0);
    lv_obj_center(label);

    lv_scr_load(screen);                            // Load as the active screen

    // Force LVGL to update immediately
    lv_timer_handler();                             
    lv_timer_handler();                             // Call twice to ensure full flush
}

void show_boot_screen() {
    if (!INCLUDE_LCD) {
        return;
    }
    
    // --- Configure PWM for backlight ---
    ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_SPEED_MODE_MAX,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = BACKLIGHT_PWM_RES,
        .freq_hz = BACKLIGHT_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&pwm_timer);

    ledc_channel_config_t pwm_channel = {
        .gpio_num = LED_CS_PIN,
        .speed_mode = LEDC_SPEED_MODE_MAX,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = BACKLIGHT_DUTY,
        .hpoint = 0
    };
    ledc_channel_config(&pwm_channel);

    // DMA Buffer size = LINES_PER_DMA * LCD_H_RES
    uint16_t black_block[LINES_PER_DMA * LCD_H_RES];
    memset(black_block, 0x00, sizeof(black_block)); // Fill with black pixels

    // Send black pixels in chunks of LINES_PER_DMA
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

typedef struct {
    lv_obj_t *chart;
    lv_chart_series_t *ch1;
    // lv_chart_series_t *ch2;
    // lv_chart_series_t *ch3;
} lv_waveform_t;
lv_waveform_t *waveform;

lv_waveform_t* update_waveform_plot(int32_t *new_data, uint16_t new_data_size) {
    lv_obj_t *chart = waveform->chart;

    uint16_t i;
    for (i = 0; i < new_data_size; i++) {
        lv_chart_set_next_value(chart, waveform->ch1, *(new_data + i));
    }

    lv_chart_refresh(chart);

    waveform->chart = chart;        // update waveform struct

    lv_timer_handler();     // update active screen

    return waveform;
}

lv_waveform_t *create_waveform_plot(void) {
    // Remove all child objects from the active screen
    lv_obj_clean(lv_screen_active());

    // Initialize chart object
    lv_obj_t *chart;
    chart = lv_chart_create(lv_screen_active());
    lv_obj_set_size(chart, 480, 300);
    lv_obj_center(chart);
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
    
    lv_chart_series_t *ch1 = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_YELLOW), LV_CHART_AXIS_PRIMARY_Y);
    
    lv_chart_refresh(chart);

    waveform->chart = chart;
    waveform->ch1 = ch1;
    
    lv_timer_handler();     // update active screen

    return waveform;
}

void show_ecg_error_message(const char *text) {
    if (!INCLUDE_LCD) {
        return;
    }

    lv_obj_t *error_box;
    error_box = lv_msgbox_create(lv_screen_active());
    error_box = lv_msgbox_add_text(error_box, text);

    lv_timer_handler();     // update active screen
}