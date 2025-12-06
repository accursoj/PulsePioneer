#include "lcd.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
// #include "esp_lcd_io_spi.h"
// #include "esp_lcd_st7796.h"
// #include "esp_lcd_panel_ops.h"
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

#define INCLUDE_LCD 1

#define LCD_H_RES 480
#define LCD_V_RES 320
#define LINES_PER_DMA 40
#define PIXEL_CLK_FREQ 20 * 1000 * 1000
#define TRANS_QUEUE_DEPTH 10

#define BACKLIGHT_PWM_FREQ  1000    // 5 kHz
#define BACKLIGHT_PWM_RES   LEDC_TIMER_13_BIT
#define BACKLIGHT_DUTY      4096    // Adjust duty (0-8191 for 13-bit resolution)

static spi_device_handle_t lcd_spi_handle;
// static esp_lcd_panel_handle_t lcd_panel_handle;
static spi_host_device_t lcd_host_device;

void lcd_reset() {
    gpio_set_level(LCD_RST_PIN, 0);     // reset with enable-low
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(LCD_RST_PIN, 1);     // disable
    vTaskDelay(pdMS_TO_TICKS(150));
}

void lvgl_timer_cb(void *arg) {
    lv_timer_handler();
    vTaskDelay(pdMS_TO_TICKS(10));
}

void init_lcd() {

    if (!INCLUDE_LCD) {
        return;
    }

    gpio_set_level(LCD_CS_PIN, 1);
    gpio_set_level(LCD_DC_RS_PIN, 1);

    lcd_host_device = SPI3_HOST;

    spi_bus_config_t lcd_bus_config = {};
    lcd_bus_config.mosi_io_num = LCD_SDI_PIN;
    lcd_bus_config.miso_io_num = LCD_SDO_PIN;
    lcd_bus_config.sclk_io_num = LCD_SCK_PIN;
    lcd_bus_config.max_transfer_sz = LCD_H_RES * LCD_V_RES * 2; // LCD_H_RES * LINES_PER_DMA * sizeof(uint16_t);

    // Not used
    lcd_bus_config.quadwp_io_num = -1;
    lcd_bus_config.quadhd_io_num = -1;
    lcd_bus_config.data4_io_num = -1;
    lcd_bus_config.data5_io_num = -1;
    lcd_bus_config.data6_io_num = -1;
    lcd_bus_config.data7_io_num = -1;

    // Defaults
    // lcd_bus_config.data_io_default_level = 0;
    // lcd_bus_config.flags = 0;
    // lcd_bus_config.isr_cpu_id = 0;
    // lcd_bus_config.intr_flags = 0;

    spi_dma_chan_t lcd_dma_chan = SPI_DMA_CH_AUTO;

    ESP_ERROR_CHECK(spi_bus_initialize(lcd_host_device, &lcd_bus_config, lcd_dma_chan));
    vTaskDelay(50 / portTICK_PERIOD_MS);

    spi_device_interface_config_t lcd_interface_config = {};
    lcd_interface_config.clock_speed_hz = 20 * 1000 * 1000;
    lcd_interface_config.mode = 3;
    lcd_interface_config.spics_io_num = LCD_CS_PIN;
    lcd_interface_config.queue_size = 6;
    ESP_ERROR_CHECK(spi_bus_add_device(lcd_host_device, &lcd_interface_config, &lcd_spi_handle));
    vTaskDelay(50 / portTICK_PERIOD_MS);

    // esp_lcd_panel_io_handle_t lcd_io_handle = NULL;
    // esp_lcd_panel_io_spi_config_t lcd_io_config = {};
    // lcd_io_config.dc_gpio_num = LCD_DC_RS_PIN;
    // lcd_io_config.cs_gpio_num = LCD_CS_PIN;//    -1; // operate the bus exclusively
    // lcd_io_config.pclk_hz = PIXEL_CLK_FREQ;
    // lcd_io_config.spi_mode = 3;
    // lcd_io_config.lcd_cmd_bits = 8;
    // lcd_io_config.lcd_param_bits = 8;
    // lcd_io_config.trans_queue_depth = TRANS_QUEUE_DEPTH;

    // ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)lcd_spi_handle, &lcd_io_config, &lcd_io_handle));

    // // lcd_panel_handle = NULL;
    // esp_lcd_panel_dev_config_t lcd_panel_config = {};
    // lcd_panel_config.reset_gpio_num = LCD_RST_PIN;
    // lcd_panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
    // lcd_panel_config.bits_per_pixel = 16;       // RGB565

    // ESP_ERROR_CHECK(esp_lcd_new_panel_st7796(lcd_io_handle, &lcd_panel_config, &lcd_panel_handle));

    // ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_panel_handle));
    // vTaskDelay(50 / portTICK_PERIOD_MS);
    // ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_panel_handle));
    // vTaskDelay(50 / portTICK_PERIOD_MS);

    // // ESP_ERROR_CHECK(esp_lcd_panel_invert_color(lcd_panel_handle, false));
    // ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(lcd_panel_handle, true));     // required for 480x320
    // // ESP_ERROR_CHECK(esp_lcd_panel_mirror(lcd_panel_handle, false, false));



}

void lcd_send_cmd(uint8_t cmd)
{
    gpio_set_level(LCD_DC_RS_PIN, 0);

    spi_transaction_t transaction = {};
    transaction.length = 8;
    transaction.tx_buffer = &cmd;

    spi_device_transmit(lcd_spi_handle, &transaction);
}

void lcd_send_data(const uint8_t *data, int len) {
    if (len == 0) {
        return;
    }

    gpio_set_level(LCD_DC_RS_PIN, 1);
    spi_transaction_t transaction = {};
    transaction.length = len * 8;       // convert to bytes to bits
    transaction.tx_buffer = data;

    spi_device_transmit(lcd_spi_handle, &transaction);
}

void lcd_send_data16(uint16_t color) {
    uint8_t d[2] = { color >> 8, color & 0xFF };
    lcd_send_data(d, 2);
}

void lv_init_st7796() {
    lcd_reset();

    lcd_send_cmd(0x11);
    vTaskDelay(pdMS_TO_TICKS(100));

    lcd_send_cmd(0x3A);
    lcd_send_data((uint8_t[]){0x55}, 1);

    lcd_send_cmd(0x36);
    lcd_send_data((uint8_t[]){0x28}, 1);

    lcd_send_cmd(0x29);     // display on
}

void init_st7796() {
    lcd_reset();

    lcd_send_cmd(0x11);
    vTaskDelay(pdMS_TO_TICKS(100));

    lcd_send_cmd(0x3A);
    lcd_send_data((uint8_t[]){0x55}, 1);

    lcd_send_cmd(0x36);
    lcd_send_data((uint8_t[]){0x28}, 1);

    lcd_send_cmd(0x29);     // display on
}

void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t data[4];

    // CASET
    lcd_send_cmd(0x2A);
    data[0] = x0 >> 8; data[1] = x0;
    data[2] = x1 >> 8; data[3] = x1;
    lcd_send_data(data, 4);

    // RASET
    lcd_send_cmd(0x2B);
    data[0] = y0 >> 8; data[1] = y0;
    data[2] = y1 >> 8; data[3] = y1;
    lcd_send_data(data, 4);

    // RAMWR
    lcd_send_cmd(0x2C);
}

void lv_lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    uint8_t data[4];

    // CASET (column address)
    lcd_send_cmd(0x2A);
    data[0] = (x0 >> 8) & 0xFF;
    data[1] = x0 & 0xFF;
    data[2] = (x1 >> 8) & 0xFF;
    data[3] = x1 & 0xFF;
    lcd_send_data(data, 4);

    // RASET (row address)
    lcd_send_cmd(0x2B);
    data[0] = (y0 >> 8) & 0xFF;
    data[1] = y0 & 0xFF;
    data[2] = (y1 >> 8) & 0xFF;
    data[3] = y1 & 0xFF;
    lcd_send_data(data, 4);

    // RAMWR
    lcd_send_cmd(0x2C);
}

void run_display_test() {
    init_st7796();
    
    lcd_set_window(50, 50, 150, 100);

    uint16_t color = 0x07FF; // cyan
    for (int i = 0; i < (100 * 50); i++) {
        lcd_send_data16(color);
    }
    printf("Test shape drawn.");

}

static uint32_t get_esp_tick(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *color_map) {
    int32_t x1 = area->x1;
    int32_t y1 = area->y1;
    int32_t x2 = area->x2;
    int32_t y2 = area->y2;

    int32_t width  = x2 - x1 + 1;
    int32_t height = y2 - y1 + 1;

    uint16_t *pixels = (uint16_t *)color_map;

    for (int32_t y = 0; y < height; y += LINES_PER_DMA) {
        int32_t chunk_h = LINES_PER_DMA;
        if (y + chunk_h > height) chunk_h = height - y;

        // Set window for this chunk
        lv_lcd_set_window(x1, y1 + y, x2, y1 + y + chunk_h - 1);

        // Send pixels MSB-first (RGB)
        for (int32_t i = 0; i < width * chunk_h; i++) {
            uint16_t c = pixels[y * width + i];
            uint8_t data[2] = { c >> 8, c & 0xFF }; // MSB first
            lcd_send_data(data, 2);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    lv_display_flush_ready(disp);

}

void init_lvgl() {
    lv_init();
    lv_tick_set_cb(get_esp_tick);

    static lv_color_t buf1[LCD_H_RES * LINES_PER_DMA];
    static lv_color_t buf2[LCD_H_RES * LINES_PER_DMA];

    lv_display_t *disp = lv_display_create(LCD_H_RES, LCD_V_RES);

    lv_display_set_buffers(disp, buf1, buf2, sizeof(buf1), LV_DISPLAY_RENDER_MODE_PARTIAL);

    lv_display_set_flush_cb(disp, lvgl_flush_cb);

}

void lvgl_task(void *arg) {

    if (!INCLUDE_LCD) {
        return;
    }

    lv_init_st7796();
    init_lvgl();

    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    // -------------------- TEXT TEST --------------------
    lv_obj_t *label = lv_label_create(lv_screen_active());
    lv_label_set_text(label, "LVGL OK");
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -40);

    // -------------------- RECTANGLE TEST --------------------
    lv_obj_t *rect = lv_obj_create(lv_screen_active());
    lv_obj_set_size(rect, 120, 60);
    lv_obj_align(rect, LV_ALIGN_CENTER, 0, 40);

    lv_obj_set_style_bg_color(rect, lv_color_hex(0x00FF00), LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(rect, LV_OPA_COVER, LV_STATE_DEFAULT);

    while (1) {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void lvgl_test_screen() {


    lv_init_st7796();

    init_lvgl();







    // lv_timer_handler();
    // const esp_timer_create_args_t lvgl_timer_args = {
    //     .callback = lvgl_timer_cb,
    //     .name = "lvgl_tick"
    // };

    // esp_timer_handle_t lvgl_timer;
    // esp_timer_create(&lvgl_timer_args, &lvgl_timer);
    // esp_timer_start_periodic(lvgl_timer, 5 * 1000);



}

// typedef struct {
//     lv_obj_t *chart;
//     lv_chart_series_t *ch1;
//     // lv_chart_series_t *ch2;
//     // lv_chart_series_t *ch3;
// } lv_waveform_t;

// lv_waveform_t *create_waveform_plot(void) {
//     static lv_waveform_t *waveform;
//     waveform->chart = NULL;
//     waveform->ch1 = NULL;

//     // Remove all child objects from the active screen
//     lv_obj_clean(lv_screen_active());

//     // Initialize chart object
//     lv_obj_t *chart;
//     chart = lv_chart_create(lv_screen_active());
//     lv_obj_set_size(chart, 480, 300);
//     lv_obj_center(chart);
//     lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
    
//     lv_chart_series_t *ch1 = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_YELLOW), LV_CHART_AXIS_PRIMARY_Y);
    
//     lv_chart_refresh(chart);
//     lv_timer_handler();     // update active screen

//     waveform->chart = chart;
//     waveform->ch1 = ch1;
    
//     return waveform;
// }

// lv_waveform_t* update_waveform_plot(lv_waveform_t *waveform, int32_t *new_data, uint16_t new_data_size) {
//     lv_obj_t *chart = waveform->chart;

//     uint16_t i;
//     for (i = 0; i < new_data_size; i++) {
//         lv_chart_set_next_value(chart, waveform->ch1, *(new_data + i));
//     }

//     lv_chart_refresh(chart);
//     lv_timer_handler();     // update active screen

//     waveform->chart = chart;        // update waveform struct

//     return waveform;
// }



void show_ecg_error_message(const char *text) {
    if (!INCLUDE_LCD) {
        return;
    }

    lv_obj_t *error_box;
    error_box = lv_msgbox_create(lv_screen_active());
    error_box = lv_msgbox_add_text(error_box, text);

    lv_timer_handler();     // update active screen
}