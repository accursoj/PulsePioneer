#include "lcd.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
// #include "esp_lcd_io_spi.h"
// #include "esp_lcd_st7796.h"
// #include "esp_lcd_panel_ops.h"
#include "driver/ledc.h"
#include "driver/gptimer.h"
#include <string.h>
#include <lvgl.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_task_wdt.h"
#include "ecg.h"

#include "esp_log.h"
static const char *TAG = "lcd.c";

const gpio_num_t LED_CS_PIN = 1;
const gpio_num_t LCD_SDO_PIN = 4;
const gpio_num_t LCD_SCK_PIN = 5;
const gpio_num_t LCD_SDI_PIN = 6;
const gpio_num_t LCD_DC_RS_PIN = 7;
const gpio_num_t LCD_RST_PIN = 15;
const gpio_num_t LCD_CS_PIN = 16;

char system_state = 0;      // start in boot mode

// Debug setting
#define _TESTING 1

#define LCD_H_RES 480
#define LCD_V_RES 320
#define LINES_PER_DMA 40
#define PIXEL_CLK_FREQ 20 * 1000 * 1000
#define TIMER_DUTY_RESOLUTION 10
#define LED_PWM_SPEED_MODE LEDC_LOW_SPEED_MODE
#define LED_PWM_CHANNEL LEDC_CHANNEL_0


static spi_device_handle_t lcd_spi_handle;
// static esp_lcd_panel_handle_t lcd_panel_handle;
static spi_host_device_t lcd_host_device;

static void set_led_pwm(uint8_t p) {
    if (p > 100) {
        ESP_LOGE(TAG, "Duty cycle parameter exceeded 100%%");
        return;
    }

    // ESP_ERROR_CHECK(ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (p * ((1 << TIMER_DUTY_RESOLUTION) - 1)) / 100, 0));     // only use if switching to a fade service
    ESP_ERROR_CHECK(ledc_set_duty(LED_PWM_SPEED_MODE, LED_PWM_CHANNEL, (p * ((1 << TIMER_DUTY_RESOLUTION) - 1)) / 100));
    ESP_ERROR_CHECK(ledc_update_duty(LED_PWM_SPEED_MODE, LED_PWM_CHANNEL));
}

static void lcd_reset() {
    uint32_t current_ledc_duty = ledc_get_duty(LED_PWM_SPEED_MODE, LED_PWM_CHANNEL);
    if (current_ledc_duty != LEDC_ERR_DUTY) {       // turn off backlight before reset
        set_led_pwm(0);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Reset
    gpio_set_level(LCD_RST_PIN, 0);     // reset with enable-low
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(LCD_RST_PIN, 1);     // disable
    vTaskDelay(pdMS_TO_TICKS(20));

    if (current_ledc_duty != LEDC_ERR_DUTY) {       // turn backlight on after reset
        set_led_pwm(100);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static TaskHandle_t lcd_timeout_handle = NULL;
static bool IRAM_ATTR lcd_timeout_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    BaseType_t high_task_awoken = pdFALSE;
    vTaskNotifyGiveFromISR(lcd_timeout_handle, &high_task_awoken);

    return high_task_awoken == pdTRUE;
}

static void lcd_timeout_task() {
    if (_TESTING) ESP_LOGI(TAG, "Started lcd_timeout_task()");
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Timer has expired
        set_led_pwm(10);
        ESP_LOGI(TAG, "LCD auto-timeout triggered. LCD Brightness has been reduced.");
    }
    if (_TESTING) ESP_LOGW(TAG, "Ended lcd_timeout_task()");        // this shoudl theoretically never be called
}

static gptimer_handle_t timer_handle = NULL;
static void init_timeout() {
    if (_TESTING) ESP_LOGI(TAG, "In init_timeout()");
    gptimer_config_t timer_config = {};
    timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    timer_config.direction = GPTIMER_COUNT_UP;
    timer_config.resolution_hz = 1000000;     // 1 micro-second per tick

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer_handle));

    gptimer_alarm_config_t timer_alarm_config = {};
    if (_TESTING) {
        timer_alarm_config.alarm_count = 60000000;       // 60,000,000 ticks (1 minute)
    }
    else {
        timer_alarm_config.alarm_count = 600000000;       // 600,000,000 ticks (10 minutes)
    }
    timer_alarm_config.flags.auto_reload_on_alarm = 0;      // will be manually reset when user input is detected

    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer_handle, &timer_alarm_config));

    gptimer_event_callbacks_t timer_callback = {};
    timer_callback.on_alarm = lcd_timeout_callback;     // set the timeout callback to trigger on alarm

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer_handle, &timer_callback, NULL));
    ESP_ERROR_CHECK(gptimer_enable(timer_handle));
}

static void init_led_pwm() {
    if (_TESTING) ESP_LOGI(TAG, "In init_led_pwm()");

    // Create lcd timeout task for auto-dimming the display with priority=5
    xTaskCreate(lcd_timeout_task, "lcd_timeout_task", 2048, NULL, 5, &lcd_timeout_handle);

    init_timeout();

    ledc_timer_config_t timer_config = {};
    timer_config.speed_mode = LED_PWM_SPEED_MODE;
    timer_config.duty_resolution = TIMER_DUTY_RESOLUTION;   // can probably be reduced
    timer_config.timer_num = LEDC_TIMER_0;                  // must match channel_config.timer_sel below
    timer_config.freq_hz = 1000;
    timer_config.clk_cfg = LEDC_AUTO_CLK;

    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    ledc_channel_config_t channel_config = {};
    channel_config.gpio_num = LED_CS_PIN;
    channel_config.speed_mode = LED_PWM_SPEED_MODE;
    channel_config.channel = LED_PWM_CHANNEL;
    channel_config.intr_type = LEDC_INTR_DISABLE;       // enable if implementing PWM fade
    channel_config.timer_sel = LEDC_TIMER_0;            // must match timer_config.timer_num above
    channel_config.duty = 0;        // 0% for init

    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
}

void init_lcd() {
    if (!INCLUDE_LCD) {
        return;
    }
    if(_TESTING) ESP_LOGI(TAG, "In start_system_boot()");

    gpio_set_level(LCD_CS_PIN, 1);
    gpio_set_level(LCD_DC_RS_PIN, 1);

    init_led_pwm();
    set_led_pwm(0);

    lcd_host_device = SPI3_HOST;

    spi_bus_config_t lcd_bus_config = {};
    lcd_bus_config.mosi_io_num = LCD_SDI_PIN;
    lcd_bus_config.miso_io_num = LCD_SDO_PIN;
    lcd_bus_config.sclk_io_num = LCD_SCK_PIN;
    lcd_bus_config.max_transfer_sz = LCD_H_RES * LCD_V_RES * 2; // LCD_H_RES * LINES_PER_DMA * sizeof(uint16_t);
    lcd_bus_config.isr_cpu_id = ESP_INTR_CPU_AFFINITY_0;

    // Not used
    lcd_bus_config.quadwp_io_num = -1;
    lcd_bus_config.quadhd_io_num = -1;
    lcd_bus_config.data4_io_num = -1;
    lcd_bus_config.data5_io_num = -1;
    lcd_bus_config.data6_io_num = -1;
    lcd_bus_config.data7_io_num = -1;

    spi_dma_chan_t lcd_dma_chan = SPI_DMA_CH_AUTO;

    ESP_ERROR_CHECK(spi_bus_initialize(lcd_host_device, &lcd_bus_config, lcd_dma_chan));        // Use DMA
    vTaskDelay(50 / portTICK_PERIOD_MS);

    spi_device_interface_config_t lcd_interface_config = {};
    lcd_interface_config.clock_speed_hz = PIXEL_CLK_FREQ;     // 20 MHz
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

static void turn_on_display() {
    if (!INCLUDE_LCD) {
        return;
    }

    // Start timer for lcd auto-timeout
    ESP_ERROR_CHECK(gptimer_set_raw_count(timer_handle, 0));
    ESP_ERROR_CHECK(gptimer_start(timer_handle));
}

// Simple test function for writing to lcd registers
static void lcd_send_cmd(uint8_t cmd)
{
    gpio_set_level(LCD_DC_RS_PIN, 0);

    spi_transaction_t transaction = {};
    transaction.length = 8;
    transaction.tx_buffer = &cmd;

    spi_device_transmit(lcd_spi_handle, &transaction);
}

// Simple test function for writing to lcd registers
static void lcd_send_data(const uint8_t *data, int len) {
    if (len == 0) {
        return;
    }

    gpio_set_level(LCD_DC_RS_PIN, 1);
    spi_transaction_t transaction = {};
    transaction.length = len * 8;       // convert to bytes to bits
    transaction.tx_buffer = data;

    spi_device_transmit(lcd_spi_handle, &transaction);
}

// Simple test function for writing to lcd registers
static void lcd_send_data16(uint16_t color) {
    uint8_t d[2] = { color >> 8, color & 0xFF };
    lcd_send_data(d, 2);
}

// Configure ST7796 registers before streaming display data
static void lv_init_st7796() {
    lcd_reset();

    lcd_send_cmd(0x11);
    vTaskDelay(pdMS_TO_TICKS(100));

    lcd_send_cmd(0x3A);
    lcd_send_data((uint8_t[]){0x55}, 1);

    lcd_send_cmd(0x36);
    lcd_send_data((uint8_t[]){0x28}, 1);

    lcd_send_cmd(0x29);     // display on
}

// For non-lvgl use
static void init_st7796() {
    lcd_reset();

    lcd_send_cmd(0x11);
    vTaskDelay(pdMS_TO_TICKS(100));

    lcd_send_cmd(0x3A);
    lcd_send_data((uint8_t[]){0x55}, 1);

    lcd_send_cmd(0x36);
    lcd_send_data((uint8_t[]){0x28}, 1);

    lcd_send_cmd(0x29);     // display on
}

// For non-LVGL use
static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
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

// Indicates where incoming pixels should go and what X/Y region should be written to.
static void lv_lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
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

// Simple display test without LVGL
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

// static void old_lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *color_map) {
//     int32_t x1 = area->x1;
//     int32_t y1 = area->y1;
//     int32_t x2 = area->x2;
//     int32_t y2 = area->y2;

//     int32_t width  = x2 - x1 + 1;
//     int32_t height = y2 - y1 + 1;

//     uint16_t *pixels = (uint16_t *)color_map;

//     for (int32_t y = 0; y < height; y += LINES_PER_DMA) {
//         int32_t chunk_h = LINES_PER_DMA;
//         if (y + chunk_h > height) chunk_h = height - y;

//         // Set window for this chunk
//         lv_lcd_set_window(x1, y1 + y, x2, y1 + y + chunk_h - 1);

//         // Send pixels MSB-first (RGB)
//         for (int32_t i = 0; i < width * chunk_h; i++) {
//             uint16_t c = pixels[y * width + i];
//             uint8_t data[2] = { c >> 8, c & 0xFF }; // MSB first
//             lcd_send_data(data, 2);
//         }
//         // vTaskDelay(pdMS_TO_TICKS(5));       // reduces screen reload tear
//         vTaskDelay(0);
//     }

//     lv_display_flush_ready(disp);
// }

static void lcd_send_line_dma(const uint16_t *pixels, int width)
{
    gpio_set_level(LCD_DC_RS_PIN, 1);  // data mode

    spi_transaction_t t = {
        .tx_buffer = pixels,
        .length = width * 16,   // one line
    };

    ESP_ERROR_CHECK(spi_device_transmit(lcd_spi_handle, &t));
}

// Main LVGL callback for data streaming
static void lvgl_flush_cb(lv_display_t *disp,
                          const lv_area_t *area,
                          uint8_t *color_map)
{
    int32_t x1 = area->x1;
    int32_t y1 = area->y1;
    int32_t x2 = area->x2;
    int32_t y2 = area->y2;

    int32_t width  = x2 - x1 + 1;
    int32_t height = y2 - y1 + 1;

    uint16_t *pixels = (uint16_t *)color_map;

    for (int32_t y = 0; y < height; y += LINES_PER_DMA) {

        int32_t chunk_h = LINES_PER_DMA;
        if (y + chunk_h > height) {
            chunk_h = height - y;
        }

        // Set LCD window for this chunk
        lv_lcd_set_window(
            x1,
            y1 + y,
            x2,
            y1 + y + chunk_h - 1
        );

        // Send line-by-line using DMA
        for (int32_t line = 0; line < chunk_h; line++) {
            lcd_send_line_dma(
                pixels + (y + line) * width,
                width
            );
        }

        // Allows FreeRTOS to run higher-priority tasks if necessary
        taskYIELD();
    }

    lv_display_flush_ready(disp);
}

//  Initialize LVGL library and components
static void init_lvgl() {
    lv_init();
    lv_tick_set_cb(get_esp_tick);

    static lv_color_t buf1[LCD_H_RES * LINES_PER_DMA];
    static lv_color_t buf2[LCD_H_RES * LINES_PER_DMA];

    lv_display_t *disp = lv_display_create(LCD_H_RES, LCD_V_RES);

    // LVGL will render small rectangular regions using LV_DISPLAY_RENDER_MODE_PARTIAL
    // This calls lvgl_flush_cb() with an area (lv_area_t) and a pointer to pixels only for that area
    lv_display_set_buffers(disp, buf1, buf2, sizeof(buf1), LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(disp, lvgl_flush_cb);
}

/*
Begins by starting auto-timeout timer, configuring ST7796 registers for data streaming, and initializing LVGL services.
Turns off LCD backlight, creates black background, draws text, and animates a loading bar.
*/ 
static void show_boot_screen() {
    if (_TESTING) ESP_LOGI(TAG, "In show_boot_screen()");

    turn_on_display();      // starts auto-timeout timer
    lv_init_st7796();       // configure ST7796 registers for data streaming
    init_lvgl();

    // Turn off backlight
    set_led_pwm(0);     

    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    lv_obj_t *text = lv_label_create(lv_screen_active());
    lv_label_set_text(text, "Booting PulsePioneer...");
    lv_obj_set_style_text_color(text, lv_color_white(), 0);
    lv_obj_align(text, LV_ALIGN_CENTER, 0, -40);

    // Update screen
    lv_timer_handler();

    // Turn on backlight
    set_led_pwm(100);

    lv_obj_t *boot_bar = lv_bar_create(lv_screen_active());
    lv_obj_set_size(boot_bar, 240, 30);
    lv_obj_align(boot_bar, LV_ALIGN_CENTER, 0, 40);

    // Bar objects have a default range of 0 to 100
    // Update the loading bar every 100 milliseconds
    for (int i = 0; i < 100; i+=10) {
        lv_bar_set_value(boot_bar, i, LV_ANIM_OFF);
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // lv_obj_t *rect = lv_obj_create(lv_screen_active());
    // lv_obj_set_size(rect, 240, 30);
    // lv_obj_align(rect, LV_ALIGN_CENTER, 0, 40);
    // lv_obj_set_style_bg_color(rect, lv_color_white(), 0);
    // lv_obj_set_style_bg_opa(rect, LV_OPA_COVER, 0);
    // lv_obj_set_style_border_width(rect, 0, 0);

    // lv_timer_handler();

    // set_led_pwm(100); // turn on backlight

}

static void show_main_menu() {
    if (_TESTING) ESP_LOGI(TAG, "In show_main_menu()");

    set_led_pwm(0);

    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0xc3eefa), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    lv_obj_t *text = lv_label_create(lv_screen_active());
    lv_label_set_text(text, "Welcome to PulsePioneer");
    lv_obj_set_style_text_color(text, lv_color_black(), 0);
    lv_obj_align(text, LV_ALIGN_TOP_LEFT, 10, 0);

    // Create header line style
    static lv_style_t header_line_style;
    lv_style_init(&header_line_style);
    lv_style_set_line_width(&header_line_style, 5);

    // Create header line
    lv_point_precise_t header_line_points[] = {{0, 0}, {120, 0}};
    lv_obj_t *header_line = lv_line_create(lv_screen_active());
    lv_line_set_points(header_line, header_line_points, 2);
    lv_obj_add_style(header_line, &header_line_style, 0);
    // lv_obj_align(header_line, LV_ALIGN_TOP_LEFT, 10, 0);
    lv_obj_align_to(header_line, text, LV_ALIGN_OUT_LEFT_BOTTOM, 0, 10);

    lv_timer_handler();

    set_led_pwm(100);
}

typedef struct {
    lv_obj_t *chart;
    lv_chart_series_t *ch1;
    lv_chart_series_t *ch2;
    lv_chart_series_t *ch3;
} lv_waveform_t;

static lv_waveform_t *waveform = NULL;

static lv_waveform_t *create_waveform_plot(void) {
    static lv_waveform_t waveform = {};
    waveform.chart = NULL;
    waveform.ch1 = NULL;
    waveform.ch2 = NULL;
    waveform.ch3 = NULL;

    // Remove all child objects from the active screen
    lv_obj_clean(lv_screen_active());

    // Initialize chart object
    lv_obj_t *chart;
    chart = lv_chart_create(lv_screen_active());
    lv_obj_set_size(chart, 400, 300);       // currently takes up a subwindow of the display
    lv_obj_center(chart);
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
    
    lv_chart_series_t *ch1 = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_YELLOW), LV_CHART_AXIS_PRIMARY_Y);
    // lv_chart_series_t *ch2 = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_PRIMARY_Y);
    // lv_chart_series_t *ch3 = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_LIGHT_BLUE), LV_CHART_AXIS_PRIMARY_Y);

    lv_chart_refresh(chart);
    lv_timer_handler();     // update active screen

    waveform.chart = chart;
    waveform.ch1 = ch1;
    // waveform.ch2 = ch2;
    // waveform.ch3 = ch3;

    // Create pointer for waveform data
    lv_waveform_t *wave_ptr = &waveform;

    return wave_ptr;
}

static lv_waveform_t *update_waveform_plot(lv_waveform_t *waveform, int32_t *new_data, uint16_t new_data_size) {
    if (!waveform) {        // check for null
        ESP_LOGW(TAG, "Waveform pointer is null. Waveform plot was not updated.");
        return waveform;
    }
    lv_obj_t *chart = waveform->chart;

    uint16_t i;
    for (i = 0; i < new_data_size; i++) {
        lv_chart_set_next_value(chart, waveform->ch1, *(new_data + i));
    }

    lv_chart_refresh(chart);
    lv_timer_handler();     // update active screen

    waveform->chart = chart;        // update waveform struct

    return waveform;
}

static void show_waveform_plots() {
    if (_TESTING) ESP_LOGI(TAG, "In show_waveform_plots()");
    // Initialize waveform if NULL
    if (!waveform) {
        waveform = create_waveform_plot();
    }

    // Update each waveform plot with queued data until queue is empty
    ecg_sample_t *sample_buffer;
    while (xQueueReceive(ecg_sample_queue, &sample_buffer, 0) != pdFALSE)
    {
        waveform = update_waveform_plot(waveform, &(sample_buffer->ch1), sizeof(sample_buffer->ch1));
        waveform = update_waveform_plot(waveform, &(sample_buffer->ch2), sizeof(sample_buffer->ch2));
        waveform = update_waveform_plot(waveform, &(sample_buffer->ch3), sizeof(sample_buffer->ch3));

        // Render waveform
        lv_timer_handler();
        vTaskDelay(0);
    }

    ESP_LOGI(TAG, "ecg_sample_queue is empty. Returning from show_waveform_plots()...");
}

// ------------------------------
// Main LVGL task
// ------------------------------
void lvgl_task(void *pvParameters) {
    if (!INCLUDE_LCD) {
        return;
    }
    if (_TESTING) ESP_LOGI(TAG, "Started lvgl_task()");

    for( ;; ) {     // loop indefinitely while waiting for new data frames
        lv_timer_handler();     // update display
        vTaskDelay(pdMS_TO_TICKS(20));

        switch (system_state) {
            case 0:
                // Render boot screen
                show_boot_screen();
                vTaskDelay(pdMS_TO_TICKS(1000));
                show_main_menu();
                system_state = 1;
                break;
            case 2:
                // Render waveform plots
                show_waveform_plots();
                system_state = 1;
                break;
            default:
                // Wait for new data frames
                break;
        }
        // if (_TESTING) ESP_LOGI(TAG, "Suspending lvgl_task()...");
        // vTaskSuspend(NULL);
    }

    if (_TESTING) ESP_LOGW(TAG, "Ended lvgl_task()");       // this should theoretically never be called
}


// Show ADS1293 error message in a message box on the display
// TODO: Implement fucntionality
void show_ecg_error_message(const char *text) {
    if (!INCLUDE_LCD) {
        return;
    }

    lv_obj_t *error_box;
    error_box = lv_msgbox_create(lv_screen_active());
    error_box = lv_msgbox_add_text(error_box, text);

    lv_timer_handler();     // update active screen
}