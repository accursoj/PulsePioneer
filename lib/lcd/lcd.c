#include "lcd.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "driver/gptimer.h"
// #include "esp_lcd_io_spi.h"
#include "esp_lcd_st7796.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include <string.h>
#include <lvgl.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_task_wdt.h"
#include "demos/lv_demos.h"

#include "ecg.h"
#include "waveform.h"

#include "esp_log.h"
static const char *TAG = "lcd.c";

const gpio_num_t LED_CS_PIN = 1;
const gpio_num_t LCD_SDO_PIN = 4;
const gpio_num_t LCD_SCK_PIN = 5;
const gpio_num_t LCD_SDI_PIN = 6;
const gpio_num_t LCD_DC_RS_PIN = 7;
const gpio_num_t LCD_RST_PIN = 15;
const gpio_num_t LCD_CS_PIN = 16;

#define LCD_H_RES 480
#define LCD_V_RES 320
#define LINES_PER_DMA 40
#define PIXEL_CLK_FREQ 20 * 1000 * 1000
#define TIMER_DUTY_RESOLUTION 10
#define LED_PWM_SPEED_MODE LEDC_LOW_SPEED_MODE
#define LED_PWM_CHANNEL LEDC_CHANNEL_0

#define SPI_HW_MAX_BYTES (32 * 1024) // A safe limit for DMA

static esp_lcd_panel_handle_t panel_handle = NULL;
static esp_lcd_panel_io_handle_t io_handle = NULL;

// static spi_device_handle_t lcd_spi_handle;
// static spi_host_device_t lcd_host_device;
static TaskHandle_t lcd_timeout_handle = NULL;
static gptimer_handle_t timer_handle = NULL;

char system_state = 0;      // start in boot mode

// Debug setting
#define _TESTING 1

/*
Sets the duty cycle value for the PWM signal used to control the brightness of the LCD backlight.
E.g. 0 --> backlight off; 100 --> max brightness
*/
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
    if (_TESTING) ESP_LOGW(TAG, "Ended lcd_timeout_task()");        // this should theoretically never be called
}

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
/*
A helper function for lvgl_flush_cb
*/
static inline uint16_t bswap16(uint16_t x) {
    return (x << 8) | (x >> 8);
}

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *color_map) {
    
    uint16_t *pixels = (uint16_t *)color_map;
    int32_t x1 = area->x1, y1 = area->y1;
    int32_t x2 = area->x2, y2 = area->y2;

    int32_t px_cnt = (x2 - x1 + 1) * (y2 - y1 + 1);
    for (int i = 0; i < px_cnt; i++) {
        pixels[i] = bswap16(pixels[i]);
    }

    // ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, x1, y1, x2, y2, pixels));
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, x1, y1, x2+1, y2+1, pixels));
    
    lv_display_flush_ready(disp);
}

static uint32_t lv_tick_cb(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void init_lvgl(void) {
    if (_TESTING) ESP_LOGI(TAG, "In init_lvgl()");
    lv_init();
    lv_tick_set_cb(lv_tick_cb);
    static lv_color_t buf1[LCD_H_RES * 40];
    static lv_color_t buf2[LCD_H_RES * 40];

    lv_display_t *disp = lv_display_create(LCD_H_RES, LCD_V_RES);
    lv_display_set_buffers(disp, buf1, buf2, sizeof(buf1), LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(disp, lvgl_flush_cb);
}

static void init_st7796(void) {
    if (_TESTING) ESP_LOGI(TAG, "In init_st7796()");
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = LCD_SDI_PIN;
    buscfg.miso_io_num = -1;
    buscfg.sclk_io_num = LCD_SCK_PIN;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = SPI_HW_MAX_BYTES;
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_spi_config_t io_config = {};
    io_config.dc_gpio_num = LCD_DC_RS_PIN;
    io_config.cs_gpio_num = LCD_CS_PIN;
    io_config.pclk_hz = PIXEL_CLK_FREQ * 2;
    io_config.spi_mode = 3;
    io_config.lcd_cmd_bits = 8;
    io_config.lcd_param_bits = 8;
    io_config.trans_queue_depth = 6; // size of pipelined DMA

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &io_handle));

    esp_lcd_panel_dev_config_t panel_cfg = {};
    panel_cfg.reset_gpio_num = LCD_RST_PIN;
    panel_cfg.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR;        //set to BGR with bswap16() in callback for correct colors
    panel_cfg.bits_per_pixel = 16;

    ESP_ERROR_CHECK(esp_lcd_new_panel_st7796(io_handle, &panel_cfg, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));

    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
}

// Abstract function to reset the LCD auto-dimming timer
// init_led_pwm() must be called before this function in order to initialize timer_handle
static void turn_on_display() {
    if (!INCLUDE_LCD) {
        return;
    }

    // Start timer for lcd auto-timeout
    ESP_ERROR_CHECK(gptimer_set_raw_count(timer_handle, 0));
    ESP_ERROR_CHECK(gptimer_start(timer_handle));
}

/*
Begins by starting auto-timeout timer, configuring ST7796 registers for data streaming, and initializing LVGL services.
Turns off LCD backlight, creates black background, draws text, and animates a loading bar.
*/ 
static lv_obj_t *show_boot_screen() {
    if (_TESTING) ESP_LOGI(TAG, "In show_boot_screen()"); 
    
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
    turn_on_display();      // starts auto-timeout timer
    set_led_pwm(100);

    lv_obj_t *boot_bar = lv_bar_create(lv_screen_active());
    lv_obj_set_size(boot_bar, 240, 30);
    lv_obj_align(boot_bar, LV_ALIGN_CENTER, 0, 40);

    // Bar objects have a default range of 0 to 100
    // Update the loading bar every 100 milliseconds
    for (int i = 0; i <= 100; i+=10) {
        lv_bar_set_value(boot_bar, i, LV_ANIM_OFF);
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    return scr;
}

/*
Removes all styles from the parent object scr.
Synchronously removes all child objects from the parent object scr.
Updates the display when finished.
*/
static void clean_screen(lv_obj_t *scr) {
    if (_TESTING) ESP_LOGI(TAG, "In clean_screen");

    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

    uint32_t child_count = lv_obj_get_child_count(scr);
    // if (_TESTING) ESP_LOGI(TAG, "%u child objects will be deleted...", child_count);

    for (int32_t i = 0; i < child_count; i++) {
        // Get youngest child object
        // The youngest child object is always at idx=0 until it is deleted. At that point, the next youngest assumes idx=0.
        lv_obj_t *child_obj = lv_obj_get_child(scr, 0);

        if (child_obj) {        // if not null
            // Delete the child object
            lv_obj_delete(child_obj);
        } 
    }

    // if (_TESTING) ESP_LOGI(TAG, "Number of child objects after deletion: %u", lv_obj_get_child_count(scr));

    // Update screen
    lv_timer_handler();
}

static void show_main_menu(lv_obj_t *scr) {
    if (_TESTING) ESP_LOGI(TAG, "In show_main_menu()");

    set_led_pwm(0);
    clean_screen(scr);
    
    lv_obj_set_style_bg_color(scr, lv_color_hex(0xd4f8fc), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    lv_obj_t *text = lv_label_create(scr);
    lv_label_set_text(text, "Welcome to PulsePioneer");
    lv_obj_set_style_text_color(text, lv_color_black(), 0);
    lv_obj_align(text, LV_ALIGN_TOP_LEFT, 10, 0);

    // Create header line style
    static lv_style_t header_line_style;
    lv_style_init(&header_line_style);
    lv_style_set_line_width(&header_line_style, 5);

    // Create header line
    lv_point_precise_t header_line_points[] = {{0, 0}, {120, 0}};
    lv_obj_t *header_line = lv_line_create(scr);
    lv_line_set_points(header_line, header_line_points, 2);
    lv_obj_add_style(header_line, &header_line_style, 0);
    // lv_obj_align(header_line, LV_ALIGN_TOP_LEFT, 10, 0);
    lv_obj_align_to(header_line, text, LV_ALIGN_OUT_LEFT_BOTTOM, 0, 10);

    lv_timer_handler();

    set_led_pwm(100);

    // Show for 5 seconds
    vTaskDelay(pdMS_TO_TICKS(1000));
}

/*
A test function that should fill the display in the following order: red, green, blue, red, green, blue.
The LVGL demo benchmark will then be run.
The demo functionality must be enabled in lv_conf.h.
*/
static void show_test_animation(lv_obj_t *scr) {
    if (_TESTING) ESP_LOGI(TAG, "In show_test_animation()");

    clean_screen(scr);
    set_led_pwm(100);

    lv_obj_set_style_bg_color(scr, lv_color_hex(0xFF0000), 0);      //red
    lv_timer_handler();

    vTaskDelay(pdMS_TO_TICKS(1000));
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x00FF00), 0);      //green
    lv_timer_handler();

    vTaskDelay(pdMS_TO_TICKS(1000));
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x0000FF), 0);      //blue
    lv_timer_handler();

    vTaskDelay(pdMS_TO_TICKS(1000));
    lv_obj_set_style_bg_color(scr, lv_color_make(255, 0, 0), 0);      //red
    lv_timer_handler();

    vTaskDelay(pdMS_TO_TICKS(1000));
    lv_obj_set_style_bg_color(scr, lv_color_make(0, 255, 0), 0);      //green
    lv_timer_handler();

    vTaskDelay(pdMS_TO_TICKS(1000));
    lv_obj_set_style_bg_color(scr, lv_color_make(0, 0, 255), 0);      //blue
    lv_timer_handler();    

    vTaskDelay(pdMS_TO_TICKS(1000));

    // Start benchmark
    lv_demo_benchmark();
}

void init_lcd() {
    if (_TESTING) ESP_LOGI(TAG, "In init_lcd()");

    init_led_pwm();
    init_st7796();
    init_lvgl();
}

// ------------------------------
// Main LVGL task
// ------------------------------
void lvgl_task(void *pvParameters) {
    if (!INCLUDE_LCD) {
        return;
    }
    if (_TESTING) ESP_LOGI(TAG, "Started lvgl_task()");

    lv_obj_t *scr = NULL;

    for( ;; ) {     // loop indefinitely while waiting for new data frames
        lv_timer_handler();     // update display
        vTaskDelay(pdMS_TO_TICKS(20));

        switch (system_state) {
            case 0:
                // Render boot screen
                scr = show_boot_screen();

                // Render main display
                show_main_menu(scr);
                system_state = 3;
                break;
            case 2:
                // Render waveform plots
                show_waveform_plots(scr);
                system_state = 3;
                break;
            case 3:
                test_waveform_plot();
                system_state = 1;
                break;
            case 5:
                show_test_animation(scr);
                system_state = 1;
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