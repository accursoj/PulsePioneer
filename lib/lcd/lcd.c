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
#include "encoder.h"

#include "ecg.h"
#include "waveform.h"
#include "gui.h"

#include "esp_log.h"
static const char *TAG = "lcd.c";

const gpio_num_t LED_CS_PIN = 1;

const gpio_num_t LCD_SDO_PIN = 4;
const gpio_num_t LCD_SCK_PIN = 5;
const gpio_num_t LCD_SDI_PIN = 6;
const gpio_num_t LCD_DC_RS_PIN = 7;
const gpio_num_t LCD_RST_PIN = 15;
const gpio_num_t LCD_CS_PIN = 16;

const gpio_num_t ENC_DT_PIN = 17;
const gpio_num_t ENC_CLK_PIN = 18;
const gpio_num_t ENC_SW_PIN = 8;

#define LCD_H_RES 480
#define LCD_V_RES 320
#define LINES_PER_DMA 40
#define PIXEL_CLK_FREQ 40 * 1000 * 1000
#define TIMER_DUTY_RESOLUTION 10
#define LED_PWM_SPEED_MODE LEDC_LOW_SPEED_MODE
#define LED_PWM_CHANNEL LEDC_CHANNEL_0

#define SPI_HW_MAX_BYTES (32 * 1024) // Safe Direct Memory Addressing (DMA) limit

static lv_obj_t *boot_scr = NULL;
static lv_obj_t *main_scr = NULL;
static lv_obj_t *ecg_scr = NULL;

static esp_lcd_panel_handle_t panel_handle = NULL;
static esp_lcd_panel_io_handle_t io_handle = NULL;

// static spi_device_handle_t lcd_spi_handle;
// static spi_host_device_t lcd_host_device;
static TaskHandle_t lcd_timeout_handle = NULL;
static gptimer_handle_t timer_handle = NULL;

TaskHandle_t gui_task_handle = NULL;

char system_state;

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
    ESP_ERROR_CHECK(gptimer_start(timer_handle));
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
    io_config.pclk_hz = PIXEL_CLK_FREQ;
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

QueueHandle_t enc_queue = NULL;
QueueHandle_t forwarded_enc_queue = NULL;
rotary_encoder_t enc = {};
static void init_encoder(void) {
    if (_TESTING) ESP_LOGI(TAG, "In init_encoder()");
    enc_queue = xQueueCreate(8, sizeof(rotary_encoder_event_t));
    forwarded_enc_queue = xQueueCreate(8, sizeof(rotary_encoder_event_t));

    ESP_ERROR_CHECK(rotary_encoder_init(enc_queue));

    enc.pin_a = ENC_CLK_PIN;
    enc.pin_b = ENC_DT_PIN;
    enc.pin_btn = ENC_SW_PIN;
    // enc = {
    //     .pin_a = ENC_CLK_PIN,
    //     .pin_b = ENC_DT_PIN,
    //     .pin_btn = (gpio_num_t)8//GPIO_NUM_MAX,        // eventually will be used with pin 8
    // };

    ESP_ERROR_CHECK(rotary_encoder_add(&enc));

    if (_TESTING) ESP_LOGI(TAG, "Rotary encoder was successfully initialized.");
}

// Abstract function to reset the LCD auto-dimming timer
// init_led_pwm() must be called before this function in order to initialize timer_handle
static void reset_display_timeout() {
    // Reset timer for lcd auto-timeout
    ESP_ERROR_CHECK(gptimer_set_raw_count(timer_handle, 0));
}

// void load_system_state(system_state_t state) {
//     if (system_state != state) {
//         system_state = state;
//         reset_display_timeout();
//         if (_TESTING) ESP_LOGI(TAG, "Switched system state to %d", state);
//     }
// }

void load_system_state(system_state_t state) {
    if (system_state != state) {
        system_state = state;
        reset_display_timeout();

        lv_obj_set_flag(main_scr, LV_OBJ_FLAG_HIDDEN, true);
        lv_obj_set_flag(ecg_scr, LV_OBJ_FLAG_HIDDEN, true);

        if (gui_task_handle) {
            if (_TESTING) ESP_LOGI(TAG, "Switched system state to %d", state);
            xTaskNotifyGive(gui_task_handle);   // notify the GUI task
        } else {
            if (_TESTING) ESP_LOGW(TAG, "State change received, but gui_task_handle is still undefined.");
        }

    }
}

static lv_obj_t *boot_bar = NULL;
static void create_boot_screen() {
    if (_TESTING) ESP_LOGI(TAG, "In create_boot_screen()");   
    
    boot_scr = lv_obj_create(NULL);

    lv_obj_set_style_bg_color(boot_scr, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(boot_scr, LV_OPA_COVER, 0);
    lv_obj_align(boot_scr, LV_ALIGN_LEFT_MID, 0, 0);
    
    lv_obj_t *text = lv_label_create(boot_scr);
    lv_label_set_text(text, "Booting PulsePioneer...");
    lv_obj_set_style_text_color(text, lv_color_white(), 0);
    lv_obj_align(text, LV_ALIGN_CENTER, 0, -40);
    
    boot_bar = lv_bar_create(boot_scr);
    lv_obj_set_size(boot_bar, 240, 30);
    lv_obj_align(boot_bar, LV_ALIGN_CENTER, 0, 40);
}



// ------------------------------
// Main Screen
// ------------------------------
static void create_main_screen(void) {
    if (_TESTING) ESP_LOGI(TAG, "In create_main_screen()");

    main_scr = lv_obj_create(scr_container);

    lv_obj_set_style_bg_color(main_scr, lv_color_hex(0xd4f8fc), 0);
    lv_obj_set_style_bg_opa(main_scr, LV_OPA_COVER, 0);
    lv_obj_set_size(main_scr, lv_pct(100), lv_pct(100));
    lv_obj_align(main_scr, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t *text = lv_label_create(main_scr);
    lv_label_set_text(text, "Welcome to PulsePioneer\n\n<Insert System Setup Instructions Here>");
    lv_obj_set_style_text_color(text, lv_color_black(), 0);
    lv_obj_set_style_text_align(text, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(text, LV_ALIGN_CENTER, 0, 0);

    lv_obj_add_flag(main_scr, LV_OBJ_FLAG_HIDDEN);

}

// ------------------------------
// ECG Screen
// ------------------------------
static lv_waveform_t *waveform_ptr = NULL;
static void add_waveform_plot() {
    if(_TESTING) ESP_LOGI(TAG, "In add_waveform_plot()");
    if (!waveform_ptr) {
        ESP_LOGE(TAG, "Waveform_ptr has not be initialized. Call create_ECG_screen prior to add_waveform_plot().");
        return;
    }

    // lv_waveform_t waveform = *waveform_ptr;
    if (!waveform_ptr->ch1) {
        waveform_ptr->ch1 = lv_chart_add_series(waveform_ptr->chart, lv_palette_main(LV_PALETTE_YELLOW), LV_CHART_AXIS_PRIMARY_Y);
    }
    if (!waveform_ptr->ch2) {
        waveform_ptr->ch2 = lv_chart_add_series(waveform_ptr->chart, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_PRIMARY_Y);
    }
    if (!waveform_ptr->ch3) {
        waveform_ptr->ch3 = lv_chart_add_series(waveform_ptr->chart, lv_palette_main(LV_PALETTE_LIGHT_BLUE), LV_CHART_AXIS_PRIMARY_Y);
    }

    // waveform_ptr = &waveform;
}

static void create_ECG_screen(uint8_t num_charts) {
    if (_TESTING) ESP_LOGI(TAG, "In create_ECG_screen()");
    static lv_waveform_t waveform = {};
    waveform.chart = NULL;
    waveform.ch1 = NULL;
    waveform.ch2 = NULL;
    waveform.ch3 = NULL;

    ecg_scr = lv_obj_create(scr_container);
    lv_obj_set_size(ecg_scr, lv_pct(100), lv_pct(100));
    lv_obj_align(ecg_scr, LV_ALIGN_CENTER, 0, 0);
    lv_obj_remove_flag(ecg_scr, LV_OBJ_FLAG_SCROLLABLE);

    // Initialize chart object
    waveform.chart = lv_chart_create(ecg_scr);

    lv_obj_set_size(waveform.chart, lv_pct(100), lv_pct(100));       // currently takes up a subwindow of the display
    lv_obj_center(waveform.chart);
    lv_chart_set_type(waveform.chart, LV_CHART_TYPE_LINE);
    
    waveform_ptr = &waveform;

    if (num_charts > 0 && num_charts < 4) {
        while (num_charts-- != 0) {
            add_waveform_plot();
        }
    } else {
        if (_TESTING) ESP_LOGE(TAG, "Parameter num_charts is out of bounds.");
        return;
    }

    lv_obj_add_flag(ecg_scr, LV_OBJ_FLAG_HIDDEN);
}

// ------------------------------
// Boot Screen
// ------------------------------
/*
Callback function used to update the GUI system state to show the main screen once the boot bar animation is completed.
*/
static void boot_bar_completed_cb(void) {
    create_scr_container();     // make container for GUI subscreens

    create_main_screen();       // initialize LVGL objects and hide
    create_ECG_screen(1);       // initialize LVGL items and hide
    create_sidebar();
    load_system_state(GUI_MAIN);
}

/*
Increments the value of the boot bar and call boot_bar_completed_cb when done.
*/
static void start_boot_bar_animation() {
    if (_TESTING) ESP_LOGI(TAG, "In show_boot_bar_animation()");

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, boot_bar);
    lv_anim_set_values(&a, 0, 100);
    lv_anim_set_time(&a, 1000);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_bar_set_value);
    lv_anim_set_completed_cb(&a, (lv_anim_completed_cb_t)boot_bar_completed_cb);
    lv_anim_start(&a);
}

/*
Turns on LCD backlight, loads the boot screen object, and starts the boot bar animation.
*/
static void show_boot_screen() {
    if (_TESTING) ESP_LOGI(TAG, "In show_boot_screen()");   
    
    create_boot_screen();
    set_led_pwm(100);

    lv_screen_load(boot_scr);

    start_boot_bar_animation();
}

static void show_main_screen(void) {
    if (_TESTING) ESP_LOGI(TAG, "In show_main_menu()");

    // if (!main_scr) {
    //     create_main_screen();
    // }

    lv_obj_set_flag(main_scr, LV_OBJ_FLAG_HIDDEN, false);
}

static void show_ECG_screen(void) {
    if (_TESTING) ESP_LOGI(TAG, "In show_ECG_screen()");
    // if (!ecg_scr) {
    //     uint8_t num_ECG_channels = 1;
    //     create_ECG_screen(num_ECG_channels);
    // }

    lv_obj_set_flag(ecg_scr, LV_OBJ_FLAG_HIDDEN, false);
}

// ------------------------------
// Test Screen
// ------------------------------
/*
A test function that should fill the display in the following order: red, green, blue, red, green, blue.
The LVGL demo benchmark will then be run.
The demo functionality must be enabled in lv_conf.h.
*/
static void show_test_animation(lv_obj_t *scr) {
    if (_TESTING) ESP_LOGI(TAG, "In show_test_animation()");

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
    set_led_pwm(0);

    init_st7796();
    init_lvgl();
}

// ------------------------------
// Main LVGL task
// ------------------------------
static QueueHandle_t lvgl_cmd_queue = NULL;
typedef enum {
    LVGL_CMD_SHOW_BOOT,
    LVGL_CMD_SHOW_MAIN,
    LVGL_CMD_SHOW_ECG,
    LVGL_CMD_RUN_WAVEFORM_TEST
} lvgl_cmd_t;

void lvgl_task(void *pvParameters) {
    if (_TESTING) ESP_LOGI(TAG, "Started lvgl_task()");

    if (!lvgl_cmd_queue) {
        lvgl_cmd_queue = xQueueCreate(4, sizeof(lvgl_cmd_t));
    }
    lvgl_cmd_t cmd;

    for (;;) {
        if (xQueueReceive(lvgl_cmd_queue, &cmd, pdMS_TO_TICKS(5))) {
            switch (cmd) {
                case LVGL_CMD_SHOW_BOOT:
                    show_boot_screen();
                    break;
                case LVGL_CMD_SHOW_MAIN:
                    show_main_screen();
                    break;
                case LVGL_CMD_SHOW_ECG:
                    show_ECG_screen();
                    break;
                case LVGL_CMD_RUN_WAVEFORM_TEST:
                    test_waveform_plot(waveform_ptr);
                    break;
                default:
                    break;
            }
        }

        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void input_task(void *pvParameters) {
    if (_TESTING) ESP_LOGI(TAG, "Started input_task()");

    const TickType_t delay = pdMS_TO_TICKS(100);
    rotary_encoder_event_t enc_event;
    int32_t enc_val = 0;

    // Set up rotary encoder
    init_encoder();

    // Check rotary encoder state
    for ( ;; ) {
        // Check event queue for new events to be processed
        // Delay indefinitely if queue is empty
        xQueueReceive(enc_queue, &enc_event, portMAX_DELAY);
        // forward event for gui processing
        xQueueSend(forwarded_enc_queue, &enc_event, 0);

        // Print the type of event that occurred
        switch (enc_event.type) {
            case RE_ET_BTN_PRESSED:
                if (_TESTING) ESP_LOGI(TAG, "Button pressed");
                break;
            case RE_ET_BTN_RELEASED:
                if (_TESTING) ESP_LOGI(TAG, "Button released");
                break;
            case RE_ET_BTN_CLICKED:
                if (_TESTING) ESP_LOGI(TAG, "Button clicked");
                break;
            case RE_ET_BTN_LONG_PRESSED:
                if (_TESTING) ESP_LOGI(TAG, "Button long-pressed");
                break;
            case RE_ET_CHANGED:
                enc_val += enc_event.diff;
                if (_TESTING) ESP_LOGI(TAG, "Value = %" PRIi32, enc_val);
                break;
            default:
                break;
        }

        vTaskDelay(delay);
    }
}

void gui_task(void *pvParameters) {
    if (_TESTING) ESP_LOGI(TAG, "Started gui_task()");

    gui_task_handle = get_gui_task_handle();

    system_state = GUI_BOOT;

    if (!lvgl_cmd_queue) {
        lvgl_cmd_queue = xQueueCreate(4, sizeof(lvgl_cmd_t));
    }

    // Always run boot screen first
    lvgl_cmd_t cmd = LVGL_CMD_SHOW_BOOT;
    xQueueSend(lvgl_cmd_queue, &cmd, portMAX_DELAY);

    for (;;) {
        // Sleep until notified that the state has changed
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);        // Binary sempahore with maximum wait time

        switch (system_state) {
            case GUI_MAIN:
                cmd = LVGL_CMD_SHOW_MAIN;
                xQueueSend(lvgl_cmd_queue, &cmd, portMAX_DELAY);
                break;
            case GUI_ECG:
                cmd = LVGL_CMD_SHOW_ECG;
                xQueueSend(lvgl_cmd_queue, &cmd, portMAX_DELAY);
                break;
            case GUI_DEMO:      // not implemented but fully functional
                cmd = LVGL_CMD_RUN_WAVEFORM_TEST;
                xQueueSend(lvgl_cmd_queue, &cmd, portMAX_DELAY);
            case GUI_IDLE:
            default:
                break;
        }
    }
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