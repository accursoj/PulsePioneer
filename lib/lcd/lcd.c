/**
 * @file lcd.c
 * @brief LCD, LVGL, and hardware input management for PulsePioneer.
 *
 * This file initializes and manages the ST7796 display panel, LVGL UI 
 * framework, rotary encoder, and system power management (sleep/wake).
 * It defines the FreeRTOS tasks responsible for rendering the UI, 
 * processing hardware inputs, and controlling the LCD backlight.
 */
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

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_task_wdt.h"
#include "demos/lv_demos.h"
#include "encoder.h"
#include "esp_sleep.h"
#include "esp_heap_caps.h"

#include "../ecg/ecg.h"
#include "../waveform/waveform.h"
#include "../gui/gui.h"
#include "../tflm_wrapper/tflm_wrapper.h"
#include "../preprocessing/preprocessing.h"

#include "esp_log.h"
static const char *TAG = "lcd.c";

// const char* output_classes[3] = {"AFIB  ", "NORMAL", "VFIB  "};

const gpio_num_t LED_CS_PIN = 1;

const gpio_num_t POWER_BTN_PIN = 3;

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
#define LINES_PER_DMA 20
#define PIXEL_CLK_FREQ 40 * 1000 * 1000
#define TIMER_DUTY_RESOLUTION 10
#define LED_PWM_SPEED_MODE LEDC_LOW_SPEED_MODE
#define LED_PWM_CHANNEL LEDC_CHANNEL_0

#define SPI_HW_MAX_BYTES (32 * 1024) // Safe Direct Memory Addressing (DMA) limit

static esp_lcd_panel_handle_t panel_handle = NULL;
static esp_lcd_panel_io_handle_t io_handle = NULL;

static TaskHandle_t lcd_timeout_handle = NULL;
static gptimer_handle_t timer_handle = NULL;

SemaphoreHandle_t xLVGLSemaphore = NULL;

QueueHandle_t enc_queue = NULL;
QueueHandle_t forwarded_enc_queue = NULL;
rotary_encoder_t enc = {};

typedef enum {
    LVGL_CMD_SHOW_BOOT,
    LVGL_CMD_SHOW_MAIN,
    LVGL_CMD_SHOW_ECG
} lvgl_cmd_t;
static QueueHandle_t lvgl_cmd_queue = NULL;

char system_state;

// Debug setting
#define _TESTING 1

/**
 * @brief Sets the duty cycle value for the LCD backlight PWM signal.
 * 
 * @param p Duty cycle percentage (0-100). 0 is off, 100 is max brightness.
 */
void set_led_pwm(uint8_t p) {
    if (p > 100) {
        ESP_LOGE(TAG, "Duty cycle parameter exceeded 100%%");
        return;
    }

    // ESP_ERROR_CHECK(ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (p * ((1 << TIMER_DUTY_RESOLUTION) - 1)) / 100, 0));     // only use if switching to a fade service
    ESP_ERROR_CHECK(ledc_set_duty(LED_PWM_SPEED_MODE, LED_PWM_CHANNEL, (p * ((1 << TIMER_DUTY_RESOLUTION) - 1)) / 100));
    ESP_ERROR_CHECK(ledc_update_duty(LED_PWM_SPEED_MODE, LED_PWM_CHANNEL));
}

static bool display_dimmed = false;

/**
 * @brief Hardware timer callback for LCD auto-timeout.
 * 
 * @param timer Timer handle.
 * @param edata Event data.
 * @param user_data User context data.
 * @return true if a high priority task has been awoken, false otherwise.
 */
static bool IRAM_ATTR lcd_timeout_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    BaseType_t high_task_awoken = pdFALSE;
    vTaskNotifyGiveFromISR(lcd_timeout_handle, &high_task_awoken);

    return high_task_awoken == pdTRUE;
}

/**
 * @brief FreeRTOS task handling the LCD dimming upon timeout.
 */
static void lcd_timeout_task() {
    if (_TESTING) ESP_LOGI(TAG, "Started lcd_timeout_task()");
    for (;;) {   
        // Block until lcd_timeout_callback() is triggered
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Timer has expired
        if (!display_dimmed) {
            set_led_pwm(10);
            display_dimmed = true;
            ESP_LOGI(TAG, "LCD auto-timeout triggered. LCD Brightness has been reduced.");
        }
    }
    if (_TESTING) ESP_LOGW(TAG, "Ended lcd_timeout_task()");        // this should theoretically never be called
}

/**
 * @brief Initializes the hardware timer used for the LCD backlight timeout.
 */
static void init_timeout() {
    if (_TESTING) ESP_LOGI(TAG, "In init_timeout()");
    gptimer_config_t timer_config = {};
    timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    timer_config.direction = GPTIMER_COUNT_UP;
    timer_config.resolution_hz = 1000000;     // 1 micro-second per tick

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer_handle));

    gptimer_alarm_config_t timer_alarm_config = {};
    if (_TESTING) {
        timer_alarm_config.alarm_count = 300000000;       // 300,000,000 ticks (5 minutes)
    }
    else {
        timer_alarm_config.alarm_count = 600000000;       // 600,000,000 ticks (10 minutes)
    }
    // Reload the timer after it expires
    // This will not continuously trigger the LCD-timeout because it will set the display_dimmed flag
    // reset_display_timeout() will unset the display_dimmed flag when user input is detected
    timer_alarm_config.flags.auto_reload_on_alarm = 1;

    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer_handle, &timer_alarm_config));

    gptimer_event_callbacks_t timer_callback = {};
    timer_callback.on_alarm = lcd_timeout_callback;     // set the timeout callback to trigger on alarm

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer_handle, &timer_callback, NULL));
    ESP_ERROR_CHECK(gptimer_enable(timer_handle));
    ESP_ERROR_CHECK(gptimer_start(timer_handle));
}

/**
 * @brief Initializes the LEDC peripheral for LCD backlight PWM control.
 */
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

/**
 * @brief Helper function for lvgl_flush_cb to swap bytes.
 * 
 * @param x 16-bit integer to byte-swap.
 * @return uint16_t Byte-swapped 16-bit integer.
 */
static inline uint16_t bswap16(uint16_t x) {
    return (x << 8) | (x >> 8);
}

/**
 * @brief LVGL display flush callback.
 * 
 * @details Pushes a rendered image buffer to the display over SPI.
 * 
 * @param disp LVGL display object.
 * @param area Area of the display to update.
 * @param color_map Pointer to the pixel color data.
 */
static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *color_map) {
    
    uint16_t *pixels = (uint16_t *)color_map;
    int32_t x1 = area->x1, y1 = area->y1;
    int32_t x2 = area->x2, y2 = area->y2;

    int32_t px_cnt = (x2 - x1 + 1) * (y2 - y1 + 1);
    for (int i = 0; i < px_cnt; i++) {
        pixels[i] = bswap16(pixels[i]);
    }

    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, x1, y1, x2+1, y2+1, pixels));
    
    lv_display_flush_ready(disp);
}

/**
 * @brief LVGL tick callback providing system time.
 * 
 * @return uint32_t Current system time in milliseconds.
 */
static uint32_t lv_tick_cb(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/**
 * @brief Initializes the LVGL graphics library and display buffers.
 */
static void init_lvgl(void) {
    if (_TESTING) ESP_LOGI(TAG, "In init_lvgl()");
    lv_init();
    lv_tick_set_cb(lv_tick_cb);
    static lv_color_t buf1[LCD_H_RES * LINES_PER_DMA];
    static lv_color_t buf2[LCD_H_RES * LINES_PER_DMA];

    lv_display_t *disp = lv_display_create(LCD_H_RES, LCD_V_RES);
    lv_display_set_buffers(disp, buf1, buf2, sizeof(buf1), LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(disp, lvgl_flush_cb);

    // // Allocate DMA-capable memory dynamically instead of consuming static BSS
    // size_t buf_size = LCD_H_RES * LINES_PER_DMA * sizeof(lv_color_t);
    // lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(buf_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    // lv_color_t *buf2 = (lv_color_t *)heap_caps_malloc(buf_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

    // lv_display_t *disp = lv_display_create(LCD_H_RES, LCD_V_RES);
    // lv_display_set_buffers(disp, buf1, buf2, buf_size, LV_DISPLAY_RENDER_MODE_PARTIAL);
    // lv_display_set_flush_cb(disp, lvgl_flush_cb);
}

/**
 * @brief Initializes the SPI bus and ST7796 LCD panel driver.
 */
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

/**
 * @brief Initializes the rotary encoder hardware and event queues.
 */
static void init_encoder(void) {
    if (_TESTING) ESP_LOGI(TAG, "In init_encoder()");

    enc_queue = xQueueCreate(8, sizeof(rotary_encoder_event_t));
    if (!enc_queue) {
        ESP_LOGE(TAG, "enc_queue could not be created.");
    }
    forwarded_enc_queue = xQueueCreate(8, sizeof(rotary_encoder_event_t));
    if (!forwarded_enc_queue) {
        ESP_LOGE(TAG, "forwarded_enc_queue could not be created.");
    }

    ESP_ERROR_CHECK(rotary_encoder_init(enc_queue));

    enc.pin_a = ENC_CLK_PIN;
    enc.pin_b = ENC_DT_PIN;
    enc.pin_btn = ENC_SW_PIN;


    ESP_ERROR_CHECK(rotary_encoder_add(&enc));

    if (_TESTING) ESP_LOGI(TAG, "Rotary encoder was successfully initialized.");
}

/**
 * @brief Resets the LCD auto-dimming timer and restores brightness.
 * @note init_led_pwm() must be called before this function to initialize timer_handle.
 */
void reset_display_timeout() {
    // Reset timer for lcd auto-timeout
    ESP_ERROR_CHECK(gptimer_set_raw_count(timer_handle, 0));
    if (display_dimmed) {
        set_led_pwm(100);
        display_dimmed = false;
    }
}

/**
 * @brief Loads and transitions to a new system GUI state.
 * 
 * @param new_state The target system state (e.g., GUI_BOOT, GUI_MAIN, GUI_ECG).
 */
void load_system_state(system_state_t new_state) {
    ESP_LOGI(TAG, "In system state. Old state: %d. New state: %d", system_state, new_state);

    if (!gui_task_handle) {
        if (_TESTING) ESP_LOGW(TAG, "load_system_state() was called, but gui_task_handle is still undefined. Returning...");
        return;
    }

    if (system_state != GUI_BOOT && system_state != new_state) {
        system_state = new_state;

        if (main_scr && ecg_scr) {
            lv_obj_set_flag(main_scr, LV_OBJ_FLAG_HIDDEN, true);
            lv_obj_set_flag(ecg_scr, LV_OBJ_FLAG_HIDDEN, true);
        } else {
            if (_TESTING) ESP_LOGW(TAG, "main_scr or ecg_scr still NULL prior to load_system_state()");
        }

        if (_TESTING) ESP_LOGI(TAG, "Switched system state to %d", new_state);
        xTaskNotifyGive(gui_task_handle);   // notify the GUI task

    } else if (system_state == GUI_BOOT) {
        if (_TESTING) ESP_LOGI(TAG, "Processing initial state change.");
        system_state = new_state;
        xTaskNotifyGive(gui_task_handle);
    }

}

/**
 * @brief Orchestrates the complete initialization of the LCD subsystem.
 */
void init_lcd() {
    if (_TESTING) ESP_LOGI(TAG, "In init_lcd().");

    init_led_pwm();
    set_led_pwm(0);

    init_st7796();
    init_lvgl();

    if (_TESTING) ESP_LOGI(TAG, "init_lcd() was successfully completed.");
}

/**
 * @brief Fetches ECG samples from the queue and updates the waveform plot.
 */
static void plot_ecg_data(void) {
    ecg_sample_t sample_buffer;
    #define MAX_DRAIN_SAMPLES 25
    int32_t samples[MAX_DRAIN_SAMPLES];
    uint16_t count = 0;
    
    // Drain enough samples to easily catch up after heavy ML inference preemptions
    while (count < MAX_DRAIN_SAMPLES && xQueueReceive(ecg_sample_queue, &sample_buffer, 0) == pdTRUE) {
        samples[count++] = sample_buffer.ch1;
    }
    
    if (count > 0) {
        // ESP_LOGI(TAG, "Updating plot with %u samples", count);
        update_waveform_plot(get_waveform_ptr(), samples, count);
    }
}

/**
 * @brief Renders the ML prediction output to the GUI.
 * 
 * @param output_buffer Pointer to the array of prediction probabilities.
 */
static void render_prediction(float* output_buffer) {
    uint8_t classified_idx = get_prediction_idx(output_buffer, 3);   // OUTPUT_SIZE (see tflm_wrapper.cc)
    const char *output_class = output_classes[classified_idx];
    update_data_bar_text(output_class, output_buffer[classified_idx]);
}


/**
 * @brief Main LVGL FreeRTOS task handling GUI rendering and UI updates.
 * 
 * @param pvParameters Task parameters (unused).
 */
void lvgl_task(void *pvParameters) {
    if (_TESTING) ESP_LOGI(TAG, "Started lvgl_task()");

    // Create the LVGL mutex
    xLVGLSemaphore = xSemaphoreCreateMutex();
    if (xLVGLSemaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create LVGL mutex. Deleting lvgl_task()...");
        vTaskDelete(NULL);
    }

    create_LVGL_screens();

    // model_output_queue = get_model_output_queue();

    if (!lvgl_cmd_queue) {
        lvgl_cmd_queue = xQueueCreate(4, sizeof(lvgl_cmd_t));
    }
    if (!lvgl_cmd_queue) {
        ESP_LOGE(TAG, "lvgl_cmd_queue could not be created.");
    }

    lvgl_cmd_t cmd;

    // Allocate memory on the RAM heap for the output buffer
    float *output_buffer = (float *)heap_caps_malloc(
        3 * sizeof(float),
        MALLOC_CAP_DEFAULT
    );

    const TickType_t delay = pdMS_TO_TICKS(10);

    for (;;) {
        // // Lock the GUI. Wait indefinitely if another task is updating the UI.
        // // ---- Add all LVGL operations within the Semaphore ---- //
        // if (xSemaphoreTake(xLVGLSemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {

            if (xQueueReceive(lvgl_cmd_queue, &cmd, 0)) {
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
                    default:
                        break;
                }
            }
            
            if (get_waveform_ptr() && ecg_sample_queue) {
                plot_ecg_data();
            }

            if (model_output_queue) {
                // Process the most recent model output and render the prediction
                //  If empty, do nothing and continue immediately
                if (xQueueReceive(model_output_queue, output_buffer, 0)) {
                    render_prediction(output_buffer);
                }
            } else {
                ESP_LOGE(TAG, "LVGL tried to use model_output_queue, but it was NULL.");
                // model_output_queue = get_model_output_queue();
            }

            // Update the core LVGL handler
            lv_timer_handler();

            // // Unlock GUI for other tasks
            // xSemaphoreGive(xLVGLSemaphore);
        // } else {
        //     ESP_LOGE(TAG, "xLVGLSemaphore was not returned to lvgl_task");
        //     // Still update lvgl
        //     lv_timer_handler();
        // }

        vTaskDelay(delay);
    
    }
}

static uint8_t enc_prev_state = 0;
static bool enc_init = true;
/**
 * @brief Polls the rotary encoder GPIO pins to determine position changes.
 */
static void poll_gpio() {

    static const int8_t transition_table[16] = {
        0, -1, 1, 0,
        1, 0, 0, -1,
        -1, 0, 0, 1,
        0, 1, -1, 0
    };

    // Create 2-bit value of current state of input A and B
    uint8_t curr_state = gpio_get_level(ENC_CLK_PIN) |
                            (gpio_get_level(ENC_DT_PIN) << 1);

    if (enc_init) {
        enc_prev_state = curr_state;
        enc_init = false;
        return;
    }

    uint8_t index = (enc_prev_state << 2) | curr_state;
    enc_prev_state = curr_state;

    // Set inc_val to 1 or -1 based on direction change
    int8_t inc_val = transition_table[index];

    if (inc_val) {
        rotary_encoder_event_t enc_event = {};
        enc_event.type = RE_ET_CHANGED;
        enc_event.diff = inc_val;

        //  Pass the position detection to the forwarded_enc_queue so that it can be directly processed by enc_read() in gui.c    
        xQueueSend(forwarded_enc_queue, &enc_event, 0);

        if (_TESTING) ESP_LOGI(TAG, "New position diff: %" PRIi8, inc_val);
    }
}

/**
 * @brief Puts the system into deep sleep (standby mode).
 * 
 * @details If the system was just powered on, configures the appropriate 
 *          wake stubs before halting execution.
 * 
 * @param is_sys_on True if the system was actively on, false if initial boot.
 */
void start_deep_sleep(bool is_sys_on) {
    if (_TESTING) ESP_LOGI(TAG, "In start_deep_sleep()");
    if (esp_sleep_is_valid_wakeup_gpio(POWER_BTN_PIN))      // check if GPIO3 can enable ext0 wakeup
    {
        if (!is_sys_on) {
            esp_sleep_enable_ext0_wakeup(POWER_BTN_PIN, 0);
            ESP_LOGI(TAG, "Enabled EXT0 wakeup");
        } else {
            uint64_t gpio_power_btn_en_mask = 0x1 << POWER_BTN_PIN;
            esp_sleep_enable_ext1_wakeup(gpio_power_btn_en_mask, ESP_EXT1_WAKEUP_ANY_LOW);
            ESP_LOGI(TAG, "Enabled EXT1 wakeup");
        }
        ESP_LOGI(TAG, "ENTERING SYSTEM DEEP SLEEP...");
        esp_deep_sleep_start();
    }
}

/**
 * @brief Polls the power button state to trigger sleep mode if pressed.
 */
static void poll_power_button() {
    uint8_t is_pulled_low = !gpio_get_level(POWER_BTN_PIN);

    if (is_pulled_low) {      // active-low button
        start_deep_sleep(true);
    }
}

/**
 * @brief FreeRTOS task responsible for handling hardware inputs (encoder, buttons).
 * 
 * @param pvParameters Task parameters (unused).
 */
void input_task(void *pvParameters) {
    if (_TESTING) ESP_LOGI(TAG, "Started input_task()");

    const TickType_t delay = pdMS_TO_TICKS(20);
    rotary_encoder_event_t enc_event;
    int32_t enc_val = 0;

    // Set up rotary encoder
    init_encoder();

    // Check rotary encoder state
    for ( ;; ) {
        // Check GPIO states
        poll_gpio();

        poll_power_button();

        // Check event queue for new events to be processed
        if (xQueueReceive(enc_queue, &enc_event, 0)) {
            // forward event for gui processing
            // TODO: test this state check. DONE
            //      purpose is to send proprietary position detections to forwarded_enc_queue
            //      instead of enc_queue. Therefore, ignore the position detection from encoder.c.
            //      Currently still need encoder.c for button presses.
            if (enc_event.type != RE_ET_CHANGED) {
                xQueueSend(forwarded_enc_queue, &enc_event, 0);
            }
    
            // Print the type of event that occurred
            // Can be removed
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
        }

        vTaskDelay(delay);
    }
}

/**
 * @brief FreeRTOS task managing high-level GUI state transitions.
 * 
 * @param pvParameters Task parameters (unused).
 */
void gui_task(void *pvParameters) {
    if (_TESTING) ESP_LOGI(TAG, "Started gui_task()");

    system_state = GUI_BOOT;

    if (!lvgl_cmd_queue) {
        lvgl_cmd_queue = xQueueCreate(4, sizeof(lvgl_cmd_t));
    }
    if (!lvgl_cmd_queue) {
        ESP_LOGE(TAG, "lvgl_cmd_queue could not be created.");
    }

    // Always run boot screen first
    lvgl_cmd_t cmd = LVGL_CMD_SHOW_BOOT;
    xQueueSend(lvgl_cmd_queue, &cmd, portMAX_DELAY);

    for (;;) {
        // Sleep until notified that the state has changed
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);        // Binary sempahore with maximum wait
        if (_TESTING) ESP_LOGI(TAG, "gui_task() was notified of new state change.");
        switch (system_state) {
            case GUI_MAIN:
                cmd = LVGL_CMD_SHOW_MAIN;
                xQueueSend(lvgl_cmd_queue, &cmd, portMAX_DELAY);
                break;
            case GUI_ECG:
                cmd = LVGL_CMD_SHOW_ECG;
                xQueueSend(lvgl_cmd_queue, &cmd, portMAX_DELAY);
                break;
            case GUI_IDLE:
            default:
                break;
        }
    }
}
