#ifndef LCD_H
#define LCD_H
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include <lvgl.h>

// Enable LCD functionality
#define INCLUDE_LCD 1

extern const gpio_num_t LED_CS_PIN;

extern const gpio_num_t LCD_SDO_PIN;
extern const gpio_num_t LCD_SCK_PIN;
extern const gpio_num_t LCD_SDI_PIN;
extern const gpio_num_t LCD_DC_RS_PIN;
extern const gpio_num_t LCD_RST_PIN;
extern const gpio_num_t LCD_CS_PIN;

extern const gpio_num_t ENC_DT_PIN;
extern const gpio_num_t ENC_CLK_PIN;

extern char system_state;

typedef enum {
    GUI_IDLE = 0,
    GUI_BOOT = 1,
    GUI_MAIN = 2,
    GUI_ECG = 3,
    GUI_DEMO = 10
} system_state_t;

extern QueueHandle_t enc_queue;
extern QueueHandle_t forwarded_enc_queue;

void set_led_pwm(uint8_t p);

void init_lcd(void);

void lvgl_task(void *pvParameters);

void input_task(void *pvParameters);

void pass_gui_task_handle(TaskHandle_t *handle);

void reset_display_timeout(void);

void load_system_state(system_state_t state);

void gui_task(void *pvParameters);

lv_obj_t *get_ecg_scr_label(void);

// void boot_bar_completed_cb(void);

#endif // LCD_H