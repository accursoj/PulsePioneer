#ifndef LCD_H
#define LCD_H
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

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

void init_lcd(void);

void lvgl_task(void *pvParameters);

void input_task(void *pvParameters);

TaskHandle_t get_gui_task_handle(void);

void load_system_state(system_state_t state);

void gui_task(void *pvParameters);

#endif // LCD_H