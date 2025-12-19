#ifndef LCD_H
#define LCD_H
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

// ENable LCD functionality
#define INCLUDE_LCD 1

extern const gpio_num_t LED_CS_PIN;
extern const gpio_num_t LCD_SDO_PIN;
extern const gpio_num_t LCD_SCK_PIN;
extern const gpio_num_t LCD_SDI_PIN;
extern const gpio_num_t LCD_DC_RS_PIN;
extern const gpio_num_t LCD_RST_PIN;
extern const gpio_num_t LCD_CS_PIN;

extern char system_state;

void init_lcd(void);

void run_display_test(void);

void lvgl_task(void *pvParameters);

#endif // LCD_H