#ifndef LCD_H
#define LCD_H
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

extern const gpio_num_t LED_CS_PIN;
extern const gpio_num_t LCD_SDO_PIN;
extern const gpio_num_t LCD_SCK_PIN;
extern const gpio_num_t LCD_SDI_PIN;
extern const gpio_num_t LCD_DC_RS_PIN;
extern const gpio_num_t LCD_RST_PIN;
extern const gpio_num_t LCD_CS_PIN;

void lvgl_timer_cb(void *arg);

void init_lcd(void);

void show_boot_screen_no_dma(void);

void show_boot_screen_lvgl(void);

void show_boot_screen(void);

void show_ecg_error_messsage(const char *text);

void run_display_test(void);

void lvgl_test_screen(void);

void lvgl_task(void *arg);

#endif // LCD_H