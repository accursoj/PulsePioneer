#ifndef LCD_H
#define LCD_H
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

extern const LED_CS_PIN = 1;
extern const LCD_SDO_PIN = 4;
extern const LCD_SCK_PIN = 5;
extern const LCD_SDI_PIN = 6;
extern const LCD_DC_RS_PIN = 7;
extern const LCD_RST_PIN = 15;
extern const LCD_CS_PIN = 16;

void init_lcd(void);

void show_boot_screen_no_dma(void);

void show_boot_screen(void);

#endif // LCD_H