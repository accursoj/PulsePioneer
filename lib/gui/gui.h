#ifndef GUI_H
#define GUI_H

#include <lvgl.h>
#include "encoder.h"
#include "esp_log.h"

#include "lcd.h"

extern lv_obj_t *scr_container;


void create_root_screen(void);

lv_obj_t *get_root_screen(void);

void create_scr_container(void);

void create_sidebar(void);

void create_sidebar_item(const char* label_text, system_state_t gui_state);

// void lv_indev_pass_enc_event(rotary_encoder_event_t *enc_event);

#endif // gui.h