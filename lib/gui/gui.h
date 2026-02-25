#ifndef GUI_H
#define GUI_H

#include <lvgl.h>
#include "encoder.h"
#include "esp_log.h"

#include "lcd.h"
#include "waveform.h"

extern lv_obj_t *scr_container;
extern lv_obj_t *main_scr;
extern lv_obj_t *ecg_scr;

lv_waveform_t *get_waveform_ptr(void);
void pass_ecg_stream_task_handle(TaskHandle_t *handle);
TaskHandle_t get_ecg_stream_task_handle(void);


void create_root_screen(void);

lv_obj_t *get_root_screen(void);

void create_sidebar(void);

void create_sidebar_item(const char* label_text, system_state_t gui_state);

void show_boot_screen(void);

void show_main_screen(void);

void show_ECG_screen(void);

void create_LVGL_screens(void);

#endif // gui.h