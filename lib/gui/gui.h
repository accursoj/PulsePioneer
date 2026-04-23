#ifdef __cplusplus
extern "C" {
#endif

#ifndef GUI_H
#define GUI_H



#include "../lcd/lcd.h"
#include "../waveform/waveform.h"

#include <../lvgl/lvgl.h>

extern lv_obj_t *scr_container;
extern lv_obj_t *main_scr;
extern lv_obj_t *ecg_scr;

extern TaskHandle_t gui_task_handle;
extern TaskHandle_t ecg_stream_task_handle;

lv_waveform_t *get_waveform_ptr(void);

void create_root_screen(void);

lv_obj_t *get_root_screen(void);

void update_sys_state_text(void *new_state_data);

void update_tool_text(const char *new_tool_text);

void update_data_bar_text(const char *new_classification_text, float new_confidence_level);

void create_sidebar(void);

void create_sidebar_item(const char* label_text, system_state_t gui_state);

void show_boot_screen(void);

void show_main_screen(void);

void show_ECG_screen(void);

void create_LVGL_screens(void);

#endif // gui.h

#ifdef __cplusplus
}
#endif