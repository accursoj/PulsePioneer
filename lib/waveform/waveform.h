#ifndef WAVEFORM_H
#define WAVEFORM_H
#include <stdint.h>
#include <lvgl.h>
#include "esp_log.h"

typedef struct {
    lv_obj_t *chart;
    lv_chart_series_t *ch1;
    lv_chart_series_t *ch2;
    lv_chart_series_t *ch3;
    lv_obj_t *y_scale;
} lv_waveform_t;

extern bool broke_update_loop_flag;

void create_chart_scale(lv_waveform_t *waveform);

void new_update_waveform_plot(lv_waveform_t *waveform, int32_t *new_data, uint16_t new_data_size);

lv_waveform_t *update_waveform_plot(lv_waveform_t *waveform, int32_t *new_data, uint16_t new_data_size);

bool test_waveform_plot(lv_waveform_t *waveform);

#endif //waveform.h