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
} lv_waveform_t;

extern bool broke_update_loop_flag;

lv_waveform_t *update_waveform_plot(lv_waveform_t *waveform, int32_t *new_data, uint16_t new_data_size);

bool test_waveform_plot(lv_waveform_t *waveform);

#endif //waveform.h