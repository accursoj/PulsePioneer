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

void show_waveform_plots(lv_obj_t *);

void test_waveform_plot(void);

#endif //waveform.h