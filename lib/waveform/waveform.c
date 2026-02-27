#include "esp_timer.h"
#include "waveform.h"
#include "ecg.h"
#include "lcd.h"
#include <stdlib.h>

#define _TESTING 1

static const char *TAG = "waveform.c";

bool broke_update_loop_flag = false;

#define ECG_CALIBRATION_SAMPLES     1000
#define ECG_AXIS_PADDING_PERCENT    0.05f

#define ECG_Y_TICK_COUNT 2   // Only min and max

/*
Create and assign an LVGL scale object to the passed waveform.
Creates a scale with a number of ticks equal to the value set in ECG_Y_TICK_COUNT.
If ECG_Y_TICK_COUNT=2, only max and min ticks will be added.
*/
void create_chart_scale(lv_waveform_t *waveform) {
    waveform->y_scale = lv_scale_create(lv_obj_get_parent(waveform->chart));

    lv_obj_set_size(waveform->y_scale,
                    40,
                    lv_obj_get_height(waveform->chart));

    lv_obj_align_to(waveform->y_scale,
                    waveform->chart,
                    LV_ALIGN_OUT_LEFT_MID,
                    -5,
                    0);

    // Vertical orientation (ticks on left side)
    lv_scale_set_mode(waveform->y_scale, LV_SCALE_MODE_VERTICAL_LEFT);
    // Only two ticks (min and max)
    lv_scale_set_total_tick_count(waveform->y_scale, ECG_Y_TICK_COUNT);
    // Every tick is major
    lv_scale_set_major_tick_every(waveform->y_scale, 1);
    // Show labels
    lv_scale_set_label_show(waveform->y_scale, true);
}

static bool ecg_axis_locked = false;
static uint16_t ecg_sample_count = 0;
static int32_t ecg_min = INT32_MAX;
static int32_t ecg_max = INT32_MIN;
static void update_ecg_fixed_axis(lv_waveform_t *waveform, int32_t val) {
    if (ecg_axis_locked) {
        return;  // Axis already determined
    }

    lv_obj_t *chart = waveform->chart;

    // Hide chart and scale while calibrating
    lv_obj_set_flag(chart, LV_OBJ_FLAG_HIDDEN, true);
    lv_obj_set_flag(waveform->y_scale, LV_OBJ_FLAG_HIDDEN, true);

    // Update min/max during calibration phase
    if (val < ecg_min) ecg_min = val;
    if (val > ecg_max) ecg_max = val;

    ecg_sample_count++;

    // Lock axis once enough samples have been collected
    if (ecg_sample_count >= ECG_CALIBRATION_SAMPLES) {
        lv_obj_add_flag(get_ecg_scr_label(), LV_OBJ_FLAG_HIDDEN);

        int32_t range = abs(ecg_max - ecg_min);
        if (range == 0) range = 1;
        
        int32_t padding = (int32_t)(range * ECG_AXIS_PADDING_PERCENT);
        
        int32_t axis_min = ecg_min - padding;
        int32_t axis_max = ecg_max + padding;
        
        // Set axis range
        lv_chart_set_axis_range(chart,
            LV_CHART_AXIS_PRIMARY_Y,
            axis_min,
            axis_max);
            
        ESP_LOGI(TAG, "ECG Plot Calibrated to %i through %i", axis_min, axis_max);

        // Set Y-axis ticks
        lv_scale_set_range(waveform->y_scale,
                                axis_min,
                                axis_max);

        // lv_timer_handler();

        ecg_axis_locked = true;

        // Show chart
        lv_obj_set_flag(chart, LV_OBJ_FLAG_HIDDEN, false);
        lv_obj_set_flag(waveform->y_scale, LV_OBJ_FLAG_HIDDEN, false);
    }

}

/*
Checks that the passed waveform pointer has been initialized.
Use create_waveform_plot() prior to calling this function.
Must be called from within the LVGL task.
*/
void update_waveform_plot(lv_waveform_t *waveform, int32_t *new_data, uint16_t new_data_size) {

    if (!waveform) {        // check for null
        ESP_LOGW(TAG, "Waveform pointer is null. Waveform plot was not updated.");
        return;
    }

    lv_obj_t *chart = waveform->chart;

    for (uint16_t i = 0; i < new_data_size; i++) {

        int32_t val = new_data[i];

        lv_chart_set_next_value(chart, waveform->ch1, val);

        update_ecg_fixed_axis(waveform, val);
    }

}


// /*
// Checks that the passed waveform pointer has been initialized. Use create_waveform_plot() prior to calling this function.
// Updates the LVGL timer handler. Must be called from within the LVGL task.
// */
// static int32_t init_y_max = 6080000;
// static int32_t init_y_min = 6010000;
// static int32_t y_range_max = 6080000;
// static int32_t y_range_min = 6010000;
// static int32_t counter = 0;

// lv_waveform_t *update_waveform_plot(lv_waveform_t *waveform, int32_t *new_data, uint16_t new_data_size) {
//     if (!waveform) {        // check for null
//         ESP_LOGW(TAG, "Waveform pointer is null. Waveform plot was not updated.");
//         return waveform;
//     }
//     lv_obj_t *chart = waveform->chart;
    
//     uint16_t i;
//     for (i = 0; i < new_data_size; i++) {
//         lv_chart_set_next_value(chart, waveform->ch1, *(new_data + i));

//         if (counter > 100) {
//             // counter = 0;
//             lv_chart_set_axis_range(chart, LV_CHART_AXIS_PRIMARY_Y, init_y_min, init_y_max);
//         }

//         if (*new_data > y_range_max) {
//             y_range_max = *new_data;
//             lv_chart_set_axis_range(chart, LV_CHART_AXIS_PRIMARY_Y, y_range_min, y_range_max);
//         } else if (*new_data < y_range_min) {
//             y_range_min = *new_data;
//             lv_chart_set_axis_range(chart, LV_CHART_AXIS_PRIMARY_Y, y_range_min, y_range_max);
//         }

//         lv_timer_handler();
//         counter++;
//     }

//     waveform->chart = chart;        // update waveform struct
    
//     return waveform;
// }

QueueHandle_t waveform_test_queue = NULL;
/*
Returns true when all test data points have been uploaded to the display.
Returns false if the queue is full or if a
*/
bool test_waveform_plot(lv_waveform_t *waveform) {
    if (_TESTING) ESP_LOGI(TAG, "In test_waveform_plot()");
    if (!waveform) {
        // waveform = create_waveform_plot();
        ESP_LOGE(TAG, "Waveform pointer is null. Waveform plot was not updated. Call create_ECG_screen() prior to test_waveform_plot().");
        return false;
    }
    const uint8_t queue_size = UINT8_MAX;
    if (_TESTING) ESP_LOGI(TAG, "Created waveform plot");

    if (!waveform_test_queue) {
        waveform_test_queue = xQueueCreate(queue_size, sizeof(ecg_sample_t));
    }
    if (_TESTING) ESP_LOGI(TAG, "Created queue");

    // Populate data queue with linear data
    for (uint8_t i = 0; i < queue_size; i++) {
        ecg_sample_t sample;
        sample.timestamp_us = esp_timer_get_time();
        sample.ch1 = (int32_t) i;

        if (xQueueSend(waveform_test_queue, &sample, 0) != pdTRUE) {
            if (_TESTING) ESP_LOGI(TAG, "Waveform Test Queue is full at timestamp %ld for sample value %ld", sample.timestamp_us, sample.ch1);
            break;  // exit for loop
        }
    }
    if (_TESTING) ESP_LOGI(TAG, "Populated queue");

    ecg_sample_t sample_buffer;
    while (xQueueReceive(waveform_test_queue, &sample_buffer, 0) != pdFALSE) {
        // Try changing this to pdFALSE so that GUI_MAIN is loaded when returning from this function (only if the update_waveform_flag solution does not work)
        if (ulTaskNotifyTake(pdTRUE, 0)) {      // Binary semaphore with wait time of 0
            // broke_update_loop_flag = true;
            if (_TESTING) ESP_LOGI(TAG, "Hit semaphore");
            return false;
        }
        // if (_TESTING) ESP_LOGI(TAG, "Plotting value %ld at time %ld", sample_buffer.ch1, sample_buffer.timestamp_us);

        update_waveform_plot(waveform, &(sample_buffer.ch1), 1);
    }

    // Clear chart data
    lv_chart_set_all_values(waveform->chart, waveform->ch1, LV_CHART_POINT_NONE);

    if (_TESTING) ESP_LOGI(TAG, "Updated waveform");

    return true;
}