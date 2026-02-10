#include "esp_timer.h"
#include "waveform.h"
#include "ecg.h"

#define _TESTING 1

static const char *TAG = "waveform.c";

bool broke_update_loop_flag = false;

// static lv_waveform_t *waveform = NULL;

// // extern QueueHandle_t ecg_sample_queue;
// /*
// Creates a LVGL waveform plot on the current active screen.
// Returns the LVGL waveform plot as a lv_waveform_t pointer.
// */
// static lv_waveform_t *create_waveform_plot(void) {
//     static lv_waveform_t waveform = {};
//     waveform.chart = NULL;
//     waveform.ch1 = NULL;
//     waveform.ch2 = NULL;
//     waveform.ch3 = NULL;

//     // Remove all child objects from the active screen
//     lv_obj_clean(lv_screen_active());

//     // Initialize chart object
//     lv_obj_t *chart;
//     chart = lv_chart_create(lv_screen_active());
//     lv_obj_set_size(chart, 400, 300);       // currently takes up a subwindow of the display
//     lv_obj_center(chart);
//     lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
    
//     lv_chart_series_t *ch1 = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_YELLOW), LV_CHART_AXIS_PRIMARY_Y);
//     // lv_chart_series_t *ch2 = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_PRIMARY_Y);
//     // lv_chart_series_t *ch3 = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_LIGHT_BLUE), LV_CHART_AXIS_PRIMARY_Y);

//     lv_chart_refresh(chart);
//     lv_timer_handler();     // update active screen

//     waveform.chart = chart;
//     waveform.ch1 = ch1;
//     // waveform.ch2 = ch2;
//     // waveform.ch3 = ch3;

//     // Create pointer for waveform data
//     lv_waveform_t *wave_ptr = &waveform;

//     return wave_ptr;
// }

/*
Checks that the passed waveform pointer has been initialized. Use create_waveform_plot() prior to calling this function.
Updates the LVGL timer handler and yields the task.
*/
static int32_t y_range_max = 100;
static int32_t y_range_min = 0;
static lv_waveform_t *update_waveform_plot(lv_waveform_t *waveform, int32_t *new_data, uint16_t new_data_size) {
    if (!waveform) {        // check for null
        ESP_LOGW(TAG, "Waveform pointer is null. Waveform plot was not updated.");
        return waveform;
    }
    lv_obj_t *chart = waveform->chart;
    
    uint16_t i;
    for (i = 0; i < new_data_size; i++) {
        lv_chart_set_next_value(chart, waveform->ch1, *(new_data + i));

        if (*new_data > y_range_max) {
            y_range_max = *new_data;
            lv_chart_set_axis_range(chart, LV_CHART_AXIS_PRIMARY_Y, y_range_min, y_range_max);
        } else if (*new_data < y_range_min) {
            y_range_min = *new_data;
            lv_chart_set_axis_range(chart, LV_CHART_AXIS_PRIMARY_Y, y_range_min, y_range_max);
        }
        lv_timer_handler();
    }

    waveform->chart = chart;        // update waveform struct
    
    return waveform;
}

/*
Should be called from lvgl_main_task() as a state of the GUI.
Creates an lv_waveform_t pointer.
Calls update_waveform_plot() to add queued data to the waveform pointer.
Updates the screen after the waveform has been updated by each element of queued data.
*/
// void show_waveform_plots(lv_obj_t *scr) {
//     if (_TESTING) ESP_LOGI(TAG, "In show_waveform_plots()");
//     // Initialize waveform if NULL
//     if (!waveform) {
//         waveform = create_waveform_plot();
//     }

//     // Update each waveform plot with queued data until queue is empty
//     ecg_sample_t *sample_buffer;
//     if (ecg_sample_queue) {
//         while (xQueueReceive(ecg_sample_queue, &sample_buffer, 0) != pdFALSE)
//         {
//             waveform = update_waveform_plot(waveform, &(sample_buffer->ch1), sizeof(sample_buffer->ch1));
//             waveform = update_waveform_plot(waveform, &(sample_buffer->ch2), sizeof(sample_buffer->ch2));
//             waveform = update_waveform_plot(waveform, &(sample_buffer->ch3), sizeof(sample_buffer->ch3));
//         }
//     }

//     ESP_LOGI(TAG, "ecg_sample_queue is empty or is NULL. Returning from show_waveform_plots()...");
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

        waveform = update_waveform_plot(waveform, &(sample_buffer.ch1), 1);
    }

    if (_TESTING) ESP_LOGI(TAG, "Updated waveform");

    return true;
}