/**
 * @file waveform.c
 * @brief LVGL waveform chart management and auto-calibration.
 *
 * This file contains functions to initialize, scale, offset, and 
 * update the LVGL chart objects used for rendering ECG data. It 
 * also handles the auto-calibration logic to dynamically adjust 
 * the chart's Y-axis based on incoming signal bounds.
 */
#include <stdlib.h>
#include <inttypes.h>

#include "esp_timer.h"
#include "esp_log.h"

#include "../waveform/waveform.h"
#include "../ecg/ecg.h"
#include "../lcd/lcd.h"
#include "../preprocessing/preprocessing.h"
#include "../gui/gui.h"

#define _TESTING 1

static const char *TAG = "waveform.c";

bool broke_update_loop_flag = false;

#define ECG_CALIBRATION_SAMPLES     300
#define ECG_AXIS_PADDING_PERCENT    0.05f

#define ECG_Y_TICK_COUNT 2   // Only min and max

static int32_t ch1_axis_min = 0;
static int32_t ch1_axis_max = 0;

static int32_t auto_calibration_time_start = 0;
static uint16_t auto_calibration_count = 0;

/**
 * @brief Creates and assigns an LVGL scale object to the passed waveform.
 * 
 * @details Creates a vertical scale with a number of ticks equal to the 
 *          value set in ECG_Y_TICK_COUNT. If ECG_Y_TICK_COUNT=2, only 
 *          the maximum and minimum ticks will be added.
 * 
 * @param waveform Pointer to the waveform structure containing the chart.
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
    lv_scale_set_label_show(waveform->y_scale, false);
}

static bool ecg_axis_locked = false;
static uint16_t ecg_sample_count = 0;
static int32_t ecg_min = INT32_MAX;
static int32_t ecg_max = INT32_MIN;

/**
 * @brief Determines the initial fixed Y-axis bounds based on early samples.
 * 
 * @details Accumulates min/max values over ECG_CALIBRATION_SAMPLES. Once 
 *          sufficient samples are collected, it locks the axis, applies 
 *          padding, and makes the chart visible.
 * 
 * @param waveform Pointer to the waveform structure.
 * @param val The latest ECG sample value.
 */
static void update_ecg_fixed_axis(lv_waveform_t *waveform, int32_t val) {
    if (ecg_axis_locked) {
        return;  // Axis already determined
    }
    if (!ecg_scr) {
        ESP_LOGE(TAG, "ecg_scr is null");
        return;
    }
    if (lv_obj_has_flag(ecg_scr, LV_OBJ_FLAG_HIDDEN)) {
        return;
    }

    lv_obj_t *chart = waveform->chart;

    if (ecg_sample_count == 0) {    // init
        // Hide the chart
        lv_obj_set_flag(chart, LV_OBJ_FLAG_HIDDEN, true);
        lv_obj_set_flag(waveform->y_scale, LV_OBJ_FLAG_HIDDEN, true);

        ESP_LOGI(TAG, "ECG plot calibration started.");
    }

    // Update min/max during calibration phase
    if (val < ecg_min) ecg_min = val;
    if (val > ecg_max) ecg_max = val;

    ecg_sample_count++;

    // Lock axis once enough samples have been collected
    if (ecg_sample_count >= ECG_CALIBRATION_SAMPLES) {
        // Hide the ecg screen calibration message
        lv_obj_add_flag(get_ecg_scr_label(), LV_OBJ_FLAG_HIDDEN);

        int32_t range = abs(ecg_max - ecg_min);
        if (range == 0) range = 1;
        
        int32_t padding = (int32_t)(range * ECG_AXIS_PADDING_PERCENT);
        
        ch1_axis_min = ecg_min - padding;
        ch1_axis_max = ecg_max + padding;
        
        // Set axis range
        lv_chart_set_axis_range(chart,
            LV_CHART_AXIS_PRIMARY_Y,
            ch1_axis_min,
            ch1_axis_max
        );
            
        ESP_LOGI(TAG, "ECG plot calibration finished.");
        ESP_LOGI(TAG, "ECG plot calibrated to %i through %i", ch1_axis_min, ch1_axis_max);

        // Set Y-axis ticks
        lv_scale_set_range(waveform->y_scale,
                                ch1_axis_min,
                                ch1_axis_max);

        ecg_axis_locked = true;

        auto_calibration_time_start = esp_timer_get_time();

        // Show chart
        lv_obj_set_flag(chart, LV_OBJ_FLAG_HIDDEN, false);
        lv_obj_set_flag(waveform->y_scale, LV_OBJ_FLAG_HIDDEN, false);
    }

}

/**
 * @brief Adjusts the Y-axis scale (zoom) of the chart.
 * 
 * @param waveform Pointer to the waveform structure.
 * @param enc_diff Rotary encoder difference (positive to zoom out, negative to zoom in).
 */
void update_chart_scale(lv_waveform_t *waveform, int32_t enc_diff) {
    const float scale = 0.5;

    const int32_t ch1_range = ch1_axis_max - ch1_axis_min;
    const int32_t ch1_amp = ch1_range / 2;
    const int32_t ch1_center = ch1_amp + ch1_axis_min;

    if (ch1_amp == 0) {
        ESP_LOGE(TAG, "ch1_amp evaluated to 0. ch1_axis_min == ch1_axis_max. Returning from update_chart_scale...");
        return;
    }

    if (enc_diff > 0) {
        ch1_axis_max = (int32_t) (ch1_center + (ch1_amp * (scale + 1)));
        ch1_axis_min = (int32_t) (ch1_center - (ch1_amp * (scale + 1)));
        if (_TESTING) ESP_LOGI(TAG, "Scaling chart up.");
    } else {
        ch1_axis_max = (int32_t) (ch1_center + (ch1_amp * scale));
        ch1_axis_min = (int32_t) (ch1_center - (ch1_amp * scale));
        if (_TESTING) ESP_LOGI(TAG, "Scaling chart down");
    }
    if (_TESTING) ESP_LOGI(TAG, "New chart scale: %"PRId32" - %"PRId32, ch1_axis_min, ch1_axis_max);
    
    lv_chart_set_axis_range(
        waveform->chart,
        LV_CHART_AXIS_PRIMARY_Y,
        ch1_axis_min,
        ch1_axis_max
    );

    return;     
}

/**
 * @brief Adjusts the Y-axis offset (pan) of the chart.
 * 
 * @param waveform Pointer to the waveform structure.
 * @param enc_diff Rotary encoder difference (positive to pan up, negative to pan down).
 */
void update_chart_offset(lv_waveform_t *waveform, int32_t enc_diff) {
    const int32_t ch1_range = ch1_axis_max - ch1_axis_min;
    if (ch1_range == 0) {
        ESP_LOGE(TAG, "ch1_range evaluated to 0. ch1_axis_min == ch1_axis_max. Returning from update_chart_scale...");
        return;
    }
    int32_t offset_val = (int32_t)(ch1_range / 10);

    if (enc_diff > 0) {
        ch1_axis_max += offset_val;
        ch1_axis_min += offset_val;
        if (_TESTING) ESP_LOGI(TAG, "Increasing chart offset");

    } else {
        ch1_axis_max -= offset_val;
        ch1_axis_min -= offset_val;
        if (_TESTING) ESP_LOGI(TAG, "Decreasing chart offset");
    }

    lv_chart_set_axis_range(
        waveform->chart,
        LV_CHART_AXIS_PRIMARY_Y,
        ch1_axis_min - ECG_AXIS_PADDING_PERCENT,
        ch1_axis_max + ECG_AXIS_PADDING_PERCENT
    );

    return; 
}

/**
 * @brief Checks if the initial ECG plot calibration has completed.
 * 
 * @return true If the axis is locked and calibrated.
 * @return false If still collecting initial calibration samples.
 */
bool is_plot_calibrated() {
    return ecg_axis_locked;
}

// static int32_t auto_calibration_count = 0;
static int32_t auto_calibration_max = INT32_MIN;
static int32_t auto_calibration_min = INT32_MAX;

/**
 * @brief Periodically auto-calibrates the Y-axis based on recent extrema.
 * 
 * @param waveform Pointer to the waveform structure.
 * @param val The current ECG sample value triggering the calibration reset.
 */
static void auto_calibrate(lv_waveform_t *waveform, int32_t val) {
    // Do not render the first calibration (prevents any transient data from skewing the calibration)
    if (auto_calibration_count > 0) {
        lv_chart_set_axis_range(
            waveform->chart,
            LV_CHART_AXIS_PRIMARY_Y,
            auto_calibration_min,
            auto_calibration_max
        );
    }


    ESP_LOGI(TAG, "Auto calibration set waveform range to (%"PRId32", %"PRId32")", auto_calibration_min, auto_calibration_max);

    auto_calibration_max = val;
    auto_calibration_min = val;

    auto_calibration_time_start = esp_timer_get_time();

    return;
}

/**
 * @brief Tracks the rolling minimum and maximum values for auto-calibration.
 * 
 * @param val The latest ECG sample value.
 */
static void update_auto_calibration_vals(int32_t val) {
    if (val > auto_calibration_max) {
        auto_calibration_max = val;
    }
    if (val < auto_calibration_min) {
        auto_calibration_min = val;
    }

    return;
}

/**
 * @brief Appends new ECG data to the waveform plot and triggers calibration.
 * 
 * @details Checks that the passed waveform pointer has been initialized. 
 *          Must be called from within the LVGL task context.
 * 
 * @param waveform Pointer to the waveform structure to update.
 * @param new_data Pointer to the array of new ECG samples.
 * @param new_data_size Number of samples in the new_data array.
 */
void update_waveform_plot(lv_waveform_t *waveform, int32_t *new_data, uint16_t new_data_size) {

    if (!waveform) {        // check for null
        ESP_LOGW(TAG, "Waveform pointer is null. Waveform plot was not updated.");
        return;
    }

    lv_obj_t *chart = waveform->chart;

    for (uint16_t i = 0; i < new_data_size; i++) {

        int32_t val = new_data[i];

        // Add the new data point to the chart
        lv_chart_set_next_value(chart, waveform->ch1, val);
        
        update_auto_calibration_vals(val);

        if (!ecg_axis_locked) {
            update_ecg_fixed_axis(waveform, val);
        } 
        else if (esp_timer_get_time() - auto_calibration_time_start > 10000000) {   // 10 seconds
            auto_calibrate(waveform, val);
            auto_calibration_count++;
        }

    }

    return;
}