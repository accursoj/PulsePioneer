/**
 * @file preprocessing.c
 * @brief ECG signal preprocessing pipeline for TensorFlow Lite Micro.
 *
 * This file contains digital signal processing (DSP) utilities such as
 * resampling, bandpass filtering (Biquad IIR), mean-centering, and
 * z-score normalization to prepare raw ECG data for ML inference.
 */
#define TF_LITE_STATIC_MEMORY

#include <stdint.h>
#include <inttypes.h>
#include <math.h>

#include "esp_log.h"

#include "preprocessing.h"


static const char *TAG = "preprocessing.c";

/**
 * ============================================================
 *                    UTILITY FUNCTIONS
 * ============================================================
 */

/**
 * @brief Estimates the sampling frequency from timestamped samples.
 *
 * @param samples Array of ECG samples with timestamps.
 * @param len Number of samples in the array.
 * @return float Estimated sampling frequency in Hz.
 */
static float compute_fs(ecg_sample_t *samples, int len) {
    if (len < 2) return TARGET_FS;

    uint32_t dt = samples[len - 1].timestamp_us - samples[0].timestamp_us;
    if (dt == 0) return TARGET_FS;

    float time_s = dt / 1e6f;
    return (float)len / time_s;
}

/**
 * @brief Linearly resamples a signal to a fixed number of points.
 *
 * @param in Pointer to the input signal array.
 * @param in_len Number of elements in the input array.
 * @param out Pointer to the output buffer for the resampled signal.
 * @param out_len Desired number of elements in the output array.
 */
static void resample_signal(float *in, int in_len, float *out, int out_len) {
    float scale = (float)(in_len - 1) / (out_len - 1);

    for (int i = 0; i < out_len; i++) {
        float idx = i * scale;
        int i0 = (int)idx;
        int i1 = i0 + 1;

        if (i1 >= in_len) i1 = in_len - 1;

        float t = idx - i0;
        out[i] = in[i0] + t * (in[i1] - in[i0]);
    }
}

#define NUM_STAGES 3
/**
 * @brief Structure representing a Direct Form 1 Biquad IIR filter stage.
 */
typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float x1, x2; // previous inputs
    float y1, y2; // previous outputs
} Biquad;

/**
 * @brief Array of biquad coefficients for each bandpass filter stage.
 * @note These coefficients should be computed from Python scipy.signal.butter(..., output='sos').
 */
static Biquad filters[NUM_STAGES] = {
    // replace with output from scipy.signal.butter
    { 0.05616238f, 0.11232477f, 0.05616238f, -0.76366721f, 0.41479668f, 0,0,0,0 },
    { 1.0f, 0.0f, -1.0f, -1.28842101f, 0.29735293f, 0,0,0,0 },
    { 1.0f, -2.0f, 1.0f, -1.98750036f, 0.98765905f, 0,0,0,0 }
};

/**
 * @brief Applies a single biquad filter stage to one sample.
 *
 * @param bq Pointer to the Biquad filter state and coefficients.
 * @param x Input sample value.
 * @param stage_idx Index of the filter stage.
 * @return float Filtered output sample.
 */
static float apply_biquad(Biquad* bq, float x, size_t stage_idx) {
    float y = bq->b0 * x + bq->b1 * bq->x1 + bq->b2 * bq->x2
                        - bq->a1 * bq->y1 - bq->a2 * bq->y2;

    // Shift states
    bq->x2 = bq->x1;
    bq->x1 = x;
    bq->y2 = bq->y1;
    bq->y1 = y;

    return y;
}
/**
 * @brief Applies a cascaded bandpass IIR filter to a single sample.
 *
 * @details Processes the sample through NUM_STAGES biquad filters.
 *          Maintains internal state across calls.
 * 
 * @param x Input sample.
 * @return float Filtered output sample.
 */
static float bandpass_filter(float x) {
    float y = x;
    for (size_t i = 0; i < NUM_STAGES; i++) {
        y = apply_biquad(&filters[i], y, i);
    }
    return y;
    ESP_LOGI(TAG, "Completed bandpass_filter");
}

/**
 * @brief Resets the internal state (delay lines) of all biquad filters.
 * 
 * @note To be called once when a new ECG segment is ready to be processed
 *       to prevent ringing from previous data segments.
 */
static void reset_bandpass_states(void) {
    for (size_t i = 0; i < NUM_STAGES; i++) {
        filters[i].x1 = filters[i].x2 = 0.0f;
        filters[i].y1 = filters[i].y2 = 0.0f;
    }
}

/**
 * @brief Normalizes a signal to zero mean and unit variance (z-score normalization).
 *
 * @param buf Input/output buffer containing the signal to normalize.
 * @param len Number of samples in the buffer.
 */
static void normalize(float *buf, int len) {
    float mean = 0.0f;
    float std = 0.0f;

    for (int i = 0; i < len; i++) mean += buf[i];
    mean /= len;

    for (int i = 0; i < len; i++) {
        float d = buf[i] - mean;
        std += d * d;
    }

    std = sqrtf(std / len);

    for (int i = 0; i < len; i++) {
        buf[i] = (buf[i] - mean) / (std + 1e-8f);
    }
}


/**
 * ============================================================
 *                    PREPROCESSING PIPELINE
 * ============================================================
 */

/**
 * @brief Computes the arithmetic mean of a signal array.
 * 
 * @param buf Pointer to the signal array.
 * @param len Number of samples in the array.
 * @return float The arithmetic mean.
 */
static float compute_mean(float *buf, int len){
    float sum = 0;
    for(int i=0;i<len;i++) sum += buf[i];
    return sum/len;
}

/**
 * @brief Computes the standard deviation of a signal array.
 * 
 * @param buf Pointer to the signal array.
 * @param len Number of samples in the array.
 * @return float The standard deviation.
 */
static float compute_std(float *buf, int len){
    float mean = compute_mean(buf,len);
    float s = 0;
    for(int i=0;i<len;i++){
        float d = buf[i]-mean;
        s += d*d;
    }
    return sqrtf(s/len);
}

/**
 * @brief Converts raw ECG samples into a model-ready input tensor.
 * 
 * @details The pipeline performs the following steps:
 *          1. Acquires samples from the raw input buffer.
 *          2. (Optional) Estimates sampling frequency.
 *          3. Resamples the data to a fixed length (MODEL_INPUT_SIZE).
 *          4. Mean-centers the data to remove DC offset.
 *          5. (Optional) Applies cascaded bandpass filtering.
 *          6. Normalizes the signal to zero mean and unit variance.
 *
 * @param model_input_buffer Pointer to the final preprocessed output array.
 * @param raw_samples Pointer to the raw incoming ECG samples.
 * @param filtered Pointer to an intermediate buffer used during processing.
 * @param num_received_samples Number of raw samples received.
 * @return true Upon successful preprocessing.
 * @return false Upon failure.
 */
bool create_input_buffer(float *model_input_buffer, int32_t *raw_samples, float *filtered, int num_received_samples) {
    ESP_LOGI(TAG, "In create_input_buffer().");
    
    // // Estimate sampling frequency (currently unused but kept for extensibility)
    // float fs = compute_fs(raw_samples, num_received_samples);

    // Print a preview of the raw input data
    ESP_LOGI(TAG, "Raw sample preview:");
    for (int i=0;i<15;i++){
        ESP_LOGI(TAG, "%d", raw_samples[i]);
    }
    ESP_LOGI(TAG, "Raw float sample preview:");
    for (int i=0;i<15;i++){
        ESP_LOGI(TAG, "%f", (float)raw_samples[i]);
    }

    // // Reset the bandpass states once for each group of samples
    // reset_bandpass_states();
    // ESP_LOGI(TAG, "Reset bandpass states successfully.");

    for (int i = 0; i < num_received_samples; i++) {
        filtered[i] = (float)raw_samples[i];
    }
    resample_signal(filtered, num_received_samples, model_input_buffer, MODEL_INPUT_SIZE);
    
    ESP_LOGI(TAG,"First 15 resampled samples:");
    for(int i=0;i<15;i++) ESP_LOGI(TAG,"%f", model_input_buffer[i]);
    ESP_LOGI(TAG,"Last 15 resampled samples:");
    for(int i=MODEL_INPUT_SIZE-1;i>MODEL_INPUT_SIZE-17;i--) ESP_LOGI(TAG,"%f", model_input_buffer[i]);

    // Mean-center the data
    // Dev note: This is critical to prevent ringing caused by initial application of the Biquad
    //  after the filter is reset.
    float baseline_mean = 0.0f;
    for (int i = 0; i < MODEL_INPUT_SIZE; i++) {
        baseline_mean += model_input_buffer[i];
    }
    baseline_mean /= MODEL_INPUT_SIZE;
    
    for (int i = 0; i < MODEL_INPUT_SIZE; i++) {
        model_input_buffer[i] -= baseline_mean;
    }
    ESP_LOGI(TAG,"First 15 baselined samples:");
    for(int i=0;i<15;i++) ESP_LOGI(TAG,"%f", model_input_buffer[i]);

    // // For production:
    // // Cast and filter each sample
    // // Dev note: This filtering currently seems to break the code.
    // //   It seemed to be a critical preprocessing component because it corresponds with the
    // //   bandpass filter applied to the training data. Even with mean-centering, the biquad
    // //   still creates undesired data swings.
    // for (int i = 0; i < MODEL_INPUT_SIZE; i++) {
    //     model_input_buffer[i] = bandpass_filter(model_input_buffer[i]);
    // }

    // // For testing:
    // // Cast and filter each hardcoded sample
    // for (int i = 0; i < RAW_BUFFER_SIZE; i++) {
    //     filtered[i] = bandpass_filter((float)raw_data[i]);
    // }

    ESP_LOGI(TAG, "Filtered inference data.");

    // Print a preview of the preprocessed data
    ESP_LOGI(TAG,"First 15 filtered samples:");
    for(int i=0;i<15;i++) ESP_LOGI(TAG,"%f", model_input_buffer[i]);
    ESP_LOGI(TAG,"Last 15 filtered samples:");
    for(int i=MODEL_INPUT_SIZE-1;i>MODEL_INPUT_SIZE-17;i--) ESP_LOGI(TAG,"%f", model_input_buffer[i]);

    // Normalize signal
    normalize(model_input_buffer, MODEL_INPUT_SIZE);

    ESP_LOGI(TAG,"First 15 normalized samples:");
    for(int i=0;i<15;i++) ESP_LOGI(TAG,"%f", model_input_buffer[i]);

    ESP_LOGI(TAG,"Model input mean/std: mean=%f std=%f", compute_mean(model_input_buffer, MODEL_INPUT_SIZE), compute_std(model_input_buffer, MODEL_INPUT_SIZE));

    // // Load the processed input data into the TFLM input tensor
    // for (int i = 0; i < MODEL_INPUT_SIZE; i++) {
    //     input->data.f[i] = model_input_buffer[i];
    // }
    ESP_LOGI(TAG, "Normalized inference data.");

    return true;    // success
}
