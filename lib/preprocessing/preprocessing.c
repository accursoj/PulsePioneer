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
 * @brief Estimate sampling frequency from timestamped samples.
 *
 * @param samples Array of ECG samples with timestamps
 * @param len Number of samples
 * @return Estimated sampling frequency (Hz)
 */
static float compute_fs(ecg_sample_t *samples, int len) {
    if (len < 2) return TARGET_FS;

    uint32_t dt = samples[len - 1].timestamp_us - samples[0].timestamp_us;
    if (dt == 0) return TARGET_FS;

    float time_s = dt / 1e6f;
    return (float)len / time_s;
}

/**
 * @brief Linearly resample a signal to a fixed number of points.
 *
 * @param in Input signal
 * @param in_len Input length
 * @param out Output buffer
 * @param out_len Desired output length
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
typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float x1, x2; // previous inputs
    float y1, y2; // previous outputs
} Biquad;

// Initialize biquad coefficients for each stage
// These coefficients should be computed from Python scipy.signal.butter(..., output='sos')
// Example SOS array for 0.5-40Hz bandpass at FS=250Hz (replace with exact values)
static Biquad filters[NUM_STAGES] = {
    // replace with output from scipy.signal.butter
    { 0.05616238f, 0.11232477f, 0.05616238f, -0.76366721f, 0.41479668f, 0,0,0,0 },
    { 1.0f, 0.0f, -1.0f, -1.28842101f, 0.29735293f, 0,0,0,0 },
    { 1.0f, -2.0f, 1.0f, -1.98750036f, 0.98765905f, 0,0,0,0 }
};
// Apply a single biquad to one sample
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
 * @brief Apply bandpass IIR filter to a single sample.
 *
 * Maintains internal state across calls.
 */
static float bandpass_filter(float x) {
    float y = x;
    for (size_t i = 0; i < NUM_STAGES; i++) {
        y = apply_biquad(&filters[i], y, i);
    }
    return y;
    ESP_LOGI(TAG, "Completed bandpass_filter");
}

// To be called once when a new ECG segment is ready to be processed
static void reset_bandpass_states(void) {
    for (size_t i = 0; i < NUM_STAGES; i++) {
        filters[i].x1 = filters[i].x2 = 0.0f;
        filters[i].y1 = filters[i].y2 = 0.0f;
    }
}

/**
 * @brief Normalize signal to zero mean and unit variance.
 *
 * @param buf Input/output buffer
 * @param len Number of samples
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

static float compute_mean(float *buf, int len){
    float sum = 0;
    for(int i=0;i<len;i++) sum += buf[i];
    return sum/len;
}

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
 * @brief Convert raw ECG samples into model-ready input.
 *  1. Acquire samples from queue
 *  2. Estimate sampling frequency (not implemented)
 *  3. Apply bandpass filtering
 *  4. Resample to fixed length
 *  5. Normalize signal
 *  6. Load the TFLM input tensor
 *
 * @return true upon success, false upon failure
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

    // 2. MEAN-CENTER (Crucial step to prevent IIR step response)
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

    // For production:
    // Cast and filter each sample
    for (int i = 0; i < MODEL_INPUT_SIZE; i++) {    // was num_received_smaples TEST:
    //     // Cast channel 1 from int32_t to float
    //     // Apply bandpass filter
    //     // filtered[i] = bandpass_filter((float)raw_samples[i].ch1);
    //     // filtered[i] = bandpass_filter(filtered[i]);

        // model_input_buffer[i] = bandpass_filter(model_input_buffer[i]);

    //     // // Cast channel 1 from int32_t to float
    //     // filtered[i] = (float)raw_samples[i].ch1;
    }

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

    // Normalize signal (zero mean, unit variance)
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
