// TF_LITE_STATIC_MEMORY must be defined prior to any tensorflow library inclusions
// This flag must be defined in order to align with the configuration of structs
//  within the tesnorflow library. Otherwise, input->data.f will return null.
#define TF_LITE_STATIC_MEMORY

#include <inttypes.h>

#include "tflm_wrapper.h"

#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "esp_heap_caps.h"
#include "esp_psram.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "esp_log.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/queue.h"

#include "../ecg/ecg.h"
#include "../gui/gui.h"
#include "../preprocessing/preprocessing.h"
#include "models.h"


static const tflite::Model* model = nullptr;
static tflite::MicroInterpreter* interpreter = nullptr;
static TfLiteTensor* input = nullptr;
static TfLiteTensor* output = nullptr;
static uint8_t *model_psram = nullptr;
static uint8_t *tensor_arena_psram = nullptr;

static constexpr int kTensorArenaSize = 512 * 1024;    // minimum 350kB

static esp_task_wdt_config_t wdt_config = {};

QueueHandle_t model_output_queue = NULL;

const char *output_classes[OUTPUT_SIZE] = {"AFIB  ", "NORMAL", "VFIB  "};   // extra spaces keep padding on the data bar consistent

static const char *TAG = "tflm_wrapper.cc";



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

    // // 2. MEAN-CENTER (Crucial step to prevent IIR step response)
    // float baseline_mean = 0.0f;
    // for (int i = 0; i < MODEL_INPUT_SIZE; i++) {
    //     baseline_mean += model_input_buffer[i];
    // }
    // baseline_mean /= MODEL_INPUT_SIZE;
    
    // for (int i = 0; i < MODEL_INPUT_SIZE; i++) {
    //     model_input_buffer[i] -= baseline_mean;
    // }
    // ESP_LOGI(TAG,"First 15 baselined samples:");
    // for(int i=0;i<15;i++) ESP_LOGI(TAG,"%f", model_input_buffer[i]);

    // For production:
    // Cast and filter each sample
    for (int i = 0; i < MODEL_INPUT_SIZE; i++) {    // was num_received_smaples TEST:
    //     // Cast channel 1 from int32_t to float
    //     // Apply bandpass filter
    //     // filtered[i] = bandpass_filter((float)raw_samples[i].ch1);
    //     // filtered[i] = bandpass_filter(filtered[i]);

        model_input_buffer[i] = bandpass_filter(model_input_buffer[i]);

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

/**
 * ============================================================
 *                    TFLM INIT
 * ============================================================
 */

int tflm_init(void) {
    ESP_LOGI(TAG, "Starting tflm_init...");

    // Allocate Model in PSRAM
    model_psram = (uint8_t *) heap_caps_aligned_alloc(
        16,
        model_afib_vfib_25_tflite_len,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
    );
    if (!model_psram) {
        ESP_LOGE(TAG, "model_psram could not be allocated.");
        return 0;
    }
    memcpy(model_psram, model_afib_vfib_25_tflite, model_afib_vfib_25_tflite_len);

    // Point TFLM to the trained model
    model = tflite::GetModel(model_psram);

    if (model->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "model->version does not match TFLITE_SCHEMA_VERSION.");
        return 0;
    } 

    // Add the necessary layers to the resolver
    // The equivalent of every type of layer used to construct the original model in Python must be added to
    //  this local TFLM resolver
    static tflite::MicroMutableOpResolver<10> resolver;
    resolver.AddAdd();
    resolver.AddConv2D();
    resolver.AddExpandDims();
    resolver.AddFullyConnected();
    resolver.AddMaxPool2D();
    resolver.AddMean();
    resolver.AddMul();
    resolver.AddReshape();
    resolver.AddSoftmax();

    // Allocate Arena in PSRAM
    tensor_arena_psram = (uint8_t*)heap_caps_aligned_alloc(
        16,
        kTensorArenaSize,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
    );
    if (!tensor_arena_psram) {
        ESP_LOGE(TAG, "tensor_arena_psram could not be allocated");
        return 0;
    }

    // Use a persistent static object for the interpreter
    static tflite::MicroInterpreter static_interpreter(
        model,
        resolver,
        tensor_arena_psram,
        kTensorArenaSize
    );
    interpreter = &static_interpreter;

    if (interpreter->AllocateTensors() != kTfLiteOk) {
        ESP_LOGE(TAG, "AllocateTensors() failed!");
        return 0;
    }

    input = interpreter->input(0);
    output = interpreter->output(0);

    if (!input || !input->data.f) {
        ESP_LOGE(TAG, "input->data.f is null.");
        return 0;
    }

    ESP_LOGI(TAG, "Input pointer address: %p", input->data.f);

    // Load model output queue to be used in gui.c
    // Create output queue if it does not already exist
    if (!model_output_queue) {
        model_output_queue = xQueueCreate(5, OUTPUT_SIZE * sizeof(float));
    }
    if (!model_output_queue) {
        ESP_LOGE(TAG, "Failed to create model_output_queue.");
        return 0;
    }

    ESP_LOGI(TAG, "Returning from tflm_init()");
    
    return 1;
}

/**
 * @brief Create a custom task watchdog timer.
 * Allows the task watchdog timer to be reconfigured and extended so that reasonable timeouts from
 *  resource-intensive tasks do not cause system panic (and crashes).
 */
static bool set_wdt() {
    wdt_config.timeout_ms = 120000;     // 2 minutes
    wdt_config.idle_core_mask = (1 << portNUM_PROCESSORS) - 1;
    wdt_config.trigger_panic = true;

    ESP_ERROR_CHECK(esp_task_wdt_init(&wdt_config));
    ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&wdt_config));
    return true;
}

uint8_t get_prediction_idx(float *output_buffer, uint8_t output_buffer_len) {
    uint8_t max_val_idx = 0;

    // Find the maximum value
    for (uint8_t i = 1; i < output_buffer_len; i++) {
        if (output_buffer[i] > output_buffer[max_val_idx]) {
            max_val_idx = i;
        }
    }
    return max_val_idx;
}

void inference_task(void *pvParameters) {
    // Check if the custom TWDT has been initialized
    if (!set_wdt()) {
        ESP_LOGE(TAG, "TWDT configure failed. Exiting task...");
        // Delete this task if the custom TWDT has not been initialized
        vTaskDelete(NULL);
    }
    // Assign custom TWDT to this task
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    // Begin Inference Task
    // Check for valid data structures
    if (!input || !input->data.f) {
        ESP_LOGE(TAG, "inference_task sees null input->data.f. Suspending task...");
        vTaskSuspend(NULL); 
    } else {
        ESP_LOGI(TAG,"inference_task verified input pointer: %p", input->data.f);
    }

    // Allocate memory on the PSRAM heap for the input buffer
    float *input_buffer = (float *)heap_caps_malloc(
        MODEL_INPUT_SIZE * sizeof(float),
        MALLOC_CAP_SPIRAM
    );
    // Allocate memory on the PSRAM heap for the filtered data buffer
    float *filtered = (float *)heap_caps_malloc(
        RAW_BUFFER_SIZE * sizeof(float),
        MALLOC_CAP_SPIRAM
    );
    // Allocate memory on the PSRAM heap for the buffer of raw channel 1 samples
    int32_t *raw_samples = (int32_t *)heap_caps_malloc(
        RAW_BUFFER_SIZE * sizeof(int32_t),
        MALLOC_CAP_SPIRAM
    );
    // Allocate memory on the SRAM heap for the output buffer
    float *output_buffer = (float *)heap_caps_malloc(
        OUTPUT_SIZE * sizeof(float),
        MALLOC_CAP_INTERNAL     // TODO: TEST; changed from MALLOC_CAP_INTERNAL
        // Dev note: setting MALL_CAP_SPIRAM seems to make the system crash sooner??
    );

    // Check for proper memory allocation
    if (!input_buffer || !filtered || !raw_samples || !output_buffer) {
        ESP_LOGE(TAG, "Failed to allocate preprocessing buffers in PSRAM");
        vTaskSuspend(NULL);
    }

    // Wait for plot to be calibrated before beginning inference
    // Theoretically helps prevent transient data from skewing ML predictions
    while(!is_plot_calibrated()) {
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_task_wdt_reset();
    }
    ESP_LOGI(TAG, "Starting main inference loop.");
    // RUN MAIN INFERENCE LOOP
    for ( ;; ) {
        // Feed regular TWDT and delay for 50ms
        vTaskDelay(pdMS_TO_TICKS(50));
        // Feed custom TWDT
        esp_task_wdt_reset();

        // // For testing:
        // // Stuff the TFLM input tensor with hardcoded test data from models.cc
        // for (int i = 0; i < test_data_1_len; i++) {
        //     input->data.f[i] = test_data_1[i];
        // }

        // For production
        // Copy ecg samples from queue into the raw samples buffer
        // Wait until the queue is completely full
        // Although the queue may not always be full, the data will always be processed sequentially
        //  as xQueueReceive will will wait until the queue has more data
        // If there is no data to raw samples to be retrieved, then keep waiting for 5 seconds.
        // If there is still no data to be retrieved, then reset the TWDT and try again
        int num_samples = 0;
        while (num_samples < RAW_BUFFER_SIZE) {
            if (xQueueReceive(ecg_data_model_input_queue, &raw_samples[num_samples], pdMS_TO_TICKS(5000)) == pdTRUE) {
                num_samples++;
                esp_task_wdt_reset();
            } else {
                // No samples are ready
                
                // Feed TWDT just in case the ECG stream is slow
                esp_task_wdt_reset();
                ESP_LOGE(TAG, "Failed to receive data from ecg_data_model_input_queue. Received %d samples so far.", num_samples);

                continue;
            }
        }   // exit once the raw sample buffer has RAW_BUFFER_SIZE sequential samples

        // Preprocess the raw data samples
        // Check that the input buffer is successfully created
        if (!create_input_buffer(input_buffer, raw_samples, filtered, num_samples)) {
            ESP_LOGE(TAG, "The input buffer failed to be created.");
            continue;
        }

        // Load the processed input data into the TFLM input tensor
        for (int i = 0; i < MODEL_INPUT_SIZE; i++) {
            // input_buffer[i] = 0.0f;
            input->data.f[i] = input_buffer[i];

        }
        
        ESP_LOGI(TAG, "Invoking model inference...");
        // Invoke model inference
        if (interpreter->Invoke() != kTfLiteOk) {
            ESP_LOGE(TAG, "Invoke Failed.");
            continue;
        }
        ESP_LOGI(TAG, "Invoke success.");

        // Feed the custom TWDT to prevent timeout errors
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(50));

        // Load the output buffer with data from TFLM output tensor
        ESP_LOGI(TAG, "Loading output buffer...");
        for (int i = 0; i < OUTPUT_SIZE; i++) {
            output_buffer[i] = output->data.f[i];
        }
        // Send output buffer to the output queue
        if (xQueueSend(model_output_queue, output_buffer, 0) == pdTRUE) {
            ESP_LOGI(TAG, "Output data added to model output queue.");
        } else {
            ESP_LOGW(TAG, "Failed to add output data to model output queue.");
        }

        // Print model inference output
        ESP_LOGI(TAG, 
            "Inference Output:\n\tAfib:\t%f\n\tNormal:\t%f\n\tVfib:\t%f",
            output_buffer[0], output_buffer[1], output_buffer[2]
        );

        // uint8_t classified_idx = get_prediction_idx(output_buffer, OUTPUT_SIZE);

        // ESP_LOGI(TAG, "Predicted class has class id of %d", classified_idx);

        // const char *output_class = output_classes[classified_idx];

        // vTaskDelay(pdMS_TO_TICKS(10));
        // MicroPrintf("=========================");
        // MicroPrintf("Predicted Class:");
        // MicroPrintf("\t%s", output_class);
        // MicroPrintf("=========================");
    }
}