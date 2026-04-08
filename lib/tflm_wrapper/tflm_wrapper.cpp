#include "tflm_wrapper.h"

// FreeRTOS for tasking and queues
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// ESP-IDF system utilities
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_psram.h"
#include "esp_task_wdt.h"

// TensorFlow Lite Micro core headers
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/core/api/error_reporter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/kernels/kernel_util.h" 
#include "tensorflow/lite/kernels/internal/runtime_shape.h"



#include <math.h>
#include <string.h>

#include "ecg.h"
#include "waveform.h"

// TFLite model (converted to C array)
#include "models.h"

/**
 * ============================================================
 *                    SYSTEM CONFIGURATION
 * ============================================================
 */
#define TF_LITE_STATIC_MEMORY
// Target resampling frequency expected by the model (Hz)
#define TARGET_FS       250

// Duration of ECG window used for inference (seconds)
#define WINDOW_SEC      6

// Raw buffer size for incoming ECG samples (must exceed MODEL_INPUT_SIZE)
#define RAW_BUFFER_SIZE 2000

// Model input size (1500 samples = 250 Hz × 6 sec)
#define MODEL_INPUT_SIZE (TARGET_FS * WINDOW_SEC)

// Number of output classes from the model
#define OUTPUT_SIZE 3

static const char *TAG = "tflm_wrapper.cpp";

/**
 * ============================================================
 *                TFLM / MEMORY MANAGEMENT
 * ============================================================
 */
static float *test_data_psram = NULL;

// Pointer to model stored in PSRAM (or fallback to flash)
static uint8_t *model_psram = NULL;

// Tensor arena used by TFLM for all intermediate buffers
static uint8_t *tensor_arena = NULL;

// Size of tensor arena (must accommodate all model tensors)
static constexpr int kTensorArenaSize = 1024 * 1024;

// TFLM interpreter and tensor handles
static tflite::MicroInterpreter* interpreter = nullptr;
static TfLiteTensor* input = nullptr;
static TfLiteTensor* output = nullptr;

// Queue used to publish inference results
static QueueHandle_t model_output_queue = NULL;

static SemaphoreHandle_t tflm_init_done = NULL;

static esp_task_wdt_config_t wdt_config = {};



/**
 * ============================================================
 *                DIGITAL FILTER IMPLEMENTATION
 * ============================================================
 * Second-order Infinite Impulse Response (IIR) bandpass filter implemented as a biquad.
 * Maintains internal state across samples.
 */

// typedef struct {
//     float x1, x2; // Previous input samples
//     float y1, y2; // Previous output samples
// } biquad_state_t;

// Persistent filter state
// static biquad_state_t bp_state = {};

// IIR Filter coefficients (precomputed)
// #define B0  0.2066f
// #define B1  0.0f
// #define B2 -0.2066f
// #define A1 -0.3695f
// #define A2  0.5868f

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
 *                TFLM INITIALIZATION
 * ============================================================
 */
// class SimpleErrorReporter : public tflite::ErrorReporter {
// public:
//     int Report(const char* format, va_list args) override {
//         char buffer[256];
//         vsnprintf(buffer, sizeof(buffer), format, args);
//         ESP_LOGE("TFLM", "%s", buffer);
//         return 0;
//     }
// };

extern "C" {

/**
 * @brief Get handle to inference output queue.
 */
QueueHandle_t get_model_output_queue() {
    return model_output_queue;
}

/**
 * @brief Initialize TensorFlow Lite Micro interpreter.
 *
 * Responsibilities:
 *  - Allocate model in PSRAM
 *  - Configure operator resolver
 *  - Allocate tensor arena in PSRAM
 *  - Initialize interpreter and tensors
 */
void tflm_init(void) {
    bool init_ok = true;
    ESP_LOGI(TAG, "Free heap: %d", heap_caps_get_free_size(MALLOC_CAP_8BIT));
    ESP_LOGI(TAG, "Free PSRAM: %d", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    tflm_init_done = xSemaphoreCreateBinary();

    // Allocate test data in PSRAM
    // size_t test_data_size = (size_t)test_data_1_len;
    // test_data_psram = (float*) heap_caps_aligned_alloc(
    //     4,
    //     test_data_size,
    //     MALLOC_CAP_SPIRAM
    // );
    // memcpy(test_data_psram, test_data_1, test_data_size);    size_t test_data_size = (size_t)test_data_2_len;
    size_t test_data_size = (size_t) test_data_1_len;
    test_data_psram = (float*) heap_caps_aligned_alloc(
        4,
        test_data_size,
        MALLOC_CAP_SPIRAM
    );
    memcpy(test_data_psram, test_data_2, test_data_size);

    // Allocate model in PSRAM
    size_t model_size = (size_t)model_afib_vfib_25_tflite_len;

    model_psram = (uint8_t*) heap_caps_aligned_alloc(
        4,
        model_size,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
    );

    if (!model_psram) {
        ESP_LOGW(TAG, "PSRAM allocation failed, using flash model");
        model_psram = (uint8_t*)model_afib_vfib_25_tflite;
    } else {
        memcpy(model_psram, model_afib_vfib_25_tflite, model_size);
    }

    const tflite::Model* model = tflite::GetModel(model_psram);

    if (model->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "Model schema version mismatch: %d", model->version());
        init_ok = false;
    }

    ESP_LOGI(TAG, "Model schema version: %d", model->version());
    ESP_LOGI(TAG, "Expected schema version: %d", TFLITE_SCHEMA_VERSION);


    // Register only required operators to minimize memory usage
    static tflite::MicroMutableOpResolver<32> resolver;

    resolver.AddAdd();
    resolver.AddConv2D();
    resolver.AddExpandDims();
    resolver.AddFullyConnected();
    resolver.AddMaxPool2D();
    
    resolver.AddMean();
    resolver.AddMul();
    
    resolver.AddReshape();
    resolver.AddSoftmax();


    // Allocate tensor arena in PSRAM
    tensor_arena = (uint8_t*) heap_caps_aligned_alloc(
        16,
        kTensorArenaSize,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
    );

    if (tensor_arena == NULL) {
        ESP_LOGE(TAG, "Tensor arena allocation failed");
        init_ok = false;
    } else {
        ESP_LOGI(TAG, "Tensor arena allocated at %p", tensor_arena);
    }

    // Create interpreter
    static tflite::MicroInterpreter static_interpreter(
        model,
        resolver,
        tensor_arena,
        kTensorArenaSize
    );

    interpreter = &static_interpreter;

    if (interpreter->AllocateTensors() != kTfLiteOk) {
        ESP_LOGE(TAG, "AllocateTensors() failed");
        init_ok = false;
    } else {
        ESP_LOGI(TAG, "AllocateTensors() OK, input->data.f=%p", interpreter->input(0)->data.f);
    }

    input = interpreter->input(0);
    output = interpreter->output(0);

    ESP_LOGI(TAG, "Input pointer: %p", (void*)input);

    if (input) {
        ESP_LOGI(TAG, "Input dims pointer: %p", (void*)input->dims);
    }

    // auto* model = tflite::GetModel(model_psram);
    // auto* subgraphs = model->subgraphs();
    // auto* input_tensor = subgraphs->Get(0)->tensors()->Get(0);
    // auto* shape = input_tensor->shape();

    // if (shape == nullptr) {
    //     ESP_LOGE(TAG, "Model FlatBuffer actually has NO shape data!");
    // } else {
    //     ESP_LOGI(TAG, "Shape size in FlatBuffer: %d", shape->size());
    // }

    // Try accessing via the Micro-specific helper
    // TfLiteIntArray* ConvertToTfLiteIntArray(const tflite::RuntimeShape& shape) {
    //     // Create the array with the correct size
    //     TfLiteIntArray* dst = TfLiteIntArrayCreate(shape.DimensionsCount());
        
    //     for (int i = 0; i < shape.DimensionsCount(); ++i) {
    //         dst->data[i] = shape.Dims(i);
    //     }
    //     return dst;
    // }
    const int32_t micro_dims = tflite::GetTensorShape(input).DimensionsCount();
    if (micro_dims) {
        ESP_LOGI(TAG, "Micro helper found dims! Size: %d", micro_dims);
    } else {
        ESP_LOGE(TAG, "Micro helper also returned NULL");
    }

    // 1. Get the shape from the tensor safely
    tflite::RuntimeShape shape = tflite::GetTensorShape(input);

    // 2. Convert it to a TfLiteIntArray on the stack for your print function
    int stack_buf[4]; // Size = 1 (for .size) + 3 (your dims)
    TfLiteIntArray* safe_dims = reinterpret_cast<TfLiteIntArray*>(stack_buf);

    safe_dims->size = shape.DimensionsCount();
    for (int i = 0; i < safe_dims->size; ++i) {
        safe_dims->data[i] = shape.Dims(i);
    }

    for (int i = 0; i < safe_dims->size; ++i) {
        printf("%d ", safe_dims->data[i]);
    }
    printf("\n");


    ESP_LOGI(TAG, "Input type: %d", input->type);
    ESP_LOGI(TAG, "Output type: %d", output->type);

    // Allocate input buffer in PSRAM
    static float *input_buffer_psram = (float*) heap_caps_malloc(
        MODEL_INPUT_SIZE * sizeof(float),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
    );
    if (!input_buffer_psram) {
        ESP_LOGE(TAG, "Failed to allocate input buffer in PSRAM");
    } else {
        input->data.f = input_buffer_psram;  // manually assign backing memory
        ESP_LOGI(TAG, "Assigned input->data.f=%p", input->data.f);
    }

    // Allocate output buffer in PSRAM
    static float *output_buffer_psram = (float*) heap_caps_malloc(
        OUTPUT_SIZE * sizeof(float),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
    );
    if (!output_buffer_psram) {
        ESP_LOGE(TAG, "Failed to allocate output buffer in PSRAM");
    } else {
        output->data.f = output_buffer_psram; // manually assign backing memory
        ESP_LOGI(TAG, "Assigned output->data.f=%p", output->data.f);
    }

    ESP_LOGI(TAG, "Input type: %d", input->type);
    ESP_LOGI(TAG, "Output type: %d", output->type);

    if (!input->type || !output->type) {
        ESP_LOGE(TAG, "Either input->type or output->type is kTfLiteNoType. Check that the model has allocated properly.");
        init_ok = false;
    }

    ESP_LOGI(TAG, "Input tensor bytes: %d, data.f=%p", input->bytes, input->data.f);
    ESP_LOGI(TAG, "Output tensor bytes: %d, data.f=%p", output->bytes, output->data.f);
    
    ESP_LOGI(TAG, "Input size: %zu", interpreter->inputs_size());
    ESP_LOGI(TAG, "arena_used_bytes(): %zu", interpreter->arena_used_bytes());

    ESP_LOGI(TAG, "FLASH model bytes: %02x %02x %02x %02x",
        model_afib_vfib_25_tflite[4], model_afib_vfib_25_tflite[5],
        model_afib_vfib_25_tflite[6], model_afib_vfib_25_tflite[7]);

    ESP_LOGI(TAG, "Model first bytes: %02x %02x %02x %02x",
         model_psram[4], model_psram[5],
         model_psram[6], model_psram[7]);

    // Create output queue
    model_output_queue = xQueueCreate(5, OUTPUT_SIZE * sizeof(float));
    if (!model_output_queue) {
        ESP_LOGE(TAG, "model_output_queue could not be created.");
        init_ok = false;
    }

    if (init_ok) {
        ESP_LOGI(TAG, "TFLM initialization complete");
    } else {
        ESP_LOGE(TAG, "TFLM initialization failed.");
    }
    xSemaphoreGive(tflm_init_done);

}

/**
 * ============================================================
 *                    INFERENCE EXECUTION
 * ============================================================
 */

/**
 * @brief Run inference on prepared input data.
 *
 * @param input_data Preprocessed ECG signal
 * @param output_data Output probabilities
 */
bool tflm_run(float *input_data, float *output_data) {
    ESP_LOGI(TAG, "In tflm_run().");

    if (!interpreter || !input || !output || !input_data || !output_data) {
        ESP_LOGE(TAG, "Interpreter or tensors not ready");
        return false;
    }

    if (!input->data.f) {
        ESP_LOGE(TAG, "input->data.f is null");
        // return false;
    }
    if (!output->data.f) {
        ESP_LOGE(TAG, "output->data.f is null");
        // return false;
    }

    int input_len = input->bytes / sizeof(float);

    ESP_LOGI(TAG, "Started copying input data...");

    ESP_LOGI(TAG, "Input sample preview:");
    for (int i=0;i<15;i++){
        ESP_LOGI(TAG, "%f", input_data[i]);
    }

    // Copy data into input->data.f (assigned to PSRAM buffer)
    // for (int i = 0; i < input_len; i++) {
    for (int i = 0; i < 1500; i++) {
        // input->data.f[i] = input_data[i];
        if (i < 15) {
            ESP_LOGI(TAG, "Test data sample %d: %f", i, test_data_psram[i]);
        }
        input->data.f[i] = test_data_psram[i];
    }

    ESP_LOGI(TAG, "Copied input data");



    // TfLiteIntArray* dims = input->dims;

    // ESP_LOGI(TAG, "Dims pointer: %p", dims);
    // ESP_LOGI(TAG, "Dims size: %d", dims->size);

    //     // VERY IMPORTANT: guard against garbage size
    // if (dims->size <= 0 || dims->size > 8) {
    //     ESP_LOGE(TAG, "Dims size is invalid: %d", dims->size);
    //     return false;
    // }

    // for (int i = 0; i < dims->size; i++) {
    //     ESP_LOGI(TAG, "dim[%d] = %d", i, dims->data[i]);
    // }

    // TfLiteIntArray* dims = input->dims;
    // ESP_LOGI(TAG, "Input dims size: %d", dims->size);
    // for (int i = 0; i < dims->size; i++) {
    //     ESP_LOGI(TAG, "dim[%d] = %d", i, dims->data[i]);
    // }

    // int* dims_data = input->dims->data;

    // for (int i = 0; i < input->dims->size; i++) {
    //     ESP_LOGI(TAG, "dim[%d] = %d", i, dims_data[i]);
    // }

    // int total = 1;
    // for (int i = 0; i < input->dims->size; i++) {
    //     total *= input->dims->data[i];
    // }

    // ESP_LOGI(TAG, "Total elements: %d", total);

    esp_task_wdt_reset();

    if (interpreter->Invoke() != kTfLiteOk) {
        ESP_LOGE(TAG, "Model inference failed");
        return false;
    }

    esp_task_wdt_reset();

    ESP_LOGI(TAG, "Copying output...");

    for (int i = 0; i < OUTPUT_SIZE; i++) {
        ESP_LOGI(TAG, "Copying output %d:\t%f", i, output->data.f[i]);
        output_data[i] = output->data.f[i];
    }

    ESP_LOGI(TAG, "Copied output.");

    return true;

}

/**
 * ============================================================
 *                    PREPROCESSING PIPELINE
 * ============================================================
 */

float compute_mean(float *buf, int len){
    float sum = 0;
    for(int i=0;i<len;i++) sum += buf[i];
    return sum/len;
}

float compute_std(float *buf, int len){
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
 *
 * Steps:
 *  1. Acquire samples from queue
 *  2. Estimate sampling frequency
 *  3. Apply bandpass filtering
 *  4. Resample to fixed length
 *  5. Normalize signal
 *
 * @return true if successful
 */
bool create_input_buffer(QueueHandle_t input_queue, float *model_input, ecg_sample_t *raw_samples, float *filtered, int num_received_samples) {
    ESP_LOGI(TAG, "In create_input_buffer().");
    
    // Estimate sampling frequency (currently unused but kept for extensibility)
    float fs = compute_fs(raw_samples, num_received_samples);

    
    ESP_LOGI(TAG, "Raw sample preview:");
    for (int i=0;i<15;i++){
        ESP_LOGI(TAG, "%d", raw_samples[i].ch1);
    }

    ESP_LOGI(TAG, "Raw float sample preview:");
    for (int i=0;i<15;i++){
        ESP_LOGI(TAG, "%f", (float)raw_samples[i].ch1);
    }

    reset_bandpass_states();

    // Apply bandpass filter to ECG channel
    for (int i = 0; i < num_received_samples; i++) {
        // filtered[i] = bandpass_filter((float)raw_samples[i].ch1);
        filtered[i] = (float)raw_samples[i].ch1;
    }
    
    // Resample to model-required input length
    resample_signal(filtered, num_received_samples, model_input, MODEL_INPUT_SIZE);

    // Normalize signal (zero mean, unit variance)
    normalize(model_input, MODEL_INPUT_SIZE);

    ESP_LOGI(TAG,"First 15 filtered samples:");
    for(int i=0;i<15;i++) ESP_LOGI(TAG,"%f", filtered[i]);

    ESP_LOGI(TAG,"First 15 resampled samples:");
    for(int i=0;i<15;i++) ESP_LOGI(TAG,"%f", model_input[i]);

    ESP_LOGI(TAG,"Model input mean/std: mean=%f std=%f", compute_mean(model_input, MODEL_INPUT_SIZE), compute_std(model_input, MODEL_INPUT_SIZE));

    float* out = output->data.f;

    for (int i = 0; i < 3; i++) {
        ESP_LOGI(TAG, "RAW out[%d] = %e", i, out[i]);
    }
    ESP_LOGI(TAG, "Output bytes: %d", output->bytes);
    return true;
}

static bool set_wdt() {
    ESP_LOGI(TAG, "In set_wdt()");

    // set to 120 seconds
    wdt_config.timeout_ms = 120000;
    // monitor both processing cores
    wdt_config.idle_core_mask = (1 << portNUM_PROCESSORS) - 1;
    // abort system if timeout is reached
    wdt_config.trigger_panic = true;
    
    ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&wdt_config));

    return true;
}

/**
 * ============================================================
 *                    INFERENCE TASK
 * ============================================================
 */

/**
 * @brief FreeRTOS task that continuously performs inference.
 *
 * Workflow:
 *  - Wait for sufficient ECG samples
 *  - Preprocess data
 *  - Run model inference
 *  - Publish results to queue
 */
void inference_task(void *pvParameters) {
    ESP_LOGI(TAG, "Started inference_task().");

    if (!set_wdt()) {
        ESP_LOGI(TAG, "Custom TWDT could not be configured. Exiting inference_task()...");
        vTaskDelete(NULL);
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    // Assign custom TWDT config to this task
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    esp_task_wdt_reset();

    // float input_buffer[MODEL_INPUT_SIZE];
    // float output_buffer[OUTPUT_SIZE];

    QueueHandle_t input_queue = get_ecg_data_model_input_queue();

    // Allocate PSRAM for large arrays that hold either raw sample values and filtered sample values
    ecg_sample_t *raw_samples = (ecg_sample_t*) heap_caps_malloc(
        RAW_BUFFER_SIZE * sizeof(ecg_sample_t),
        MALLOC_CAP_SPIRAM
    );

    float *filtered = (float*) heap_caps_malloc(
        RAW_BUFFER_SIZE * sizeof(float),
        MALLOC_CAP_SPIRAM
    );

    // Allocate PSRAM for the input buffer
    float *input_buffer = (float*) heap_caps_malloc(
        MODEL_INPUT_SIZE * sizeof(float),
        MALLOC_CAP_SPIRAM
    );

    // Allocate PSRAM for the output buffer
    float *output_buffer = (float*) heap_caps_malloc(
        OUTPUT_SIZE * sizeof(float),
        MALLOC_CAP_SPIRAM
    );

    ESP_LOGI(TAG, "Checking inference_task stack highwater before inference_task(): %d", uxTaskGetStackHighWaterMark(NULL));
    
    // Wait until tflm_init() is done
    if (tflm_init_done) {
        xSemaphoreTake(tflm_init_done, portMAX_DELAY);
        ESP_LOGI(TAG, "Received semaphore from tflm_init().");
    }

    for ( ;; ) {
        vTaskDelay(pdMS_TO_TICKS(50));
        ESP_ERROR_CHECK(esp_task_wdt_reset());


        // Can be removed after testing
        if (!raw_samples || !filtered || !input_buffer || !output_buffer) {
            ESP_LOGE(TAG, "Buffer pointer null! Skipping inference task...");
            continue;
        }

        if (!is_plot_calibrated()) {
            // ESP_LOGW(TAG, "Plot not yet calibrated. Skipping inference task...");
            continue;
        }

        // Copy ecg samples from queue into the raw samples buffer
        int received_samples_idx = 0;
        while (received_samples_idx < RAW_BUFFER_SIZE) {
            if (xQueueReceive(input_queue, &raw_samples[received_samples_idx], portMAX_DELAY) == pdTRUE) {
                received_samples_idx++;
            } else {
                // No samples are ready
                continue;
            }
        }

        int num_received_samples = received_samples_idx;
        if (!create_input_buffer(input_queue, input_buffer, raw_samples, filtered, num_received_samples)) {
            continue;
        }

        if (!interpreter || !interpreter->input(0) || !interpreter->output(0)) {
            ESP_LOGE(TAG, "TFLM interpreter not ready!");
            continue;
        }

        esp_task_wdt_reset();
        if (!tflm_run(input_buffer, output_buffer)) {
            ESP_LOGI(TAG, "tflm_run() failed.");
            continue;
        } else {
            ESP_LOGI(TAG, "tflm_run() completed successfully.");
            // Manually reset the custom TWDT
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        ESP_LOGI(TAG,
            "Inference Output:\n\tAfib:\t%.3f\n\tNormal:\t%.3f\n\tVfib:\t%.3f",
            output_buffer[0],
            output_buffer[1],
            output_buffer[2]
        );

        if (uxQueueSpacesAvailable(model_output_queue)) {
            xQueueSend(model_output_queue, output_buffer, 0);
        }
    }
}

} // extern "C"