/**
 * @file tflm_wrapper.cc
 * @brief TensorFlow Lite Micro (TFLM) integration and inference task.
 *
 * This file contains the setup and execution of the TFLM interpreter,
 * memory allocation for the model and tensor arena in PSRAM, and the
 * FreeRTOS task that performs continuous ML inference on ECG data.
 */
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


/** @brief Pointer to the TensorFlow Lite model */
static const tflite::Model* model = nullptr;
/** @brief Pointer to the TFLM interpreter */
static tflite::MicroInterpreter* interpreter = nullptr;
/** @brief Pointer to the model's input tensor */
static TfLiteTensor* input = nullptr;
/** @brief Pointer to the model's output tensor */
static TfLiteTensor* output = nullptr;
/** @brief Pointer to the model data allocated in PSRAM */
static uint8_t *model_psram = nullptr;
/** @brief Pointer to the tensor arena allocated in PSRAM */
static uint8_t *tensor_arena_psram = nullptr;

/** @brief Required size for the tensor arena (minimum 350kB, set to 512KB) */
static constexpr int kTensorArenaSize = 512 * 1024;

/** @brief Task watchdog timer configuration */
static esp_task_wdt_config_t wdt_config = {};

/** @brief FreeRTOS queue for sending model predictions to the GUI */
QueueHandle_t model_output_queue = NULL;

/** @brief String labels corresponding to the output classifications */
const char *output_classes[OUTPUT_SIZE] = {"AFIB  ", "NORMAL", "VFIB  "};   // extra spaces keep padding on the data bar consistent

static const char *TAG = "tflm_wrapper.cc";

/**
 * @brief Initializes the TensorFlow Lite Micro framework.
 *
 * @details Allocates memory for the model and tensor arena in PSRAM,
 *          registers necessary operations, and allocates tensors.
 *          Also initializes the FreeRTOS queue for model output.
 *
 * @return 1 if initialization was successful, 0 otherwise.
 */

int tflm_init(void) {
    ESP_LOGI(TAG, "Starting tflm_init...");

    // Allocate Model in PSRAM
    model_psram = (uint8_t *) heap_caps_aligned_alloc(
        16,
        // model_afib_vfib_25_tflite_len,
        model_v4_1_tflite_len,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
    );
    if (!model_psram) {
        ESP_LOGE(TAG, "model_psram could not be allocated.");
        return 0;
    }
    // memcpy(model_psram, model_afib_vfib_25_tflite, model_afib_vfib_25_tflite_len);
    memcpy(model_psram, model_v4_1_tflite, model_v4_1_tflite_len);

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

    // Allocate tensor arena in PSRAM
    tensor_arena_psram = (uint8_t*)heap_caps_aligned_alloc(
        16,
        kTensorArenaSize,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
    );
    // Catch invalid tensor arena allocation in PSRAM
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

    // Catch invalid interpreter tensor allocation
    if (interpreter->AllocateTensors() != kTfLiteOk) {
        ESP_LOGE(TAG, "AllocateTensors() failed!");
        return 0;
    }

    input = interpreter->input(0);
    output = interpreter->output(0);

    // Catch invalid input and input data pointers
    if (!input || !input->data.f) {
        ESP_LOGE(TAG, "input->data.f is null.");
        return 0;
    }

    // Check that this log prints a valid address and not just 0x00
    // If it is 0x00/null then model inference will be invalid
    ESP_LOGI(TAG, "Input pointer address: %p", input->data.f);

    // Load model output queue to be used in gui.c
    // Create output queue if it does not already exist
    if (!model_output_queue) {
        model_output_queue = xQueueCreate(5, OUTPUT_SIZE * sizeof(float));
    }
    // Catch invalid output queue allocation
    if (!model_output_queue) {
        ESP_LOGE(TAG, "Failed to create model_output_queue.");
        return 0;
    }

    ESP_LOGI(TAG, "Returning from tflm_init()");
    
    return 1;
}

/**
 * @brief Configures a custom task watchdog timer (TWDT).
 * 
 * @details Allows the task watchdog timer to be reconfigured and extended so that
 *          reasonable timeouts from resource-intensive tasks do not cause system panic (and crashes).
 *
 * @return true if successfully configured.
 */
static bool set_wdt() {
    wdt_config.timeout_ms = 120000;     // 2 minutes
    wdt_config.idle_core_mask = (1 << portNUM_PROCESSORS) - 1;
    wdt_config.trigger_panic = true;

    ESP_ERROR_CHECK(esp_task_wdt_init(&wdt_config));
    ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&wdt_config));
    return true;
}

/**
 * @brief Finds the index of the maximum value in an array.
 *
 * @param output_buffer Pointer to the array of float predictions.
 * @param output_buffer_len Number of elements in the buffer.
 * @return uint8_t Index of the maximum value (predicted class).
 */
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

/**
 * @brief FreeRTOS task for running continuous model inference.
 *
 * @details This task waits for the ECG plot to be calibrated, then continuously
 *          pulls raw data from the ECG queue, preprocesses it, runs TFLM inference,
 *          and sends the prediction results to the output queue for the GUI.
 *
 * @param pvParameters Task parameters (unused).
 */
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
        MALLOC_CAP_INTERNAL   // TODO: TEST; changed from MALLOC_CAP_INTERNAL
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