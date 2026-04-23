
#ifndef TFLM_WRAPPER
#define TFLM_WRAPPER

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"


#ifdef __cplusplus
extern "C" {
#endif

// Number of output classes from the model
#define OUTPUT_SIZE 3

typedef enum {
    NORMAL = 0,
    AFIB = 1,
    VFIB = 2
} model_output_t;

extern const char *output_classes[3];

extern QueueHandle_t ecg_data_model_input_queue;

// QueueHandle_t get_model_output_queue(void);

int tflm_init(void);

void inference_task(void *pvParameters);

uint8_t get_prediction_idx(float *output_buffer, uint8_t output_buffer_len);


#ifdef __cplusplus
}
#endif

#endif  // TFLM_WRAPPER
