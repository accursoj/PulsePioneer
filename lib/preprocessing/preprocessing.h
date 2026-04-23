#pragma once
#ifdef __cplusplus
extern "C" {
#endif


#ifndef PULSEPIONEER_PREPROCESSING_H
#define PULSEPIONEER_PREPROCESSING_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Target resampling frequency expected by the model (Hz)
#define TARGET_FS       250

// Duration of ECG window used for inference (seconds)
#define WINDOW_SEC      6

// Raw buffer size for incoming ECG samples
#define RAW_BUFFER_SIZE 2000

// Model input size (1500 samples = 250 Hz × 6 sec)
#define MODEL_INPUT_SIZE (TARGET_FS * WINDOW_SEC)

// ECG sample structure
typedef struct {
    uint32_t timestamp_us;
    uint8_t data_status;
    int32_t ch1;
    int32_t ch2;
    int32_t ch3;
} ecg_sample_t;

bool create_input_buffer(float *model_input_buffer, int32_t *raw_samples, float *filtered, int num_received_samples);


#endif // PULSEPIONEER_PREPROCESSING_H

#ifdef __cplusplus
}
#endif