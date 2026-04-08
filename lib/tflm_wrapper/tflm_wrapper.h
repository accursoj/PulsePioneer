#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void tflm_init(void);

bool tflm_run(float *input, float *output);

void inference_task(void *pvParameters);

#ifdef __cplusplus
}
#endif