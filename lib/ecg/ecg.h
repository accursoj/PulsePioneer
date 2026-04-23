#pragma once
#ifdef __cplusplus
extern "C" {
#endif


#ifndef ECG_H
#define ECG_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include <stdint.h>
#include "driver/gpio.h"
#include "led_strip.h"
#include "led_strip_rmt.h"

// Enable ECG functionality
#define INCLUDE_ECG 1

extern const gpio_num_t ECG_SCLK_PIN;
extern const gpio_num_t ECG_SDI_PIN;
extern const gpio_num_t ECG_SDO_PIN;
extern const gpio_num_t ECG_CSB_PIN;
extern const gpio_num_t ECG_ALAB_PIN;
extern const gpio_num_t ECG_DRDB_PIN;


// Queue handle for streaming samples
extern QueueHandle_t ecg_sample_queue;

// Task handle for the main ECG streaming task
extern TaskHandle_t ecg_stream_task_handle;

extern led_strip_handle_t board_led_handle;

extern SemaphoreHandle_t rgb_led_mutex;
extern led_strip_handle_t board_led_handle;

// Initialize ECG SPI interface and ADS1293 registers
void init_ecg(void);

void show_rgb_led(uint32_t color_r, uint32_t color_g, uint32_t color_b, uint32_t brightness);


// Power down ECG SPI interface
void ecg_power_down(void);

void ecg_stream_task(void *pvParameters);

// QueueHandle_t get_ecg_data_model_input_queue(void);

void pass_rgb_led_mutex(SemaphoreHandle_t mutex);

#endif // ECG_H


#ifdef __cplusplus
}
#endif
