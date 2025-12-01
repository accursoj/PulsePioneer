#ifndef ECG_H
#define ECG_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>
#include "driver/gpio.h"

extern const gpio_num_t ECG_SCLK_PIN;
extern const gpio_num_t ECG_SDI_PIN;
extern const gpio_num_t ECG_SDO_PIN;
extern const gpio_num_t ECG_CSB_PIN;
extern const gpio_num_t ECG_ALAB_PIN;
extern const gpio_num_t ECG_DRDB_PIN;

// ECG sample structure
typedef struct
{
    uint32_t timestamp_us;
    uint8_t data_status;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
} ecg_sample_t;

// Queue handle for streaming samples
extern QueueHandle_t ecg_sample_queue;

// Initialize ECG SPI interface and ADS1293 registers
void init_ecg(void);

// Write single register to ADS1293
void write_ecg_data(uint8_t addr, uint8_t data);

// Read one sample from ADS1293
void ecg_read_sample(uint8_t *status, int16_t *ch1, int16_t *ch2);

// Stream ECG samples continuously to queue
void stream_ecg_data(void);

// Power down ECG SPI interface
void ecg_power_down(void);

#endif // ECG_H