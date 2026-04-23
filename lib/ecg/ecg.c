#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_heap_caps.h"

#include <../lvgl/lvgl.h>

#include "../ecg/ecg.h"
#include "../gui/gui.h"
#include "../preprocessing/preprocessing.h"

#define SETTLING_TIME 1000


static const char *TAG = "ecg.c";

// ECG SPI device handles
spi_host_device_t ecg_host_device;
spi_device_handle_t ecg_handle;

// Queue for all ECG samples to be processed (base type of ecg_sample_t)
QueueHandle_t ecg_sample_queue = NULL;
// Queue to forward ECG data to model task (base type of int32_t)
QueueHandle_t ecg_data_model_input_queue = NULL;
// Queue to pass the ALAB pin state from the ISR to the handler task
QueueHandle_t alarm_event_queue = NULL;
TaskHandle_t alarm_task_handle = NULL;

// SPI bus mutex
// Allows both ecg_streaming_task and alarm_handler_task to share the spi bus
SemaphoreHandle_t ecg_spi_mutex = NULL;


const gpio_num_t ECG_SCLK_PIN = 9;
const gpio_num_t ECG_SDI_PIN = 10;
const gpio_num_t ECG_SDO_PIN = 11;
const gpio_num_t ECG_CSB_PIN = 12;
const gpio_num_t ECG_ALAB_PIN = 13;
const gpio_num_t ECG_DRDB_PIN = 14;

static bool full_queue_flag = false;
static bool error_flag = false;
static bool is_first_normal_pass = true;

// Debug setting
#define _TESTING 1

#define NUM_BYTES_ECG_SAMPLE 8 // DO NOT MODIFY; dummy (1) + DATA_STATUS (1) + CH1(3) + CH2(3)
#define ECG_CLOCK_FREQUENCY 4000000 // do not modify

#define ECG_QUEUE_SIZE 2000//256
#define ECG_QUEUE_ITEM_SIZE sizeof(ecg_sample_t)

// Static memory allocation for the ML input queue to avoid runtime heap fragmentation
#define MODEL_QUEUE_SIZE 2000 //2048
static uint8_t model_queue_storage[MODEL_QUEUE_SIZE * sizeof(int32_t)];
static StaticQueue_t model_queue_struct;


void show_rgb_led(uint32_t color_r, uint32_t color_g, uint32_t color_b, uint32_t brightness) {
    // if (_TESTING) ESP_LOGI(TAG, "In show_rgb_led()");
    if (board_led_handle == NULL || rgb_led_mutex == NULL) return;
    
    if (rgb_led_mutex) {
        if (xSemaphoreTake(rgb_led_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            esp_err_t err = led_strip_set_pixel(
                board_led_handle,
                0,
                (color_r * brightness) / 255,
                (color_g * brightness) / 255,
                (color_b * brightness) / 255);
                
            if (err == ESP_OK) {
                err = led_strip_refresh(board_led_handle);
                if (err != ESP_OK && _TESTING) {
                    ESP_LOGW(TAG, "RGB LED refresh failed (likely busy): %s", esp_err_to_name(err));
                }
            }
    
            xSemaphoreGive(rgb_led_mutex);
        } else {
            ESP_LOGE(TAG, "xSemaphoreTake() called on rgb_led_mutex failed in show_rgb_led()");
        }
    } else {
        ESP_LOGE(TAG, "xSemaphoreTake() called while rgb_led_mutex is NULL in show_rgb_led()");
    }
}

void write_ecg_data(uint8_t addr, uint8_t data) {
    if (!INCLUDE_ECG) {
        return;
    }

    spi_transaction_t transaction = {};
    transaction.flags = SPI_TRANS_USE_TXDATA;
    transaction.length = 16;
    transaction.rxlength = 0;
    transaction.tx_data[0] = addr & 0x7F; // clear MSB for WRITE
    transaction.tx_data[1] = data;

    // if (ecg_spi_mutex) {
        // if (xSemaphoreTake(ecg_spi_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            ESP_ERROR_CHECK(spi_device_transmit(ecg_handle, &transaction));
            // xSemaphoreGive(ecg_spi_mutex);
        // } else {
        //     ESP_LOGE(TAG, "xSemaphoreTake() called on ecg_spie_mutex failed in write_ecg_data()");
        // }
    // } else {
    //     ESP_LOGE(TAG, "xSemaphoreTake() called while ecg_spi_mutex is NULL in write_ecg_data()");
    // }
}

void read_register(uint8_t reg_address, uint8_t reg_i) {
    uint8_t tx_buffer_data[2] = {(0x80 | reg_address), 0x00};
    uint8_t rx_buffer_data[3] = {0};

    spi_transaction_t transaction = {};
    transaction.flags = 0;
    transaction.length = 3 * 8;

    transaction.tx_buffer = tx_buffer_data;
    transaction.rx_buffer = rx_buffer_data;

    // if (ecg_spi_mutex) {
        // if (xSemaphoreTake(ecg_spi_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            ESP_ERROR_CHECK(spi_device_transmit(ecg_handle, &transaction));
        //     xSemaphoreGive(ecg_spi_mutex);
        // } else {
        //     ESP_LOGE(TAG, "xSemaphoreTake() called on ecg_spi_mutex failed in read_register()");
        // }
    // } else {
    //     ESP_LOGE(TAG, "xSemaphoreTake() called while ecg_spi_mutex is NULL in read_register()");
    // }   

    if (reg_i >= sizeof(rx_buffer_data)) {
        return;
    }

    uint16_t i;
    // printf("\n");
    for (i = 0; i < sizeof(rx_buffer_data); i++) {
        if (i != reg_i) {
            continue;
        }
        // printf("%#08x was read from register %#08x[%d].\n", rx_buffer_data[i], reg_address, i);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

const char* alarm_types[] = {"CMOR", "RLDRAIL", "BATLOW", "LEADOFF", "CH1ERR", "CH2ERR", "CH3ERR", "SYNCEDGEERR"};

uint8_t process_alarms() {
    uint8_t rx_buffer_data[NUM_BYTES_ECG_SAMPLE] = {0};
    uint8_t tx_buffer_data[NUM_BYTES_ECG_SAMPLE] = {(0b10000000 | 0x19), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    spi_transaction_t transaction = {};
    transaction.flags = 0;
    transaction.length = NUM_BYTES_ECG_SAMPLE * 8;

    transaction.tx_buffer = tx_buffer_data;
    transaction.rx_buffer = rx_buffer_data;

    // if (ecg_spi_mutex) {
    //     if (xSemaphoreTake(ecg_spi_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            ESP_ERROR_CHECK(spi_device_transmit(ecg_handle, &transaction));
    //         xSemaphoreGive(ecg_spi_mutex);
    //     } else {
    //         ESP_LOGE(TAG, "xSemaphoreTake() called on ecg_spi_mutex failed in process_alarms()");
    //     }
    // } else {
    //     ESP_LOGE(TAG, "xSemaphoreTake() while ecg_spi_mutex is NULL in process_alarms()");
    // }

    uint8_t alarm = rx_buffer_data[1];

    vTaskDelay(pdMS_TO_TICKS(1));

    return alarm;
}

void print_alarm_errors(uint8_t alarm) {

    if (alarm & (0x1 << 0)) ESP_LOGW(TAG, "CMOR");
    if (alarm & (0x1 << 1)) ESP_LOGW(TAG, "RLDRAIL");
    if (alarm & (0x1 << 2)) ESP_LOGW(TAG, "BATLOW");
    if (alarm & (0x1 << 3)) ESP_LOGW(TAG, "LEADOFF");
    if (alarm & (0x1 << 4)) ESP_LOGW(TAG, "CH1ERR");
    if (alarm & (0x1 << 5)) ESP_LOGW(TAG, "CH2ERR");
    if (alarm & (0x1 << 6)) ESP_LOGW(TAG, "CH3ERR");
    if (alarm & (0x1 << 7)) ESP_LOGW(TAG, "SYNCEDGEERR");

    return;
}


// static void IRAM_ATTR ecg_alarm_isr_handler(void *arg) {
//     esp_rom_printf("ECG ALARM ISR FIRED\n");
//     // Disable interrupt to prevent bounce
//     gpio_intr_disable(ECG_ALAB_PIN);
    
//     BaseType_t xHigherPriorityTaskWoken = pdFALSE;

//     // Send a generic signal
//     int dummy_event = 1;
//     if (xQueueSendFromISR(alarm_event_queue, &dummy_event, &xHigherPriorityTaskWoken) != pdTRUE) {
//         // If event fails, then still re-enable the ISR
//         gpio_intr_enable(ECG_ALAB_PIN);
//     }

//     if (xHigherPriorityTaskWoken == pdTRUE) {
//         portYIELD_FROM_ISR();
//     }
// }

// void alarm_handler_task(void *pvParameters) {
//     ESP_LOGI(TAG, "Started alarm_handler_task");
//     int pin_state;
//     int dummy_event = 1;

//     // Catch alarms that were already active before the system booted
//     if (gpio_get_level(ECG_ALAB_PIN) == 0) {
//         xQueueSend(alarm_event_queue, &dummy_event, 0); 
//     }

//     if (xLVGLSemaphore) {
//         if (xSemaphoreTake(xLVGLSemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
//             update_sys_state_text("Ready");
//             xSemaphoreGive(xLVGLSemaphore);
//         } else {
//             ESP_LOGE(TAG, "xSemaphoreTake() called on xLVGLSemaphore failed.");
//         }
//     } else {
//         ESP_LOGW(TAG, "xSemaphoreTake() called on xLVGLSemaphore while it is NULL!");
//     }

//     // // Ensure we start in a known UI state
//     // lv_async_call((lv_async_cb_t)update_sys_state_text, "Ready");
//     // show_rgb_led(0, 255, 0, 25);


//     for ( ;; ) {
//         TickType_t wait_time = pdMS_TO_TICKS(1000);

//         if (xQueueReceive(alarm_event_queue, &dummy_event, wait_time) == pdTRUE) {
//             // Debounce
//             vTaskDelay(pdMS_TO_TICKS(50));
//             pin_state = gpio_get_level(ECG_ALAB_PIN);

//             // ESP_LOGI(TAG,"Received pin state: %d", pin_state);
//             if (!pin_state) {
//                 // ESP_LOGW(TAG, "Hardware Alarm Triggered");
//                 show_rgb_led(255, 0, 0, 25); // Red
//                 if (xLVGLSemaphore) {
//                     if (xSemaphoreTake(xLVGLSemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
//                         update_sys_state_text("Alarm");
//                         xSemaphoreGive(xLVGLSemaphore);
//                     } else {
//                         ESP_LOGE(TAG, "xSemaphoreTake() called on xLVGLSemaphore failed.");
//                     }
//                 } else {
//                     ESP_LOGW(TAG, "xSemaphoreTake() called on xLVGLSemaphore while it is NULL!");
//                 }

//                 // Reading the alarm register clears the latch, allowing the pin to return high if the fault is gone
//                 uint8_t alarm_val = process_alarms();
//                 vTaskDelay(wait_time);
//                 // if (_TESTING) print_alarm_errors(alarm_val);
//             } else {
//                 // ESP_LOGI(TAG, "Hardware Alarm Cleared");
//                 show_rgb_led(0, 255, 0, 25); // Green
//                 if (xLVGLSemaphore) {
//                     if (xSemaphoreTake(xLVGLSemaphore, pdMS_TO_TICKS(1000))) {
//                         update_sys_state_text("Ready");
//                         xSemaphoreGive(xLVGLSemaphore);
//                     } else {
//                         ESP_LOGE(TAG, "xSemaphoreTake() called on xLVGLSemaphore failed.");
//                     }
//                 } else {
//                     ESP_LOGW(TAG, "xSemaphoreTake() called on xLVGLSemaphore while it is NULL!");
//                 }
//             }
//             // Re-enable the interrupt after processing the state change
//             gpio_intr_enable(ECG_ALAB_PIN);
//         } else {
//             // If we timed out and the pin is still stuck low due to a persistent physical fault, 
//             // we must periodically clear the latch so a rising edge can be generated the moment it is fixed.
//             if (gpio_get_level(ECG_ALAB_PIN) == 0) {
//                 process_alarms();
//             }
//         }
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }

void init_ecg() {
    if (!INCLUDE_ECG) {
        return;
    }
    if (_TESTING) ESP_LOGI(TAG, "In init_ecg()");

    // // Create SPI bus mutex
    // ecg_spi_mutex = xSemaphoreCreateMutex();
    // if (ecg_spi_mutex == NULL) {
    //     ESP_LOGE(TAG, "Failed to create SPI mutex!");
    // }

    // Setup SPI Bus

    ecg_host_device = SPI2_HOST;

    spi_bus_config_t ecg_bus_config = {};
    ecg_bus_config.mosi_io_num = ECG_SDI_PIN;
    ecg_bus_config.miso_io_num = ECG_SDO_PIN;
    ecg_bus_config.sclk_io_num = ECG_SCLK_PIN;
    ecg_bus_config.max_transfer_sz = 4096;
    ecg_bus_config.isr_cpu_id = ESP_INTR_CPU_AFFINITY_1;

    // Not used
    ecg_bus_config.quadwp_io_num = -1;
    ecg_bus_config.quadhd_io_num = -1;
    ecg_bus_config.data_io_default_level = 0;

    ESP_ERROR_CHECK(spi_bus_initialize(ecg_host_device, &ecg_bus_config, SPI_DMA_CH_AUTO));     // Use DMA

    spi_device_interface_config_t ecg_interface_config = {};
    ecg_interface_config.command_bits = 0;
    ecg_interface_config.address_bits = 0;
    ecg_interface_config.dummy_bits = 0;
    ecg_interface_config.mode = 0; // CPOL=0, CPHA=0
    ecg_interface_config.clock_speed_hz = ECG_CLOCK_FREQUENCY;
    ecg_interface_config.spics_io_num = ECG_CSB_PIN;
    ecg_interface_config.queue_size = 1;

    ESP_ERROR_CHECK(spi_bus_add_device(ecg_host_device, &ecg_interface_config, &ecg_handle));


    // // Create the alarm event queue
    // alarm_event_queue = xQueueCreate(10, sizeof(int));
    // if (alarm_event_queue == NULL) {
    //     ESP_LOGE(TAG, "Failed to create alarm_event_queue");
    // }

    // // Continue if this install fails
    // esp_err_t isr_err = gpio_install_isr_service(0);
    // if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
    //     ESP_ERROR_CHECK(isr_err);
    // }

    // ESP_ERROR_CHECK(gpio_isr_handler_add(ECG_ALAB_PIN, ecg_alarm_isr_handler, NULL));
    
    // // Explicitly enable the interrupt to ensure it is not masked
    // gpio_intr_enable(ECG_ALAB_PIN);

    // // Add the alarm handler task with priority 4 on core 0
    // //      uxPriority and core selection should match ecg_stream_task to allow for proper time slicing
    // xTaskCreatePinnedToCore(alarm_handler_task, "alarm_task", 4096, NULL, 2, &alarm_task_handle, 0);

    // ESP_LOGI(TAG, "Added alarm event task");

    // Configure ADS1293 registers for 3-lead ECG
    write_ecg_data(0x00, 0x00); // stop lingering conversions
    write_ecg_data(0x01, 0x11); // connect CH1's INP to IN2 and INN to IN1
    write_ecg_data(0x02, 0x19); // connect CH2's INP to IN3 and INN to IN1
    write_ecg_data(0x0A, 0x07); // enable common-mode detector on IN1, IN2, IN3
    write_ecg_data(0x0C, 0x04); // internally connect output of RLD amplifier to IN4
    write_ecg_data(0x12, 0x04); // use external crystal oscilaltor
    // write_ecg_data(0x13, 0x0);  // disable high-resolution mode
    write_ecg_data(0x13, 0x03); // set CH1 and CH2 to high-resolution mode
    write_ecg_data(0x14, 0x24); // disable CH3 signal path
    write_ecg_data(0x21, 0x08); // R2_decimation=8 for all channels
    // write_ecg_data(0x21, 0x02); // R2_decimation=5 for all channels
    // write_ecg_data(0x22, 0x02); // R3_decimation=6 for CH1
    // write_ecg_data(0x23, 0x02); // R3_decimation=6 for CH2
    write_ecg_data(0x22, 0x08); // R3_decimation=12 for CH1
    write_ecg_data(0x23, 0x08); // R3_decimation=12 for CH2
    write_ecg_data(0x27, 0x08); // set DRDYB source to CH1
    write_ecg_data(0x2F, 0x30); // enable CH1 and CH2 ECGs for loop read-back mode

    vTaskDelay(pdMS_TO_TICKS(100));

    if (_TESTING) {
        read_register(0x00, 1);
        read_register(0x01, 1);
        read_register(0x02, 1);
        read_register(0x0A, 1);
        read_register(0x0C, 1);
        read_register(0x12, 1);
        read_register(0x13, 1);
        read_register(0x14, 1);
        read_register(0x21, 1);
        read_register(0x22, 1);
        read_register(0x23, 1);
        read_register(0x27, 1);
        read_register(0x2F, 1);
    }
}

void ecg_read_sample(uint8_t *status, int32_t *ch1, int32_t *ch2) {
    if (!INCLUDE_ECG) {
        return;
    }

    uint8_t rx_buffer_data[NUM_BYTES_ECG_SAMPLE];
    uint8_t tx_buffer_data[NUM_BYTES_ECG_SAMPLE] = {0xD0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    spi_transaction_t transaction = {};
    transaction.flags = 0;  //SPI_TRANS_CS_KEEP_ACTIVE;
    transaction.length = NUM_BYTES_ECG_SAMPLE * 8;
    transaction.tx_buffer = tx_buffer_data;
    transaction.rx_buffer = rx_buffer_data;

    if (ecg_handle == NULL) {       // debug
        ESP_LOGE(TAG, "handle not initialized");
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // if (ecg_spi_mutex) {
    //     if (xSemaphoreTake(ecg_spi_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            ESP_ERROR_CHECK(spi_device_transmit(ecg_handle, &transaction));
    //         xSemaphoreGive(ecg_spi_mutex);
    //     } else {
    //         ESP_LOGE(TAG, "xSemaphoreTake() called on ecg_spi_mutex failed in ecg_read_sample()");
    //     }
    // } else {
    //     ESP_LOGE(TAG, "xSemaphoreTake() called while ecg_spi_mutex is NULL in ecg_read_sample()");
    // }

    *status = rx_buffer_data[1];

    int32_t ch1_raw = (rx_buffer_data[2] << 16) | (rx_buffer_data[3] << 8) | rx_buffer_data[4];
    int32_t ch2_raw = (rx_buffer_data[5] << 16) | (rx_buffer_data[6] << 8) | rx_buffer_data[7];

    if (ch1_raw & 0x800000) ch1_raw |= 0xFF000000;
    if (ch2_raw & 0x800000) ch2_raw |= 0xFF000000;

    *ch1 = ch1_raw;
    *ch2 = ch2_raw;
}


/*
Checks if the ADS1293 has data ready to send to the ESP.
Reads the available sample.
If an alarm is present, immediately return from this function without adding data to the queue.
Else, add the data to the queue. If the queue is full, then set the global full_queue_flag to true.
*/
bool is_init_val = true;
void stream_ecg_data() {
    if (!INCLUDE_ECG) {
        return;
    }

    uint8_t status;
    int32_t ch1_raw;
    int32_t ch2_raw;

    static int32_t ch1_accum = 0;
    static int32_t ch2_accum = 0;
    static int decim = 0;
    const int decim_rate = 128;

    if (gpio_get_level(ECG_DRDB_PIN) == 1) {        // Check if ADS1293 Data Ready
        // Stream data status, ch1 data, and ch2 data
        ecg_read_sample(&status, &ch1_raw, &ch2_raw);

        // Load the ecg_sample_t struct
        ecg_sample_t sample;
        sample.timestamp_us = esp_timer_get_time();
        sample.data_status = status;
        sample.ch1 = ch1_raw;
        sample.ch2 = ch2_raw;
        sample.ch3 = sample.ch1 - sample.ch2;

        // Check for valid data
        // if (sample.data_status != 0) {
            // Accumulate the raw samples
            ch1_accum += sample.ch1;
            ch2_accum += sample.ch2;

            if (++decim >= decim_rate) {
                // Average the accumulated samples
                sample.ch1 = ch1_accum / decim_rate;
                sample.ch2 = ch2_accum / decim_rate;
                sample.ch3 = sample.ch1 - sample.ch2;

                // Reset accumulators
                ch1_accum = 0;
                ch2_accum = 0;
                decim = 0;

                if (!uxQueueSpacesAvailable(ecg_sample_queue)) {
                    ESP_LOGW(TAG, "ecg_sample_queue overflowed. Resetting queues...");

                    while (uxQueueSpacesAvailable(ecg_sample_queue) != ECG_QUEUE_SIZE) {
                        ecg_sample_t dummy;
                        xQueueReceive(ecg_sample_queue, &dummy, 0);
                    }
                    while (uxQueueSpacesAvailable(ecg_data_model_input_queue) != MODEL_QUEUE_SIZE) {
                        int32_t dummy;
                        xQueueReceive(ecg_data_model_input_queue, &dummy, 0);
                    }
                    ESP_LOGI(TAG, "ecg_sample_queue and ecg_data_model_input_queue EMPTIED");
                    return;
                }

                if (xQueueSend(ecg_sample_queue, &sample, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "Failed to add sample to ecg_sample_queue");
                    return;     // skip adding data to the model output queue
                }

                if (is_plot_calibrated()) {
                    if (is_init_val) {
                        ESP_LOGI(TAG, "ECG data is now being forwarded to ecg_data_model_input_queue");
                        is_init_val = false;
                    }
                    if (!uxQueueSpacesAvailable(ecg_data_model_input_queue)) {
                        int32_t dummy;
                        // Log the failure
                        ESP_LOGE(TAG, "ML Queue Overflow! Window compromised. Flushing queue.");
                        
                        // Empty the queue completely to invalidate the corrupted window
                        xQueueReceive(ecg_data_model_input_queue, &dummy, 0);
                        
                    }
                    // Push the current sample to start building a fresh, contiguous window
                    xQueueSend(ecg_data_model_input_queue, &sample.ch1, 0);
                }
            }
        // } else {
        //     ESP_LOGW(TAG, "Sample data status == 0");
        // }
    }
}

// ------------------------
// Main ECG Streaming Task
// ------------------------
void ecg_stream_task(void *pvParameters) {
    if (_TESTING) ESP_LOGI(TAG, "Started ecg_stream_task()");

    // Create data queue in internal SRAM to avoid PSRAM bus contention with ML inference
    ecg_sample_queue = xQueueCreate(ECG_QUEUE_SIZE, ECG_QUEUE_ITEM_SIZE);
    if (!ecg_sample_queue) {
        ESP_LOGE(TAG, "ECG sample queue could not be created. Deleting ecg_stream_task...");
        vTaskDelete(NULL);
    }
    // Create data queue for forwarding data to model inference task
    ecg_data_model_input_queue = xQueueCreateStatic(MODEL_QUEUE_SIZE, sizeof(int32_t), model_queue_storage, &model_queue_struct);
    if (ecg_data_model_input_queue == NULL) {
        ESP_LOGE(TAG, "ecg_data_model_queue could not be created.");
    }

    // ecg_sample_queue = xQueueCreate(1024, sizeof(ecg_sample_t));        // ecg_sample_t is 17 bytes
    // if (!ecg_sample_queue) {
    //     ESP_LOGE(TAG, "ECG sample queue could not be created.");
    //     vTaskDelete(NULL);
    // }
    // // Create data queue for forwarding data to model inference task
    // ecg_data_model_input_queue = xQueueCreate(1024, sizeof(int32_t));  // estimated to hold 8.5s worth of data
    // if (ecg_data_model_input_queue == NULL) {
    //     ESP_LOGE(TAG, "ecg_data_model_queue could not be created.");
    // }
    
    write_ecg_data(0x2F, 0x31); // enable CH2, CH1, and DATA_STATUS for loop read-back mode
    write_ecg_data(0x00, 0x01); // start conversion

    for ( ;; ) {

        stream_ecg_data();

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (_TESTING) ESP_LOGW(TAG, "Ended ecg_stream_task()");     // this should theoretically never be called
}

// QueueHandle_t get_ecg_data_model_input_queue() {
//     if (ecg_data_model_input_queue == NULL) {
//         ESP_LOGW(TAG, "ecg_model_input_queue was requested, but it has a value of NULL.");
//     }
//     return ecg_data_model_input_queue;
// }