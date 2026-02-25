#include "ecg.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "lvgl.h"

#include "esp_log.h"
static const char *TAG = "ecg.c";

// ECG SPI device handles
spi_host_device_t ecg_host_device;
spi_device_handle_t ecg_handle;

// Queue for all ECG samples to be processed
QueueHandle_t ecg_sample_queue = NULL;

const gpio_num_t ECG_SCLK_PIN = 9;
const gpio_num_t ECG_SDI_PIN = 10;
const gpio_num_t ECG_SDO_PIN = 11;
const gpio_num_t ECG_CSB_PIN = 12;
const gpio_num_t ECG_ALAB_PIN = 13;
const gpio_num_t ECG_DRDB_PIN = 14;

static bool full_queue_flag = false;
static bool error_flag = false;

led_strip_handle_t board_led_handle;

// Debug setting
#define _TESTING 1

#define NUM_BYTES_ECG_SAMPLE 8 // do not modify; dummy (1) + DATA_STATUS (1) + CH1(3) + CH2(3)
#define ECG_CLOCK_FREQUENCY 4000000 // do not modify

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
    ESP_ERROR_CHECK(spi_device_transmit(ecg_handle, &transaction));
}

void read_register(uint8_t reg_address, uint8_t reg_i) {
    uint8_t tx_buffer_data[2] = {(0x80 | reg_address), 0x00};
    uint8_t rx_buffer_data[3] = {0};

    spi_transaction_t transaction = {};
    transaction.flags = 0;
    transaction.length = 3 * 8;

    transaction.tx_buffer = tx_buffer_data;
    transaction.rx_buffer = rx_buffer_data;

    ESP_ERROR_CHECK(spi_device_transmit(ecg_handle, &transaction));

    if (reg_i >= sizeof(rx_buffer_data)) {
        return;
    }

    uint16_t i;
    printf("\n");
    for (i = 0; i < sizeof(rx_buffer_data); i++) {
        if (i != reg_i) {
            continue;
        }
        printf("%#08x was read from register %#08x[%d].\n", rx_buffer_data[i], reg_address, i);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void init_ecg() {
    if (!INCLUDE_ECG) {
        return;
    }
    if (_TESTING) ESP_LOGI(TAG, "In init_ecg()");
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
    write_ecg_data(0x21, 0x02); // R2_decimation=5 for all channels
    write_ecg_data(0x22, 0x02); // R3_decimation=6 for CH1
    write_ecg_data(0x23, 0x02); // R3_decimation=6 for CH2
    // write_ecg_data(0x22, 0x08); // R3_decimation=12 for CH1
    // write_ecg_data(0x23, 0x08); // R3_decimation=12 for CH2
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
    // uint8_t rx_buffer_data[2] = {0};
    // uint8_t tx_buffer_data[2] = {0x20 | 0x00, 0x00};

    spi_transaction_t transaction = {};
    transaction.flags = 0;  //SPI_TRANS_CS_KEEP_ACTIVE;
    transaction.length = NUM_BYTES_ECG_SAMPLE * 8;
    // transaction.tx_data[0] = 0xD0; // start read
    // transaction.tx_data[1] = 0x00;
    // transaction.tx_data[2] = 0x00;
    // transaction.tx_data[3] = 0x00;
    transaction.tx_buffer = tx_buffer_data;
    transaction.rx_buffer = rx_buffer_data;

    if (ecg_handle == NULL) {       // debug
        ESP_LOGE(TAG, "handle not initialized");
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    ESP_ERROR_CHECK(spi_device_transmit(ecg_handle, &transaction));

    // printf("ID reg: 0x%02X\n", rx_buffer_data[1]);

    *status = rx_buffer_data[1];

    int32_t ch1_raw = (rx_buffer_data[2] << 16) | (rx_buffer_data[3] << 8) | rx_buffer_data[4];
    int32_t ch2_raw = (rx_buffer_data[5] << 16) | (rx_buffer_data[6] << 8) | rx_buffer_data[7];

    if (ch1_raw & 0x800000) ch1_raw |= 0xFF000000;
    if (ch2_raw & 0x800000) ch2_raw |= 0xFF000000;

    // *ch1 = (int16_t)(ch1_raw >> 8);
    // *ch2 = (int16_t)(ch2_raw >> 8);
    *ch1 = ch1_raw;
    *ch2 = ch2_raw;
}

void print_alarm_errors() {
    uint8_t rx_buffer_data[NUM_BYTES_ECG_SAMPLE] = {0};
    uint8_t tx_buffer_data[NUM_BYTES_ECG_SAMPLE] = {(0b10000000 | 0x19), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    spi_transaction_t transaction = {};
    transaction.flags = 0;
    transaction.length = NUM_BYTES_ECG_SAMPLE * 8;

    transaction.tx_buffer = tx_buffer_data;
    transaction.rx_buffer = rx_buffer_data;

    ESP_ERROR_CHECK(spi_device_transmit(ecg_handle, &transaction));

    uint8_t alarm = rx_buffer_data[1];
    if (alarm & (0x1 << 0)) ESP_LOGW(TAG, "CMOR");
    if (alarm & (0x1 << 1)) ESP_LOGW(TAG, "RLDRAIL");
    if (alarm & (0x1 << 2)) ESP_LOGW(TAG, "BATLOW");
    if (alarm & (0x1 << 3)) ESP_LOGW(TAG, "LEADOFF");
    if (alarm & (0x1 << 4)) ESP_LOGW(TAG, "CH1ERR");
    if (alarm & (0x1 << 5)) ESP_LOGW(TAG, "CH2ERR");
    if (alarm & (0x1 << 6)) ESP_LOGW(TAG, "CH3ERR");
    if (alarm & (0x1 << 7)) ESP_LOGW(TAG, "SYNCEDGEERR");
    vTaskDelay(pdMS_TO_TICKS(1));
}


void show_rgb_led(uint32_t color_r, uint32_t color_g, uint32_t color_b, uint32_t brightness) {
    // if (_TESTING) ESP_LOGI(TAG, "In show_rgb_led()");
    ESP_ERROR_CHECK(led_strip_set_pixel(
        board_led_handle,
        (0 * brightness) / 255,
        (color_r * brightness) / 255,
        (color_g * brightness) / 255,
        (color_b * brightness) / 255));
    ESP_ERROR_CHECK(led_strip_refresh(board_led_handle));
}


/*
Checks if the ADS1293 has data ready to send to the ESP.
Reads the available sample.
If an alarm is present, immediately return from this function without adding data to the queue.
Else, add the data to the queue. If the queue is full, then set the global full_queue_flag to true.
*/
void stream_ecg_data() {
    if (!INCLUDE_ECG) {
        return;
    }

    uint8_t status;
    int32_t ch1_raw;
    int32_t ch2_raw;
    static int decim = 0;

    if (gpio_get_level(ECG_DRDB_PIN) == 1) {        // check if Data Ready Data Bus has been set high (ADS1293 has data to send)
        // Stream data status, ch1 data, and ch2 data
        ecg_read_sample(&status, &ch1_raw, &ch2_raw);

        // Load the ecg_sample_t struct
        ecg_sample_t sample;
        sample.timestamp_us = esp_timer_get_time();
        sample.data_status = status;
        sample.ch1 = ch1_raw;
        sample.ch2 = ch2_raw;
        sample.ch3 = sample.ch1 - sample.ch2;

        // Debug print alarms
        if (_TESTING && gpio_get_level(ECG_ALAB_PIN) == 0) {
            print_alarm_errors();
            show_rgb_led(255, 0, 0, 25);
            error_flag = true;
            return;
        } else {
            if (error_flag) {
                error_flag = false;
                show_rgb_led(0, 255, 0, 25);
            }
        }
        // Check for valid data
        if (sample.data_status != 0) {
            if (_TESTING) {
                if (++decim >= 64) {        //5
                    // printf("%ld %ld\n", sample.timestamp_us, sample.ch1);
                    decim = 0;
                    if (uxQueueSpacesAvailable(ecg_sample_queue)) {     // check for queue space
                        xQueueSend(ecg_sample_queue, &sample, 0);
                        if (full_queue_flag) full_queue_flag = false;
                    } else {
                        if (_TESTING) ESP_LOGI(TAG, "ECG Data Sample Queue is Full at timestamp:%ld", sample.timestamp_us);
                        full_queue_flag = true;
                        // stop streaming if queue is full
                        return;
                    }
                }
            }
        }
        // if (uxQueueSpacesAvailable(ecg_sample_queue)) {     // check for queue space
        //     xQueueSend(ecg_sample_queue, &sample, 0);
        //     if (full_queue_flag) full_queue_flag = false;
        // } else {
        //     // if (_TESTING) ESP_LOGI(TAG, "ECG Data Sample Queue is Full at timestamp:%ld", sample.timestamp_us);
        //     // full_queue_flag = true;
        //     // stop streaming if queue is full
        //     return;
        // }

    }
    vTaskDelay(0);
}

// ------------------------
// Main ECG Streaming Task
// ------------------------
void ecg_stream_task(void *pvParameters) {
    if (_TESTING) ESP_LOGI(TAG, "Started ecg_stream_task()");

    // Create data queue for asynchronous data processing
    ecg_sample_queue = xQueueCreate(1024, sizeof(ecg_sample_t));        // ecg_sample_t is 17 bytes
    if (!ecg_sample_queue) {
        ESP_LOGE(TAG, "ECG sample queue could not be created.");
        vTaskDelete(NULL);
    }
    
    write_ecg_data(0x2F, 0x31); // enable CH2, CH1, and DATA_STATUS for loop read-back mode
    write_ecg_data(0x00, 0x01); // start conversion

    ESP_LOGI(TAG, "First suspend of ecg stream task...");
    vTaskSuspend(NULL);
    for ( ;; ) {
        if (!full_queue_flag) {
            stream_ecg_data();
        } else {
            show_rgb_led(255, 255, 0, 25);
            vTaskSuspend(NULL);
            show_rgb_led(0, 255, 0, 25);
            full_queue_flag = false;
        } 
        if (error_flag) {
            vTaskDelay(pdMS_TO_TICKS(1000));        // wiat before re-checking status
            write_ecg_data(0x00, 0x00); // stop lingering conversions
            write_ecg_data(0x2F, 0x31); // enable CH2, CH1, and DATA_STATUS for loop read-back mode
            write_ecg_data(0x00, 0x01); // start conversion

            ESP_LOGI(TAG, "Rechecking alarms...");
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (_TESTING) ESP_LOGW(TAG, "Ended ecg_stream_task()");     // this should theoretically never be called
}

void ecg_power_down() {
    if (!INCLUDE_ECG) {
        return;
    }
    
    ESP_ERROR_CHECK(spi_bus_remove_device(ecg_handle));
    ESP_ERROR_CHECK(spi_bus_free(ecg_host_device));
}