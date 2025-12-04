#include "ecg.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "lvgl.h"

// ECG SPI device handles
spi_host_device_t ecg_host_device;
spi_device_handle_t ecg_handle;

// Queue for all ECG samples to be processed
QueueHandle_t ecg_sample_queue;

// GPIO for DRDY pin
const gpio_num_t ECG_SCLK_PIN = 9;
const gpio_num_t ECG_SDI_PIN = 10;
const gpio_num_t ECG_SDO_PIN = 11;
const gpio_num_t ECG_CSB_PIN = 12;
const gpio_num_t ECG_ALAB_PIN = 13;
const gpio_num_t ECG_DRDB_PIN = 14;

#define INCLUDE_ECG 0

#define NUM_BYTES_ECG_SAMPLE 8  // dummy (1) + DATA_STATUS (1) + CH1(3) + CH2(3)

#define ECG_CLOCK_FREQUENCY 4000000

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

void init_ecg() {
    if (!INCLUDE_ECG) {
        return;
    }

    ecg_host_device = SPI2_HOST;

    spi_bus_config_t ecg_bus_config = {};
    ecg_bus_config.mosi_io_num = 10; // ECG_SDI_PIN
    ecg_bus_config.miso_io_num = 11; // ECG_SDO_PIN
    ecg_bus_config.sclk_io_num = 9;  // ECG_SCLK_PIN
    ecg_bus_config.max_transfer_sz = 4096;
    ecg_bus_config.quadwp_io_num = -1;
    ecg_bus_config.quadhd_io_num = -1;
    ecg_bus_config.data_io_default_level = 0;

    ESP_ERROR_CHECK(spi_bus_initialize(ecg_host_device, &ecg_bus_config, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t ecg_interface_config = {};
    ecg_interface_config.command_bits = 0;
    ecg_interface_config.address_bits = 0;
    ecg_interface_config.dummy_bits = 0;
    ecg_interface_config.mode = 0; // CPOL=0, CPHA=0
    ecg_interface_config.clock_speed_hz = ECG_CLOCK_FREQUENCY;
    ecg_interface_config.queue_size = 1;

    ESP_ERROR_CHECK(spi_bus_add_device(ecg_host_device, &ecg_interface_config, &ecg_handle));

    // Configure ADS1293 registers for 3-lead ECG
    write_ecg_data(0x00, 0x00); // stop conversion
    write_ecg_data(0x01, 0x11);
    write_ecg_data(0x02, 0x19);
    write_ecg_data(0x0A, 0x07);
    write_ecg_data(0x0C, 0x04);
    write_ecg_data(0x12, 0x04);
    write_ecg_data(0x14, 0x24);
    write_ecg_data(0x21, 0x02);
    write_ecg_data(0x22, 0x02);
    write_ecg_data(0x23, 0x02);
    write_ecg_data(0x27, 0x08);
    write_ecg_data(0x2F, 0x30);
}

void ecg_read_sample(uint8_t *status, int16_t *ch1, int16_t *ch2) {
    if (!INCLUDE_ECG) {
        return;
    }

    uint8_t rx_buffer_data[NUM_BYTES_ECG_SAMPLE];

    spi_transaction_t transaction = {};
    transaction.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_CS_KEEP_ACTIVE;
    transaction.length = NUM_BYTES_ECG_SAMPLE * 8;
    transaction.tx_data[0] = 0xD0; // start read
    transaction.tx_data[1] = 0x00;
    transaction.tx_data[2] = 0x00;
    transaction.tx_data[3] = 0x00;
    transaction.rx_buffer = rx_buffer_data;

    ESP_ERROR_CHECK(spi_device_transmit(ecg_handle, &transaction));

    *status = rx_buffer_data[1];

    int32_t ch1_raw = (rx_buffer_data[2] << 16) | (rx_buffer_data[3] << 8) | rx_buffer_data[4];
    int32_t ch2_raw = (rx_buffer_data[5] << 16) | (rx_buffer_data[6] << 8) | rx_buffer_data[7];

    if (ch1_raw & 0x800000) ch1_raw |= 0xFF000000;
    if (ch2_raw & 0x800000) ch2_raw |= 0xFF000000;

    *ch1 = (int16_t)(ch1_raw >> 8);
    *ch2 = (int16_t)(ch2_raw >> 8);
}

void stream_ecg_data() {
    if (!INCLUDE_ECG) {
        return;
    }

    uint8_t status;
    int16_t ch1_raw;
    int16_t ch2_raw;

    ecg_sample_queue = xQueueCreate(256, sizeof(ecg_sample_t));
    write_ecg_data(0x2F, 0x31); // enable CH2, CH1, DATA_STATUS
    write_ecg_data(0x00, 0x01); // start conversion

    while (1) {
        if (gpio_get_level(ECG_DRDB_PIN) == 1) {
            ecg_read_sample(&status, &ch1_raw, &ch2_raw);

            ecg_sample_t sample;
            sample.timestamp_us = esp_timer_get_time();
            sample.data_status = status;
            sample.ch1 = ch1_raw;
            sample.ch2 = ch2_raw;
            sample.ch3 = sample.ch1 - sample.ch2;

            if (xQueueSend(ecg_sample_queue, &sample, 0) != pdTRUE) {
                return;
            }
        }
    }
}

void ecg_power_down() {
    if (!INCLUDE_ECG) {
        return;
    }
    
    ESP_ERROR_CHECK(spi_bus_remove_device(ecg_handle));
    ESP_ERROR_CHECK(spi_bus_free(ecg_host_device));
}