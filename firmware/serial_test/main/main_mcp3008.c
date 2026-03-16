/*
 * MCP3008 variant — reads 20 analog channels from 3 MCP3008 ADCs over SPI.
 * Sends 20 generic control floats per packet. Packet indices are channel
 * indices, not robot-joint semantics.
 *
 * Chip 0 ch 0-7  => packet floats[0..7]
 * Chip 1 ch 0-7  => packet floats[8..15]
 * Chip 2 ch 0-7  => packet floats[16..23]
 *
 * Packet: [0xAA 0x55] [uint16 seq] [24x float32 LE] [uint16 CRC16-CCITT]
 * Total: 102 bytes
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"

// Temporary diagnostic mode: send per-channel voltages instead of centered output.
#define DEBUG_RAW       1

// ===== SPI PIN ASSIGNMENTS (XIAO ESP32-S3) =====
// XIAO ESP32-S3 pin labels do NOT match GPIO numbers:
// D8=GPIO7 (SCK), D9=GPIO8 (MISO), D10=GPIO9 (MOSI)
#define PIN_SPI_MISO    GPIO_NUM_8
#define PIN_SPI_MOSI    GPIO_NUM_9
#define PIN_SPI_SCLK    GPIO_NUM_7
#define PIN_CS_CHIP0    GPIO_NUM_1
#define PIN_CS_CHIP1    GPIO_NUM_2
#define PIN_CS_CHIP2    GPIO_NUM_3

// ===== CONFIGURATION =====
#define SPI_HOST_ID     SPI2_HOST
#define SPI_CLOCK_HZ    100000      // 100 kHz — slow for breadboard debugging

#define NUM_CHIPS       3
#define CHANNELS_CHIP0  8
#define CHANNELS_CHIP1  8
#define CHANNELS_CHIP2  8
#define TOTAL_CHANNELS  (CHANNELS_CHIP0 + CHANNELS_CHIP1 + CHANNELS_CHIP2)

#define ADC_MAX         1023
#define ADC_VREF_VOLTS  3.3f
#define DEADZONE        5
#define SMOOTHING       0.05f
#define OUTPUT_MAX      1.5f

#define SEND_RATE_HZ    100

#define HEADER_0        0xAA
#define HEADER_1        0x55
#define FLOAT_COUNT     24
#define USB_WRITE_TIMEOUT_MS    2
#define STARTUP_BEACON_REPEATS  20
#define STARTUP_BEACON_DELAY_MS 50

static const char *TAG = "mcp3008";

static int adc_center[TOTAL_CHANNELS];
static float smooth_adc[TOTAL_CHANNELS];
static uint16_t packet_sequence = 0;
static bool cdc_connected = false;

static spi_device_handle_t spi_dev[NUM_CHIPS];

static const int channels_per_chip[NUM_CHIPS] = {
    CHANNELS_CHIP0, CHANNELS_CHIP1, CHANNELS_CHIP2
};

typedef struct __attribute__((packed)) {
    uint8_t header[2];
    uint16_t sequence;
    float values[FLOAT_COUNT];
    uint16_t crc;
} packet_t;

// ===== MCP3008 SPI READ =====

static int mcp3008_read(spi_device_handle_t dev, int channel)
{
    /*
     * MCP3008 3-byte transaction (single-ended):
     *   TX: [0x01, 0x80 | (ch<<4), 0x00]
     *   RX: byte 1 bits[1:0] = MSBs, byte 2 = lower 8 bits
     */
    uint8_t tx[3] = { 0x01, (uint8_t)(0x80 | (channel << 4)), 0x00 };
    uint8_t rx[3] = { 0 };

    spi_transaction_t t = {
        .length = 24,          // 3 bytes
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    esp_err_t err = spi_device_transmit(dev, &t);
    if (err != ESP_OK) {
        return -1;
    }

    return ((rx[1] & 0x03) << 8) | rx[2];
}

// Debug version: returns parsed value AND full rx bytes
static int mcp3008_read_debug(spi_device_handle_t dev, int channel,
                               uint8_t *rx_out)
{
    uint8_t tx[3] = { 0x01, (uint8_t)(0x80 | (channel << 4)), 0x00 };
    uint8_t rx[3] = { 0 };

    spi_transaction_t t = {
        .length = 24,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    esp_err_t err = spi_device_transmit(dev, &t);
    if (err != ESP_OK) {
        return -1;
    }

    if (rx_out) {
        rx_out[0] = rx[0];
        rx_out[1] = rx[1];
        rx_out[2] = rx[2];
    }
    return ((rx[1] & 0x03) << 8) | rx[2];
}

// ===== CRC =====

static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
        }
    }

    return crc;
}

// ===== ADC-to-output conversion =====

static float adc_to_output(int adc_val, int center)
{
    int offset = adc_val - center;

    if (abs(offset) < DEADZONE) {
        offset = 0;
    }

    float normalized = (float)offset / (float)(ADC_MAX / 2);
    if (normalized > 1.0f) normalized = 1.0f;
    if (normalized < -1.0f) normalized = -1.0f;

    return normalized * OUTPUT_MAX;
}

// ===== PACKET =====

static void build_packet(packet_t *pkt, const float vals[FLOAT_COUNT])
{
    pkt->header[0] = HEADER_0;
    pkt->header[1] = HEADER_1;
    pkt->sequence = packet_sequence++;
    memcpy(pkt->values, vals, sizeof(pkt->values));
    pkt->crc = crc16_ccitt((const uint8_t *)&pkt->sequence,
                            sizeof(pkt->sequence) + sizeof(pkt->values));
}

// ===== USB CDC =====

static void cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    (void)itf;
    cdc_connected = event->line_state_changed_data.dtr;
}

static void init_usb_cdc(void)
{
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
        .configuration_descriptor = NULL,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 64,
        .callback_rx = NULL,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = &cdc_line_state_changed_callback,
        .callback_line_coding_changed = NULL,
    };
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
}

static void cdc_write_bytes(const uint8_t *data, size_t len)
{
    if (tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, data, len) == ESP_OK) {
        tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, pdMS_TO_TICKS(USB_WRITE_TIMEOUT_MS));
    }
}

static void send_startup_beacon(void)
{
    static const uint8_t beacon[] = {
        'B', 'E', 'A', 'C', 'O', 'N',
        0x0D, 0x0A
    };

    for (int i = 0; i < STARTUP_BEACON_REPEATS; i++) {
        tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, beacon, sizeof(beacon));
        tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, pdMS_TO_TICKS(USB_WRITE_TIMEOUT_MS));
        vTaskDelay(pdMS_TO_TICKS(STARTUP_BEACON_DELAY_MS));
    }
}

// ===== SPI INIT =====

static void init_spi(void)
{
    spi_bus_config_t bus_cfg = {
        .miso_io_num = PIN_SPI_MISO,
        .mosi_io_num = PIN_SPI_MOSI,
        .sclk_io_num = PIN_SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST_ID, &bus_cfg, SPI_DMA_DISABLED));

    const int cs_pins[NUM_CHIPS] = { PIN_CS_CHIP0, PIN_CS_CHIP1, PIN_CS_CHIP2 };

    for (int i = 0; i < NUM_CHIPS; i++) {
        spi_device_interface_config_t dev_cfg = {
            .clock_speed_hz = SPI_CLOCK_HZ,
            .mode = 0,                     // MCP3008: CPOL=0, CPHA=0
            .spics_io_num = cs_pins[i],
            .queue_size = 1,
        };
        ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST_ID, &dev_cfg, &spi_dev[i]));
    }

    ESP_LOGI(TAG, "SPI bus initialized — %d MCP3008 device(s)", NUM_CHIPS);
}

// ===== CHANNEL READING =====

/*
 * Fail-closed: if any SPI read fails, return false and leave out[] unchanged.
 * The caller must skip packet transmit for that frame so a transient bus
 * fault never injects a false sample into the smoothing filter.
 */
static bool read_all_channels(int out[TOTAL_CHANNELS])
{
    int idx = 0;
    for (int chip = 0; chip < NUM_CHIPS; chip++) {
        for (int ch = 0; ch < channels_per_chip[chip]; ch++) {
            int val = mcp3008_read(spi_dev[chip], ch);
            if (val < 0) {
                return false;
            }
            out[idx++] = val;
        }
    }
    return true;
}

// Returns true on success. On failure the control loop must not run.
static bool calibrate(void)
{
    ESP_LOGI(TAG, "Calibrating %d channels — keep inputs at rest", TOTAL_CHANNELS);
    vTaskDelay(pdMS_TO_TICKS(500));

    long sum[TOTAL_CHANNELS];
    memset(sum, 0, sizeof(sum));
    const int target_samples = 100;
    const int max_attempts = target_samples * 2;
    int raw[TOTAL_CHANNELS];
    int good_samples = 0;

    // Fail-closed: skip samples where any channel read fails.
    for (int attempt = 0; attempt < max_attempts && good_samples < target_samples; attempt++) {
        if (read_all_channels(raw)) {
            for (int i = 0; i < TOTAL_CHANNELS; i++) {
                sum[i] += raw[i];
            }
            good_samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (good_samples == 0) {
        ESP_LOGE(TAG, "Calibration failed — no successful SPI reads");
        return false;
    }

    for (int i = 0; i < TOTAL_CHANNELS; i++) {
        adc_center[i] = (int)(sum[i] / good_samples);
    }

    ESP_LOGI(TAG, "Calibration done (%d/%d samples, centers: %d %d %d %d)",
             good_samples, target_samples,
             adc_center[0], adc_center[1], adc_center[2], adc_center[3]);
    return true;
}

// ===== MAIN =====

void app_main(void)
{
    init_usb_cdc();
    send_startup_beacon();

#if DEBUG_RAW
    init_spi();
    send_startup_beacon();
    esp_log_level_set("*", ESP_LOG_NONE);

    const TickType_t period = pdMS_TO_TICKS(1000 / SEND_RATE_HZ);
    TickType_t next_wake = xTaskGetTickCount();

    while (1) {
        float vals[FLOAT_COUNT] = { 0.0f };
        int raw[TOTAL_CHANNELS];

        if (read_all_channels(raw)) {
            for (int i = 0; i < TOTAL_CHANNELS; i++) {
                vals[i] = ((float)raw[i] / (float)ADC_MAX) * ADC_VREF_VOLTS;
            }
        }

        packet_t pkt = { 0 };
        build_packet(&pkt, vals);

        if (cdc_connected) {
            cdc_write_bytes((const uint8_t *)&pkt, sizeof(pkt));
        }

        vTaskDelayUntil(&next_wake, period);
    }
#else
    init_spi();
    // Fail-closed: block here until calibration succeeds. Never enter the
    // control loop with bogus centers — that would command saturated outputs.
    while (!calibrate()) {
        ESP_LOGE(TAG, "Retrying calibration in 1 s...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    send_startup_beacon();

    esp_log_level_set("*", ESP_LOG_NONE);

    const TickType_t period = pdMS_TO_TICKS(1000 / SEND_RATE_HZ);
    bool first_read = true;
    TickType_t next_wake = xTaskGetTickCount();
    int raw[TOTAL_CHANNELS];

    while (1) {
        // Fail-closed: skip entire frame if any channel read fails
        if (!read_all_channels(raw)) {
            vTaskDelayUntil(&next_wake, period);
            continue;
        }

        // Low-pass smoothing
        if (first_read) {
            for (int i = 0; i < TOTAL_CHANNELS; i++) {
                smooth_adc[i] = (float)raw[i];
            }
            first_read = false;
        } else {
            for (int i = 0; i < TOTAL_CHANNELS; i++) {
                smooth_adc[i] = SMOOTHING * raw[i]
                              + (1.0f - SMOOTHING) * smooth_adc[i];
            }
        }

        // Convert to output values
        float vals[FLOAT_COUNT] = { 0.0f };
        for (int i = 0; i < TOTAL_CHANNELS; i++) {
            vals[i] = adc_to_output((int)smooth_adc[i], adc_center[i]);
        }
        // vals[TOTAL_CHANNELS .. FLOAT_COUNT-1] remain 0.0 (spare)

        packet_t pkt = { 0 };
        build_packet(&pkt, vals);

        if (cdc_connected) {
            cdc_write_bytes((const uint8_t *)&pkt, sizeof(pkt));
        }

        vTaskDelayUntil(&next_wake, period);
    }
#endif
}
