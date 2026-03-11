/*
 * G1 Arm Teleop - XIAO ESP32-S3 Firmware (USB Serial)
 *
 * Reads 3 potentiometers on GPIO1 (A0), GPIO2 (A1), GPIO3 (A2)
 * and sends joint angle data over USB serial.
 *
 * Pot mapping:
 *   A0 (GPIO1) -> Left shoulder pitch  (joint 0)
 *   A1 (GPIO2) -> Left shoulder roll   (joint 1)
 *   A2 (GPIO3) -> Left elbow           (joint 3)
 *
 * Serial protocol:
 *   Sends 14 little-endian floats (56 bytes) prefixed by 2-byte header 0xAA 0x55
 *   Total packet: 58 bytes
 *   7 left arm + 7 right arm joint angles in radians
 *
 * Hardware:
 *   XIAO ESP32-S3
 *   3x rotary potentiometers (10k recommended)
 *     Pot wiper -> A0, A1, A2
 *     Pot ends  -> 3.3V and GND
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/usb_serial_jtag.h"

// ===== CONFIGURATION =====
#define SEND_RATE_HZ    50

// XIAO ESP32-S3 ADC pins
#define POT_0_CHANNEL   ADC_CHANNEL_0   // GPIO1 (A0)
#define POT_1_CHANNEL   ADC_CHANNEL_1   // GPIO2 (A1)
#define POT_2_CHANNEL   ADC_CHANNEL_2   // GPIO3 (A2)

#define ADC_MAX         4095
#define DEADZONE        20
#define SMOOTHING       0.15f
#define JOINT_MAX       1.5f    // radians

// Packet header
#define HEADER_0        0xAA
#define HEADER_1        0x55
#define JOINT_COUNT     14
#define ADC_READ_RETRIES 3
#define USB_WRITE_TIMEOUT_MS 2

static const char *TAG = "teleop";

static int adc_center[3] = {0, 0, 0};
static float smooth_adc[3] = {0.0f, 0.0f, 0.0f};
static uint16_t packet_sequence = 0;

typedef struct __attribute__((packed)) {
    uint8_t header[2];
    uint16_t sequence;
    float joints[JOINT_COUNT];
    uint16_t crc;
} teleop_packet_t;


static float adc_to_radians(int adc_val, int center)
{
    int offset = adc_val - center;

    if (abs(offset) < DEADZONE) {
        offset = 0;
    }

    float normalized = (float)offset / (float)(ADC_MAX / 2);
    if (normalized > 1.0f) normalized = 1.0f;
    if (normalized < -1.0f) normalized = -1.0f;

    return normalized * JOINT_MAX;
}

static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }

    return crc;
}

static esp_err_t read_adc_with_retry(adc_oneshot_unit_handle_t adc_handle, adc_channel_t channel, int *out_value)
{
    esp_err_t err = ESP_FAIL;

    for (int attempt = 0; attempt < ADC_READ_RETRIES; attempt++) {
        err = adc_oneshot_read(adc_handle, channel, out_value);
        if (err == ESP_OK) {
            return ESP_OK;
        }

        vTaskDelay(1);
    }

    return err;
}

static void build_packet(teleop_packet_t *packet, const float joints[JOINT_COUNT])
{
    packet->header[0] = HEADER_0;
    packet->header[1] = HEADER_1;
    packet->sequence = packet_sequence++;
    memcpy(packet->joints, joints, sizeof(packet->joints));
    packet->crc = crc16_ccitt((const uint8_t *)&packet->sequence,
                              sizeof(packet->sequence) + sizeof(packet->joints));
}

static void calibrate(adc_oneshot_unit_handle_t adc_handle)
{
    ESP_LOGI(TAG, "Calibrating... keep pots at rest position");
    vTaskDelay(pdMS_TO_TICKS(500));

    long sum[3] = {0, 0, 0};
    const int samples = 100;
    int val;

    for (int i = 0; i < samples; i++) {
        ESP_ERROR_CHECK(read_adc_with_retry(adc_handle, POT_0_CHANNEL, &val));
        sum[0] += val;
        ESP_ERROR_CHECK(read_adc_with_retry(adc_handle, POT_1_CHANNEL, &val));
        sum[1] += val;
        ESP_ERROR_CHECK(read_adc_with_retry(adc_handle, POT_2_CHANNEL, &val));
        sum[2] += val;
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    adc_center[0] = sum[0] / samples;
    adc_center[1] = sum[1] / samples;
    adc_center[2] = sum[2] / samples;

    ESP_LOGI(TAG, "Calibrated centers: %d, %d, %d",
             adc_center[0], adc_center[1], adc_center[2]);
}


void app_main(void)
{
    // Init USB serial
    usb_serial_jtag_driver_config_t usb_cfg = {
        .tx_buffer_size = 256,
        .rx_buffer_size = 256,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));

    // Init ADC
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t adc_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_config, &adc_handle));

    adc_oneshot_chan_cfg_t chan_config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, POT_0_CHANNEL, &chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, POT_1_CHANNEL, &chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, POT_2_CHANNEL, &chan_config));

    // Calibrate
    calibrate(adc_handle);

    // Stop application logging before binary streaming begins.
    esp_log_level_set("*", ESP_LOG_NONE);

    // Main loop
    const TickType_t period = pdMS_TO_TICKS(1000 / SEND_RATE_HZ);
    bool first_read = true;
    TickType_t next_wake = xTaskGetTickCount();

    while (1) {
        // Read ADC
        int raw[3] = {0};
        if (read_adc_with_retry(adc_handle, POT_0_CHANNEL, &raw[0]) != ESP_OK ||
            read_adc_with_retry(adc_handle, POT_1_CHANNEL, &raw[1]) != ESP_OK ||
            read_adc_with_retry(adc_handle, POT_2_CHANNEL, &raw[2]) != ESP_OK) {
            vTaskDelay(period);
            continue;
        }

        // Smooth
        if (first_read) {
            smooth_adc[0] = raw[0];
            smooth_adc[1] = raw[1];
            smooth_adc[2] = raw[2];
            first_read = false;
        } else {
            for (int i = 0; i < 3; i++) {
                smooth_adc[i] = SMOOTHING * raw[i] + (1.0f - SMOOTHING) * smooth_adc[i];
            }
        }

        // Convert to radians
        float pot_rad[3];
        pot_rad[0] = adc_to_radians((int)smooth_adc[0], adc_center[0]);
        pot_rad[1] = adc_to_radians((int)smooth_adc[1], adc_center[1]);
        pot_rad[2] = adc_to_radians((int)smooth_adc[2], adc_center[2]);

        // Build packet: 14 floats (7 left + 7 right)
        float joints[14] = {0.0f};
        joints[0] = pot_rad[0];    // A0 -> left shoulder pitch
        joints[1] = pot_rad[1];    // A1 -> left shoulder roll
        joints[3] = pot_rad[2];    // A2 -> left elbow

        teleop_packet_t packet = {0};
        build_packet(&packet, joints);

        int written = usb_serial_jtag_write_bytes((const void *)&packet,
                                                  sizeof(packet),
                                                  pdMS_TO_TICKS(USB_WRITE_TIMEOUT_MS));
        if (written < 0) {
            vTaskDelay(1);
        }

        vTaskDelayUntil(&next_wake, period);
    }
}
