//mcp3008 test




//xiao esp32-s3 pin labels dont match gpio numbers (off by one for d8-d10)

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "driver/spi_master.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"

//xiao d8=gpio7, d9=gpio8, d10=gpio9 (NOT d8=gpio8)
#define SCLK  GPIO_NUM_7
#define MISO  GPIO_NUM_8
#define MOSI  GPIO_NUM_9
#define CS    GPIO_NUM_1 //xiao a0

static spi_device_handle_t spi;

//reads one channel from the mcp3008 over spi

//mcp3008 expects 3 bytes sent and gives 3 bytes back:
//mcp3008 is full duplex -> for every byte recieved get a byte back

//tx[0] = 0x01 start bit

//tx[1] = 0x80 (ch0)
//msb = 1 sets single-ended mode (return v rel to gnd)

//(ch << 4) chooses which channel (0-7) to read
//number in bits 6-4 of this byte so shift left 4
//ch0 = 0x80 -> 1[000] 0000
//ch1 = 0x90 -> 1[001] 0000
//ch2 = 0xA0 -> 1[010] 0000
//ex...

//tx[2] = 0x00 just padding so we can recieve the 3rd byte back

//response is the 10 bit adc value (0-1023) split across rx[1] and rx[2]:

//rx[0] = garbage, chip is still processing

//rx[1] = bottom 2 bits are bits 9-8 of the result

//rx[2] = bits 7-0 of the result
//mask rx[1] with 0x03 to get those 2 bits, shift up 8,
//OR rx[2] to get the full 10 bit number

//returns value from rx
static int mcp3008_read(int ch)
{
    uint8_t tx[3] = {0x01, 0x80 | (ch << 4), 0x00};
    uint8_t rx[3] = {0};

    spi_transaction_t t = {.length = 24, .tx_buffer = tx, .rx_buffer = rx}; //espi idf messenger command
    if (spi_device_transmit(spi, &t) != ESP_OK) return -1;
    return ((rx[1] & 0x03) << 8) | rx[2];
}

//idk cdc stuff
static void cdc_print(const char *s)
{
    if (tinyusb_cdcacm_write_queue(0, (const uint8_t *)s, strlen(s)) == ESP_OK)
        tinyusb_cdcacm_write_flush(0, pdMS_TO_TICKS(10));
}

void app_main(void) //called as a freetrtos task for esp
{
    //usb cdc init !!!!!!!1 idk
    tinyusb_config_t tusb_cfg = {0};
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    tinyusb_config_cdcacm_t acm_cfg = {0};
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));

    //spi bus + device init, 100khz is slow but safe for breadboard
    spi_bus_config_t bus = {
        .miso_io_num = MISO, .mosi_io_num = MOSI, .sclk_io_num = SCLK,
        .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 32,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED));
    spi_device_interface_config_t dev = {
        .clock_speed_hz = 100000, .mode = 0, .spics_io_num = CS, .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev, &spi));

    vTaskDelay(pdMS_TO_TICKS(1000)); //let usb enumerate

    //read all 8 channels, print 10bit values at 10hz
    char line[128];
    while (1) {
        int raw[8];
        for (int ch = 0; ch < 8; ch++)
            raw[ch] = mcp3008_read(ch);

        snprintf(line, sizeof(line),
            "%4d %4d %4d %4d %4d %4d %4d %4d\r\n",
            raw[0], raw[1], raw[2], raw[3],
            raw[4], raw[5], raw[6], raw[7]);
        cdc_print(line);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
