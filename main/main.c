#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "led_strip.h"

#define CR 0x0D
#define LF 0x0A
#define RGB_LED_GPIO 48
#define LED_STRIP_LED_NUM 1
#define wait_time_out 5000

led_strip_handle_t led_strip;
const uint8_t uart_num = UART_NUM_0;
const char IDRequestMsg[] = {'/', '?', '!', CR, LF};
const char IDResponse_expicted[10] = {'/', 'M', '0', '4', '1', '5', '2', '2', CR, LF}; // hardcoded response
const char BreakCond[] = {01, 'B', 03, 'A'};

void led_blink(void);
void uart_setup(void);
void led_setup(void);
void recive_uart_data(void);
void GW_Bypass();
void GW_TxRx_En();
void GW_Tx(const char *message, size_t message_length);
void read_uart_data(void);

// UART configuration
void uart_setup(void)
{
    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (1024 * 2);
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 0, NULL, 0));

    uart_config_t uart_config = {
        .baud_rate = 2400,
        .data_bits = UART_DATA_7_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
    ESP_ERROR_CHECK(uart_set_pin(uart_num, 43, 44, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

// LED strip configuration
void led_setup(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = RGB_LED_GPIO,
        .max_leds = LED_STRIP_LED_NUM,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .mem_block_symbols = 64,
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(
        led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
}
void led_blink(void)
{
    // WHITE
    led_strip_set_pixel(led_strip, 0, 255, 255, 255);
    led_strip_refresh(led_strip);
    vTaskDelay(pdMS_TO_TICKS(100));

    // OFF
    led_strip_clear(led_strip);
}

void GW_TxRx_En()
{
    gpio_set_level(GPIO_NUM_10, 1);  // dummy load enable pin
    gpio_set_level(GPIO_NUM_12, 1);  // Bus controlled by GW TX only
    gpio_set_level(GPIO_NUM_36, 0);  // GW Rx bypass
    gpio_set_level(RGB_LED_GPIO, 0); // GW bypass
}

void GW_Bypass(void)
{
    gpio_set_level(GPIO_NUM_10, 0);  // dummy load disable
    gpio_set_level(GPIO_NUM_36, 1);  // GW Rx bypass
    gpio_set_level(RGB_LED_GPIO, 1); // GW bypass
}
void GW_Tx(const char *message, size_t message_length)
{
    gpio_set_level(GPIO_NUM_12, 0); // Bus controlled by GW TX only
    vTaskDelay(pdMS_TO_TICKS(20));  // delay before sending
    uart_write_bytes(uart_num, (const char *)message, message_length);
    vTaskDelay(pdMS_TO_TICKS(60));  // delay after the send (it can be moved to the Tx routine, after finish return GPIO_12 to high)
    gpio_set_level(GPIO_NUM_12, 1); // Bus Controlled by Meter Tx only
}

void read_uart_data(void)
{
    // Read data from UART.
    uint8_t UART_DATA[128];
    int length = uart_read_bytes(uart_num, UART_DATA, sizeof(UART_DATA), pdMS_TO_TICKS(wait_time_out));

    if (length > 0)
    {
        if (strncmp((const char *)UART_DATA, (const char *)IDResponse_expicted, length) == 0)
        {
            led_blink();
        }
    }
}

void app_main(void)
{
    gpio_set_direction(GPIO_NUM_10, GPIO_MODE_OUTPUT);  // dummy load control pin
    gpio_set_direction(GPIO_NUM_12, GPIO_MODE_OUTPUT);  // Bus control pin
    gpio_set_direction(GPIO_NUM_36, GPIO_MODE_OUTPUT);  // GW Rx bypass control pin
    gpio_set_direction(RGB_LED_GPIO, GPIO_MODE_OUTPUT); // GW bypass control pin

    uart_setup();
    led_setup();

    while (1)
    {
        GW_TxRx_En();
        GW_Tx(BreakCond, sizeof(BreakCond));
        vTaskDelay(pdMS_TO_TICKS(2000));
        GW_Tx(BreakCond, sizeof(BreakCond));
        vTaskDelay(pdMS_TO_TICKS(2000));
        GW_Tx(IDRequestMsg, sizeof(IDRequestMsg));
        read_uart_data();
        vTaskDelay(pdMS_TO_TICKS(2000));
        GW_Bypass();
        vTaskDelay(pdMS_TO_TICKS(300000));
    }
}
