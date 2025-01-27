/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

#include "esp_dmx.h"

static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

static uint8_t s_led_state = 0;

#ifdef CONFIG_BLINK_LED_STRIP

static led_strip_handle_t led_strip;

static void blink_led(void)
{
    /* If the addressable LED is enabled */
    if (s_led_state) {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        led_strip_set_pixel(led_strip, 0, 16, 16, 16);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
    } else {
        /* Set all LED off to clear all pixels */
        led_strip_clear(led_strip);
    }
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };
#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

#elif CONFIG_BLINK_LED_GPIO

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

#else
#error "unsupported LED type"
#endif




void app_main(void)
{

    /* Configure the peripheral according to the LED type */
    configure_led();

    const dmx_port_t dmx_num = DMX_NUM_1;

    // First, use the default DMX configuration...
    dmx_config_t config = DMX_CONFIG_DEFAULT;

    // ...declare the driver's DMX personalities...
    const int personality_count = 1;
    dmx_personality_t personalities[] = {
    {1, "Default Personality"}
    };

    // ...install the DMX driver...
    dmx_driver_install(dmx_num, &config, personalities, personality_count);

    // ...and then set the communication pins!
    const int tx_pin = 14;
    const int rx_pin = 17;
    const int rts_pin = 16;
    dmx_set_pin(dmx_num, tx_pin, rx_pin, rts_pin);

    
    uint8_t data[DMX_PACKET_SIZE];

    dmx_packet_t packet;

    while (1) {
        if (dmx_receive(DMX_NUM_1, &packet, DMX_TIMEOUT_TICK)) {

            // Check that no errors occurred.
            if (packet.err == DMX_OK) {
                ESP_LOGI(TAG, "%i", dmx_read_slot(DMX_NUM_1, 1));

                if (dmx_read_slot(DMX_NUM_1, 1) > 0) {
                    s_led_state = true;
                } else {
                    s_led_state = false;
                }
                blink_led();
            } else {
                ESP_LOGE(TAG,"An error occurred receiving DMX!");
            }

        } else {
            ESP_LOGW(TAG,"Timed out waiting for DMX.");
        }
        /*
        ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        blink_led();
        
        s_led_state = !s_led_state;
        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);

        */
    }
}
