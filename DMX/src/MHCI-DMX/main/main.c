#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

#include "driver/ledc.h"
#include "math.h"
#include "esp_dmx.h"
#include "rdm/responder.h"
#include "esp_timer.h"

static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (1000) // Frequency in Hertz. Set frequency at 4 kHz

#define BLINK_GPIO 38

static uint8_t s_led_state = 0;

static led_strip_handle_t led_strip;

// PWM Configuration
const uint8_t pwmChannels[6] = {LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3, LEDC_CHANNEL_4, LEDC_CHANNEL_5};
const uint8_t pwmResolution = 16; // PWM resolution of 16 bits
const uint16_t pwmThreshold = 0;  // Low Value Threshold for PWM output

// PWM frequencies for each personality
const uint16_t pwmFreqPersonality[6] = {1000, 500, 60, 1000, 500, 60};

// Pins
const uint8_t builtInLedPin = 13;
const uint8_t outputPins[6] = {11, 10, 9, 8, 7, 6};
const uint8_t enablePin = 16;
const uint8_t transmitPin = 14;
const uint8_t receivePin = 17;

// DMX Configuration
const dmx_port_t dmxPort = DMX_NUM_1;
uint16_t dmxAddress;

uint8_t dmxData[DMX_PACKET_SIZE];

dmx_packet_t packet;

// DMX Connectivity and Timing
bool dmxIsConnected = false;
unsigned long lastUpdate = 0;
bool dmxAddressChanged = false;
// Variable to store the current personality
uint8_t currentPersonality;
// PWM frequency variable
uint16_t pwmFreq;

// S-curve Parameters for Channel 1
const float midpointBrightness1 = 8441.0f; // PWM output at DMX value x0_1 (midpoint brightness)
const float x0_1 = 32768.0f;               // Midpoint of the DMX input range (65535 / 2)
const float k1_1 = 0.0002f;                // Steepness for the lower S-curve
const float s0_1 = 6.0f;///1.0f / (1.0f + expf(k1_1 * x0_1)); // Sigmoid at input = 0
const float s1_1 = 0.5f;                                // Sigmoid at input = x0

// S-curve Parameters for Channel 2
const float midpointBrightness2 = 8441.0f; // PWM output at DMX value x0_2 (midpoint brightness)
const float x0_2 = 32768.0f;               // Midpoint of the DMX input range (65535 / 2)
const float k1_2 = 0.0002f;                // Steepness for the lower S-curve
const float s0_2 = 6.0f;//1.0f / (1.0f + expf(k1_2 * x0_2)); // Sigmoid at input = 0
const float s1_2 = 0.5f;                                // Sigmoid at input = x0

// Function to map DMX value to PWM value using an S-curve (sigmoid function)
uint16_t computeSCurve(uint16_t input, float B, float x0, float k1, float s0, float s1) {
    if (input <= x0) {
        // Lower S-curve
        float s = 1.0f / (1.0f + expf(-k1 * (input - x0))); // Sigmoid at current input
        float sNormalized = (s - s0) / (s1 - s0);           // Normalize s to [0, 1]
        float output = B * sNormalized;                     // Scale output from 0 to B
        return (uint16_t)output;
    } else {
        // Upper linear curve
        float u = (input - x0) / (65535.0f - x0);           // Normalize input to [0, 1]
        float output = B + (65535.0f - B) * u;              // Linear interpolation from B to 65535
        return (uint16_t)output;
    }
}

// Function to get PWM frequency based on personality
uint16_t getPWMFrequencyForPersonality(const uint8_t personality) {
    return pwmFreqPersonality[personality - 1];
}

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
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

void configure_dmx() {
    
    // Install the DMX driver
    dmx_config_t config = {
        .interrupt_flags = DMX_INTR_FLAGS_DEFAULT,
        .root_device_parameter_count = 32,
        .sub_device_parameter_count = 0,
        .model_id = 1,
        .product_category = RDM_PRODUCT_CATEGORY_FIXTURE,
        .software_version_id = ESP_DMX_VERSION_ID,
        .software_version_label = ESP_DMX_VERSION_LABEL,
        .queue_size_max = 0, // default 32 but 0 is smoother dimming
    };
    dmx_personality_t personalities[] = {
        {4, "16Bit sCurve 1000Hz          "},
        {4, "16Bit sCurve 500Hz          "},
        {4, "16Bit sCurve 60Hz          "},
        {4, "16Bit Linear 1000Hz          "},
        {4, "16Bit Linear 500Hz          "},
        {4, "16Bit Linear 60Hz          "}
    };
    dmx_driver_install(dmxPort, &config, personalities, 6);
    dmx_set_pin(dmxPort, transmitPin, receivePin, enablePin);

    // Get the initial DMX start address and personality
    dmxAddress = dmx_get_start_address(dmxPort);
    currentPersonality = dmx_get_current_personality(dmxPort);
}

void configure_pwm() {
    // Set PWM frequency based on initial personality
    //pwmFreq = getPWMFrequencyForPersonality(currentPersonality);



    // Set up the PWM channels for both LEDs
    //ledcSetup(pwmChannels[0], pwmFreq, pwmResolution);
    //ledcAttachPin(outputPins[0], pwmChannels[0]);
    //ledcAttachPin(builtInLedPin, pwmChannels[0]);

    //ledcSetup(pwmChannels[1], pwmFreq, pwmResolution);
    //ledcAttachPin(outputPins[1], pwmChannels[1]);


        // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = 11,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void setup() {
    /* Configure the peripheral according to the LED type */
    configure_led();

    /* Configure PWM pins */
    configure_pwm();

    /* Configure DMX peripheral */
    configure_dmx();


    #if DEBUG
    ESP_LOGI(TAG, "Initial DMX Start Address: %d\n", dmxAddress);
    ESP_LOGI("Initial Personality: %d\n", currentPersonality);
    ESP_LOGI("Initial PWM frequency: %d Hz\n", ledcReadFreq(pwmChannels[0]));
    #endif

    lastUpdate = esp_timer_get_time() / 1000;
}

void loop() {
    
    dmx_packet_t packet;

    if (dmx_receive(dmxPort, &packet, DMX_TIMEOUT_TICK)) {
        if (packet.is_rdm && !packet.err) {
            // Handle RDM packet
            rdm_send_response(dmxPort);
            ESP_LOGD(TAG,"RDM packet received and response sent.");

            // Indicate that the DMX address or personality may have changed
            dmxAddressChanged = true;
        } else if (!packet.err) {
            // Handle DMX packet
            if (!dmxIsConnected) {
                ESP_LOGI(TAG, "DMX is connected!");
                dmxIsConnected = true;
            }

            // Update the DMX start address and personality if they have changed
            if (dmxAddressChanged) {
                uint16_t newDmxAddress = dmx_get_start_address(dmxPort);
                if (newDmxAddress != dmxAddress) {
                    dmxAddress = newDmxAddress;
                    ESP_LOGI(TAG, "DMX Start Address updated to %d\n", dmxAddress);
                }

                uint8_t newPersonality = dmx_get_current_personality(dmxPort);
                if (newPersonality != currentPersonality) {
                    currentPersonality = newPersonality;
                    ESP_LOGI(TAG, "Personality changed to %d\n", currentPersonality);

                    // Get the new PWM frequency based on personality
                    uint16_t newPwmFreq = getPWMFrequencyForPersonality(currentPersonality);
                    pwmFreq = newPwmFreq;
                    //ledcSetup(pwmChannels[0], pwmFreq, pwmResolution);
                    //ledcSetup(pwmChannels[1], pwmFreq, pwmResolution);

                    // Use ledcReadFreq to display the updated PWM frequency
                    //ESP_LOGI(TAG, "PWM frequency set to %d Hz\n", ledcReadFreq(pwmChannels[0]));
                }

                dmxAddressChanged = false;
            }

            dmx_read(dmxPort, dmxData, packet.size);

            int index = dmxAddress;
            if (index + 3 < DMX_PACKET_SIZE) {

                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, dmxData[index]);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);

                /*

                // Read DMX values for Channel 1
                uint16_t rawBrightness1 = ((uint16_t)dmxData[index] << 8) | dmxData[index + 1];
                // Read DMX values for Channel 2
                uint16_t rawBrightness2 = ((uint16_t)dmxData[index + 2] << 8) | dmxData[index + 3];

                uint16_t scaledBrightness1 = 0;
                uint16_t scaledBrightness2 = 0;

                if (currentPersonality == 1 || currentPersonality == 2 || currentPersonality == 3) {
                    // Apply S-curve
                    scaledBrightness1 = computeSCurve(rawBrightness1, midpointBrightness1, x0_1, k1_1, s0_1, s1_1);
                    scaledBrightness2 = computeSCurve(rawBrightness2, midpointBrightness2, x0_2, k1_2, s0_2, s1_2);
                } else if (currentPersonality == 4 || currentPersonality == 5 || currentPersonality == 6) {
                    // Use raw DMX values directly
                    scaledBrightness1 = rawBrightness1;
                    scaledBrightness2 = rawBrightness2;
                }

                // Apply low value threshold
                if (scaledBrightness1 < pwmThreshold) {
                    scaledBrightness1 = 0;
                }
                if (scaledBrightness2 < pwmThreshold) {
                    scaledBrightness2 = 0;
                }

                */

                // Write to PWM outputs
                //ledcWrite(pwmChannels[0], scaledBrightness1);
                //ledcWrite(pwmChannels[1], scaledBrightness2);

                unsigned long now = esp_timer_get_time() / 1000;
                if (now - lastUpdate > 750) {
                    ESP_LOGI(TAG, "Startcode: %d, Address: %d\n", dmxData[0], dmxAddress);
                    ESP_LOGI(TAG, "Personality: %d\n", currentPersonality);
                    // Use ledcReadFreq to display the current PWM frequency
                    //ESP_LOGI(TAG, "PWM Frequency: %d Hz\n", ledcReadFreq(pwmChannels[0]));
                    //ESP_LOGI(TAG, "Channel 1 - DMX: %d, PWM: %d\n", rawBrightness1, scaledBrightness1);
                    //ESP_LOGI(TAG, "Channel 2 - DMX: %d, PWM: %d\n", rawBrightness2, scaledBrightness2);
                    lastUpdate = now;
                }
            } else {
                ESP_LOGI(TAG, "DMX Address out of range.");
            }
        } else {
            ESP_LOGI(TAG, "A DMX error occurred.");
        }
    } else if (dmxIsConnected) {
        ESP_LOGI(TAG, "DMX was disconnected.");
        dmxIsConnected = false;
    }
}


void app_main(void)
{
    setup();

    while (1) {
        loop();
    }

    

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
