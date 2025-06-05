//written using a clone of esp_dmx v4.1.0

#define DEBUG  // Uncomment this line to enable debugging messages

#ifdef DEBUG
#define DEBUG_PRINT(x)    Serial.print(x)
#define DEBUG_PRINTLN(x)  Serial.println(x)
#define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTF(...)
#endif

#include <Arduino.h>
#include <math.h>
#include <esp_dmx.h>
#include <rdm/responder.h>

// PWM Configuration
const uint8_t pwmChannel1 = 1;    // PWM Channel 1
const uint8_t pwmChannel2 = 2;    // PWM Channel 2
const uint8_t pwmResolution = 16; // PWM resolution of 16 bits
const uint16_t pwmThreshold = 0;  // Low Value Threshold for PWM output

// PWM frequencies for each personality
const uint16_t pwmFreqPersonality1 = 1000; // PWM Frequency for Personality 1 
const uint16_t pwmFreqPersonality2 =  500; // PWM Frequency for Personality 2
const uint16_t pwmFreqPersonality3 =   60; // PWM Frequency for Personality 3 
const uint16_t pwmFreqPersonality4 = 1000; // PWM Frequency for Personality 4 
const uint16_t pwmFreqPersonality5 =  500; // PWM Frequency for Personality 5 
const uint16_t pwmFreqPersonality6 =   60; // PWM Frequency for Personality 6 

// Pins
const uint8_t builtInLedPin = BUILTIN_LED;
const uint8_t outputLedPin1 = 27; // Output pin for LED 1 (Sparkfun D2)
const uint8_t outputLedPin2 = 19; // Output pin for LED 2 (Sparkfun D0)
const uint8_t enablePin = ENABLE_PIN;
const uint8_t transmitPin = TRANSMIT_PIN;
const uint8_t receivePin = RECEIVE_PIN;

// DMX Configuration
const dmx_port_t dmxPort = DMX_NUM_1;
uint16_t dmxAddress;
byte dmxData[DMX_PACKET_SIZE];

// DMX Connectivity and Timing
bool dmxIsConnected = false;
unsigned long lastUpdate = 0;
bool dmxAddressChanged = false;

// S-curve Parameters for Channel 1
const float midpointBrightness1 = 8441.0f; // PWM output at DMX value x0_1 (midpoint brightness)
const float x0_1 = 32768.0f;               // Midpoint of the DMX input range (65535 / 2)
const float k1_1 = 0.0002f;                // Steepness for the lower S-curve
const float s0_1 = 1.0f / (1.0f + expf(k1_1 * x0_1)); // Sigmoid at input = 0
const float s1_1 = 0.5f;                                // Sigmoid at input = x0

// S-curve Parameters for Channel 2
const float midpointBrightness2 = 8441.0f; // PWM output at DMX value x0_2 (midpoint brightness)
const float x0_2 = 32768.0f;               // Midpoint of the DMX input range (65535 / 2)
const float k1_2 = 0.0002f;                // Steepness for the lower S-curve
const float s0_2 = 1.0f / (1.0f + expf(k1_2 * x0_2)); // Sigmoid at input = 0
const float s1_2 = 0.5f;                                // Sigmoid at input = x0

// Variable to store the current personality
uint8_t currentPersonality;

// PWM frequency variable
uint16_t pwmFreq;

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
uint16_t getPWMFrequencyForPersonality(uint8_t personality) {
    switch(personality) {
        case 1:
            return pwmFreqPersonality1;
        case 2:
            return pwmFreqPersonality2;
        case 3:
            return pwmFreqPersonality3;
        case 4:
            return pwmFreqPersonality4;  
        case 5:
            return pwmFreqPersonality5;
        case 6:
            return pwmFreqPersonality6;          
    }
}

void nukeDriver(){
    dmx_driver_delete(dmxPort);
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

void setup() {
    #ifdef DEBUG
    Serial.begin(921600);
    #endif

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

    // Set PWM frequency based on initial personality
    pwmFreq = getPWMFrequencyForPersonality(currentPersonality);

    // Set up the PWM channels for both LEDs
    ledcSetup(pwmChannel1, pwmFreq, pwmResolution);
    ledcAttachPin(outputLedPin1, pwmChannel1);
    ledcAttachPin(builtInLedPin, pwmChannel1);

    ledcSetup(pwmChannel2, pwmFreq, pwmResolution);
    ledcAttachPin(outputLedPin2, pwmChannel2);

    // Now we can read the actual PWM frequency
    DEBUG_PRINTF("Initial DMX Start Address: %d\n", dmxAddress);
    DEBUG_PRINTF("Initial Personality: %d\n", currentPersonality);
    DEBUG_PRINTF("Initial PWM frequency: %d Hz\n", ledcReadFreq(pwmChannel1));

    lastUpdate = millis();
}

void loop() {
    dmx_packet_t packet;

    if (dmx_receive(dmxPort, &packet, DMX_TIMEOUT_TICK)) {
        if (packet.is_rdm && !packet.err) {
            // Handle RDM packet
            rdm_send_response(dmxPort);
            DEBUG_PRINTLN("RDM packet received and response sent.");

            // Indicate that the DMX address or personality may have changed
            dmxAddressChanged = true;
        } else if (!packet.err) {
            // Handle DMX packet
            if (!dmxIsConnected) {
                DEBUG_PRINTLN("DMX is connected!");
                dmxIsConnected = true;
            }

            // Update the DMX start address and personality if they have changed
            if (dmxAddressChanged) {
                uint16_t newDmxAddress = dmx_get_start_address(dmxPort);
                if (newDmxAddress != dmxAddress) {
                    dmxAddress = newDmxAddress;
                    DEBUG_PRINTF("DMX Start Address updated to %d\n", dmxAddress);
                }

                uint8_t newPersonality = dmx_get_current_personality(dmxPort);
                if (newPersonality != currentPersonality) {
                    currentPersonality = newPersonality;
                    DEBUG_PRINTF("Personality changed to %d\n", currentPersonality);

                    // Get the new PWM frequency based on personality
                    uint16_t newPwmFreq = getPWMFrequencyForPersonality(currentPersonality);
                    pwmFreq = newPwmFreq;
                    ledcSetup(pwmChannel1, pwmFreq, pwmResolution);
                    ledcSetup(pwmChannel2, pwmFreq, pwmResolution);

                    // Use ledcReadFreq to display the updated PWM frequency
                    DEBUG_PRINTF("PWM frequency set to %d Hz\n", ledcReadFreq(pwmChannel1));
                }

                dmxAddressChanged = false;
                nukeDriver();
            }

            dmx_read(dmxPort, dmxData, packet.size);

            int index = dmxAddress;
            if (index + 3 < DMX_PACKET_SIZE) {
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

                // Write to PWM outputs
                ledcWrite(pwmChannel1, scaledBrightness1);
                ledcWrite(pwmChannel2, scaledBrightness2);

                unsigned long now = millis();
                if (now - lastUpdate > 750) {
                    DEBUG_PRINTF("Startcode: %d, Address: %d\n", dmxData[0], dmxAddress);
                    DEBUG_PRINTF("Personality: %d\n", currentPersonality);
                    // Use ledcReadFreq to display the current PWM frequency
                    DEBUG_PRINTF("PWM Frequency: %d Hz\n", ledcReadFreq(pwmChannel1));
                    DEBUG_PRINTF("Channel 1 - DMX: %d, PWM: %d\n", rawBrightness1, scaledBrightness1);
                    DEBUG_PRINTF("Channel 2 - DMX: %d, PWM: %d\n", rawBrightness2, scaledBrightness2);
                    lastUpdate = now;
                }
            } else {
                DEBUG_PRINTLN("DMX Address out of range.");
            }
        } else {
            DEBUG_PRINTLN("A DMX error occurred.");
        }
    } else if (dmxIsConnected) {
        DEBUG_PRINTLN("DMX was disconnected.");
        dmxIsConnected = false;
    }
}
