//written using a clone of esp_dmx v4.1.0

//#define DEBUG  // Uncomment this line to enable debugging messages

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

#include "scurve.hpp"

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ElegantOTA.h>

const char* ssid = "W-Streetlight-5-DMX";
const char* password = "94499449";
WebServer server(80);
unsigned long ota_progress_millis = 0;
IPAddress local_IP(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 0, 0);

// PWM Configuration
const uint8_t pwmChannels[6] = {1, 2, 3, 4, 5, 6};
const uint8_t dmxBits = 16;
const uint8_t pwmResolution = 14; // PWM resolution of 14 bits
const uint16_t pwmThreshold = 0;  // Low Value Threshold for PWM output

// PWM frequencies for each personality
const uint16_t pwmFreqPersonality[6] = {300, 500, 1000, 500, 500, 1000};

const uint16_t lowThreshold[6] = {260, 500, 950, 260, 500, 950};
uint16_t pwmOutput[6] = {0, 0, 0, 0, 0, 0};
uint16_t pwmOutputOld[6] = {0, 0, 0, 0, 0, 0};

// Pins
const uint8_t builtInLedPin = 13;
const uint8_t outputPins[6] = {OUTPUT_PIN1, OUTPUT_PIN2, OUTPUT_PIN3, OUTPUT_PIN4, OUTPUT_PIN5, OUTPUT_PIN6};
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

// Variable to store the current personality
uint8_t currentPersonality;

// PWM frequency variable
uint16_t pwmFreq;

// RDM Identify mode
bool identifyMode = false;

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

uint16_t scaleBrightness(uint16_t rawBrightness) {
    uint16_t scaledBrightness;
    if (currentPersonality == 1 || currentPersonality == 2 || currentPersonality == 3) {
        // Apply S-curve
        scaledBrightness = computeSCurve(rawBrightness, midpointBrightness1, x0_1, k1_1, s0_1, s1_1);
    } else if (currentPersonality == 4 || currentPersonality == 5 || currentPersonality == 6) {
        // Use raw DMX values directly
        scaledBrightness = rawBrightness;
    }

    // Apply low value threshold
    if (scaledBrightness < pwmThreshold) {
        scaledBrightness = 0;
    }

    return scaledBrightness;
}

void onOTAStart() {
    DEBUG_PRINTLN("OTA Update Started");
}
void onOTAProgress(size_t current, size_t final) {
    if (millis() - ota_progress_millis > 1000) {
        ota_progress_millis = millis();
        DEBUG_PRINTF("OTA Progress Current: %u bytes, Final: %u bytes\n\r", current, final);
    }
}

void onOTAEnd(bool success) {
    if (success) {
        DEBUG_PRINTLN("OTA Updated Successfully");
    } else {
        DEBUG_PRINTLN("OTA Update Failed");
    }
}

void startOTA() {
    if (WiFi.getMode() != WIFI_OFF) return;

    DEBUG_PRINTLN("Starting OTA mode");
    WiFi.softAPConfig(local_IP, gateway, subnet);
    WiFi.softAP(ssid, password);

    IPAddress IP = WiFi.softAPIP();
    DEBUG_PRINTLN(IP);

    server.on("/", []() {
        server.send(200, "text/plain", "Upload your new code here!");
    });

    ElegantOTA.begin(&server);
    ElegantOTA.onStart(onOTAStart);
    ElegantOTA.onEnd(onOTAEnd);
    ElegantOTA.onProgress(onOTAProgress);

    server.begin();
}

void stopOTA() {
    if (WiFi.getMode() == WIFI_OFF) return;

    DEBUG_PRINTLN("Stopping OTA mode");
    server.stop();
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
}

void setup() {
    #ifdef DEBUG
    Serial.begin(115200);
    #endif

    WiFi.mode(WIFI_OFF);

    // Install the DMX driver
    dmx_config_t config = {
        .interrupt_flags = DMX_INTR_FLAGS_DEFAULT,
        .root_device_parameter_count = 32,
        .sub_device_parameter_count = 0,
        .model_id = 1,
        .product_category = RDM_PRODUCT_CATEGORY_FIXTURE,
        .software_version_id = ((1 << 16) | (0 << 8) | 0),
        .software_version_label = "1.0.1 March 2025",
        .queue_size_max = 0, // default 32 but 0 is smoother dimming
    };
    dmx_personality_t personalities[] = {
        {6, "16Bit sCurve 300Hz          "},
        {6, "16Bit sCurve 500Hz          "},
        {6, "16Bit sCurve 1000Hz         "},
        {6, "16Bit Linear 300Hz          "},
        {6, "16Bit Linear 500Hz          "},
        {6, "16Bit Linear 1000Hz         "}
    };
    dmx_driver_install(dmxPort, &config, personalities, 6);
    dmx_set_pin(dmxPort, transmitPin, receivePin, enablePin);

    

    // Get the initial DMX start address and personality
    dmxAddress = dmx_get_start_address(dmxPort);
    currentPersonality = dmx_get_current_personality(dmxPort);

    // Set PWM frequency based on initial personality
    pwmFreq = getPWMFrequencyForPersonality(currentPersonality);

    // Set up the PWM channels for LEDs
    for (int i = 0; i < 6; i++) {
        ledcSetup(pwmChannels[i], pwmFreq, pwmResolution);
        ledcAttachPin(outputPins[i], pwmChannels[i]);
    }

    // Now we can read the actual PWM frequency
    DEBUG_PRINTF("Startcode: %d, Address: %d\n\r", dmxData[0], dmxAddress);
    DEBUG_PRINTF("Personality: %u\n\r", currentPersonality);
    // Use ledcReadFreq to display the current PWM frequency
    DEBUG_PRINTF("PWM Frequency: %u Hz\n\r", pwmFreqPersonality[currentPersonality - 1]);

    lastUpdate = millis();
}

void loop() {
    dmx_packet_t packet;

    // If identify mode then start the OTA server

    if (identifyMode) {
        startOTA();
        server.handleClient();
        ElegantOTA.loop();
    } else {
        stopOTA();
    }

    if (dmx_receive(dmxPort, &packet, DMX_TIMEOUT_TICK)) {

        if (packet.is_rdm && !packet.err) {
            // Handle RDM packet
            rdm_send_response(dmxPort);
            DEBUG_PRINTLN("RDM packet received and response sent.");
            rdm_get_identify_device(dmxPort, &identifyMode);

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
                    DEBUG_PRINTF("DMX Start Address updated to %d\n\r", dmxAddress);
                }

                uint8_t newPersonality = dmx_get_current_personality(dmxPort);
                if (newPersonality != currentPersonality) {
                    currentPersonality = newPersonality;
                    DEBUG_PRINTF("Personality changed to %d\n\r", currentPersonality);

                    // Get the new PWM frequency based on personality
                    uint16_t newPwmFreq = getPWMFrequencyForPersonality(currentPersonality);
                    pwmFreq = newPwmFreq;
                    ledcSetup(pwmChannels[0], pwmFreq, pwmResolution);
                    ledcSetup(pwmChannels[1], pwmFreq, pwmResolution);

                    // Use ledcReadFreq to display the updated PWM frequency
                    DEBUG_PRINTF("PWM frequency set to %d Hz\n", ledcReadFreq(pwmChannels[0]));
                }

                dmxAddressChanged = false;
            }

            dmx_read(dmxPort, dmxData, packet.size);

            int index = dmxAddress;
            if (index + 5 < DMX_PACKET_SIZE) {
                uint16_t rawBrigtnesses[3];
                uint16_t scaledBrightnesses[3];

                rawBrigtnesses[0] = (((uint16_t)dmxData[index + 0] << 8) | dmxData[index + 1]);
                rawBrigtnesses[1] = (((uint16_t)dmxData[index + 2] << 8) | dmxData[index + 3]);
                rawBrigtnesses[2] = (((uint16_t)dmxData[index + 4] << 8) | dmxData[index + 5]);
 
                uint8_t bitshift = dmxBits - pwmResolution;
                
                scaledBrightnesses[0] = scaleBrightness(rawBrigtnesses[0]);
                scaledBrightnesses[1] = scaleBrightness(rawBrigtnesses[1]);
                scaledBrightnesses[2] = scaleBrightness(rawBrigtnesses[2]);

                pwmOutput[0] = (scaledBrightnesses[0] < lowThreshold[currentPersonality - 1]) ? 0 : scaledBrightnesses[0];
                pwmOutput[1] = (scaledBrightnesses[0] < lowThreshold[currentPersonality - 1]) ? 0 : scaledBrightnesses[0];

                pwmOutput[2] = (scaledBrightnesses[1] < lowThreshold[currentPersonality - 1]) ? 0 : scaledBrightnesses[1];
                pwmOutput[3] = (scaledBrightnesses[1] < lowThreshold[currentPersonality - 1]) ? 0 : scaledBrightnesses[1];

                pwmOutput[4] = (scaledBrightnesses[2] < lowThreshold[currentPersonality - 1]) ? 0 : scaledBrightnesses[2];
                pwmOutput[5] = (scaledBrightnesses[2] < lowThreshold[currentPersonality - 1]) ? 0 : scaledBrightnesses[2];

                for (int i = 0; i < 6; i++) {
                    if (pwmOutputOld[i] < lowThreshold[currentPersonality - 1] && pwmOutput[i] >= lowThreshold[currentPersonality - 1]) {
                        pwmOutput[i] = lowThreshold[currentPersonality - 1];
                    }
                }

                for (int i = 0; i < 6; i++) {
                    ledcWrite(pwmChannels[i], pwmOutput[i] >> bitshift);
                    pwmOutputOld[i] = pwmOutput[i];
                }

                unsigned long now = millis();
                if (now - lastUpdate > 2000) {
                    DEBUG_PRINTF("Startcode: %d, Address: %d\n\r", dmxData[0], dmxAddress);
                    DEBUG_PRINTF("Personality: %u\n\r", currentPersonality);
                    // Use ledcReadFreq to display the current PWM frequency
                    DEBUG_PRINTF("PWM Frequency: %u Hz\n\r", pwmFreqPersonality[currentPersonality - 1]);
                    for (int i = 0; i < 3; i++) {
                        DEBUG_PRINTF("Channel %i - DMX: %d, SCALE: %d, PWM: %d\n\r", i, rawBrigtnesses[i], scaledBrightnesses[i], scaledBrightnesses[i] >> bitshift);
                    }
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
    } else {
        DEBUG_PRINTLN("No DMX packet detected");
    }
}