#pragma once
#include <Arduino.h>
#include <Adafruit_seesaw.h>
#include <seesaw_neopixel.h>

#define SS_SWITCH 24
#define SS_NEOPIX 6
#define SEESAW_BASE_ADDR          0x36 

enum EncoderMode {
    RAW,
    BOUNDED,
    SCALED,
};

class Encoder {
  public:
    Encoder(const int32_t inputMin, const int32_t inputMax, const int32_t outputMin, const int32_t outputMax, const EncoderMode mode, const uint8_t switchScale = 1) 
      : inputMin(inputMin), inputMax(inputMax), outputMin(outputMin), outputMax(outputMax), mode(mode), switchScale(switchScale) {
        hardwareEncoder = Adafruit_seesaw();
        hardwareNeopixel = seesaw_NeoPixel(1, SS_NEOPIX, NEO_GRB + NEO_KHZ800);
        found = false;
    }

    bool begin(const uint8_t addressOffset) {
        if (!hardwareEncoder.begin(SEESAW_BASE_ADDR + addressOffset) || !hardwareNeopixel.begin(SEESAW_BASE_ADDR + addressOffset)) {
            Serial.printf("Couldn't find encoder #%d\n", addressOffset);
            return found;
        } 

        Serial.printf("Found encoder + pixel #%d\n", addressOffset);

        uint32_t version = ((hardwareEncoder.getVersion() >> 16) & 0xFFFF);
        if (version != 4991) {
            Serial.printf("Wrong firmware loaded: %d\n", version);
            return found;
        }

        // Configure the encoder and neopixel
        hardwareEncoder.pinMode(SS_SWITCH, INPUT_PULLUP);
        hardwareEncoder.setGPIOInterrupts((uint32_t)1 << SS_SWITCH, 1);
        hardwareEncoder.enableEncoderInterrupt();
        hardwareNeopixel.setBrightness(30);
        hardwareNeopixel.show();

        found = true;
        return found;
    }

    bool updateEncoder() {
        if (!found) return false;

        // Check for encoder switch press
        switchPressed = hardwareEncoder.digitalRead(SS_SWITCH);

        // Read the encoder value and delta
        
        encoderDelta = hardwareEncoder.getEncoderDelta();

        if (switchPressed) {
            encoderDelta *= switchScale; // Scale the delta by the switch scale factor
        }
        encoderVal += encoderDelta;

        if (mode == EncoderMode::RAW) return true;

        encoderVal = constrain(encoderVal, inputMin, inputMax);
        
        return true;
    }

    bool updateNeopixel(const uint8_t r, const uint8_t g, const uint8_t b) {
        if (!found) return false;

        // Update the neopixel color
        hardwareNeopixel.setPixelColor(0, r, g, b);
        hardwareNeopixel.show();

        return true;
    }

    int32_t getValue() {
        updateEncoder();

        switch (mode)
        {
        case EncoderMode::RAW:
            return encoderVal;
            break;
        
        case EncoderMode::BOUNDED:
            return constrain(encoderVal, inputMin, inputMax);
        
        case EncoderMode::SCALED:
            return map(encoderVal, inputMin, inputMax, outputMin, outputMax);
            break;

        default:
            return -1;
            break;
        }
    }

    int32_t getRawValue() {
        return encoderVal;
    }



private:
    Adafruit_seesaw hardwareEncoder;
    seesaw_NeoPixel hardwareNeopixel;
    bool found = false;

    int32_t encoderVal = 0;
    int32_t encoderDelta = 0;
    bool switchPressed = false;
    uint8_t switchScale;

    EncoderMode mode;

    int32_t inputMin;
    int32_t inputMax;

    int32_t outputMin;
    int32_t outputMax;

};
