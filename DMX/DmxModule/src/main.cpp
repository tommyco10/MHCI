/*****************************************************************
 *  I²C DMX receiver                                             *
 *  - Listens at addr 0x30                                       *
 *  - Rebuilds 512-byte universe                                 *
 *  - Mirrors DMX channel 1 (two bytes) on its built-in LED      *
 *****************************************************************/
#include <Arduino.h>
#include <Wire.h>

#define BAUD            921600
#define I2C_ADDR        0x30
#define SDA_PIN         23
#define SCL_PIN         22
#define I2C_CLOCK_HZ    400000UL
#define CHUNK_LEN       128                 // must match sender
#define DMX_UNIVERSE    512

byte dmx[DMX_UNIVERSE];
volatile int  nextOffset  = 0;              // where the next bytes go
volatile bool frameReady  = false;

void IRAM_ATTR onReceive(int nBytes)
{
  // copy straight into the universe buffer
  for (int i = 0; i < nBytes && nextOffset < DMX_UNIVERSE; ++i)
    dmx[nextOffset++] = Wire.read();

  // full universe received?
  if (nextOffset >= DMX_UNIVERSE) {
    frameReady = true;
    nextOffset = 0;                         // start over for next frame
  }
}

void setup()
{
  Serial.begin(BAUD);
  delay(50);
  Serial.println("\n\n=== I2C DMX receiver ===");

  pinMode(LED_BUILTIN, OUTPUT);
  ledcSetup(0, 1000, 16);                   // channel 0, 16-bit PWM
  ledcAttachPin(LED_BUILTIN, 0);

  Wire.begin(SDA_PIN, SCL_PIN, I2C_CLOCK_HZ);
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.onReceive(onReceive);
  Wire.begin(I2C_ADDR);
  Serial.printf("Listening on 0x%02X  @%lu Hz\n", I2C_ADDR, I2C_CLOCK_HZ);
}

void loop()
{
  static uint32_t t0 = 0;

  if (frameReady) {
    frameReady = false;

    uint16_t value = ((uint16_t)dmx[0] << 8) | dmx[1];  // DMX address 1
    ledcWrite(0, value);                                // 16-bit PWM

    Serial.printf("CH1 DMX %u → PWM %u\n", value, value);
  }

  // heartbeat every second
  if (millis() - t0 > 1000) {
    t0 = millis();
    Serial.printf("ISR offs %d  frameReady %d\n", nextOffset, frameReady);
  }
}