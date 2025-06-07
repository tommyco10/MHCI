/********************************************************************
 * Board B – receives 3 bytes over I²C and sets its BUILTIN_LED
 *           to the same 16-bit PWM value coming from Board A.
 ********************************************************************/
#define DEBUG
#ifdef DEBUG
  #define DBG(x)    Serial.print(x)
  #define DBGLN(x)  Serial.println(x)
  #define DBGF(...) Serial.printf(__VA_ARGS__)
#else
  #define DBG(x)
  #define DBGLN(x)
  #define DBGF(...)
#endif

#include <Arduino.h>
#include <Wire.h>

/* I²C parameters (must match Board A) */
constexpr uint8_t  SDA_PIN   = 23;
constexpr uint8_t  SCL_PIN   = 22;
constexpr uint8_t  I2C_ADDR  = 0x30;
constexpr uint32_t I2C_FREQ  = 400000;

/* PWM parameters */
const uint8_t  PWM_CH   = 1;
const uint8_t  PWM_BITS = 16;
const uint16_t pwmFreqPers[] = { 0, 1000, 500, 60, 1000, 500, 60 };

/* state */
volatile uint8_t  rxBuf[3] = {1, 0, 0};
volatile bool     frameReady = false;

/* I²C receive ISR */
void IRAM_ATTR onReceive(int len)
{
  if (len == 3) {
    for (int i = 0; i < 3; ++i) rxBuf[i] = Wire.read();
    frameReady = true;
  } else while (Wire.available()) Wire.read();
}

void applyPersonality(uint8_t p)
{
  if (p < 1 || p > 6) return;
  ledcSetup(PWM_CH, pwmFreqPers[p], PWM_BITS);
  DBGF("Personality %u → %u Hz\n", p, pwmFreqPers[p]);
}

void setup()
{
#ifdef DEBUG
  Serial.begin(921600);
#endif
  applyPersonality(1);
  ledcAttachPin(BUILTIN_LED, PWM_CH);

  Wire.setBufferSize(16);
  Wire.begin(I2C_ADDR, SDA_PIN, SCL_PIN, I2C_FREQ);
  Wire.onReceive(onReceive);

  DBGLN("Slave ready.");
}

void loop()
{
  if (!frameReady) { delay(1); return; }
  frameReady = false;

  uint8_t  pers  = rxBuf[0];
  uint16_t pwm16 = (rxBuf[1] << 8) | rxBuf[2];

  static uint8_t persPrev = 1;
  if (pers != persPrev && pers >= 1 && pers <= 6) {
    applyPersonality(pers);
    persPrev = pers;
  }

  ledcWrite(PWM_CH, pwm16);

#ifdef DEBUG
  static unsigned long t0 = 0;
  if (millis() - t0 > 750) {
    DBGF("Pers %u  PWM %u\n", pers, pwm16);
    t0 = millis();
  }
#endif
}