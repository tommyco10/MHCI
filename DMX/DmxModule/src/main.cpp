/*****************************************************************
 *  DMX / RDM  →  I²C sender                                     *
 *  - Listens on UART (esp_dmx v4.1)                             *
 *  - Scales CH1+CH2 exactly like your original sketch           *
 *  - Ships the *whole* 512-byte universe to the receiver        *
 *****************************************************************/
#include <Arduino.h>
#include <Wire.h>
#include <esp_dmx.h>
#include <rdm/responder.h>

// ─── USER SETTINGS ─────────────────────────────────────────────
#define BAUD            921600
#define I2C_ADDR        0x30
#define SDA_PIN         23
#define SCL_PIN         22
#define I2C_CLOCK_HZ    400000UL             // 400 kHz
#define CHUNK_LEN       128                  // 128-byte slices
// ───────────────────────────────────────────────────────────────

// === DMX & PWM CONFIG (unaltered from your original) ===========
const uint8_t  pwmChannel1   = 1;
const uint8_t  pwmChannel2   = 2;
const uint8_t  pwmResolution = 16;
const uint16_t pwmThreshold  = 0;
const uint8_t  builtInLedPin = BUILTIN_LED;
const uint8_t  outputLedPin1 = 27;
const uint8_t  outputLedPin2 = 19;

const uint16_t pwmFreqPersonality[] = {0,1000,500,60,1000,500,60};
const float midpointBrightness1 = 8441.0f, midpointBrightness2 = 8441.0f;
const float x0   = 32768.0f,      k1 = 0.0002f, s0 = 1.0f/(1.0f+expf(k1*x0));
const float s1   = 0.5f;
// ==============================================================

inline uint16_t computeSCurve(uint16_t in, float B)
{
  if (in <= x0) {
    float s = 1.0f / (1.0f + expf(-k1 * (in - x0)));
    float out = B * ((s - s0) / (s1 - s0));
    return (uint16_t)out;
  }
  float u = (in - x0) / (65535.0f - x0);
  return (uint16_t)(B + (65535.0f - B) * u);
}

// DMX driver objects
const dmx_port_t dmxPort = DMX_NUM_1;
byte  dmxUniverse[DMX_PACKET_SIZE];
uint16_t dmxAddress;
uint8_t  personality;

// helpers
void sendUniverseI2C()
{
  // send 512 bytes in 4 × 128-byte chunks
  for (int i = 0; i < DMX_PACKET_SIZE; i += CHUNK_LEN) {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(&dmxUniverse[i], CHUNK_LEN);
    uint8_t err = Wire.endTransmission();

    if (err == 0)
      Serial.printf("I2C_OK  chunk %d  len %d\n", i / CHUNK_LEN, CHUNK_LEN);
    else
      Serial.printf("I2C_ERR %u  chunk %d\n", err, i / CHUNK_LEN);
  }
}

void setup()
{
  Serial.begin(BAUD);
  delay(50);
  Serial.println("\n\n=== DMX → I2C Sender ===");

  // I²C
  Wire.begin(SDA_PIN, SCL_PIN, I2C_CLOCK_HZ);
  Serial.printf("I2C ready @%lu Hz, addr 0x%02X\n", I2C_CLOCK_HZ, I2C_ADDR);

  // DMX driver
  dmx_config_t cfg = {
      .interrupt_flags = DMX_INTR_FLAGS_DEFAULT,
      .root_device_parameter_count = 32,
      .sub_device_parameter_count  = 0,
      .model_id        = 1,
      .product_category= RDM_PRODUCT_CATEGORY_FIXTURE,
      .software_version_id     = ESP_DMX_VERSION_ID,
      .software_version_label  = ESP_DMX_VERSION_LABEL,
      .queue_size_max          = 0
  };
  dmx_personality_t pers[] = {
      {4,"16Bit sCurve 1000Hz"}, {4,"16Bit sCurve 500Hz"},
      {4,"16Bit sCurve 60Hz"},   {4,"16Bit Linear 1000Hz"},
      {4,"16Bit Linear 500Hz"},  {4,"16Bit Linear 60Hz"}
  };
  dmx_driver_install(dmxPort, &cfg, pers, 6);
  dmx_set_pin(dmxPort, TRANSMIT_PIN, RECEIVE_PIN, ENABLE_PIN);

  // PWM channels
  ledcSetup(pwmChannel1, 1000, pwmResolution);
  ledcAttachPin(outputLedPin1, pwmChannel1);
  ledcAttachPin(builtInLedPin, pwmChannel1);
  ledcSetup(pwmChannel2, 1000, pwmResolution);
  ledcAttachPin(outputLedPin2, pwmChannel2);
}

void loop()
{
  dmx_packet_t pkt;
  if (!dmx_receive(dmxPort, &pkt, DMX_TIMEOUT_TICK) || pkt.err) return;

  static bool first = true;
  if (first) { Serial.println("DMX connected!"); first = false; }

  dmxAddress   = dmx_get_start_address(dmxPort);
  personality  = dmx_get_current_personality(dmxPort);
  uint16_t pwmFreq = pwmFreqPersonality[personality];
  ledcSetup(pwmChannel1, pwmFreq, pwmResolution);
  ledcSetup(pwmChannel2, pwmFreq, pwmResolution);

  dmx_read(dmxPort, dmxUniverse, pkt.size);

  // === your original scaling for CH1+2 ========================
  uint16_t raw1 = ((uint16_t)dmxUniverse[dmxAddress] << 8) |
                   dmxUniverse[dmxAddress + 1];
  uint16_t raw2 = ((uint16_t)dmxUniverse[dmxAddress + 2] << 8) |
                   dmxUniverse[dmxAddress + 3];

  uint16_t pwm1 = (personality <= 3)
                  ? computeSCurve(raw1, midpointBrightness1)
                  : raw1;
  uint16_t pwm2 = (personality <= 3)
                  ? computeSCurve(raw2, midpointBrightness2)
                  : raw2;

  if (pwm1 < pwmThreshold) pwm1 = 0;
  if (pwm2 < pwmThreshold) pwm2 = 0;

  ledcWrite(pwmChannel1, pwm1);
  ledcWrite(pwmChannel2, pwm2);

  Serial.printf("pers %d  PWM %u Hz  Ch1 %u→%u  Ch2 %u→%u\n",
                personality, pwmFreq, raw1, pwm1, raw2, pwm2);
  // ===========================================================

  sendUniverseI2C();          // <-- NEW: ship all 512 bytes
}