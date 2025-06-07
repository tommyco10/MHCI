/********************************************************************
 * Board A – DMX / RDM front-end that also streams its brightness
 *           to another ESP32 over I²C (SDA 23, SCL 22).
 * Library versions: esp_dmx 4.1.0, Arduino-ESP32 2.0.14
 ********************************************************************/
#define DEBUG
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
#include <Wire.h>            // NEW
#include <math.h>
#include <esp_dmx.h>
#include <rdm/responder.h>

/* ---------- I²C settings --------------------------------------- */
constexpr uint8_t  I2C_SDA_PIN   = 23;
constexpr uint8_t  I2C_SCL_PIN   = 22;
constexpr uint8_t  SLAVE_ADDR    = 0x30;
constexpr uint32_t I2C_FREQ_HZ   = 400000;   // fast mode

/* ---------- PWM configuration ---------------------------------- */
const uint8_t  pwmChannel1   = 1;
const uint8_t  pwmChannel2   = 2;
const uint8_t  pwmResolution = 16;
const uint16_t pwmThreshold  = 0;

/* ---------- personality → PWM frequency table ------------------ */
const uint16_t pwmFreqPersonality[] = {
  0, 1000, 500,  60, 1000, 500,  60
};
/*            ^ dummy for index 0 */

/* ---------- pins ------------------------------------------------ */
const uint8_t builtInLedPin = BUILTIN_LED;
const uint8_t outputLedPin1 = 27;
const uint8_t outputLedPin2 = 19;
const uint8_t enablePin     = ENABLE_PIN;
const uint8_t transmitPin   = TRANSMIT_PIN;
const uint8_t receivePin    = RECEIVE_PIN;

/* ---------- DMX ------------------------------------------------- */
const dmx_port_t dmxPort = DMX_NUM_1;
byte     dmxData[DMX_PACKET_SIZE];
uint16_t dmxAddress;
bool     dmxIsConnected = false;
bool     dmxAddressChanged = false;

/* ---------- S-curve parameters (unchanged) ---------------------- */
const float midpointBrightness1 = 8441.0f, x0_1 = 32768.0f, k1_1 = 0.0002f;
const float s0_1 = 1.0f / (1.0f + expf(k1_1 * x0_1)), s1_1 = 0.5f;

const float midpointBrightness2 = 8441.0f, x0_2 = 32768.0f, k1_2 = 0.0002f;
const float s0_2 = 1.0f / (1.0f + expf(k1_2 * x0_2)), s1_2 = 0.5f;

/* ---------- runtime state -------------------------------------- */
uint8_t  currentPersonality;
uint16_t pwmFreq;
unsigned long lastUpdate = 0;

/* ---------- helpers -------------------------------------------- */
uint16_t computeSCurve(uint16_t in, float B, float x0, float k,
                       float s0, float s1)
{
  if (in <= x0) {
    float s = 1.0f / (1.0f + expf(-k * (in - x0)));
    float sn = (s - s0) / (s1 - s0);
    return uint16_t(B * sn);
  } else {
    float u = (in - x0) / (65535.0f - x0);
    return uint16_t(B + (65535.0f - B) * u);
  }
}

uint16_t getPWMFrequencyForPersonality(uint8_t p)
{
  if (p >= 1 && p <= 6) return pwmFreqPersonality[p];
  return 1000;
}

/* ---------- DMX driver helper ---------------------------------- */
void reinstallDriver()
{
  dmx_driver_delete(dmxPort);

  dmx_config_t cfg = {
      .interrupt_flags             = DMX_INTR_FLAGS_DEFAULT,
      .root_device_parameter_count = 32,
      .sub_device_parameter_count  = 0,
      .model_id                    = 1,
      .product_category            = RDM_PRODUCT_CATEGORY_FIXTURE,
      .software_version_id         = ESP_DMX_VERSION_ID,
      .software_version_label      = ESP_DMX_VERSION_LABEL,
      .queue_size_max              = 32            // IDF-4 safe
  };
  static dmx_personality_t pers[] = {
      {4, "16Bit sCurve 1000Hz"},  {4, "16Bit sCurve 500Hz"},
      {4, "16Bit sCurve 60Hz"},    {4, "16Bit Linear 1000Hz"},
      {4, "16Bit Linear 500Hz"},   {4, "16Bit Linear 60Hz"}
  };
  dmx_driver_install(dmxPort, &cfg, pers, 6);
  dmx_set_pin(dmxPort, transmitPin, receivePin, enablePin);

  dmxAddress        = dmx_get_start_address(dmxPort);
  currentPersonality = dmx_get_current_personality(dmxPort);
  pwmFreq           = getPWMFrequencyForPersonality(currentPersonality);
}

/* ----------------------------------------------------------------*/
void setup()
{
#ifdef DEBUG
  Serial.begin(921600);
#endif
  reinstallDriver();

  /* PWM initialisation */
  ledcSetup(pwmChannel1, pwmFreq, pwmResolution);
  ledcAttachPin(outputLedPin1, pwmChannel1);
  ledcAttachPin(builtInLedPin, pwmChannel1);

  ledcSetup(pwmChannel2, pwmFreq, pwmResolution);
  ledcAttachPin(outputLedPin2, pwmChannel2);

  /* I²C master */
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
  Wire.setBufferSize(16);

  DEBUG_PRINTF("DMX addr %u, personality %u, PWM %u Hz\n",
               dmxAddress, currentPersonality, pwmFreq);
}

/* ----------------------------------------------------------------*/
void loop()
{
  dmx_packet_t packet;
  if (dmx_receive(dmxPort, &packet, DMX_TIMEOUT_TICK)) {

    if (packet.is_rdm && !packet.err) {           // ---- RDM
      rdm_send_response(dmxPort);
      dmxAddressChanged = true;
    }
    else if (!packet.err) {                       // ---- DMX

      if (!dmxIsConnected) { DEBUG_PRINTLN("DMX linked"); dmxIsConnected = true; }

      if (dmxAddressChanged) {                    // re-read addr / personality
        dmxAddress = dmx_get_start_address(dmxPort);
        uint8_t newPers = dmx_get_current_personality(dmxPort);
        if (newPers != currentPersonality) {
          currentPersonality = newPers;
          pwmFreq = getPWMFrequencyForPersonality(currentPersonality);
          ledcSetup(pwmChannel1, pwmFreq, pwmResolution);
          ledcSetup(pwmChannel2, pwmFreq, pwmResolution);
        }
        dmxAddressChanged = false;
      }

      dmx_read(dmxPort, dmxData, packet.size);

      int idx = dmxAddress;
      if (idx + 3 < DMX_PACKET_SIZE) {

        /* ----- DMX → PWM mapping -------------------------------- */
        uint16_t raw1 = (dmxData[idx]   << 8) | dmxData[idx+1];
        uint16_t raw2 = (dmxData[idx+2] << 8) | dmxData[idx+3];

        uint16_t pwm1, pwm2;
        if (currentPersonality <= 3) {
          pwm1 = computeSCurve(raw1, midpointBrightness1, x0_1, k1_1, s0_1, s1_1);
          pwm2 = computeSCurve(raw2, midpointBrightness2, x0_2, k1_2, s0_2, s1_2);
        } else {
          pwm1 = raw1;
          pwm2 = raw2;
        }
        if (pwm1 < pwmThreshold) pwm1 = 0;
        if (pwm2 < pwmThreshold) pwm2 = 0;

        ledcWrite(pwmChannel1, pwm1);
        ledcWrite(pwmChannel2, pwm2);

        /* ----- send 3-byte I²C packet --------------------------- */
        uint8_t pkt[3] = { currentPersonality,
                           uint8_t(pwm1 >> 8), uint8_t(pwm1 & 0xFF) };
        Wire.beginTransmission(SLAVE_ADDR);
        Wire.write(pkt, sizeof(pkt));
        Wire.endTransmission();

        /* ----- optional debug every 750 ms ---------------------- */
        unsigned long now = millis();
        if (now - lastUpdate > 750) {
          DEBUG_PRINTF("Pers %u  PWMfreq %u Hz  Ch1 %u→%u  Ch2 %u→%u\n",
                       currentPersonality, pwmFreq, raw1, pwm1, raw2, pwm2);
          lastUpdate = now;
        }
      }

    }
  }
  else if (dmxIsConnected) {
    DEBUG_PRINTLN("DMX lost");
    dmxIsConnected = false;
  }
}