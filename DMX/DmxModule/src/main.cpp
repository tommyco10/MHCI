/********************************************************************
 * Board A – sends full DMX universe plus personality & start-address
 *           to a second ESP32 over I²C.
 *           Uses esp_dmx 4.1.0  ·  Arduino-ESP32 ≥ 2.0.14
 ********************************************************************/
#define DEBUG
#ifdef DEBUG
  #define DBG(x)     Serial.print(x)
  #define DBGLN(x)   Serial.println(x)
  #define DBGF(...)  Serial.printf(__VA_ARGS__)
#else
  #define DBG(x)
  #define DBGLN(x)
  #define DBGF(...)
#endif

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <esp_dmx.h>
#include <rdm/responder.h>

/*──────────── I²C ───────────────────────────────────────────────*/
constexpr uint8_t  I2C_SDA_PIN   = 23;
constexpr uint8_t  I2C_SCL_PIN   = 22;
constexpr uint8_t  SLAVE_ADDR    = 0x30;
constexpr uint32_t I2C_FREQ      = 400000;      // fast mode

/*──────────── DMX universe transfer parameters ──────────────────*/
constexpr size_t   DMX_UNIVERSE  = 513;         // start-code + 512 data
constexpr uint8_t  CHUNK_BYTES   = 173;         // 173 × 3 = 519 ≥ 513
static   uint8_t   txBuf[CHUNK_BYTES];

/*──────────── your original PWM & S-curve constants ─────────────*/
const uint8_t pwmChannel1 = 1, pwmChannel2 = 2, pwmResolution = 16;
const uint16_t pwmThreshold = 0;

/* personality → PWM frequency table (index 0 dummy) */
const uint16_t pwmFreqPers[] = { 0, 1000, 500, 60, 1000, 500, 60 };

/* pins */
const uint8_t builtInLedPin = BUILTIN_LED;
const uint8_t outputLedPin1 = 27, outputLedPin2 = 19;
const uint8_t enablePin = ENABLE_PIN, transmitPin = TRANSMIT_PIN, receivePin = RECEIVE_PIN;

/* DMX driver */
const dmx_port_t dmxPort = DMX_NUM_1;
byte dmxData[DMX_PACKET_SIZE];
uint16_t dmxAddress;
uint8_t  currentPersonality;
bool dmxIsConnected = false, dmxAddressChanged = false;

/* S-curve params (same as before, shortened here) */
const float mid1 = 8441.0f, x0_1 = 32768.0f, k1_1 = 0.0002f,
            s0_1 = 1.0f / (1.0f + expf(k1_1 * x0_1)), s1_1 = 0.5f;

uint16_t pwmFreq;
unsigned long lastPrint = 0;

/*── helper: S-curve ─────────────────────────────────────────────*/
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

/*── reinstall DMX driver (unchanged except queue size) ──────────*/
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
      .queue_size_max              = 32
  };
  static dmx_personality_t pers[] = {
      {4, "16Bit sCurve 1000Hz"},  {4, "16Bit sCurve 500Hz"},
      {4, "16Bit sCurve 60Hz"},    {4, "16Bit Linear 1000Hz"},
      {4, "16Bit Linear 500Hz"},   {4, "16Bit Linear 60Hz"}
  };
  dmx_driver_install(dmxPort, &cfg, pers, 6);
  dmx_set_pin(dmxPort, transmitPin, receivePin, enablePin);

  dmxAddress         = dmx_get_start_address(dmxPort);
  currentPersonality = dmx_get_current_personality(dmxPort);
  pwmFreq            = pwmFreqPers[currentPersonality];
}

/*───────────────────────────────────────────────────────────────*/
void setup()
{
#ifdef DEBUG
  Serial.begin(921600);
#endif
  reinstallDriver();

  /* PWM init */
  ledcSetup(pwmChannel1, pwmFreq, pwmResolution);
  ledcAttachPin(outputLedPin1, pwmChannel1);
  ledcAttachPin(builtInLedPin,   pwmChannel1);
  ledcSetup(pwmChannel2, pwmFreq, pwmResolution);
  ledcAttachPin(outputLedPin2, pwmChannel2);

  /* I²C master */
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ);
  Wire.setBufferSize(256);

  DBGF("Init – addr %u  pers %u  PWM %u Hz\n",
       dmxAddress, currentPersonality, pwmFreq);
}

/*───────────────────────────────────────────────────────────────*/
void loop()
{
  dmx_packet_t pkt;
  if (!dmx_receive(dmxPort, &pkt, DMX_TIMEOUT_TICK)) {   // timeout
    if (dmxIsConnected) { DBGLN("DMX lost"); dmxIsConnected = false; }
    return;
  }

  if (pkt.err) return;                 // ignore error

  if (pkt.is_rdm) {                    // handle RDM
    rdm_send_response(dmxPort);
    dmxAddressChanged = true;
    return;
  }

  /* DMX data frame arrived -------------------------------------*/
  if (!dmxIsConnected) { DBGLN("DMX linked"); dmxIsConnected = true; }

  if (dmxAddressChanged) {
    dmxAddress = dmx_get_start_address(dmxPort);
    uint8_t p  = dmx_get_current_personality(dmxPort);
    if (p != currentPersonality) {
      currentPersonality = p;
      pwmFreq = pwmFreqPers[p];
      ledcSetup(pwmChannel1, pwmFreq, pwmResolution);
      ledcSetup(pwmChannel2, pwmFreq, pwmResolution);
    }
    dmxAddressChanged = false;
  }

  dmx_read(dmxPort, dmxData, pkt.size);

  /* ── brightness calculation exactly like your original ────── */
  int idx = dmxAddress;
  uint16_t raw1 = (dmxData[idx]   << 8) | dmxData[idx+1];
  uint16_t raw2 = (dmxData[idx+2] << 8) | dmxData[idx+3];

  uint16_t pwm1, pwm2;
  if (currentPersonality <= 3) {
    pwm1 = computeSCurve(raw1, mid1, x0_1, k1_1, s0_1, s1_1);
    pwm2 = computeSCurve(raw2, mid1, x0_1, k1_1, s0_1, s1_1);
  } else {
    pwm1 = raw1;  pwm2 = raw2;
  }
  if (pwm1 < pwmThreshold) pwm1 = 0;
  if (pwm2 < pwmThreshold) pwm2 = 0;

  ledcWrite(pwmChannel1, pwm1);
  ledcWrite(pwmChannel2, pwm2);

  /* ── send universe in three chunks ─────────────────────────── */
  for (uint16_t ofs = 0; ofs < DMX_UNIVERSE; ofs += CHUNK_BYTES) {
    uint8_t headerExtra = (ofs == 0) ? 3 : 0;        // pers + addrHi + addrLo
    uint16_t room = CHUNK_BYTES - headerExtra;
    uint16_t n    = min<uint16_t>(room, DMX_UNIVERSE - ofs);

    Wire.beginTransmission(SLAVE_ADDR);
    Wire.write(uint8_t(ofs >> 8));   // offset high
    Wire.write(uint8_t(ofs));        // offset low
    if (ofs == 0) {                  // first chunk carries meta
      Wire.write(currentPersonality);
      Wire.write(uint8_t(dmxAddress >> 8));
      Wire.write(uint8_t(dmxAddress));
    }
    Wire.write(dmxData + ofs, n);
    Wire.endTransmission();          // separate STARTs are fine
  }

  /* ── debug every 750 ms ────────────────────────────────────── */
  unsigned long now = millis();
  if (now - lastPrint > 750) {
    DBGF("pers %u  PWM %u Hz  Ch1 %u→%u  Ch2 %u→%u\n",
         currentPersonality, pwmFreq, raw1, pwm1, raw2, pwm2);
    lastPrint = now;
  }
}