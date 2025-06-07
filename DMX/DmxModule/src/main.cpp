// ────────────────────────  compile-time options  ────────────────────────
#define DEBUG                           // comment-out to silence Serial
#define I2C_SCL_PIN          22         // GPIO 22 on Feather ESP32
#define I2C_SDA_PIN          23         // GPIO 23 on Feather ESP32
#define I2C_PORT             I2C_NUM_0  // use I2C0 hardware peripheral
#define I2C_SLAVE_ADDR       0x28       // second ESP32’s slave-ID
#define I2C_CLOCK_HZ         400000     // 400 kHz; safe for 512-byte DMA
// ────────────────────────────────────────────────────────────────────────

#ifdef DEBUG
#  define DEBUG_PRINT(x)     Serial.print(x)
#  define DEBUG_PRINTLN(x)   Serial.println(x)
#  define DEBUG_PRINTF(...)  Serial.printf(__VA_ARGS__)
#else
#  define DEBUG_PRINT(x)
#  define DEBUG_PRINTLN(x)
#  define DEBUG_PRINTF(...)
#endif

#include <Arduino.h>
#include <math.h>
#include <esp_dmx.h>
#include <rdm/responder.h>
#include <driver/i2c.h>        // ESP-IDF I²C driver (DMA capable)

// ──────────────────────────────  PWM setup  ─────────────────────────────
const uint8_t pwmChannel1   = 1;
const uint8_t pwmChannel2   = 2;
const uint8_t pwmResolution = 16;
const uint16_t pwmThreshold = 0;
const uint16_t pwmFreqPersonality1 = 1000;
const uint16_t pwmFreqPersonality2 =  500;
const uint16_t pwmFreqPersonality3 =   60;
const uint16_t pwmFreqPersonality4 = 1000;
const uint16_t pwmFreqPersonality5 =  500;
const uint16_t pwmFreqPersonality6 =   60;

// ───────────────────────────────  pins  ────────────────────────────────
const uint8_t builtInLedPin = BUILTIN_LED;
const uint8_t outputLedPin1 = 27;   // SparkFun D2
const uint8_t outputLedPin2 = 19;   // SparkFun D0
const uint8_t enablePin     = ENABLE_PIN;
const uint8_t transmitPin   = TRANSMIT_PIN;
const uint8_t receivePin    = RECEIVE_PIN;

// ───────────────────────── DMX / RDM globals ───────────────────────────
const dmx_port_t dmxPort = DMX_NUM_1;
uint16_t dmxAddress;
uint8_t  dmxData[DMX_PACKET_SIZE];
bool     dmxIsConnected     = false;
unsigned long lastUpdate    = 0;
bool dmxAddressChanged      = false;

// ─────────── S-curve params Ch-1 (repeat for Ch-2) ────────────
const float midpointBrightness1 = 8441.0f;
const float x0_1  = 32768.0f;
const float k1_1  = 0.0002f;
const float s0_1  = 1.0f / (1.0f + expf(k1_1 * x0_1));
const float s1_1  = 0.5f;

const float midpointBrightness2 = 8441.0f;
const float x0_2  = 32768.0f;
const float k1_2  = 0.0002f;
const float s0_2  = 1.0f / (1.0f + expf(k1_2 * x0_2));
const float s1_2  = 0.5f;

// ───────────────────────── other globals ───────────────────────────────
uint8_t  currentPersonality;
uint16_t pwmFreq;

/* ─────────────────── helper: DMX→PWM 16-bit S-curve ─────────────────── */
uint16_t computeSCurve(uint16_t input, float B, float x0,
                       float k1, float s0, float s1)
{
    if (input <= x0) {
        float s = 1.0f / (1.0f + expf(-k1 * (input - x0)));
        float sNorm = (s - s0) / (s1 - s0);
        return (uint16_t)(B * sNorm);
    } else {
        float u = (input - x0) / (65535.0f - x0);
        return (uint16_t)(B + (65535.0f - B) * u);
    }
}

/* ─────────── helper: personality → PWM frequency ─────────── */
uint16_t getPWMFrequencyForPersonality(uint8_t p)
{
    switch (p) {
        case 1:  return pwmFreqPersonality1;
        case 2:  return pwmFreqPersonality2;
        case 3:  return pwmFreqPersonality3;
        case 4:  return pwmFreqPersonality4;
        case 5:  return pwmFreqPersonality5;
        case 6:  return pwmFreqPersonality6;
        default: return pwmFreqPersonality1;
    }
}

/* ─────────────────────── nukeDriver()  (re-added) ───────────────────── */
void nukeDriver()
{
    dmx_driver_delete(dmxPort);

    dmx_config_t config = {
        .interrupt_flags             = DMX_INTR_FLAGS_DEFAULT,
        .root_device_parameter_count = 32,
        .sub_device_parameter_count  = 0,
        .model_id                    = 1,
        .product_category            = RDM_PRODUCT_CATEGORY_FIXTURE,
        .software_version_id         = ESP_DMX_VERSION_ID,
        .software_version_label      = ESP_DMX_VERSION_LABEL,
        .queue_size_max              = 0     // smoother dimming
    };

    dmx_personality_t personalities[] = {
        {4, "16Bit sCurve 1000Hz          "},
        {4, "16Bit sCurve 500Hz           "},
        {4, "16Bit sCurve 60Hz            "},
        {4, "16Bit Linear 1000Hz          "},
        {4, "16Bit Linear 500Hz           "},
        {4, "16Bit Linear 60Hz            "}
    };

    dmx_driver_install(dmxPort, &config, personalities, 6);
    dmx_set_pin(dmxPort, transmitPin, receivePin, enablePin);

    dmxAddress         = dmx_get_start_address(dmxPort);
    currentPersonality = dmx_get_current_personality(dmxPort);
}

/* ─────────────── helper: send universe over I²C (DMA) ──────────────── */
void sendUniverseI2C(const uint8_t *buf, size_t len = DMX_PACKET_SIZE)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_SLAVE_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, (uint8_t*)buf, len, true);          // DMA kicks in
    i2c_master_stop(cmd);
    /* Non-blocking: timeout = 0 means queue the transaction and return */
    i2c_master_cmd_begin(I2C_PORT, cmd, 0 /* ticks */);
    i2c_cmd_link_delete(cmd);
}

/* ─────────────────────────────  setup  ──────────────────────────────── */
void setup()
{
#ifdef DEBUG
    Serial.begin(921600);
    delay(100);
#endif

    /* ---------- I²C master (DMA capable) ---------- */
    i2c_config_t i2c_cfg = {};
    i2c_cfg.mode             = I2C_MODE_MASTER;
    i2c_cfg.sda_io_num       = I2C_SDA_PIN;
    i2c_cfg.scl_io_num       = I2C_SCL_PIN;
    i2c_cfg.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    i2c_cfg.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    i2c_cfg.master.clk_speed = I2C_CLOCK_HZ;
    i2c_param_config(I2C_PORT, &i2c_cfg);
    i2c_driver_install(I2C_PORT, i2c_cfg.mode, 0, 0, 0);   // 0 = default flags

    /* ---------- DMX driver ---------- */
    nukeDriver();   // installs driver & caches address/personality

    pwmFreq = getPWMFrequencyForPersonality(currentPersonality);

    ledcSetup(pwmChannel1, pwmFreq, pwmResolution);
    ledcAttachPin(outputLedPin1, pwmChannel1);
    ledcAttachPin(builtInLedPin, pwmChannel1);

    ledcSetup(pwmChannel2, pwmFreq, pwmResolution);
    ledcAttachPin(outputLedPin2, pwmChannel2);

    DEBUG_PRINTF("DMX start: %u  pers: %u  PWM %u Hz\n",
                 dmxAddress, currentPersonality, ledcReadFreq(pwmChannel1));

    lastUpdate = millis();
}

/* ─────────────────────────────  loop  ──────────────────────────────── */
void loop()
{
    dmx_packet_t packet;

    if (dmx_receive(dmxPort, &packet, DMX_TIMEOUT_TICK))
    {
        if (packet.is_rdm && !packet.err)
        {
            rdm_send_response(dmxPort);
            dmxAddressChanged = true;
        }
        else if (!packet.err)              // pure DMX
        {
            if (!dmxIsConnected) {
                DEBUG_PRINTLN("DMX connected");
                dmxIsConnected = true;
            }

            if (dmxAddressChanged) {
                uint16_t newAddr = dmx_get_start_address(dmxPort);
                uint8_t  newPer  = dmx_get_current_personality(dmxPort);

                if (newAddr != dmxAddress) {
                    dmxAddress = newAddr;
                    DEBUG_PRINTF("Addr → %u\n", dmxAddress);
                }
                if (newPer != currentPersonality) {
                    currentPersonality = newPer;
                    pwmFreq = getPWMFrequencyForPersonality(currentPersonality);
                    ledcSetup(pwmChannel1, pwmFreq, pwmResolution);
                    ledcSetup(pwmChannel2, pwmFreq, pwmResolution);
                    DEBUG_PRINTF("Pers → %u  PWM → %u Hz\n",
                                 currentPersonality, pwmFreq);
                }
                dmxAddressChanged = false;
                /* If you still want to rebuild driver on change, keep this: */
                // nukeDriver();
            }

            dmx_read(dmxPort, dmxData, packet.size);
            sendUniverseI2C(dmxData, DMX_PACKET_SIZE);   // <── DMA I²C push

            int idx = dmxAddress;
            if (idx + 3 < DMX_PACKET_SIZE)
            {
                uint16_t raw1 = (dmxData[idx] << 8) | dmxData[idx + 1];
                uint16_t raw2 = (dmxData[idx + 2] << 8) | dmxData[idx + 3];
                uint16_t pwm1, pwm2;

                if (currentPersonality <= 3) {
                    pwm1 = computeSCurve(raw1, midpointBrightness1,
                                         x0_1, k1_1, s0_1, s1_1);
                    pwm2 = computeSCurve(raw2, midpointBrightness2,
                                         x0_2, k1_2, s0_2, s1_2);
                } else {
                    pwm1 = raw1;
                    pwm2 = raw2;
                }
                if (pwm1 < pwmThreshold) pwm1 = 0;
                if (pwm2 < pwmThreshold) pwm2 = 0;

                ledcWrite(pwmChannel1, pwm1);
                ledcWrite(pwmChannel2, pwm2);

                unsigned long now = millis();
                if (now - lastUpdate > 750) {
                    DEBUG_PRINTF("SC:%u  Pers:%u  PWM:%u Hz\n",
                                 dmxData[0], currentPersonality,
                                 ledcReadFreq(pwmChannel1));
                    DEBUG_PRINTF("Ch1 DMX:%u → %u  Ch2 DMX:%u → %u\n",
                                 raw1, pwm1, raw2, pwm2);
                    lastUpdate = now;
                }
            }
        }
    }
    else if (dmxIsConnected) {
        DEBUG_PRINTLN("DMX disconnected");
        dmxIsConnected = false;
    }
}