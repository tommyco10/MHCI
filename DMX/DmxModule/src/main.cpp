/***********************************************************************
 *  I²C-DMX   RECEIVER — dumps the whole DMX universe to Serial
 *  Board: Adafruit Feather ESP32  (built-in LED on GPIO 13)
 ***********************************************************************/

#define DEBUG                       // comment to silence info prints
#define PRINT_UNIVERSE              // comment to disable slot dump

/* -------- how often to dump the universe -------------------------- */
#define DUMP_INTERVAL_MS  1000      // every 1 s

#include <Arduino.h>
#include <driver/i2c.h>

/* -------- I²C configuration --------------------------------------- */
#define I2C_PORT        I2C_NUM_0
#define SDA_PIN         23
#define SCL_PIN         22
#define SLAVE_ADDR      0x28

/* DMA buffer lengths — 1 KiB each is plenty for 513 bytes ----------- */
#define DMA_BYTES       1024

/* -------- DMX frame ----------------------------------------------- */
#define SLOTS           512
#define FRAME_BYTES     (SLOTS + 1)   // start-code + data bytes

/* -------- LED PWM -------------------------------------------------- */
#define LED_PIN         LED_BUILTIN
#define LED_PWM_CH      0
#define LED_PWM_HZ      1000          // 1 kHz, 16-bit
#define LED_ACTIVE_LOW  false         // change if your LED is wired LOW-active

/* -------- debug macros -------------------------------------------- */
#ifdef DEBUG
  #define DBG(x)    Serial.print(x)
  #define DBGLN(x)  Serial.println(x)
  #define DBGF(...) Serial.printf(__VA_ARGS__)
#else
  #define DBG(x)
  #define DBGLN(x)
  #define DBGF(...)
#endif

/* -------- frame buffer -------------------------------------------- */
static uint8_t  dmx[FRAME_BYTES] = {0};
static uint32_t frameCnt = 0;
static uint32_t lastDump  = 0;

/* ------------------------------------------------------------------ */
#if defined(PRINT_UNIVERSE) && defined(DEBUG)
void dumpUniverse()
{
  DBGF("\n--- DMX frame #%lu ---\nStart-code: %3u\n", frameCnt, dmx[0]);

  for (uint16_t i = 1; i <= SLOTS; i++)
  {
    DBGF("%3u ", dmx[i]);              // decimal, 0-255
    if (i % 16 == 0) DBGLN("");        // newline every 16 slots
  }
  DBGLN("\n--- end of frame ---\n");
}
#endif
/* ================================================================== */
void setup()
{
#ifdef DEBUG
  Serial.begin(921600);
  delay(100);
  DBGLN("\nReceiver booting…");
#endif

  /* ---------- LED PWM set-up -------------------------------------- */
  pinMode(LED_PIN, OUTPUT);
  ledcSetup(LED_PWM_CH, LED_PWM_HZ, 16);
  ledcAttachPin(LED_PIN, LED_PWM_CH);
  ledcWrite(LED_PWM_CH, LED_ACTIVE_LOW ? 0xFFFF : 0);   // LED off

  /* ---------- I²C slave ------------------------------------------- */
  i2c_config_t cfg = {};
  cfg.mode                = I2C_MODE_SLAVE;
  cfg.sda_io_num          = SDA_PIN;
  cfg.scl_io_num          = SCL_PIN;
  cfg.sda_pullup_en       = GPIO_PULLUP_ENABLE;
  cfg.scl_pullup_en       = GPIO_PULLUP_ENABLE;
  cfg.slave.addr_10bit_en = false;
  cfg.slave.slave_addr    = SLAVE_ADDR;

  i2c_param_config(I2C_PORT, &cfg);
  i2c_driver_install(I2C_PORT, I2C_MODE_SLAVE,
                     DMA_BYTES, DMA_BYTES, 0);

  DBGLN("Ready – waiting for DMX frames.");
}

/* ================================================================== */
void loop()
{
  /* blocking read – up to 50 ms */
  int n = i2c_slave_read_buffer(I2C_PORT, dmx, FRAME_BYTES,
                                pdMS_TO_TICKS(50));

  if (n == FRAME_BYTES)                           // got full frame
  {
    frameCnt++;

    /* LED brightness from first two slots (16-bit) */
    uint16_t level = (static_cast<uint16_t>(dmx[1]) << 8) | dmx[2];
    uint16_t duty  = LED_ACTIVE_LOW ? (0xFFFF - level) : level;
    ledcWrite(LED_PWM_CH, duty);

#ifdef DEBUG
    /* dump entire universe every DUMP_INTERVAL_MS */
    uint32_t now = millis();
    if (now - lastDump >= DUMP_INTERVAL_MS) {
  #if defined(PRINT_UNIVERSE)
      dumpUniverse();
  #else
      DBGF("Frame %lu  Ch1-2 level: %u  PWM duty: %u\n",
           frameCnt, level, duty);
  #endif
      lastDump = now;
    }
#endif
  }
  else if (n > 0)                                 // partial junk
  {
    uint8_t dumpBuf[FRAME_BYTES];
    while (i2c_slave_read_buffer(I2C_PORT, dumpBuf, FRAME_BYTES, 0) > 0) {}
    DBGLN("⚠︎ partial frame discarded");
  }

  /* place for other background tasks */
}