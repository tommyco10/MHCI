/***********************************************************************
 *  I²C-DMX RECEIVER  (ESP32, Arduino core 2.x)
 *  – Listens as I²C slave at 0x28 on SDA-23 / SCL-22
 *  – Collects 513-byte frames pushed by the master (see previous code)
 *  – Uses ESP-IDF I²C driver (DMA kicks in automatically for >32 B)
 ***********************************************************************/

#define DEBUG                       // comment-out to disable Serial prints

/* -------- I²C settings (match the sender!) ------------------------- */
#define I2C_SDA_PIN        23
#define I2C_SCL_PIN        22
#define I2C_PORT           I2C_NUM_0
#define I2C_SLAVE_ADDR     0x28        // 7-bit
#define I2C_BUFFER_BYTES   1024        // RX/TX DMA buffers

/* -------- DMX universe size --------------------------------------- */
#define DMX_SLOTS          512
#define FRAME_BYTES        (DMX_SLOTS + 1)   // + start-code

/* -------- convenience macros -------------------------------------- */
#ifdef DEBUG
#  define DEBUG_PRINT(x)    Serial.print(x)
#  define DEBUG_PRINTLN(x)  Serial.println(x)
#  define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#  define DEBUG_PRINT(x)
#  define DEBUG_PRINTLN(x)
#  define DEBUG_PRINTF(...)
#endif

#include <Arduino.h>
#include <driver/i2c.h>

/* ---------- global data ------------------------------------------- */
static uint8_t  dmxUniverse[FRAME_BYTES] = {0};   // latest frame
static uint32_t frameCounter  = 0;                // how many frames received
static uint8_t  ledState      = LOW;

/* ------------------------------------------------------------------ */
void setup()
{
#ifdef DEBUG
    Serial.begin(921600);
    delay(100);
    DEBUG_PRINTLN("\nI²C-DMX receiver booting…");
#endif

    /* ---- configure I²C peripheral in SLAVE mode ------------------ */
    i2c_config_t cfg = {};
    cfg.mode             = I2C_MODE_SLAVE;
    cfg.sda_io_num       = I2C_SDA_PIN;
    cfg.scl_io_num       = I2C_SCL_PIN;
    cfg.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    cfg.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    cfg.slave.addr_10bit_en = false;
    cfg.slave.slave_addr    = I2C_SLAVE_ADDR;
    i2c_param_config(I2C_PORT, &cfg);

    /*  The ESP-IDF driver allocates two DMA-capable circular buffers:
     *  one for RX, one for TX.  Each must be ≥ the largest transfer
     *  you expect.  1 kB is fine for a 513-byte frame. */
    esp_err_t err = i2c_driver_install(I2C_PORT, I2C_MODE_SLAVE,
                                       I2C_BUFFER_BYTES, I2C_BUFFER_BYTES, 0);
    if (err != ESP_OK) {
        DEBUG_PRINTF("I2C install failed: 0x%X\n", err);
        while (true) { delay(1000); }
    }

    pinMode(LED_BUILTIN, OUTPUT);
    DEBUG_PRINTLN("Ready – waiting for DMX frames over I²C.");
}

/* ------------------------------------------------------------------ */
void loop()
{
    /*  Wait (blocking) until the master has written an entire frame.
     *  We use a timeout so we can blink the LED if nothing arrives. */
    int bytesRead = i2c_slave_read_buffer(I2C_PORT,
                                          dmxUniverse, FRAME_BYTES,
                                          pdMS_TO_TICKS(50));  // 50 ms

    if (bytesRead == FRAME_BYTES)
    {
        /* ---------- got a complete universe! ---------------------- */
        frameCounter++;

#ifdef DEBUG
        /* Show first four channels just to prove it works */
        uint16_t ch1 = (dmxUniverse[1] << 8) | dmxUniverse[2];
        uint16_t ch2 = (dmxUniverse[3] << 8) | dmxUniverse[4];
        DEBUG_PRINTF("Frame %lu  SC:%u  Ch1:%u  Ch2:%u\n",
                     frameCounter, dmxUniverse[0], ch1, ch2);
#endif
        /* quick blink */
        digitalWrite(LED_BUILTIN, HIGH);
        delayMicroseconds(100);
        digitalWrite(LED_BUILTIN, LOW);
    }
    else if (bytesRead > 0)   /* partial frame – flush it */
    {
        /* drain remaining bytes so we realign on next frame */
        uint8_t dummy[FRAME_BYTES];
        while (i2c_slave_read_buffer(I2C_PORT, dummy,
                                     FRAME_BYTES, 0) > 0) { /* discard */ }
        DEBUG_PRINTLN("⚠︎ partial frame discarded");
    }

    /* Nothing arrived?  Do other background work here if you need it */
}