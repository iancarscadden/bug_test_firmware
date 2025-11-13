#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Adafruit_NeoPixel.h>
#include "driver/gpio.h" // Added for GPIO_NUM_xx definitions
extern "C"
{
#include "driver/i2s_std.h" // IDF 5.x I2S standard mode API
}

/***** ============ USER CONFIG ============ *****/

// I2S pins (SPH0645 SEL -> 3V => RIGHT channel)
#define PIN_I2S_LRCLK GPIO_NUM_5 // Feather D5  → LRCLK/WS
#define PIN_I2S_BCLK GPIO_NUM_6  // Feather D6  → BCLK
#define PIN_I2S_DIN GPIO_NUM_9   // Feather D9  → DOUT

// NeoPixel (onboard)
#define NEOPIXEL_PIN 33
#define NEOPIXEL_NUM 1

// Audio capture config
#define SAMPLE_RATE_HZ 16000 // 16 kHz, speech
#define STREAM_MS 5000       // record 5 sec per START
#define CAPTURE_SAMPLES 512  // samples per I2S read (32-bit containers)

// BLE notify payload sizing (~5–6ms per packet at 16k mono 16-bit)
#define BLE_PCM_BYTES_PER_NOTIFY 160 // 80 samples * 2 bytes each

// BLE names/UUIDs
static const char *BLE_DEVICE_NAME = "ESP32S3-Audio";
static const char *UUID_AUDIO_SERVICE = "3c3b0001-8c5a-4b78-9c3a-1d5a6db3a001";
static const char *UUID_CONTROL_CHAR = "3c3b0002-8c5a-4b78-9c3a-1d5a6db3a001"; // Write
static const char *UUID_AUDIO_CHAR = "3c3b0003-8c5a-4b78-9c3a-1d5a6db3a001";   // Notify

/***** ============ GLOBALS ============ *****/

// NeoPixel
Adafruit_NeoPixel pixel(NEOPIXEL_NUM, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
static inline void setPixel(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness = 40)
{
  pixel.setBrightness(brightness);
  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.show();
}

// BLE
static NimBLEServer *g_server = nullptr;
static NimBLEService *g_service = nullptr;
static NimBLECharacteristic *g_controlChar = nullptr;
static NimBLECharacteristic *g_audioChar = nullptr;

static volatile bool g_isConnected = false;
static volatile bool g_wantStart = false; // flipped in callback only
static bool g_streaming = false;          // controlled in loop
static uint32_t g_streamEndMs = 0;
static portMUX_TYPE g_stateMux = portMUX_INITIALIZER_UNLOCKED;

// I2S (STD driver)
static i2s_chan_handle_t g_i2sRx = nullptr;
static bool g_i2sReady = false;

// Buffers
static int32_t g_i2sBuf32[CAPTURE_SAMPLES];             // 32-bit raw samples
static int16_t g_pcm16Buf[CAPTURE_SAMPLES];             // converted 16-bit samples for BLE
static uint8_t g_bleFrag[4 + BLE_PCM_BYTES_PER_NOTIFY]; // BLE notification buffer (header + audio)
static uint16_t g_seq = 0;                              // sequence counter for BLE packets

/***** ============ I2S (IDF 5.x STD) ============ *****/

static void i2sInitStd()
{
  // Configure and allocate an I2S RX channel (master mode, uses I2S0)
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chan_cfg.dma_desc_num = 8; // deeper DMA to tolerate BLE delays
  chan_cfg.dma_frame_num = CAPTURE_SAMPLES;
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &g_i2sRx));

  // Set up standard I2S configuration (clock, slot, GPIO)
  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE_HZ),                // 16 kHz sample rate
      .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, // 32-bit data container
                                                  I2S_SLOT_MODE_MONO),      // mono mode
      .gpio_cfg = {
          .mclk = I2S_GPIO_UNUSED, // MCLK not used
          .bclk = PIN_I2S_BCLK,    // Bit Clock line
          .ws = PIN_I2S_LRCLK,     // Word Select (LR clock) line
          .dout = I2S_GPIO_UNUSED, // Not transmitting (TX unused)
          .din = PIN_I2S_DIN,      // Data input from mic
          .invert_flags = {
              .mclk_inv = false,
              .bclk_inv = false,
              .ws_inv = false}}};
  // Limit to right channel (mic is on right channel due to SEL=3V)
  std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_RIGHT;

  // Initialize I2S channel in standard mode with the combined config
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(g_i2sRx, &std_cfg));
  ESP_ERROR_CHECK(i2s_channel_enable(g_i2sRx));

  // Perform a non-blocking dummy read to warm up the I2S DMA
  size_t bytes_read = 0;
  (void)i2s_channel_read(g_i2sRx, g_i2sBuf32, sizeof(g_i2sBuf32), &bytes_read, 10);

  g_i2sReady = true;
  Serial.println("[I2S] STD ready.");
}

static void i2sDeinitStd()
{
  if (g_i2sRx)
  {
    i2s_channel_disable(g_i2sRx);
    i2s_del_channel(g_i2sRx);
    g_i2sRx = nullptr;
  }
  g_i2sReady = false;
}

static inline void convert32to16(const int32_t *in32, int16_t *out16, int n)
{
  // Convert 18-bit effective data (in 32-bit container) to 16-bit PCM
  for (int i = 0; i < n; ++i)
  {
    int32_t s = in32[i] >> 14; // downshift to ~18-bit range
    if (s > 32767)
      s = 32767;
    else if (s < -32768)
      s = -32768;
    out16[i] = (int16_t)s;
  }
}

/***** ============ BLE ============ *****/

class ServerCB : public NimBLEServerCallbacks
{
  void onConnect(NimBLEServer *, ble_gap_conn_desc *) override
  {
    g_isConnected = true;
    setPixel(0, 255, 0); // GREEN LED when central is connected
    Serial.println("[BLE] Central connected.");
  }
  void onDisconnect(NimBLEServer *) override
  {
    g_isConnected = false;
    // Safely reset streaming state on disconnect
    portENTER_CRITICAL(&g_stateMux);
    g_wantStart = false;
    portEXIT_CRITICAL(&g_stateMux);
    g_streaming = false;
    i2sDeinitStd();
    setPixel(0, 0, 255); // BLUE LED for advertising state
    Serial.println("[BLE] Central disconnected, advertising...");
    NimBLEDevice::startAdvertising();
  }
};

class ControlWriteCB : public NimBLECharacteristicCallbacks
{
  void onWrite(NimBLECharacteristic *c) override
  {
    // This callback runs in NimBLE stack task context; just flag the request
    std::string val = c->getValue();
    for (char &ch : val)
    {
      ch = toupper((unsigned char)ch);
    }
    if (val.find("START") != std::string::npos)
    {
      // Signal the main loop to start streaming
      portENTER_CRITICAL(&g_stateMux);
      g_wantStart = true;
      portEXIT_CRITICAL(&g_stateMux);
    }
  }
};

static void bleInit()
{
  NimBLEDevice::init(BLE_DEVICE_NAME);
  NimBLEDevice::setMTU(247);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  g_server = NimBLEDevice::createServer();
  g_server->setCallbacks(new ServerCB());

  g_service = g_server->createService(UUID_AUDIO_SERVICE);
  g_controlChar = g_service->createCharacteristic(
      UUID_CONTROL_CHAR,
      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  g_controlChar->setCallbacks(new ControlWriteCB());

  g_audioChar = g_service->createCharacteristic(
      UUID_AUDIO_CHAR,
      NIMBLE_PROPERTY::NOTIFY);

  g_service->start();
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(UUID_AUDIO_SERVICE);
  adv->setScanResponse(true);
  adv->start();
}

/***** ============ STREAMING HELPERS ============ *****/

static void bleSendPcm16(const int16_t *pcm, int samples)
{
  if (!g_isConnected)
    return;
  if (g_audioChar->getSubscribedCount() == 0)
    return;

  const uint8_t *src = (const uint8_t *)pcm;
  int totalBytes = samples * sizeof(int16_t);
  int offset = 0;
  uint32_t tick = 0;
  while (offset < totalBytes && g_isConnected && g_streaming)
  {
    int chunk = totalBytes - offset;
    if (chunk > BLE_PCM_BYTES_PER_NOTIFY)
    {
      chunk = BLE_PCM_BYTES_PER_NOTIFY;
    }
    // Prepare 4-byte header: sequence (2 bytes) + sample count (2 bytes)
    uint16_t seq = g_seq++;
    uint16_t nSamp = chunk / 2; // number of int16 samples in this chunk
    memcpy(g_bleFrag, &seq, sizeof(seq));
    memcpy(g_bleFrag + 2, &nSamp, sizeof(nSamp));
    memcpy(g_bleFrag + 4, src + offset, chunk);

    g_audioChar->setValue(g_bleFrag, 4 + chunk);
    g_audioChar->notify(); // send notification (non-blocking in NimBLE)

    offset += chunk;
    // Yield periodically to avoid WDT reset
    if ((++tick & 0x7) == 0)
    {
      delay(0);
    }
  }
}

/***** ============ ARDUINO SETUP/LOOP ============ *****/

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    delay(10);
  } // wait for serial port

  pixel.begin();
  setPixel(0, 0, 255); // BLUE LED: waiting/advertising

  Serial.println("[BOOT] Starting BLE advertising; waiting for connection...");
  bleInit();
}

void loop()
{
  if (!g_isConnected)
  {
    // Not connected to any BLE central; just idle
    delay(20);
    return;
  }

  // If a START command was received via BLE, begin streaming
  if (!g_streaming && g_wantStart)
  {
    // Promote the request flag to actual streaming action
    portENTER_CRITICAL(&g_stateMux);
    g_wantStart = false;
    portEXIT_CRITICAL(&g_stateMux);

    if (!g_i2sReady)
    {
      i2sInitStd(); // initialize I2S if not already
    }
    g_streaming = true;
    g_streamEndMs = millis() + STREAM_MS;
    g_seq = 0; // reset sequence counter for new stream

    setPixel(255, 0, 0); // RED LED: streaming active
    Serial.println("[CTRL] START received; streaming for 5s...");
  }

  if (g_streaming)
  {
    // Check if time limit reached
    if ((int32_t)(millis() - g_streamEndMs) >= 0)
    {
      g_streaming = false;
      i2sDeinitStd();
      setPixel(0, 255, 0); // GREEN LED: stream finished, idle state
      Serial.println("[I2S] Finished 5s capture. Streaming done; back to idle.");
      return;
    }

    // Read a block of 32-bit samples from I2S (blocking read)
    size_t bytesRead = 0;
    esp_err_t ret = i2s_channel_read(g_i2sRx, g_i2sBuf32, sizeof(g_i2sBuf32), &bytesRead, portMAX_DELAY);
    if (ret == ESP_OK && bytesRead > 0)
    {
      int nSamples = bytesRead / sizeof(int32_t);
      convert32to16(g_i2sBuf32, g_pcm16Buf, nSamples);
      bleSendPcm16(g_pcm16Buf, nSamples);
    }
    else
    {
      // In case of read error or no data, yield briefly
      delay(1);
    }
  }
  else
  {
    // Connected but not streaming, small delay to prevent tight loop
    delay(10);
  }
}
