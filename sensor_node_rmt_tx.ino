#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "esp_err.h"
#include "esp_heap_caps.h"

// -----------------------------
// Hardware configuration
// -----------------------------
static const gpio_num_t ULTRASONIC_TRIG_PIN = GPIO_NUM_12;
static const gpio_num_t ULTRASONIC_ECHO_PIN = GPIO_NUM_13;
static const gpio_num_t ONEWIRE_PIN = GPIO_NUM_21;
static const gpio_num_t RMT_TX_PIN = GPIO_NUM_17;  // Replaces UART TX

// -----------------------------
// Sensor/RMT timing constants
// -----------------------------
static const uint32_t LOOP_PERIOD_MS = 2000;
static const uint32_t ULTRASONIC_TIMEOUT_US = 30000;
static const uint32_t MANCHESTER_HALF_BIT_US = 200;
static const uint32_t END_GAP_US = 2400;  // End-of-frame idle marker
static const uint8_t PREAMBLE_BYTES = 8;

OneWire oneWire(ONEWIRE_PIN);
DallasTemperature tempSensor(&oneWire);
rmt_channel_handle_t txChannel = nullptr;
rmt_encoder_handle_t copyEncoder = nullptr;

struct Measurement {
  float temperatureC;
  float distanceCm;
};

static void initSensors();
static void initRmtTransmitter();
static float readTemperatureC();
static float readDistanceCm();
static Measurement readMeasurements();
static String formatPayload(const Measurement &m);
static size_t manchesterEncodeByte(uint8_t value, rmt_symbol_word_t *symbols, size_t index);
static size_t buildManchesterFrame(const String &payload, rmt_symbol_word_t *symbols, size_t maxSymbols);
static bool sendManchesterFrame(const String &payload);

void setup() {
  Serial.begin(115200);
  delay(300);

  initSensors();
  initRmtTransmitter();

  Serial.println("Sensor Node (RMT TX + Manchester) Ready");
}

void loop() {
  Measurement m = readMeasurements();
  String payload = formatPayload(m);  // "temperature,distance\n"

  Serial.print("TX Payload: ");
  Serial.print(payload);
  if (isnan(m.temperatureC)) {
    Serial.println("TX warning: DS18B20 read failed (nan)");
  }
  if (m.distanceCm < 0.0f) {
    Serial.println("TX warning: Ultrasonic timeout (-1)");
  }

  if (!sendManchesterFrame(payload)) {
    Serial.println("RMT TX failed");
  }

  delay(LOOP_PERIOD_MS);
}

static void initSensors() {
  tempSensor.begin();
  Serial.print("DS18B20 devices found: ");
  Serial.println(tempSensor.getDeviceCount());

  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
}

static void initRmtTransmitter() {
  rmt_tx_channel_config_t txCfg = {};
  txCfg.gpio_num = RMT_TX_PIN;
  txCfg.clk_src = RMT_CLK_SRC_DEFAULT;
  txCfg.resolution_hz = 1000000;  // 1 tick = 1 us
  txCfg.mem_block_symbols = 256;
  txCfg.trans_queue_depth = 4;

  ESP_ERROR_CHECK(rmt_new_tx_channel(&txCfg, &txChannel));
  rmt_copy_encoder_config_t copyCfg = {};
  ESP_ERROR_CHECK(rmt_new_copy_encoder(&copyCfg, &copyEncoder));
  ESP_ERROR_CHECK(rmt_enable(txChannel));
}

static float readTemperatureC() {
  for (int attempt = 0; attempt < 3; ++attempt) {
    tempSensor.requestTemperatures();
    float t = tempSensor.getTempCByIndex(0);
    if (t != DEVICE_DISCONNECTED_C && t >= -55.0f && t <= 125.0f) {
      return t;
    }
    delay(50);
  }
  return NAN;
}

static float readDistanceCm() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);  // Required 10 us trigger pulse
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);
  if (duration == 0) {
    return -1.0f;  // Timeout case required by spec
  }

  // Required formula: distance = (duration * 0.1500) / 2
  return (duration * 0.1500f) / 2.0f;
}

static Measurement readMeasurements() {
  Measurement m;
  m.temperatureC = readTemperatureC();
  m.distanceCm = readDistanceCm();
  return m;
}

static String formatPayload(const Measurement &m) {
  String t = isnan(m.temperatureC) ? String("nan") : String(m.temperatureC, 2);
  String d = String(m.distanceCm, 2);

  String payload = t;
  payload += ",";
  payload += d;
  payload += "\n";
  return payload;
}

// Manchester mapping per bit:
// bit 1 -> HIGH then LOW
// bit 0 -> LOW then HIGH
static size_t manchesterEncodeByte(uint8_t value, rmt_symbol_word_t *symbols, size_t index) {
  for (int bit = 7; bit >= 0; --bit) {
    bool one = (value >> bit) & 0x01;

    rmt_symbol_word_t sym = {};
    sym.duration0 = MANCHESTER_HALF_BIT_US;
    sym.duration1 = MANCHESTER_HALF_BIT_US;
    sym.level0 = one ? 1 : 0;
    sym.level1 = one ? 0 : 1;

    symbols[index++] = sym;
  }
  return index;
}

static size_t buildManchesterFrame(const String &payload, rmt_symbol_word_t *symbols, size_t maxSymbols) {
  size_t index = 0;

  // Longer preamble increases chance of synchronization when RX starts mid-frame.
  for (uint8_t i = 0; i < PREAMBLE_BYTES; ++i) {
    if (index + 8 >= maxSymbols) {
      return 0;
    }
    index = manchesterEncodeByte(0x55, symbols, index);
  }

  for (size_t i = 0; i < payload.length(); ++i) {
    if (index + 8 >= maxSymbols) {
      return 0;
    }
    index = manchesterEncodeByte(static_cast<uint8_t>(payload[i]), symbols, index);
  }

  // End gap helps RX frame separation
  if (index < maxSymbols) {
    rmt_symbol_word_t gap = {};
    gap.duration0 = END_GAP_US;
    gap.level0 = 0;
    gap.duration1 = 0;
    gap.level1 = 0;
    symbols[index++] = gap;
  }

  return index;
}

static bool sendManchesterFrame(const String &payload) {
  const size_t maxSymbols = PREAMBLE_BYTES * 8 + payload.length() * 8 + 1;
  rmt_symbol_word_t *symbols = static_cast<rmt_symbol_word_t *>(
      heap_caps_malloc(maxSymbols * sizeof(rmt_symbol_word_t), MALLOC_CAP_INTERNAL));
  if (!symbols) {
    return false;
  }

  size_t used = buildManchesterFrame(payload, symbols, maxSymbols);
  if (used == 0) {
    free(symbols);
    return false;
  }

  rmt_transmit_config_t txConfig = {};
  txConfig.loop_count = 0;

  esp_err_t err = rmt_transmit(txChannel, copyEncoder, symbols, used * sizeof(rmt_symbol_word_t), &txConfig);
  if (err == ESP_OK) {
    err = rmt_tx_wait_all_done(txChannel, pdMS_TO_TICKS(100));
  }

  free(symbols);
  return err == ESP_OK;
}
