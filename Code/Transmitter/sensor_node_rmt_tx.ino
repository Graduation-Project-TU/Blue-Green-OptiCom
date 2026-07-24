#include <Arduino.h>
#include <string.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

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
static const uint32_t MANCHESTER_HALF_BIT_US = 10;
static const uint32_t END_GAP_US = 2400;  // End-of-frame idle marker
static const uint8_t PREAMBLE_BYTES = 8;

// -----------------------------
// FreeRTOS: queues & task params
// -----------------------------
static const UBaseType_t MEASUREMENT_QUEUE_LENGTH = 8;
static const UBaseType_t PAYLOAD_QUEUE_LENGTH = 8;
static const size_t PAYLOAD_BUF_SIZE = 128;

static QueueHandle_t xMeasurementQueue = nullptr;
static QueueHandle_t xPayloadQueue = nullptr;

static const uint8_t SENSOR_TASK_PRIO = 1;
static const uint8_t FORMATTER_TASK_PRIO = 2;
static const uint8_t RMT_TASK_PRIO = 2;

static const uint32_t SENSOR_TASK_STACK = 4096;
static const uint32_t FORMATTER_TASK_STACK = 3072;
static const uint32_t RMT_TASK_STACK = 6144;

struct Measurement {
  float temperatureC;
  float distanceCm;
};

struct FormattedPayload {
  char text[PAYLOAD_BUF_SIZE];
};

OneWire oneWire(ONEWIRE_PIN);
DallasTemperature tempSensor(&oneWire);
rmt_channel_handle_t txChannel = nullptr;
rmt_encoder_handle_t copyEncoder = nullptr;

static void initSensors();
static void initRmtTransmitter();
static float readTemperatureC();
static float readDistanceCm();
static Measurement readMeasurements();
static String formatPayload(const Measurement &m);
static void formatPayloadToBuffer(const Measurement &m, FormattedPayload *out);
static size_t manchesterEncodeByte(uint8_t value, rmt_symbol_word_t *symbols, size_t index);
static size_t buildManchesterFrame(const char *payload, size_t payloadLen, rmt_symbol_word_t *symbols,
                                   size_t maxSymbols);
static bool sendManchesterFrame(const char *payload, size_t payloadLen);

static void sensorReaderTask(void *pvParameters);
static void dataFormatterTask(void *pvParameters);
static void rmtTransmitterTask(void *pvParameters);

void setup() {
  Serial.begin(115200);
  delay(300);

  initSensors();
  initRmtTransmitter();

  xMeasurementQueue = xQueueCreate(MEASUREMENT_QUEUE_LENGTH, sizeof(Measurement));
  xPayloadQueue = xQueueCreate(PAYLOAD_QUEUE_LENGTH, sizeof(FormattedPayload));
  if (!xMeasurementQueue || !xPayloadQueue) {
    Serial.println("Fatal: queue creation failed");
    while (true) {
      delay(1000);
    }
  }

  BaseType_t ok = xTaskCreatePinnedToCore(sensorReaderTask, "sensorReader", SENSOR_TASK_STACK, nullptr,
                                          SENSOR_TASK_PRIO, nullptr, 0);
  if (ok != pdPASS) {
    Serial.println("Fatal: sensorReaderTask create failed");
    while (true) {
      delay(1000);
    }
  }

  ok = xTaskCreatePinnedToCore(dataFormatterTask, "dataFormatter", FORMATTER_TASK_STACK, nullptr,
                               FORMATTER_TASK_PRIO, nullptr, 0);
  if (ok != pdPASS) {
    Serial.println("Fatal: dataFormatterTask create failed");
    while (true) {
      delay(1000);
    }
  }

  ok = xTaskCreatePinnedToCore(rmtTransmitterTask, "rmtTx", RMT_TASK_STACK, nullptr, RMT_TASK_PRIO, nullptr, 1);
  if (ok != pdPASS) {
    Serial.println("Fatal: rmtTransmitterTask create failed");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("Sensor Node (RMT TX + Manchester, dual-core FreeRTOS) Ready");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

static void sensorReaderTask(void *pvParameters) {
  (void)pvParameters;
  for (;;) {
    Measurement m = readMeasurements();
    if (xQueueSend(xMeasurementQueue, &m, portMAX_DELAY) != pdTRUE) {
      // Should not happen with portMAX_DELAY; kept for completeness
    }
    vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
  }
}

static void dataFormatterTask(void *pvParameters) {
  (void)pvParameters;
  Measurement m{};
  for (;;) {
    if (xQueueReceive(xMeasurementQueue, &m, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    FormattedPayload fp{};
    formatPayloadToBuffer(m, &fp);

    Serial.print("TX Payload: ");
    Serial.print(fp.text);
    if (isnan(m.temperatureC)) {
      Serial.println("TX warning: DS18B20 read failed (nan)");
    }
    if (m.distanceCm < 0.0f) {
      Serial.println("TX warning: Ultrasonic timeout (-1)");
    }

    if (xQueueSend(xPayloadQueue, &fp, portMAX_DELAY) != pdTRUE) {
      // Should not happen with portMAX_DELAY
    }
  }
}

static void rmtTransmitterTask(void *pvParameters) {
  (void)pvParameters;
  FormattedPayload fp{};
  for (;;) {
    if (xQueueReceive(xPayloadQueue, &fp, portMAX_DELAY) != pdTRUE) {
      continue;
    }
    const size_t len = strnlen(fp.text, sizeof(fp.text));
    if (!sendManchesterFrame(fp.text, len)) {
      Serial.println("RMT TX failed");
    }
  }
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

static void formatPayloadToBuffer(const Measurement &m, FormattedPayload *out) {
  String payload = formatPayload(m);
  const size_t n = payload.length();
  if (n >= PAYLOAD_BUF_SIZE) {
    out->text[0] = '\0';
    return;
  }
  memcpy(out->text, payload.c_str(), n);
  out->text[n] = '\0';
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

static size_t buildManchesterFrame(const char *payload, size_t payloadLen, rmt_symbol_word_t *symbols,
                                   size_t maxSymbols) {
  size_t index = 0;

  // Longer preamble increases chance of synchronization when RX starts mid-frame.
  for (uint8_t i = 0; i < PREAMBLE_BYTES; ++i) {
    if (index + 8 >= maxSymbols) {
      return 0;
    }
    index = manchesterEncodeByte(0x55, symbols, index);
  }

  for (size_t i = 0; i < payloadLen; ++i) {
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

static bool sendManchesterFrame(const char *payload, size_t payloadLen) {
  const size_t maxSymbols = PREAMBLE_BYTES * 8 + payloadLen * 8 + 1;
  rmt_symbol_word_t *symbols = static_cast<rmt_symbol_word_t *>(
      heap_caps_malloc(maxSymbols * sizeof(rmt_symbol_word_t), MALLOC_CAP_INTERNAL));
  if (!symbols) {
    return false;
  }

  size_t used = buildManchesterFrame(payload, payloadLen, symbols, maxSymbols);
  if (used == 0) {
    heap_caps_free(symbols);
    return false;
  }

  rmt_transmit_config_t txConfig = {};
  txConfig.loop_count = 0;

  esp_err_t err = rmt_transmit(txChannel, copyEncoder, symbols, used * sizeof(rmt_symbol_word_t), &txConfig);
  if (err == ESP_OK) {
    err = rmt_tx_wait_all_done(txChannel, pdMS_TO_TICKS(100));
  }

  heap_caps_free(symbols);
  return err == ESP_OK;
}
