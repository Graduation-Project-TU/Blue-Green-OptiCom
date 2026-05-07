#include <Arduino.h>
#include <vector>
#include "driver/rmt_rx.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// -----------------------------
// Hardware configuration
// -----------------------------
static const gpio_num_t RMT_RX_PIN = GPIO_NUM_16;  // Replaces UART RX

// -----------------------------
// Manchester/RMT constants
// -----------------------------
static const uint32_t MANCHESTER_HALF_BIT_US = 10;
static const uint16_t RX_MEM_SYMBOLS = 512;
static const uint16_t HALF_BIT_TOLERANCE_US = 80;
static const uint8_t PREAMBLE_BYTES = 8;

rmt_channel_handle_t rxChannel = nullptr;
static volatile bool rxDone = false;
static size_t rxNumSymbols = 0;
static SemaphoreHandle_t rxDoneSemaphore = nullptr;
static bool rxInProgress = false;
static rmt_symbol_word_t rxRawBuffer[RX_MEM_SYMBOLS];
static uint32_t lastRxStatusMs = 0;

static void initRmtReceiver();
static bool receiveSymbols(std::vector<rmt_symbol_word_t> &outSymbols);
static int durationToHalfBits(uint16_t durationUs);
static bool decodeManchesterSymbols(const std::vector<rmt_symbol_word_t> &symbols, String &decodedText);
static bool parseAndPrint(const String &line);
static bool IRAM_ATTR onRxDone(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *userData);
static bool parseFloatField(const String &s, float &outValue, bool &isNan);

void setup() {
  Serial.begin(115200);
  delay(300);

  initRmtReceiver();
  Serial.println("Receiver Node (RMT RX + Manchester) Ready");
}

void loop() {
  std::vector<rmt_symbol_word_t> symbols;
  if (!receiveSymbols(symbols)) {
    delay(5);
    return;
  }

  String rxText;
  if (!decodeManchesterSymbols(symbols, rxText)) {
    Serial.println("RX decode error");
    return;
  }

  // Payload can contain one or more '\n' terminated lines.
  int start = 0;
  while (true) {
    int end = rxText.indexOf('\n', start);
    if (end < 0) break;

    String line = rxText.substring(start, end);
    line.trim();
    if (line.length() > 0) {
      parseAndPrint(line);
    }
    start = end + 1;
  }
}

static void initRmtReceiver() {
  pinMode(static_cast<int>(RMT_RX_PIN), INPUT_PULLDOWN);

  rmt_rx_channel_config_t rxCfg = {};
  rxCfg.gpio_num = RMT_RX_PIN;
  rxCfg.clk_src = RMT_CLK_SRC_DEFAULT;
  rxCfg.resolution_hz = 1000000;  // 1 tick = 1 us
  rxCfg.mem_block_symbols = RX_MEM_SYMBOLS;

  ESP_ERROR_CHECK(rmt_new_rx_channel(&rxCfg, &rxChannel));

  rxDoneSemaphore = xSemaphoreCreateBinary();
  rmt_rx_event_callbacks_t cbs = {};
  cbs.on_recv_done = onRxDone;
  ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rxChannel, &cbs, nullptr));

  ESP_ERROR_CHECK(rmt_enable(rxChannel));
}

static bool receiveSymbols(std::vector<rmt_symbol_word_t> &outSymbols) {
  if (!rxInProgress) {
    rxDone = false;
    rxNumSymbols = 0;

    rmt_receive_config_t cfg = {};
    cfg.signal_range_min_ns = 1000;        // Ignore pulses shorter than 1 us
    cfg.signal_range_max_ns = 60000000;    // Must stay below driver max (~65.535 ms)

    esp_err_t err = rmt_receive(rxChannel, rxRawBuffer, sizeof(rxRawBuffer), &cfg);
    if (err != ESP_OK) {
      return false;
    }

    rxInProgress = true;
    return false;
  }

  if (xSemaphoreTake(rxDoneSemaphore, pdMS_TO_TICKS(5)) != pdTRUE) {
    if (millis() - lastRxStatusMs > 1000) {
      Serial.println("RX waiting for signal...");
      lastRxStatusMs = millis();
    }
    return false;
  }

  rxInProgress = false;
  if (!rxDone || rxNumSymbols == 0 || rxNumSymbols > RX_MEM_SYMBOLS) {
    rxDone = false;
    rxNumSymbols = 0;
    return false;
  }

  outSymbols.assign(rxRawBuffer, rxRawBuffer + rxNumSymbols);
  Serial.print("RX frame captured, symbols: ");
  Serial.println(static_cast<unsigned int>(rxNumSymbols));
  rxDone = false;
  rxNumSymbols = 0;
  return true;
}

static int durationToHalfBits(uint16_t durationUs) {
  // Convert raw pulse duration to half-bit count with rounding.
  int n = (durationUs + (MANCHESTER_HALF_BIT_US / 2)) / MANCHESTER_HALF_BIT_US;
  if (n < 1) {
    n = 1;
  }
  if (n > 32) {
    n = 32;
  }
  return n;
}

static bool decodeManchesterSymbols(const std::vector<rmt_symbol_word_t> &symbols, String &decodedText) {
  decodedText = "";

  // Rebuild an idealized half-bit level stream from captured pulse durations.
  std::vector<uint8_t> halfLevels;
  halfLevels.reserve(symbols.size() * 2);

  for (size_t i = 0; i < symbols.size(); ++i) {
    const rmt_symbol_word_t &s = symbols[i];

    if (s.duration0 > 0) {
      int n0 = durationToHalfBits(s.duration0);
      for (int k = 0; k < n0; ++k) {
        halfLevels.push_back(static_cast<uint8_t>(s.level0));
      }
    }
    if (s.duration1 > 0) {
      int n1 = durationToHalfBits(s.duration1);
      for (int k = 0; k < n1; ++k) {
        halfLevels.push_back(static_cast<uint8_t>(s.level1));
      }
    }
  }

  if (halfLevels.size() < 16) {
    return false;
  }

  auto decodeFromPhase = [&](size_t phase, String &outText) -> bool {
    std::vector<uint8_t> bitStream;
    bitStream.reserve(halfLevels.size() / 2);

    // Decode Manchester from pairs of half-bits.
    // 10 => bit 1
    // 01 => bit 0
    for (size_t i = phase; i + 1 < halfLevels.size(); i += 2) {
      uint8_t a = halfLevels[i];
      uint8_t b = halfLevels[i + 1];

      if (a == 1 && b == 0) {
        bitStream.push_back(1);
      } else if (a == 0 && b == 1) {
        bitStream.push_back(0);
      } else {
        // Skip invalid pair to allow loose re-synchronization.
        continue;
      }
    }

    auto bitsToTextFrom = [&](size_t startBit, String &txt) {
      txt = "";
      size_t p = startBit;
      while (p + 8 <= bitStream.size()) {
        uint8_t value = 0;
        for (int i = 0; i < 8; ++i) {
          value = (value << 1) | bitStream[p + i];
        }
        txt += static_cast<char>(value);
        p += 8;
      }
    };

    auto extractCsvLine = [&](const String &txt, String &lineOut) -> bool {
      int nl = txt.indexOf('\n');
      while (nl >= 0) {
        int start = nl;
        while (start > 0 && txt[start - 1] != '\n') {
          --start;
        }
        String line = txt.substring(start, nl);
        int comma = line.indexOf(',');
        if (comma > 0) {
          lineOut = line + "\n";
          return true;
        }
        nl = txt.indexOf('\n', nl + 1);
      }
      return false;
    };

    // Strategy 1: strict preamble lock then byte decode.
    size_t bitPos = 0;
    while (bitPos + PREAMBLE_BYTES * 8 <= bitStream.size()) {
      bool syncOk = true;
      for (uint8_t p = 0; p < PREAMBLE_BYTES; ++p) {
        uint8_t b = 0;
        for (int i = 0; i < 8; ++i) {
          b = (b << 1) | bitStream[bitPos + p * 8 + i];
        }
        if (b != 0x55) {
          syncOk = false;
          break;
        }
      }
      if (syncOk) {
        String txt;
        bitsToTextFrom(bitPos + PREAMBLE_BYTES * 8, txt);
        if (extractCsvLine(txt, outText)) {
          return true;
        }
      }
      ++bitPos;
    }

    // Strategy 2: no preamble lock, test all byte alignments.
    for (size_t offset = 0; offset < 8; ++offset) {
      String txt;
      bitsToTextFrom(offset, txt);
      if (extractCsvLine(txt, outText)) {
        return true;
      }
    }
    return false;
  };

  String candidate;
  if (decodeFromPhase(0, candidate) || decodeFromPhase(1, candidate)) {
    decodedText = candidate;
    return true;
  }
  return false;
}

static bool parseAndPrint(const String &line) {
  String clean = line;
  clean.trim();
  // Remove leading preamble artifacts (e.g., 'U' from 0x55) and other junk
  // until a likely float start is found.
  int start = 0;
  while (start < clean.length()) {
    char c = clean[start];
    bool floatStart = (c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.' || c == 'n' || c == 'N';
    if (floatStart) break;
    ++start;
  }
  if (start > 0) {
    clean = clean.substring(start);
  }

  int comma = clean.indexOf(',');
  if (comma < 0) {
    return false;
  }

  String tempStr = clean.substring(0, comma);
  String distStr = clean.substring(comma + 1);
  tempStr.trim();
  distStr.trim();

  float temp = 0.0f;
  float dist = 0.0f;
  bool tempIsNan = false;
  bool distIsNan = false;
  if (!parseFloatField(tempStr, temp, tempIsNan) || !parseFloatField(distStr, dist, distIsNan)) {
    Serial.print("RX invalid line: ");
    Serial.println(line);
    return false;
  }

  Serial.print("RX raw line: ");
  Serial.println(clean);
  Serial.println("========== DATA RECEIVED ==========");
  Serial.print("Water Temperature: ");
  if (tempIsNan) {
    Serial.print("NaN");
  } else {
    Serial.print(temp, 2);
  }
  Serial.println(" C");
  Serial.print("Water Distance   : ");
  if (distIsNan) {
    Serial.print("NaN");
  } else {
    Serial.print(dist, 2);
  }
  Serial.println(" cm");
  Serial.println("===================================");
  return true;
}

static bool IRAM_ATTR onRxDone(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *userData) {
  (void)channel;
  (void)userData;

  BaseType_t highTaskWoken = pdFALSE;
  rxNumSymbols = edata->num_symbols;
  rxDone = true;
  xSemaphoreGiveFromISR(rxDoneSemaphore, &highTaskWoken);
  return highTaskWoken == pdTRUE;
}

static bool parseFloatField(const String &s, float &outValue, bool &isNan) {
  String t = s;
  t.trim();
  t.toLowerCase();
  if (t == "nan") {
    isNan = true;
    outValue = NAN;
    return true;
  }

  const char *c = t.c_str();
  char *endPtr = nullptr;
  float v = strtof(c, &endPtr);
  if (endPtr == c || *endPtr != '\0') {
    return false;
  }

  isNan = false;
  outValue = v;
  return true;
}
