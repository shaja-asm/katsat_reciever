#include "communication.h"

#include <RadioLib.h>
#include <SPI.h>
#include <stdarg.h>

namespace {

class Logger {
public:
  static void begin(unsigned long baud) {
    Serial.begin(baud);

    const uint32_t start = millis();
    while (!Serial && (millis() - start < 4000)) {
      delay(10);
    }

    delay(200);
    Serial.println();
    Serial.println("==================================================");
    Serial.println("[BOOT] Nano 33 BLE Sense Lite LoRa receiver");
    Serial.println("==================================================");
  }

  static void info(const char* tag, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    log("INFO", tag, fmt, args);
    va_end(args);
  }

  static void warn(const char* tag, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    log("WARN", tag, fmt, args);
    va_end(args);
  }

  static void error(const char* tag, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    log("ERROR", tag, fmt, args);
    va_end(args);
  }

private:
  static void log(const char* level, const char* tag, const char* fmt, va_list args) {
    char message[256];
    vsnprintf(message, sizeof(message), fmt, args);

    Serial.print("[");
    Serial.print((unsigned long)millis());
    Serial.print(" ms] ");
    Serial.print(level);
    Serial.print(" ");
    Serial.print(tag);
    Serial.print(" ");
    Serial.println(message);
  }
};

// Nano 33 BLE Sense Lite <-> Ra-02 wiring
static constexpr int PIN_LORA_CS   = 10;   // NSS
static constexpr int PIN_LORA_RST  = 9;    // RST
static constexpr int PIN_LORA_DIO0 = 2;    // DIO0
static constexpr int PIN_LORA_DIO1 = 3;    // DIO1

/*
3.3V -> 3.3V
GND -> GND
D13 -> SCK
D12 -> MISO
D11 -> MOSI
D10 -> NSS
D9 -> RST
D2 -> DIO0
D3 -> DIO1
*/

// Must match transmitter exactly
static constexpr float    LORA_FREQUENCY_MHZ    = 434.0;
static constexpr float    LORA_BANDWIDTH_KHZ    = 125.0;
static constexpr uint8_t  LORA_SPREADING_FACTOR = 12;
static constexpr uint8_t  LORA_CODING_RATE      = 8;     // 4/8
static constexpr uint8_t  LORA_SYNC_WORD        = 0x12;
static constexpr int8_t   LORA_POWER_DBM        = 17;    // not used for RX, but begin() expects it
static constexpr uint16_t LORA_PREAMBLE_LENGTH  = 12;

SX1278 radio = new Module(PIN_LORA_CS, PIN_LORA_DIO0, PIN_LORA_RST, PIN_LORA_DIO1);

volatile bool receivedFlag = false;

long lastSeq = -1;
uint32_t okCount = 0;
uint32_t crcCount = 0;
uint32_t missedCount = 0;
uint32_t lastHeartbeat = 0;

void setFlag() {
  receivedFlag = true;
}

const char* statusToString(int16_t code) {
  switch (code) {
    case RADIOLIB_ERR_NONE: return "No error";
    case RADIOLIB_ERR_CHIP_NOT_FOUND: return "Chip not found";
    case RADIOLIB_ERR_INVALID_FREQUENCY: return "Invalid frequency";
    case RADIOLIB_ERR_INVALID_BANDWIDTH: return "Invalid bandwidth";
    case RADIOLIB_ERR_INVALID_SPREADING_FACTOR: return "Invalid spreading factor";
    case RADIOLIB_ERR_INVALID_CODING_RATE: return "Invalid coding rate";
    case RADIOLIB_ERR_INVALID_OUTPUT_POWER: return "Invalid output power";
    case RADIOLIB_ERR_CRC_MISMATCH: return "CRC mismatch";
    case RADIOLIB_ERR_SPI_WRITE_FAILED: return "SPI write/read verification failed";
    default: return "Unknown RadioLib status";
  }
}

bool restartReceive() {
  const int16_t state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Logger::error("RX", "startReceive() failed | code=%d | %s", state, statusToString(state));
    return false;
  }
  return true;
}

bool beginRadio() {
  Logger::info("PINS", "Nano -> Ra-02");
  Logger::info("PINS", "D13=SCK, D12=MISO, D11=MOSI, D10=NSS, D9=RST, D2=DIO0, D3=DIO1");

  Logger::info("CFG", "Frequency: %.3f MHz", LORA_FREQUENCY_MHZ);
  Logger::info("CFG", "Bandwidth: %.1f kHz", LORA_BANDWIDTH_KHZ);
  Logger::info("CFG", "Spreading Factor: %u", LORA_SPREADING_FACTOR);
  Logger::info("CFG", "Coding Rate: 4/%u", LORA_CODING_RATE);
  Logger::info("CFG", "Sync Word: 0x%02X", LORA_SYNC_WORD);
  Logger::info("CFG", "Preamble Length: %u", LORA_PREAMBLE_LENGTH);

  const int16_t state = radio.begin(
      LORA_FREQUENCY_MHZ,
      LORA_BANDWIDTH_KHZ,
      LORA_SPREADING_FACTOR,
      LORA_CODING_RATE,
      LORA_SYNC_WORD,
      LORA_POWER_DBM,
      LORA_PREAMBLE_LENGTH);

  if (state != RADIOLIB_ERR_NONE) {
    Logger::error("RADIO", "radio.begin() failed | code=%d | %s", state, statusToString(state));
    return false;
  }

  Logger::info("RADIO", "radio.begin() OK");

  const int16_t crcState = radio.setCRC(true);
  if (crcState != RADIOLIB_ERR_NONE) {
    Logger::error("RADIO", "setCRC(true) failed | code=%d | %s", crcState, statusToString(crcState));
    return false;
  }

  Logger::info("RADIO", "CRC enabled");

  radio.setPacketReceivedAction(setFlag);

  if (!restartReceive()) {
    return false;
  }

  Logger::info("RADIO", "Receiver listening...");
  return true;
}

void logHeartbeat() {
  if (millis() - lastHeartbeat >= 30000) {
    Logger::info("HEARTBEAT", "alive, waiting for packets... ok=%lu missed=%lu crc=%lu",
                 static_cast<unsigned long>(okCount),
                 static_cast<unsigned long>(missedCount),
                 static_cast<unsigned long>(crcCount));
    lastHeartbeat = millis();
  }
}

void handlePacket() {
  if (!receivedFlag) {
    return;
  }

  receivedFlag = false;

  String payload;
  const int16_t state = radio.readData(payload);

  if (state == RADIOLIB_ERR_NONE) {
    okCount++;

    Logger::info("RX", "Packet received");
    Logger::info("RX", "Payload: %s", payload.c_str());
    Logger::info("RX", "RSSI: %.2f dBm", radio.getRSSI());
    Logger::info("RX", "SNR: %.2f dB", radio.getSNR());

    const float feiHz = radio.getFrequencyError();
    Logger::info("RX", "Freq offset estimate: %.2f Hz", feiHz);

    if (fabs(feiHz) > 5000.0f) {
      Logger::warn("RX", "Large frequency offset detected: %.2f Hz", feiHz);
    }

    unsigned long seq = 0;
    unsigned long uptimeMs = 0;
    char msg[40] = {0};

    const int parsed = sscanf(
        payload.c_str(),
        "{\"seq\":%lu,\"uptime_ms\":%lu,\"msg\":\"%39[^\"]\"}",
        &seq,
        &uptimeMs,
        msg);

    if (parsed == 3) {
      if (lastSeq >= 0 && seq > static_cast<unsigned long>(lastSeq + 1)) {
        const unsigned long missed = seq - static_cast<unsigned long>(lastSeq + 1);
        missedCount += missed;
        Logger::warn("RX", "Missed %lu packet(s)", missed);
      }

      lastSeq = static_cast<long>(seq);

      Logger::info("PARSED", "seq=%lu uptime_ms=%lu msg=%s",
                   seq,
                   uptimeMs,
                   msg);
    } else {
      Logger::warn("PARSED", "Could not parse JSON payload");
    }
  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    crcCount++;
    Logger::warn("RX", "CRC mismatch");
  } else {
    Logger::error("RX", "readData() failed | code=%d | %s", state, statusToString(state));
  }

  restartReceive();
}

}  // namespace

namespace Communication {

bool begin(unsigned long serialBaud) {
  Logger::begin(serialBaud);
  Logger::info("SETUP", "setup() entered");

  if (!beginRadio()) {
    Logger::error("SETUP", "Receiver init failed");
    return false;
  }

  Logger::info("SETUP", "setup() completed");
  return true;
}

void update() {
  logHeartbeat();
  handlePacket();
  delay(2);
}

}  // namespace Communication
