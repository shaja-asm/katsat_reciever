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

struct LoRaReceiverConfig {
  float frequencyMHz;
  float bandwidthkHz;
  uint8_t spreadingFactor;
  uint8_t codingRateDenom;
  uint8_t syncWord;
  int8_t txPowerdBm;
  uint16_t preambleLength;
  uint32_t sendIntervalMs;
};

// Must match transmitter exactly.
const LoRaReceiverConfig kReceiverLoRaConfig = {
    .frequencyMHz = 434.0,
    .bandwidthkHz = 125.0,
    .spreadingFactor = 12,
    .codingRateDenom = 8,
    .syncWord = 0x12,
    .txPowerdBm = 17,
    .preambleLength = 12,
    .sendIntervalMs = 10000,
};

SX1278 radio = new Module(PIN_LORA_CS, PIN_LORA_DIO0, PIN_LORA_RST, PIN_LORA_DIO1);

volatile bool receivedFlag = false;

long lastSeq = -1;
uint32_t okCount = 0;
uint32_t crcCount = 0;
uint32_t missedCount = 0;
uint32_t lastHeartbeat = 0;
uint32_t lastPacketMillis = 0;
uint32_t learnedPacketIntervalMs = 0;
unsigned long lastTxUptimeMs = 0;
bool overduePacketWarningActive = false;

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

uint32_t expectedPacketIntervalMs() {
  if (learnedPacketIntervalMs != 0) {
    return learnedPacketIntervalMs;
  }
  return kReceiverLoRaConfig.sendIntervalMs;
}

uint32_t overduePacketThresholdMs() {
  // Allow some slack for airtime, scheduler jitter, and clock drift.
  const uint32_t expectedMs = expectedPacketIntervalMs();
  const uint32_t slackMs = (expectedMs / 4U > 1500U) ? (expectedMs / 4U) : 1500U;
  return expectedMs + slackMs;
}

bool beginRadio() {
  Logger::info("PINS", "Nano -> Ra-02");
  Logger::info("PINS", "D13=SCK, D12=MISO, D11=MOSI, D10=NSS, D9=RST, D2=DIO0, D3=DIO1");

  Logger::info("CFG", "Frequency: %.3f MHz", kReceiverLoRaConfig.frequencyMHz);
  Logger::info("CFG", "Bandwidth: %.1f kHz", kReceiverLoRaConfig.bandwidthkHz);
  Logger::info("CFG", "Spreading Factor: %u", kReceiverLoRaConfig.spreadingFactor);
  Logger::info("CFG", "Coding Rate: 4/%u", kReceiverLoRaConfig.codingRateDenom);
  Logger::info("CFG", "Sync Word: 0x%02X", kReceiverLoRaConfig.syncWord);
  Logger::info("CFG", "Preamble Length: %u", kReceiverLoRaConfig.preambleLength);
  Logger::info("CFG", "Nominal TX interval: %lu ms",
               static_cast<unsigned long>(kReceiverLoRaConfig.sendIntervalMs));

  const int16_t state = radio.begin(
      kReceiverLoRaConfig.frequencyMHz,
      kReceiverLoRaConfig.bandwidthkHz,
      kReceiverLoRaConfig.spreadingFactor,
      kReceiverLoRaConfig.codingRateDenom,
      kReceiverLoRaConfig.syncWord,
      kReceiverLoRaConfig.txPowerdBm,
      kReceiverLoRaConfig.preambleLength);

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
  if (millis() - lastHeartbeat >= expectedPacketIntervalMs()) {
    Logger::info("HEARTBEAT", "alive, waiting for packets... ok=%lu missed=%lu crc=%lu",
                 static_cast<unsigned long>(okCount),
                 static_cast<unsigned long>(missedCount),
                 static_cast<unsigned long>(crcCount));
    lastHeartbeat = millis();
  }
}

void checkForOverduePacket() {
  if (lastPacketMillis == 0) {
    return;
  }

  const uint32_t elapsedMs = millis() - lastPacketMillis;
  if (elapsedMs < overduePacketThresholdMs()) {
    if (overduePacketWarningActive) {
      Logger::info("RX", "Packet cadence back to normal");
      overduePacketWarningActive = false;
    }
    return;
  }

  if (!overduePacketWarningActive) {
    Logger::warn("RX", "No packet for %lu ms, expected one every %lu ms",
                 static_cast<unsigned long>(elapsedMs),
                 static_cast<unsigned long>(expectedPacketIntervalMs()));
    overduePacketWarningActive = true;
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
    const uint32_t now = millis();

    Logger::info("RX", "Packet received");
    Logger::info("RX", "Payload: %s", payload.c_str());
    Logger::info("RX", "RSSI: %.2f dBm", radio.getRSSI());
    Logger::info("RX", "SNR: %.2f dB", radio.getSNR());

    const float feiHz = radio.getFrequencyError();
    Logger::info("RX", "Freq offset estimate: %.2f Hz", feiHz);

    if (fabs(feiHz) > 5000.0f) {
      Logger::warn("RX", "Large frequency offset detected: %.2f Hz", feiHz);
    }

    if (lastPacketMillis != 0) {
      const uint32_t packetGapMs = now - lastPacketMillis;
      Logger::info("RX", "Packet gap: %lu ms", static_cast<unsigned long>(packetGapMs));
    }

    lastPacketMillis = now;
    overduePacketWarningActive = false;

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
      if (lastTxUptimeMs != 0 && uptimeMs > lastTxUptimeMs) {
        learnedPacketIntervalMs = static_cast<uint32_t>(uptimeMs - lastTxUptimeMs);
      }
      lastTxUptimeMs = uptimeMs;

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
  checkForOverduePacket();
  handlePacket();
  delay(2);
}

}  // namespace Communication
