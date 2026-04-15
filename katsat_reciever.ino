#include <Arduino.h>

#include "communication.h"

void setup() {
  if (!Communication::begin(115200)) {
    while (true) {
      delay(1000);
      Serial.println("[FAIL] Check 3.3V, GND, SPI pins, NSS, RST, DIO0, DIO1, and antenna.");
    }
  }
}

void loop() {
  Communication::update();
}
