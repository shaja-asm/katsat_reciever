#pragma once

#include <Arduino.h>

namespace Communication {

bool begin(unsigned long serialBaud = 115200);
void update();

}  // namespace Communication
