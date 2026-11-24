// i2c_scan — lists I2C devices on both QT Py RP2040 buses.
//   Wire  = header pins SDA / SCL
//   Wire1 = STEMMA QT (white) connector
// Open Tools -> Serial Monitor (115200). Rescans every 3 seconds.
// The 14-segment displays should show up as 0x70 and 0x71.

#include <Wire.h>

void scan(TwoWire &bus, const char *name) {
  int found = 0;
  Serial.print(name); Serial.print(": ");
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    bus.beginTransmission(addr);
    if (bus.endTransmission() == 0) {
      Serial.print("0x"); if (addr < 16) Serial.print('0');
      Serial.print(addr, HEX); Serial.print(' ');
      found++;
    }
  }
  if (!found) Serial.print("nothing found");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire1.begin();
}

void loop() {
  scan(Wire,  "Header SDA/SCL (Wire) ");
  scan(Wire1, "STEMMA QT     (Wire1)");
  Serial.println();
  delay(3000);
}
