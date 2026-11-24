// qtpy_strip_test — continuous strip test for the QT Py RP2040.
// All 50 lamps cycle red -> green -> blue at low brightness, forever,
// so you have time to check wiring and measure voltages.
// The QT Py's onboard light shows the same colour as the strip should.
// Upload with Tools -> USB Stack -> "Pico SDK" or "Adafruit TinyUSB" (either works).

#include <Adafruit_NeoPixel.h>

const int STRIP_PIN    = A3;   // data pin under test
const int STRIP_PIXELS = 50;

Adafruit_NeoPixel strip(STRIP_PIXELS, STRIP_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel status(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

void setup() {
#if defined(NEOPIXEL_POWER)
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);
#endif
  status.begin();
  status.setBrightness(20);
  strip.begin();
  strip.setBrightness(20);
}

void loop() {
  uint32_t colors[] = { 0xFF0000, 0x00FF00, 0x0000FF };
  for (int c = 0; c < 3; c++) {
    strip.fill(colors[c]);
    strip.show();
    status.setPixelColor(0, colors[c]);
    status.show();
    delay(1500);
  }
}
