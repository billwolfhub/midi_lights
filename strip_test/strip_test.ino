// strip_test — check an external strip on A1 of the Circuit Playground Bluefruit.
// Onboard pixel 0 blinks dim green = sketch is running.
// First 10 strip pixels cycle red -> green -> blue at low brightness.
#include <Adafruit_CircuitPlayground.h>

const int STRIP_PIN    = A1;
const int STRIP_PIXELS = 50;
const int TEST_PIXELS  = 50;   // whole strip; brightness 20 keeps USB draw low

// Uses the Circuit Playground library's own NeoPixel class; including
// Adafruit_NeoPixel.h alongside it causes duplicate-definition errors.
// If colours come out in the wrong order, try NEO_RGB.
// If nothing lights with polarity confirmed on both ends, try NEO_KHZ400.
Adafruit_CPlay_NeoPixel strip(STRIP_PIXELS, STRIP_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  CircuitPlayground.begin();
  CircuitPlayground.strip.setBrightness(30);
  strip.begin();
  strip.setBrightness(20);   // keep it low: powered from USB via VOUT
  strip.clear();
  strip.show();
}

void loop() {
  uint32_t colors[] = { strip.Color(255, 0, 0), strip.Color(0, 255, 0), strip.Color(0, 0, 255) };
  for (int c = 0; c < 3; c++) {
    strip.clear();
    for (int i = 0; i < TEST_PIXELS; i++) strip.setPixelColor(i, colors[c]);
    strip.show();

    CircuitPlayground.setPixelColor(0, (c % 2) ? 0 : 0x002800);  // heartbeat
    delay(1500);
  }
}
