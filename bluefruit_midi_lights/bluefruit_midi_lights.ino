#include <Adafruit_CircuitPlayground.h>
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>

const int     NUM_PIXELS   = 10;
const float   FADE_MS      = 1500.0f;
const int     IDLE_TIMEOUT = 100;

const float HUE_CC21_CENTER = 0.05f;
const float HUE_CC22_CENTER = 0.55f;
const float HUE_CC64_CENTER = 0.80f;
const float HUE_RANGE       = 0.15f;

const float KICK_DECAY_MS  = 300.0f;
const float SNARE_DECAY_MS = 200.0f;
const float STRUM_DECAY_MS = 600.0f;

const int STRUM_CHANNEL = 3;
const int DRUM_CHANNEL  = 10;
const int NOTE_KICK     = 36;
const int NOTE_SNARE    = 38;

Adafruit_USBD_MIDI usb_midi;
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

struct CCChannel {
  int     cc;
  float   hueCenter;
  float   brightness;
  float   spinAngle;
  float   spinSpeed;
  float   targetHue;
  uint8_t lastVal;
  unsigned long lastEventMs;
  bool    active;
};

CCChannel channels[] = {
  { 21, HUE_CC21_CENTER, 0, 0, 0, HUE_CC21_CENTER, 0, 0, false },
  { 22, HUE_CC22_CENTER, 0, 0, 0, HUE_CC22_CENTER, 0, 0, false },
  { 64, HUE_CC64_CENTER, 0, 0, 0, HUE_CC64_CENTER, 0, 0, false },
};
const int NUM_CHANNELS = sizeof(channels) / sizeof(channels[0]);

struct NoteFlash {
  float hue;
  float brightness;
  float decayRate;
  int   notePixel;
  bool  active;
  bool  holding;
};

const int MAX_FLASHES = 6;
NoteFlash flashes[MAX_FLASHES];

NoteFlash* allocFlash() {
  for (int i = 0; i < MAX_FLASHES; i++) {
    if (!flashes[i].active) return &flashes[i];
  }
  NoteFlash* dimmest = &flashes[0];
  for (int i = 1; i < MAX_FLASHES; i++) {
    if (flashes[i].brightness < dimmest->brightness) dimmest = &flashes[i];
  }
  return dimmest;
}

uint32_t hsvToRgb(float h, float s, float v) {
  if (s == 0) {
    uint8_t val = (uint8_t)(v * 255);
    return CircuitPlayground.strip.Color(val, val, val);
  }
  int   i = (int)(h * 6);
  float f = h * 6 - i;
  float p = v * (1 - s);
  float q = v * (1 - f * s);
  float t = v * (1 - (1 - f) * s);
  float r, g, b;
  switch (i % 6) {
    case 0: r=v; g=t; b=p; break;
    case 1: r=q; g=v; b=p; break;
    case 2: r=p; g=v; b=t; break;
    case 3: r=p; g=q; b=v; break;
    case 4: r=t; g=p; b=v; break;
    default: r=v; g=p; b=q; break;
  }
  return CircuitPlayground.strip.Color(
    (uint8_t)(r * 255),
    (uint8_t)(g * 255),
    (uint8_t)(b * 255)
  );
}

float wrapf(float v) {
  while (v < 0) v += 1.0f;
  while (v >= 1.0f) v -= 1.0f;
  return v;
}

void onControlChange(byte channel, byte cc, byte value) {
  Serial.print("CC: "); Serial.print(cc);
  Serial.print(" val: "); Serial.println(value);

  for (int i = 0; i < NUM_CHANNELS; i++) {
    if (channels[i].cc == cc) {
      float norm = value / 127.0f;
      channels[i].targetHue   = wrapf(channels[i].hueCenter + (norm - 0.5f) * HUE_RANGE);
      channels[i].spinSpeed   = 0.3f + norm * 2.5f;
      channels[i].brightness  = 1.0f;
      channels[i].lastVal     = value;
      channels[i].lastEventMs = millis();
      channels[i].active      = true;
    }
  }
}

void onNoteOn(byte channel, byte note, byte velocity) {
  if (channel != DRUM_CHANNEL && channel != STRUM_CHANNEL) return;
  if (velocity == 0) return;

  float norm = velocity / 127.0f;
  NoteFlash* f = allocFlash();

  if (channel == DRUM_CHANNEL && note == NOTE_KICK) {
    f->hue       = 0.05f;
    f->brightness = 0.5f + norm * 0.5f;
    f->decayRate  = 1.0f / (KICK_DECAY_MS / 1000.0f);
    f->notePixel  = NOTE_KICK % NUM_PIXELS;
  } else if (channel == DRUM_CHANNEL && note == NOTE_SNARE) {
    f->hue        = 0.0f;
    f->brightness = 0.4f + norm * 0.6f;
    f->decayRate  = 1.0f / (SNARE_DECAY_MS / 1000.0f);
    f->notePixel  = NOTE_SNARE % NUM_PIXELS;
  } else {
    f->hue        = wrapf((note - 36) / 48.0f);
    f->brightness = 0.3f + norm * 0.7f;
    f->decayRate  = 1.0f / (STRUM_DECAY_MS / 1000.0f);
    f->notePixel  = note % NUM_PIXELS;
  }
  f->holding = true;
  f->active  = true;
}

void onNoteOff(byte channel, byte note, byte velocity) {
  if (channel != DRUM_CHANNEL && channel != STRUM_CHANNEL) return;
  for (int i = 0; i < MAX_FLASHES; i++) {
    if (flashes[i].active && flashes[i].holding)
      flashes[i].holding = false;
  }
}

void setup() {
  CircuitPlayground.begin();
  CircuitPlayground.strip.setBrightness(200);

  Serial.begin(115200);

  usb_midi.begin();
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.turnThruOff();
  MIDI.setHandleControlChange(onControlChange);
  MIDI.setHandleNoteOn(onNoteOn);
  MIDI.setHandleNoteOff(onNoteOff);

  while (!USBDevice.mounted()) delay(1);

  Serial.println("Bluefruit booted");

  for (int i = 0; i < NUM_PIXELS; i++)
    CircuitPlayground.strip.setPixelColor(i, 0);
  CircuitPlayground.strip.show();
}

void loop() {
  while (MIDI.read()) {}

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 2000) {
    Serial.println("loop running");
    lastPrint = millis();
  }

  unsigned long now = millis();
  static unsigned long lastFrame = 0;
  float dt = (now - lastFrame) / 1000.0f;
  if (dt <= 0) dt = 0.001f;
  lastFrame = now;

  float r[NUM_PIXELS] = {};
  float g[NUM_PIXELS] = {};
  float b[NUM_PIXELS] = {};

  // CC comet layer
  for (int i = 0; i < NUM_CHANNELS; i++) {
    CCChannel &ch = channels[i];
    if (ch.brightness < 0.001f) continue;

    if (ch.active && (now - ch.lastEventMs) > IDLE_TIMEOUT)
      ch.active = false;

    if (!ch.active) {
      ch.brightness -= dt / (FADE_MS / 1000.0f);
      if (ch.brightness < 0) ch.brightness = 0;
    }

    ch.spinAngle = wrapf(ch.spinAngle + ch.spinSpeed * dt);

    for (int p = 0; p < NUM_PIXELS; p++) {
      float dist = wrapf(p / (float)NUM_PIXELS - ch.spinAngle);
      float comet = 0;
      if (dist < 0.4f) {
        comet = 1.0f - (dist / 0.4f);
        comet = comet * comet;
      }
      float pixBright = comet * ch.brightness;
      uint32_t color  = hsvToRgb(ch.targetHue, 1.0f, pixBright);
      r[p] += (uint8_t)(color >> 16);
      g[p] += (uint8_t)(color >> 8);
      b[p] += (uint8_t)(color);
    }
  }

  // Note flash layer — 3-pixel cluster
  for (int i = 0; i < MAX_FLASHES; i++) {
    NoteFlash &f = flashes[i];
    if (!f.active) continue;

    if (!f.holding) f.brightness -= f.decayRate * dt;
    if (f.brightness <= 0) { f.active = false; continue; }

    float sat = (f.hue == 0.0f) ? 0.1f : 1.0f;
    int center = f.notePixel;

    for (int p = 0; p < NUM_PIXELS; p++) {
      if (p != center) continue;
      float pixBright = f.brightness;
      uint32_t color = hsvToRgb(f.hue, sat, pixBright);
      r[p] += (uint8_t)(color >> 16);
      g[p] += (uint8_t)(color >> 8);
      b[p] += (uint8_t)(color);
    }
  }

  for (int p = 0; p < NUM_PIXELS; p++) {
    CircuitPlayground.strip.setPixelColor(p,
      min((int)r[p], 255),
      min((int)g[p], 255),
      min((int)b[p], 255)
    );
  }
  CircuitPlayground.strip.show();
}