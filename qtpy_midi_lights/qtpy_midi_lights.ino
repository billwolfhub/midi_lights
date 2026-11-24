// qtpy_midi_lights — USB MIDI -> external NeoPixel/WS2811 strip on an Adafruit QT Py
//
// Works on QT Py RP2040 (primary target) and QT Py SAMD21 / ESP32-S2/S3.
// Arduino IDE: Tools -> USB Stack -> "Adafruit TinyUSB" (RP2040 / SAMD21).
//
// Wiring (strip):
//   QT Py A3 -> 330R -> strip DIN (middle wire)
//   QT Py GND -> strip GND
//   strip +  -> 5V. While testing on USB only, the QT Py 5V pin is fine with
//               STRIP_BRIGHTNESS kept low. With the external 5V supply: supply +
//               to strip +, supply - to strip GND AND QT Py GND. Do not connect
//               the supply to the QT Py 5V pin while USB is plugged in.
//
// Onboard NeoPixel = status: blue at boot, then flickers white on MIDI input.
//
// Effects:
//   CC 21 / 22 / 3   -> travelling comets; value shifts hue and sets speed
//   Ch10 note 36/38  -> kick / snare flashes at the strip ends
//   Ch3 notes        -> strum flashes; pitch sets colour and position

#include <Adafruit_TinyUSB.h>
#include <Adafruit_NeoPixel.h>
#include <MIDI.h>

// ---------- Hardware ----------
const int     STRIP_PIN        = A3;
const int     STRIP_PIXELS     = 50;
const uint8_t STRIP_BRIGHTNESS = 40;    // low for USB power; raise once the 5V supply is in

// Colour order: NEO_GRB if the strip test showed red, green, blue;
// NEO_BRG if it showed red, blue, green.
#define STRIP_ORDER NEO_GRB

// Software current limit for the strip, in mA. Frames that would draw more
// are scaled down. ~400 is safe on USB; set near the supply rating minus
// headroom once on the external supply (e.g. 3500 for a 4A supply).
const int   MAX_STRIP_MA   = 400;
const float MA_PER_CHANNEL = 20.0f;     // per colour channel at full value
const float MA_IDLE        = 1.0f;      // per pixel, chips switched on

Adafruit_NeoPixel strip(STRIP_PIXELS, STRIP_PIN, STRIP_ORDER + NEO_KHZ800);

#if defined(PIN_NEOPIXEL)
Adafruit_NeoPixel status(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
#define HAS_STATUS 1
#endif

// ---------- Timing / look ----------
const float FADE_MS        = 1500.0f;
const int   IDLE_TIMEOUT   = 100;
const float COMET_TAIL     = 0.4f;   // fraction of the strip length
const unsigned long FRAME_MS = 8;    // ~120 fps cap leaves time for MIDI

const float HUE_CC21_CENTER = 0.05f;
const float HUE_CC22_CENTER = 0.55f;
const float HUE_CC3_CENTER  = 0.80f;
const float HUE_RANGE       = 0.15f;

const float KICK_DECAY_MS  = 300.0f;
const float SNARE_DECAY_MS = 200.0f;
const float STRUM_DECAY_MS = 600.0f;

// Positions are fractions of the strip length (0 = input end).
const float POS_KICK      = 0.02f;
const float POS_SNARE     = 0.97f;
const float POS_STRUM_LO  = 0.10f;  // strum notes spread across 0.10..0.90
const float POS_STRUM_HI  = 0.90f;
const int   STRUM_LOW_NOTE = 36;
const int   STRUM_SPAN     = 48;    // 4 octaves

const int STRUM_CHANNEL = 3;
const int DRUM_CHANNEL  = 10;
const int NOTE_KICK     = 36;
const int NOTE_SNARE    = 38;

Adafruit_USBD_MIDI usb_midi;
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

// ---------- State ----------
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
  {  3, HUE_CC3_CENTER,  0, 0, 0, HUE_CC3_CENTER,  0, 0, false },
};
const int NUM_CHANNELS = sizeof(channels) / sizeof(channels[0]);

struct NoteFlash {
  float   hue;
  float   sat;
  float   brightness;
  float   decayRate;
  float   pos;        // 0..1 along the strip
  uint8_t channel;
  uint8_t note;
  bool    active;
  bool    holding;
};

const int MAX_FLASHES = 12;
NoteFlash flashes[MAX_FLASHES];

unsigned long lastMidiMs = 0;   // for the status pixel

// ---------- Helpers ----------
float wrapf(float v) {
  while (v < 0) v += 1.0f;
  while (v >= 1.0f) v -= 1.0f;
  return v;
}

float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// HSV (0..1) -> RGB (0..255 floats)
void hsvToRgb(float h, float s, float v, float &r, float &g, float &b) {
  if (s <= 0) { r = g = b = v * 255; return; }
  h = wrapf(h);
  int   i = (int)(h * 6);
  float f = h * 6 - i;
  float p = v * (1 - s);
  float q = v * (1 - f * s);
  float t = v * (1 - (1 - f) * s);
  switch (i % 6) {
    case 0: r=v; g=t; b=p; break;
    case 1: r=q; g=v; b=p; break;
    case 2: r=p; g=v; b=t; break;
    case 3: r=p; g=q; b=v; break;
    case 4: r=t; g=p; b=v; break;
    default: r=v; g=p; b=q; break;
  }
  r *= 255; g *= 255; b *= 255;
}

// Reuse a flash already playing this channel+note, else a free one, else the dimmest.
NoteFlash* allocFlash(uint8_t channel, uint8_t note) {
  for (int i = 0; i < MAX_FLASHES; i++)
    if (flashes[i].active && flashes[i].channel == channel && flashes[i].note == note)
      return &flashes[i];
  for (int i = 0; i < MAX_FLASHES; i++)
    if (!flashes[i].active) return &flashes[i];
  NoteFlash* dimmest = &flashes[0];
  for (int i = 1; i < MAX_FLASHES; i++)
    if (flashes[i].brightness < dimmest->brightness) dimmest = &flashes[i];
  return dimmest;
}

// ---------- MIDI handlers ----------
void onNoteOff(byte channel, byte note, byte velocity);

void onControlChange(byte channel, byte cc, byte value) {
  lastMidiMs = millis();
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
  lastMidiMs = millis();
  if (channel != DRUM_CHANNEL && channel != STRUM_CHANNEL) return;
  if (velocity == 0) { onNoteOff(channel, note, 0); return; }

  float norm = velocity / 127.0f;

  float hue, sat, bright, decay, pos;
  if (channel == DRUM_CHANNEL && note == NOTE_KICK) {
    hue = 0.05f; sat = 1.0f; bright = 0.5f + norm * 0.5f;
    decay = 1.0f / (KICK_DECAY_MS / 1000.0f);  pos = POS_KICK;
  } else if (channel == DRUM_CHANNEL && note == NOTE_SNARE) {
    hue = 0.0f;  sat = 0.1f; bright = 0.4f + norm * 0.6f;   // near-white
    decay = 1.0f / (SNARE_DECAY_MS / 1000.0f); pos = POS_SNARE;
  } else if (channel == STRUM_CHANNEL) {
    float span = clampf((note - STRUM_LOW_NOTE) / (float)STRUM_SPAN, 0.0f, 1.0f);
    hue = wrapf((note - STRUM_LOW_NOTE) / (float)STRUM_SPAN);
    sat = 1.0f; bright = 0.3f + norm * 0.7f;
    decay = 1.0f / (STRUM_DECAY_MS / 1000.0f);
    pos = POS_STRUM_LO + span * (POS_STRUM_HI - POS_STRUM_LO);
  } else {
    return;   // other drum notes: ignored for now
  }

  NoteFlash* f = allocFlash(channel, note);
  f->hue = hue; f->sat = sat; f->brightness = bright;
  f->decayRate = decay; f->pos = pos;
  f->channel = channel; f->note = note;
  f->holding = true; f->active = true;
}

// Release only the flash for this channel+note.
void onNoteOff(byte channel, byte note, byte velocity) {
  lastMidiMs = millis();
  for (int i = 0; i < MAX_FLASHES; i++) {
    if (flashes[i].active && flashes[i].holding &&
        flashes[i].channel == channel && flashes[i].note == note)
      flashes[i].holding = false;
  }
}

// ---------- Animation ----------
void updateState(unsigned long now, float dt) {
  for (int i = 0; i < NUM_CHANNELS; i++) {
    CCChannel &ch = channels[i];
    if (ch.brightness < 0.001f) continue;
    if (ch.active && (now - ch.lastEventMs) > IDLE_TIMEOUT) ch.active = false;
    if (!ch.active) {
      ch.brightness -= dt / (FADE_MS / 1000.0f);
      if (ch.brightness < 0) ch.brightness = 0;
    }
    ch.spinAngle = wrapf(ch.spinAngle + ch.spinSpeed * dt);
  }

  for (int i = 0; i < MAX_FLASHES; i++) {
    NoteFlash &f = flashes[i];
    if (!f.active) continue;
    if (!f.holding) f.brightness -= f.decayRate * dt;
    if (f.brightness <= 0) f.active = false;
  }
}

float rBuf[STRIP_PIXELS], gBuf[STRIP_PIXELS], bBuf[STRIP_PIXELS];

void render() {
  const int n = STRIP_PIXELS;
  float cr, cg, cb;
  for (int p = 0; p < n; p++) rBuf[p] = gBuf[p] = bBuf[p] = 0;

  // CC comet layer
  for (int i = 0; i < NUM_CHANNELS; i++) {
    CCChannel &ch = channels[i];
    if (ch.brightness < 0.001f) continue;
    for (int p = 0; p < n; p++) {
      float dist = wrapf(p / (float)n - ch.spinAngle);
      if (dist >= COMET_TAIL) continue;
      float comet = 1.0f - (dist / COMET_TAIL);
      comet *= comet;
      hsvToRgb(ch.targetHue, 1.0f, comet * ch.brightness, cr, cg, cb);
      rBuf[p] += cr; gBuf[p] += cg; bBuf[p] += cb;
    }
  }

  // Note flash layer: centre lamp plus dimmer neighbours (clipped at the ends)
  for (int i = 0; i < MAX_FLASHES; i++) {
    NoteFlash &f = flashes[i];
    if (!f.active) continue;
    int center = (int)(f.pos * n + 0.5f);
    if (center >= n) center = n - 1;
    for (int k = -1; k <= 1; k++) {
      int p = center + k;
      if (p < 0 || p >= n) continue;
      float weight = (k == 0) ? 1.0f : 0.35f;
      hsvToRgb(f.hue, f.sat, f.brightness * weight, cr, cg, cb);
      rBuf[p] += cr; gBuf[p] += cg; bBuf[p] += cb;
    }
  }

  // Clamp, estimate current, scale down if over budget
  float total = 0;
  for (int p = 0; p < n; p++) {
    rBuf[p] = min(rBuf[p], 255.0f);
    gBuf[p] = min(gBuf[p], 255.0f);
    bBuf[p] = min(bBuf[p], 255.0f);
    total += rBuf[p] + gBuf[p] + bBuf[p];
  }
  float estMa = total / 255.0f * MA_PER_CHANNEL * (STRIP_BRIGHTNESS / 255.0f)
              + MA_IDLE * n;
  float scale = 1.0f;
  if (estMa > MAX_STRIP_MA) scale = (MAX_STRIP_MA - MA_IDLE * n) / (estMa - MA_IDLE * n);

  for (int p = 0; p < n; p++) {
    strip.setPixelColor(p,
      (uint8_t)(rBuf[p] * scale),
      (uint8_t)(gBuf[p] * scale),
      (uint8_t)(bBuf[p] * scale));
  }
  strip.show();
}

void updateStatus(unsigned long now) {
#ifdef HAS_STATUS
  static uint32_t lastColor = 0xFFFFFFFF;
  uint32_t c = (now - lastMidiMs < 40) ? status.Color(30, 30, 30)   // MIDI activity
                                       : status.Color(0, 0, 6);     // idle: dim blue
  if (c != lastColor) { status.setPixelColor(0, c); status.show(); lastColor = c; }
#endif
}

// Boot self-test: one red, green, blue sweep down the strip, so wiring and
// colour order can be checked without sending any MIDI.
void selfTest() {
  uint32_t colors[] = { strip.Color(255, 0, 0), strip.Color(0, 255, 0), strip.Color(0, 0, 255) };
  for (int c = 0; c < 3; c++) {
    for (int p = 0; p < STRIP_PIXELS; p++) {
      strip.clear();
      strip.setPixelColor(p, colors[c]);
      if (p > 0) strip.setPixelColor(p - 1, colors[c] & 0x3F3F3F);
      strip.show();
      delay(8);
    }
  }
  strip.clear();
  strip.show();
}

// ---------- Setup / loop ----------
void setup() {
  // Cores without built-in TinyUSB support need manual init.
  if (!TinyUSBDevice.isInitialized()) TinyUSBDevice.begin(0);

#ifdef HAS_STATUS
#if defined(NEOPIXEL_POWER)
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);
#endif
  status.begin();
  status.setBrightness(255);
  status.setPixelColor(0, status.Color(0, 0, 40));   // blue: booting
  status.show();
#endif

  strip.begin();
  strip.setBrightness(STRIP_BRIGHTNESS);
  strip.clear();
  strip.show();

  Serial.begin(115200);

  usb_midi.setStringDescriptor("QT Py MIDI Lights");
  usb_midi.begin();
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.turnThruOff();
  MIDI.setHandleControlChange(onControlChange);
  MIDI.setHandleNoteOn(onNoteOn);
  MIDI.setHandleNoteOff(onNoteOff);

  // Re-enumerate so the MIDI interface shows up if the core started USB already.
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  selfTest();
  Serial.println("QT Py MIDI lights ready");
}

void loop() {
#ifdef TINYUSB_NEED_POLLING_TASK
  TinyUSBDevice.task();
#endif

  while (MIDI.read()) {}

  unsigned long now = millis();
  updateStatus(now);

  static unsigned long lastFrame = 0;
  if (now - lastFrame < FRAME_MS) return;
  float dt = (lastFrame == 0) ? 0.001f : (now - lastFrame) / 1000.0f;
  lastFrame = now;

  updateState(now, dt);
  render();
}
