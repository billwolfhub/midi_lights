// bluefruit_midi_lights — USB MIDI -> NeoPixels on a Circuit Playground Bluefruit
//
// Drives two outputs with the same effects:
//   - the 10 onboard pixels (ring)
//   - an external 5V WS2811 strip of 50 lamps on A1
//
// Wiring (strip): A1 -> 330R -> strip DIN, GND -> strip GND, strip + -> 5V.
// On USB only, strip + can come from VOUT with STRIP_BRIGHTNESS kept low.
// With an external 5V supply: supply + to strip +, supply - to strip GND AND
// Bluefruit GND. Do not connect the supply to VOUT.
//
// Effects:
//   CC 21 / 22 / 3   -> spinning comets; value shifts hue and sets speed
//   Ch10 note 36/38  -> kick / snare flashes at the strip ends
//   Ch3 notes        -> strum flashes; pitch sets colour and position

#include <Adafruit_CircuitPlayground.h>
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>

// ---------- Hardware ----------
const int     STRIP_PIN        = A1;
const int     STRIP_PIXELS     = 50;
const int     RING_PIXELS      = 10;
const uint8_t STRIP_BRIGHTNESS = 40;    // low for USB power; raise once the 5V supply is in
const uint8_t RING_BRIGHTNESS  = 200;

// Colour order: NEO_GRB if the strip test showed red, green, blue;
// NEO_BRG if it showed red, blue, green.
#define STRIP_ORDER NEO_GRB

const int MAX_PIXELS = (STRIP_PIXELS > RING_PIXELS) ? STRIP_PIXELS : RING_PIXELS;

Adafruit_CPlay_NeoPixel extStrip(STRIP_PIXELS, STRIP_PIN, STRIP_ORDER + NEO_KHZ800);

// ---------- Timing / look ----------
const float FADE_MS        = 1500.0f;
const int   IDLE_TIMEOUT   = 100;
const float COMET_TAIL     = 0.4f;   // fraction of the display length
const unsigned long FRAME_MS = 8;    // ~120 fps cap leaves time for MIDI

const float HUE_CC21_CENTER = 0.05f;
const float HUE_CC22_CENTER = 0.55f;
const float HUE_CC3_CENTER  = 0.80f;
const float HUE_RANGE       = 0.15f;

const float KICK_DECAY_MS  = 300.0f;
const float SNARE_DECAY_MS = 200.0f;
const float STRUM_DECAY_MS = 600.0f;

// Positions are fractions of the display length, so the same layout works
// on the 10-pixel ring and the 50-lamp strip.
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
  float   pos;        // 0..1 along the display
  uint8_t channel;
  uint8_t note;
  bool    active;
  bool    holding;
};

const int MAX_FLASHES = 12;
NoteFlash flashes[MAX_FLASHES];

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
void onControlChange(byte channel, byte cc, byte value) {
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
  if (velocity == 0) { onNoteOff(channel, note, 0); return; }

  float norm = velocity / 127.0f;
  NoteFlash* f = allocFlash(channel, note);

  if (channel == DRUM_CHANNEL && note == NOTE_KICK) {
    f->hue        = 0.05f;
    f->sat        = 1.0f;
    f->brightness = 0.5f + norm * 0.5f;
    f->decayRate  = 1.0f / (KICK_DECAY_MS / 1000.0f);
    f->pos        = POS_KICK;
  } else if (channel == DRUM_CHANNEL && note == NOTE_SNARE) {
    f->hue        = 0.0f;
    f->sat        = 0.1f;   // near-white
    f->brightness = 0.4f + norm * 0.6f;
    f->decayRate  = 1.0f / (SNARE_DECAY_MS / 1000.0f);
    f->pos        = POS_SNARE;
  } else if (channel == STRUM_CHANNEL) {
    float span    = clampf((note - STRUM_LOW_NOTE) / (float)STRUM_SPAN, 0.0f, 1.0f);
    f->hue        = wrapf((note - STRUM_LOW_NOTE) / (float)STRUM_SPAN);
    f->sat        = 1.0f;
    f->brightness = 0.3f + norm * 0.7f;
    f->decayRate  = 1.0f / (STRUM_DECAY_MS / 1000.0f);
    f->pos        = POS_STRUM_LO + span * (POS_STRUM_HI - POS_STRUM_LO);
  } else {
    return;   // other drum notes: ignored for now
  }
  f->channel = channel;
  f->note    = note;
  f->holding = true;
  f->active  = true;
}

// Release only the flash for this channel+note.
void onNoteOff(byte channel, byte note, byte velocity) {
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

// Render all layers into one output. `wraps` = true for the ring (circular),
// false for the strip (flash clusters are clipped at the ends).
void render(Adafruit_CPlay_NeoPixel &out, int n, bool wraps) {
  float r[MAX_PIXELS] = {};
  float g[MAX_PIXELS] = {};
  float b[MAX_PIXELS] = {};
  float cr, cg, cb;

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
      r[p] += cr; g[p] += cg; b[p] += cb;
    }
  }

  // Note flash layer: centre pixel plus dimmer neighbours
  for (int i = 0; i < MAX_FLASHES; i++) {
    NoteFlash &f = flashes[i];
    if (!f.active) continue;
    int center = (int)(f.pos * n + 0.5f);
    if (center >= n) center = n - 1;
    for (int k = -1; k <= 1; k++) {
      int p = center + k;
      if (wraps) p = (p + n) % n;
      else if (p < 0 || p >= n) continue;
      float weight = (k == 0) ? 1.0f : 0.35f;
      hsvToRgb(f.hue, f.sat, f.brightness * weight, cr, cg, cb);
      r[p] += cr; g[p] += cg; b[p] += cb;
    }
  }

  for (int p = 0; p < n; p++) {
    out.setPixelColor(p,
      (uint8_t)min((int)r[p], 255),
      (uint8_t)min((int)g[p], 255),
      (uint8_t)min((int)b[p], 255));
  }
  out.show();
}

// ---------- Setup / loop ----------
void setup() {
  CircuitPlayground.begin();
  CircuitPlayground.strip.setBrightness(RING_BRIGHTNESS);

  extStrip.begin();
  extStrip.setBrightness(STRIP_BRIGHTNESS);
  extStrip.clear();
  extStrip.show();

  Serial.begin(115200);

  usb_midi.begin();
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.turnThruOff();
  MIDI.setHandleControlChange(onControlChange);
  MIDI.setHandleNoteOn(onNoteOn);
  MIDI.setHandleNoteOff(onNoteOff);

  while (!USBDevice.mounted()) delay(1);

  Serial.println("Bluefruit MIDI lights booted");

  CircuitPlayground.strip.clear();
  CircuitPlayground.strip.show();
}

void loop() {
  while (MIDI.read()) {}

  static unsigned long lastFrame = 0;
  unsigned long now = millis();
  if (now - lastFrame < FRAME_MS) return;

  float dt = (lastFrame == 0) ? 0.001f : (now - lastFrame) / 1000.0f;
  lastFrame = now;

  updateState(now, dt);
  render(CircuitPlayground.strip, RING_PIXELS, true);
  render(extStrip, STRIP_PIXELS, false);
}
