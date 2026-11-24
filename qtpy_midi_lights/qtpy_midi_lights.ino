// qtpy_midi_lights — USB MIDI -> NeoPixel/WS2811 lights on an Adafruit QT Py RP2040
//
// Arduino IDE: Board "Adafruit QT Py RP2040", Tools -> USB Stack -> "Adafruit TinyUSB".
//
// OUTPUT PORTS — each is a connector with +5V, data and GND:
//   port 0 = A3   the 50-lamp strip (comets + strum/kick/snare flashes)
//   port 1 = A2   rings
//   port 2 = A1   rings
//   port 3 = A0   rings
// Data: pin -> 74AHCT125 (one channel per port) -> 330R -> device DIN.
// Several rings can be chained on one port (DOUT of one -> DIN of the next);
// give each its own entry in RINGS[] with the right `first` offset.
// Unused ports and missing pixels are harmless: data just goes nowhere.
//
// Power: on USB only, keep brightness low. With the external 5V supply:
// supply + to every port's +5V, supply - to every port's GND AND QT Py GND.
// Never connect the supply to the QT Py 5V pin while USB is plugged in.
//
// Onboard NeoPixel = status: blue at boot, then flickers white on MIDI input.
//
// Strip effects (unchanged):
//   CC 21 / 22 / 3   -> travelling comets; value shifts hue and sets speed
//   Ch10 note 36/38  -> kick / snare flashes at the strip ends
//   Ch3 notes        -> strum flashes; pitch sets colour and position
// Ring effects: see RINGS[] below — each ring flashes on one channel+note.
//
// Optional: one or two Adafruit quad 14-segment displays (HT16K33 backpack)
// on the STEMMA QT port. Left shows the channel ("Ch10" / "cc21"), right
// shows the note name ("D#1", MIDI Monitor naming: C3 = 60) or CC value.
// Second display needs its A0 jumper bridged on the back (address 0x71).
// Libraries: Adafruit LED Backpack, Adafruit GFX, Adafruit BusIO.

#include <Adafruit_TinyUSB.h>
#include <Adafruit_NeoPixel.h>
#include <MIDI.h>
#include <Wire.h>
#include <Adafruit_LEDBackpack.h>

// =====================================================================
// CONFIGURATION — edit this section to match what's plugged in
// =====================================================================

// ---- Ports ----
struct PortConfig {
  int      pin;
  int      pixels;      // total pixels on this port (strip length, or all chained rings)
  neoPixelType order;   // NEO_GRB, NEO_BRG, NEO_GRBW (RGBW rings), ...
  uint8_t  brightness;  // 0..255
};

PortConfig PORTS[] = {
  // pin  pixels  order                    brightness
  {  A3,  50,     NEO_GRB  + NEO_KHZ800,   40 },   // 0: the strip
  {  A2,  24,     NEO_GRB  + NEO_KHZ800,   40 },   // 1: 24-ring
  {  A1,  24,     NEO_GRB  + NEO_KHZ800,   40 },   // 2: 24-ring
  {  A0,  7,      NEO_GRB  + NEO_KHZ800,   40 },   // 3: 7-pixel jewel
};
const int NUM_PORTS = sizeof(PORTS) / sizeof(PORTS[0]);
const int STRIP_PORT = 0;

// ---- Rings: one entry per ring, each triggered by one channel + note ----
enum RingStyle { FLASH, SPIN };

struct RingConfig {
  int       port;       // which port it's on
  int       first;      // first pixel index on that port (0 unless chained)
  int       count;      // pixels in this ring
  uint8_t   channel;    // MIDI channel 1..16
  uint8_t   note;       // MIDI note number (MIDI Monitor's C1 = 36)
  float     hue;        // 0..1 (0 red, 0.08 orange, 0.16 yellow, 0.33 green, 0.55 cyan, 0.66 blue, 0.8 purple)
  float     sat;        // 0 white .. 1 full colour
  float     decayMs;    // fade time after the hit
  RingStyle style;      // FLASH = whole ring; SPIN = bright arc that runs round as it fades
};

RingConfig RINGS[] = {
  // port first count  ch  note  hue    sat   decay  style
  {  1,   0,    24,    10, 36,   0.05f, 1.0f, 350,   FLASH },  // kick
  {  2,   0,    24,    10, 38,   0.00f, 0.1f, 250,   SPIN  },  // snare (near-white)
  {  3,   0,    7,     10, 42,   0.55f, 0.8f, 150,   FLASH },  // closed hi-hat
};
const int NUM_RINGS = sizeof(RINGS) / sizeof(RINGS[0]);

// ---- Power ----
// Software current limit across ALL ports, in mA. Frames that would draw more
// are scaled down. ~400 on USB; ~3500 for a 4A external supply.
const int   MAX_TOTAL_MA   = 400;
const float MA_PER_CHANNEL = 20.0f;     // per colour channel at full value
const float MA_IDLE        = 1.0f;      // per pixel, chips switched on

// ---- 14-segment displays ----
// STEMMA QT connector on the QT Py RP2040 = Wire1. If you wire the displays
// to the SDA/SCL header pins instead, change this to &Wire.
TwoWire* const DISPLAY_BUS   = &Wire1;  // STEMMA QT connector (use &Wire for the header SDA/SCL pins)
const uint8_t  DISPLAY_LEFT  = 0x70;   // default address
const uint8_t  DISPLAY_RIGHT = 0x71;   // A0 jumper bridged
const uint8_t  DISPLAY_BRIGHTNESS = 8; // 0..15
const unsigned long DISPLAY_MS = 50;   // max ~20 updates/sec
const bool     SHOW_NOTE_OFFS = false; // true to show note-offs too

// Display style:
//   DISPLAY_TEXT  = channel + note name ("Ch10" "D#1")
//   DISPLAY_ALIEN = every note has its own made-up glyph; hits scroll in
//                   from the right like an alien script being written.
enum DisplayStyle { DISPLAY_TEXT, DISPLAY_ALIEN };
const DisplayStyle DISPLAY_STYLE = DISPLAY_ALIEN;
const bool     ALIEN_IDLE_CHATTER = true;   // glyphs murmur quietly when idle
const unsigned long ALIEN_IDLE_MS = 4000;   // idle time before chatter starts

// =====================================================================

const int MAX_PORT_PIXELS = 64;   // buffer size per port

Adafruit_NeoPixel* ports[NUM_PORTS];
float rBuf[NUM_PORTS][MAX_PORT_PIXELS];
float gBuf[NUM_PORTS][MAX_PORT_PIXELS];
float bBuf[NUM_PORTS][MAX_PORT_PIXELS];
float wBuf[NUM_PORTS][MAX_PORT_PIXELS];   // white channel for RGBW rings

#if defined(PIN_NEOPIXEL)
Adafruit_NeoPixel status(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
#define HAS_STATUS 1
#endif

// ---------- Strip timing / look ----------
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

struct RingState {
  float brightness;   // 0..1, decays after a hit
  float angle;        // 0..1, for SPIN
};
RingState ringState[NUM_RINGS];

unsigned long lastMidiMs = 0;   // for the status pixel

// ---------- Displays ----------
Adafruit_AlphaNum4 dispLeft, dispRight;
bool hasLeft = false, hasRight = false;

struct DisplayEvent {
  bool    pending;
  bool    isCC;
  uint8_t channel;
  uint8_t number;   // note or CC number
  uint8_t value;    // velocity or CC value
};
DisplayEvent dispEvent = { false, false, 0, 0, 0 };

// ---- Alien script ----
// Segment bits (Adafruit layout): A0 B1 C2 D3 E4 F5 G1 6 G2 7 H8 J9 K10 L11 M12 N13
// Glyphs are left/right mirror-symmetric, which makes them read like a script.
const uint16_t SEG_A = 1<<0, SEG_D = 1<<3, SEG_J = 1<<9, SEG_M = 1<<12;
struct SegPair { uint16_t left, right; };
const SegPair MIRROR[] = {
  { 1<<5,  1<<1  },   // F  | B   upper verticals
  { 1<<4,  1<<2  },   // E  | C   lower verticals
  { 1<<6,  1<<7  },   // G1 | G2  middle bars
  { 1<<8,  1<<10 },   // H  | K   upper diagonals
  { 1<<11, 1<<13 },   // L  | N   lower diagonals
};

uint32_t hash32(uint32_t x) {
  x ^= x >> 16; x *= 0x7feb352d; x ^= x >> 15; x *= 0x846ca68b; x ^= x >> 16;
  return x;
}

// Same input always gives the same glyph, so each note has a fixed "letter".
uint16_t alienGlyph(uint32_t seed) {
  for (uint32_t salt = 0; ; salt++) {
    uint32_t h = hash32(seed * 2654435761u + salt * 97u + 12345u);
    uint16_t g = 0;
    for (int i = 0; i < 5; i++)
      if (h & (1u << i)) g |= MIRROR[i].left | MIRROR[i].right;
    if (h & (1u << 5)) g |= SEG_A;
    if (h & (1u << 6)) g |= SEG_D;
    if (h & (1u << 7)) g |= SEG_J;
    if (h & (1u << 8)) g |= SEG_M;
    int bits = __builtin_popcount(g);
    // keep it legible: not too sparse, not a solid block, and not a real letter-ish 8
    if (bits >= 4 && bits <= 9) return g;
  }
}

uint16_t alienLine[8];          // what's on the two displays, left to right
uint16_t alienQueue[8];         // glyphs waiting to scroll in
int      alienQueued = 0;
unsigned long lastAlienInput = 0;

void noteDisplay(bool isCC, uint8_t channel, uint8_t number, uint8_t value) {
  if (DISPLAY_STYLE == DISPLAY_ALIEN) {
    if (isCC || value == 0) return;            // notes only
    if (alienQueued < 8) alienQueue[alienQueued++] = alienGlyph(channel * 128u + number);
    lastAlienInput = millis();
    return;
  }
  dispEvent = { true, isCC, channel, number, value };   // latest wins
}

void drawAlien() {
  int width = (hasLeft && hasRight) ? 8 : 4;
  int offset = 8 - width;                      // show the newest glyphs
  Adafruit_AlphaNum4 *a = hasLeft ? &dispLeft : &dispRight;
  for (int i = 0; i < width; i++) {
    Adafruit_AlphaNum4 *d = (width == 8 && i >= 4) ? &dispRight : a;
    d->writeDigitRaw(i % 4, alienLine[offset + i]);
  }
  if (hasLeft)  dispLeft.writeDisplay();
  if (hasRight) dispRight.writeDisplay();
}

void updateAlien(unsigned long now) {
  static unsigned long lastDraw = 0, lastChatter = 0;
  if (now - lastDraw < DISPLAY_MS) return;

  if (alienQueued > 0) {
    // scroll everything left and append the queued glyphs
    int n = alienQueued;
    for (int i = 0; i < 8; i++) alienLine[i] = (i + n < 8) ? alienLine[i + n] : alienQueue[i + n - 8];
    alienQueued = 0;
    lastDraw = now;
    drawAlien();
    return;
  }

  // idle chatter: now and then one glyph mutates, or the line fades a glyph out
  if (ALIEN_IDLE_CHATTER && now - lastAlienInput > ALIEN_IDLE_MS && now - lastChatter > 700) {
    lastChatter = now;
    uint32_t r = hash32(now);
    int pos = r % 8;
    if ((r >> 8) % 3 == 0) alienLine[pos] = 0;
    else                   alienLine[pos] = alienGlyph(r >> 4);
    lastDraw = now;
    drawAlien();
  }
}

void writeText(Adafruit_AlphaNum4 &d, const char *t) {
  for (int i = 0; i < 4; i++) d.writeDigitAscii(i, t[i] ? t[i] : ' ');
  d.writeDisplay();
}

// MIDI Monitor / DrumStrum naming: C3 = note 60, so octave = n/12 - 2.
void noteName(uint8_t n, char *out) {
  static const char *names[] = { "C ", "C#", "D ", "D#", "E ", "F ",
                                 "F#", "G ", "G#", "A ", "A#", "B " };
  int oct = n / 12 - 2;
  const char *nm = names[n % 12];
  if (oct < 0) snprintf(out, 5, "%c%c-%d", nm[0], nm[1], -oct);
  else         snprintf(out, 5, "%c%c%d ", nm[0], nm[1], oct);
}

void updateDisplays(unsigned long now) {
  static unsigned long lastDraw = 0;
  if (!hasLeft && !hasRight) return;
  if (DISPLAY_STYLE == DISPLAY_ALIEN) { updateAlien(now); return; }
  if (!dispEvent.pending || (!hasLeft && !hasRight)) return;
  if (now - lastDraw < DISPLAY_MS) return;
  lastDraw = now;
  DisplayEvent e = dispEvent;
  dispEvent.pending = false;

  char left[6], right[6];
  if (e.isCC) {
    snprintf(left,  sizeof(left),  "cc%2d", e.number % 100);
    snprintf(right, sizeof(right), "%4d", e.value);
  } else {
    snprintf(left,  sizeof(left),  "Ch%2d", e.channel);
    noteName(e.number, right);
  }

  if (hasLeft && hasRight) {
    writeText(dispLeft, left);
    writeText(dispRight, right);
  } else {
    // one display: channel + compact note, e.g. "3D#1", "10C1"; CC: "2187"
    char one[12], compact[6]; int k = 0;
    for (int i = 0; right[i] && k < 5; i++) if (right[i] != ' ') compact[k++] = right[i];
    compact[k] = 0;
    if (e.isCC) snprintf(one, sizeof(one), "%2d%2d", e.number % 100, e.value % 100);
    else        snprintf(one, sizeof(one), "%d%s", e.channel, compact);
    one[4] = 0;
    writeText(hasLeft ? dispLeft : dispRight, one);
  }
}

// ---------- Helpers ----------
float wrapf(float v) {
  while (v < 0) v += 1.0f;
  while (v >= 1.0f) v -= 1.0f;
  return v;
}

float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

bool isRGBW(neoPixelType t) {
  // Adafruit encodes RGBW orders with the white offset differing from red's.
  return ((t >> 6) & 0b11) != ((t >> 4) & 0b11);
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

// Add a colour into a port buffer. On RGBW ports, the white-ish part goes to W.
void addPixel(int port, int p, float h, float s, float v) {
  if (p < 0 || p >= PORTS[port].pixels || p >= MAX_PORT_PIXELS) return;
  float r, g, b;
  if (isRGBW(PORTS[port].order) && s < 0.5f) {
    // low saturation: use the dedicated white LED plus a tint
    float tint = s / 0.5f;
    hsvToRgb(h, 1.0f, v * tint, r, g, b);
    wBuf[port][p] += v * (1.0f - tint) * 255;
  } else {
    hsvToRgb(h, s, v, r, g, b);
  }
  rBuf[port][p] += r; gBuf[port][p] += g; bBuf[port][p] += b;
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
  noteDisplay(true, channel, cc, value);
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

void triggerRings(byte channel, byte note, byte velocity) {
  float norm = velocity / 127.0f;
  for (int i = 0; i < NUM_RINGS; i++) {
    if (RINGS[i].channel == channel && RINGS[i].note == note) {
      ringState[i].brightness = 0.35f + norm * 0.65f;
      ringState[i].angle      = 0;
    }
  }
}

void onNoteOn(byte channel, byte note, byte velocity) {
  lastMidiMs = millis();
  if (velocity == 0) { onNoteOff(channel, note, 0); return; }
  noteDisplay(false, channel, note, velocity);

  triggerRings(channel, note, velocity);

  if (channel != DRUM_CHANNEL && channel != STRUM_CHANNEL) return;
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
    return;   // other drum notes: rings only
  }

  NoteFlash* f = allocFlash(channel, note);
  f->hue = hue; f->sat = sat; f->brightness = bright;
  f->decayRate = decay; f->pos = pos;
  f->channel = channel; f->note = note;
  f->holding = true; f->active = true;
}

// Release only the strip flash for this channel+note. Rings ignore note-off:
// drum hits are short, so rings always fade from the moment of the hit.
void onNoteOff(byte channel, byte note, byte velocity) {
  lastMidiMs = millis();
  if (SHOW_NOTE_OFFS) noteDisplay(false, channel, note, 0);
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

  for (int i = 0; i < NUM_RINGS; i++) {
    RingState &s = ringState[i];
    if (s.brightness <= 0) continue;
    s.brightness -= dt / (RINGS[i].decayMs / 1000.0f);
    if (s.brightness < 0) s.brightness = 0;
    s.angle = wrapf(s.angle + dt * 2.0f);   // SPIN: ~2 turns per second
  }
}

void renderStrip() {
  const int port = STRIP_PORT;
  const int n = PORTS[port].pixels;

  // CC comet layer
  for (int i = 0; i < NUM_CHANNELS; i++) {
    CCChannel &ch = channels[i];
    if (ch.brightness < 0.001f) continue;
    for (int p = 0; p < n; p++) {
      float dist = wrapf(p / (float)n - ch.spinAngle);
      if (dist >= COMET_TAIL) continue;
      float comet = 1.0f - (dist / COMET_TAIL);
      comet *= comet;
      addPixel(port, p, ch.targetHue, 1.0f, comet * ch.brightness);
    }
  }

  // Note flash layer: centre lamp plus dimmer neighbours (clipped at the ends)
  for (int i = 0; i < MAX_FLASHES; i++) {
    NoteFlash &f = flashes[i];
    if (!f.active) continue;
    int center = (int)(f.pos * n + 0.5f);
    if (center >= n) center = n - 1;
    for (int k = -1; k <= 1; k++) {
      float weight = (k == 0) ? 1.0f : 0.35f;
      addPixel(port, center + k, f.hue, f.sat, f.brightness * weight);
    }
  }
}

void renderRings() {
  for (int i = 0; i < NUM_RINGS; i++) {
    const RingConfig &rc = RINGS[i];
    const RingState  &s  = ringState[i];
    if (s.brightness <= 0) continue;
    for (int k = 0; k < rc.count; k++) {
      float v = s.brightness;
      if (rc.style == SPIN) {
        float dist = wrapf(k / (float)rc.count - s.angle);
        float arc  = (dist < 0.35f) ? 1.0f - dist / 0.35f : 0.0f;
        v *= 0.15f + 0.85f * arc * arc;   // dim ring with a bright running arc
      }
      addPixel(rc.port, rc.first + k, rc.hue, rc.sat, v);
    }
  }
}

// Clamp all buffers, apply the global current limit, push to the pixels.
void flush() {
  float estMa = 0, idleMa = 0;
  for (int pt = 0; pt < NUM_PORTS; pt++) {
    int n = min(PORTS[pt].pixels, MAX_PORT_PIXELS);
    float bscale = PORTS[pt].brightness / 255.0f;
    float sum = 0;
    for (int p = 0; p < n; p++) {
      rBuf[pt][p] = min(rBuf[pt][p], 255.0f);
      gBuf[pt][p] = min(gBuf[pt][p], 255.0f);
      bBuf[pt][p] = min(bBuf[pt][p], 255.0f);
      wBuf[pt][p] = min(wBuf[pt][p], 255.0f);
      sum += rBuf[pt][p] + gBuf[pt][p] + bBuf[pt][p] + wBuf[pt][p];
    }
    estMa  += sum / 255.0f * MA_PER_CHANNEL * bscale;
    idleMa += MA_IDLE * n;
  }
  estMa += idleMa;

  float scale = 1.0f;
  if (estMa > MAX_TOTAL_MA && estMa > idleMa)
    scale = max(0.0f, (MAX_TOTAL_MA - idleMa) / (estMa - idleMa));

  for (int pt = 0; pt < NUM_PORTS; pt++) {
    int n = min(PORTS[pt].pixels, MAX_PORT_PIXELS);
    bool rgbw = isRGBW(PORTS[pt].order);
    for (int p = 0; p < n; p++) {
      uint8_t r = rBuf[pt][p] * scale, g = gBuf[pt][p] * scale, b = bBuf[pt][p] * scale;
      if (rgbw) ports[pt]->setPixelColor(p, r, g, b, (uint8_t)(wBuf[pt][p] * scale));
      else      ports[pt]->setPixelColor(p, r, g, b);
      rBuf[pt][p] = gBuf[pt][p] = bBuf[pt][p] = wBuf[pt][p] = 0;
    }
    ports[pt]->show();
  }
}

void updateStatus(unsigned long now) {
#ifdef HAS_STATUS
  static uint32_t lastColor = 0xFFFFFFFF;
  uint32_t c = (now - lastMidiMs < 40) ? status.Color(30, 30, 30)   // MIDI activity
                                       : status.Color(0, 0, 6);     // idle: dim blue
  if (c != lastColor) { status.setPixelColor(0, c); status.show(); lastColor = c; }
#endif
}

// Boot self-test: a red, green, blue dot runs along every port at once,
// so wiring and colour order can be checked without sending any MIDI.
void selfTest() {
  uint32_t colors[] = { 0xFF0000, 0x00FF00, 0x0000FF };
  int longest = 0;
  for (int pt = 0; pt < NUM_PORTS; pt++) longest = max(longest, PORTS[pt].pixels);
  for (int c = 0; c < 3; c++) {
    for (int p = 0; p < longest; p++) {
      for (int pt = 0; pt < NUM_PORTS; pt++) {
        ports[pt]->clear();
        if (p < PORTS[pt].pixels) ports[pt]->setPixelColor(p, colors[c]);
        ports[pt]->show();
      }
      delay(8);
    }
  }
  for (int pt = 0; pt < NUM_PORTS; pt++) { ports[pt]->clear(); ports[pt]->show(); }
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

  for (int pt = 0; pt < NUM_PORTS; pt++) {
    ports[pt] = new Adafruit_NeoPixel(PORTS[pt].pixels, PORTS[pt].pin, PORTS[pt].order);
    ports[pt]->begin();
    ports[pt]->setBrightness(PORTS[pt].brightness);
    ports[pt]->clear();
    ports[pt]->show();
  }

  Serial.begin(115200);

  DISPLAY_BUS->begin();
  hasLeft  = dispLeft.begin(DISPLAY_LEFT, DISPLAY_BUS);
  hasRight = dispRight.begin(DISPLAY_RIGHT, DISPLAY_BUS);
  if (hasLeft)  { dispLeft.setBrightness(DISPLAY_BRIGHTNESS);  writeText(dispLeft,  "MIDI"); }
  if (hasRight) { dispRight.setBrightness(DISPLAY_BRIGHTNESS); writeText(dispRight, "LITE"); }
  if (DISPLAY_STYLE == DISPLAY_ALIEN && (hasLeft || hasRight)) {
    delay(600);
    for (int i = 0; i < 8; i++) alienLine[i] = alienGlyph(1000 + i);   // greeting
    drawAlien();
  }

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
  updateDisplays(now);

  static unsigned long lastFrame = 0;
  if (now - lastFrame < FRAME_MS) return;
  float dt = (lastFrame == 0) ? 0.001f : (now - lastFrame) / 1000.0f;
  lastFrame = now;

  updateState(now, dt);
  renderStrip();
  renderRings();
  flush();
}
