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

// ---- MIDI channels (one per lighting category) ----
//   ch 1  STRIP      CC 21/22/3 comets + pitched note flashes along the strip
//   ch 2  DISPLAYS   alien glyphs, held-note brightness, ALL-chord message
//   ch 3  PERCUSSIVE one-shot hits: rings (RINGS[] table) + strip kick/snare
const uint8_t STRIP_CHANNEL      = 1;
const uint8_t DISPLAY_CHANNEL    = 2;
const uint8_t PERCUSSIVE_CHANNEL = 3;

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
  {  A2,  24,     NEO_GRB  + NEO_KHZ800,   80 },   // 1: 24-ring
  {  A1,  144,    NEO_GRB  + NEO_KHZ800,   80 },   // 2: six 24-rings chained (DOUT -> next ring's DIN)
  {  A0,  7,      NEO_GRB  + NEO_KHZ800,   80 },   // 3: 7-pixel jewel
};
const int NUM_PORTS = sizeof(PORTS) / sizeof(PORTS[0]);
const int STRIP_PORT = 0;

// ---- Rings: one entry per ring, each triggered by one channel + note ----
enum RingStyle { FLASH, SPIN, CHASE };

// Shared look for the drum-pad rings (all use these, so they blink the same).
// CHASE = several short bright arcs whipping round the ring, fading fast.
#define   PAD_STYLE   CHASE
#define   PAD_DECAY   220      // ms to fade out
const int   CHASE_ARMS          = 3;      // bright arcs around the ring
const float CHASE_TURNS_PER_SEC = 4.0f;   // rotation speed
const float CHASE_ARC           = 0.25f;  // arc length as a fraction of one arm

// Holding, sustain and aftertouch (rings and strip flashes)
const bool  HOLD_WHILE_HELD   = true;   // rings stay lit and keep animating until note-off
const uint8_t SUSTAIN_CC      = 64;     // sustain pedal: defers note-offs until released
const bool  AFTERTOUCH_ON     = true;   // poly + channel pressure shape held notes
const float AT_MIN_LEVEL      = 0.25f;  // brightness at the lightest pressure
const float AT_SPEED_BOOST    = 1.5f;   // chase/spin speed multiplier added at full pressure

// Velocity-driven colour (rings)
const float VEL_HUE_SHIFT   = 0.05f;  // harder hits push the hue this far (red -> crimson/magenta, green -> yellow-green)
const float VEL_WHITE_FLASH = 0.6f;   // harder hits start whiter ("white-hot"), colour returns as it fades
const float VEL_SPEED_MIN   = 0.6f;   // chase/spin speed at the softest hit...
const float VEL_SPEED_MAX   = 1.4f;   // ...and at the hardest (x the base speed)
const float ARC_HUE_SPREAD  = 0.05f;  // colour drift from the head to the tail of each chase arc

// Stuck-note safety and debugging
const unsigned long MAX_HOLD_MS = 4000;  // held notes auto-release after this (0 = never)
const bool  DEBUG_MIDI        = false;   // true: print every incoming MIDI message to
                                         // the Serial Monitor (115200) to trace stuck notes

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
  // port first count  ch                  note  hue    sat   decay  style
  {  1,   0,    24,    PERCUSSIVE_CHANNEL, 36,   0.00f, 1.0f, 350,   FLASH },  // kick (single ring on A2)
  // A1 chain: six 24-rings, two pad notes each (first red / second green)
  {  2,   0,    24,    PERCUSSIVE_CHANNEL, 1 ,   0.97f, 1.0f, PAD_DECAY, PAD_STYLE },  // ring 1, note 1: red
  {  2,   0,    24,    PERCUSSIVE_CHANNEL, 2 ,   0.33f, 1.0f, PAD_DECAY, PAD_STYLE },  // ring 1, note 2: green
  {  2,   24,   24,    PERCUSSIVE_CHANNEL, 3 ,   0.97f, 1.0f, PAD_DECAY, PAD_STYLE },  // ring 2, note 3: red
  {  2,   24,   24,    PERCUSSIVE_CHANNEL, 4 ,   0.33f, 1.0f, PAD_DECAY, PAD_STYLE },  // ring 2, note 4: green
  {  2,   48,   24,    PERCUSSIVE_CHANNEL, 5 ,   0.97f, 1.0f, PAD_DECAY, PAD_STYLE },  // ring 3, note 5: red
  {  2,   48,   24,    PERCUSSIVE_CHANNEL, 6 ,   0.33f, 1.0f, PAD_DECAY, PAD_STYLE },  // ring 3, note 6: green
  {  2,   72,   24,    PERCUSSIVE_CHANNEL, 7 ,   0.97f, 1.0f, PAD_DECAY, PAD_STYLE },  // ring 4, note 7: red
  {  2,   72,   24,    PERCUSSIVE_CHANNEL, 8 ,   0.33f, 1.0f, PAD_DECAY, PAD_STYLE },  // ring 4, note 8: green
  {  2,   96,   24,    PERCUSSIVE_CHANNEL, 9 ,   0.97f, 1.0f, PAD_DECAY, PAD_STYLE },  // ring 5, note 9: red
  {  2,   96,   24,    PERCUSSIVE_CHANNEL, 10,   0.33f, 1.0f, PAD_DECAY, PAD_STYLE },  // ring 5, note 10: green
  {  2,   120,  24,    PERCUSSIVE_CHANNEL, 11,   0.97f, 1.0f, PAD_DECAY, PAD_STYLE },  // ring 6, note 11: red
  {  2,   120,  24,    PERCUSSIVE_CHANNEL, 12,   0.33f, 1.0f, PAD_DECAY, PAD_STYLE },  // ring 6, note 12: green
  {  3,   0,    7,     PERCUSSIVE_CHANNEL, 42,   0.55f, 0.8f, 150,   FLASH },  // closed hi-hat
};
const int NUM_RINGS = sizeof(RINGS) / sizeof(RINGS[0]);

// ---- Power ----
// Software current limit across ALL ports, in mA. Frames that would draw more
// are scaled down. ~400 on USB; ~3500 for a 4A external supply.
const int   MAX_TOTAL_MA   = 400;    // USB power only. Set to 4250 once the 5V 5A supply is connected.
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

// Alien routing: which channels write to which display, weighted.
// weightLeft = chance a note lands on the LEFT display (the rest go right).
// A display's brightness also follows its channels: held notes keep it lit,
// hits flare it up, and it sinks back when everything is released.
struct AlienRoute { uint8_t channel; float weightLeft; };
const AlienRoute ALIEN_ROUTES[] = {
  { DISPLAY_CHANNEL, 0.5f },   // everything on the display channel, split evenly
};
const bool  ALIEN_OTHER_CHANNELS = false;  // true: unlisted channels split 50/50
const uint8_t ALIEN_MIN_BRIGHTNESS = 1;    // 0..15, resting level
const float ALIEN_FLARE_MS = 600.0f;       // how long a hit's flare takes to fade

// ALL-chord message: when DrumStrum plays a whole chord at once (several
// strum notes arriving together on one channel), flash a message.
const bool     ALL_MSG_ENABLED     = true;
const uint8_t  ALL_CHORD_CHANNEL   = DISPLAY_CHANNEL;
const int      ALL_CHORD_MIN_NOTES = 4;      // notes in one burst that count as ALL
const unsigned long ALL_CHORD_WINDOW_MS = 8; // max spread of the burst
const char*    ALL_MSG_LEFT        = "FUCK";
const char*    ALL_MSG_RIGHT       = " DJT";
const unsigned long ALL_MSG_MS     = 1500;   // how long it stays up

// =====================================================================

const int MAX_PORT_PIXELS = 192;  // buffer size per port (e.g. eight 24-rings chained)

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

const float HUE_CC21_CENTER = 0.00f;
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

const int STRUM_CHANNEL = STRIP_CHANNEL;        // pitched flashes
const int DRUM_CHANNEL  = PERCUSSIVE_CHANNEL;   // strip kick/snare
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
  bool    sustained;  // note released while the sustain pedal was down
  float   baseLevel;  // brightness from velocity, for aftertouch to work from
  unsigned long startMs;  // when the note started, for MAX_HOLD_MS
};

const int MAX_FLASHES = 12;
NoteFlash flashes[MAX_FLASHES];

struct RingState {
  float brightness;   // 0..1, decays after a hit
  float angle;        // 0..1, for SPIN
  float baseLevel;    // brightness from velocity
  float pressure;     // 0..1 aftertouch, or -1 if none received
  bool  held;         // note still down (or sustained)
  bool  sustained;    // released while the sustain pedal was down
  unsigned long startMs;  // when the note started, for MAX_HOLD_MS
  float vel;          // 0..1 velocity of the hit
  float hue;          // hue for this hit, shifted by velocity
};
RingState ringState[NUM_RINGS];

bool sustainDown[17];   // per MIDI channel 1..16

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

// Two independent lines, one per display (index 0 = left, 1 = right).
uint16_t alienLine[2][4];
uint16_t alienQueue[2][8];
int      alienQueued[2] = { 0, 0 };
float    alienEnergy[2] = { 0, 0 };   // 0..1, drives brightness
uint8_t  heldCount[17];               // held notes per MIDI channel (1..16)
unsigned long lastAlienInput = 0;

int alienDisplays() { return (hasLeft ? 1 : 0) + (hasRight ? 1 : 0); }

// Returns the route weight for a channel, or -1 if the channel isn't shown.
float alienWeight(uint8_t channel) {
  for (const AlienRoute &r : ALIEN_ROUTES) if (r.channel == channel) return r.weightLeft;
  return ALIEN_OTHER_CHANNELS ? 0.5f : -1.0f;
}

void alienNoteOn(uint8_t channel, uint8_t note, uint8_t velocity) {
  float wl = alienWeight(channel);
  if (wl < 0) return;
  if (channel <= 16 && heldCount[channel] < 255) heldCount[channel]++;

  int side;
  if (alienDisplays() < 2) side = 0;   // single display: everything on line 0
  else side = ((hash32(micros() ^ (note << 8)) & 0xFFFF) / 65536.0f) < wl ? 0 : 1;

  if (alienQueued[side] < 8) alienQueue[side][alienQueued[side]++] = alienGlyph(channel * 128u + note);
  alienEnergy[side] = max(alienEnergy[side], 0.5f + 0.5f * velocity / 127.0f);
  lastAlienInput = millis();
}

void alienNoteOff(uint8_t channel) {
  if (alienWeight(channel) < 0) return;
  if (channel <= 16 && heldCount[channel] > 0) heldCount[channel]--;
}

// ---- ALL-chord message ----
void writeText(Adafruit_AlphaNum4 &d, const char *t);
unsigned long allMsgUntil = 0;
bool allMsgShown = false;
bool displaysNeedRedraw = false;

void detectAllChord(uint8_t channel) {
  static unsigned long burstStart = 0;
  static int burstCount = 0;
  if (!ALL_MSG_ENABLED || channel != ALL_CHORD_CHANNEL) return;
  unsigned long now = millis();
  if (now - burstStart > ALL_CHORD_WINDOW_MS) { burstStart = now; burstCount = 0; }
  if (++burstCount == ALL_CHORD_MIN_NOTES) {   // fires once per burst
    allMsgUntil = now + ALL_MSG_MS;
    allMsgShown = false;
  }
}

// Returns true while the message owns the displays.
bool updateAllMsg(unsigned long now) {
  if (allMsgUntil == 0) return false;
  if ((long)(now - allMsgUntil) >= 0) {        // message over: hand displays back
    allMsgUntil = 0;
    displaysNeedRedraw = true;
    return false;
  }
  if (hasLeft && hasRight) {
    if (!allMsgShown) {
      dispLeft.setBrightness(15);  dispRight.setBrightness(15);
      writeText(dispLeft, ALL_MSG_LEFT);
      writeText(dispRight, ALL_MSG_RIGHT);
      allMsgShown = true;
    }
  } else {
    // one display: first half shows the left word, second half the right word
    static int phase = -1;
    int p = (allMsgUntil - now) > ALL_MSG_MS / 2 ? 0 : 1;
    if (!allMsgShown) phase = -1;
    if (p != phase) {
      Adafruit_AlphaNum4 &d = hasLeft ? dispLeft : dispRight;
      d.setBrightness(15);
      writeText(d, p == 0 ? ALL_MSG_LEFT : ALL_MSG_RIGHT);
      phase = p;
      allMsgShown = true;
    }
  }
  return true;
}

void noteDisplay(bool isCC, uint8_t channel, uint8_t number, uint8_t value) {
  if (!isCC && value > 0) detectAllChord(channel);
  if (DISPLAY_STYLE == DISPLAY_ALIEN) {
    if (!isCC && value > 0) alienNoteOn(channel, number, value);   // notes only
    return;
  }
  dispEvent = { true, isCC, channel, number, value };   // latest wins
}

Adafruit_AlphaNum4* alienDisp(int side) {
  if (alienDisplays() < 2) return hasLeft ? &dispLeft : &dispRight;
  return side == 0 ? &dispLeft : &dispRight;
}

void drawAlien(int side) {
  Adafruit_AlphaNum4 *d = alienDisp(side);
  for (int i = 0; i < 4; i++) d->writeDigitRaw(i, alienLine[side][i]);
  d->writeDisplay();
}

void updateAlien(unsigned long now) {
  static unsigned long lastDraw = 0, lastChatter = 0, lastTick = 0;
  static int lastLevel[2] = { -1, -1 };
  if (now - lastDraw < DISPLAY_MS) return;
  if (displaysNeedRedraw) {                    // after the ALL message
    displaysNeedRedraw = false;
    lastLevel[0] = lastLevel[1] = -1;
    drawAlien(0);
    if (alienDisplays() == 2) drawAlien(1);
  }
  float dt = (lastTick == 0) ? 0 : (now - lastTick) / 1000.0f;
  lastTick = now;
  lastDraw = now;
  int sides = alienDisplays() < 2 ? 1 : 2;

  for (int side = 0; side < sides; side++) {
    // scroll queued glyphs in from the right
    if (alienQueued[side] > 0) {
      int n = min(alienQueued[side], 4);
      int q = alienQueued[side];
      for (int i = 0; i < 4; i++)
        alienLine[side][i] = (i + n < 4) ? alienLine[side][i + n] : alienQueue[side][q - n + (i + n - 4)];
      alienQueued[side] = 0;
      drawAlien(side);
    }

    // brightness: flare decays, but held notes keep it up in proportion to
    // how strongly their channel is routed to this side
    float sustain = 0;
    for (const AlienRoute &r : ALIEN_ROUTES) {
      if (r.channel <= 16 && heldCount[r.channel] > 0) {
        float w = (sides < 2) ? 1.0f : (side == 0 ? r.weightLeft : 1.0f - r.weightLeft);
        sustain = max(sustain, 0.35f + 0.5f * w);
      }
    }
    alienEnergy[side] = max(alienEnergy[side] - dt / (ALIEN_FLARE_MS / 1000.0f), sustain);
    if (alienEnergy[side] < 0) alienEnergy[side] = 0;
    int level = ALIEN_MIN_BRIGHTNESS +
                (int)(alienEnergy[side] * (DISPLAY_BRIGHTNESS - ALIEN_MIN_BRIGHTNESS) + 0.5f);
    if (level != lastLevel[side]) { alienDisp(side)->setBrightness(level); lastLevel[side] = level; }
  }

  // idle chatter: now and then one glyph mutates or blinks out
  if (ALIEN_IDLE_CHATTER && now - lastAlienInput > ALIEN_IDLE_MS && now - lastChatter > 700) {
    lastChatter = now;
    uint32_t r = hash32(now);
    int side = (sides < 2) ? 0 : (r >> 20) & 1;
    int pos = r % 4;
    if ((r >> 8) % 3 == 0) alienLine[side][pos] = 0;
    else                   alienLine[side][pos] = alienGlyph(r >> 4);
    drawAlien(side);
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
  if (updateAllMsg(now)) return;
  if (DISPLAY_STYLE == DISPLAY_ALIEN) { updateAlien(now); return; }
  if (displaysNeedRedraw) {                    // text mode: restore brightness, show last event
    displaysNeedRedraw = false;
    if (hasLeft)  dispLeft.setBrightness(DISPLAY_BRIGHTNESS);
    if (hasRight) dispRight.setBrightness(DISPLAY_BRIGHTNESS);
    if (dispEvent.channel != 0) dispEvent.pending = true;
    else { if (hasLeft) writeText(dispLeft, "    "); if (hasRight) writeText(dispRight, "    "); }
  }
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
void allNotesOff(uint8_t channel);
void midiLog(const char *kind, byte ch, int a, int b);
void setSustain(uint8_t channel, bool down);

void onControlChange(byte channel, byte cc, byte value) {
  lastMidiMs = millis();
  midiLog("CC  ", channel, cc, value);
  noteDisplay(true, channel, cc, value);
  if (cc == 120 || cc == 123) { allNotesOff(channel); return; }
  if (cc == SUSTAIN_CC && channel <= 16) { setSustain(channel, value >= 64); return; }
  if (channel != STRIP_CHANNEL) return;          // comets listen on the strip channel only
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

void midiLog(const char *kind, byte ch, int a, int b) {
  if (!DEBUG_MIDI) return;
  Serial.print(millis()); Serial.print("  "); Serial.print(kind);
  Serial.print(" ch"); Serial.print(ch);
  if (a >= 0) { Serial.print(" "); Serial.print(a); }
  if (b >= 0) { Serial.print(" "); Serial.print(b); }
  Serial.println();
}

// All Notes Off (CC 123) / All Sound Off (CC 120): release everything held
// on that channel, including anything the sustain pedal is holding.
void allNotesOff(uint8_t channel) {
  if (channel <= 16) sustainDown[channel] = false;
  for (int i = 0; i < NUM_RINGS; i++)
    if (RINGS[i].channel == channel) { ringState[i].held = false; ringState[i].sustained = false; }
  for (int i = 0; i < MAX_FLASHES; i++)
    if (flashes[i].active && flashes[i].channel == channel) { flashes[i].holding = false; flashes[i].sustained = false; }
}

// Sustain pedal: while down, note-offs only mark notes as sustained; on
// release, everything sustained on that channel lets go.
void setSustain(uint8_t channel, bool down) {
  sustainDown[channel] = down;
  if (down) return;
  for (int i = 0; i < NUM_RINGS; i++)
    if (RINGS[i].channel == channel && ringState[i].sustained) {
      ringState[i].sustained = false;
      ringState[i].held = false;
    }
  for (int i = 0; i < MAX_FLASHES; i++)
    if (flashes[i].active && flashes[i].channel == channel && flashes[i].sustained) {
      flashes[i].sustained = false;
      flashes[i].holding = false;
    }
}

float pressureLevel(float base, float pressure) {
  return (pressure < 0) ? base : AT_MIN_LEVEL + (1.0f - AT_MIN_LEVEL) * pressure;
}

// Poly aftertouch: pressure for one held note.
void onAfterTouchPoly(byte channel, byte note, byte pressure) {
  lastMidiMs = millis();
  midiLog("PAT ", channel, note, pressure);
  if (!AFTERTOUCH_ON) return;
  float p = pressure / 127.0f;
  for (int i = 0; i < NUM_RINGS; i++)
    if (RINGS[i].channel == channel && RINGS[i].note == note && ringState[i].held)
      ringState[i].pressure = p;
  for (int i = 0; i < MAX_FLASHES; i++)
    if (flashes[i].active && flashes[i].holding && flashes[i].channel == channel && flashes[i].note == note)
      flashes[i].brightness = pressureLevel(flashes[i].baseLevel, p);
}

// Channel pressure: applies to every held note on the channel.
void onAfterTouchChannel(byte channel, byte pressure) {
  lastMidiMs = millis();
  midiLog("CAT ", channel, pressure, -1);
  if (!AFTERTOUCH_ON) return;
  float p = pressure / 127.0f;
  for (int i = 0; i < NUM_RINGS; i++)
    if (RINGS[i].channel == channel && ringState[i].held)
      ringState[i].pressure = p;
  for (int i = 0; i < MAX_FLASHES; i++)
    if (flashes[i].active && flashes[i].holding && flashes[i].channel == channel)
      flashes[i].brightness = pressureLevel(flashes[i].baseLevel, p);
}

void triggerRings(byte channel, byte note, byte velocity) {
  float norm = velocity / 127.0f;
  for (int i = 0; i < NUM_RINGS; i++) {
    if (RINGS[i].channel == channel && RINGS[i].note == note) {
      ringState[i].baseLevel  = 0.35f + norm * 0.65f;
      ringState[i].brightness = ringState[i].baseLevel;
      ringState[i].angle      = 0;
      ringState[i].pressure   = -1;
      ringState[i].held       = HOLD_WHILE_HELD;
      ringState[i].sustained  = false;
      ringState[i].startMs    = millis();
      ringState[i].vel        = norm;
      ringState[i].hue        = wrapf(RINGS[i].hue - VEL_HUE_SHIFT * norm);
    }
  }
}

void onNoteOn(byte channel, byte note, byte velocity) {
  lastMidiMs = millis();
  midiLog(velocity ? "ON  " : "ON0 ", channel, note, velocity);
  if (velocity == 0) { onNoteOff(channel, note, 0); return; }
  noteDisplay(false, channel, note, velocity);

  triggerRings(channel, note, velocity);

  if (channel != DRUM_CHANNEL && channel != STRUM_CHANNEL) return;
  float norm = velocity / 127.0f;

  float hue, sat, bright, decay, pos;
  if (channel == DRUM_CHANNEL && note == NOTE_KICK) {
    hue = 0.00f; sat = 1.0f; bright = 0.5f + norm * 0.5f;
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
  f->sustained = false; f->baseLevel = bright;
  f->startMs = millis();
}

// Release the strip flash and rings for this channel+note (or mark them
// sustained if the pedal is down). Rings fade from release.
void onNoteOff(byte channel, byte note, byte velocity) {
  lastMidiMs = millis();
  if (velocity) midiLog("OFF ", channel, note, velocity);
  else if (DEBUG_MIDI) midiLog("OFF ", channel, note, -1);
  if (SHOW_NOTE_OFFS) noteDisplay(false, channel, note, 0);
  if (DISPLAY_STYLE == DISPLAY_ALIEN) alienNoteOff(channel);
  bool sus = (channel <= 16) && sustainDown[channel];
  for (int i = 0; i < MAX_FLASHES; i++) {
    if (flashes[i].active && flashes[i].holding &&
        flashes[i].channel == channel && flashes[i].note == note) {
      if (sus) flashes[i].sustained = true;
      else     flashes[i].holding = false;
    }
  }
  for (int i = 0; i < NUM_RINGS; i++) {
    if (RINGS[i].channel == channel && RINGS[i].note == note && ringState[i].held) {
      if (sus) ringState[i].sustained = true;
      else     ringState[i].held = false;
    }
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
    if (f.holding && MAX_HOLD_MS && now - f.startMs > MAX_HOLD_MS) { f.holding = false; f.sustained = false; }
    if (!f.holding) f.brightness -= f.decayRate * dt;
    if (f.brightness <= 0) f.active = false;
  }

  for (int i = 0; i < NUM_RINGS; i++) {
    RingState &s = ringState[i];
    if (s.held && MAX_HOLD_MS && now - s.startMs > MAX_HOLD_MS) {
      s.held = false; s.sustained = false;                 // safety: auto-release
      if (DEBUG_MIDI) { Serial.print("auto-release ring "); Serial.println(i); }
    }
    if (s.held) {
      s.brightness = pressureLevel(s.baseLevel, s.pressure);   // hold, shaped by aftertouch
    } else {
      if (s.brightness <= 0) continue;
      s.brightness -= dt / (RINGS[i].decayMs / 1000.0f);
      if (s.brightness < 0) s.brightness = 0;
    }
    float turns = (RINGS[i].style == CHASE) ? CHASE_TURNS_PER_SEC : 2.0f;   // SPIN: ~2 turns/s
    turns *= VEL_SPEED_MIN + (VEL_SPEED_MAX - VEL_SPEED_MIN) * s.vel;
    if (s.held && s.pressure > 0) turns *= 1.0f + AT_SPEED_BOOST * s.pressure;
    s.angle = wrapf(s.angle + dt * turns);
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
    // white-hot attack: hard hits start desaturated and regain colour as they fade
    float life  = (s.baseLevel > 0) ? clampf(s.brightness / s.baseLevel, 0.0f, 1.0f) : 0.0f;
    float sat   = rc.sat * (1.0f - VEL_WHITE_FLASH * s.vel * s.vel * life);
    for (int k = 0; k < rc.count; k++) {
      float v = s.brightness;
      float h = s.hue;
      if (rc.style == SPIN) {
        float dist = wrapf(k / (float)rc.count - s.angle);
        float arc  = (dist < 0.35f) ? 1.0f - dist / 0.35f : 0.0f;
        v *= 0.15f + 0.85f * arc * arc;   // dim ring with a bright running arc
      } else if (rc.style == CHASE) {
        float dist = wrapf(k / (float)rc.count - s.angle);
        float m    = wrapf(dist * CHASE_ARMS);          // position within one arm
        float arc  = (m < CHASE_ARC) ? 1.0f - m / CHASE_ARC : 0.0f;
        v *= arc * arc;                                 // dark between the arcs
        h = wrapf(h - ARC_HUE_SPREAD * (m / CHASE_ARC)); // tail drifts in colour (away from orange)
      }
      addPixel(rc.port, rc.first + k, h, sat, v);
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

  // Add the MIDI interface first thing, before the Mac finishes enumerating,
  // so the very first connection already includes MIDI. This avoids a
  // "serial-only, then MIDI" double connection after a reset.
  usb_midi.setStringDescriptor("QT Py MIDI Lights");
  usb_midi.begin();

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
    for (int side = 0; side < 2; side++)
      for (int i = 0; i < 4; i++) alienLine[side][i] = alienGlyph(1000 + side * 4 + i);   // greeting
    drawAlien(0);
    if (alienDisplays() == 2) drawAlien(1);
  }

  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.turnThruOff();
  MIDI.setHandleControlChange(onControlChange);
  MIDI.setHandleNoteOn(onNoteOn);
  MIDI.setHandleNoteOff(onNoteOff);
  MIDI.setHandleAfterTouchPoly(onAfterTouchPoly);
  MIDI.setHandleAfterTouchChannel(onAfterTouchChannel);

  // Only if the Mac had already enumerated us before the MIDI interface was
  // added (rare now that it's added first): reconnect so MIDI appears.
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(1000);
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
