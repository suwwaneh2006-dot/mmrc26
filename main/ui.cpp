// =============================================================================
//  ui.cpp - see ui.h
// =============================================================================
#include "ui.h"

namespace {

// ---------------------------------------------------------------------------
//  Button
// ---------------------------------------------------------------------------
constexpr uint8_t EVENT_QUEUE = 8;
ui::Button g_events[EVENT_QUEUE];
uint8_t  g_evHead = 0, g_evCount = 0;

bool     g_rawDown = false;
bool     g_stableDown = false;
uint32_t g_rawChangeMs = 0;
uint32_t g_pressStartMs = 0;
bool     g_longFired = false;

void pushEvent(ui::Button b) {
  if (g_evCount == EVENT_QUEUE) return;   // drop when full
  g_events[(g_evHead + g_evCount) % EVENT_QUEUE] = b;
  ++g_evCount;
}

void updateButton(uint32_t now) {
  const bool raw = digitalRead(PIN_BUTTON) == LOW;
  if (raw != g_rawDown) {
    g_rawDown = raw;
    g_rawChangeMs = now;
  }
  if (now - g_rawChangeMs >= BUTTON_DEBOUNCE_MS && g_stableDown != g_rawDown) {
    g_stableDown = g_rawDown;
    if (g_stableDown) {
      g_pressStartMs = now;
      g_longFired = false;
    } else if (!g_longFired) {
      pushEvent(ui::Button::SHORT);
    }
  }
  if (g_stableDown && !g_longFired && now - g_pressStartMs >= BUTTON_LONG_MS) {
    g_longFired = true;
    pushEvent(ui::Button::LONG);
  }
}

// ---------------------------------------------------------------------------
//  Buzzer: a queue of patterns, each "count x (on, off)".
// ---------------------------------------------------------------------------
struct Pattern {
  uint8_t  count;
  uint16_t onMs;
  uint16_t offMs;
  bool     low;
};
constexpr uint8_t  PATTERN_QUEUE = 6;
constexpr uint16_t PATTERN_PAUSE_MS = 350;   // silence between queued patterns
Pattern  g_pat[PATTERN_QUEUE];
uint8_t  g_patHead = 0, g_patCount = 0;
uint8_t  g_beepsLeft = 0;
bool     g_buzzOn = false;
uint32_t g_buzzNextMs = 0;
bool     g_inPause = false;

void buzzer(bool on, bool low) {
  if (BUZZER_ACTIVE) {
    digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
  } else {
    ledcWriteTone(PIN_BUZZER, on ? (low ? BUZZER_LOW_TONE_HZ : BUZZER_TONE_HZ) : 0);
  }
  g_buzzOn = on;
}

void updateBuzzer(uint32_t now) {
  if (g_patCount == 0 || static_cast<int32_t>(now - g_buzzNextMs) < 0) return;
  Pattern& p = g_pat[g_patHead];
  if (g_inPause) {                       // gap after a finished pattern
    g_inPause = false;
    g_patHead = (g_patHead + 1) % PATTERN_QUEUE;
    --g_patCount;
    if (g_patCount > 0) g_beepsLeft = g_pat[g_patHead].count;
    return;
  }
  if (g_buzzOn) {
    buzzer(false, p.low);
    --g_beepsLeft;
    if (g_beepsLeft == 0) {
      g_inPause = true;
      g_buzzNextMs = now + PATTERN_PAUSE_MS;
    } else {
      g_buzzNextMs = now + p.offMs;
    }
  } else {
    buzzer(true, p.low);
    g_buzzNextMs = now + p.onMs;
  }
}

// ---------------------------------------------------------------------------
//  Status indication. The on-board LED pin (GPIO 22) now carries the right
//  wheel encoder, so there is no LED: the steady patterns are silent and an
//  error code is BEEPED (code x low beep) every ERROR_BEEP_PERIOD_MS.
// ---------------------------------------------------------------------------
constexpr uint32_t ERROR_BEEP_PERIOD_MS = 6000;
ui::Led  g_ledMode = ui::Led::OFF;
uint8_t  g_ledCode = 0;
uint32_t g_lastCodeBeepMs = 0;

void updateLed(uint32_t now) {
  if (g_ledMode != ui::Led::CODE || g_ledCode == 0 || g_patCount > 0) return;
  if (now - g_lastCodeBeepMs >= ERROR_BEEP_PERIOD_MS) {
    g_lastCodeBeepMs = now;
    ui::beep(g_ledCode, 250, 300, true);
  }
}

}  // namespace

namespace ui {

void begin() {
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  if (BUZZER_ACTIVE) {
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
  } else {
    // Different frequency than the motors -> gets its own LEDC timer, so
    // changing the tone never disturbs the 20 kHz motor PWM.
    ledcAttach(PIN_BUZZER, BUZZER_TONE_HZ, BUZZER_PWM_BITS);
    ledcWrite(PIN_BUZZER, 0);
  }
  g_rawDown = g_stableDown = digitalRead(PIN_BUTTON) == LOW;
  g_rawChangeMs = g_pressStartMs = millis();
  g_longFired = false;
}

void update() {
  const uint32_t now = millis();
  updateButton(now);
  updateBuzzer(now);
  updateLed(now);
}

Button event() {
  if (g_evCount == 0) return Button::NONE;
  const Button b = g_events[g_evHead];
  g_evHead = (g_evHead + 1) % EVENT_QUEUE;
  --g_evCount;
  return b;
}

void clearEvents() { g_evCount = 0; }
bool buttonDown() { return g_stableDown; }
uint32_t heldMs() { return g_stableDown ? millis() - g_pressStartMs : 0; }

void beep(uint8_t count, uint16_t onMs, uint16_t offMs, bool lowTone) {
  if (count == 0 || g_patCount == PATTERN_QUEUE) return;
  g_pat[(g_patHead + g_patCount) % PATTERN_QUEUE] = Pattern{count, onMs, offMs, lowTone};
  if (g_patCount == 0) {
    g_beepsLeft = count;
    g_inPause = false;
    g_buzzNextMs = millis();
  }
  ++g_patCount;
}

bool beepBusy() { return g_patCount > 0; }

void beepStop() {
  g_patCount = 0;
  g_inPause = false;
  buzzer(false, false);
}

void soundOk()          { beep(1, 60, 100); }
void soundError()       { beep(3, 400, 200, true); }
void soundAbort()       { beep(2, 400, 200, true); }
void soundWiped()       { beep(1, 800, 100); }
void soundCalSaved()    { beep(5, 40, 60); }
void soundBatteryWarn() { beep(2, 40, 60, true); }

void led(Led mode, uint8_t code) {
  // A new error code is beeped at once, then every ERROR_BEEP_PERIOD_MS.
  if (mode == Led::CODE && (mode != g_ledMode || code != g_ledCode)) g_lastCodeBeepMs = millis() - ERROR_BEEP_PERIOD_MS;
  g_ledMode = mode;
  g_ledCode = code;
}

}  // namespace ui
