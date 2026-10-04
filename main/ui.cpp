#include "ui.h"

#include "motors.h"
#include "sonar.h"

namespace {

constexpr uint8_t EVENT_QUEUE = 8;
ui::Button g_events[EVENT_QUEUE];
uint8_t  g_evHead = 0, g_evCount = 0;

bool     g_rawDown = false;
bool     g_stableDown = false;
uint32_t g_rawChangeMs = 0;
uint32_t g_pressStartMs = 0;
bool     g_longFired = false;

void pushEvent(ui::Button b) {
  if (g_evCount == EVENT_QUEUE) return;
  g_events[(g_evHead + g_evCount) % EVENT_QUEUE] = b;
  ++g_evCount;
}

bool g_sonarArmed = false;
bool g_sonarDown = false;

bool sonarPressed() {
  if (motors::enabled()) {
    g_sonarArmed = false;
    g_sonarDown = false;
    return false;
  }
  const sonar::Reading f = sonar::read(sonar::FRONT);
  if (!f.valid || sonar::ageMs(sonar::FRONT) > SONAR_FRESH_MS) return g_sonarDown;
  const bool near = f.inRange && f.mm < SONAR_BUTTON_NEAR_MM;
  const bool clear = !f.inRange || f.mm > 2.0f * SONAR_BUTTON_NEAR_MM;
  if (clear) {
    g_sonarArmed = true;
    g_sonarDown = false;
  } else if (near && g_sonarArmed) {
    g_sonarDown = true;
  } else if (!near && f.mm > SONAR_BUTTON_NEAR_MM + 20.0f) {
    g_sonarDown = false;
  }
  return g_sonarDown;
}

void updateButton(uint32_t now) {
  bool raw = digitalRead(PIN_BUTTON) == LOW;
  if (USE_SONAR_BUTTON) raw = raw || sonarPressed();
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

struct Pattern {
  uint8_t  count;
  uint16_t onMs;
  uint16_t offMs;
  bool     low;
};
constexpr uint8_t  PATTERN_QUEUE = 6;
constexpr uint16_t PATTERN_PAUSE_MS = 350;
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
  if (g_inPause) {
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

}

namespace ui {

void begin() {
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  if (BUZZER_ACTIVE) {
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
  } else {

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

  if (mode == Led::CODE && (mode != g_ledMode || code != g_ledCode)) g_lastCodeBeepMs = millis() - ERROR_BEEP_PERIOD_MS;
  g_ledMode = mode;
  g_ledCode = code;
}

}
