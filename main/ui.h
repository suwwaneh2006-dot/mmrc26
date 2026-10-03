// =============================================================================
//  ui.h - button, status LED and buzzer. Everything is non-blocking: patterns
//  are queued and played by update(), called from the main loop.
//
//  Sound meanings (also in the README):
//    N short beeps        a number: working-sensor count at boot, selected mode
//    1 short high         OK / confirmed
//    3 long low           error / refused (the error code is beeped next)
//    2 long low           aborted (button, collision, fault)
//    1 very long          map wiped
//    5 rapid              calibration saved
//    2 rapid low          battery warning
//
//  Error codes (no LED: GPIO 22 carries the right encoder). The code is
//  BEEPED as N long low beeps, repeated every 6 s while the error persists:
//    1 front sonar  2 left sonar  3 right sonar  4 IMU  5 battery too low
//    6 motor fault latched (watchdog / overrun / cutoff)
//  led() keeps its other patterns as no-ops so callers stay unchanged.
// =============================================================================
#pragma once

#include <Arduino.h>
#include "config.h"

namespace ui {

enum class Button : uint8_t { NONE, SHORT, LONG };
enum class Led : uint8_t { OFF, ON, BLINK_SLOW, BLINK_FAST, CODE };

enum ErrorCode : uint8_t {
  ERR_SONAR_FRONT = 1,
  ERR_SONAR_LEFT  = 2,
  ERR_SONAR_RIGHT = 3,
  ERR_IMU         = 4,
  ERR_BATTERY     = 5,
  ERR_MOTOR_FAULT = 6,
};

void begin();
void update();

// --- Button ---
Button   event();        // next queued event, NONE if empty
void     clearEvents();
bool     buttonDown();   // debounced level
uint32_t heldMs();       // how long the button has been held (0 if released)

// --- Buzzer ---
void beep(uint8_t count, uint16_t onMs = 80, uint16_t offMs = 150, bool lowTone = false);
bool beepBusy();
void beepStop();

void soundOk();
void soundError();
void soundAbort();
void soundWiped();
void soundCalSaved();
void soundBatteryWarn();

// --- LED ---
void led(Led mode, uint8_t code = 0);

}  // namespace ui
