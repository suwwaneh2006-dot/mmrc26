// =============================================================================
//  sonar.h - non-blocking driver for 3x HC-SR04 (front, left, right).
//
//  * TRIG: 10 us pulse. ECHO: GPIO CHANGE interrupt, micros() timestamps.
//  * Only ONE sensor is in flight at a time (no crosstalk). The next ping is
//    fired once the previous measurement has ended (echo fell, or the reading
//    was already classified NO_WALL) AND SONAR_GAP_MS has passed.
//  * Firing order F, L, F, R: the front sensor runs at double rate because it
//    is the primary odometer.
//  * An HC-SR04 holds ECHO high for 30-200 ms when nothing reflects and
//    ignores triggers meanwhile. Such a sensor is BUSY and its slot is skipped
//    until ECHO falls; its reading is already published as NO_WALL as soon as
//    the echo outlives the maximum range.
//  * Filtering: plausibility gate on the raw value, then median-of-3.
//  * Wall flags use geometry-derived thresholds with hysteresis.
// =============================================================================
#pragma once

#include <Arduino.h>
#include "config.h"

namespace sonar {

enum Id : uint8_t { FRONT = 0, LEFT = 1, RIGHT = 2, COUNT = 3 };

struct Reading {
  float    mm;          // filtered (gate + median-of-3) distance, valid if inRange
  float    rawMm;       // latest raw measurement (maxMm + 1 if NO_WALL)
  bool     inRange;     // filtered value is a real distance (not NO_WALL)
  bool     wall;        // wall-present flag (with hysteresis)
  bool     valid;       // at least one measurement since start
  uint32_t tUs;         // micros() timestamp of the latest raw measurement
  uint32_t seq;         // increments with every new raw measurement
};

struct Stats {
  uint32_t pings;       // triggers sent
  uint32_t echoes;      // echo pulses seen (rising edge)
  uint32_t misses;      // no rising edge after a trigger
  uint32_t gateRejects; // raw readings rejected by the plausibility gate
  uint8_t  missRun;     // consecutive misses
  bool     stuck;       // ECHO currently high for longer than SONAR_STUCK_MS
  bool     faulty;      // stuck, or too many consecutive misses
};

void begin();
// Call as often as possible from the main loop; never blocks except for the
// 10 us trigger pulse.
void update();

Reading read(Id id);
Stats   stats(Id id);
bool    faulty(Id id);
// Age of the latest reading in ms (large if never measured).
uint32_t ageMs(Id id);

// Motion hint from the controller, used by the plausibility gate.
void setMotionHint(float speed_mm_s, float rate_dps);

// Calibration only: ping the front sensor in every slot (sides not updated).
void setFrontOnly(bool frontOnly);
// Drop the measurement in flight (its ISR timing may have been delayed by a
// flash write). Called after sched::blockingSection().
void discardInFlight();

// Side readings are only meaningful when the robot is nearly parallel to the
// walls (specular reflection): |heading error| < SIDE_TRUST_DEG.
inline bool sideTrusted(float headingErrorDeg) { return fabsf(headingErrorDeg) < SIDE_TRUST_DEG; }

// Boot self-check (blocking, SONAR_SELFTEST_MS): returns a bit mask of
// sensors that produced echo pulses and are not stuck (bit n = Id n).
uint8_t selfTest();

const char* name(Id id);

}  // namespace sonar
