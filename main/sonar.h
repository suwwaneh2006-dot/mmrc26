#pragma once

#include <Arduino.h>
#include "config.h"

namespace sonar {

enum Id : uint8_t { FRONT = 0, LEFT = 1, RIGHT = 2, COUNT = 3 };

struct Reading {
  float    mm;
  float    rawMm;
  bool     inRange;
  bool     wall;
  bool     valid;
  uint32_t tUs;
  uint32_t seq;
};

struct Stats {
  uint32_t pings;
  uint32_t echoes;
  uint32_t misses;
  uint32_t gateRejects;
  uint8_t  missRun;
  bool     stuck;
  bool     faulty;
};

void begin();

void update();

Reading read(Id id);
Stats   stats(Id id);
bool    faulty(Id id);

uint32_t ageMs(Id id);

void setMotionHint(float speed_mm_s, float rate_dps);

void setFrontOnly(bool frontOnly);

void discardInFlight();

inline bool sideTrusted(float headingErrorDeg) { return fabsf(headingErrorDeg) < SIDE_TRUST_DEG; }

uint8_t selfTest();

const char* name(Id id);

}
