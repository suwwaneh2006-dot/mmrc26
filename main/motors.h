#pragma once

#include <Arduino.h>
#include "config.h"

enum class Fault : uint8_t {
  NONE = 0,
  WATCHDOG,
  LOOP_OVERRUN,
  BATTERY,
  IMU,
  SONAR,
  COLLISION,
  STUCK,
  USER_ABORT,
};

const char* faultName(Fault f);

struct VelocityModel {
  uint32_t magic;
  uint8_t  count;
  float    volts[MODEL_MAX_POINTS];
  float    mm_s[MODEL_MAX_POINTS];
  float    tau_s;

  float speedFor(float v) const;

  float voltsFor(float speed_mm_s) const;
  bool  valid() const;
};

namespace motors {

void begin();

bool enable();

void disable();
bool enabled();

void setVolts(float left_v, float right_v);

void update(float dt_s);

void feedWatchdog();

void watchdogGrace(uint32_t ms);

float appliedLeftV();
float appliedRightV();

bool  idle();

void  fault(Fault f);
Fault faultCode();
void  clearFault();

const VelocityModel& model();
bool  modelIsCalibrated();
bool  saveModel(const VelocityModel& m);
VelocityModel defaultModel();

}
