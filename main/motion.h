#pragma once

#include <Arduino.h>
#include "config.h"

namespace motion {

enum class Result : uint8_t {
  RUNNING,
  DONE,
  ABORT_COLLISION,
  ABORT_TIMEOUT,
  ABORT_FAULT,
  ABORT_USER,
  ABORT_STUCK,
  ABORT_LOST,
};

struct Telemetry {
  float sRefMm;
  float vRefMmS;
  float sEstMm;
  float vEstMmS;
  float sigmaMm;
  float leftV;
  float rightV;
  float headingDeg;
  float targetDeg;
};

void begin();

void runBegin(const SpeedTier& tier, float sStartMm, float sigmaMm);
void runExtend(float mm);
float runTargetMm();

void straight(float dist_mm, const SpeedTier& tier);

void reverse(float dist_mm, const SpeedTier& tier);

void setSpeedCap(float mm_s);

void pivot(float angle_deg, const SpeedTier& tier);

void alignFront();

void constantVolts(float volts, bool holdHeading);
void rawVolts(float left_v, float right_v);

void stop();

bool   busy();
Result result();
const char* resultName(Result r);

Telemetry telemetry();
float headingTargetDeg();

bool   arm();
Result waitDone();
float  settledFrontMm();
void   reportFault();
void  setHeadingTargetDeg(float deg);

}
