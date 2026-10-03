// =============================================================================
//  motion.h - motion primitives run inside the 1 kHz control tick.
//
//  Straights ("runs"): start at rest on a cell centre (s = 0) and drive to a
//  target distance that the caller may EXTEND while moving (search: one cell
//  at a time, decided at each cell's read point; speed run: the whole
//  straight at once). Speed follows a trapezoid with the accel limit, braking
//  as sqrt(2 a d) on the ESTIMATED remaining distance, so front-wall fixes
//  make the robot stop on the cell centre. Feed-forward volts come from the
//  calibrated model (lag-compensated), steering from gyro heading hold
//  blended with side-wall centring.
//
//  Pivots: in place, trapezoid on angle, gyro PD, settle criterion.
//  Front alignment: creep until the front reading equals the centred value.
//  Safety in every moving mode: collision guard (front sonar vs stopping
//  distance), stuck detection, confidence speed cap, motor fault abort.
//
//  Sign conventions: distance + = forward, angle + = CCW (left turn).
// =============================================================================
#pragma once

#include <Arduino.h>
#include "config.h"

namespace motion {

enum class Result : uint8_t {
  RUNNING,
  DONE,
  ABORT_COLLISION,   // collision guard braked
  ABORT_TIMEOUT,     // took far longer than the profile allows
  ABORT_FAULT,       // motors latched a fault (battery, IMU, watchdog, ...)
  ABORT_USER,        // stop() requested by the caller
  ABORT_STUCK,       // commanded motion but nothing moved for STUCK_MS
  ABORT_LOST,        // position confidence lost
};

struct Telemetry {
  float sRefMm;      // target distance of the current run
  float vRefMmS;     // profile speed
  float sEstMm;      // estimated position (estimator)
  float vEstMmS;     // estimated speed
  float sigmaMm;     // position uncertainty
  float leftV;       // applied motor voltages
  float rightV;
  float headingDeg;
  float targetDeg;   // heading the controller is holding / turning to
};

void begin();              // registers the control law with the scheduler

// --- Straights ---
// Start a run at rest: axle at sStartMm (0 = on the cell centre), position
// uncertainty sigmaMm. Target = sStartMm until extended.
void runBegin(const SpeedTier& tier, float sStartMm, float sigmaMm);
void runExtend(float mm);
float runTargetMm();
// Convenience for test modes: runBegin + runExtend(dist_mm).
void straight(float dist_mm, const SpeedTier& tier);
// Short reverse move (re-centring before a pivot). No rear sensor: only into
// space the robot has just driven through. No wall centring, no front guard.
void reverse(float dist_mm, const SpeedTier& tier);
// Upper speed limit on top of the tier (0 = none), e.g. low battery.
void setSpeedCap(float mm_s);

// --- Turns and alignment ---
void pivot(float angle_deg, const SpeedTier& tier);
// Creep until the front sonar reads FRONT_CENTRE_READING_MM (axle on the cell
// centre). Ends immediately if no wall is in range.
void alignFront();

// --- Open-loop drive (calibration / motor test) ---
void constantVolts(float volts, bool holdHeading);
void rawVolts(float left_v, float right_v);

// Ramp both motors to zero; busy() until the robot is still.
void stop();

bool   busy();
Result result();
const char* resultName(Result r);

Telemetry telemetry();
float headingTargetDeg();
void  setHeadingTargetDeg(float deg);

}  // namespace motion
