// =============================================================================
//  estimator.h - localisation WITHOUT encoders.
//
//  Estimates the forward position s (mm) along the current straight, measured
//  from the centre of the cell where the straight started, with a 1-D Kalman
//  filter:
//   a) PREDICT  travel from the wheel encoders (times a learned wheel-
//               diameter scale). If an encoder fails: speed from the
//               calibrated volts->velocity model with its lag (fallback).
//   b) CORRECT  front sonar against a wall the MAP predicts (primary
//               odometer, only below FRONT_TRUST_MM and when the reading
//               agrees with the prediction within the gate). The latency of
//               each reading is removed using a short history of s.
//   c) CORRECT  side wall->gap / gap->wall transitions, which happen at the
//               posts (cell boundaries): snap s to that boundary.
//   d) HEADING  gyro (imu.*); re-zeroed to the nearest 90 deg + measured wall
//               angle whenever a side wall has been seen parallel for >= 1
//               cell. Gyro bias is re-estimated by imu.* when stationary.
//  sigma = sqrt(variance) is the position confidence: GOOD / REDUCED (slow down)
//  / LOST (stop and beep).
//
//  The estimator also provides the lateral offset from the corridor centre
//  for wall centring, using only side readings that face a wall segment
//  (not a post), are fresh, and were taken nearly parallel to the walls.
// =============================================================================
#pragma once

#include <Arduino.h>
#include "config.h"

namespace estimator {

enum class Confidence : uint8_t { GOOD, REDUCED, LOST };

struct Stats {
  uint32_t frontFixes;     // accepted front-wall corrections
  uint32_t frontRejects;   // gated-out front readings
  uint32_t edgeFixes;      // accepted post-edge corrections
  uint32_t edgeRejects;
  uint32_t headingFixes;   // parallel-wall heading re-zeros
};

void begin();

// A new straight begins with the axle at sMm (normally 0 = cell centre),
// heading aligned to cardinalDeg (multiple of 90), uncertainty sigmaMm.
// useSonar = false: prediction only (e.g. the angled re-centring reverse,
// where the sensors do not look along the maze grid).
void beginStraight(float sMm, float sigmaMm, float cardinalDeg, bool useSonar = true);
// Pivoting / stopped: position frozen, sonar not used for s.
void endStraight();
bool straightActive();

// Face of the next wall ahead that the MAP predicts, measured in the same
// frame as s. Negative = no map-predicted wall (front fixes disabled).
void setFrontWallFace(float faceMm);
float frontWallFace();

// 1 kHz: predict with the applied motor voltages, then consume new sonar
// readings. Called by motion's control tick.
void tick(float dt_s);

float s();                 // mm along the straight
float v();                 // mm/s
float sigma();             // mm
float scale();             // learned scale of the odometry in use (encoder or model)
float modelScale();        // learned volts->speed model scale (feed-forward)
Confidence confidence();
void inflate(float sigmaMm);   // add uncertainty (map contradiction)

// Lateral offset from the corridor centre (+ = robot left of centre).
// Returns false if no side gives a usable reading right now.
bool lateralMm(float& out);

// s at an earlier micros() time (for logging / latency).
float sAt(uint32_t tUs);

Stats stats();
const char* confidenceName(Confidence c);

}  // namespace estimator
