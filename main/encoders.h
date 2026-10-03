// =============================================================================
//  encoders.h - wheel odometry from the N20 magnetic hall encoders.
//
//  One channel (A) per wheel, both edges counted by a GPIO interrupt. With a
//  single channel the hardware cannot tell the direction, so each edge is
//  signed with the direction of that motor's applied voltage (the last
//  non-zero one while braking/coasting). The only ambiguity is during the
//  few milliseconds in which a wheel reverses, which the sonar corrections
//  absorb.
//
//  Health: a wheel that stays silent while its motor is clearly driven and the
//  other wheel counts is marked faulty (wire off, magnet loose). Then
//  healthy() is false and the estimator / motion fall back to the
//  volts->velocity model instead of stopping the robot.
// =============================================================================
#pragma once

#include <Arduino.h>
#include "config.h"

namespace encoders {

void begin();
// Called once per control tick (before the motion controller).
void update(float dt_s);

float leftMm();          // signed travel since begin()
float rightMm();
float leftSpeed();       // filtered wheel speed, mm/s (signed)
float rightSpeed();
float speed();           // (left + right) / 2
uint32_t leftEdges();    // raw edge counts (unsigned), for calibration
uint32_t rightEdges();

bool healthy();          // both encoders working
bool leftFaulty();
bool rightFaulty();
void clearFaults();      // e.g. after a rescue
// Not faulty and not momentarily suspect (silent > ENC_SUSPECT_MS while driven).
bool leftUsable();
bool rightUsable();

}  // namespace encoders
