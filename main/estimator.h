#pragma once

#include <Arduino.h>
#include "config.h"

namespace estimator {

enum class Confidence : uint8_t { GOOD, REDUCED, LOST };

struct Stats {
  uint32_t frontFixes;
  uint32_t frontRejects;
  uint32_t edgeFixes;
  uint32_t edgeRejects;
  uint32_t headingFixes;
};

void begin();

void beginStraight(float sMm, float sigmaMm, float cardinalDeg, bool useSonar = true);

void endStraight();
bool straightActive();

void setFrontWallFace(float faceMm);
float frontWallFace();

void tick(float dt_s);

float s();
float v();
float sigma();
float scale();
float modelScale();
Confidence confidence();
void inflate(float sigmaMm);

bool lateralMm(float& out);

float sAt(uint32_t tUs);

Stats stats();
const char* confidenceName(Confidence c);

}
