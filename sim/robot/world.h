#pragma once

#include <stdint.h>
#include <string>

#include "truemaze.h"

namespace world {

struct Params {
  unsigned seed = 1;
  bool     mirrored = false;
  float    cellTolerance = 0.05f;

  float    motorDeadbandV = 0.7f;
  float    motorMmSPerV = 236.0f;
  float    motorTauS = 0.07f;
  float    rightWheelGain = 1.03f;

  float    sonarNoiseMm = 1.5f;
  float    sonarNoEchoMs = 150.0f;
  float    gyroBiasDps = 0.6f;
  float    gyroNoiseDps = 0.05f;

  bool     encoders = true;
  bool     invertLeftMotor = false;
  bool     gyroFlipped = false;
  bool     openFloor = false;
  float    encoderFailAtS = -1.0f;
  float    wheelDiameterFactor = 1.02f;
};

void init(const TrueMaze& maze, const Params& p);

uint64_t nowUs();
void setButton(bool pressed);
void setHand(bool present);
void teleportToStart();
void truePose(float& x, float& y, float& thDeg, float& vl, float& vr);

using LineHook = void (*)(const char* line);
using TickHook = void (*)(uint64_t us);
void setLineHook(LineHook h);
void setTickHook(TickHook h);
void setEcho(bool echo);

struct Stats {
  int    crashes;
  int    pivotCrashes;
  double distanceMm;
  float  minWallClearanceMm;
  int    stops;
  double sumAbsAlongMm;
  float  maxAbsAlongMm;
  double sumAbsLateralMm;
  float  maxAbsLateralMm;
  float  maxAbsHeadingDeg;
};
Stats stats();

bool nvsGet(const std::string& ns, const std::string& key, std::string& out);

}
