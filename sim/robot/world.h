// =============================================================================
//  sim/robot/world.h - the simulated robot and maze behind the Arduino shim.
//
//  Physics: two DC motors driven through the real TB6612 pin/PWM states the
//  firmware writes (true motor constants deliberately differ from the
//  firmware's default model), differential-drive kinematics, collision check
//  of the robot footprint against walls and posts.
//  Sensors: HC-SR04 echoes timed on the real ECHO pins (ISR called at the exact
//  simulated microsecond), ray cast with a beam cone and specular loss;
//  MPU6050 on the emulated I2C bus with bias, drift and noise; battery sag.
//  Maze: every cell's inner size drawn from 171..189 mm (MMRC +-5 %).
// =============================================================================
#pragma once

#include <stdint.h>
#include <string>

#include "truemaze.h"

namespace world {

struct Params {
  unsigned seed = 1;
  bool     mirrored = false;     // physical maze extends to the left
  float    cellTolerance = 0.05f;
  // true motors (the firmware assumes MODEL_DEFAULT_*)
  float    motorDeadbandV = 0.7f;
  float    motorMmSPerV = 236.0f;
  float    motorTauS = 0.07f;
  float    rightWheelGain = 1.03f; // asymmetry the heading hold must absorb
  // sensors
  float    sonarNoiseMm = 1.5f;
  float    sonarNoEchoMs = 150.0f;
  float    gyroBiasDps = 0.6f;
  float    gyroNoiseDps = 0.05f;
  // wheel encoders (one channel per wheel, both edges)
  bool     encoders = true;          // false: no pulses at all (dead encoders)
  float    encoderFailAtS = -1.0f;   // >= 0: left encoder dies at this time
  float    wheelDiameterFactor = 1.02f;   // true wheel vs WHEEL_DIAMETER_MM
};

void init(const TrueMaze& maze, const Params& p);

uint64_t nowUs();
void setButton(bool pressed);
void setHand(bool present);      // hand in front of the front sonar
void teleportToStart();          // operator rescue
void truePose(float& x, float& y, float& thDeg, float& vl, float& vr);

using LineHook = void (*)(const char* line);
using TickHook = void (*)(uint64_t us);
void setLineHook(LineHook h);    // every complete Serial line
void setTickHook(TickHook h);    // every simulated millisecond
void setEcho(bool echo);         // copy Serial to stdout

struct Stats {
  int    crashes;
  double distanceMm;
  float  minWallClearanceMm;     // smallest footprint-to-wall gap seen while moving
  int    stops;                  // times the robot came to rest after driving
  double sumAbsAlongMm;          // |distance from true cell centre| along heading
  float  maxAbsAlongMm;
  double sumAbsLateralMm;        // |sideways offset from the corridor centre|
  float  maxAbsLateralMm;
  float  maxAbsHeadingDeg;       // |heading error| at rest
};
Stats stats();

// NVS contents (for checking the stored map after a run).
bool nvsGet(const std::string& ns, const std::string& key, std::string& out);

}  // namespace world
