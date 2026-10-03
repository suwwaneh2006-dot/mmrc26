// =============================================================================
//  motors.h - TB6612FNG driver, safety latch and the volts->velocity model.
//
//  The rest of the firmware commands motors in VOLTS, never raw PWM. The duty
//  cycle is computed from the measured battery voltage every tick, so a given
//  command produces the same speed on a full and on a tired pack.
//
//  Safety:
//   * STBY is LOW from the first line of begin() and in every fault state.
//   * A fault (watchdog, overrun, battery, IMU, ...) is latched: motors stay
//     disabled until clearFault() is called explicitly by the operator flow.
//   * An independent esp_timer watchdog pulls STBY LOW if the control tick
//     stops feeding it (frozen or overrunning main loop).
// =============================================================================
#pragma once

#include <Arduino.h>
#include "config.h"

enum class Fault : uint8_t {
  NONE = 0,
  WATCHDOG,      // control tick stopped
  LOOP_OVERRUN,  // control tick far too late
  BATTERY,       // voltage under load below cutoff
  IMU,           // gyro stopped answering
  SONAR,         // a range sensor failed while moving
  COLLISION,     // collision guard fired
  STUCK,         // commanded motion but nothing moved
  USER_ABORT,    // operator pressed the button
};

const char* faultName(Fault f);

// Piecewise-linear steady-state speed of the robot driving straight with the
// same voltage on both motors, plus a first-order lag time constant.
struct VelocityModel {
  uint32_t magic;                     // identifies a valid stored model
  uint8_t  count;                     // number of points in use
  float    volts[MODEL_MAX_POINTS];   // strictly increasing, volts[0] = dead-band
  float    mm_s[MODEL_MAX_POINTS];    // mm_s[0] = 0
  float    tau_s;                     // first-order lag

  // Steady-state speed for a (signed) voltage. Symmetric for reverse.
  float speedFor(float v) const;
  // Voltage needed for a (signed) steady-state speed. Any non-zero speed
  // request returns at least the dead-band voltage (static friction feed-forward).
  float voltsFor(float speed_mm_s) const;
  bool  valid() const;
};

namespace motors {

void begin();

// Enable the H-bridge (STBY HIGH). Refused while a fault is latched.
bool enable();
// Disable the H-bridge (STBY LOW) and zero all commands.
void disable();
bool enabled();

// Target voltages, + = forward. Clamped to +-MOTOR_VMAX_V and slew-limited.
void setVolts(float left_v, float right_v);
// Called once per control tick: slew limit, duty computation, pin output.
void update(float dt_s);
// Called once per control tick: proves to the watchdog that the loop runs.
void feedWatchdog();
// Allow the control tick to pause for up to ms (planned blocking section,
// e.g. a flash write). Used only through sched::blockingSection().
void watchdogGrace(uint32_t ms);

float appliedLeftV();
float appliedRightV();
// True when both applied voltages are (almost) zero.
bool  idle();

// Latch a fault: STBY LOW immediately. Safe to call from any context.
void  fault(Fault f);
Fault faultCode();
void  clearFault();

// Velocity model: loaded from NVS at begin(), default if absent.
const VelocityModel& model();
bool  modelIsCalibrated();
bool  saveModel(const VelocityModel& m);
VelocityModel defaultModel();

}  // namespace motors
