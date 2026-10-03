// =============================================================================
//  motion.cpp - see motion.h
// =============================================================================
#include "motion.h"

#include "encoders.h"
#include "estimator.h"
#include "imu.h"
#include "motors.h"
#include "sched.h"
#include "sonar.h"

namespace {

using motion::Result;

constexpr float DEG2RAD = 0.01745329f;
constexpr float PIVOT_MIN_RATE_DPS = 15.0f;   // pivot profile crawl rate (always finishes)

enum class Mode : uint8_t { IDLE, RAW, CONSTANT, RUN, PIVOT, ALIGN, STOPPING };

// Trapezoidal profile on |angle| for pivots.
struct AngleProfile {
  float total = 0.0f, vmax = 0.0f, accel = 0.0f;
  float pos = 0.0f, vel = 0.0f, acc = 0.0f;
  bool  done = true;

  void start(float total_, float vmax_, float accel_) {
    total = total_;
    vmax = vmax_;
    accel = accel_;
    pos = vel = acc = 0.0f;
    done = total <= 0.0f;
  }
  void step(float dt) {
    if (done) {
      vel = acc = 0.0f;
      return;
    }
    const float vBrake = sqrtf(2.0f * accel * fmaxf(total - pos, 0.0f));
    const float v = fmaxf(fminf(fminf(vmax, vel + accel * dt), vBrake), PIVOT_MIN_RATE_DPS);
    acc = (v - vel) / dt;
    vel = v;
    pos += vel * dt;
    if (pos >= total) {
      pos = total;
      vel = 0.0f;
      done = true;
    }
  }
  float duration() const { return total / vmax + vmax / accel; }
};

Mode     g_mode = Mode::IDLE;
Result   g_result = Result::DONE;
Result   g_pendingResult = Result::DONE;   // reported once STOPPING finishes
SpeedTier g_tier = TIERS[0];
float    g_speedCap = 0.0f;

// Run state
float    g_runTarget = 0.0f;
float    g_runSign = 1.0f;          // +1 forward run, -1 reverse (re-centring)
float    g_vRef = 0.0f;
float    g_aRef = 0.0f;

// Pivot state
AngleProfile g_turn;
float    g_turnSign = 1.0f;
float    g_pivotStart = 0.0f;

// Open-loop state
float    g_constV = 0.0f;
bool     g_holdHeading = false;
float    g_rawL = 0.0f, g_rawR = 0.0f;

float    g_headingTarget = 0.0f;   // cardinal heading being held
uint32_t g_startMs = 0;
uint32_t g_timeoutMs = 0;
uint32_t g_settleSinceMs = 0;
bool     g_settling = false;

// Stuck detection
uint32_t g_lastMotionMs = 0;
uint32_t g_lastWheelMotionMs = 0;
float    g_stuckRefRange = -1.0f;

float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

void enterStopping(Result r) {
  g_pendingResult = r;
  g_mode = Mode::STOPPING;
  g_settling = false;
  g_vRef = 0.0f;
  motors::setVolts(0.0f, 0.0f);
  estimator::endStraight();
}

void startCommon(Mode mode, uint32_t timeoutMs) {
  g_mode = mode;
  g_result = Result::RUNNING;
  g_startMs = millis();
  g_timeoutMs = timeoutMs;
  g_settling = false;
  g_lastMotionMs = millis();
  g_lastWheelMotionMs = millis();
  g_stuckRefRange = -1.0f;
}

bool timedOut() { return millis() - g_startMs > g_timeoutMs; }

// Integral of the heading error on the current straight (wheel mismatch).
float g_headingI = 0.0f;
// Integral of the forward speed error (encoder speed loop).
float g_speedI = 0.0f;

// Feedback volts for a wheel / forward speed error, encoders only.
float speedFeedback(float target, float measured, float* integral, float dt, bool usable) {
  if (!usable) return 0.0f;
  const float err = target - measured;
  float i = 0.0f;
  if (integral != nullptr) {
    *integral = clampf(*integral + SPEED_KI_V_PER_MM * err * dt, -SPEED_I_MAX_V, SPEED_I_MAX_V);
    i = *integral;
  }
  return clampf(SPEED_KP_V_PER_MM_S * err + i, -SPEED_FB_MAX_V, SPEED_FB_MAX_V);
}

// Differential voltage holding thetaCmd (CCW positive). PD, plus a clamped
// integral (only while driving a run) that removes the steady error a
// left/right motor mismatch would otherwise leave.
float headingCorrection(float thetaCmd, float dt = 0.0f) {
  const float err = thetaCmd - imu::headingDeg();
  if (dt > 0.0f) {
    g_headingI = clampf(g_headingI + HEADING_KI_V_PER_DEG_S * err * dt, -HEADING_I_MAX_V, HEADING_I_MAX_V);
  }
  const float c = HEADING_KP_V_PER_DEG * err - HEADING_KD_V_PER_DPS * imu::rateDps() + g_headingI;
  return clampf(c, -HEADING_MAX_CORR_V, HEADING_MAX_CORR_V);
}

// Heading command for a straight: cardinal + wall-centring offset.
float centringHeading(float speed) {
  static float lastLat = 0.0f;
  static uint32_t lastUs = 0;
  float lat;
  if (speed < CENTER_MIN_SPEED_MM_S || !estimator::lateralMm(lat)) return g_headingTarget;
  const uint32_t now = micros();
  const float dt = (now - lastUs) * 1e-6f;
  const float rate = (dt > 0.0f && dt < 0.2f) ? (lat - lastLat) / dt : 0.0f;
  lastLat = lat;
  lastUs = now;
  // Robot left of centre (lat > 0) -> steer right (negative heading offset).
  const float offset = -(CENTER_KP_DEG_PER_MM * lat + CENTER_KD_DEG_S_PER_MM * rate);
  return g_headingTarget + clampf(offset, -CENTER_MAX_DEG, CENTER_MAX_DEG);
}

// Brake if the gap in front of the nose is below stopping distance + margin.
bool collisionImminent(float speed_mm_s) {
  if (speed_mm_s <= 0.0f) return false;
  const sonar::Reading f = sonar::read(sonar::FRONT);
  if (!f.inRange || sonar::ageMs(sonar::FRONT) > SONAR_FRESH_MS) return false;
  const float gapMm = f.mm - (ROBOT_NOSE_X_MM - SONAR_F_X_MM);
  const float stopMm = speed_mm_s * speed_mm_s / (2.0f * BRAKE_DECEL_MM_S2) +
                       speed_mm_s * SONAR_LATENCY_S + COLLISION_MARGIN_MM;
  return gapMm < stopMm;
}

// Spec: commanded motion but no front-range change and no gyro motion for
// STUCK_MS -> stop. Only judged while the command is well above dead-band.
bool stuck(bool useFrontRange) {
  const uint32_t now = millis();
  const float cmd = fmaxf(fabsf(motors::appliedLeftV()), fabsf(motors::appliedRightV()));
  if (cmd < STUCK_MIN_VOLTS) {
    g_lastMotionMs = now;
    g_lastWheelMotionMs = now;
    return false;
  }
  // Wheels blocked: the encoders see no rotation although clearly driven.
  if (encoders::healthy() && fabsf(encoders::leftSpeed()) < STUCK_MIN_WHEEL_MM_S &&
      fabsf(encoders::rightSpeed()) < STUCK_MIN_WHEEL_MM_S && now - g_lastWheelMotionMs > STUCK_MS) {
    return true;
  }
  if (fabsf(encoders::leftSpeed()) >= STUCK_MIN_WHEEL_MM_S || fabsf(encoders::rightSpeed()) >= STUCK_MIN_WHEEL_MM_S) {
    g_lastWheelMotionMs = now;
  }
  // Spec: no front-range change and no gyro motion (also catches wheels
  // spinning in place, which the encoders cannot see).
  bool moving = fabsf(imu::rateDps()) > STUCK_MAX_RATE_DPS;
  if (useFrontRange) {
    const sonar::Reading f = sonar::read(sonar::FRONT);
    if (!f.inRange) {
      moving = true;   // no wall in range: cannot judge, assume moving
    } else if (g_stuckRefRange < 0.0f || fabsf(f.mm - g_stuckRefRange) > STUCK_MIN_RANGE_CHANGE_MM) {
      g_stuckRefRange = f.mm;
      moving = true;
    }
  }
  if (moving) g_lastMotionMs = now;
  return now - g_lastMotionMs > STUCK_MS;
}

void tickRun(float dt) {
  const VelocityModel& m = motors::model();
  const bool forward = g_runSign > 0.0f;
  const float remaining = g_runSign * (g_runTarget - estimator::s());

  float vmax = g_tier.speed_mm_s;
  if (g_speedCap > 0.0f) vmax = fminf(vmax, g_speedCap);
  const estimator::Confidence conf = estimator::confidence();
  if (conf == estimator::Confidence::REDUCED) vmax = fminf(vmax, CONF_LOW_SPEED_MM_S);

  float vNew = 0.0f;
  if (remaining > STOP_TOLERANCE_MM) {
    const float vBrake = sqrtf(2.0f * g_tier.accel_mm_s2 * (remaining - STOP_TOLERANCE_MM));
    const float vTarget = fminf(vmax, vBrake);
    vNew = vTarget > g_vRef ? fminf(vTarget, g_vRef + g_tier.accel_mm_s2 * dt)
                            : fmaxf(vTarget, g_vRef - PROFILE_MAX_DECEL_MM_S2 * dt);
    vNew = fmaxf(vNew, PROFILE_MIN_SPEED_MM_S);
  }
  g_aRef = (vNew - g_vRef) / dt;
  g_vRef = vNew;

  // Feed-forward through the model, plus closed-loop speed from the encoders.
  const float vCmd = fmaxf(g_vRef + m.tau_s * g_aRef, 0.0f) / estimator::modelScale();
  const bool encUsable = encoders::healthy() && (encoders::leftUsable() || encoders::rightUsable());
  const float fb = speedFeedback(g_vRef, g_runSign * estimator::v(), &g_speedI, dt, encUsable);
  const float base = g_vRef > 0.0f ? g_runSign * (m.voltsFor(vCmd) + fb) : 0.0f;
  // Centring steers through the heading, which only works driving forward.
  const float theta = forward ? centringHeading(fmaxf(g_vRef, estimator::v())) : g_headingTarget;
  const float corr = headingCorrection(theta, g_vRef > 0.0f ? dt : 0.0f);
  motors::setVolts(base - corr, base + corr);
  sonar::setMotionHint(estimator::v(), imu::rateDps());

  if (conf == estimator::Confidence::LOST) {
    enterStopping(Result::ABORT_LOST);
  } else if (forward && collisionImminent(fmaxf(estimator::v(), g_vRef))) {
    enterStopping(Result::ABORT_COLLISION);
  } else if (remaining <= STOP_TOLERANCE_MM) {
    enterStopping(Result::DONE);
  } else if (stuck(true)) {
    enterStopping(Result::ABORT_STUCK);
  } else if (timedOut()) {
    enterStopping(Result::ABORT_TIMEOUT);
  }
}

void tickPivot(float dt) {
  g_turn.step(dt);
  const VelocityModel& m = motors::model();
  const float halfTrack = WHEEL_TRACK_MM * 0.5f;
  const float refDeg  = g_pivotStart + g_turnSign * g_turn.pos;
  const float refRate = g_turnSign * g_turn.vel;
  const float err     = refDeg - imu::headingDeg();
  const float rate    = imu::rateDps();

  // Wheel speed (mm/s): feed-forward + PD on angle and rate.
  float u = g_turnSign * (g_turn.vel + m.tau_s * g_turn.acc) * DEG2RAD * halfTrack +
            PIVOT_KP_MM_S_PER_DEG * err + PIVOT_KD_MM_S_PER_DPS * (refRate - rate);

  const float finalErr = g_headingTarget - imu::headingDeg();
  const bool inBand = fabsf(finalErr) < TURN_SETTLE_DEG && fabsf(rate) < TURN_SETTLE_RATE_DPS;
  // Inside half the settle band output nothing, so the dead-band feed-forward
  // cannot limit-cycle around the target.
  if (g_turn.done && fabsf(finalErr) < 0.5f * TURN_SETTLE_DEG && fabsf(rate) < TURN_SETTLE_RATE_DPS) u = 0.0f;
  // Each wheel follows +-u; encoder feedback keeps both wheels at that speed.
  float vr = m.voltsFor(u);
  float vl = -vr;
  if (PIVOT_WHEEL_FEEDBACK && fabsf(u) > 1.0f) {
    vr += speedFeedback(u, encoders::rightSpeed(), nullptr, dt, encoders::rightUsable());
    vl += speedFeedback(-u, encoders::leftSpeed(), nullptr, dt, encoders::leftUsable());
  }
  motors::setVolts(vl, vr);
  sonar::setMotionHint(0.0f, rate);

  if (g_turn.done && inBand) {
    if (!g_settling) {
      g_settling = true;
      g_settleSinceMs = millis();
    } else if (millis() - g_settleSinceMs >= TURN_SETTLE_HOLD_MS) {
      enterStopping(Result::DONE);
      return;
    }
  } else {
    g_settling = false;
  }
  if (stuck(false)) enterStopping(Result::ABORT_STUCK);
  else if (timedOut()) enterStopping(Result::ABORT_TIMEOUT);
}

void tickAlign() {
  const sonar::Reading f = sonar::read(sonar::FRONT);
  if (!f.inRange || sonar::ageMs(sonar::FRONT) > SONAR_FRESH_MS) {
    enterStopping(Result::DONE);   // nothing to align to
    return;
  }
  const float err = f.mm - FRONT_CENTRE_READING_MM;   // + = too far from the wall
  const float v = clampf(ALIGN_KP_PER_S * err, -ALIGN_MAX_SPEED_MM_S, ALIGN_MAX_SPEED_MM_S);
  const float base = fabsf(err) < ALIGN_TOLERANCE_MM ? 0.0f : motors::model().voltsFor(v);
  const float corr = headingCorrection(g_headingTarget, fabsf(base) > 0.0f ? CONTROL_TICK_S : 0.0f);
  motors::setVolts(base - corr, base + corr);

  if (fabsf(err) < ALIGN_TOLERANCE_MM) {
    if (!g_settling) {
      g_settling = true;
      g_settleSinceMs = millis();
    } else if (millis() - g_settleSinceMs >= ALIGN_HOLD_MS) {
      enterStopping(Result::DONE);
      return;
    }
  } else {
    g_settling = false;
  }
  if (stuck(true)) enterStopping(Result::ABORT_STUCK);
  else if (timedOut()) enterStopping(Result::DONE);   // best effort: alignment is optional
}

void tickStopping() {
  motors::setVolts(0.0f, 0.0f);
  const bool still = motors::idle() && fabsf(imu::rateDps()) < TURN_SETTLE_RATE_DPS;
  if (!still) {
    g_settling = false;
    return;
  }
  if (!g_settling) {
    g_settling = true;
    g_settleSinceMs = millis();
  } else if (millis() - g_settleSinceMs >= MOTION_STOP_SETTLE_MS) {
    g_mode = Mode::IDLE;
    g_result = g_pendingResult;
  }
}

// The control law, run by the scheduler every tick.
void tick(float dt) {
  estimator::tick(dt);

  if (g_mode != Mode::IDLE && motors::faultCode() != Fault::NONE) {
    g_mode = Mode::IDLE;
    g_result = Result::ABORT_FAULT;
    estimator::endStraight();
  }

  switch (g_mode) {
    case Mode::IDLE:
      motors::setVolts(0.0f, 0.0f);
      sonar::setMotionHint(0.0f, imu::rateDps());
      break;
    case Mode::RAW:
      motors::setVolts(g_rawL, g_rawR);
      break;
    case Mode::CONSTANT: {
      const float corr = g_holdHeading ? headingCorrection(g_headingTarget) : 0.0f;
      motors::setVolts(g_constV - corr, g_constV + corr);
      sonar::setMotionHint(estimator::v(), imu::rateDps());
      if (g_constV > 0.0f && collisionImminent(estimator::v())) enterStopping(Result::ABORT_COLLISION);
      break;
    }
    case Mode::RUN:      tickRun(dt); break;
    case Mode::PIVOT:    tickPivot(dt); break;
    case Mode::ALIGN:    tickAlign(); break;
    case Mode::STOPPING: tickStopping(); break;
  }
}

}  // namespace

namespace motion {

void begin() {
  g_mode = Mode::IDLE;
  g_result = Result::DONE;
  g_headingTarget = imu::headingDeg();
  sched::setController(&tick);
}

void runBegin(const SpeedTier& tier, float sStartMm, float sigmaMm) {
  g_tier = tier;
  g_runSign = 1.0f;
  g_runTarget = sStartMm;
  g_vRef = g_aRef = 0.0f;
  g_headingI = 0.0f;
  g_speedI = 0.0f;
  estimator::beginStraight(sStartMm, sigmaMm, g_headingTarget);
  startCommon(Mode::RUN, UINT32_MAX);
}

void runExtend(float mm) {
  g_runTarget += g_runSign * mm;
  // Generous timeout: the whole remaining run at the slowest crawl + 2 s.
  const float remaining = fabsf(g_runTarget - estimator::s());
  const uint32_t expectedMs = static_cast<uint32_t>(
      1000.0f * (remaining / fminf(g_tier.speed_mm_s, CONF_LOW_SPEED_MM_S) + g_tier.speed_mm_s / g_tier.accel_mm_s2));
  g_startMs = millis();
  g_timeoutMs = 2u * expectedMs + 2000u;
}

float runTargetMm() { return g_runTarget; }

void straight(float dist_mm, const SpeedTier& tier) {
  runBegin(tier, 0.0f, START_SIGMA_MM);
  runExtend(fmaxf(dist_mm, 0.0f));
}

void reverse(float dist_mm, const SpeedTier& tier) {
  runBegin(tier, 0.0f, ALIGNED_SIGMA_MM);
  // Driven at an angle to the grid: sonar fixes would be wrong.
  estimator::beginStraight(0.0f, ALIGNED_SIGMA_MM, g_headingTarget, false);
  g_runSign = -1.0f;
  runExtend(fmaxf(dist_mm, 0.0f));
}

void setSpeedCap(float mm_s) { g_speedCap = mm_s; }

void pivot(float angle_deg, const SpeedTier& tier) {
  estimator::endStraight();
  g_turnSign = angle_deg >= 0.0f ? 1.0f : -1.0f;
  g_pivotStart = g_headingTarget;
  g_headingTarget += angle_deg;   // targets accumulate: no drift from settle errors
  g_turn.start(fabsf(angle_deg), tier.turnRate_dps, TURN_ACCEL_DPS2);
  const uint32_t expectedMs = static_cast<uint32_t>(g_turn.duration() * 1000.0f);
  startCommon(Mode::PIVOT, expectedMs + TURN_TIMEOUT_MS);
}

void alignFront() {
  estimator::endStraight();
  startCommon(Mode::ALIGN, ALIGN_TIMEOUT_MS);
}

void constantVolts(float volts, bool holdHeading) {
  g_constV = clampf(volts, -MOTOR_VMAX_V, MOTOR_VMAX_V);
  g_holdHeading = holdHeading;
  startCommon(Mode::CONSTANT, UINT32_MAX);
}

void rawVolts(float left_v, float right_v) {
  g_rawL = left_v;
  g_rawR = right_v;
  startCommon(Mode::RAW, UINT32_MAX);
}

void stop() {
  if (g_mode == Mode::IDLE || g_mode == Mode::STOPPING) return;
  // A deliberate stop of an open-ended mode (raw / constant) is a normal end.
  const bool openEnded = g_mode == Mode::RAW || g_mode == Mode::CONSTANT;
  enterStopping(openEnded ? Result::DONE : Result::ABORT_USER);
}

bool busy() { return g_mode != Mode::IDLE; }
Result result() { return g_result; }

const char* resultName(Result r) {
  switch (r) {
    case Result::RUNNING:         return "running";
    case Result::DONE:            return "done";
    case Result::ABORT_COLLISION: return "collision guard";
    case Result::ABORT_TIMEOUT:   return "timeout";
    case Result::ABORT_FAULT:     return "motor fault";
    case Result::ABORT_USER:      return "user stop";
    case Result::ABORT_STUCK:     return "stuck";
    case Result::ABORT_LOST:      return "position lost";
  }
  return "?";
}

Telemetry telemetry() {
  Telemetry t;
  t.sRefMm = g_runTarget;
  t.vRefMmS = g_vRef;
  t.sEstMm = estimator::s();
  t.vEstMmS = estimator::v();
  t.sigmaMm = estimator::sigma();
  t.leftV = motors::appliedLeftV();
  t.rightV = motors::appliedRightV();
  t.headingDeg = imu::headingDeg();
  t.targetDeg = g_headingTarget;
  return t;
}

float headingTargetDeg() { return g_headingTarget; }
void  setHeadingTargetDeg(float deg) { g_headingTarget = deg; }

}  // namespace motion
