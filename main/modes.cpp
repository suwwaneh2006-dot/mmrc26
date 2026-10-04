#include "modes.h"

#include <Preferences.h>

#include "battery.h"
#include "calib.h"
#include "encoders.h"
#include "estimator.h"
#include "imu.h"
#include "motion.h"
#include "motors.h"
#include "sched.h"
#include "sonar.h"
#include "strategy.h"
#include "ui.h"

namespace {

constexpr uint8_t MODE_COUNT = 8;
constexpr float   MOTOR_TEST_V = 2.0f;
constexpr uint32_t MOTOR_TEST_PHASE_MS = 1500;
constexpr uint32_t BATT_WARN_PERIOD_MS = 10000;
constexpr uint32_t LOG_PERIOD_MS = 5;
constexpr uint32_t LOG_TAIL_MS = 400;

uint8_t g_sonarMask = 0;
bool    g_imuOk = false;

bool allPartsOk() {
  for (uint8_t i = 0; i < sonar::COUNT; ++i) {
    if (!(g_sonarMask & (1u << i)) || sonar::faulty(static_cast<sonar::Id>(i))) return false;
  }
  return g_imuOk && imu::ok();
}

uint8_t errorCode() {
  if (!(g_sonarMask & (1u << sonar::FRONT)) || sonar::faulty(sonar::FRONT)) return ui::ERR_SONAR_FRONT;
  if (!(g_sonarMask & (1u << sonar::LEFT))  || sonar::faulty(sonar::LEFT))  return ui::ERR_SONAR_LEFT;
  if (!(g_sonarMask & (1u << sonar::RIGHT)) || sonar::faulty(sonar::RIGHT)) return ui::ERR_SONAR_RIGHT;
  if (!g_imuOk || !imu::ok()) return ui::ERR_IMU;
  if (!battery::armAllowed()) return ui::ERR_BATTERY;
  if (motors::faultCode() != Fault::NONE) return ui::ERR_MOTOR_FAULT;
  return 0;
}

void showIdleLed() {
  const uint8_t code = errorCode();
  if (code != 0) ui::led(ui::Led::CODE, code);
  else ui::led(ui::Led::BLINK_SLOW);
}

void waitBeeps() {
  while (ui::beepBusy()) sched::service();
}

float settledFrontMm() {
  sched::waitMs(150);
  const sonar::Reading r = sonar::read(sonar::FRONT);
  return r.inRange ? r.mm : -1.0f;
}

bool waitStartPress() {
  ui::clearEvents();
  ui::led(ui::Led::ON);
  while (true) {
    sched::service();
    const ui::Button b = ui::event();
    if (b == ui::Button::SHORT) return true;
    if (b == ui::Button::LONG) return false;
  }
}

bool armMotors() {
  motors::clearFault();
  encoders::clearFaults();
  motion::setHeadingTargetDeg(imu::headingDeg());
  return motors::enable();
}

motion::Result waitMotion() {
  ui::clearEvents();
  while (motion::busy()) {
    sched::service();
    if (ui::event() == ui::Button::SHORT) motion::stop();
  }
  return motion::result();
}

void reportFaultIfAny() {
  if (motors::faultCode() != Fault::NONE) {
    DBG_PRINTF("!! motor fault latched: %s\n", faultName(motors::faultCode()));
    ui::soundAbort();
  }
}

uint8_t readClicks() {
  ui::clearEvents();
  uint8_t n = 0;
  uint32_t lastClickMs = 0;
  uint32_t lastWarnMs = millis();
  while (true) {
    sched::service();
    showIdleLed();
    if (battery::warnLow() && millis() - lastWarnMs > BATT_WARN_PERIOD_MS) {
      lastWarnMs = millis();
      ui::soundBatteryWarn();
    }
    const ui::Button b = ui::event();
    if (b == ui::Button::SHORT) {
      ++n;
      lastClickMs = millis();
    } else if (b == ui::Button::LONG) {
      return 0;
    }
    if (n > 0 && !ui::buttonDown() && millis() - lastClickMs > CLICK_GAP_MS) return n;
  }
}

void printReading(const char* tag, sonar::Id id) {
  const sonar::Reading r = sonar::read(id);
  const sonar::Stats s = sonar::stats(id);
  if (!r.valid) {
    DBG_PRINTF("%s ---- ", tag);
  } else if (r.inRange) {
    DBG_PRINTF("%s %5.0f%c ", tag, r.mm, r.wall ? 'W' : '.');
  } else {
    DBG_PRINTF("%s    NW ", tag);
  }
  DBG_PRINTF("(%s) ", s.faulty ? "FAULT" : "ok");
}

void modeSensorDump() {
  DBG_PRINTF("\n== MODE 1: sensor dump (short press = exit) ==\n");
  DBG_PRINTF("W = wall flag, NW = no wall in range. Turn the robot LEFT by hand: hdg must increase.\n");
  DBG_PRINTF("Encoder calibration: turn one wheel exactly 10 turns by hand; ENC_EDGES_PER_WHEEL_REV =\n"
             "(change of that wheel's edge count) / 10.\n");
  ui::clearEvents();
  uint32_t lastPrintMs = 0;
  uint32_t lastStatsMs = millis();
  while (true) {
    sched::service();
    if (ui::event() == ui::Button::SHORT) break;
    ui::led(sonar::read(sonar::FRONT).wall ? ui::Led::ON : ui::Led::OFF);

    const uint32_t now = millis();
    if (now - lastPrintMs >= 100) {
      lastPrintMs = now;
      printReading("F", sonar::FRONT);
      printReading("L", sonar::LEFT);
      printReading("R", sonar::RIGHT);
      DBG_PRINTF("| gyro %6.1f dps hdg %7.1f | enc L %lu R %lu edges | batt %.2f V%s\n", imu::rateDps(),
                 imu::headingDeg(), static_cast<unsigned long>(encoders::leftEdges()),
                 static_cast<unsigned long>(encoders::rightEdges()), battery::volts(),
                 battery::warnLow() ? " LOW" : "");
    }
    if (now - lastStatsMs >= 5000) {
      lastStatsMs = now;
      for (uint8_t i = 0; i < sonar::COUNT; ++i) {
        const sonar::Stats s = sonar::stats(static_cast<sonar::Id>(i));
        DBG_PRINTF("  [%s] pings %lu echoes %lu misses %lu gate-rejects %lu\n",
                   sonar::name(static_cast<sonar::Id>(i)), static_cast<unsigned long>(s.pings),
                   static_cast<unsigned long>(s.echoes), static_cast<unsigned long>(s.misses),
                   static_cast<unsigned long>(s.gateRejects));
      }
      DBG_PRINTF("  [imu] ok %d who 0x%02X bias %.3f dps i2c-errors %lu | [loop] overruns %lu max-late %lu us\n",
                 imu::ok() ? 1 : 0, imu::whoAmI(), imu::biasDps(), static_cast<unsigned long>(imu::errorCount()),
                 static_cast<unsigned long>(sched::overruns()), static_cast<unsigned long>(sched::maxLateUs()));
    }
  }
}

void modeMotorTest() {
  struct Phase { float l, r; const char* text; };
  const Phase phases[] = {
    { MOTOR_TEST_V, 0.0f, "LEFT wheel FORWARD"},
    {-MOTOR_TEST_V, 0.0f, "LEFT wheel BACKWARD"},
    {0.0f,  MOTOR_TEST_V, "RIGHT wheel FORWARD"},
    {0.0f, -MOTOR_TEST_V, "RIGHT wheel BACKWARD"},
    { MOTOR_TEST_V, MOTOR_TEST_V, "BOTH wheels FORWARD"},
  };
  DBG_PRINTF("\n== MODE 2: motor direction test (%.1f V). Beep count = phase number ==\n", MOTOR_TEST_V);
  DBG_PRINTF("If a wheel turns the wrong way, flip MOTOR_L_INVERT / MOTOR_R_INVERT in config.h.\n");
  DBG_PRINTF("An encoder line saying CHECK while the wheel turned right: encoder wire / supply / pin.\n");
  if (!armMotors()) return;
  uint8_t idx = 0;
  bool aborted = false;
  for (const Phase& p : phases) {
    ++idx;
    ui::beep(idx, 60, 120);
    waitBeeps();
    DBG_PRINTF("phase %u: expect %s\n", idx, p.text);
    const float l0 = encoders::leftMm(), r0 = encoders::rightMm();
    motion::rawVolts(p.l, p.r);
    const uint32_t t0 = millis();
    ui::clearEvents();
    while (millis() - t0 < MOTOR_TEST_PHASE_MS && motors::faultCode() == Fault::NONE) {
      sched::service();
      if (ui::event() == ui::Button::SHORT) {
        aborted = true;
        break;
      }
    }
    motion::stop();
    waitMotion();

    const float dl = encoders::leftMm() - l0, dr = encoders::rightMm() - r0;
    const bool lOk = p.l == 0.0f ? fabsf(dl) < 5.0f : (dl * p.l > 0.0f && fabsf(dl) > 20.0f);
    const bool rOk = p.r == 0.0f ? fabsf(dr) < 5.0f : (dr * p.r > 0.0f && fabsf(dr) > 20.0f);
    DBG_PRINTF("   encoders: left %+.0f mm %s, right %+.0f mm %s\n", dl, lOk ? "ok" : "CHECK", dr,
               rOk ? "ok" : "CHECK");
    if (aborted || motors::faultCode() != Fault::NONE) break;
    sched::waitMs(300);
  }
  motors::disable();
  reportFaultIfAny();
  if (aborted) ui::soundAbort();
  else ui::soundOk();
}

void modeStraight5() {
  const float dist = 5.0f * CELL_PITCH_MM;
  DBG_PRINTF("\n== MODE 3: straight %.0f mm (5 cells) at T1 ==\n", dist);
  if (!motors::modelIsCalibrated()) DBG_PRINTF("NOTE: velocity model not calibrated (run mode 5 first).\n");
  const float f0 = settledFrontMm();
  if (!armMotors()) return;
  const float h0 = motion::headingTargetDeg();
  const float e0 = 0.5f * (encoders::leftMm() + encoders::rightMm());
  motion::straight(dist, TIERS[0]);
  const motion::Result res = waitMotion();
  const motion::Telemetry t = motion::telemetry();
  motors::disable();
  const float f1 = settledFrontMm();

  DBG_PRINTF("result: %s\n", motion::resultName(res));
  DBG_PRINTF("encoders: %.0f mm (raw wheel travel; %s)\n", 0.5f * (encoders::leftMm() + encoders::rightMm()) - e0,
             encoders::healthy() ? "healthy" : "FAULT - model fallback used");
  DBG_PRINTF("Wheel calibration: measure the travel with a ruler; WHEEL_DIAMETER_MM *= ruler / (encoder mm above).\n");
  const estimator::Stats es = estimator::stats();
  DBG_PRINTF("estimated distance: %.0f mm (commanded %.0f), sigma %.1f mm, model scale %.3f\n", t.sEstMm, dist,
             t.sigmaMm, estimator::scale());
  DBG_PRINTF("fixes: front %lu (rejected %lu), post edges %lu (rejected %lu), heading %lu\n",
             static_cast<unsigned long>(es.frontFixes), static_cast<unsigned long>(es.frontRejects),
             static_cast<unsigned long>(es.edgeFixes), static_cast<unsigned long>(es.edgeRejects),
             static_cast<unsigned long>(es.headingFixes));
  if (f0 > 0.0f && f1 > 0.0f) DBG_PRINTF("front sonar moved: %.0f mm\n", f0 - f1);
  else DBG_PRINTF("front sonar: no wall at start or end - measure the travel with a ruler.\n");
  DBG_PRINTF("heading error at end: %.1f deg\n", t.headingDeg - h0);
  reportFaultIfAny();
  if (res == motion::Result::DONE) ui::soundOk();
  else ui::soundAbort();
}

void modePivot4() {
  DBG_PRINTF("\n== MODE 4: pivot 4x90 deg left at T1 ==\n");
  const float f0 = settledFrontMm();
  if (!armMotors()) return;
  const float start = motion::headingTargetDeg();
  motion::Result res = motion::Result::DONE;
  for (int i = 1; i <= 4; ++i) {
    const uint32_t t0 = millis();
    motion::pivot(90.0f, TIERS[0]);
    res = waitMotion();
    const float err = imu::headingDeg() - motion::headingTargetDeg();
    DBG_PRINTF("turn %d: %s, %lu ms, gyro error %+.2f deg\n", i, motion::resultName(res),
               static_cast<unsigned long>(millis() - t0), err);
    if (res != motion::Result::DONE) break;
    sched::waitMs(300);
  }
  const float total = imu::headingDeg() - (start + 360.0f);
  motors::disable();
  const float f1 = settledFrontMm();
  DBG_PRINTF("total gyro error after 360 deg: %+.2f deg\n", total);
  if (f0 > 0.0f && f1 > 0.0f) DBG_PRINTF("front range before %.0f / after %.0f mm\n", f0, f1);
  DBG_PRINTF("Now look at the robot: if it over-rotated by X deg in total (under = negative X),\n"
             "set IMU_GYRO_SCALE = %.4f * 360 / (360 + X).\n", IMU_GYRO_SCALE);
  reportFaultIfAny();
  if (res == motion::Result::DONE) ui::soundOk();
  else ui::soundAbort();
}

struct CalStep {
  float volts;
  float mm_s;
  float tau_s;
  bool  aborted;
};

constexpr int CAL_MAX_SAMPLES = 160;
float g_calT[CAL_MAX_SAMPLES];
float g_calD[CAL_MAX_SAMPLES];

bool calReverseToStart() {
  const float f = settledFrontMm();
  if (f > 0.0f && f >= CAL_START_MM - 20.0f) return true;
  motion::constantVolts(-CAL_REVERSE_V, true);
  const uint32_t t0 = millis();
  ui::clearEvents();
  bool ok = false;
  while (millis() - t0 < CAL_REVERSE_TIMEOUT_MS && motion::busy()) {
    sched::service();
    if (ui::event() == ui::Button::SHORT) break;
    const sonar::Reading r = sonar::read(sonar::FRONT);
    if (r.inRange && r.rawMm >= CAL_START_MM) {
      ok = true;
      break;
    }
  }
  motion::stop();
  waitMotion();
  return ok && motors::faultCode() == Fault::NONE;
}

bool fitLine(int n, float tMin, float& a, float& b, int& used) {
  double st = 0, sd = 0, stt = 0, std_ = 0;
  used = 0;
  for (int i = 0; i < n; ++i) {
    if (g_calT[i] < tMin) continue;
    st += g_calT[i];
    sd += g_calD[i];
    stt += static_cast<double>(g_calT[i]) * g_calT[i];
    std_ += static_cast<double>(g_calT[i]) * g_calD[i];
    ++used;
  }
  if (used < CAL_MIN_SAMPLES) return false;
  const double den = used * stt - st * st;
  if (fabs(den) < 1e-9) return false;
  b = static_cast<float>((used * std_ - st * sd) / den);
  a = static_cast<float>((sd - b * st) / used);
  return true;
}

CalStep calRunStep(float volts) {
  CalStep out{volts, 0.0f, -1.0f, false};
  const float d0 = settledFrontMm();
  if (d0 < 0.0f) {
    out.aborted = true;
    return out;
  }
  int n = 0;
  uint32_t seq = sonar::read(sonar::FRONT).seq;
  const uint32_t t0us = micros();
  const uint32_t t0ms = millis();
  float lastD = d0, prevD = d0, prevT = 0.0f, vEst = 0.0f;
  bool noMotion = false;

  motion::constantVolts(volts, true);
  ui::clearEvents();
  while (motion::busy()) {
    sched::service();
    if (ui::event() == ui::Button::SHORT) {
      out.aborted = true;
      break;
    }
    const sonar::Reading r = sonar::read(sonar::FRONT);
    if (r.seq != seq) {
      seq = r.seq;
      if (r.rawMm <= FRONT_MAX_MM) {
        const float t = static_cast<int32_t>(r.tUs - t0us) * 1e-6f;
        if (n < CAL_MAX_SAMPLES) {
          g_calT[n] = t;
          g_calD[n] = r.rawMm;
          ++n;
        }
        if (t > prevT + 1e-3f) {
          vEst = 0.7f * vEst + 0.3f * (prevD - r.rawMm) / (t - prevT);
          prevT = t;
          prevD = r.rawMm;
        }
        lastD = r.rawMm;
      }
    }
    const uint32_t elapsed = millis() - t0ms;
    const float v = fmaxf(vEst, 0.0f);
    if (lastD < CAL_STOP_MM + v * v / (2.0f * BRAKE_DECEL_MM_S2)) break;
    if (elapsed > CAL_STEP_TIMEOUT_MS) break;
    if (elapsed > CAL_NO_MOTION_MS && d0 - lastD < CAL_NO_MOTION_MM) {
      noMotion = true;
      break;
    }
  }
  motion::stop();
  waitMotion();
  if (motors::faultCode() != Fault::NONE) out.aborted = true;
  if (out.aborted || noMotion) return out;

  float a = 0.0f, b = 0.0f;
  int used = 0;
  if (!fitLine(n, CAL_SETTLE_MS * 1e-3f, a, b, used) || -b < CAL_MIN_SPEED_MM_S) return out;
  out.mm_s = -b;

  const float lag = (a - d0) / out.mm_s;
  out.tau_s = lag - 0.5f * volts / MOTOR_SLEW_V_PER_S;
  DBG_PRINTF("  %.2f V -> %.0f mm/s (%d samples, lag %.0f ms, tau %.0f ms)\n", volts, out.mm_s, used,
             lag * 1000.0f, out.tau_s * 1000.0f);
  return out;
}

void modeAutoCal() {
  DBG_PRINTF("\n== MODE 5: auto-calibration. Face a wall %.0f-%.0f mm ahead, straight corridor ==\n",
             CAL_START_MIN_MM, CAL_START_MAX_MM);
  const float f0 = settledFrontMm();
  if (f0 < CAL_START_MIN_MM || f0 > CAL_START_MAX_MM) {
    DBG_PRINTF("front range %.0f mm is outside the start window - refused.\n", f0);
    ui::soundError();
    return;
  }
  if (!armMotors()) return;
  sonar::setFrontOnly(true);

  CalStep steps[CAL_STEP_COUNT];
  int stepCount = 0;
  bool aborted = false;
  for (int i = 0; i < CAL_STEP_COUNT; ++i) {
    if (!calReverseToStart()) {
      aborted = true;
      break;
    }
    sched::waitMs(300);
    steps[stepCount] = calRunStep(CAL_STEP_V[i]);
    if (steps[stepCount].aborted) {
      aborted = true;
      break;
    }
    if (steps[stepCount].mm_s == 0.0f) {
      DBG_PRINTF("  %.2f V -> no motion (dead-band)\n", CAL_STEP_V[i]);
    }
    const bool tooFast = steps[stepCount].mm_s > CAL_MAX_SPEED_MM_S;
    ++stepCount;
    if (tooFast) break;
  }
  sonar::setFrontOnly(false);
  motors::disable();
  reportFaultIfAny();
  if (aborted) {
    DBG_PRINTF("calibration aborted - nothing saved.\n");
    ui::soundAbort();
    return;
  }

  VelocityModel m = motors::defaultModel();
  float pv[CAL_STEP_COUNT], ps[CAL_STEP_COUNT], taus[CAL_STEP_COUNT];
  int np = 0, nt = 0;
  float highestStill = 0.0f;
  for (int i = 0; i < stepCount; ++i) {
    if (steps[i].mm_s <= 0.0f) {
      if (np == 0) highestStill = steps[i].volts;
      continue;
    }
    if (np > 0 && steps[i].mm_s <= ps[np - 1]) continue;
    pv[np] = steps[i].volts;
    ps[np] = steps[i].mm_s;
    ++np;
    if (steps[i].tau_s > 0.0f) taus[nt++] = steps[i].tau_s;
  }
  if (np < CAL_MIN_POINTS) {
    DBG_PRINTF("only %d usable points (need %d) - nothing saved.\n", np, CAL_MIN_POINTS);
    ui::soundError();
    return;
  }

  float v0 = pv[0] - ps[0] * (pv[1] - pv[0]) / (ps[1] - ps[0]);
  v0 = fmaxf(v0, highestStill);
  v0 = fminf(fmaxf(v0, 0.0f), pv[0] - 0.05f);
  m.count = 0;
  m.volts[m.count] = v0;
  m.mm_s[m.count] = 0.0f;
  ++m.count;
  for (int i = 0; i < np && m.count < MODEL_MAX_POINTS; ++i) {
    m.volts[m.count] = pv[i];
    m.mm_s[m.count] = ps[i];
    ++m.count;
  }

  for (int i = 1; i < nt; ++i) {
    for (int j = i; j > 0 && taus[j] < taus[j - 1]; --j) {
      const float tmp = taus[j];
      taus[j] = taus[j - 1];
      taus[j - 1] = tmp;
    }
  }
  const float tau = nt > 0 ? taus[nt / 2] : MODEL_DEFAULT_TAU_S;
  m.tau_s = fminf(fmaxf(tau, 0.01f), 0.3f);

  DBG_PRINTF("model: tau %.0f ms, points:", m.tau_s * 1000.0f);
  for (int i = 0; i < m.count; ++i) DBG_PRINTF(" (%.2f V, %.0f)", m.volts[i], m.mm_s[i]);
  DBG_PRINTF("\n");
  if (motors::saveModel(m)) {
    DBG_PRINTF("saved to NVS.\n");
    ui::soundCalSaved();
  } else {
    DBG_PRINTF("model invalid or NVS write failed - nothing saved.\n");
    ui::soundError();
  }
}

struct LogRow {
  uint16_t tMs;
  float sRef, vRef, sEst, vEst, sigma, leftV, rightV, heading, frontMm, leftMm, rightMm;
};
constexpr int LOG_ROWS = 500;
LogRow g_log[LOG_ROWS];

void modeStepTest() {
  DBG_PRINTF("\n== MODE 6: one-cell step test at T1 ==\n");
  const float f0 = settledFrontMm();
  if (!armMotors()) return;
  int rows = 0;
  const uint32_t t0 = millis();
  uint32_t nextLog = t0;
  uint32_t stoppedAt = 0;
  motion::straight(CELL_PITCH_MM, TIERS[0]);
  ui::clearEvents();
  while (true) {
    sched::service();
    if (ui::event() == ui::Button::SHORT) motion::stop();
    const uint32_t now = millis();
    if (static_cast<int32_t>(now - nextLog) >= 0 && rows < LOG_ROWS) {
      nextLog += LOG_PERIOD_MS;
      const motion::Telemetry t = motion::telemetry();
      const sonar::Reading f = sonar::read(sonar::FRONT);
      g_log[rows++] = LogRow{static_cast<uint16_t>(now - t0), t.sRefMm, t.vRefMmS, t.sEstMm, t.vEstMmS, t.sigmaMm,
                             t.leftV, t.rightV, t.headingDeg, f.inRange ? f.mm : -1.0f,
                             sonar::read(sonar::LEFT).rawMm, sonar::read(sonar::RIGHT).rawMm};
    }
    if (!motion::busy() && stoppedAt == 0) stoppedAt = now;
    if (stoppedAt != 0 && now - stoppedAt >= LOG_TAIL_MS) break;
  }
  const motion::Result res = motion::result();
  motors::disable();
  const float f1 = settledFrontMm();

  DBG_PRINTF("result: %s\n", motion::resultName(res));
  DBG_PRINTF("t_ms,s_target,v_ref,s_est,v_est,sigma,left_v,right_v,heading,front_mm,left_raw,right_raw\n");
  for (int i = 0; i < rows; ++i) {
    const LogRow& r = g_log[i];
    DBG_PRINTF("%u,%.1f,%.0f,%.1f,%.0f,%.1f,%.2f,%.2f,%.2f,%.0f,%.0f,%.0f\n", r.tMs, r.sRef, r.vRef, r.sEst, r.vEst,
               r.sigma, r.leftV, r.rightV, r.heading, r.frontMm, r.leftMm, r.rightMm);
  }
  if (f0 > 0.0f && f1 > 0.0f) DBG_PRINTF("front sonar moved %.0f mm (target %.0f)\n", f0 - f1, CELL_PITCH_MM);
  reportFaultIfAny();
  if (res == motion::Result::DONE) ui::soundOk();
  else ui::soundAbort();
}

float encoderTravel() { return 0.5f * (encoders::leftMm() + encoders::rightMm()); }

constexpr int CAL8_MAX_SAMPLES = 900;
float g_scanHeading[CAL8_MAX_SAMPLES];
float g_scanRange[CAL8_MAX_SAMPLES];

bool fitMinimum(int n, float centre, float& minHeading) {
  double s[5] = {0, 0, 0, 0, 0}, t[3] = {0, 0, 0};
  int used = 0;
  for (int i = 0; i < n; ++i) {
    const double x = g_scanHeading[i] - centre;
    if (fabs(x) > CAL8_WINDOW_DEG) continue;
    const double y = g_scanRange[i];
    double p = 1.0;
    for (int k = 0; k < 5; ++k) {
      s[k] += p;
      if (k < 3) t[k] += p * y;
      p *= x;
    }
    ++used;
  }
  if (used < CAL8_MIN_FIT_SAMPLES) return false;
  const double a[3][3] = {{s[0], s[1], s[2]}, {s[1], s[2], s[3]}, {s[2], s[3], s[4]}};
  const double det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
                     a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
                     a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
  if (fabs(det) < 1e-12) return false;
  const double db = a[0][0] * (t[1] * a[2][2] - a[1][2] * t[2]) - t[0] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
                    a[0][2] * (a[1][0] * t[2] - t[1] * a[2][0]);
  const double dc = a[0][0] * (a[1][1] * t[2] - t[1] * a[2][1]) - a[0][1] * (a[1][0] * t[2] - t[1] * a[2][0]) +
                    t[0] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
  const double b = db / det, c = dc / det;
  if (c <= 0.0) return false;
  const double vertex = -b / (2.0 * c);
  if (fabs(vertex) > CAL8_WINDOW_DEG) return false;
  minHeading = static_cast<float>(centre + vertex);
  return true;
}

bool scanTurn(float dir, float& measuredDeg) {
  const float start = motion::headingTargetDeg();
  motion::pivot(-dir * CAL8_SCAN_MARGIN_DEG, CAL8_TIER);
  if (waitMotion() != motion::Result::DONE) return false;
  int n = 0;
  uint32_t seq = sonar::read(sonar::FRONT).seq;
  motion::pivot(dir * (360.0f + 2.0f * CAL8_SCAN_MARGIN_DEG), CAL8_TIER);
  ui::clearEvents();
  while (motion::busy()) {
    sched::service();
    if (ui::event() == ui::Button::SHORT) motion::stop();
    const sonar::Reading r = sonar::read(sonar::FRONT);
    if (r.seq != seq) {
      seq = r.seq;
      if (r.rawMm <= FRONT_MAX_MM && n < CAL8_MAX_SAMPLES) {
        g_scanHeading[n] = imu::headingDeg();
        g_scanRange[n] = r.rawMm;
        ++n;
      }
    }
  }
  if (motion::result() != motion::Result::DONE) return false;
  motion::pivot(-dir * CAL8_SCAN_MARGIN_DEG, CAL8_TIER);
  if (waitMotion() != motion::Result::DONE) return false;
  float m1, m2;
  if (!fitMinimum(n, start, m1) || !fitMinimum(n, start + dir * 360.0f, m2)) {
    DBG_PRINTF("   no clear wall minimum in the scan (%d samples)\n", n);
    return false;
  }
  measuredDeg = fabsf(m2 - m1);
  return measuredDeg > 300.0f && measuredDeg < 420.0f;
}

void modeDirectionCal() {
  DBG_PRINTF("\n== MODE 8: direction calibration: forward, right, left, back ==\n");
  DBG_PRINTF("Face a flat wall %.0f-%.0f mm ahead, open space around, room behind.\n", CAL8_MIN_FRONT_MM,
             CAL8_MAX_FRONT_MM);
  const float f0 = settledFrontMm();
  if (f0 < CAL8_MIN_FRONT_MM || f0 > CAL8_MAX_FRONT_MM) {
    DBG_PRINTF("front range %.0f mm outside the window - refused.\n", f0);
    ui::soundError();
    return;
  }
  if (!armMotors()) return;
  sonar::setFrontOnly(true);
  calib::Direction d = calib::get();
  bool ok = true;

  const float e0 = encoderTravel();
  motion::straight(f0 - CAL8_STOP_FRONT_MM, CAL8_TIER);
  ok = waitMotion() == motion::Result::DONE;
  const float f1 = settledFrontMm();
  const float fwdTrue = f0 - f1, fwdEnc = encoderTravel() - e0;
  if (ok && f1 > 0.0f && fwdEnc > CAL8_MIN_TRAVEL_MM && fwdTrue > CAL8_MIN_TRAVEL_MM) {
    d.forward *= fwdTrue / fwdEnc;
    DBG_PRINTF("forward : sonar %.1f mm, encoders %.1f mm -> scale %.4f\n", fwdTrue, fwdEnc, d.forward);
  } else {
    ok = false;
    DBG_PRINTF("forward : FAILED\n");
  }

  float deg = 0.0f;
  if (ok && scanTurn(-1.0f, deg)) {
    d.cw *= 360.0f / deg;
    DBG_PRINTF("right   : gyro measured %.2f deg for 360 -> scale %.4f\n", deg, d.cw);
  } else {
    ok = false;
    DBG_PRINTF("right   : FAILED\n");
  }

  if (ok && scanTurn(1.0f, deg)) {
    d.ccw *= 360.0f / deg;
    DBG_PRINTF("left    : gyro measured %.2f deg for 360 -> scale %.4f\n", deg, d.ccw);
  } else {
    ok = false;
    DBG_PRINTF("left    : FAILED\n");
  }

  if (ok) {
    const float b0 = settledFrontMm();
    const float eb = encoderTravel();
    motion::reverse(fminf(fwdTrue, CAL8_MAX_FRONT_MM) - 20.0f, CAL8_TIER);
    ok = waitMotion() == motion::Result::DONE;
    const float b1 = settledFrontMm();
    const float backTrue = b1 - b0, backEnc = eb - encoderTravel();
    if (ok && b0 > 0.0f && b1 > 0.0f && backEnc > CAL8_MIN_TRAVEL_MM && backTrue > CAL8_MIN_TRAVEL_MM) {
      d.backward *= backTrue / backEnc;
      DBG_PRINTF("back    : sonar %.1f mm, encoders %.1f mm -> scale %.4f\n", backTrue, backEnc, d.backward);
    } else {
      ok = false;
      DBG_PRINTF("back    : FAILED\n");
    }
  }

  sonar::setFrontOnly(false);
  motors::disable();
  reportFaultIfAny();
  if (ok && calib::save(d)) {
    DBG_PRINTF("saved: forward %.4f back %.4f right %.4f left %.4f\n", d.forward, d.backward, d.cw, d.ccw);
    ui::soundCalSaved();
  } else {
    DBG_PRINTF("calibration not saved (a step failed or a scale is outside %.2f-%.2f).\n", CAL8_SCALE_MIN,
               CAL8_SCALE_MAX);
    ui::soundError();
  }
}
void runMode(uint8_t n) {
  ui::beep(n, 80, 150);
  waitBeeps();
  if (n < 1 || n > MODE_COUNT) {
    DBG_PRINTF("mode %u is not available in this build.\n", n);
    ui::soundError();
    return;
  }
  if (n == 1) {
    modeSensorDump();
    return;
  }

  if (!battery::armAllowed()) {
    DBG_PRINTF("battery %.2f V < %.2f V - refused.\n", battery::volts(), BATT_REFUSE_ARM_V);
    ui::soundError();
    return;
  }
  if (n >= 3 && !allPartsOk()) {
    DBG_PRINTF("a sensor is missing or faulty (error code %u) - refused.\n", errorCode());
    ui::soundError();
    return;
  }
  if (n == 7) {
    if (!ROBOT_MAZE_READY) {
      DBG_PRINTF("match refused: the chassis cannot pivot / stop on a cell centre in a 171-189 mm cell "
                 "(front corner swing %.0f mm, nose %.0f mm ahead of the axle). Move the axle forward.\n",
                 sqrtf(PIVOT_CORNER_RADIUS_SQ_MM2), ROBOT_NOSE_X_MM);
      ui::soundError();
      return;
    }
    strategy::runMatch();
    motors::disable();
    return;
  }
  DBG_PRINTF("mode %u: short press = start, long press = cancel\n", n);
  if (!waitStartPress()) {
    ui::soundAbort();
    return;
  }
  ui::led(ui::Led::BLINK_FAST);
  sched::waitMs(MODE_START_DELAY_MS);

  switch (n) {
    case 2: modeMotorTest(); break;
    case 3: modeStraight5(); break;
    case 4: modePivot4(); break;
    case 5: modeAutoCal(); break;
    case 6: modeStepTest(); break;
    case 8: modeDirectionCal(); break;
    default: break;
  }
  motors::disable();
}

}

namespace modes {

void bootWipeCheck() {
  ui::update();
  if (digitalRead(PIN_BUTTON) != LOW) return;
  const uint32_t t0 = millis();
  ui::led(ui::Led::BLINK_FAST);
  while (digitalRead(PIN_BUTTON) == LOW) {
    ui::update();
    if (millis() - t0 >= BOOT_WIPE_HOLD_MS) {
      Preferences prefs;
      if (prefs.begin(NVS_MAP_NAMESPACE, false)) {
        prefs.clear();
        prefs.end();
      }
      DBG_PRINTF("map wiped.\n");
      ui::soundWiped();
      while (digitalRead(PIN_BUTTON) == LOW || ui::beepBusy()) ui::update();
      break;
    }
  }
  ui::led(ui::Led::OFF);
  delay(300);
  ui::clearEvents();
}

void bootSelfCheck(bool imuOk) {
  g_imuOk = imuOk;
  g_sonarMask = sonar::selfTest();
  uint8_t working = imuOk ? 1 : 0;
  for (uint8_t i = 0; i < sonar::COUNT; ++i) {
    const bool ok = (g_sonarMask & (1u << i)) != 0;
    if (ok) ++working;
    DBG_PRINTF("sonar %-5s : %s\n", sonar::name(static_cast<sonar::Id>(i)), ok ? "ok" : "MISSING");
  }
  DBG_PRINTF("imu         : %s (WHO_AM_I 0x%02X, bias %.3f dps)\n", imuOk ? "ok" : "MISSING", imu::whoAmI(),
             imu::biasDps());
  DBG_PRINTF("battery     : %.2f V\n", battery::volts());
  DBG_PRINTF("model       : %s\n", motors::modelIsCalibrated() ? "calibrated (NVS)" : "DEFAULT (run mode 5)");
  ui::beep(working, 80, 150);
  waitBeeps();
  if (working < 4) ui::soundError();
  showIdleLed();
  DBG_PRINTF("ready: click 1-8 to select a mode.\n");
}

void menu() {
  const uint8_t n = readClicks();
  if (n > 0) runMode(n);
  DBG_PRINTF("ready: click 1-8 to select a mode.\n");
}

}
