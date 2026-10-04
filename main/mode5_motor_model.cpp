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
  const float f = motion::settledFrontMm();
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
  motion::waitDone();
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
  const float d0 = motion::settledFrontMm();
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
  motion::waitDone();
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
  const float f0 = motion::settledFrontMm();
  if (f0 < CAL_START_MIN_MM || f0 > CAL_START_MAX_MM) {
    DBG_PRINTF("front range %.0f mm is outside the start window - refused.\n", f0);
    ui::soundError();
    return;
  }
  if (!motion::arm()) return;
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
  motion::reportFault();
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

}

namespace modes {

void mode5MotorModel() { modeAutoCal(); }

}
