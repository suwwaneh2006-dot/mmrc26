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
  const double b = db / det, cc = dc / det;
  if (cc <= 0.0) return false;
  const double vertex = -b / (2.0 * cc);
  if (fabs(vertex) > CAL8_WINDOW_DEG) return false;
  minHeading = static_cast<float>(centre + vertex);
  return true;
}

bool sweep(float angleDeg, int& n, uint32_t& edgesL, uint32_t& edgesR) {
  n = 0;
  const uint32_t l0 = encoders::leftEdges(), r0 = encoders::rightEdges();
  uint32_t seq = sonar::read(sonar::FRONT).seq;
  motion::pivot(angleDeg, CAL8_TIER);
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
  edgesL = encoders::leftEdges() - l0;
  edgesR = encoders::rightEdges() - r0;
  return motion::result() == motion::Result::DONE;
}

bool turnBy(float angleDeg) {
  motion::pivot(angleDeg, CAL8_TIER);
  return motion::waitDone() == motion::Result::DONE;
}

struct Probe {
  bool  ok;
  float dFront;
  float dHeading;
};

Probe probe(float vl, float vr) {
  Probe p{false, 0.0f, 0.0f};
  const float f0 = motion::settledFrontMm();
  const float h0 = imu::headingDeg();
  motion::rawVolts(vl, vr);
  sched::waitMs(CAL_PROBE_MS);
  motion::stop();
  motion::waitDone();
  const float f1 = motion::settledFrontMm();
  p.dHeading = imu::headingDeg() - h0;
  p.dFront = f1 - f0;
  p.ok = f0 > 0.0f && f1 > 0.0f && motors::faultCode() == Fault::NONE;
  if (motors::faultCode() != Fault::NONE) p.ok = false;
  return p;
}

void resetHeadingFrame() {
  imu::setHeading(0.0f);
  motion::setHeadingTargetDeg(0.0f);
}

bool backToRange(float targetFront) {
  const float f = motion::settledFrontMm();
  if (f <= 0.0f || f >= targetFront - 5.0f) return true;
  motion::reverse(targetFront - f, CAL8_TIER);
  return motion::waitDone() == motion::Result::DONE;
}

bool calWiring(calib::Data& d) {
  Probe p = probe(CAL_PROBE_V, CAL_PROBE_V);
  if (!p.ok) return false;
  const bool spin = fabsf(p.dHeading) > CAL_PROBE_SPIN_DEG;
  if (!spin && p.dFront < -CAL_PROBE_MOVE_MM) return true;
  if (!spin && p.dFront > CAL_PROBE_MOVE_MM) {
    d.invertLeft ^= 1;
    d.invertRight ^= 1;
    calib::applyWithoutSaving(d);
    return true;
  }
  if (!spin) return false;
  probe(-CAL_PROBE_V, -CAL_PROBE_V);
  d.invertLeft ^= 1;
  calib::applyWithoutSaving(d);
  const Probe q = probe(CAL_PROBE_V, CAL_PROBE_V);
  if (!q.ok || fabsf(q.dHeading) > CAL_PROBE_SPIN_DEG) {
    probe(-CAL_PROBE_V, -CAL_PROBE_V);
    return false;
  }
  if (q.dFront > CAL_PROBE_MOVE_MM) {
    d.invertLeft ^= 1;
    d.invertRight ^= 1;
    calib::applyWithoutSaving(d);
    return true;
  }
  return q.dFront < -CAL_PROBE_MOVE_MM;
}

bool calGyroSign(calib::Data& d) {
  const Probe p = probe(CAL_PROBE_V, -CAL_PROBE_V);
  if (fabsf(p.dHeading) < CAL_PROBE_SPIN_DEG) return false;
  if (p.dHeading > 0.0f) {
    d.gyroSign = -d.gyroSign;
    calib::applyWithoutSaving(d);
  }
  const Probe q = probe(-CAL_PROBE_V, CAL_PROBE_V);
  resetHeadingFrame();
  return motors::faultCode() == Fault::NONE && fabsf(q.dHeading) >= CAL_PROBE_SPIN_DEG;
}

bool findWall() {
  resetHeadingFrame();
  if (!turnBy(-CAL_FIND_WALL_DEG)) return false;
  int n;
  uint32_t el, er;
  if (!sweep(2.0f * CAL_FIND_WALL_DEG, n, el, er)) return false;
  float m;
  if (!fitMinimum(n, 0.0f, m)) {
    turnBy(-CAL_FIND_WALL_DEG);
    resetHeadingFrame();
    return false;
  }
  if (!turnBy(m - motion::headingTargetDeg())) return false;
  resetHeadingFrame();
  return true;
}

bool calStraight(bool forward, calib::Data& d, float& travelTrue) {
  const float f0 = motion::settledFrontMm();
  const uint32_t l0 = encoders::leftEdges(), r0 = encoders::rightEdges();
  if (forward) motion::straight(f0 - CAL8_STOP_FRONT_MM, CAL8_TIER);
  else motion::reverse(travelTrue - 20.0f, CAL8_TIER);
  const motion::Result res = motion::waitDone();
  if (res != motion::Result::DONE) {
    DBG_PRINTF("%-8s: motion ended with %s\n", forward ? "forward" : "back", motion::resultName(res));
    return false;
  }
  const float f1 = motion::settledFrontMm();
  const float dTrue = forward ? f0 - f1 : f1 - f0;
  DBG_PRINTF("%-8s: front %.0f -> %.0f mm\n", forward ? "forward" : "back", f0, f1);
  const float el = static_cast<float>(encoders::leftEdges() - l0);
  const float er = static_cast<float>(encoders::rightEdges() - r0);
  if (f0 <= 0.0f || f1 <= 0.0f || dTrue < CAL8_MIN_TRAVEL_MM || el < 20.0f || er < 20.0f) return false;
  const int k = forward ? 0 : 1;
  d.mmPerEdge[0][k] = dTrue / el;
  d.mmPerEdge[1][k] = dTrue / er;
  calib::applyWithoutSaving(d);
  travelTrue = dTrue;
  DBG_PRINTF("%-8s: sonar %.1f mm, edges L %.0f R %.0f -> mm/edge L %.4f R %.4f\n", forward ? "forward" : "back",
             dTrue, el, er, d.mmPerEdge[0][k], d.mmPerEdge[1][k]);
  return true;
}

bool calTurn(float dir, calib::Data& d, uint32_t& edgesL, uint32_t& edgesR, float& trueSweepRad) {
  resetHeadingFrame();
  const float sweepDeg = 360.0f + 2.0f * CAL8_SCAN_MARGIN_DEG;
  if (!turnBy(-dir * CAL8_SCAN_MARGIN_DEG)) return false;
  int n;
  if (!sweep(dir * sweepDeg, n, edgesL, edgesR)) return false;
  if (!turnBy(-dir * CAL8_SCAN_MARGIN_DEG)) return false;
  float m1, m2;
  if (!fitMinimum(n, 0.0f, m1) || !fitMinimum(n, dir * 360.0f, m2)) return false;
  const float measured = fabsf(m2 - m1);
  if (measured < 300.0f || measured > 420.0f) return false;
  float& scale = dir < 0.0f ? d.cw : d.ccw;
  scale *= 360.0f / measured;
  trueSweepRad = sweepDeg * (360.0f / measured) * 0.01745329f;
  calib::applyWithoutSaving(d);
  resetHeadingFrame();
  DBG_PRINTF("%-8s: gyro %.2f deg per true 360 -> scale %.4f\n", dir < 0.0f ? "right" : "left", measured, scale);
  return true;
}

bool calModel() {
  VelocityModel m = motors::defaultModel();
  float pv[CAL_SPIN_STEPS], ps[CAL_SPIN_STEPS];
  int np = 0;
  float tau = MODEL_DEFAULT_TAU_S;
  for (int i = 0; i < CAL_SPIN_STEPS; ++i) {
    const float v = CAL_SPIN_V[i];
    const uint32_t t0 = millis();
    motion::rawVolts(v, -v);
    float sum = 0.0f;
    int cnt = 0;
    float target63 = -1.0f;
    uint32_t t63 = 0;
    while (millis() - t0 < CAL_SPIN_HOLD_MS) {
      sched::service();
      if (motors::faultCode() != Fault::NONE) return false;
      const float sp = 0.5f * (fabsf(encoders::leftSpeed()) + fabsf(encoders::rightSpeed()));
      if (millis() - t0 >= CAL_SPIN_HOLD_MS - CAL_SPIN_MEASURE_MS) {
        sum += sp;
        ++cnt;
      }
      if (i == CAL_SPIN_STEPS / 2 && t63 == 0 && target63 > 0.0f && sp >= target63) t63 = millis() - t0;
      if (i == CAL_SPIN_STEPS / 2 && target63 < 0.0f && np > 0) target63 = 0.63f * ps[np - 1] * v / pv[np - 1];
    }
    motion::stop();
    motion::waitDone();
    const float speed = cnt > 0 ? sum / cnt : 0.0f;
    if (speed >= CAL_MIN_SPEED_MM_S && (np == 0 || speed > ps[np - 1])) {
      pv[np] = v;
      ps[np] = speed;
      ++np;
    }
    if (t63 > 0) tau = fminf(fmaxf(t63 * 1e-3f - 0.5f * v / MOTOR_SLEW_V_PER_S, 0.01f), 0.3f);
  }
  if (np < CAL_MIN_POINTS) return false;
  float v0 = pv[0] - ps[0] * (pv[1] - pv[0]) / (ps[1] - ps[0]);
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
  m.tau_s = tau;
  DBG_PRINTF("model   : dead-band %.2f V, tau %.0f ms, %d points, top %.0f mm/s at %.1f V\n", v0, tau * 1000.0f,
             np, ps[np - 1], pv[np - 1]);
  return motors::saveModel(m);
}

void modeAutoCalibrateAll() {
  DBG_PRINTF("\n== MODE 8: full auto-calibration ==\n");
  DBG_PRINTF("Face a flat wall %.0f-%.0f mm ahead, open floor all around (robot spins), room behind.\n",
             CAL8_MIN_FRONT_MM, CAL8_MAX_FRONT_MM);
  const float fStart = motion::settledFrontMm();
  if (fStart < CAL8_MIN_FRONT_MM || fStart > CAL8_MAX_FRONT_MM) {
    DBG_PRINTF("front range %.0f mm outside the window - refused.\n", fStart);
    ui::soundError();
    return;
  }
  calib::Data d = calib::get();
  d.mmPerEdge[0][0] = d.mmPerEdge[0][1] = d.mmPerEdge[1][0] = d.mmPerEdge[1][1] = ENC_MM_PER_EDGE;
  d.cw = d.ccw = 1.0f;
  calib::applyWithoutSaving(d);
  if (!motion::arm()) return;
  sonar::setFrontOnly(true);
  const char* failed = nullptr;
  uint32_t cwL = 0, cwR = 0, ccwL = 0, ccwR = 0;
  float cwRad = 0.0f, ccwRad = 0.0f, travel = 0.0f;

  if (!calWiring(d)) failed = "motor wiring";
  else DBG_PRINTF("wiring  : left %s, right %s\n", d.invertLeft ? "INVERTED" : "normal", d.invertRight ? "INVERTED" : "normal");
  if (!failed && !calGyroSign(d)) failed = "gyro sign";
  else if (!failed) DBG_PRINTF("gyro    : sign %+.0f\n", d.gyroSign);
  if (!failed && !backToRange(fStart)) failed = "return to start";
  if (!failed && !findWall()) failed = "square up on the wall";
  if (!failed && !calStraight(true, d, travel)) failed = "forward";
  if (!failed && !calTurn(-1.0f, d, cwL, cwR, cwRad)) failed = "right turn";
  if (!failed && !calTurn(1.0f, d, ccwL, ccwR, ccwRad)) failed = "left turn";
  if (!failed && !calStraight(false, d, travel)) failed = "back";
  if (!failed) {
    const float trackCw = (cwL * d.mmPerEdge[0][0] + cwR * d.mmPerEdge[1][1]) / cwRad;
    const float trackCcw = (ccwL * d.mmPerEdge[0][1] + ccwR * d.mmPerEdge[1][0]) / ccwRad;
    d.trackMm = 0.5f * (trackCw + trackCcw);
    DBG_PRINTF("track   : right %.1f mm, left %.1f mm -> %.1f mm\n", trackCw, trackCcw, d.trackMm);
  }
  if (!failed && !calModel()) failed = "motor model";

  sonar::setFrontOnly(false);
  motors::disable();
  motion::reportFault();
  if (!failed && calib::save(d)) {
    DBG_PRINTF("saved: wiring L%d R%d, gyro %+.0f, mm/edge L %.4f/%.4f R %.4f/%.4f, right %.4f, left %.4f, track %.1f\n",
               d.invertLeft, d.invertRight, d.gyroSign, d.mmPerEdge[0][0], d.mmPerEdge[0][1], d.mmPerEdge[1][0],
               d.mmPerEdge[1][1], d.cw, d.ccw, d.trackMm);
    ui::soundCalSaved();
  } else {
    DBG_PRINTF("calibration not saved: %s failed or a value is out of range.\n", failed ? failed : "a check");
    calib::begin();
    ui::soundError();
  }
}

}

namespace modes {

void mode8AutoCalibration() { modeAutoCalibrateAll(); }

}
