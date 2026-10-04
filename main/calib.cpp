#include "calib.h"

#include <Preferences.h>

#include "encoders.h"
#include "imu.h"
#include "motors.h"

namespace {

constexpr uint32_t CALIB_MAGIC = 0x43414C32;

calib::Data g_data;
bool g_calibrated = false;

bool between(float v, float lo, float hi) { return v >= lo && v <= hi; }

bool valid(const calib::Data& d) {
  if (d.magic != CALIB_MAGIC || (d.gyroSign != 1.0f && d.gyroSign != -1.0f)) return false;
  for (int w = 0; w < 2; ++w) {
    for (int k = 0; k < 2; ++k) {
      if (!between(d.mmPerEdge[w][k], ENC_MM_PER_EDGE * CAL_EDGE_MIN_FACTOR, ENC_MM_PER_EDGE * CAL_EDGE_MAX_FACTOR)) {
        return false;
      }
    }
  }
  return between(d.cw, CAL8_SCALE_MIN, CAL8_SCALE_MAX) && between(d.ccw, CAL8_SCALE_MIN, CAL8_SCALE_MAX) &&
         between(d.trackMm, CAL_TRACK_MIN_MM, CAL_TRACK_MAX_MM);
}

void apply(const calib::Data& d) {
  motors::setInvert(d.invertLeft != 0, d.invertRight != 0);
  imu::setSign(d.gyroSign);
  imu::setDirectionScale(d.cw, d.ccw);
  encoders::setMmPerEdge(0, d.mmPerEdge[0][0], d.mmPerEdge[0][1]);
  encoders::setMmPerEdge(1, d.mmPerEdge[1][0], d.mmPerEdge[1][1]);
}

}

namespace calib {

Data defaults() {
  Data d;
  d.magic = CALIB_MAGIC;
  d.invertLeft = 0;
  d.invertRight = 0;
  d.gyroSign = 1.0f;
  for (int w = 0; w < 2; ++w) {
    for (int k = 0; k < 2; ++k) d.mmPerEdge[w][k] = ENC_MM_PER_EDGE;
  }
  d.cw = 1.0f;
  d.ccw = 1.0f;
  d.trackMm = WHEEL_TRACK_MM;
  return d;
}

void begin() {
  g_data = defaults();
  g_calibrated = false;
  Data r{};
  Preferences prefs;
  if (prefs.begin(NVS_DIRCAL_NAMESPACE, true)) {
    g_calibrated = prefs.getBytes("all", &r, sizeof(r)) == sizeof(r) && valid(r);
    prefs.end();
  }
  if (g_calibrated) g_data = r;
  apply(g_data);
}

const Data& get() { return g_data; }
bool isCalibrated() { return g_calibrated; }
float trackMm() { return g_data.trackMm; }

void applyWithoutSaving(const Data& d) {
  g_data = d;
  apply(d);
}

bool save(const Data& d) {
  Data c = d;
  c.magic = CALIB_MAGIC;
  if (!valid(c)) return false;
  Preferences prefs;
  if (!prefs.begin(NVS_DIRCAL_NAMESPACE, false)) return false;
  const bool ok = prefs.putBytes("all", &c, sizeof(c)) == sizeof(c);
  prefs.end();
  if (ok) {
    g_calibrated = true;
    applyWithoutSaving(c);
  }
  return ok;
}

}
