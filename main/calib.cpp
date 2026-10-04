#include "calib.h"

#include <Preferences.h>

#include "encoders.h"
#include "imu.h"

namespace {

constexpr uint32_t DIRCAL_MAGIC = 0x44495231;

struct Record {
  uint32_t magic;
  calib::Direction d;
};

calib::Direction g_dir = {1.0f, 1.0f, 1.0f, 1.0f};
bool g_calibrated = false;

bool inRange(float v) { return v >= CAL8_SCALE_MIN && v <= CAL8_SCALE_MAX; }

bool valid(const calib::Direction& d) {
  return inRange(d.forward) && inRange(d.backward) && inRange(d.cw) && inRange(d.ccw);
}

void apply() {
  encoders::setDirectionScale(g_dir.forward, g_dir.backward);
  imu::setDirectionScale(g_dir.cw, g_dir.ccw);
}

}

namespace calib {

void begin() {
  Preferences prefs;
  Record r{};
  g_calibrated = false;
  if (prefs.begin(NVS_DIRCAL_NAMESPACE, true)) {
    g_calibrated = prefs.getBytes("dir", &r, sizeof(r)) == sizeof(r) && r.magic == DIRCAL_MAGIC && valid(r.d);
    prefs.end();
  }
  if (g_calibrated) g_dir = r.d;
  apply();
}

const Direction& get() { return g_dir; }
bool isCalibrated() { return g_calibrated; }

bool save(const Direction& d) {
  if (!valid(d)) return false;
  Record r{DIRCAL_MAGIC, d};
  Preferences prefs;
  if (!prefs.begin(NVS_DIRCAL_NAMESPACE, false)) return false;
  const bool ok = prefs.putBytes("dir", &r, sizeof(r)) == sizeof(r);
  prefs.end();
  if (ok) {
    g_dir = d;
    g_calibrated = true;
    apply();
  }
  return ok;
}

}
