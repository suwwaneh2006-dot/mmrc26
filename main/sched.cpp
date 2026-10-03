#include "sched.h"

#include "battery.h"
#include "encoders.h"
#include "imu.h"
#include "motors.h"
#include "sonar.h"
#include "ui.h"

namespace {

sched::Controller g_controller = nullptr;
uint32_t g_nextTickUs = 0;
uint32_t g_lastTickUs = 0;
uint32_t g_overruns = 0;
uint32_t g_maxLateUs = 0;
uint32_t g_ticks = 0;
bool     g_afterBlocking = false;

void controlTick(float dt_s) {
  imu::update(dt_s, motors::idle());
  battery::update();
  encoders::update(dt_s);
  if (g_controller != nullptr) g_controller(dt_s);
  motors::update(dt_s);
  motors::feedWatchdog();

  if (motors::enabled()) {
    if (battery::belowCutoff()) motors::fault(Fault::BATTERY);
    if (!imu::ok()) motors::fault(Fault::IMU);
  }
  ++g_ticks;
}

}

namespace sched {

void begin() {
  g_nextTickUs = micros() + CONTROL_TICK_US;
  g_lastTickUs = micros();
}

void setController(Controller fn) { g_controller = fn; }

void service() {
  const uint32_t now = micros();
  if (static_cast<int32_t>(now - g_nextTickUs) >= 0) {
    const uint32_t late = now - g_nextTickUs;
    if (g_afterBlocking) {
      g_nextTickUs = now;
    } else {
      if (late > g_maxLateUs) g_maxLateUs = late;
      if (late > CONTROL_LATE_US) {
        ++g_overruns;
        if (late > CONTROL_OVERRUN_FAULT_US && motors::enabled()) motors::fault(Fault::LOOP_OVERRUN);
        g_nextTickUs = now;
      }
    }

    float dt = (now - g_lastTickUs) * 1e-6f;
    const float maxDt = g_afterBlocking ? BLOCKING_SECTION_MAX_MS * 1e-3f : 5.0f * CONTROL_TICK_S;
    if (dt > maxDt) dt = maxDt;
    g_afterBlocking = false;
    g_lastTickUs = now;
    g_nextTickUs += CONTROL_TICK_US;
    controlTick(dt);
  }
  sonar::update();
  ui::update();
}

void waitMs(uint32_t ms) {
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) service();
}

void blockingSection(void (*fn)()) {
  motors::watchdogGrace(BLOCKING_SECTION_MAX_MS);
  fn();
  sonar::discardInFlight();
  g_afterBlocking = true;
  motors::feedWatchdog();
}

void resetStats() {
  g_overruns = 0;
  g_maxLateUs = 0;
}

uint32_t overruns()  { return g_overruns; }
uint32_t maxLateUs() { return g_maxLateUs; }
uint32_t tickCount() { return g_ticks; }

}
