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

constexpr float   MOTOR_TEST_V = 2.0f;
constexpr uint32_t MOTOR_TEST_PHASE_MS = 1500;

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
  if (!motion::arm()) return;
  uint8_t idx = 0;
  bool aborted = false;
  for (const Phase& p : phases) {
    ++idx;
    ui::beep(idx, 60, 120);
    ui::waitBeeps();
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
    motion::waitDone();

    const float dl = encoders::leftMm() - l0, dr = encoders::rightMm() - r0;
    const bool lOk = p.l == 0.0f ? fabsf(dl) < 5.0f : (dl * p.l > 0.0f && fabsf(dl) > 20.0f);
    const bool rOk = p.r == 0.0f ? fabsf(dr) < 5.0f : (dr * p.r > 0.0f && fabsf(dr) > 20.0f);
    DBG_PRINTF("   encoders: left %+.0f mm %s, right %+.0f mm %s\n", dl, lOk ? "ok" : "CHECK", dr,
               rOk ? "ok" : "CHECK");
    if (aborted || motors::faultCode() != Fault::NONE) break;
    sched::waitMs(300);
  }
  motors::disable();
  motion::reportFault();
  if (aborted) ui::soundAbort();
  else ui::soundOk();
}

}

namespace modes {

void mode2Motors() { modeMotorTest(); }

}
