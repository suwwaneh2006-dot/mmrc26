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

void modePivot4() {
  DBG_PRINTF("\n== MODE 4: pivot 4x90 deg left at T1 ==\n");
  const float f0 = motion::settledFrontMm();
  if (!motion::arm()) return;
  const float start = motion::headingTargetDeg();
  motion::Result res = motion::Result::DONE;
  for (int i = 1; i <= 4; ++i) {
    const uint32_t t0 = millis();
    motion::pivot(90.0f, TIERS[0]);
    res = motion::waitDone();
    const float err = imu::headingDeg() - motion::headingTargetDeg();
    DBG_PRINTF("turn %d: %s, %lu ms, gyro error %+.2f deg\n", i, motion::resultName(res),
               static_cast<unsigned long>(millis() - t0), err);
    if (res != motion::Result::DONE) break;
    sched::waitMs(300);
  }
  const float total = imu::headingDeg() - (start + 360.0f);
  motors::disable();
  const float f1 = motion::settledFrontMm();
  DBG_PRINTF("total gyro error after 360 deg: %+.2f deg\n", total);
  if (f0 > 0.0f && f1 > 0.0f) DBG_PRINTF("front range before %.0f / after %.0f mm\n", f0, f1);
  DBG_PRINTF("Now look at the robot: if it over-rotated by X deg in total (under = negative X),\n"
             "set IMU_GYRO_SCALE = %.4f * 360 / (360 + X).\n", IMU_GYRO_SCALE);
  motion::reportFault();
  if (res == motion::Result::DONE) ui::soundOk();
  else ui::soundAbort();
}

}

namespace modes {

void mode4Pivot() { modePivot4(); }

}
