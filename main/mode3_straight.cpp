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

void modeStraight5() {
  const float dist = 5.0f * CELL_PITCH_MM;
  DBG_PRINTF("\n== MODE 3: straight %.0f mm (5 cells) at T1 ==\n", dist);
  if (!motors::modelIsCalibrated()) DBG_PRINTF("NOTE: velocity model not calibrated (run mode 5 first).\n");
  const float f0 = motion::settledFrontMm();
  if (!motion::arm()) return;
  const float h0 = motion::headingTargetDeg();
  const float e0 = 0.5f * (encoders::leftMm() + encoders::rightMm());
  motion::straight(dist, TIERS[0]);
  const motion::Result res = motion::waitDone();
  const motion::Telemetry t = motion::telemetry();
  motors::disable();
  const float f1 = motion::settledFrontMm();

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
  motion::reportFault();
  if (res == motion::Result::DONE) ui::soundOk();
  else ui::soundAbort();
}

}

namespace modes {

void mode3Straight() { modeStraight5(); }

}
