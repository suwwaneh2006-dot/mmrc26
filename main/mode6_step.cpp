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

constexpr uint32_t LOG_PERIOD_MS = 5;
constexpr uint32_t LOG_TAIL_MS = 400;

struct LogRow {
  uint16_t tMs;
  float sRef, vRef, sEst, vEst, sigma, leftV, rightV, heading, frontMm, leftMm, rightMm;
};
constexpr int LOG_ROWS = 500;
LogRow g_log[LOG_ROWS];

void modeStepTest() {
  DBG_PRINTF("\n== MODE 6: one-cell step test at T1 ==\n");
  const float f0 = motion::settledFrontMm();
  if (!motion::arm()) return;
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
  const float f1 = motion::settledFrontMm();

  DBG_PRINTF("result: %s\n", motion::resultName(res));
  DBG_PRINTF("t_ms,s_target,v_ref,s_est,v_est,sigma,left_v,right_v,heading,front_mm,left_raw,right_raw\n");
  for (int i = 0; i < rows; ++i) {
    const LogRow& r = g_log[i];
    DBG_PRINTF("%u,%.1f,%.0f,%.1f,%.0f,%.1f,%.2f,%.2f,%.2f,%.0f,%.0f,%.0f\n", r.tMs, r.sRef, r.vRef, r.sEst, r.vEst,
               r.sigma, r.leftV, r.rightV, r.heading, r.frontMm, r.leftMm, r.rightMm);
  }
  if (f0 > 0.0f && f1 > 0.0f) DBG_PRINTF("front sonar moved %.0f mm (target %.0f)\n", f0 - f1, CELL_PITCH_MM);
  motion::reportFault();
  if (res == motion::Result::DONE) ui::soundOk();
  else ui::soundAbort();
}

}

namespace modes {

void mode6Step() { modeStepTest(); }

}
