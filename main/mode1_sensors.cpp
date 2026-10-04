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

}

namespace modes {

void mode1Sensors() { modeSensorDump(); }

}
