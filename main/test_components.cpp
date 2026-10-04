#include "modes.h"

#include "battery.h"
#include "encoders.h"
#include "imu.h"
#include "motion.h"
#include "motors.h"
#include "sched.h"
#include "sonar.h"
#include "ui.h"

namespace {

constexpr uint32_t PRINT_MS = 200;
constexpr float    MOTOR_TEST_VOLTS = 2.0f;
constexpr uint32_t MOTOR_RUN_MS = 2000;
constexpr uint32_t MOTOR_PAUSE_MS = 1000;
constexpr int      MOTOR_CYCLES = 3;
constexpr int      COUNTDOWN_S = 5;

const char* const NAMES[] = {"menu",        "front sonar",  "left sonar",    "right sonar", "gyro",
                             "battery",     "left encoder", "right encoder", "left motor",  "right motor"};

bool stopRequested() { return ui::event() == ui::Button::SHORT || motors::faultCode() == Fault::USER_ABORT; }

void printSonar(sonar::Id id) {
  const sonar::Reading r = sonar::read(id);
  const sonar::Stats s = sonar::stats(id);
  if (!r.valid) {
    DBG_PRINTF("%-5s: no reading yet  (pings %lu, echoes %lu)\n", sonar::name(id), static_cast<unsigned long>(s.pings),
               static_cast<unsigned long>(s.echoes));
  } else if (r.inRange) {
    DBG_PRINTF("%-5s: %6.0f mm  raw %6.0f  wall %s  %s\n", sonar::name(id), r.mm, r.rawMm, r.wall ? "YES" : "no",
               s.faulty ? "FAULT" : "ok");
  } else {
    DBG_PRINTF("%-5s: nothing in range  %s\n", sonar::name(id), s.faulty ? "FAULT" : "ok");
  }
}

void printOnce(uint8_t test) {
  switch (test) {
    case 1: printSonar(sonar::FRONT); break;
    case 2: printSonar(sonar::LEFT); break;
    case 3: printSonar(sonar::RIGHT); break;
    case 4:
      DBG_PRINTF("gyro: rate %7.2f deg/s  heading %8.2f deg  %s\n", imu::rateDps(), imu::headingDeg(),
                 imu::ok() ? "ok" : "FAULT");
      break;
    case 5: DBG_PRINTF("battery: %.2f V\n", battery::volts()); break;
    case 6:
      DBG_PRINTF("left encoder: %lu edges  %.1f mm\n", static_cast<unsigned long>(encoders::leftEdges()),
                 encoders::leftMm());
      break;
    case 7:
      DBG_PRINTF("right encoder: %lu edges  %.1f mm\n", static_cast<unsigned long>(encoders::rightEdges()),
                 encoders::rightMm());
      break;
    default: break;
  }
}

void sensorTest(uint8_t test) {
  DBG_PRINTF("running until you type s\n");
  ui::clearEvents();
  uint32_t last = 0;
  while (!stopRequested()) {
    sched::service();
    if (millis() - last >= PRINT_MS) {
      last = millis();
      printOnce(test);
    }
  }
}

bool runMotor(bool left, float volts) {
  const uint32_t e0 = left ? encoders::leftEdges() : encoders::rightEdges();
  motion::rawVolts(left ? volts : 0.0f, left ? 0.0f : volts);
  const uint32_t t0 = millis();
  uint32_t last = 0;
  while (millis() - t0 < MOTOR_RUN_MS) {
    sched::service();
    if (stopRequested()) return false;
    if (millis() - last >= PRINT_MS) {
      last = millis();
      DBG_PRINTF("  %s %+.1f V  edges %lu\n", left ? "left" : "right", volts,
                 static_cast<unsigned long>((left ? encoders::leftEdges() : encoders::rightEdges()) - e0));
    }
  }
  motion::stop();
  motion::waitDone();
  sched::waitMs(MOTOR_PAUSE_MS);
  return true;
}

void motorTest(bool left) {
  DBG_PRINTF("WHEELS MUST BE OFF THE TABLE. Type s to stop.\n");
  for (int i = COUNTDOWN_S; i > 0; --i) {
    DBG_PRINTF("starting in %d\n", i);
    sched::waitMs(1000);
    if (stopRequested()) return;
  }
  if (!battery::armAllowed()) {
    DBG_PRINTF("battery %.2f V too low - refused\n", battery::volts());
    return;
  }
  if (!motion::arm()) {
    DBG_PRINTF("motors refused to enable (%s)\n", faultName(motors::faultCode()));
    return;
  }
  ui::setModeActive(true);
  for (int c = 1; c <= MOTOR_CYCLES; ++c) {
    DBG_PRINTF("cycle %d: forward\n", c);
    if (!runMotor(left, MOTOR_TEST_VOLTS)) break;
    DBG_PRINTF("cycle %d: backward\n", c);
    if (!runMotor(left, -MOTOR_TEST_VOLTS)) break;
  }
  ui::setModeActive(false);
  motors::disable();
  DBG_PRINTF("%s motor test finished\n", left ? "left" : "right");
}

}

namespace modes {

void autoTest() {
  const uint8_t test = AUTO_TEST;
  if (test == 0 || test > 9) return;
  DBG_PRINTF("\n== AUTO TEST %u: %s ==\n", test, NAMES[test]);
  if (test <= 7) sensorTest(test);
  else motorTest(test == 8);
  motors::disable();
  motors::clearFault();
  DBG_PRINTF("== auto test ended, menu follows ==\n");
}

}
