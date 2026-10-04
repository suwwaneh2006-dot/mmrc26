#include "config.h"

#include "battery.h"
#include "calib.h"
#include "encoders.h"
#include "estimator.h"
#include "imu.h"
#include "modes.h"
#include "motion.h"
#include "motors.h"
#include "sched.h"
#include "sonar.h"
#include "ui.h"

void setup() {

  motors::begin();
  ui::begin();
  DBG_BEGIN(115200);
  DBG_PRINTF("\nMMRC26 firmware\n");

  battery::begin();
  modes::bootWipeCheck();

  ui::led(ui::Led::ON);
  ui::update();
  const bool imuOk = imu::begin();

  sonar::begin();
  encoders::begin();
  calib::begin();
  estimator::begin();
  sched::begin();
  motion::begin();
  modes::bootSelfCheck(imuOk);
}

void loop() {
  modes::menu();
}
