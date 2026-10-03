// =============================================================================
//  MMRC26 micromouse firmware - main.ino
//
//  Board : WEMOS LOLIN32 Lite (ESP32), Arduino "esp32" core 3.x
//  Parts : TB6612FNG + 2x N20 with hall encoders, 3x HC-SR04, MPU6050, 2S LiPo
//
//  COMPETITION DECLARATION
//   * This code contains NO maze data. The maze is discovered at run time by
//     the robot's own sensors only; nothing is hard-coded or fed in.
//   * This code contains NO wireless code. WiFi, Bluetooth, BLE and ESP-NOW
//     are never included, initialised or used.
//
//  Structure (one module per file):
//    config.h   all pins, constants, thresholds (units in the names)
//    motors.*   TB6612 driver in volts, fault latch, watchdog, velocity model
//    sonar.*    non-blocking interrupt-driven HC-SR04 scheduler + filters
//    imu.*      MPU6050 yaw rate, heading, automatic bias re-estimation
//    battery.*  pack voltage monitor
//    encoders.* wheel odometry (N20 hall, one channel per wheel)
//    ui.*       button events, LED patterns, buzzer patterns
//    sched.*    fixed 1 kHz control tick + cooperative waiting
//    estimator.* localisation without encoders (model + front wall + posts)
//    motion.*   straights, pivots, alignment, collision guard, stuck check
//    maze.*     maze brain, pure C++ (verified in the mms simulator, /sim)
//    strategy.* match state machine (mode 7)
//    modes.*    boot sequence, mode menu, test and calibration modes
//
//  See modes.h for the button menu and BRINGUP.md for the test order.
// =============================================================================
#include "config.h"

#include "battery.h"
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
  // Safety first: H-bridge disabled (STBY LOW) before anything else.
  motors::begin();
  ui::begin();
  DBG_BEGIN(115200);
  DBG_PRINTF("\nMMRC26 firmware\n");

  battery::begin();
  modes::bootWipeCheck();      // hold the button at power-on to wipe the map

  ui::led(ui::Led::ON);        // "keep still": gyro bias is being measured
  ui::update();
  const bool imuOk = imu::begin();

  sonar::begin();
  encoders::begin();
  estimator::begin();
  sched::begin();
  motion::begin();
  modes::bootSelfCheck(imuOk);
}

void loop() {
  modes::menu();
}
