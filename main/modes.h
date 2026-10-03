// =============================================================================
//  modes.h - boot sequence, mode menu and the test / calibration modes.
//
//  Boot: hold the button while powering on for BOOT_WIPE_HOLD_MS to wipe the
//  stored map. Then the self-check beeps the number of working parts
//  (3 sonars + IMU = 4). If anything is missing its error code is beeped
//  (N long low beeps, repeated).
//
//  Menu: click the button N times (clicks closer than CLICK_GAP_MS). The robot
//  answers with N beeps. Modes that move then wait for one more short press
//  (start) or a long press (cancel), then give MODE_START_DELAY_MS to take
//  your hand away. A short press during any test aborts it.
//
//    1  Sensor dump (Serial 115200; works even if the self-check failed)
//    2  Motor direction test (wheels in the air)
//    3  Straight 5 cells at T1
//    4  Pivot 4x90 deg left, report the error
//    5  Auto-calibration of the volts -> velocity model (face a wall ~3 cells away)
//    6  One-cell step test (logs the response + estimator as CSV)
//    7  Match mode (strategy.h): mirror selection, then hand-wave start
// =============================================================================
#pragma once

#include <Arduino.h>
#include "config.h"

namespace modes {

// Before the IMU calibrates: if the button is held at power-on for
// BOOT_WIPE_HOLD_MS, wipe the stored maze map.
void bootWipeCheck();
// After all drivers started: sonar self-test, beep the working-part count,
// show an error code if something is missing. imuOk = result of imu::begin().
void bootSelfCheck(bool imuOk);
// One pass of the menu: wait for a mode number, run it, return.
void menu();

}  // namespace modes
