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

constexpr uint8_t MODE_COUNT = 8;
constexpr uint32_t BATT_WARN_PERIOD_MS = 10000;

uint8_t g_sonarMask = 0;
bool    g_imuOk = false;

bool allPartsOk() {
  for (uint8_t i = 0; i < sonar::COUNT; ++i) {
    if (!(g_sonarMask & (1u << i)) || sonar::faulty(static_cast<sonar::Id>(i))) return false;
  }
  return g_imuOk && imu::ok();
}

uint8_t errorCode() {
  if (!(g_sonarMask & (1u << sonar::FRONT)) || sonar::faulty(sonar::FRONT)) return ui::ERR_SONAR_FRONT;
  if (!(g_sonarMask & (1u << sonar::LEFT))  || sonar::faulty(sonar::LEFT))  return ui::ERR_SONAR_LEFT;
  if (!(g_sonarMask & (1u << sonar::RIGHT)) || sonar::faulty(sonar::RIGHT)) return ui::ERR_SONAR_RIGHT;
  if (!g_imuOk || !imu::ok()) return ui::ERR_IMU;
  if (!battery::armAllowed()) return ui::ERR_BATTERY;
  if (motors::faultCode() != Fault::NONE) return ui::ERR_MOTOR_FAULT;
  return 0;
}

void showIdleLed() {
  const uint8_t code = errorCode();
  if (code != 0) ui::led(ui::Led::CODE, code);
  else ui::led(ui::Led::BLINK_SLOW);
}

uint8_t readClicks(bool& fromSerial) {
  ui::clearEvents();
  uint8_t n = 0;
  uint32_t lastClickMs = 0;
  uint32_t lastWarnMs = millis();
  while (true) {
    sched::service();
    const uint8_t req = ui::takeSerialMode();
    if (req > 0) {
      fromSerial = true;
      return req;
    }
    showIdleLed();
    if (battery::warnLow() && millis() - lastWarnMs > BATT_WARN_PERIOD_MS) {
      lastWarnMs = millis();
      ui::soundBatteryWarn();
    }
    const ui::Button b = ui::event();
    if (b == ui::Button::SHORT) {
      ++n;
      lastClickMs = millis();
    } else if (b == ui::Button::LONG) {
      Preferences prefs;
      if (prefs.begin(NVS_MAP_NAMESPACE, false)) {
        prefs.clear();
        prefs.end();
      }
      DBG_PRINTF("map wiped.\n");
      ui::soundWiped();
      return 0;
    }
    if (n > 0 && !ui::buttonDown() && millis() - lastClickMs > CLICK_GAP_MS) return n;
  }
}

void runMode(uint8_t n, bool fromSerial) {
  ui::beep(n, 80, 150);
  ui::waitBeeps();
  if (n < 1 || n > MODE_COUNT) {
    DBG_PRINTF("mode %u is not available in this build.\n", n);
    ui::soundError();
    return;
  }
  if (n == 1) {
    ui::setModeActive(true);
    modes::mode1Sensors();
    ui::setModeActive(false);
    return;
  }
  if (!battery::armAllowed()) {
    DBG_PRINTF("battery %.2f V < %.2f V - refused.\n", battery::volts(), BATT_REFUSE_ARM_V);
    ui::soundError();
    return;
  }
  if (n >= 3 && !allPartsOk()) {
    DBG_PRINTF("a sensor is missing or faulty (error code %u) - refused.\n", errorCode());
    ui::soundError();
    return;
  }
  if (n == 7) {
    modes::mode7Match();
    return;
  }
  if (!fromSerial) {
    DBG_PRINTF("mode %u: short press = start, long press = cancel\n", n);
    if (!ui::waitStartPress()) {
      ui::soundAbort();
      return;
    }
  }
  ui::led(ui::Led::BLINK_FAST);
  sched::waitMs(MODE_START_DELAY_MS);
  ui::setModeActive(true);
  switch (n) {
    case 2: modes::mode2Motors(); break;
    case 3: modes::mode3Straight(); break;
    case 4: modes::mode4Pivot(); break;
    case 5: modes::mode5MotorModel(); break;
    case 6: modes::mode6Step(); break;
    case 8: modes::mode8AutoCalibration(); break;
    default: break;
  }
  ui::setModeActive(false);
  motors::disable();
}

}

namespace modes {

void bootWipeCheck() {
  ui::update();
  if (digitalRead(PIN_BUTTON) != LOW) return;
  const uint32_t t0 = millis();
  ui::led(ui::Led::BLINK_FAST);
  while (digitalRead(PIN_BUTTON) == LOW) {
    ui::update();
    if (millis() - t0 >= BOOT_WIPE_HOLD_MS) {
      Preferences prefs;
      if (prefs.begin(NVS_MAP_NAMESPACE, false)) {
        prefs.clear();
        prefs.end();
      }
      DBG_PRINTF("map wiped.\n");
      ui::soundWiped();
      while (digitalRead(PIN_BUTTON) == LOW || ui::beepBusy()) ui::update();
      break;
    }
  }
  ui::led(ui::Led::OFF);
  delay(300);
  ui::clearEvents();
}

void bootSelfCheck(bool imuOk) {
  g_imuOk = imuOk;
  g_sonarMask = sonar::selfTest();
  uint8_t working = imuOk ? 1 : 0;
  for (uint8_t i = 0; i < sonar::COUNT; ++i) {
    const bool ok = (g_sonarMask & (1u << i)) != 0;
    if (ok) ++working;
    DBG_PRINTF("sonar %-5s : %s\n", sonar::name(static_cast<sonar::Id>(i)), ok ? "ok" : "MISSING");
  }
  DBG_PRINTF("imu         : %s (WHO_AM_I 0x%02X, bias %.3f dps)\n", imuOk ? "ok" : "MISSING", imu::whoAmI(),
             imu::biasDps());
  DBG_PRINTF("battery     : %.2f V\n", battery::volts());
  DBG_PRINTF("model       : %s\n", motors::modelIsCalibrated() ? "calibrated (NVS)" : "DEFAULT (run mode 5)");
  ui::beep(working, 80, 150);
  ui::waitBeeps();
  if (working < 4) ui::soundError();
  showIdleLed();
  DBG_PRINTF("ready: click 1-8 to select a mode.\n");
}

void menu() {
  bool fromSerial = false;
  const uint8_t n = readClicks(fromSerial);
  if (n > 0) runMode(n, fromSerial);
  DBG_PRINTF("ready: click 1-8 to select a mode.\n");
}

}
