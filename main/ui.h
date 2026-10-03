#pragma once

#include <Arduino.h>
#include "config.h"

namespace ui {

enum class Button : uint8_t { NONE, SHORT, LONG };
enum class Led : uint8_t { OFF, ON, BLINK_SLOW, BLINK_FAST, CODE };

enum ErrorCode : uint8_t {
  ERR_SONAR_FRONT = 1,
  ERR_SONAR_LEFT  = 2,
  ERR_SONAR_RIGHT = 3,
  ERR_IMU         = 4,
  ERR_BATTERY     = 5,
  ERR_MOTOR_FAULT = 6,
};

void begin();
void update();

Button   event();
void     clearEvents();
bool     buttonDown();
uint32_t heldMs();

void beep(uint8_t count, uint16_t onMs = 80, uint16_t offMs = 150, bool lowTone = false);
bool beepBusy();
void beepStop();

void soundOk();
void soundError();
void soundAbort();
void soundWiped();
void soundCalSaved();
void soundBatteryWarn();

void led(Led mode, uint8_t code = 0);

}
