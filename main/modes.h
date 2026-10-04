#pragma once

#include <Arduino.h>
#include "config.h"

namespace modes {

void bootWipeCheck();

void bootSelfCheck(bool imuOk);

void menu();

void mode1Sensors();
void mode2Motors();
void mode3Straight();
void mode4Pivot();
void mode5MotorModel();
void mode6Step();
void mode7Match();
void mode8AutoCalibration();

}
