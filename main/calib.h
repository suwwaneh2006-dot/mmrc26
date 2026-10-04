#pragma once

#include <Arduino.h>
#include "config.h"

namespace calib {

struct Direction {
  float forward;
  float backward;
  float cw;
  float ccw;
};

void begin();
const Direction& get();
bool isCalibrated();
bool save(const Direction& d);

}
