#pragma once

#include <Arduino.h>
#include "config.h"

namespace calib {

struct Data {
  uint32_t magic;
  uint8_t  invertLeft;
  uint8_t  invertRight;
  float    gyroSign;
  float    mmPerEdge[2][2];
  float    cw;
  float    ccw;
  float    trackMm;
};

void begin();
const Data& get();
bool isCalibrated();
bool save(const Data& d);
Data defaults();
float trackMm();
void applyWithoutSaving(const Data& d);

}
