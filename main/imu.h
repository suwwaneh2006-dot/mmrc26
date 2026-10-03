#pragma once

#include <Arduino.h>
#include "config.h"

namespace imu {

bool begin();

void update(float dt_s, bool motorsIdle);

bool  ok();
float rateDps();
float headingDeg();
void  setHeading(float deg);
float biasDps();
bool  stationary();
uint32_t errorCount();
uint8_t  whoAmI();

}
