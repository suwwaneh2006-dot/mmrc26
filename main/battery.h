#pragma once

#include <Arduino.h>
#include "config.h"

namespace battery {

void  begin();
void  update();
float volts();

inline bool warnLow()      { return volts() < BATT_WARN_V; }
inline bool fastAllowed()  { return volts() >= BATT_BLOCK_FAST_V; }
inline bool armAllowed()   { return volts() >= BATT_REFUSE_ARM_V; }
inline bool belowCutoff()  { return volts() < BATT_CUTOFF_LOAD_V; }

}
