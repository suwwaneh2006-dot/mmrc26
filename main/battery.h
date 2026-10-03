// =============================================================================
//  battery.h - 2S LiPo voltage monitor (GPIO 34, 20k/10k divider).
//
//  One ADC sample every BATT_SAMPLE_PERIOD_MS into a 16-sample moving average,
//  so the control tick never waits for 16 conversions in a row.
//  Vbatt = analogReadMilliVolts * 3 * BATT_CAL_FACTOR.
// =============================================================================
#pragma once

#include <Arduino.h>
#include "config.h"

namespace battery {

void  begin();          // fills the averaging window
void  update();         // call every control tick
float volts();          // averaged pack voltage

inline bool warnLow()      { return volts() < BATT_WARN_V; }
inline bool fastAllowed()  { return volts() >= BATT_BLOCK_FAST_V; }
inline bool armAllowed()   { return volts() >= BATT_REFUSE_ARM_V; }
inline bool belowCutoff()  { return volts() < BATT_CUTOFF_LOAD_V; }

}  // namespace battery
