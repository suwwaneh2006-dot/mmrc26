// =============================================================================
//  battery.cpp - see battery.h
// =============================================================================
#include "battery.h"

namespace {

float    g_samples[BATT_SAMPLES];
uint8_t  g_next = 0;
float    g_sum = 0.0f;
uint32_t g_lastSampleMs = 0;

float sampleVolts() {
  return analogReadMilliVolts(PIN_BATT_SENSE) * 0.001f * BATT_DIVIDER_RATIO * BATT_CAL_FACTOR;
}

}  // namespace

namespace battery {

void begin() {
  analogSetPinAttenuation(PIN_BATT_SENSE, ADC_11db);   // 0-3.1 V input range
  g_sum = 0.0f;
  for (float& s : g_samples) {
    s = sampleVolts();
    g_sum += s;
  }
  g_next = 0;
  g_lastSampleMs = millis();
}

void update() {
  const uint32_t now = millis();
  if (now - g_lastSampleMs < BATT_SAMPLE_PERIOD_MS) return;
  g_lastSampleMs = now;
  const float v = sampleVolts();
  g_sum += v - g_samples[g_next];
  g_samples[g_next] = v;
  g_next = (g_next + 1) % BATT_SAMPLES;
  if (g_next == 0) {   // re-sum once per window so rounding error cannot creep
    g_sum = 0.0f;
    for (float s : g_samples) g_sum += s;
  }
}

float volts() { return g_sum / BATT_SAMPLES; }

}  // namespace battery
