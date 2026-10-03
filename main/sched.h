// =============================================================================
//  sched.h - the fixed 1 kHz control tick and cooperative waiting.
//
//  Single core, no RTOS tasks of our own (simpler to audit, no data races):
//  the main loop calls sched::service() continuously. service() runs the
//  control tick whenever it is due and then gives the non-blocking sonar
//  scheduler and the UI a turn. Higher-level code (test modes, strategy) is
//  written as straight-line procedures that wait with sched::waitMs() /
//  sched::service(), so the tick keeps running while they "block".
//
//  Control tick order: IMU -> battery -> encoders -> controller -> motors -> safety.
//  The tick measures its own lateness; a frozen loop is caught by the
//  independent motor watchdog (motors.cpp).
// =============================================================================
#pragma once

#include <Arduino.h>
#include "config.h"

namespace sched {

using Controller = void (*)(float dt_s);

void begin();
// The motion law run inside every tick (after sensors, before motor output).
void setController(Controller fn);
// Run everything that is due. Call as often as possible.
void service();
// Cooperative delay: keeps servicing the tick, sonar and UI.
void waitMs(uint32_t ms);

// Run fn() (which may stall the CPU, e.g. an NVS flash write) without
// tripping the overrun fault or the motor watchdog, as long as it finishes
// within BLOCKING_SECTION_MAX_MS. The motors hold their last output meanwhile.
void blockingSection(void (*fn)());

void resetStats();          // clear overrun statistics (after Serial dumps)
uint32_t overruns();        // ticks that started more than CONTROL_LATE_US late
uint32_t maxLateUs();       // worst lateness seen
uint32_t tickCount();

}  // namespace sched
