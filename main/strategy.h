// =============================================================================
//  strategy.h - the match state machine (mode 7).
//
//   IDLE -> ARMED (hand < 80 mm in front for 0.5 s, then removed; beep;
//   1 s delay; STBY HIGH only now) -> SEARCH RUN (T1) -> enter goal, stop,
//   turn 180 -> RETURN (explores if the optimistic route is > 10 % faster,
//   else drives the verified path home) -> stop in the start cell, re-zero
//   gyro bias, turn 180, pause 1 s -> SPEED RUN -> RETURN -> ... until the
//   8-minute match clock runs out.
//
//  Tier policy: loops at T2 until 2 clean loops; then ONE run at T3; if clean
//  ONE at T4; afterwards the fastest tier with 2 clean completions. Any abort
//  at a tier drops one tier for the rest of the match. T3/T4 are blocked
//  below BATT_BLOCK_FAST_V.
//
//  Rescue: after any abort the robot stops (STBY LOW) and waits. The operator
//  places it at the start and short-presses the button: pose = (0,0,N), the
//  map is kept, and the robot waits for the hand-wave again.
//
//  Buttons while waiting at the start: short press = rescue / pose reset,
//  long press = leave match mode. During a run: short press = abort.
// =============================================================================
#pragma once

#include <Arduino.h>
#include "config.h"

namespace strategy {

// Runs the match until the operator leaves with a long press.
void runMatch();

}  // namespace strategy
