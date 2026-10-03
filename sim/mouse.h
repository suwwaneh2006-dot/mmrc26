// =============================================================================
//  sim/mouse.h - the mouse program for the mms simulator.
//
//  It drives the firmware's maze brain (main/maze.*, compiled unchanged)
//  through the mms API with the same run sequence as the robot's strategy:
//    search to goal -> about-turn -> return (explores if worthwhile)
//    -> about-turn -> speed run -> about-turn -> return -> ...
//  Used by mms_main.cpp (real mms) and by test_runner.cpp (batch tests with
//  a fake mms API).
// =============================================================================
#pragma once

#include "maze.h"

struct MouseOptions {
  int  loops = 3;               // run+return pairs after the search
  bool mirror = false;          // maze extends to the left of the start
  bool useGoal = false;         // true: use 'goal' below, false: centre goal
  mm::CellSet goal;
  bool visualize = true;        // draw walls / colours in mms
  bool logSummary = true;       // one summary line on stderr (mms log panel)
};

struct MouseReport {
  bool  searchOk = false;       // reached the goal on the search run
  bool  allOk = false;          // every run and return completed
  int   runsDone = 0;           // successful runs to the goal
  int   returnsDone = 0;        // successful returns to the start
  int   searchMoves = 0;        // cells driven on the search run
  int   totalMoves = 0;         // cells driven in total
  int   exploringReturns = 0;   // returns that explored for a faster path
  float speedPathS = 0.0f;      // planned time of the last verified speed path
  int   conflicts = 0;          // wall contradictions (0 with perfect sensors)
  int   postViolations = 0;     // post-rule violations in the learned map
  bool  explorePending = false; // at the end, exploring would still pay off
  const char* failure = "";     // reason if allOk is false
};

// Runs the full sequence. Throws whatever the API throws on a crash.
MouseReport runMouse(const MouseOptions& options);

// The map learned by the last runMouse() (for the batch tester).
const mm::Maze& mouseMaze();
// Planning time model of a speed tier (0 = T1 ... 3 = T4), from config.h.
mm::TimeModel mouseTimeModel(int tier);
