#pragma once

#include "maze.h"

struct MouseOptions {
  int  loops = 3;
  bool mirror = false;
  bool useGoal = false;
  mm::CellSet goal;
  bool visualize = true;
  bool logSummary = true;
};

struct MouseReport {
  bool  searchOk = false;
  bool  allOk = false;
  int   runsDone = 0;
  int   returnsDone = 0;
  int   searchMoves = 0;
  int   totalMoves = 0;
  int   exploringReturns = 0;
  float speedPathS = 0.0f;
  int   conflicts = 0;
  int   postViolations = 0;
  bool  explorePending = false;
  const char* failure = "";
};

MouseReport runMouse(const MouseOptions& options);

const mm::Maze& mouseMaze();

mm::TimeModel mouseTimeModel(int tier);
