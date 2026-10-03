// =============================================================================
//  sim/fake_api.h - an in-process stand-in for the mms simulator so the mouse
//  program (mouse.cpp) can be tested on hundreds of mazes without the GUI.
//  It implements the exact API.h interface; moving through a wall throws.
// =============================================================================
#pragma once

#include <stdexcept>
#include <string>

#include "truemaze.h"

struct Crash : std::runtime_error {
  explicit Crash(const std::string& s) : std::runtime_error(s) {}
};

namespace fake {
// Install the world. startX is 0 (normal) or width-1 (mirrored maze).
void setWorld(const TrueMaze& maze, int startX);
long moves();
long turns();
}  // namespace fake
