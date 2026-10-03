#pragma once

#include <stdexcept>
#include <string>

#include "truemaze.h"

struct Crash : std::runtime_error {
  explicit Crash(const std::string& s) : std::runtime_error(s) {}
};

namespace fake {

void setWorld(const TrueMaze& maze, int startX);
long moves();
long turns();
}
