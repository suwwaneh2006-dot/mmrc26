// =============================================================================
//  sim/fake_api.cpp - see fake_api.h. Implements API.h against a TrueMaze.
// =============================================================================
#include "fake_api.h"

#include "API.h"

// ---------------------------------------------------------------------------
//  Fake mms
// ---------------------------------------------------------------------------
namespace {
TrueMaze g_world;
int g_x = 0, g_y = 0;
mm::Dir g_dir = mm::NORTH;
long g_moves = 0, g_turns = 0;
}  // namespace

namespace fake {
void setWorld(const TrueMaze& maze, int startX) {
  g_world = maze;
  g_x = startX;
  g_y = 0;
  g_dir = mm::NORTH;
  g_moves = g_turns = 0;
}
long moves() { return g_moves; }
long turns() { return g_turns; }
}  // namespace fake

int API::mazeWidth() { return g_world.width; }
int API::mazeHeight() { return g_world.height; }
bool API::wallFront() { return g_world.wall[g_x][g_y][g_dir]; }
bool API::wallRight() { return g_world.wall[g_x][g_y][mm::turnRight(g_dir)]; }
bool API::wallLeft() { return g_world.wall[g_x][g_y][mm::turnLeft(g_dir)]; }

void API::moveForward(int distance) {
  for (int i = 0; i < distance; ++i) {
    if (g_world.wall[g_x][g_y][g_dir]) throw Crash("crashed into a wall");
    g_x += mm::dx(g_dir);
    g_y += mm::dy(g_dir);
    ++g_moves;
  }
}
void API::turnRight() {
  g_dir = mm::turnRight(g_dir);
  ++g_turns;
}
void API::turnLeft() {
  g_dir = mm::turnLeft(g_dir);
  ++g_turns;
}
void API::setWall(int, int, char) {}
void API::clearWall(int, int, char) {}
void API::setColor(int, int, char) {}
void API::clearColor(int, int) {}
void API::clearAllColor() {}
void API::setText(int, int, const std::string&) {}
void API::clearText(int, int) {}
void API::clearAllText() {}
bool API::wasReset() { return false; }
void API::ackReset() {}
