// =============================================================================
//  sim/mouse.cpp - see mouse.h
// =============================================================================
#include "mouse.h"

#include <iostream>
#include <sstream>
#include <string>

#include "API.h"
#include "config.h"

namespace {

// Big objects live in static storage (same as on the robot).
mm::Maze     g_maze;
mm::Router   g_router(g_maze);
mm::Explorer g_explorer(g_maze, g_router);

void log(const std::string& s) { std::cerr << s << std::endl; }

mm::TimeModel tierModel(int tier) {
  const SpeedTier& t = TIERS[tier];
  return mm::TimeModel::fromProfile(CELL_PITCH_MM, t.speed_mm_s, t.accel_mm_s2, t.turnRate_dps,
                                    TURN_ACCEL_DPS2, PLAN_TURN_SETTLE_S, PLAN_SEGMENT_S);
}

// mms draws in its own (canonical) frame, which is ours when not mirrored.
void drawCell(mm::Cell c, const MouseOptions& o) {
  if (!o.visualize || o.mirror) return;
  for (int d = 0; d < 4; ++d) {
    const mm::Dir dir = static_cast<mm::Dir>(d);
    if (g_maze.hasWall(c, dir)) API::setWall(c.x, c.y, mm::dirChar(dir));
  }
}

void turnAround() {
  API::turnRight();
  API::turnRight();
  g_explorer.setPose(g_explorer.pos(), mm::turnBack(g_explorer.heading()));
}

// Cell-by-cell run (search, or exploring return). Returns true on arrival.
bool exploreRun(mm::Explorer::Target target, bool exploreForSpeed, const MouseOptions& o,
                int& moves, MouseReport& rep) {
  g_explorer.startRun(target, exploreForSpeed, tierModel(0), PLAN_EXPLORE_GAIN);
  bool explored = false;
  while (true) {
    const mm::Cell here = g_explorer.pos();
    const bool l = API::wallLeft(), f = API::wallFront(), r = API::wallRight();
    const mm::Action a = g_explorer.step(l, f, r);
    drawCell(here, o);
    explored = explored || g_explorer.exploring();
    switch (a) {
      case mm::Action::ARRIVED:
        if (explored) ++rep.exploringReturns;
        return true;
      case mm::Action::NO_PATH:
        return false;
      case mm::Action::LEFT:
        API::turnLeft();
        break;
      case mm::Action::RIGHT:
        API::turnRight();
        break;
      case mm::Action::BACK:
        API::turnRight();
        API::turnRight();
        break;
      case mm::Action::FORWARD:
        break;
    }
    API::moveForward();
    ++moves;
  }
}

// Speed run on verified walls only. Returns false if no verified path.
bool speedRun(mm::Explorer::Target target, const MouseOptions& o, int& moves, MouseReport& rep) {
  static mm::Path path;
  if (!g_explorer.planSpeedRun(target, tierModel(1), false, path)) return false;
  if (target == mm::Explorer::TO_GOAL) rep.speedPathS = path.timeS;
  mm::Cell c = g_explorer.pos();
  mm::Dir h = g_explorer.heading();
  for (int i = 0; i < path.count; ++i) {
    const mm::PathStep& s = path.steps[i];
    if (s.turn == 1) {
      API::turnLeft();
      h = mm::turnLeft(h);
    } else if (s.turn == -1) {
      API::turnRight();
      h = mm::turnRight(h);
    } else if (s.turn == 2) {
      API::turnRight();
      API::turnRight();
      h = mm::turnBack(h);
    }
    for (int k = 0; k < s.cells; ++k) {
      c = mm::neighbour(c, h);
      if (o.visualize && !o.mirror && !g_maze.isGoal(c)) API::setColor(c.x, c.y, 'B');
    }
    API::moveForward(s.cells);
    moves += s.cells;
  }
  g_explorer.setPose(path.end, path.endHeading);
  return true;
}

// Return to start: explore if the optimistic route is worth checking,
// otherwise drive the verified fastest path home.
bool returnRun(const MouseOptions& o, int& moves, MouseReport& rep) {
  const bool explore = !EXPLORE_WALL_HUG && g_explorer.explorationWorthwhile(tierModel(1), PLAN_EXPLORE_GAIN, nullptr);
  if (!explore && speedRun(mm::Explorer::TO_START, o, moves, rep)) return true;
  return exploreRun(mm::Explorer::TO_START, true, o, moves, rep);
}

}  // namespace

const mm::Maze& mouseMaze() { return g_maze; }

mm::TimeModel mouseTimeModel(int tier) { return tierModel(tier); }

MouseReport runMouse(const MouseOptions& o) {
  MouseReport rep;
  g_maze.reset(API::mazeWidth(), API::mazeHeight());
  if (o.useGoal) g_maze.setGoal(o.goal);
  g_explorer.setMirror(o.mirror);
  g_explorer.setMethod(EXPLORE_WALL_HUG ? mm::Explorer::WALL_HUG : mm::Explorer::FLOOD_FILL, WALL_HUG_LEFT_HAND);
  const mm::Cell start = {0, 0};
  g_explorer.setPose(start, mm::NORTH);
  if (o.visualize && !o.mirror) {
    API::clearAllColor();
    API::clearAllText();
    for (int x = 0; x < g_maze.width(); ++x) {
      for (int y = 0; y < g_maze.height(); ++y) {
        const mm::Cell c = {static_cast<int8_t>(x), static_cast<int8_t>(y)};
        if (g_maze.isGoal(c)) API::setColor(x, y, 'G');
      }
    }
  }

  int moves = 0;
  // 1. Search to the goal.
  rep.searchOk = exploreRun(mm::Explorer::TO_GOAL, false, o, moves, rep);
  rep.searchMoves = moves;
  if (!rep.searchOk) {
    rep.failure = g_explorer.looped() ? "wall follower looped (goal not reachable by wall hugging)"
                                      : "search did not reach the goal";
    return rep;
  }
  ++rep.runsDone;
  turnAround();

  // 2. Return, then speed run / return loops.
  for (int loop = 0; loop <= o.loops; ++loop) {
    if (!returnRun(o, moves, rep)) {
      rep.failure = "return did not reach the start";
      rep.totalMoves = moves;
      return rep;
    }
    ++rep.returnsDone;
    turnAround();
    if (loop == o.loops) break;
    if (!speedRun(mm::Explorer::TO_GOAL, o, moves, rep) &&
        !exploreRun(mm::Explorer::TO_GOAL, false, o, moves, rep)) {
      rep.failure = "run to the goal failed";
      rep.totalMoves = moves;
      return rep;
    }
    ++rep.runsDone;
    turnAround();
  }
  rep.totalMoves = moves;
  rep.conflicts = g_maze.conflicts();
  rep.postViolations = g_maze.postViolations();
  rep.explorePending = g_explorer.explorationWorthwhile(tierModel(1), PLAN_EXPLORE_GAIN, nullptr);
  rep.allOk = true;
  if (o.logSummary) {
    std::ostringstream msg;
    msg << "runs " << rep.runsDone << ", returns " << rep.returnsDone << " (" << rep.exploringReturns
        << " exploring), search cells " << rep.searchMoves << ", total cells " << moves
        << ", verified speed path " << rep.speedPathS << " s";
    log(msg.str());
  }
  return rep;
}
