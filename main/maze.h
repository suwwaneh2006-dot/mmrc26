// =============================================================================
//  maze.h - the maze brain. PURE C++11: no Arduino headers, no heap, no I/O.
//  The same files compile into the ESP32 firmware and into the PC simulator
//  harness (/sim), so the exact code that runs on the robot is the code that
//  was verified in mms.
//
//  Coordinates: cell (0,0) is the start corner, x grows to the right, y grows
//  away from the start (NORTH). The mouse starts at (0,0) facing NORTH with
//  the outer wall on its left. A "mirrored" maze (extending to the LEFT) is
//  handled by the Explorer, which swaps left/right on the way in and out, so
//  the map is always stored in this canonical frame.
//
//  Three layers:
//   Maze      wall map with known/unknown bits, both sides of every wall kept
//             consistent, conflict detection, the post rule as a check,
//             serialisation for NVS.
//   Router    flood fill (search, unknown = open) and time-optimal Dijkstra
//             (speed runs, unknown = wall) on (cell, heading) states.
//   Explorer  the per-cell decision loop used by search and return runs.
// =============================================================================
#pragma once

#include <stdint.h>

#ifndef MAZE_MAX_SIZE
#define MAZE_MAX_SIZE 16   // firmware: MMRC maze is 10x10; sim: up to 32
#endif

namespace mm {

constexpr int MAX_SIZE  = MAZE_MAX_SIZE;
constexpr int MAX_CELLS = MAX_SIZE * MAX_SIZE;
constexpr uint16_t UNREACHABLE = 0xFFFF;

// ---------------------------------------------------------------------------
//  Directions and cells
// ---------------------------------------------------------------------------
enum Dir : uint8_t { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

inline Dir turnLeft(Dir d)  { return static_cast<Dir>((d + 3) & 3); }
inline Dir turnRight(Dir d) { return static_cast<Dir>((d + 1) & 3); }
inline Dir turnBack(Dir d)  { return static_cast<Dir>((d + 2) & 3); }
inline int dx(Dir d) { return d == EAST ? 1 : (d == WEST ? -1 : 0); }
inline int dy(Dir d) { return d == NORTH ? 1 : (d == SOUTH ? -1 : 0); }
char dirChar(Dir d);   // 'n', 'e', 's', 'w' (mms convention)

struct Cell {
  int8_t x;
  int8_t y;
};
inline bool operator==(Cell a, Cell b) { return a.x == b.x && a.y == b.y; }
inline bool operator!=(Cell a, Cell b) { return !(a == b); }
inline Cell neighbour(Cell c, Dir d) {
  Cell n = {static_cast<int8_t>(c.x + dx(d)), static_cast<int8_t>(c.y + dy(d))};
  return n;
}

// A set of cells (goal area, start cell, exploration targets).
class CellSet {
 public:
  void clear();
  void add(Cell c);
  bool has(Cell c) const;
  int  count() const { return count_; }
 private:
  uint8_t bits_[(MAX_CELLS + 7) / 8] = {};
  int count_ = 0;
};

// What the mouse does next. LEFT/RIGHT/BACK = pivot in place, then drive one
// cell forward. ARRIVED = the current cell is a target: stop here.
enum class Action : uint8_t { FORWARD, LEFT, RIGHT, BACK, ARRIVED, NO_PATH };
const char* actionName(Action a);

enum class WallResult : uint8_t { UNCHANGED, LEARNED, CONFLICT };

// ---------------------------------------------------------------------------
//  Maze: the wall map
// ---------------------------------------------------------------------------
class Maze {
 public:
  // All interior walls unknown, the boundary known and present.
  void reset(int width, int height);

  int  width() const  { return w_; }
  int  height() const { return h_; }
  bool inside(Cell c) const { return c.x >= 0 && c.y >= 0 && c.x < w_ && c.y < h_; }

  bool isKnown(Cell c, Dir d) const;
  bool hasWall(Cell c, Dir d) const;   // known AND present
  // Can the mouse cross this wall? optimistic: unknown counts as open
  // (search). Not optimistic: only walls verified absent are open (speed run).
  bool isOpen(Cell c, Dir d, bool optimistic) const;

  // Record a sensor reading. A reading that contradicts a known wall is a
  // CONFLICT: the wall is reset to unknown (to be sensed again) and counted.
  WallResult setWall(Cell c, Dir d, bool present);
  bool visited(Cell c) const;          // all four walls known
  uint16_t conflicts() const { return conflicts_; }
  // Recovery from an impossible map (goal unreachable even with unknown =
  // open): a false "wall present" is what blocks, so every interior wall
  // known as present becomes unknown again. Known-open walls are kept.
  // Returns the number of walls forgotten.
  int forgetPresentWalls();

  // Goal area. setCentreGoal(): the 2x2 (even size) or 1x1 (odd) centre.
  void setCentreGoal();
  void setGoal(const CellSet& goal) { goal_ = goal; }
  const CellSet& goal() const { return goal_; }
  bool isGoal(Cell c) const { return goal_.has(c); }

  // Consistency check: "every lattice post has at least one wall". Returns
  // the number of interior posts whose four walls are all known absent,
  // ignoring posts inside the goal area (the centre post is wall-less).
  int postViolations() const;

  // Persistence (NVS mirror on the robot). Fixed-size blob.
  static constexpr int SERIAL_BYTES = 8 + MAX_CELLS;
  int  serialize(uint8_t* buf, int capacity) const;   // bytes written, 0 on error
  bool deserialize(const uint8_t* buf, int length);

 private:
  // bits 0-3: wall present (N,E,S,W); bits 4-7: wall known (N,E,S,W)
  uint8_t cell_[MAX_SIZE][MAX_SIZE] = {};
  uint8_t w_ = 0, h_ = 0;
  uint16_t conflicts_ = 0;
  CellSet goal_;

  void setRaw(Cell c, Dir d, bool known, bool present);
};

// ---------------------------------------------------------------------------
//  Time model for speed-run planning. The robot pivots in place, so every
//  straight segment starts and ends at rest: a straight of k cells costs one
//  trapezoidal profile, which keeps the path cost exactly additive.
// ---------------------------------------------------------------------------
struct TimeModel {
  float cellMm;          // cell pitch
  float speedMmS;        // cruise speed of the tier
  float accelMmS2;       // acceleration limit
  float turn90S;         // time of one 90 deg pivot (incl. settle)
  float turn180S;        // time of one 180 deg pivot
  float segmentS;        // fixed overhead per straight (stop/settle)
  float straightS(int cells) const;

  // Build from motion parameters: turns use the same trapezoid on angle
  // (peak rate, angular acceleration) plus a settle time per pivot.
  static TimeModel fromProfile(float cellMm, float speedMmS, float accelMmS2, float turnRateDps,
                               float turnAccelDps2, float turnSettleS, float segmentS);
};

// A planned speed run: each step = optional turn, then a straight.
struct PathStep {
  int8_t  turn;    // 0 none, +1 left 90, -1 right 90, 2 about-turn
  uint8_t cells;   // straight length after the turn (>= 1)
};
constexpr int MAX_PATH_STEPS = MAX_CELLS;
struct Path {
  PathStep steps[MAX_PATH_STEPS];
  int   count = 0;
  float timeS = 0.0f;
  Cell  end = {0, 0};
  Dir   endHeading = NORTH;
  int   cells() const;
};

// ---------------------------------------------------------------------------
//  Router
// ---------------------------------------------------------------------------
class Router {
 public:
  explicit Router(const Maze& maze) : maze_(maze) {}

  // Breadth-first flood (distance in cells) from the target set.
  void flood(const CellSet& targets, bool optimistic);
  uint16_t dist(Cell c) const { return dist_[c.x][c.y]; }

  // Best move from pos toward the flooded targets: strictly downhill,
  // preferring straight on (fewer pivots), then left/right, then back.
  Action nextAction(Cell pos, Dir heading, bool optimistic) const;

  // Time-optimal path from (from, heading) to any target cell. Returns false
  // if no path exists. optimistic = false uses verified-open walls only.
  bool fastest(Cell from, Dir heading, const CellSet& targets, bool optimistic,
               const TimeModel& tm, Path& out);

 private:
  const Maze& maze_;
  uint16_t dist_[MAX_SIZE][MAX_SIZE] = {};
  // Dijkstra work arrays over (cell, heading) states.
  float    cost_[MAX_CELLS * 4] = {};
  uint16_t prev_[MAX_CELLS * 4] = {};
  uint16_t heap_[MAX_CELLS * 4] = {};
  int16_t  heapPos_[MAX_CELLS * 4] = {};
  int      heapSize_ = 0;

  void heapPush(uint16_t s);
  void heapDecrease(uint16_t s);
  uint16_t heapPop();
  void siftUp(int i);
  void siftDown(int i);
};

// ---------------------------------------------------------------------------
//  Explorer: the per-cell loop of search and return runs.
//
//  Usage (robot or simulator):
//    explorer.startRun(Explorer::TO_GOAL, false);
//    loop at the wall-reading point of each cell:
//      Action a = explorer.step(wallLeft, wallFront, wallRight);
//      if a == ARRIVED  -> stop, run finished
//      if a == NO_PATH  -> stop, maze contradicts itself
//      else perform a (pivot if needed, then drive one cell)
// ---------------------------------------------------------------------------
class Explorer {
 public:
  enum Target : uint8_t { TO_GOAL, TO_START };
  // How exploring runs choose their next move.
  //  FLOOD_FILL: shortest route on the map, unknown walls = open.
  //  WALL_HUG:   keep one hand on the wall (left or right hand rule). Cannot
  //              reach a goal that is an island (walls not connected to the
  //              walls around the start); this is detected as a loop.
  enum Method : uint8_t { FLOOD_FILL, WALL_HUG };
  void setMethod(Method m, bool leftHand) { method_ = m; leftHand_ = leftHand; }
  Method method() const { return method_; }
  // True if the last run ended because the wall follower came back to a
  // cell with the same heading (it is circling and will never arrive).
  bool looped() const { return looped_; }

  Explorer(Maze& maze, Router& router) : maze_(maze), router_(router) {}

  // Maze extends to the left of the start: swap left/right everywhere.
  void setMirror(bool mirror) { mirror_ = mirror; }
  bool mirror() const { return mirror_; }

  void setPose(Cell c, Dir heading) { pos_ = c; heading_ = heading; }
  Cell pos() const { return pos_; }
  Dir  heading() const { return heading_; }

  // exploreForSpeed (return runs): first visit the unknown cells on the
  // optimistic fastest path when it promises to be > gainThreshold faster
  // than the verified fastest path, then go to the start.
  void startRun(Target target, bool exploreForSpeed, const TimeModel& tm, float gainThreshold = 0.10f);

  // Walls of the current cell as seen by the robot (relative to its heading).
  // Returns the next action and advances the internal pose as if executed.
  Action step(bool wallLeft, bool wallFront, bool wallRight);

  uint16_t conflictsThisRun() const { return runConflicts_; }
  bool exploring() const { return exploringNow_; }

  // Speed-run plan in the robot's frame (mirror applied to the turns).
  bool planSpeedRun(Target target, const TimeModel& tm, bool optimistic, Path& out);
  // Is the optimistic route worth exploring? (> gainThreshold faster.)
  bool explorationWorthwhile(const TimeModel& tm, float gainThreshold, CellSet* cellsOut);

  // Start cell set (always just (0,0)).
  static CellSet startSet();

 private:
  Maze&   maze_;
  Router& router_;
  bool    mirror_ = false;
  Cell    pos_ = {0, 0};
  Dir     heading_ = NORTH;
  Target  target_ = TO_GOAL;
  bool    exploreForSpeed_ = false;
  bool    exploringNow_ = false;
  TimeModel tm_ = {192.0f, 250.0f, 1500.0f, 0.5f, 0.8f, 0.1f};
  float   gain_ = 0.10f;
  uint16_t runConflicts_ = 0;
  int     steps_ = 0;
  Path    scratch_;
  Method  method_ = FLOOD_FILL;
  bool    leftHand_ = true;
  bool    looped_ = false;
  uint8_t seenHeadings_[MAX_CELLS] = {};   // wall hug: bit d = passed here facing d

  Action wallHugStep(bool wallLeft, bool wallFront, bool wallRight);
};

}  // namespace mm
