#pragma once

#include <stdint.h>

#ifndef MAZE_MAX_SIZE
#define MAZE_MAX_SIZE 16
#endif

namespace mm {

constexpr int MAX_SIZE  = MAZE_MAX_SIZE;
constexpr int MAX_CELLS = MAX_SIZE * MAX_SIZE;
constexpr uint16_t UNREACHABLE = 0xFFFF;

enum Dir : uint8_t { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

inline Dir turnLeft(Dir d)  { return static_cast<Dir>((d + 3) & 3); }
inline Dir turnRight(Dir d) { return static_cast<Dir>((d + 1) & 3); }
inline Dir turnBack(Dir d)  { return static_cast<Dir>((d + 2) & 3); }
inline int dx(Dir d) { return d == EAST ? 1 : (d == WEST ? -1 : 0); }
inline int dy(Dir d) { return d == NORTH ? 1 : (d == SOUTH ? -1 : 0); }
char dirChar(Dir d);

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

enum class Action : uint8_t { FORWARD, LEFT, RIGHT, BACK, ARRIVED, NO_PATH };
const char* actionName(Action a);

enum class WallResult : uint8_t { UNCHANGED, LEARNED, CONFLICT };

class Maze {
 public:

  void reset(int width, int height);

  int  width() const  { return w_; }
  int  height() const { return h_; }
  bool inside(Cell c) const { return c.x >= 0 && c.y >= 0 && c.x < w_ && c.y < h_; }

  bool isKnown(Cell c, Dir d) const;
  bool hasWall(Cell c, Dir d) const;

  bool isOpen(Cell c, Dir d, bool optimistic) const;

  WallResult setWall(Cell c, Dir d, bool present);
  bool visited(Cell c) const;
  uint16_t conflicts() const { return conflicts_; }

  int forgetPresentWalls();

  void setCentreGoal();
  void setGoal(const CellSet& goal) { goal_ = goal; }
  const CellSet& goal() const { return goal_; }
  bool isGoal(Cell c) const { return goal_.has(c); }

  int postViolations() const;

  static constexpr int SERIAL_BYTES = 8 + MAX_CELLS;
  int  serialize(uint8_t* buf, int capacity) const;
  bool deserialize(const uint8_t* buf, int length);

 private:

  uint8_t cell_[MAX_SIZE][MAX_SIZE] = {};
  uint8_t w_ = 0, h_ = 0;
  uint16_t conflicts_ = 0;
  CellSet goal_;

  void setRaw(Cell c, Dir d, bool known, bool present);
};

struct TimeModel {
  float cellMm;
  float speedMmS;
  float accelMmS2;
  float turn90S;
  float turn180S;
  float segmentS;
  float straightS(int cells) const;

  static TimeModel fromProfile(float cellMm, float speedMmS, float accelMmS2, float turnRateDps,
                               float turnAccelDps2, float turnSettleS, float segmentS);
};

struct PathStep {
  int8_t  turn;
  uint8_t cells;
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

class Router {
 public:
  explicit Router(const Maze& maze) : maze_(maze) {}

  void flood(const CellSet& targets, bool optimistic);
  uint16_t dist(Cell c) const { return dist_[c.x][c.y]; }

  Action nextAction(Cell pos, Dir heading, bool optimistic) const;

  bool fastest(Cell from, Dir heading, const CellSet& targets, bool optimistic,
               const TimeModel& tm, Path& out);

 private:
  const Maze& maze_;
  uint16_t dist_[MAX_SIZE][MAX_SIZE] = {};

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

class Explorer {
 public:
  enum Target : uint8_t { TO_GOAL, TO_START };

  enum Method : uint8_t { FLOOD_FILL, WALL_HUG };
  void setMethod(Method m, bool leftHand) { method_ = m; leftHand_ = leftHand; }
  Method method() const { return method_; }

  bool looped() const { return looped_; }

  Explorer(Maze& maze, Router& router) : maze_(maze), router_(router) {}

  void setMirror(bool mirror) { mirror_ = mirror; }
  bool mirror() const { return mirror_; }

  void setPose(Cell c, Dir heading) { pos_ = c; heading_ = heading; }
  Cell pos() const { return pos_; }
  Dir  heading() const { return heading_; }

  void startRun(Target target, bool exploreForSpeed, const TimeModel& tm, float gainThreshold = 0.10f);

  Action step(bool wallLeft, bool wallFront, bool wallRight);

  uint16_t conflictsThisRun() const { return runConflicts_; }
  bool exploring() const { return exploringNow_; }

  bool planSpeedRun(Target target, const TimeModel& tm, bool optimistic, Path& out);

  bool explorationWorthwhile(const TimeModel& tm, float gainThreshold, CellSet* cellsOut);

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
  uint8_t seenHeadings_[MAX_CELLS] = {};

  Action wallHugStep(bool wallLeft, bool wallFront, bool wallRight);
};

}
