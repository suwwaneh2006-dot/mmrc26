#include "maze.h"

#include <math.h>
#include <string.h>

namespace mm {

namespace {

constexpr float INF_COST = 1e30f;
constexpr uint8_t SERIAL_MAGIC0 = 'M';
constexpr uint8_t SERIAL_MAGIC1 = 'Z';
constexpr uint8_t SERIAL_VERSION = 1;

inline int cellIndex(Cell c) { return c.x * MAX_SIZE + c.y; }
inline uint16_t stateOf(Cell c, Dir d) { return static_cast<uint16_t>((cellIndex(c) << 2) | d); }
inline Cell cellOfState(uint16_t s) {
  const int idx = s >> 2;
  Cell c = {static_cast<int8_t>(idx / MAX_SIZE), static_cast<int8_t>(idx % MAX_SIZE)};
  return c;
}
inline Dir dirOfState(uint16_t s) { return static_cast<Dir>(s & 3); }

inline int leftQuarters(Dir a, Dir b) { return (a - b) & 3; }

inline int8_t turnCode(int quarters) {
  switch (quarters & 3) {
    case 1:  return 1;
    case 2:  return 2;
    case 3:  return -1;
    default: return 0;
  }
}
inline int absi(int v) { return v < 0 ? -v : v; }
inline int manhattan(Cell a, Cell b) { return absi(a.x - b.x) + absi(a.y - b.y); }
inline int quartersOf(int8_t code) { return code == 1 ? 1 : (code == 2 ? 2 : (code == -1 ? 3 : 0)); }

float trapezoidS(float d, float vmax, float accel) {
  const float dAccel = vmax * vmax / accel;
  return d < dAccel ? 2.0f * sqrtf(d / accel) : d / vmax + vmax / accel;
}

uint16_t fletcher16(const uint8_t* data, int n) {
  uint16_t a = 0, b = 0;
  for (int i = 0; i < n; ++i) {
    a = static_cast<uint16_t>((a + data[i]) % 255);
    b = static_cast<uint16_t>((b + a) % 255);
  }
  return static_cast<uint16_t>((b << 8) | a);
}

}

char dirChar(Dir d) {
  static const char k[4] = {'n', 'e', 's', 'w'};
  return k[d & 3];
}

const char* actionName(Action a) {
  switch (a) {
    case Action::FORWARD: return "forward";
    case Action::LEFT:    return "left";
    case Action::RIGHT:   return "right";
    case Action::BACK:    return "back";
    case Action::ARRIVED: return "arrived";
    case Action::NO_PATH: return "no-path";
  }
  return "?";
}

void CellSet::clear() {
  memset(bits_, 0, sizeof(bits_));
  count_ = 0;
}

void CellSet::add(Cell c) {
  if (c.x < 0 || c.y < 0 || c.x >= MAX_SIZE || c.y >= MAX_SIZE || has(c)) return;
  const int i = cellIndex(c);
  bits_[i >> 3] = static_cast<uint8_t>(bits_[i >> 3] | (1u << (i & 7)));
  ++count_;
}

bool CellSet::has(Cell c) const {
  if (c.x < 0 || c.y < 0 || c.x >= MAX_SIZE || c.y >= MAX_SIZE) return false;
  const int i = cellIndex(c);
  return (bits_[i >> 3] >> (i & 7)) & 1u;
}

void Maze::reset(int width, int height) {
  w_ = static_cast<uint8_t>(width < 1 ? 1 : (width > MAX_SIZE ? MAX_SIZE : width));
  h_ = static_cast<uint8_t>(height < 1 ? 1 : (height > MAX_SIZE ? MAX_SIZE : height));
  memset(cell_, 0, sizeof(cell_));
  conflicts_ = 0;
  for (int x = 0; x < w_; ++x) {
    for (int y = 0; y < h_; ++y) {
      const Cell c = {static_cast<int8_t>(x), static_cast<int8_t>(y)};
      for (int d = 0; d < 4; ++d) {
        if (!inside(neighbour(c, static_cast<Dir>(d)))) setRaw(c, static_cast<Dir>(d), true, true);
      }
    }
  }
  setCentreGoal();
}

void Maze::setRaw(Cell c, Dir d, bool known, bool present) {
  const uint8_t wallBit = static_cast<uint8_t>(1u << d);
  const uint8_t knownBit = static_cast<uint8_t>(1u << (d + 4));
  uint8_t& v = cell_[c.x][c.y];
  v = static_cast<uint8_t>((v & ~(wallBit | knownBit)) | (known ? knownBit : 0) | (present ? wallBit : 0));
  const Cell n = neighbour(c, d);
  if (inside(n)) {
    const Dir b = turnBack(d);
    const uint8_t wb = static_cast<uint8_t>(1u << b);
    const uint8_t kb = static_cast<uint8_t>(1u << (b + 4));
    uint8_t& u = cell_[n.x][n.y];
    u = static_cast<uint8_t>((u & ~(wb | kb)) | (known ? kb : 0) | (present ? wb : 0));
  }
}

bool Maze::isKnown(Cell c, Dir d) const {
  return inside(c) && ((cell_[c.x][c.y] >> (d + 4)) & 1u);
}

bool Maze::hasWall(Cell c, Dir d) const {
  if (!inside(c)) return true;
  const uint8_t v = cell_[c.x][c.y];
  return ((v >> (d + 4)) & 1u) && ((v >> d) & 1u);
}

bool Maze::isOpen(Cell c, Dir d, bool optimistic) const {
  if (!inside(c) || !inside(neighbour(c, d))) return false;
  const uint8_t v = cell_[c.x][c.y];
  const bool known = (v >> (d + 4)) & 1u;
  if (!known) return optimistic;
  return !((v >> d) & 1u);
}

WallResult Maze::setWall(Cell c, Dir d, bool present) {
  if (!inside(c)) return WallResult::UNCHANGED;
  if (!inside(neighbour(c, d))) {

    return present ? WallResult::UNCHANGED : WallResult::CONFLICT;
  }
  if (isKnown(c, d)) {
    if (hasWall(c, d) == present) return WallResult::UNCHANGED;

    setRaw(c, d, false, false);
    if (conflicts_ < 0xFFFF) ++conflicts_;
    return WallResult::CONFLICT;
  }
  setRaw(c, d, true, present);
  return WallResult::LEARNED;
}

int Maze::forgetPresentWalls() {
  int n = 0;
  for (int x = 0; x < w_; ++x) {
    for (int y = 0; y < h_; ++y) {
      const Cell c = {static_cast<int8_t>(x), static_cast<int8_t>(y)};
      for (int d = 0; d < 4; ++d) {
        const Dir dir = static_cast<Dir>(d);
        if (inside(neighbour(c, dir)) && hasWall(c, dir)) {
          setRaw(c, dir, false, false);
          ++n;
        }
      }
    }
  }
  return n;
}

bool Maze::visited(Cell c) const {
  return inside(c) && (cell_[c.x][c.y] & 0xF0) == 0xF0;
}

void Maze::setCentreGoal() {
  goal_.clear();
  const int x0 = (w_ - 1) / 2, x1 = w_ / 2;
  const int y0 = (h_ - 1) / 2, y1 = h_ / 2;
  for (int x = x0; x <= x1; ++x) {
    for (int y = y0; y <= y1; ++y) {
      const Cell c = {static_cast<int8_t>(x), static_cast<int8_t>(y)};
      goal_.add(c);
    }
  }
}

int Maze::postViolations() const {
  int violations = 0;
  for (int i = 1; i < w_; ++i) {
    for (int j = 1; j < h_; ++j) {
      const Cell sw = {static_cast<int8_t>(i - 1), static_cast<int8_t>(j - 1)};
      const Cell se = {static_cast<int8_t>(i), static_cast<int8_t>(j - 1)};
      const Cell nw = {static_cast<int8_t>(i - 1), static_cast<int8_t>(j)};
      const Cell ne = {static_cast<int8_t>(i), static_cast<int8_t>(j)};
      if (isGoal(sw) && isGoal(se) && isGoal(nw) && isGoal(ne)) continue;
      const bool allKnownOpen = isKnown(sw, EAST) && !hasWall(sw, EAST) &&
                                isKnown(nw, EAST) && !hasWall(nw, EAST) &&
                                isKnown(sw, NORTH) && !hasWall(sw, NORTH) &&
                                isKnown(se, NORTH) && !hasWall(se, NORTH);
      if (allKnownOpen) ++violations;
    }
  }
  return violations;
}

int Maze::serialize(uint8_t* buf, int capacity) const {
  if (capacity < SERIAL_BYTES) return 0;
  buf[0] = SERIAL_MAGIC0;
  buf[1] = SERIAL_MAGIC1;
  buf[2] = SERIAL_VERSION;
  buf[3] = static_cast<uint8_t>(MAX_SIZE);
  buf[4] = w_;
  buf[5] = h_;
  memcpy(buf + 8, cell_, MAX_CELLS);
  const uint16_t sum = fletcher16(buf + 8, MAX_CELLS);
  buf[6] = static_cast<uint8_t>(sum & 0xFF);
  buf[7] = static_cast<uint8_t>(sum >> 8);
  return SERIAL_BYTES;
}

bool Maze::deserialize(const uint8_t* buf, int length) {
  if (length != SERIAL_BYTES || buf[0] != SERIAL_MAGIC0 || buf[1] != SERIAL_MAGIC1 ||
      buf[2] != SERIAL_VERSION || buf[3] != MAX_SIZE || buf[4] < 1 || buf[5] < 1 ||
      buf[4] > MAX_SIZE || buf[5] > MAX_SIZE) {
    return false;
  }
  const uint16_t sum = fletcher16(buf + 8, MAX_CELLS);
  if (buf[6] != (sum & 0xFF) || buf[7] != (sum >> 8)) return false;
  w_ = buf[4];
  h_ = buf[5];
  memcpy(cell_, buf + 8, MAX_CELLS);
  conflicts_ = 0;
  setCentreGoal();
  return true;
}

float TimeModel::straightS(int cells) const {
  return trapezoidS(cells * cellMm, speedMmS, accelMmS2) + segmentS;
}

TimeModel TimeModel::fromProfile(float cellMm, float speedMmS, float accelMmS2, float turnRateDps,
                                 float turnAccelDps2, float turnSettleS, float segmentS) {
  TimeModel tm;
  tm.cellMm = cellMm;
  tm.speedMmS = speedMmS;
  tm.accelMmS2 = accelMmS2;
  tm.turn90S = trapezoidS(90.0f, turnRateDps, turnAccelDps2) + turnSettleS;
  tm.turn180S = trapezoidS(180.0f, turnRateDps, turnAccelDps2) + turnSettleS;
  tm.segmentS = segmentS;
  return tm;
}

int Path::cells() const {
  int n = 0;
  for (int i = 0; i < count; ++i) n += steps[i].cells;
  return n;
}

void Router::flood(const CellSet& targets, bool optimistic) {
  static Cell queue[MAX_CELLS];
  int head = 0, tail = 0;
  for (int x = 0; x < MAX_SIZE; ++x) {
    for (int y = 0; y < MAX_SIZE; ++y) dist_[x][y] = UNREACHABLE;
  }
  for (int x = 0; x < maze_.width(); ++x) {
    for (int y = 0; y < maze_.height(); ++y) {
      const Cell c = {static_cast<int8_t>(x), static_cast<int8_t>(y)};
      if (targets.has(c)) {
        dist_[x][y] = 0;
        queue[tail++] = c;
      }
    }
  }
  while (head < tail) {
    const Cell c = queue[head++];
    const uint16_t next = static_cast<uint16_t>(dist_[c.x][c.y] + 1);
    for (int d = 0; d < 4; ++d) {
      if (!maze_.isOpen(c, static_cast<Dir>(d), optimistic)) continue;
      const Cell n = neighbour(c, static_cast<Dir>(d));
      if (dist_[n.x][n.y] != UNREACHABLE) continue;
      dist_[n.x][n.y] = next;
      queue[tail++] = n;
    }
  }
}

Action Router::nextAction(Cell pos, Dir heading, bool optimistic) const {
  const uint16_t here = dist(pos);
  if (here == UNREACHABLE) return Action::NO_PATH;
  if (here == 0) return Action::ARRIVED;

  const Dir options[4] = {heading, turnLeft(heading), turnRight(heading), turnBack(heading)};
  const Action actions[4] = {Action::FORWARD, Action::LEFT, Action::RIGHT, Action::BACK};
  int best = -1;
  uint16_t bestDist = here;
  for (int i = 0; i < 4; ++i) {
    if (!maze_.isOpen(pos, options[i], optimistic)) continue;
    const uint16_t d = dist(neighbour(pos, options[i]));
    if (d < bestDist) {
      bestDist = d;
      best = i;
    }
  }
  return best < 0 ? Action::NO_PATH : actions[best];
}

void Router::siftUp(int i) {
  while (i > 0) {
    const int parent = (i - 1) / 2;
    if (cost_[heap_[parent]] <= cost_[heap_[i]]) break;
    const uint16_t t = heap_[parent];
    heap_[parent] = heap_[i];
    heap_[i] = t;
    heapPos_[heap_[parent]] = static_cast<int16_t>(parent);
    heapPos_[heap_[i]] = static_cast<int16_t>(i);
    i = parent;
  }
}

void Router::siftDown(int i) {
  while (true) {
    const int l = 2 * i + 1, r = l + 1;
    int m = i;
    if (l < heapSize_ && cost_[heap_[l]] < cost_[heap_[m]]) m = l;
    if (r < heapSize_ && cost_[heap_[r]] < cost_[heap_[m]]) m = r;
    if (m == i) break;
    const uint16_t t = heap_[m];
    heap_[m] = heap_[i];
    heap_[i] = t;
    heapPos_[heap_[m]] = static_cast<int16_t>(m);
    heapPos_[heap_[i]] = static_cast<int16_t>(i);
    i = m;
  }
}

void Router::heapPush(uint16_t s) {
  heap_[heapSize_] = s;
  heapPos_[s] = static_cast<int16_t>(heapSize_);
  ++heapSize_;
  siftUp(heapSize_ - 1);
}

void Router::heapDecrease(uint16_t s) { siftUp(heapPos_[s]); }

uint16_t Router::heapPop() {
  const uint16_t top = heap_[0];
  --heapSize_;
  heap_[0] = heap_[heapSize_];
  heapPos_[heap_[0]] = 0;
  heapPos_[top] = -2;
  if (heapSize_ > 0) siftDown(0);
  return top;
}

bool Router::fastest(Cell from, Dir heading, const CellSet& targets, bool optimistic,
                     const TimeModel& tm, Path& out) {
  out.count = 0;
  out.timeS = 0.0f;
  out.end = from;
  out.endHeading = heading;
  if (!maze_.inside(from)) return false;

  constexpr int STATES = MAX_CELLS * 4;
  for (int s = 0; s < STATES; ++s) {
    cost_[s] = INF_COST;
    prev_[s] = 0xFFFF;
    heapPos_[s] = -1;
  }
  heapSize_ = 0;

  const uint16_t start = stateOf(from, heading);
  cost_[start] = 0.0f;
  heapPush(start);
  int goalState = -1;

  while (heapSize_ > 0) {
    const uint16_t s = heapPop();
    const Cell c = cellOfState(s);
    const Dir d = dirOfState(s);
    if (targets.has(c)) {
      goalState = s;
      break;
    }

    auto relax = [&](uint16_t t, float w) {
      if (heapPos_[t] == -2) return;
      const float nc = cost_[s] + w;
      if (nc < cost_[t]) {
        cost_[t] = nc;
        prev_[t] = s;
        if (heapPos_[t] == -1) heapPush(t);
        else heapDecrease(t);
      }
    };
    relax(stateOf(c, turnLeft(d)), tm.turn90S);
    relax(stateOf(c, turnRight(d)), tm.turn90S);
    relax(stateOf(c, turnBack(d)), tm.turn180S);
    Cell n = c;
    for (int k = 1; maze_.isOpen(n, d, optimistic); ++k) {
      n = neighbour(n, d);
      relax(stateOf(n, d), tm.straightS(k));
    }
  }
  if (goalState < 0) return false;

  int len = 0;
  for (uint16_t s = static_cast<uint16_t>(goalState); s != start; s = prev_[s]) heap_[len++] = s;
  heap_[len++] = start;

  int pendingQuarters = 0;
  bool lastWasStraight = false;
  for (int i = len - 1; i > 0; --i) {
    const uint16_t a = heap_[i], b = heap_[i - 1];
    const Cell ca = cellOfState(a), cb = cellOfState(b);
    if (ca == cb) {
      pendingQuarters += leftQuarters(dirOfState(a), dirOfState(b));
      lastWasStraight = false;
    } else {
      const int k = manhattan(ca, cb);
      if (lastWasStraight && (pendingQuarters & 3) == 0 && out.count > 0) {
        out.steps[out.count - 1].cells = static_cast<uint8_t>(out.steps[out.count - 1].cells + k);
      } else if (out.count < MAX_PATH_STEPS) {
        out.steps[out.count].turn = turnCode(pendingQuarters);
        out.steps[out.count].cells = static_cast<uint8_t>(k);
        ++out.count;
      }
      pendingQuarters = 0;
      lastWasStraight = true;
    }
  }
  float t = 0.0f;
  for (int i = 0; i < out.count; ++i) {
    const int q = quartersOf(out.steps[i].turn);
    if (q == 2) t += tm.turn180S;
    else if (q != 0) t += tm.turn90S;
    t += tm.straightS(out.steps[i].cells);
  }
  out.timeS = t;
  out.end = cellOfState(static_cast<uint16_t>(goalState));
  out.endHeading = dirOfState(static_cast<uint16_t>(goalState));
  return true;
}

CellSet Explorer::startSet() {
  CellSet s;
  const Cell start = {0, 0};
  s.add(start);
  return s;
}

void Explorer::startRun(Target target, bool exploreForSpeed, const TimeModel& tm, float gainThreshold) {
  target_ = target;
  exploreForSpeed_ = exploreForSpeed;
  exploringNow_ = false;
  tm_ = tm;
  gain_ = gainThreshold;
  runConflicts_ = 0;
  steps_ = 0;
  looped_ = false;
  memset(seenHeadings_, 0, sizeof(seenHeadings_));
}

Action Explorer::wallHugStep(bool wallLeft, bool wallFront, bool wallRight) {
  const int idx = pos_.x * MAX_SIZE + pos_.y;
  const uint8_t bit = static_cast<uint8_t>(1u << heading_);
  if (seenHeadings_[idx] & bit) {
    looped_ = true;
    return Action::NO_PATH;
  }
  seenHeadings_[idx] = static_cast<uint8_t>(seenHeadings_[idx] | bit);
  const bool firstOpen = leftHand_ ? !wallLeft : !wallRight;
  const bool lastOpen = leftHand_ ? !wallRight : !wallLeft;
  if (firstOpen) return leftHand_ ? Action::LEFT : Action::RIGHT;
  if (!wallFront) return Action::FORWARD;
  if (lastOpen) return leftHand_ ? Action::RIGHT : Action::LEFT;
  return Action::BACK;
}

bool Explorer::explorationWorthwhile(const TimeModel& tm, float gainThreshold, CellSet* cellsOut) {
  if (cellsOut != nullptr) cellsOut->clear();
  const Cell start = {0, 0};
  if (!router_.fastest(start, NORTH, maze_.goal(), true, tm, scratch_)) return false;
  const float optimistic = scratch_.timeS;

  if (cellsOut != nullptr) {
    Cell c = start;
    Dir h = NORTH;
    for (int i = 0; i < scratch_.count; ++i) {
      const int q = quartersOf(scratch_.steps[i].turn);
      for (int r = 0; r < q; ++r) h = turnLeft(h);
      for (int k = 0; k < scratch_.steps[i].cells; ++k) {
        c = neighbour(c, h);
        if (!maze_.visited(c)) cellsOut->add(c);
      }
    }
  }
  Path verified;
  if (!router_.fastest(start, NORTH, maze_.goal(), false, tm, verified)) return true;
  return optimistic < verified.timeS * (1.0f - gainThreshold);
}

Action Explorer::step(bool wallLeft, bool wallFront, bool wallRight) {
  if (mirror_) {
    const bool t = wallLeft;
    wallLeft = wallRight;
    wallRight = t;
  }
  const WallResult r[3] = {maze_.setWall(pos_, turnLeft(heading_), wallLeft),
                           maze_.setWall(pos_, heading_, wallFront),
                           maze_.setWall(pos_, turnRight(heading_), wallRight)};
  for (WallResult w : r) {
    if (w == WallResult::CONFLICT && runConflicts_ < 0xFFFF) ++runConflicts_;
  }
  if (++steps_ > 4 * MAX_CELLS) return Action::NO_PATH;

  Action a;
  const bool arrived = target_ == TO_GOAL ? maze_.isGoal(pos_) : pos_ == Cell{0, 0};
  if (method_ == WALL_HUG) {
    a = arrived ? Action::ARRIVED : wallHugStep(wallLeft, wallFront, wallRight);
  } else if (target_ == TO_GOAL) {
    router_.flood(maze_.goal(), true);
    a = router_.nextAction(pos_, heading_, true);
  } else {
    a = Action::NO_PATH;
    exploringNow_ = false;
    CellSet targets;
    if (exploreForSpeed_ && explorationWorthwhile(tm_, gain_, &targets) && targets.count() > 0) {
      router_.flood(targets, true);
      a = router_.nextAction(pos_, heading_, true);
      exploringNow_ = a != Action::NO_PATH && a != Action::ARRIVED;
    }
    if (!exploringNow_) {
      router_.flood(startSet(), true);
      a = router_.nextAction(pos_, heading_, true);
    }
  }

  switch (a) {
    case Action::FORWARD: break;
    case Action::LEFT:    heading_ = turnLeft(heading_); break;
    case Action::RIGHT:   heading_ = turnRight(heading_); break;
    case Action::BACK:    heading_ = turnBack(heading_); break;
    case Action::ARRIVED:
    case Action::NO_PATH: return a;
  }
  pos_ = neighbour(pos_, heading_);
  if (mirror_ && a == Action::LEFT) return Action::RIGHT;
  if (mirror_ && a == Action::RIGHT) return Action::LEFT;
  return a;
}

bool Explorer::planSpeedRun(Target target, const TimeModel& tm, bool optimistic, Path& out) {
  const CellSet targets = target == TO_GOAL ? maze_.goal() : startSet();
  if (!router_.fastest(pos_, heading_, targets, optimistic, tm, out)) return false;
  if (mirror_) {
    for (int i = 0; i < out.count; ++i) {
      if (out.steps[i].turn == 1) out.steps[i].turn = -1;
      else if (out.steps[i].turn == -1) out.steps[i].turn = 1;
    }
  }
  return true;
}

}
