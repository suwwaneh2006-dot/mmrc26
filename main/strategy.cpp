#include "strategy.h"

#include <Preferences.h>

#include "battery.h"
#include "encoders.h"
#include "estimator.h"
#include "imu.h"
#include "maze.h"
#include "motion.h"
#include "motors.h"
#include "sched.h"
#include "sonar.h"
#include "ui.h"

namespace {

using mm::Action;
using mm::Explorer;

enum class Outcome : uint8_t { ARRIVED, ABORTED, NO_PATH };
enum class Trigger : uint8_t { GO, RESCUE, EXIT };

mm::Maze     g_maze;
mm::Router   g_router(g_maze);
mm::Explorer g_explorer(g_maze, g_router);
mm::Path     g_scratchPath;
bool         g_mirror = false;

const char* g_abortReason = "";
uint16_t g_seenConflicts = 0;
int      g_loopConflicts = 0;
int      g_postViolations = 0;
float    g_nextStartS = 0.0f;
float    g_nextSigma = START_SIGMA_MM;

mm::TimeModel tierModel(int tier) {
  const SpeedTier& t = TIERS[tier];
  return mm::TimeModel::fromProfile(CELL_PITCH_MM, t.speed_mm_s, t.accel_mm_s2, t.turnRate_dps, TURN_ACCEL_DPS2,
                                    PLAN_TURN_SETTLE_S, PLAN_SEGMENT_S);
}

const mm::Cell START_CELL = {0, 0};

float verifiedLoopS(int tier) {
  const mm::TimeModel tm = tierModel(tier);
  if (!g_router.fastest(START_CELL, mm::NORTH, g_maze.goal(), false, tm, g_scratchPath)) return -1.0f;
  const float out = g_scratchPath.timeS;
  const mm::Cell end = g_scratchPath.end;
  const mm::Dir back = mm::turnBack(g_scratchPath.endHeading);
  if (!g_router.fastest(end, back, Explorer::startSet(), false, tm, g_scratchPath)) return -1.0f;
  return out + g_scratchPath.timeS;
}

float frontFace(mm::Cell c, mm::Dir h, int k) {
  for (int guard = 0; guard < MAZE_SIZE_CELLS + 1; ++guard) {
    const float face = k * CELL_PITCH_MM + HALF_CELL_INNER_MM;
    if (!g_maze.isKnown(c, h)) return -1.0f;
    if (g_maze.hasWall(c, h)) return face;
    c = mm::neighbour(c, h);
    ++k;
    if (!g_maze.inside(c)) return -1.0f;
  }
  return -1.0f;
}

struct MapRecord {
  uint8_t mirror;
  uint8_t blob[mm::Maze::SERIAL_BYTES];
};
MapRecord g_record;

void writeRecord() {
  Preferences prefs;
  if (prefs.begin(NVS_MAP_NAMESPACE, false)) {
    prefs.putBytes("map", &g_record, sizeof(g_record));
    prefs.end();
  }
}

void saveMap(bool moving) {
  if (moving && !NVS_MIRROR_EVERY_CELL) return;
  g_record.mirror = g_mirror ? 1 : 0;
  if (g_maze.serialize(g_record.blob, sizeof(g_record.blob)) == 0) return;
  sched::blockingSection(&writeRecord);
}

bool loadMap(bool& mirror) {
  Preferences prefs;
  if (!prefs.begin(NVS_MAP_NAMESPACE, true)) return false;
  const size_t n = prefs.getBytes("map", &g_record, sizeof(g_record));
  prefs.end();
  if (n != sizeof(g_record) || !g_maze.deserialize(g_record.blob, sizeof(g_record.blob))) return false;
  if (g_maze.width() != MAZE_SIZE_CELLS || g_maze.height() != MAZE_SIZE_CELLS) return false;
  mirror = g_record.mirror != 0;
  return true;
}

struct Walls {
  bool l, f, r;
};

bool sideWall(sonar::Id id, float expectMm, float sensorX, float centreS, uint32_t maxAgeMs) {
  const sonar::Reading r = sonar::read(id);
  if (r.valid && sonar::ageMs(id) <= maxAgeMs) {
    const float rel = estimator::sAt(r.tUs) + sensorX - centreS;
    if (fabsf(rel) < CELL_PITCH_MM * 0.5f - SIDE_POST_KEEPOUT_MM) {
      if (r.rawMm <= expectMm + SIDE_WALL_ON_MARGIN_MM) return true;
      if (r.rawMm >= expectMm + SIDE_WALL_OFF_MARGIN_MM) return false;
    }
  }
  return r.wall;
}

Walls readWalls(float centreS, uint32_t maxAgeMs) {
  Walls w;
  const sonar::Reading f = sonar::read(sonar::FRONT);
  const float centreAhead = centreS - estimator::s();
  const float expect = HALF_CELL_INNER_MM + centreAhead - SONAR_F_X_MM;
  w.f = f.inRange && f.mm < expect + CELL_PITCH_MM * 0.5f;
  w.l = sideWall(sonar::LEFT, SIDE_L_EXPECT_MM, SONAR_L_X_MM, centreS, maxAgeMs);
  w.r = sideWall(sonar::RIGHT, SIDE_R_EXPECT_MM, SONAR_R_X_MM, centreS, maxAgeMs);
  return w;
}

void accountConflicts(int newOnes) {
  for (int i = 0; i < newOnes; ++i) estimator::inflate(CONFLICT_SIGMA_MM);
  g_loopConflicts += newOnes;
  if (newOnes > 0) DBG_PRINTF("map contradiction (%d this loop)\n", g_loopConflicts);
}

void syncExplorerConflicts() {
  const uint16_t c = g_explorer.conflictsThisRun();
  if (c > g_seenConflicts) accountConflicts(c - g_seenConflicts);
  g_seenConflicts = c;
}

bool abortRequested() {
  const char* why = nullptr;
  if (ui::event() == ui::Button::SHORT) why = "button (touch)";
  else if (motors::faultCode() != Fault::NONE) why = faultName(motors::faultCode());
  else if (sonar::faulty(sonar::FRONT) || sonar::faulty(sonar::LEFT) || sonar::faulty(sonar::RIGHT)) why = "sonar fault";
  else if (g_loopConflicts >= CONFLICTS_STOP) why = "too many map contradictions";
  if (why == nullptr) return false;
  g_abortReason = why;
  motion::stop();
  while (motion::busy()) sched::service();
  return true;
}

bool waitMotion() {
  while (motion::busy()) {
    sched::service();
    if (abortRequested()) return false;
  }
  if (motion::result() != motion::Result::DONE) {
    g_abortReason = motion::resultName(motion::result());
    return false;
  }
  return true;
}

bool alignToFrontWall() {
  motion::alignFront();
  if (!waitMotion()) return false;
  g_nextStartS = 0.0f;
  g_nextSigma = ALIGNED_SIGMA_MM;
  return true;
}

bool lateralAtRest(float& lat) {
  float sum = 0.0f;
  int n = 0;
  const sonar::Reading l = sonar::read(sonar::LEFT);
  const sonar::Reading r = sonar::read(sonar::RIGHT);
  if (l.wall && l.inRange && sonar::ageMs(sonar::LEFT) < RECENTRE_READING_MAX_AGE_MS) {
    sum += SIDE_L_EXPECT_MM - l.mm;
    ++n;
  }
  if (r.wall && r.inRange && sonar::ageMs(sonar::RIGHT) < RECENTRE_READING_MAX_AGE_MS) {
    sum += r.mm - SIDE_R_EXPECT_MM;
    ++n;
  }
  if (n == 0) return false;
  lat = sum / n;
  return true;
}

bool recentreForPivot() {
  if (!PIVOT_RECENTRE) return true;
  float lat;
  if (!lateralAtRest(lat) || fabsf(lat) <= PIVOT_LATERAL_LIMIT_MM || fabsf(lat) > RECENTRE_MAX_OFFSET_MM) {
    return true;
  }
  const float sign = lat > 0.0f ? 1.0f : -1.0f;
  const float angleRad = RECENTRE_ANGLE_DEG * 0.01745329f;
  const float back = fabsf(lat) / sinf(angleRad);
  DBG_PRINTF("re-centring: %.0f mm off centre, backing up %.0f mm\n", lat, back);
  const bool wallAhead = sonar::read(sonar::FRONT).wall;
  motion::pivot(sign * RECENTRE_ANGLE_DEG, TIERS[0]);
  if (!waitMotion()) return false;
  motion::reverse(back, TIERS[0]);
  if (!waitMotion()) return false;
  motion::pivot(-sign * RECENTRE_ANGLE_DEG, TIERS[0]);
  if (!waitMotion()) return false;
  if (wallAhead) {
    motion::alignFront();
  } else {

    const float behind = back * cosf(angleRad);
    motion::runBegin(TIERS[0], -behind, ALIGNED_SIGMA_MM);
    motion::runExtend(behind);
  }
  return waitMotion();
}

bool pivotSafe() {
  if (sonar::ageMs(sonar::LEFT) > RECENTRE_READING_MAX_AGE_MS || sonar::ageMs(sonar::RIGHT) > RECENTRE_READING_MAX_AGE_MS ||
      sonar::ageMs(sonar::FRONT) > RECENTRE_READING_MAX_AGE_MS) {
    sched::waitMs(STATIONARY_READ_MS);
  }
  const sonar::Reading f = sonar::read(sonar::FRONT);
  if (f.inRange && f.mm < FRONT_STOP_READING_MM - ALIGN_TOLERANCE_MM) {
    motion::reverse(FRONT_STOP_READING_MM - f.mm, TIERS[0]);
    if (!waitMotion()) return false;
    sched::waitMs(STATIONARY_READ_MS);
  }
  return recentreForPivot();
}

bool pivotPhysical(float angleDeg, const SpeedTier& tier) {
  if (!pivotSafe()) return false;
  motion::pivot(angleDeg, tier);
  if (!waitMotion()) return false;
  g_nextSigma = fmaxf(g_nextSigma, ALIGNED_SIGMA_MM);
  return true;
}

bool turnFor(Action a, bool wallAhead) {
  if (a == Action::FORWARD) return true;
  if (wallAhead && !alignToFrontWall()) return false;
  const float angle = a == Action::LEFT ? 90.0f : (a == Action::RIGHT ? -90.0f : 180.0f);
  return pivotPhysical(angle, TIERS[0]);
}

Action stepExplorer(const Walls& w, bool moving) {
  const mm::Cell here = g_explorer.pos();
  const mm::Dir heading = g_explorer.heading();
  const Action a = g_explorer.step(w.l, w.f, w.r);
  DBG_PRINTF("cell (%d,%d) facing %c: walls L%d F%d R%d -> %s\n", here.x, here.y, mm::dirChar(heading), w.l ? 1 : 0,
             w.f ? 1 : 0, w.r ? 1 : 0, mm::actionName(a));
  syncExplorerConflicts();

  const int pv = g_maze.postViolations();
  if (pv > g_postViolations) accountConflicts(pv - g_postViolations);
  g_postViolations = pv;
  saveMap(moving);
  return a;
}

Outcome exploreRun(Explorer::Target target, bool exploreForSpeed) {
  const SpeedTier& tier = TIERS[0];
  g_explorer.startRun(target, exploreForSpeed, tierModel(0), PLAN_EXPLORE_GAIN);
  g_seenConflicts = 0;

  sched::waitMs(STATIONARY_READ_MS);
  Walls w = readWalls(estimator::s() - g_nextStartS, STATIONARY_READ_MS);
  Action a = stepExplorer(w, false);

  while (true) {
    if (a == Action::ARRIVED) return Outcome::ARRIVED;
    if (a == Action::NO_PATH && g_explorer.looped()) {

      g_abortReason = "wall follower looped: target not reachable by wall hugging";
      return Outcome::NO_PATH;
    }
    if (a == Action::NO_PATH) {

      const int n = g_maze.forgetPresentWalls();
      g_postViolations = g_maze.postViolations();
      saveMap(false);
      DBG_PRINTF("map impossible: forgot %d present walls\n", n);
      g_abortReason = "no path (map contradicted itself, present walls forgotten)";
      return Outcome::NO_PATH;
    }
    if (g_loopConflicts >= CONFLICTS_STOP) {
      g_abortReason = "too many map contradictions";
      return Outcome::ABORTED;
    }
    if (!turnFor(a, w.f)) return Outcome::ABORTED;

    motion::runBegin(tier, g_nextStartS, g_nextSigma);
    motion::runExtend(CELL_PITCH_MM - g_nextStartS);
    g_nextStartS = 0.0f;
    int j = 1;
    estimator::setFrontWallFace(frontFace(g_explorer.pos(), g_explorer.heading(), 1));

    while (true) {
      bool reached = false;
      while (motion::busy()) {
        sched::service();
        if (abortRequested()) return Outcome::ABORTED;
        if (estimator::s() >= j * CELL_PITCH_MM - READ_POINT_BEFORE_CENTRE_MM) {
          reached = true;
          break;
        }
      }
      if (!reached) {

        if (motion::result() != motion::Result::DONE) {
          g_abortReason = motion::resultName(motion::result());
          return Outcome::ABORTED;
        }
        sched::waitMs(STATIONARY_READ_MS);
        w = readWalls(j * CELL_PITCH_MM, STATIONARY_READ_MS);
      } else {
        w = readWalls(j * CELL_PITCH_MM, SIDE_SAMPLE_MAX_AGE_MS);
      }
      a = stepExplorer(w, reached);
      if (w.f) estimator::setFrontWallFace(j * CELL_PITCH_MM + HALF_CELL_INNER_MM);

      if (a == Action::FORWARD && reached && g_loopConflicts < CONFLICTS_STOP) {
        motion::runExtend(CELL_PITCH_MM);
        ++j;
        if (!w.f) estimator::setFrontWallFace(frontFace(g_explorer.pos(), g_explorer.heading(), j));
        continue;
      }

      if (reached && !waitMotion()) return Outcome::ABORTED;
      g_nextSigma = estimator::sigma();
      break;
    }
  }
}

void checkSideWalls(mm::Cell c, mm::Dir h, int j) {
  struct Side { sonar::Id id; mm::Dir dir; float expect; float x; };
  const mm::Dir physLeft  = g_mirror ? mm::turnRight(h) : mm::turnLeft(h);
  const mm::Dir physRight = g_mirror ? mm::turnLeft(h) : mm::turnRight(h);
  const Side sides[2] = {{sonar::LEFT, physLeft, SIDE_L_EXPECT_MM, SONAR_L_X_MM},
                         {sonar::RIGHT, physRight, SIDE_R_EXPECT_MM, SONAR_R_X_MM}};
  int conflicts = 0;
  for (const Side& s : sides) {
    if (sonar::ageMs(s.id) > SIDE_SAMPLE_MAX_AGE_MS) continue;
    const sonar::Reading r = sonar::read(s.id);

    const float sensorRel = estimator::sAt(r.tUs) + s.x - j * CELL_PITCH_MM;
    if (fabsf(sensorRel) > CELL_PITCH_MM * 0.5f - SIDE_POST_KEEPOUT_MM - 2.0f * estimator::sigma()) continue;
    const float raw = r.rawMm;

    const bool near = raw <= s.expect + SIDE_WALL_ON_MARGIN_MM;
    const bool far = raw >= s.expect + SIDE_WALL_OFF_MARGIN_MM;
    if (!near && !far) continue;
    if (g_maze.setWall(c, s.dir, near) == mm::WallResult::CONFLICT) {
      ++conflicts;
      DBG_PRINTF("conflict: cell (%d,%d) side %c read %.0f mm, sensor %.0f mm from cell centre, v %.0f\n", c.x,
                 c.y, mm::dirChar(s.dir), raw, sensorRel, estimator::v());
    }
  }
  if (conflicts > 0) {
    accountConflicts(conflicts);
    saveMap(true);
  }
}

Outcome pathRun(Explorer::Target target, int tierIdx) {
  const SpeedTier& tier = TIERS[tierIdx];
  static mm::Path path;
  if (!g_explorer.planSpeedRun(target, tierModel(tierIdx), false, path)) {
    g_abortReason = "no verified path";
    return Outcome::NO_PATH;
  }
  DBG_PRINTF("speed path: %d straights, %d cells, planned %.2f s at T%d\n", path.count, path.cells(), path.timeS,
             tierIdx + 1);
  mm::Cell c = g_explorer.pos();
  mm::Dir h = g_explorer.heading();
  for (int i = 0; i < path.count; ++i) {
    const mm::PathStep& step = path.steps[i];
    if (step.turn != 0) {
      if (g_maze.hasWall(c, h) && !alignToFrontWall()) return Outcome::ABORTED;
      const float angle = step.turn == 1 ? 90.0f : (step.turn == -1 ? -90.0f : 180.0f);
      if (!pivotPhysical(angle, tier)) return Outcome::ABORTED;

      const int canonicalTurn = (g_mirror && step.turn != 2) ? -step.turn : step.turn;
      h = canonicalTurn == 1 ? mm::turnLeft(h) : (canonicalTurn == -1 ? mm::turnRight(h) : mm::turnBack(h));
    }
    motion::runBegin(tier, g_nextStartS, g_nextSigma);
    motion::runExtend(step.cells * CELL_PITCH_MM - g_nextStartS);
    g_nextStartS = 0.0f;
    estimator::setFrontWallFace(frontFace(c, h, 0));
    int j = 1;
    while (motion::busy()) {
      sched::service();
      if (abortRequested()) return Outcome::ABORTED;
      if (j <= step.cells && estimator::s() >= j * CELL_PITCH_MM - READ_POINT_BEFORE_CENTRE_MM) {
        mm::Cell cj = c;
        for (int k = 0; k < j; ++k) cj = mm::neighbour(cj, h);
        if (SPEEDRUN_WALL_CHECK) checkSideWalls(cj, h, j);
        ++j;
      }
    }
    if (motion::result() != motion::Result::DONE) {
      g_abortReason = motion::resultName(motion::result());
      return Outcome::ABORTED;
    }
    for (int k = 0; k < step.cells; ++k) c = mm::neighbour(c, h);
    g_explorer.setPose(c, h);
    g_nextSigma = estimator::sigma();
  }
  return Outcome::ARRIVED;
}

struct TierPolicy {
  int  clean[4] = {0, 0, 0, 0};
  int  maxTier = MATCH_MAX_TIER;
  bool t3Tried = false, t3Clean = false, t4Tried = false;

  int choose() const {
    int cap = maxTier;
    if (!battery::fastAllowed() && cap > 1) cap = 1;
    const int safe = cap < 1 ? cap : 1;
    if (clean[1] < CLEAN_LOOPS_BEFORE_HERO) return safe;
    if (!t3Tried && cap >= 2) return 2;
    if (t3Clean && !t4Tried && cap >= 3) return 3;
    for (int t = cap; t >= 1; --t) {
      if (clean[t] >= CLEAN_LOOPS_BEFORE_HERO) return t;
    }
    return safe;
  }
  void started(int tier) {
    if (tier == 2) t3Tried = true;
    if (tier == 3) t4Tried = true;
  }
  void finished(int tier, bool ok) {
    if (ok) {
      ++clean[tier];
      if (tier == 2) t3Clean = true;
    } else if (tier >= 1) {
      maxTier = maxTier < tier - 1 ? maxTier : tier - 1;
    } else {
      maxTier = 0;
    }
  }
};

void resetPose() {
  g_explorer.setPose(START_CELL, mm::NORTH);
  imu::setHeading(0.0f);
  motion::setHeadingTargetDeg(0.0f);
  estimator::endStraight();
  g_nextStartS = START_OFFSET_MM;
  g_nextSigma = START_SIGMA_MM;
}

Trigger waitTrigger() {
  ui::clearEvents();
  ui::led(ui::Led::BLINK_SLOW);
  uint32_t nearSince = 0;
  bool armed = false;
  while (true) {
    sched::service();
    const ui::Button b = ui::event();
    if (b == ui::Button::SHORT && !USE_SONAR_BUTTON) return Trigger::RESCUE;
    if (b == ui::Button::LONG) return Trigger::EXIT;
    const sonar::Reading f = sonar::read(sonar::FRONT);
    const bool near = f.inRange && f.mm < TRIGGER_NEAR_MM && sonar::ageMs(sonar::FRONT) < SONAR_FRESH_MS;
    const bool clear = !f.inRange || f.mm > TRIGGER_CLEAR_MM;
    if (!armed) {
      if (near) {
        if (nearSince == 0) nearSince = millis();
        if (millis() - nearSince >= TRIGGER_HOLD_MS) {
          armed = true;
          ui::beep(1, 30, 30);
        }
      } else {
        nearSince = 0;
      }
    } else if (clear) {
      ui::soundOk();
      ui::led(ui::Led::BLINK_FAST);
      sched::waitMs(TRIGGER_DELAY_MS);
      ui::clearEvents();
      return Trigger::GO;
    }
  }
}

bool waitRescue() {
  motors::disable();
  ui::soundAbort();
  ui::led(ui::Led::BLINK_FAST);
  ui::clearEvents();
  while (true) {
    sched::service();
    const ui::Button b = ui::event();
    if (b == ui::Button::SHORT) {
      ui::soundOk();
      return true;
    }
    if (b == ui::Button::LONG) return false;
  }
}

bool selectMirror() {
  ui::clearEvents();
  ui::led(ui::Led::BLINK_SLOW);
  DBG_PRINTF("mirror: short press toggles, long press confirms. Now: %s\n", g_mirror ? "MIRRORED" : "normal");
  ui::beep(g_mirror ? 2 : 1);
  while (true) {
    sched::service();
    const ui::Button b = ui::event();
    if (b == ui::Button::SHORT) {
      g_mirror = !g_mirror;
      ui::beepStop();
      ui::beep(g_mirror ? 2 : 1);
      DBG_PRINTF("mirror: %s\n", g_mirror ? "MIRRORED (maze extends left)" : "normal (maze extends right)");
    } else if (b == ui::Button::LONG) {
      ui::soundOk();
      return true;
    }
  }
}

bool startCellRoutine() {
  if (!alignToFrontWall()) return false;
  const uint32_t t0 = millis();
  while (!imu::stationary() && millis() - t0 < BIAS_WAIT_MAX_MS) sched::service();
  if (!pivotPhysical(180.0f, TIERS[0])) return false;
  g_explorer.setPose(START_CELL, mm::NORTH);
  sched::waitMs(START_PAUSE_MS);
  return true;
}

bool goalTurnAround() {
  if (g_maze.hasWall(g_explorer.pos(), g_explorer.heading()) && !alignToFrontWall()) return false;
  if (!pivotPhysical(180.0f, TIERS[0])) return false;
  g_explorer.setPose(g_explorer.pos(), mm::turnBack(g_explorer.heading()));
  return true;
}

}

namespace strategy {

void runMatch() {
  DBG_PRINTF("\n== MODE 7: MATCH ==\n");
  bool storedMirror = false;
  const bool haveMap = loadMap(storedMirror);
  if (!haveMap) g_maze.reset(MAZE_SIZE_CELLS, MAZE_SIZE_CELLS);
  g_mirror = haveMap && storedMirror;
  DBG_PRINTF("map: %s\n", haveMap ? "restored from NVS (wipe it before a new match!)" : "empty");
  selectMirror();
  if (haveMap && g_mirror != storedMirror) {
    g_maze.reset(MAZE_SIZE_CELLS, MAZE_SIZE_CELLS);
    saveMap(false);
    ui::soundWiped();
  }
  g_explorer.setMirror(g_mirror);
  g_explorer.setMethod(EXPLORE_WALL_HUG ? Explorer::WALL_HUG : Explorer::FLOOD_FILL, WALL_HUG_LEFT_HAND);
  g_postViolations = g_maze.postViolations();
  DBG_PRINTF("exploring method: %s\n", EXPLORE_WALL_HUG ? (WALL_HUG_LEFT_HAND ? "wall hug, left hand"
                                                                             : "wall hug, right hand")
                                                         : "flood fill");

  TierPolicy policy;
  uint32_t matchStartMs = 0;
  bool needTrigger = true;
  bool needRescue = false;
  int runs = 0, returns = 0;
  uint32_t bestMs = 0;
  resetPose();

  while (true) {
    if (needRescue) {
      DBG_PRINTF("ABORT: %s - waiting for rescue (short press at the start)\n", g_abortReason);
      if (!waitRescue()) break;
      resetPose();
      needRescue = false;
      needTrigger = true;
    }
    if (needTrigger) {
      motors::disable();
      const Trigger t = waitTrigger();
      if (t == Trigger::EXIT) break;
      if (t == Trigger::RESCUE) {
        resetPose();
        ui::soundOk();
        continue;
      }
      if (matchStartMs == 0) matchStartMs = millis();
      needTrigger = false;
    }

    const bool searching = verifiedLoopS(1) < 0.0f;
    const int tier = searching ? 0 : policy.choose();
    const float loopS = searching ? MATCH_SEARCH_EST_S : verifiedLoopS(tier);
    const uint32_t elapsed = millis() - matchStartMs;
    const uint32_t needMs = static_cast<uint32_t>(loopS * MATCH_TIME_FACTOR * 1000.0f) + MATCH_END_MARGIN_MS;
    if (elapsed + needMs > MATCH_DURATION_MS) {
      DBG_PRINTF("match time used up: %d runs, %d returns, best %lu ms\n", runs, returns,
                 static_cast<unsigned long>(bestMs));
      motors::disable();
      ui::led(ui::Led::ON);
      ui::beep(3, 60, 100);
      ui::clearEvents();
      while (ui::event() != ui::Button::LONG) sched::service();
      break;
    }
    if (!battery::armAllowed()) {
      DBG_PRINTF("battery %.2f V - refusing to arm\n", battery::volts());
      ui::soundError();
      needTrigger = true;
      continue;
    }
    if (battery::warnLow()) ui::soundBatteryWarn();
    motors::clearFault();
    if (!motors::enable()) {
      g_abortReason = "motors refused to enable";
      needRescue = true;
      continue;
    }

    g_loopConflicts = 0;
    if (!searching) policy.started(tier);
    DBG_PRINTF("run %d: %s at T%d\n", runs + 1, searching ? "SEARCH" : "SPEED", tier + 1);
    uint32_t t0 = millis();
    Outcome run = searching ? exploreRun(Explorer::TO_GOAL, false) : pathRun(Explorer::TO_GOAL, tier);
    if (run == Outcome::NO_PATH && !searching) run = exploreRun(Explorer::TO_GOAL, false);
    if (run != Outcome::ARRIVED || !goalTurnAround()) {
      if (!searching) policy.finished(tier, false);
      needRescue = true;
      continue;
    }
    ++runs;
    uint32_t dt = millis() - t0;
    if (bestMs == 0 || dt < bestMs) bestMs = dt;
    DBG_PRINTF("  goal reached in %lu ms\n", static_cast<unsigned long>(dt));

    const bool explore = !EXPLORE_WALL_HUG && g_explorer.explorationWorthwhile(tierModel(1), PLAN_EXPLORE_GAIN, nullptr);
    const int returnTier = searching ? 1 : tier;
    t0 = millis();
    Outcome ret = Outcome::NO_PATH;
    if (!explore) ret = pathRun(Explorer::TO_START, returnTier);
    if (ret == Outcome::NO_PATH) ret = exploreRun(Explorer::TO_START, true);
    if (ret != Outcome::ARRIVED || !startCellRoutine()) {
      if (!searching) policy.finished(tier, false);
      needRescue = true;
      continue;
    }
    ++returns;
    dt = millis() - t0;
    if (bestMs == 0 || dt < bestMs) bestMs = dt;
    DBG_PRINTF("  start reached in %lu ms (%s return)\n", static_cast<unsigned long>(dt),
               explore ? "exploring" : "speed");
    saveMap(false);

    if (!searching) policy.finished(tier, g_loopConflicts == 0);
    if (!AUTO_RESTART) needTrigger = true;
  }
  motors::disable();
  DBG_PRINTF("match mode left: %d runs, %d returns\n", runs, returns);
}

}
